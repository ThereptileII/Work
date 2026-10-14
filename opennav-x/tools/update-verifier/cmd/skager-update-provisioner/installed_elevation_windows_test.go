//go:build windows

package main

import (
	"context"
	"errors"
	"testing"
	"time"

	"golang.org/x/sys/windows"
)

func TestNativeInstalledElevationBindingRetainsActualProcessAndToken(t *testing.T) {
	image, err := installedElevationImagePath(windows.CurrentProcess())
	if err != nil {
		t.Fatal(err)
	}
	p, err := openInstalledElevationProcess(context.Background(), windows.GetCurrentProcessId(), image, false)
	if p != nil {
		t.Cleanup(func() {
			if err := p.close(); err != nil {
				t.Error(err)
			}
		})
	}
	if err != nil || p == nil {
		t.Fatal("actual native current process could not be retained", err)
	}
	process, token := p.handle, p.token
	if process == 0 || token == 0 || p.identity.pid != windows.GetCurrentProcessId() || p.identity.created == 0 || p.identity.sid == "" || p.identity.session == 0 {
		t.Fatal("native identity incomplete")
	}
	if err := p.check(context.Background()); err != nil {
		t.Fatal(err)
	}
	for _, fault := range []string{"creation", "pid", "token", "image", "cancelled"} {
		t.Run(fault, func(t *testing.T) {
			originalID, originalToken, originalImage := p.identity, p.tokenIdentity, p.image
			defer func() { p.identity, p.tokenIdentity, p.image = originalID, originalToken, originalImage }()
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			switch fault {
			case "creation":
				p.identity.created++
			case "pid":
				p.identity.pid++
			case "token":
				p.tokenIdentity.sid = "S-1-5-21-9-9-9-1001"
			case "image":
				p.image += ".different"
			case "cancelled":
				cancel()
			}
			if err := p.check(ctx); err == nil {
				t.Fatal("changed native original comparison accepted")
			}
		})
	}
	if err := p.check(context.Background()); err != nil {
		t.Fatal("comparison refusal changed original custody", err)
	}
	if err := p.close(); err != nil {
		t.Fatal(err)
	}
	if err := p.check(context.Background()); err == nil {
		t.Fatal("closed native original owner reused")
	}
	var created, exit, kernel, user windows.Filetime
	if err := windows.GetProcessTimes(process, &created, &exit, &kernel, &user); !errors.Is(err, windows.ERROR_INVALID_HANDLE) {
		t.Fatal("original process handle remained open", err)
	}
	if _, err := token.GetTokenUser(); !errors.Is(err, windows.ERROR_INVALID_HANDLE) {
		t.Fatal("original token handle remained open", err)
	}
}

func TestNativeInstalledElevationDecommissionSameAccountComparison(t *testing.T) {
	image, err := installedElevationImagePath(windows.CurrentProcess())
	if err != nil {
		t.Fatal(err)
	}
	p, err := openInstalledElevationProcess(context.Background(), windows.GetCurrentProcessId(), image, false)
	if p != nil {
		t.Cleanup(func() {
			if err := p.close(); err != nil {
				t.Error(err)
			}
		})
	}
	if err != nil {
		t.Fatal(err)
	}
	// Only the actual native values seed this comparison fixture. It does not
	// elevate, create another process, or invoke any installed-maintenance API.
	parent := p.identity
	parent.elevated = false
	self := parent
	self.pid++
	self.created++
	self.elevated = true
	q := installedElevationRequest{child: true, pid: parent.pid, created: parent.created}
	if err := installedElevationParentBinding(q, self, parent); err != nil {
		t.Fatal(err)
	}
	for _, fault := range []string{"wrongSID", "wrongSession", "reusedPID", "differentPID", "selfParent", "elevatedParent", "unelevatedChild", "oldParent"} {
		t.Run(fault, func(t *testing.T) {
			s, actual, request := self, parent, q
			switch fault {
			case "wrongSID":
				s.sid = "S-1-5-21-9-9-9-1001"
			case "wrongSession":
				s.session++
			case "reusedPID":
				actual.created++
			case "differentPID":
				actual.pid++
			case "selfParent":
				s.pid = actual.pid
			case "elevatedParent":
				actual.elevated = true
			case "unelevatedChild":
				s.elevated = false
			case "oldParent":
				s.created = actual.created + uint64((installedElevationLimit+time.Second)/(100*time.Nanosecond))
			}
			if err := installedElevationParentBinding(request, s, actual); err == nil {
				t.Fatal("wrong original native parent binding accepted")
			}
		})
	}
	if err := p.check(context.Background()); err != nil {
		t.Fatal("comparison fixture lost native owner", err)
	}
}

func TestNativeInstalledElevationStartupProtectedCodeDACL(t *testing.T) {
	for _, tc := range []struct {
		name, rights      string
		allowAdds, accept bool
	}{
		{"read execute", "FRFX", false, true},
		{"foreign write", "FW", false, false},
		{"foreign delete", "SD", false, false},
		{"foreign DACL", "WD", false, false},
		{"ancestor add subdirectory", "0x4", true, true},
		{"code directory add subdirectory", "0x4", false, false},
		{"ancestor delete child", "0x40", true, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			sd, err := windows.SecurityDescriptorFromString("O:BAD:P(A;;FA;;;SY)(A;;FA;;;BA)(A;;" + tc.rights + ";;;AU)")
			if err != nil {
				t.Fatal(err)
			}
			if err := installedElevationSecurity(sd, tc.allowAdds); (err == nil) != tc.accept {
				t.Fatal("protected fixed code mutation policy differs", err)
			}
		})
	}
}
