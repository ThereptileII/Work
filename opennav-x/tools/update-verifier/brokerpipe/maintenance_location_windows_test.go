package brokerpipe

import (
	"context"
	"errors"
	"runtime"
	"strings"
	"testing"

	"golang.org/x/sys/windows"
)

func TestMaintenanceOperatorLocationActualOriginalTokenAndCallbackLifetime(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	f := actualNativePeerFixture(t)
	identity := nativePeerEvidence(f.process)
	used := false
	want := currentProcessHasEnabledAdmin(t) && windows.GetCurrentProcessToken().IsElevated()
	var expected string
	if want {
		// Independently open the documented folder-query access from the SAME
		// original process handle, never a service-user or PID/path substitute.
		var folderToken windows.Token
		if err := windows.OpenProcessToken(f.process.handle, windows.TOKEN_QUERY|windows.TOKEN_IMPERSONATE, &folderToken); err != nil {
			t.Fatal("original operator folder token cannot be opened", err)
		}
		var locationErr error
		expected, locationErr = folderToken.KnownFolderPath(windows.FOLDERID_LocalAppData, 0)
		closeErr := folderToken.Close()
		if locationErr != nil || closeErr != nil || expected == "" {
			t.Fatal("original operator token cannot resolve LocalAppData", locationErr, closeErr)
		}
	}
	callbackFailure := errors.New("inert location callback fault")
	err := f.cap.WithMaintenanceOperatorLocation(f.ctx, identity.SID, func(actual PeerIdentity, location string) error {
		used = true
		if actual != identity || !strings.EqualFold(location, expected) || location == "" {
			t.Fatal("location did not originate from same original operator token")
		}
		if err := maintenanceNoThreadImpersonation(); err != nil {
			t.Fatal("location callback inherited thread impersonation", err)
		}
		return callbackFailure
	})
	if want && (!used || !errors.Is(err, callbackFailure)) {
		t.Fatal("native operator location did not preserve callback outcome", err)
	}
	if !want && (used || err == nil) {
		t.Fatal("filtered/non-admin peer received operator location", err)
	}
	f.cap.Close()
	used = false
	if err = f.cap.WithMaintenanceOperatorLocation(f.ctx, identity.SID, func(PeerIdentity, string) error { used = true; return nil }); !errors.Is(err, ErrPeerRevoked) || used {
		t.Fatal("closed capability supplied operator location", err)
	}
}

func TestMaintenanceOperatorLocationCheckedExpiresAfterCallback(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	f := actualNativePeerFixture(t)
	if !currentProcessHasEnabledAdmin(t) || !windows.GetCurrentProcessToken().IsElevated() {
		return
	}
	identity := nativePeerEvidence(f.process)
	var escaped func(context.Context) error
	used := false
	err := f.cap.WithMaintenanceOperatorLocationChecked(f.ctx, identity.SID, func(got PeerIdentity, location string, check func(context.Context) error) error {
		used, escaped = true, check
		if got != identity || location == "" || check(f.ctx) != nil {
			t.Fatal("live operator location checker refused")
		}
		return nil
	})
	if err != nil || !used {
		t.Fatal("elevated native location checker did not complete", err)
	}
	if escaped == nil || !errors.Is(escaped(f.ctx), ErrRefused) {
		t.Fatal("location checker remained usable after callback")
	}
}
