//go:build windows

package brokerpipe

import (
	"context"
	"errors"
	"runtime"
	"sync/atomic"
	"testing"

	"golang.org/x/sys/windows"
)

func TestMaintenanceOperatorCheckedCannotExtendOriginalContext(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if !currentProcessHasEnabledAdmin(t) || !windows.GetCurrentProcessToken().IsElevated() {
		return // Other native cases require refusal for a filtered token.
	}
	f := actualNativePeerFixture(t)
	ctx, cancel := context.WithCancel(f.ctx)
	defer cancel()
	err := f.cap.WithMaintenanceOperatorChecked(ctx, nativePeerEvidence(f.process).SID, func(_ PeerIdentity, check func(context.Context) error) error {
		cancel()
		if err := check(context.Background()); !errors.Is(err, context.Canceled) {
			t.Fatal("a fresh context extended canceled operator scope", err)
		}
		return nil
	})
	if !errors.Is(err, context.Canceled) {
		t.Fatal("original scope cancellation was hidden", err)
	}
}

// Independent fixture expectation from the actual process token's group flags;
// it deliberately does not call the production bounded group reader.
func currentProcessHasEnabledAdmin(t *testing.T) bool {
	t.Helper()
	admin, err := windows.CreateWellKnownSid(windows.WinBuiltinAdministratorsSid)
	if err != nil {
		t.Fatal(err)
	}
	groups, err := windows.GetCurrentProcessToken().GetTokenGroups()
	if err != nil {
		t.Fatal(err)
	}
	for _, group := range groups.AllGroups() {
		if group.Sid != nil && group.Sid.IsValid() && group.Sid.Equals(admin) &&
			group.Attributes&windows.SE_GROUP_ENABLED != 0 &&
			group.Attributes&windows.SE_GROUP_USE_FOR_DENY_ONLY == 0 {
			return true
		}
	}
	return false
}

func TestMaintenanceAdminGroupFlags(t *testing.T) {
	admin, err := windows.CreateWellKnownSid(windows.WinBuiltinAdministratorsSid)
	if err != nil {
		t.Fatal(err)
	}
	users, err := windows.CreateWellKnownSid(windows.WinBuiltinUsersSid)
	if err != nil {
		t.Fatal(err)
	}
	for _, test := range []struct {
		name   string
		groups []windows.SIDAndAttributes
		want   bool
	}{
		{"enabled", []windows.SIDAndAttributes{{Sid: admin, Attributes: windows.SE_GROUP_ENABLED}}, true},
		{"deny-only", []windows.SIDAndAttributes{{Sid: admin, Attributes: windows.SE_GROUP_ENABLED | windows.SE_GROUP_USE_FOR_DENY_ONLY}}, false},
		{"disabled", []windows.SIDAndAttributes{{Sid: admin}}, false},
		{"different", []windows.SIDAndAttributes{{Sid: users, Attributes: windows.SE_GROUP_ENABLED}}, false},
		{"duplicate", []windows.SIDAndAttributes{{Sid: admin, Attributes: windows.SE_GROUP_ENABLED}, {Sid: admin, Attributes: windows.SE_GROUP_ENABLED}}, false},
	} {
		t.Run(test.name, func(t *testing.T) {
			if got := maintenanceAdminGroupEnabled(test.groups, admin); got != test.want {
				t.Fatalf("admin group result = %v, want %v", got, test.want)
			}
		})
	}
}

func TestMaintenanceOperatorActualNativeTokenAndLifetime(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	f := actualNativePeerFixture(t)
	identity := nativePeerEvidence(f.process)
	used := false
	callback := func(got PeerIdentity) error {
		used = true
		if got != identity {
			return ErrRefused
		}
		return nil
	}
	if err := f.cap.WithMaintenanceOperator(f.ctx, identity.SID+"-1", callback); err == nil || used {
		t.Fatal("different registered SID reached maintenance callback")
	}
	// A failed identity attempt revokes the capability. Use a fresh actual pipe.
	f = actualNativePeerFixture(t)
	identity = nativePeerEvidence(f.process)
	used = false
	want := currentProcessHasEnabledAdmin(t) && windows.GetCurrentProcessToken().IsElevated()
	err := f.cap.WithMaintenanceOperator(f.ctx, identity.SID, callback)
	if want && (err != nil || !used) {
		t.Fatal("actual elevated administrator process refused", err)
	}
	if !want && (err == nil || used) {
		t.Fatal("filtered/non-administrator process granted maintenance", err)
	}
	if want {
		// The execution-session wrapper retains the same original native peer.
		f.cap.state.backend = &executionNativePeer{native: f.backend}
		used = false
		if err := f.cap.WithMaintenanceOperator(f.ctx, identity.SID, callback); err != nil || !used {
			t.Fatal("actual wrapped native peer refused", err)
		}
		armed := &atomic.Bool{}
		armed.Store(true)
		f.cap.state.backend = &executionNativePeer{native: f.backend, incoming: armed}
		used = false
		if err := f.cap.WithMaintenanceOperator(f.ctx, identity.SID, callback); err == nil || used {
			t.Fatal("active execution input granted maintenance", err)
		}
	}
	f.cap.Close()
	used = false
	if err := f.cap.WithMaintenanceOperator(f.ctx, identity.SID, callback); !errors.Is(err, ErrPeerRevoked) || used {
		t.Fatal("closed native capability restored operator authority", err)
	}
}

func TestMaintenanceOperatorCancellationBeforeUse(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	f := actualNativePeerFixture(t)
	derived, cancel := context.WithCancel(f.ctx)
	cancel()
	used := false
	if err := f.cap.WithMaintenanceOperator(derived, nativePeerEvidence(f.process).SID, func(PeerIdentity) error {
		used = true
		return nil
	}); !errors.Is(err, context.Canceled) || used {
		t.Fatal("canceled native maintenance use reached callback", err)
	}
}

func TestMaintenanceOperatorSessionAndPostUseDisconnect(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	f := actualNativePeerFixture(t)
	identity := nativePeerEvidence(f.process)
	f.cap.state.identity.SessionID++
	used := false
	if err := f.cap.WithMaintenanceOperator(f.ctx, identity.SID, func(PeerIdentity) error {
		used = true
		return nil
	}); err == nil || used {
		t.Fatal("different session reached maintenance callback")
	}
	if !windows.GetCurrentProcessToken().IsElevated() || !currentProcessHasEnabledAdmin(t) {
		return
	}
	f = actualNativePeerFixture(t)
	identity = nativePeerEvidence(f.process)
	used = false
	if err := f.cap.WithMaintenanceOperator(f.ctx, identity.SID, func(PeerIdentity) error {
		used = true
		closeErr := windows.CloseHandle(f.client)
		f.client = 0
		return closeErr
	}); err == nil || !used {
		t.Fatal("disconnected peer survived post-use check", err)
	}
}

func TestMaintenanceOperatorCheckedLeaseRefusesCancellationOtherThreadAndLostToken(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	f := actualNativePeerFixture(t)
	if !currentProcessHasEnabledAdmin(t) || !windows.GetCurrentProcessToken().IsElevated() {
		return
	}
	identity := nativePeerEvidence(f.process)
	var escaped func(context.Context) error
	used, durableEffect := false, false
	err := f.cap.WithMaintenanceOperatorChecked(f.ctx, identity.SID, func(got PeerIdentity, check func(context.Context) error) error {
		used, escaped = true, check
		if got != identity || check(f.ctx) != nil {
			t.Fatal("live original-thread checker refused elevated operator")
		}
		crossThread := make(chan error, 1)
		go func() { crossThread <- check(f.ctx) }()
		if err := <-crossThread; !errors.Is(err, ErrRefused) {
			t.Fatal("checker escaped to another OS thread", err)
		}
		short, cancel := context.WithCancel(f.ctx)
		cancel()
		if err := check(short); !errors.Is(err, context.Canceled) {
			t.Fatal("canceled checker could authorize an effect", err)
		}
		// Lose only the fixture's retained query handle, never change the
		// shared process token or its enabled groups on the CI runner.
		if err := f.process.retainedToken.Close(); err != nil {
			t.Fatal(err)
		}
		f.process.retainedToken = 0
		if check(f.ctx) == nil {
			durableEffect = true
		}
		return nil
	})
	if !used || durableEffect || err == nil {
		t.Fatal("lost original token was not refused before durable effect", used, durableEffect, err)
	}
	if escaped == nil || !errors.Is(escaped(f.ctx), ErrRefused) {
		t.Fatal("checker remained live after callback returned")
	}
}

func TestMaintenanceOperatorClosedCapabilityRevocationPrecedesBackendReadiness(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	for _, kind := range []string{"native", "wrapped", "active-input", "missing-native"} {
		t.Run(kind, func(t *testing.T) {
			f := actualNativePeerFixture(t)
			identity := nativePeerEvidence(f.process)
			incoming := &atomic.Bool{}
			switch kind {
			case "wrapped":
				f.cap.state.backend = &executionNativePeer{native: f.backend}
			case "active-input":
				incoming.Store(true)
				f.cap.state.backend = &executionNativePeer{native: f.backend, incoming: incoming}
			case "missing-native":
				f.cap.state.backend = &executionNativePeer{}
			}
			if err := f.cap.Close(); err != nil {
				t.Fatal(err)
			}
			used := false
			callback := func(PeerIdentity) error { used = true; return nil }
			for _, ctx := range []context.Context{f.ctx, context.Background()} {
				if err := f.cap.WithMaintenanceOperator(ctx, identity.SID, callback); !errors.Is(err, ErrPeerRevoked) || used {
					t.Fatal("closed capability lost revocation", kind, err)
				}
			}
			incoming.Store(false)
			if err := f.cap.WithMaintenanceOperator(f.ctx, identity.SID, callback); !errors.Is(err, ErrPeerRevoked) || used {
				t.Fatal("readiness change revived closed capability", err)
			}
			if err := f.cap.WithMaintenanceOperatorChecked(f.ctx, identity.SID, func(PeerIdentity, func(context.Context) error) error { used = true; return nil }); !errors.Is(err, ErrPeerRevoked) || used {
				t.Fatal("closed capability lent maintenance checker", err)
			}
		})
	}
}
