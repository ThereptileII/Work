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
	admin, err := windows.CreateWellKnownSid(windows.WinBuiltinAdministratorsSid)
	if err != nil {
		t.Fatal(err)
	}
	member, err := windows.GetCurrentProcessToken().IsMember(admin)
	if err != nil {
		t.Fatal(err)
	}
	want := member && windows.GetCurrentProcessToken().IsElevated()
	err = f.cap.WithMaintenanceOperator(f.ctx, identity.SID, callback)
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
	admin, err := windows.CreateWellKnownSid(windows.WinBuiltinAdministratorsSid)
	if err != nil {
		t.Fatal(err)
	}
	member, err := windows.GetCurrentProcessToken().IsMember(admin)
	if err != nil {
		t.Fatal(err)
	}
	if !windows.GetCurrentProcessToken().IsElevated() || !member {
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
