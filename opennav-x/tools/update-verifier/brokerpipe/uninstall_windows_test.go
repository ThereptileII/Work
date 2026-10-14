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

func TestNativeProtectedUninstallArmingPreservesOrdinaryInputRefusal(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	f := actualNativePeerFixture(t)
	var incoming atomic.Bool
	protected := &nativePeerCapability{pipe: f.pipe, client: f.process, checkServer: f.backend.checkServer, allowInput: &incoming}
	q, interim, _ := uninstallFixture()
	if err := writeFrame(pipeIO{f.ctx, f.client}, uninstallContinue(q, interim.FenceSHA256)); err != nil {
		t.Fatal(err)
	}
	if protected.check() == nil || f.backend.check() == nil {
		t.Fatal("unsolicited continuation accepted")
	}
	incoming.Store(true)
	if protected.check() != nil || f.backend.check() == nil {
		t.Fatal("protected input arming changed ordinary staging policy")
	}
	if _, err := readFrame(pipeIO{f.ctx, f.pipe}); err != nil {
		t.Fatal(err)
	}
	incoming.Store(false)
	if err := protected.check(); err != nil {
		t.Fatal("drained exact continuation refused", err)
	}
	for i := 0; i < 2; i++ {
		if err := writeFrame(pipeIO{f.ctx, f.client}, uninstallContinue(q, interim.FenceSHA256)); err != nil {
			t.Fatal(err)
		}
	}
	incoming.Store(true)
	if _, err := readFrame(pipeIO{f.ctx, f.pipe}); err != nil {
		t.Fatal(err)
	}
	incoming.Store(false)
	if protected.check() == nil {
		t.Fatal("queued duplicate continuation accepted")
	}
}

func TestNativeProtectedUninstallPublisherDoesNotReenterOperatorLock(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	f := actualNativePeerFixture(t)
	identity := nativePeerEvidence(f.process)
	q, interim, final := uninstallFixture()
	interim.ClientSID, final.ClientSID = identity.SID, identity.SID
	var incoming atomic.Bool
	f.backend.allowInput = &incoming
	used := false
	handler := func(ctx context.Context, _ Peer, _ MaintenanceRequest, publish func(context.Context, MaintenanceResult) error) (MaintenanceResult, error) {
		err := f.cap.WithMaintenanceOperatorChecked(ctx, identity.SID, func(_ PeerIdentity, check func(context.Context) error) error {
			used = true
			if err := check(ctx); err != nil {
				return err
			}
			// Publish executes while the capability mutex is held, as it is in
			// the actual host's WithMaintenanceOperatorLocationChecked callback.
			return publish(ctx, interim)
		})
		return final, err
	}
	check := func(ctx context.Context) error {
		return errors.Join(ctx.Err(), f.backend.check(), maintenanceOperatorToken(f.backend, identity, identity.SID))
	}
	if !currentProcessHasEnabledAdmin(t) || !windows.GetCurrentProcessToken().IsElevated() {
		_, err := invokeProtectedUninstallHandler(f.ctx, identity.Peer, q, handler, executionNativeConnection{f.pipe}, check, &incoming)
		if err == nil || used {
			t.Fatal("filtered token entered protected callback")
		}
		return
	}
	done := make(chan error, 1)
	go func() {
		raw, err := readFrame(pipeIO{f.ctx, f.client})
		if err == nil {
			_, err = decodeUninstallFenceResult(raw, q)
		}
		if err == nil {
			err = writeFrame(pipeIO{f.ctx, f.client}, uninstallContinue(q, interim.FenceSHA256))
		}
		done <- err
	}()
	result, err := invokeProtectedUninstallHandler(f.ctx, identity.Peer, q, handler, executionNativeConnection{f.pipe}, check, &incoming)
	if err != nil || result != final || !used {
		t.Fatal("publisher deadlocked or refused while operator lock held", result, err)
	}
	if err := <-done; err != nil {
		t.Fatal(err)
	}
}

func TestNativeOldMaintenanceServerRefusesProtectedUninstallBeforeHandler(t *testing.T) {
	f := actualNativePeerFixture(t)
	q, _, _ := uninstallFixture()
	raw, err := encodeMaintenanceRequest(q)
	if err != nil || writeFrame(pipeIO{f.ctx, f.client}, raw) != nil {
		t.Fatal("request publication failed", err)
	}
	called := false
	handler := func(context.Context, Peer, MaintenanceRequest) (MaintenanceResult, error) {
		called = true
		return MaintenanceResult{}, nil
	}
	// A nil installed-server owner is intentional: the legacy endpoint must
	// reject this operation before either identity setup or an ordinary handler.
	serveMaintenanceOne(f.ctx, f.pipe, handler, nil, map[string]struct{}{}, nil)
	if called {
		t.Fatal("old final-only maintenance handler reached protected uninstall")
	}
}
