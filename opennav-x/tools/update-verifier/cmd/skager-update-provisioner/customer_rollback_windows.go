//go:build windows

package main

import (
	"context"
	"errors"
	"runtime"
	"unsafe"

	"example.com/opennav-update-verifier/brokerpipe"
	"golang.org/x/sys/windows"
)

// This is our own confirmation dialog, never a UAC approval mechanism. The
// original elevated child/unelevated parent remain owned by installedElevation.
// Cancellation returns without mutation. A late UI answer cannot resume work.
func confirmCustomerRollbackNative(ctx context.Context, plan brokerpipe.MaintenanceResult) (bool, error) {
	if ctx == nil || ctx.Err() != nil || plan.Previous == nil {
		return false, errors.New("live authenticated rollback confirmation required")
	}
	return confirmCustomerMaintenanceNative(ctx, "Confirm SKAGER rollback", "Roll back SKAGER from "+plan.Version+" to "+plan.Previous.Version+"?\n\nTarget build: "+plan.Previous.Commit+"\n\nCurrent charts, routes, waypoints, plugins, connections and profile data are preserved. Older navigation data is never restored. The application stays closed. OpenCPN and the update broker remain installed.")
}

// Shared own-dialog primitive. It never automates UAC; a late answer cannot
// resume a cancelled maintenance operation.
func confirmCustomerMaintenanceNative(ctx context.Context, titleText, messageText string) (bool, error) {
	if ctx == nil || ctx.Err() != nil {
		return false, errors.New("live customer confirmation required")
	}
	message, err := windows.UTF16PtrFromString(messageText)
	if err != nil {
		return false, err
	}
	title, err := windows.UTF16PtrFromString(titleText)
	if err != nil {
		return false, err
	}
	box := windows.NewLazySystemDLL("user32.dll").NewProc("MessageBoxW")
	if err := box.Find(); err != nil {
		return false, err
	}
	type answer struct {
		value uintptr
		err   error
	}
	done := make(chan answer, 1)
	go func() {
		runtime.LockOSThread()
		defer runtime.UnlockOSThread()
		if ctx.Err() != nil {
			done <- answer{err: ctx.Err()}
			return
		}
		// MB_YESNO | MB_ICONQUESTION | MB_DEFBUTTON2: No is the default.
		value, _, callErr := box.Call(0, uintptr(unsafe.Pointer(message)), uintptr(unsafe.Pointer(title)), 0x4|0x20|0x100)
		runtime.KeepAlive(message)
		runtime.KeepAlive(title)
		if value == 0 {
			done <- answer{err: callErr}
			return
		}
		done <- answer{value: value}
	}()
	select {
	case <-ctx.Done():
		return false, ctx.Err()
	case result := <-done:
		if err := errors.Join(result.err, ctx.Err()); err != nil {
			return false, err
		}
		return result.value == 6, nil // IDYES only; Cancel/No never authorize.
	}
}
