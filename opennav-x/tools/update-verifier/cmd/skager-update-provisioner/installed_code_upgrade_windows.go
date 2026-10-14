//go:build windows

package main

import (
	"context"
	"errors"
	"runtime"
	"strings"
	"syscall"
	"time"
	"unsafe"

	"golang.org/x/sys/windows"
)

func installedElevationOperationsForAction(action string) installedElevationOperations {
	if action != codeUpgradeCurrent {
		return installedElevationNativeOperations()
	}
	// Every original helper image owner in this explicit action denies writes
	// from the start while sharing deletion for its authenticated fixed move.
	return installedElevationOperations{
		openSelf: func(ctx context.Context) (installedElevationSelf, error) {
			return openInstalledElevationSelfMode(ctx, true)
		},
		openParent: func(ctx context.Context, q installedElevationRequest, id installedElevationIdentity) (installedElevationOwner, error) {
			return openInstalledElevationParentMode(ctx, q, id, true)
		},
		launch: func(ctx context.Context, id installedElevationIdentity, args []string) (installedElevationChild, error) {
			return launchInstalledElevationMode(ctx, id, args, true)
		},
	}
}

func installedCodeSuccessorNativeOperations() installedCodeSuccessorOperations {
	return installedCodeSuccessorOperations{openSelf: openInstalledCodeSuccessorSelf, openParent: openInstalledCodeSuccessorParent, run: runInstalledCodeSuccessor}
}

func openInstalledCodeSuccessorSelf(ctx context.Context) (installedElevationSelf, error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if err := windows.SetDefaultDllDirectories(windows.LOAD_LIBRARY_SEARCH_SYSTEM32); err != nil {
		return installedElevationSelf{}, err
	}
	if err := installedElevationNoImpersonation(); err != nil {
		return installedElevationSelf{}, err
	}
	image, err := installedElevationImagePath(windows.CurrentProcess())
	if err != nil {
		return installedElevationSelf{}, err
	}
	p, err := openInstalledElevationProcess(ctx, windows.GetCurrentProcessId(), image, false)
	if err != nil {
		return installedElevationSelf{}, err
	}
	// The next native reopener authenticates this actual running image against
	// independently fetched TUF authority before granting any mutation capability.
	return installedElevationSelf{identity: p.identity, owner: p}, nil
}

func installedCodeParentBinding(q installedElevationRequest, self, parent installedElevationIdentity) error {
	if !q.child || q.action != codeUpgradeCurrent || !self.elevated || !parent.elevated || q.pid == 0 || q.created == 0 || parent.pid != q.pid || parent.created != q.created || parent.sid == "" || parent.sid != self.sid || parent.session == 0 || parent.session != self.session || parent.pid == self.pid || self.created <= parent.created || self.created-parent.created > uint64(installedCodeUpgradeLimit/(100*time.Nanosecond)) {
		return errors.New("original same-account elevated code launch parent differs")
	}
	return nil
}

func openInstalledCodeSuccessorParent(ctx context.Context, q installedElevationRequest, self installedElevationIdentity) (installedElevationOwner, error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if err := installedElevationNoImpersonation(); err != nil {
		return nil, err
	}
	code, err := openInstalledElevationCodeMode(ctx, true)
	if err != nil {
		return nil, err
	}
	p, err := openInstalledElevationProcess(ctx, q.pid, code.image, false)
	if err != nil {
		return nil, errors.Join(err, code.close())
	}
	p.code = code
	if err := installedCodeParentBinding(q, self, p.identity); err != nil {
		return nil, errors.Join(err, p.close())
	}
	var token windows.Token
	if err := windows.OpenProcessToken(windows.CurrentProcess(), windows.TOKEN_QUERY, &token); err != nil {
		return nil, errors.Join(err, p.close())
	}
	actual, queryErr := installedElevationToken(token)
	closeErr := token.Close()
	if queryErr != nil || closeErr != nil || actual.sid != p.tokenIdentity.sid || actual.session != p.tokenIdentity.session || actual.logon != p.tokenIdentity.logon || !actual.elevated {
		return nil, errors.Join(errors.New("original elevated launch logon differs"), queryErr, closeErr, p.close())
	}
	return p, p.check(ctx)
}

func launchInstalledCodeSuccessor(ctx context.Context, owner installedCodeLaunchOwner, expected installedCodeSnapshot) (result installedElevationChild, resultErr error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	native, ok := owner.(*installedCodeNativeLaunch)
	if !ok || native == nil || native.owner == nil || ctx == nil || ctx.Err() != nil {
		return nil, errors.New("original opaque native launch admission required")
	}
	if err := installedElevationNoImpersonation(); err != nil {
		return nil, err
	}
	self, err := openInstalledElevationSelfMode(ctx, true)
	if err != nil {
		return nil, err
	}
	parent := self.owner.(*installedElevationProcess)
	defer func() { resultErr = errors.Join(resultErr, parent.close()) }()
	if !self.identity.elevated {
		return nil, errors.New("existing elevated fixed helper token required")
	}
	actual, err := native.snapshot(ctx)
	if err != nil || actual != expected || actual.path == "" || actual.directory == "" {
		return nil, errors.Join(errors.New("retained native successor snapshot differs"), err)
	}
	args := installedCodeSuccessorArgs(self.identity, actual.pins)
	if _, handled, err := parseInstalledCodeSuccessor(args); !handled || err != nil {
		return nil, errors.New("fixed successor arguments required")
	}
	application, err := windows.UTF16PtrFromString(actual.path)
	if err != nil {
		return nil, err
	}
	parts := []string{syscall.EscapeArg(actual.path)}
	for _, arg := range args {
		parts = append(parts, syscall.EscapeArg(arg))
	}
	command, err := windows.UTF16FromString(strings.Join(parts, " "))
	if err != nil {
		return nil, err
	}
	directory, err := windows.UTF16PtrFromString(actual.directory)
	if err != nil {
		return nil, err
	}
	if err := errors.Join(parent.check(ctx), native.check(ctx)); err != nil {
		return nil, err
	}
	var startup windows.StartupInfo
	startup.Cb = uint32(unsafe.Sizeof(startup))
	var process windows.ProcessInformation
	if err := windows.CreateProcess(application, &command[0], nil, nil, false, windows.CREATE_SUSPENDED|windows.CREATE_NO_WINDOW, nil, directory, &startup, &process); err != nil {
		return nil, err
	}
	// Adopt ORIGINAL handles returned by CREATE_SUSPENDED before the sole resume.
	child := &installedElevationProcess{handle: process.Process, image: actual.path, allowExit: true}
	result = child
	resumeAttempted := false
	defer func() {
		resultErr = errors.Join(resultErr, windows.CloseHandle(process.Thread))
		if resultErr != nil && !resumeAttempted {
			// This exact never-resumed child has performed no product operation.
			// Only this pre-execution failure may terminate it; never kill/retry a
			// child after ResumeThread has been attempted.
			terminateErr := windows.TerminateProcess(process.Process, 1)
			state, waitErr := windows.WaitForSingleObject(process.Process, 5000)
			resultErr = errors.Join(resultErr, terminateErr, waitErr)
			if waitErr == nil && state == windows.WAIT_OBJECT_0 {
				resultErr = errors.Join(resultErr, child.close())
				result = nil
			}
		}
	}()
	pid, err := windows.GetProcessId(process.Process)
	if err != nil || pid != process.ProcessId {
		return child, errors.Join(errors.New("original successor process handle differs"), err)
	}
	if err := windows.OpenProcessToken(process.Process, windows.TOKEN_QUERY, &child.token); err != nil {
		return child, err
	}
	child.tokenIdentity, err = installedElevationToken(child.token)
	if err != nil {
		return child, err
	}
	created, err := installedElevationCreated(process.Process)
	if err != nil {
		return child, err
	}
	child.identity = installedElevationIdentity{pid: pid, created: created, sid: child.tokenIdentity.sid, session: child.tokenIdentity.session, elevated: child.tokenIdentity.elevated}
	q := installedElevationRequest{action: codeUpgradeCurrent, child: true, pid: self.identity.pid, created: self.identity.created}
	if err := installedCodeParentBinding(q, child.identity, self.identity); err != nil {
		return child, err
	}
	if child.tokenIdentity.logon != parent.tokenIdentity.logon {
		return child, errors.New("original successor logon differs")
	}
	if err := errors.Join(parent.check(ctx), native.check(ctx), child.check(ctx)); err != nil {
		return child, err
	}
	resumeAttempted = true
	previous, err := windows.ResumeThread(process.Thread)
	if err != nil || previous != 1 {
		return child, errors.Join(errors.New("original successor single resume unverified"), err)
	}
	return child, errors.Join(parent.check(ctx), native.check(ctx), child.check(ctx))
}

func retainInstalledCodeUntilExit(child installedElevationChild, owner installedCodeLaunchOwner) error {
	p, ok := child.(*installedElevationProcess)
	if !ok || p == nil || owner == nil {
		return errors.New("original native successor custody required")
	}
	// This command must remain alive: a detached cleanup goroutine would die
	// at main's os.Exit and release a live child's image/cache. The operation
	// budget is already cancelled; only query this exact original handle until
	// exit, preserving all custody and never killing or relaunching it.
	for {
		state, err := windows.WaitForSingleObject(p.handle, windows.INFINITE)
		if err == nil && state == windows.WAIT_OBJECT_0 {
			return errors.Join(p.close(), owner.close())
		}
		// Unknown native wait state cannot authorize process exit or release.
		// Re-observe the same original handle; never adopt a PID or new owner.
		time.Sleep(time.Second)
	}
}
