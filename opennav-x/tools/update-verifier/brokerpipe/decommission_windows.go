//go:build windows

package brokerpipe

import (
	"context"
	"errors"
	"runtime"
	"sync/atomic"
	"unsafe"

	"golang.org/x/sys/windows"
)

type nativeDecommissionWitness struct {
	server      *serverIdentity
	caller      *processIdentity
	operator    maintenanceTokenProof
	serverToken maintenanceTokenProof
}

func decommissionPrimaryIdentity(token windows.Token) (maintenanceTokenProof, error) {
	var stats tokenStatistics
	var used uint32
	if err := windows.GetTokenInformation(token, windows.TokenStatistics, (*byte)(unsafe.Pointer(&stats)), uint32(unsafe.Sizeof(stats)), &used); err != nil || used != uint32(unsafe.Sizeof(stats)) || stats.TokenType != windows.TokenPrimary {
		return maintenanceTokenProof{}, errors.Join(ErrRefused, err)
	}
	return maintenanceTokenProof{tokenID: stats.TokenID, logon: stats.AuthenticationID}, nil
}
func checkDecommissionPrimary(p *processIdentity, expected maintenanceTokenProof) error {
	retained, err := decommissionPrimaryIdentity(p.retainedToken)
	if err != nil || retained != expected {
		return errors.Join(ErrRefused, err)
	}
	var token windows.Token
	if err = windows.OpenProcessToken(p.handle, windows.TOKEN_QUERY, &token); err != nil {
		return err
	}
	current, infoErr := decommissionPrimaryIdentity(token)
	closeErr := token.Close()
	if infoErr != nil || closeErr != nil || current != expected {
		return errors.Join(ErrRefused, infoErr, closeErr)
	}
	return nil
}
func (n *nativeDecommissionWitness) check(ctx context.Context) error {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if ctx == nil || ctx.Err() != nil || n == nil || n.server == nil || n.caller == nil {
		return ErrRefused
	}
	if err := errors.Join(maintenanceNoThreadImpersonation(), n.server.check(), n.caller.live()); err != nil {
		return err
	}
	if err := checkDecommissionPrimary(n.server.process, n.serverToken); err != nil {
		return err
	}
	proof, err := maintenancePrimaryToken(n.caller.retainedToken, n.caller.token.sid, nativePeerEvidence(n.caller))
	if err != nil || proof != n.operator {
		return errors.Join(ErrRefused, err)
	}
	// Compare the current primary token through the SAME retained process handle.
	// No PID lookup, impersonation, duplication or privilege change is involved.
	var current windows.Token
	if err = windows.OpenProcessToken(n.caller.handle, windows.TOKEN_QUERY, &current); err != nil {
		return err
	}
	actual, proofErr := maintenancePrimaryToken(current, n.caller.token.sid, nativePeerEvidence(n.caller))
	closeErr := current.Close()
	if proofErr != nil || closeErr != nil || actual != n.operator {
		return errors.Join(ErrRefused, proofErr, closeErr)
	}
	return ctx.Err()
}
func closeDecommissionProcess(p *processIdentity) error {
	if p == nil {
		return nil
	}
	var err error
	if p.retainedToken != 0 {
		err = errors.Join(err, p.retainedToken.Close())
		p.retainedToken = 0
	}
	if p.handle != 0 {
		err = errors.Join(err, windows.CloseHandle(p.handle))
		p.handle = 0
	}
	return err
}
func (n *nativeDecommissionWitness) close() error {
	if n == nil {
		return nil
	}
	// Cleanup after STOP/cancellation must not demand a live broker. Preserve all
	// native closure uncertainty instead of the ordinary fire-and-forget closes.
	err := closeDecommissionProcess(n.caller)
	n.caller = nil
	if s := n.server; s != nil {
		err = errors.Join(err, closeDecommissionProcess(s.process))
		s.process = nil
		if s.pins != nil {
			for i := len(s.pins.handles) - 1; i >= 0; i-- {
				err = errors.Join(err, windows.CloseHandle(s.pins.handles[i]))
			}
			s.pins.handles = nil
			s.pins = nil
		}
		if s.service != 0 {
			err = errors.Join(err, windows.CloseServiceHandle(s.service))
			s.service = 0
		}
		if s.scm != 0 {
			err = errors.Join(err, windows.CloseServiceHandle(s.scm))
			s.scm = 0
		}
		n.server = nil
	}
	return err
}

// CallDecommission performs exactly one prepare action. The response is accepted
// only from the same fixed native broker and exact request, then those original
// process/token/image handles transfer into a nonserializable witness. A lost
// response can mean a permanent fence; never retry or infer an absent fence.
func CallDecommission(ctx context.Context, q MaintenanceRequest) (witness *DecommissionWitness, resultErr error) {
	if q.Operation != PrepareDecommission {
		return nil, ErrRefused
	}
	return callDecommission(ctx, q, nil)
}

// CallProtectedUninstall performs one original authenticated pipe conversation.
// restoreAndDrain receives a borrowed fence witness to restore original shortcut
// bytes and release app/launcher descendant handles before file retirement. Only
// the final matching retirement result can transfer native custody for SCM use.
// The entire action is bounded to 90 seconds; it never reconnects or replays.
func CallProtectedUninstall(ctx context.Context, q MaintenanceRequest, restoreAndDrain func(context.Context, *UninstallFenceWitness) error) (*DecommissionWitness, error) {
	if q.Operation != PrepareUninstall || restoreAndDrain == nil {
		return nil, ErrRefused
	}
	return callDecommission(ctx, q, restoreAndDrain)
}

func callDecommission(ctx context.Context, q MaintenanceRequest, restoreAndDrain func(context.Context, *UninstallFenceWitness) error) (witness *DecommissionWitness, resultErr error) {
	if ctx == nil || ctx.Err() != nil {
		return nil, ErrRefused
	}
	raw, err := encodeMaintenanceRequest(q)
	if err != nil {
		return nil, err
	}
	timeout := operationTimeout
	if q.Operation == PrepareUninstall {
		timeout = protectedUninstallTimeout
	}
	operation, cancel := context.WithTimeout(ctx, timeout)
	defer cancel()
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if err = maintenanceNoThreadImpersonation(); err != nil {
		return nil, err
	}
	n := &nativeDecommissionWitness{}
	transferred := false
	defer func() {
		if !transferred {
			resultErr = errors.Join(resultErr, n.close())
			witness = nil
		}
	}()
	n.caller, err = openProcess(windows.GetCurrentProcessId())
	if err != nil {
		return nil, err
	}
	n.operator, err = maintenancePrimaryToken(n.caller.retainedToken, n.caller.token.sid, nativePeerEvidence(n.caller))
	if err != nil {
		return nil, err
	}
	h, err := openFixedPipe(operation, MaintenancePipeName)
	if err != nil {
		return nil, err
	}
	closed := false
	defer func() {
		if !closed {
			resultErr = errors.Join(resultErr, windows.CloseHandle(h))
			witness = nil
		}
	}()
	var pid uint32
	if err = windows.GetNamedPipeServerProcessId(h, &pid); err != nil {
		return nil, errors.Join(ErrRefused, err)
	}
	n.server, err = verifyServer(pid)
	if err != nil {
		return nil, err
	}
	n.serverToken, err = decommissionPrimaryIdentity(n.server.process.retainedToken)
	if err != nil {
		return nil, err
	}
	if err = n.check(operation); err != nil {
		return nil, err
	}
	creation := uint64(n.server.process.created.HighDateTime)<<32 | uint64(n.server.process.created.LowDateTime)
	var result MaintenanceResult
	if q.Operation == PrepareUninstall {
		var incoming atomic.Bool
		check := func(checkCtx context.Context) error {
			if checkCtx == nil {
				return ErrRefused
			}
			var actual uint32
			if windows.GetNamedPipeServerProcessId(h, &actual) != nil || actual != pid {
				return ErrRefused
			}
			return errors.Join(operation.Err(), n.check(checkCtx), executionPipeIdle(h, incoming.Load()))
		}
		join := executionWatcher(operation, cancel, func() error { return check(operation) })
		defer join()
		result, err = callProtectedUninstallConversation(operation, q, n.caller.token.sid, pid, creation, executionNativeConnection{h}, check, &incoming, restoreAndDrain)
		join() // Stop before ACK: the server may disconnect immediately after it.
		if err != nil {
			return nil, err
		}
	} else {
		handshake, stop := context.WithTimeout(operation, handshakeTimeout)
		err = writeFrame(pipeIO{handshake, h}, raw)
		stop()
		if err != nil {
			return nil, errors.Join(ErrRefused, err)
		}
		response, err := readFrame(pipeIO{operation, h})
		if err != nil {
			return nil, errors.Join(ErrRefused, err)
		}
		var actual uint32
		if err = windows.GetNamedPipeServerProcessId(h, &actual); err != nil || actual != pid {
			return nil, errors.Join(ErrRefused, err)
		}
		if err = n.check(operation); err != nil {
			return nil, err
		}
		result, err = decodeMaintenanceResult(response, q)
		if err != nil || result.ClientSID != n.caller.token.sid {
			return nil, errors.Join(ErrRefused, err)
		}
	}
	ack, ackStop := context.WithTimeout(operation, handshakeTimeout)
	err = writeFrame(pipeIO{ack, h}, maintenanceAck(q))
	ackStop()
	if err != nil {
		return nil, errors.Join(ErrRefused, err)
	}
	// The server closes this conversation after ACK. The witness retains original
	// native identity custody, not a claim that this transient pipe remains open.
	err = windows.CloseHandle(h)
	closed = true
	if err != nil {
		return nil, err
	}
	if err = errors.Join(operation.Err(), n.check(ctx)); err != nil {
		return nil, err
	}
	witness, err = newDecommissionWitness(ctx, q, result, pid, creation, n)
	if err != nil {
		return nil, err
	}
	if err = operation.Err(); err != nil {
		return nil, err // Native validation/minting cannot outlast the pipe budget.
	}
	transferred = true
	return witness, nil
}
