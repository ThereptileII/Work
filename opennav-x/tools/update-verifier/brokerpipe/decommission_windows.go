//go:build windows

package brokerpipe

import (
	"context"
	"errors"
	"runtime"
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
	if ctx == nil || ctx.Err() != nil || (q.Operation != PrepareDecommission && q.Operation != PrepareUninstall) {
		return nil, ErrRefused
	}
	raw, err := encodeMaintenanceRequest(q)
	if err != nil {
		return nil, err
	}
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
	operation, cancel := context.WithTimeout(ctx, operationTimeout)
	defer cancel()
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
	result, err := decodeMaintenanceResult(response, q)
	if err != nil || result.ClientSID != n.caller.token.sid {
		return nil, errors.Join(ErrRefused, err)
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
	if err = n.check(ctx); err != nil {
		return nil, err
	}
	creation := uint64(n.server.process.created.HighDateTime)<<32 | uint64(n.server.process.created.LowDateTime)
	witness, err = newDecommissionWitness(ctx, q, result, pid, creation, n)
	if err != nil {
		return nil, err
	}
	transferred = true
	return witness, nil
}
