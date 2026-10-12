//go:build windows

package brokerpipe

import (
	"context"
	"encoding/json"
	"errors"

	"golang.org/x/sys/windows"
)

// ServeMaintenance is an independent fixed local endpoint on the actual SCM
// broker. It accepts only an elevated original operator; handlers still enforce
// protected client history, original application exit and profile invariants.
// Refusal discloses no filesystem path, registry value or internal error.
func ServeMaintenance(ctx context.Context, handler MaintenanceHandler) error {
	if ctx == nil || handler == nil {
		return ErrRefused
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	server, err := verifyServer(windows.GetCurrentProcessId())
	if err != nil {
		return errors.Join(ErrUnavailable, err)
	}
	defer server.close()
	replay := map[string]struct{}{}
	for {
		if err := ctx.Err(); err != nil {
			return err
		}
		if err := server.check(); err != nil {
			return errors.Join(ErrUnavailable, err)
		}
		h, err := newFixedPipe(MaintenancePipeName, server.sid)
		if err != nil {
			return errors.Join(ErrUnavailable, err)
		}
		err = serveMaintenanceConnection(ctx, h, handler, replay, server)
		if ctx.Err() != nil {
			return ctx.Err()
		}
		if err != nil {
			return errors.Join(ErrUnavailable, err)
		}
	}
}
func serveMaintenanceConnection(ctx context.Context, h windows.Handle, handler MaintenanceHandler, replay map[string]struct{}, server *serverIdentity) (resultErr error) {
	defer func() {
		err := windows.DisconnectNamedPipe(h)
		if err == windows.ERROR_PIPE_NOT_CONNECTED || err == windows.ERROR_NO_DATA || err == windows.ERROR_BROKEN_PIPE {
			err = nil
		}
		resultErr = errors.Join(resultErr, err, windows.CloseHandle(h))
	}()
	_, err := overlapped(ctx, h, func(o *windows.Overlapped) error { return windows.ConnectNamedPipe(h, o) })
	if err == nil {
		serveMaintenanceOne(ctx, h, handler, replay, server)
	}
	return err
}
func serveMaintenanceOne(parent context.Context, h windows.Handle, handler MaintenanceHandler, replay map[string]struct{}, server *serverIdentity) {
	handshake, cancel := context.WithTimeout(parent, handshakeTimeout)
	raw, err := readFrame(pipeIO{handshake, h})
	if err != nil {
		cancel()
		return
	}
	request, err := decodeMaintenanceRequest(raw)
	if err != nil {
		cancel()
		return
	}
	client, err := clientIdentity(h)
	deadline := handshake.Err()
	cancel()
	if err != nil {
		return
	}
	defer client.close()
	if deadline != nil || server.check() != nil || client.live() != nil {
		return
	}
	operation, stop := context.WithTimeout(parent, operationTimeout)
	defer stop()
	native := &nativePeerCapability{pipe: h, client: client, checkServer: server.check}
	ctx, capability := withPeerCapability(operation, nativePeerEvidence(client), native)
	defer capability.Close()
	peer := Peer{SID: client.token.sid, SessionID: client.token.session, ProcessID: client.pid}
	if capability.WithMaintenanceOperator(ctx, peer.SID, func(PeerIdentity) error { return nil }) != nil {
		return
	}
	if _, seen := replay[request.RequestID]; seen || len(replay) >= maxRequests {
		return
	}
	// Consume before invoking any handler. Durable Store journal creation remains
	// the independent replay boundary across SCM restarts.
	replay[request.RequestID] = struct{}{}
	join := executionWatcher(ctx, stop, native.check)
	defer join()
	result, err := handler(ctx, peer, request)
	join()
	if err != nil || !validMaintenanceResult(result, request) || capability.WithMaintenanceOperator(ctx, peer.SID, func(PeerIdentity) error { return nil }) != nil {
		return
	}
	response, err := json.Marshal(result)
	if err != nil || writeFrame(pipeIO{ctx, h}, response) != nil {
		return
	}
	ackCtx, ackCancel := context.WithTimeout(ctx, handshakeTimeout)
	defer ackCancel()
	ack, err := readFrame(pipeIO{ackCtx, h})
	if err != nil || !validMaintenanceAck(ack, request) {
		return
	}
}

// CallMaintenance performs one request, never retries, and verifies the actual
// fixed SCM broker both before disclosure and before accepting a completion.
// An error after transmission can mean committed effects; inspect protected
// history through a new operator action rather than repeat this request.
func CallMaintenance(ctx context.Context, q MaintenanceRequest) (result MaintenanceResult, resultErr error) {
	if ctx == nil || ctx.Err() != nil {
		return result, ErrRefused
	}
	raw, err := encodeMaintenanceRequest(q)
	if err != nil {
		return result, err
	}
	ctx, cancel := context.WithTimeout(ctx, operationTimeout)
	defer cancel()
	h, err := openFixedPipe(ctx, MaintenancePipeName)
	if err != nil {
		return result, err
	}
	defer func() {
		if err := windows.CloseHandle(h); err != nil {
			result = MaintenanceResult{}
			resultErr = errors.Join(resultErr, err)
		}
	}()
	var pid uint32
	if windows.GetNamedPipeServerProcessId(h, &pid) != nil {
		return result, ErrUnavailable
	}
	server, err := verifyServer(pid)
	if err != nil {
		return result, errors.Join(ErrUnavailable, err)
	}
	defer server.close()
	handshake, stop := context.WithTimeout(ctx, handshakeTimeout)
	err = writeFrame(pipeIO{handshake, h}, raw)
	stop()
	if err != nil {
		return result, ErrRefused
	}
	response, err := readFrame(pipeIO{ctx, h})
	if err != nil {
		return result, ErrRefused
	}
	var actual uint32
	if windows.GetNamedPipeServerProcessId(h, &actual) != nil || actual != pid || server.check() != nil {
		return result, ErrRefused
	}
	result, err = decodeMaintenanceResult(response, q)
	if err != nil {
		return MaintenanceResult{}, err
	}
	ackCtx, ackCancel := context.WithTimeout(ctx, handshakeTimeout)
	defer ackCancel()
	if err := writeFrame(pipeIO{ackCtx, h}, maintenanceAck(q)); err != nil {
		return MaintenanceResult{}, ErrRefused
	}
	return result, nil
}
