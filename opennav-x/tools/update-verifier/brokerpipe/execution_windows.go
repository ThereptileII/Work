//go:build windows

package brokerpipe

import (
	"context"
	"errors"
	"sync"
	"sync/atomic"
	"time"
	"unsafe"

	"golang.org/x/sys/windows"
)

type executionNativePeer struct {
	native   *nativePeerCapability
	incoming *atomic.Bool
}

func (p *executionNativePeer) check() error {
	if p == nil || p.native == nil {
		return ErrRefused
	}
	return p.native.checkInput(p.incoming != nil && p.incoming.Load())
}

type executionNativeConnection struct{ pipe windows.Handle }

func (p executionNativeConnection) read(ctx context.Context) ([]byte, error) {
	return readFrame(pipeIO{ctx, p.pipe})
}
func (p executionNativeConnection) write(ctx context.Context, raw []byte) error {
	return writeFrame(pipeIO{ctx, p.pipe}, raw)
}
func (p executionNativeConnection) idle() error { return executionPipeIdle(p.pipe, false) }
func executionPipeIdle(h windows.Handle, allowInput bool) error {
	var available uint32
	ok, _, _ := peekPipe.Call(uintptr(h), 0, 0, 0, uintptr(unsafe.Pointer(&available)), 0)
	if ok == 0 || (!allowInput && available != 0) {
		return ErrRefused
	}
	return nil
}

// The physical serving/client owner joins this watcher before releasing any
// process/token/SCM/pipe handles. The checker never takes the session mutex, so
// cancellation can interrupt its bounded OVERLAPPED read while Exchange holds it.
func executionWatcher(ctx context.Context, cancel context.CancelFunc, check func() error) func() {
	done, stop := make(chan struct{}), make(chan struct{})
	go func() {
		defer close(done)
		ticker := time.NewTicker(100 * time.Millisecond)
		defer ticker.Stop()
		for {
			select {
			case <-ctx.Done():
				return
			case <-stop:
				return
			case <-ticker.C:
				if check() != nil {
					cancel()
					return
				}
			}
		}
	}()
	var once sync.Once
	return func() { once.Do(func() { close(stop) }); <-done }
}

// ServeExecution serves only the fixed separate execution endpoint from the
// actual installed SCM own-process broker. Handlers must bind the protected
// executor image before issuing phases, independently authorize durable effects,
// obey cancellation and explicitly exchange finish. A clean conversation is not
// installation, launch, health, rollback, selection or transaction acceptance.
// There is one active connection, no retry/queue and no client-selected endpoint.
func ServeExecution(ctx context.Context, handler ExecutionHandler) error {
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
	var replay executionRequests
	for {
		if err := ctx.Err(); err != nil {
			return err
		}
		if err := server.check(); err != nil {
			return errors.Join(ErrUnavailable, err)
		}
		h, err := newExecutionPipe(server.sid)
		if err != nil {
			return errors.Join(ErrUnavailable, err)
		}
		err = serveExecutionConnection(ctx, h, handler, &replay, server)
		if ctx.Err() != nil {
			return ctx.Err()
		}
		if err != nil {
			return errors.Join(ErrUnavailable, err)
		}
	}
}
func serveExecutionConnection(ctx context.Context, h windows.Handle, handler ExecutionHandler, replay *executionRequests, server *serverIdentity) (err error) {
	defer func() {
		disconnectErr := windows.DisconnectNamedPipe(h)
		closeErr := windows.CloseHandle(h)
		if disconnectErr == windows.ERROR_PIPE_NOT_CONNECTED || disconnectErr == windows.ERROR_NO_DATA || disconnectErr == windows.ERROR_BROKEN_PIPE {
			disconnectErr = nil
		}
		err = errors.Join(err, disconnectErr, closeErr)
	}()
	_, err = overlapped(ctx, h, func(o *windows.Overlapped) error { return windows.ConnectNamedPipe(h, o) })
	if err == nil {
		serveExecutionOne(ctx, h, handler, replay, server)
	}
	// Never FlushFileBuffers: an unresponsive peer cannot indefinitely hold shutdown.
	return err
}
func serveExecutionOne(parent context.Context, h windows.Handle, handler ExecutionHandler, replay *executionRequests, server *serverIdentity) {
	handshake, cancel := context.WithTimeout(parent, handshakeTimeout)
	raw, err := readFrame(pipeIO{handshake, h})
	if err != nil {
		cancel()
		return
	}
	request, err := decodeExecutionRequest(raw)
	clear(raw) // Do not retain the credential-bearing handshake frame.
	if err != nil {
		cancel()
		return
	}
	client, err := clientIdentity(h)
	deadlineErr := handshake.Err()
	cancel()
	if err != nil {
		return
	}
	defer client.close()
	if deadlineErr != nil || server.check() != nil || client.live() != nil {
		return
	}
	operation, stop := context.WithTimeout(parent, operationTimeout)
	defer stop()
	native := &executionNativePeer{native: &nativePeerCapability{pipe: h, client: client, checkServer: server.check}}
	ctx, capability := withPeerCapability(operation, nativePeerEvidence(client), native)
	defer capability.Close()
	session, err := newExecutionSession(ctx, stop, request, peerExecutionAuthority{capability}, executionNativeConnection{h})
	if err != nil {
		return
	}
	native.incoming = &session.state.incoming
	defer session.Close()
	joinWatcher := executionWatcher(ctx, stop, native.check)
	defer joinWatcher()
	peer := nativePeerEvidence(client).Peer
	if _, err := AuthorizePeer(ctx, peer); err != nil {
		return
	}
	if replay.reserve(peer, request) != nil {
		return
	}
	if invokeExecutionHandler(ctx, peer, request, handler, session) != nil {
		return
	}
	// Business authority is now revoked; stop/join before accepting the one exact
	// transport acknowledgement. Its bytes are not a new business report.
	joinWatcher()
	if ctx.Err() != nil || native.check() != nil {
		return
	}
	terminal := session.state.terminal // closed and no further writer remains
	closing, err := executionClosingFrame(terminal, "closed")
	if err != nil {
		return
	}
	ackCtx, ackCancel := context.WithTimeout(ctx, handshakeTimeout)
	defer ackCancel()
	if writeFrame(pipeIO{ackCtx, h}, closing) != nil {
		return
	}
	raw, err = readFrame(pipeIO{ackCtx, h})
	if err != nil || !validExecutionClosingFrame(raw, terminal, "closed-ack") {
		return
	}
	if native.check() != nil {
		return
	} // No post-ack result or transaction authority is emitted.
}

// CallExecution independently authenticates/retains the real SCM broker and
// services only exact server-issued phases. Handler observations are untrusted
// data, never proof of effects. Nil means a matching finish/close was observed and
// its acknowledgement sent; no transaction/health/launch/selection success. Callers
// must obey ctx. Current and offer startup have bounded preconnection BUSY handoff
// readiness; initial missing still refuses. No connected request/effect is replayed.
func CallExecution(ctx context.Context, request ExecutionRequest, handler ExecutionClientHandler) (resultErr error) {
	if ctx == nil || ctx.Err() != nil || handler == nil {
		return ErrRefused
	}
	raw, err := encodeExecutionRequest(request)
	if err != nil {
		return err
	}
	defer clear(raw) // Covers server-authentication and other pre-write failures.
	ctx, cancel := context.WithTimeout(ctx, operationTimeout)
	defer cancel()
	open := openExecutionPipe
	if request.Operation == "start-current" || request.Operation == "offer-current" {
		open = openExecutionCurrentPipe
	}
	h, err := open(ctx)
	if err != nil {
		return err
	}
	defer func() { resultErr = errors.Join(resultErr, windows.CloseHandle(h)) }()
	var pid uint32
	if windows.GetNamedPipeServerProcessId(h, &pid) != nil {
		return ErrUnavailable
	}
	server, err := verifyServer(pid)
	if err != nil {
		return errors.Join(ErrUnavailable, err)
	}
	defer server.close()
	var incoming atomic.Bool
	check := func() error {
		var actual uint32
		if windows.GetNamedPipeServerProcessId(h, &actual) != nil || actual != pid {
			return ErrRefused
		}
		if err := server.check(); err != nil {
			return err
		}
		return executionPipeIdle(h, incoming.Load())
	}
	// Input is armed before request publication: the broker may promptly issue its
	// first command. Every command is still checked against the bounded sequence.
	incoming.Store(true)
	joinWatcher := executionWatcher(ctx, cancel, check)
	defer joinWatcher()
	handshake, stop := context.WithTimeout(ctx, handshakeTimeout)
	err = writeFrame(pipeIO{handshake, h}, raw)
	clear(raw)
	stop()
	if err != nil {
		return errors.Join(ErrRefused, ctx.Err())
	}
	sequence := executionClientSequence{request: request}
	for i := 0; i < maxExecutionPhases; i++ {
		phaseCtx, phaseCancel := context.WithTimeout(ctx, executionPhaseTimeout)
		raw, err = readFrame(pipeIO{phaseCtx, h})
		if err != nil {
			phaseCancel()
			return errors.Join(ErrRefused, ctx.Err())
		}
		incoming.Store(false)
		command, err := decodeExecutionCommand(raw, "command")
		if err != nil || sequence.accept(command) != nil || check() != nil {
			phaseCancel()
			return ErrRefused
		}
		observation, err := handler(phaseCtx, command)
		if err != nil || phaseCtx.Err() != nil || check() != nil {
			phaseCancel()
			return errors.Join(ErrRefused, err, phaseCtx.Err())
		}
		raw, err = encodeExecutionReport(command, observation)
		if err != nil {
			phaseCancel()
			return err
		}
		incoming.Store(true)
		err = writeFrame(pipeIO{phaseCtx, h}, raw)
		phaseCancel()
		if err != nil {
			return errors.Join(ErrRefused, ctx.Err())
		}
		if command.Phase == FinishExecution {
			ackCtx, ackCancel := context.WithTimeout(ctx, handshakeTimeout)
			defer ackCancel()
			raw, err = readFrame(pipeIO{ackCtx, h})
			incoming.Store(false)
			if err != nil || !validExecutionClosingFrame(raw, command, "closed") || check() != nil {
				return ErrRefused
			}
			// No further inbound bytes are permitted after the exact close frame.
			joinWatcher()
			raw, err = executionClosingFrame(command, "closed-ack")
			if err != nil || writeFrame(pipeIO{ackCtx, h}, raw) != nil {
				return ErrRefused
			}
			// A server may disconnect immediately after consuming the ack. Recheck the
			// retained SCM/process itself, rather than requiring an extra wire round trip.
			return errors.Join(ctx.Err(), server.check())
		}
	}
	return ErrRefused
}
