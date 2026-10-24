//go:build windows

package brokerpipe

import (
	"context"
	"encoding/json"
	"errors"
	"strings"
	"time"
	"unsafe"

	"golang.org/x/sys/windows"
)

var peekPipe = windows.NewLazySystemDLL("kernel32.dll").NewProc("PeekNamedPipe")

type serverIdentity struct {
	scm, service windows.Handle
	process      *processIdentity
	pins         *imagePins
	sid          string
}

func (s *serverIdentity) close() {
	if s == nil {
		return
	}
	s.process.close()
	if s.pins != nil {
		s.pins.close()
	}
	if s.service != 0 {
		_ = windows.CloseServiceHandle(s.service)
	}
	if s.scm != 0 {
		_ = windows.CloseServiceHandle(s.scm)
	}
}
func (s *serverIdentity) check() error {
	if s == nil || s.process == nil || s.process.token.sid != s.sid || s.process.token.session != 0 {
		return ErrRefused
	}
	var status windows.SERVICE_STATUS_PROCESS
	var used uint32
	if windows.QueryServiceStatusEx(s.service, windows.SC_STATUS_PROCESS_INFO, (*byte)(unsafe.Pointer(&status)), uint32(unsafe.Sizeof(status)), &used) != nil || status.ProcessId != s.process.pid || status.ServiceType != windows.SERVICE_WIN32_OWN_PROCESS || status.CurrentState != windows.SERVICE_RUNNING {
		return ErrRefused
	}
	return s.pins.matches(s.process)
}
func verifyServer(pid uint32) (s *serverIdentity, err error) {
	s = &serverIdentity{}
	owned := s
	defer func() {
		if err != nil {
			owned.close()
		}
	}()
	s.sid, err = brokerSID()
	if err != nil {
		return nil, err
	}
	expected, err := expectedImage()
	if err != nil {
		return nil, err
	}
	// Hold the protected actual module and every ancestor before inspecting the
	// live SCM/process identity. A caller-provided hash/path is never authority.
	s.pins, err = pinImage(expected)
	if err != nil {
		return nil, err
	}
	s.scm, err = windows.OpenSCManager(nil, nil, windows.SC_MANAGER_CONNECT)
	if err != nil {
		return nil, err
	}
	name, _ := windows.UTF16PtrFromString(ServiceName)
	s.service, err = windows.OpenService(s.scm, name, windows.SERVICE_QUERY_CONFIG|windows.SERVICE_QUERY_STATUS|windows.READ_CONTROL)
	if err != nil {
		return nil, err
	}
	mutation := uint32(windows.SERVICE_CHANGE_CONFIG | windows.SERVICE_START | windows.SERVICE_STOP | windows.SERVICE_PAUSE_CONTINUE | windows.SERVICE_USER_DEFINED_CONTROL | windows.DELETE | windows.WRITE_DAC | windows.WRITE_OWNER | windows.GENERIC_ALL | windows.GENERIC_WRITE)
	if err = protectedACL(s.service, windows.SE_SERVICE, mutation); err != nil {
		return nil, err
	}
	// Bound the SCM configuration buffer rather than trusting registry lengths.
	buffer := make([]byte, 8192)
	var used uint32
	config := (*windows.QUERY_SERVICE_CONFIG)(unsafe.Pointer(&buffer[0]))
	if err = windows.QueryServiceConfig(s.service, config, uint32(len(buffer)), &used); err != nil {
		return nil, err
	}
	if config.ServiceType != windows.SERVICE_WIN32_OWN_PROCESS || !strings.EqualFold(windows.UTF16PtrToString(config.ServiceStartName), serviceAccount) || !strings.EqualFold(windows.UTF16PtrToString(config.BinaryPathName), `"`+expected+`"`) {
		return nil, ErrRefused
	}
	s.process, err = openProcess(pid)
	if err != nil {
		return nil, err
	}
	if err = s.check(); err != nil {
		return nil, err
	}
	return s, nil
}

// Serve requires the real installed, running SCM own-process virtual-account
// service. It does not turn an ordinary same-user console process into a broker.
// Exactly one instance/request is active. Other clients receive pipe-busy; no
// unbounded handler goroutines, queued jobs, automatic retries or child launches.
func Serve(ctx context.Context, handler Handler) error {
	if ctx == nil || handler == nil {
		return ErrRefused
	}
	if ctx.Err() != nil {
		return ctx.Err()
	}
	identity, e := verifyServer(windows.GetCurrentProcessId())
	if e != nil {
		return errors.Join(ErrUnavailable, e)
	}
	defer identity.close()
	var replay requests
	for {
		if e = ctx.Err(); e != nil {
			return e
		}
		if e = identity.check(); e != nil {
			return errors.Join(ErrUnavailable, e)
		}
		h, e := newPipe(identity.sid)
		if e != nil {
			return errors.Join(ErrUnavailable, e)
		}
		e = serveConnection(ctx, h, handler, &replay, identity)
		if ctx.Err() != nil {
			return ctx.Err()
		}
		if e != nil {
			return errors.Join(ErrUnavailable, e)
		}
	}
}

// All per-connection handles close even if a handler panics. Cleanup failure
// stops Serve; it cannot silently accumulate a second active connection.
func serveConnection(ctx context.Context, h windows.Handle, handler Handler, replay *requests, identity *serverIdentity) (err error) {
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
		serveOne(ctx, h, handler, replay, identity)
	}
	// No FlushFileBuffers: a malicious peer cannot hold shutdown indefinitely.
	return err
}

func serveOne(parent context.Context, h windows.Handle, handler Handler, replay *requests, server *serverIdentity) {
	handshake, cancel := context.WithTimeout(parent, handshakeTimeout)
	raw, e := readFrame(pipeIO{handshake, h})
	if e != nil {
		cancel()
		return
	}
	request, e := decodeRequest(raw)
	clear(raw) // The decoded credential is scoped to this authenticated operation.
	if e != nil {
		cancel()
		return
	}
	client, e := clientIdentity(h)
	deadlineErr := handshake.Err()
	cancel()
	if e != nil {
		return
	}
	defer client.close()
	if deadlineErr != nil {
		return
	}
	if server.check() != nil || client.live() != nil {
		return
	}
	operation, stop := context.WithTimeout(parent, operationTimeout)
	peer := Peer{SID: client.token.sid, SessionID: client.token.session, ProcessID: client.pid}
	ctx, capability := withPeerCapability(operation, nativePeerEvidence(client), &nativePeerCapability{pipe: h, client: client, checkServer: server.check})
	defer capability.Close()
	// One bounded watcher is joined before the retained process/pipe is closed.
	// It cancels a cooperative handler when the actual caller exits/disconnects,
	// or sends an unsolicited second request while staging is in progress.
	watcherDone := make(chan struct{})
	watcherStop := make(chan struct{})
	go func() {
		defer close(watcherDone)
		tick := time.NewTicker(100 * time.Millisecond)
		defer tick.Stop()
		for {
			select {
			case <-ctx.Done():
				return
			case <-watcherStop:
				return
			case <-tick.C:
				var available uint32
				ok, _, _ := peekPipe.Call(uintptr(h), 0, 0, 0, uintptr(unsafe.Pointer(&available)), 0)
				if client.live() != nil || ok == 0 || available != 0 {
					stop()
					return
				}
			}
		}
	}()
	defer func() { stop(); <-watcherDone }()
	result, e := invokeAuthenticatedHandler(ctx, capability, replay, handler, peer, request)
	// Stop/join the watcher before the response acknowledgement is permitted.
	// Its cancellation would otherwise mistake the valid ack for a second request.
	close(watcherStop)
	<-watcherDone
	// Recheck the retained process/SCM identities before disclosing even a stage
	// result. A disconnected/dead caller never receives a success on a retry.
	if e != nil || client.live() != nil || server.check() != nil {
		return
	}
	response, e := json.Marshal(result)
	if e != nil {
		return
	}
	if writeFrame(pipeIO{ctx, h}, response) != nil {
		return
	}
	ackCtx, ackCancel := context.WithTimeout(ctx, handshakeTimeout)
	defer ackCancel()
	ack, e := readFrame(pipeIO{ackCtx, h})
	if e != nil || !validAck(ack, request) {
		return
	}
}

func Call(ctx context.Context, request Request) (Result, error) {
	if ctx == nil || ctx.Err() != nil {
		return Result{}, ErrRefused
	}
	raw, e := encodeRequest(request)
	if e != nil {
		return Result{}, e
	}
	defer clear(raw) // Also clear on pre-authentication/open failure.
	ctx, cancel := context.WithTimeout(ctx, operationTimeout)
	defer cancel()
	h, e := openPipe(ctx)
	if e != nil {
		return Result{}, e
	}
	defer windows.CloseHandle(h)
	var pid uint32
	if windows.GetNamedPipeServerProcessId(h, &pid) != nil {
		return Result{}, ErrUnavailable
	}
	server, e := verifyServer(pid)
	if e != nil {
		return Result{}, errors.Join(ErrUnavailable, e)
	}
	defer server.close()
	handshake, stop := context.WithTimeout(ctx, handshakeTimeout)
	e = writeFrame(pipeIO{handshake, h}, raw)
	clear(raw)
	stop()
	if e != nil {
		return Result{}, ErrRefused
	}
	response, e := readFrame(pipeIO{ctx, h})
	if e != nil {
		return Result{}, ErrRefused
	}
	var actual uint32
	if windows.GetNamedPipeServerProcessId(h, &actual) != nil || actual != pid || server.check() != nil {
		return Result{}, ErrRefused
	}
	result, e := decodeResult(response, request)
	if e != nil {
		return Result{}, e
	}
	ackCtx, ackCancel := context.WithTimeout(ctx, handshakeTimeout)
	defer ackCancel()
	if writeFrame(pipeIO{ackCtx, h}, encodeAck(request)) != nil {
		return Result{}, ErrRefused
	}
	return result, nil
}
