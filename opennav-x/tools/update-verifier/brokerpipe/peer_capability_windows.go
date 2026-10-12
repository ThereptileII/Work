//go:build windows

package brokerpipe

import (
	"golang.org/x/sys/windows"
	"unsafe"
)

// The serving owner, not this view, owns pipe/process/token handles. All calls
// cease after capability revocation, and the watcher is joined before cleanup.
type nativePeerCapability struct {
	pipe        windows.Handle
	client      *processIdentity
	checkServer func() error
}

func (p *nativePeerCapability) check() error {
	if p == nil || p.pipe == 0 || p.pipe == windows.InvalidHandle || p.client == nil || p.checkServer == nil {
		return ErrRefused
	}
	if err := p.checkServer(); err != nil {
		return err
	}
	if err := p.client.live(); err != nil {
		return err
	}
	var pid uint32
	if windows.GetNamedPipeClientProcessId(p.pipe, &pid) != nil || pid != p.client.pid {
		return ErrRefused
	}
	var available uint32
	ok, _, _ := peekPipe.Call(uintptr(p.pipe), 0, 0, 0, uintptr(unsafe.Pointer(&available)), 0)
	// Current staging transport permits exactly one request. A future reviewed
	// duplex protocol must explicitly replace both this check and the watcher;
	// unsolicited bytes cannot silently turn this capability into a new protocol.
	if ok == 0 || available != 0 {
		return ErrRefused
	}
	return p.client.live()
}

func nativePeerEvidence(client *processIdentity) PeerIdentity {
	return PeerIdentity{
		Peer:            Peer{SID: client.token.sid, SessionID: client.token.session, ProcessID: client.pid},
		ProcessCreation: uint64(client.created.HighDateTime)<<32 | uint64(client.created.LowDateTime),
		LogonIDHigh:     client.token.logon.HighPart, LogonIDLow: client.token.logon.LowPart,
	}
}
