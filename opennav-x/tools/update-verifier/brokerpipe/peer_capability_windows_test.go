//go:build windows

package brokerpipe

import (
	"context"
	"crypto/rand"
	"errors"
	"fmt"
	"io"
	"runtime"
	"testing"
	"time"

	"golang.org/x/sys/windows"
)

type nativePeerFixture struct {
	ctx          context.Context
	cap          *PeerCapability
	process      *processIdentity
	backend      *nativePeerCapability
	pipe, client windows.Handle
}

// Disposable uniquely named pipe + this test process/token only. The private
// server checker bypasses installed SCM authentication; this fixture establishes
// no broker principal, protected launcher image, app execution or service claim.
func actualNativePeerFixture(t *testing.T) *nativePeerFixture {
	t.Helper()
	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	t.Cleanup(cancel)
	var random [16]byte
	if _, err := rand.Read(random[:]); err != nil {
		t.Fatal(err)
	}
	name, err := windows.UTF16PtrFromString(fmt.Sprintf(`\\.\pipe\SkagerPeerFixture-%x`, random))
	if err != nil {
		t.Fatal(err)
	}
	f := &nativePeerFixture{}
	f.pipe, err = windows.CreateNamedPipe(name, windows.PIPE_ACCESS_DUPLEX|windows.FILE_FLAG_FIRST_PIPE_INSTANCE|windows.FILE_FLAG_OVERLAPPED, windows.PIPE_REJECT_REMOTE_CLIENTS, 1, 4096, 4096, 0, nil)
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() {
		if f.cap != nil {
			f.cap.Close()
		}
		if f.process != nil {
			f.process.close()
		}
		if f.client != 0 {
			if err := windows.CloseHandle(f.client); err != nil {
				t.Error(err)
			}
			f.client = 0
		}
		if f.pipe != 0 {
			windows.DisconnectNamedPipe(f.pipe)
			if err := windows.CloseHandle(f.pipe); err != nil {
				t.Error(err)
			}
			f.pipe = 0
		}
	})
	f.client, err = windows.CreateFile(name, clientPipeRights, 0, nil, windows.OPEN_EXISTING, windows.FILE_FLAG_OVERLAPPED|windows.SECURITY_SQOS_PRESENT|windows.SECURITY_IDENTIFICATION, 0)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := overlapped(ctx, f.pipe, func(o *windows.Overlapped) error { return windows.ConnectNamedPipe(f.pipe, o) }); err != nil {
		t.Fatal(err)
	}
	if _, err := (pipeIO{ctx, f.client}).Write([]byte{1}); err != nil {
		t.Fatal(err)
	}
	var request [1]byte
	if _, err := io.ReadFull(pipeIO{ctx, f.pipe}, request[:]); err != nil {
		t.Fatal(err)
	}
	f.process, err = clientIdentity(f.pipe)
	if err != nil {
		t.Fatal(err)
	}
	f.backend = &nativePeerCapability{pipe: f.pipe, client: f.process, checkServer: func() error { return nil }}
	f.ctx, f.cap = withPeerCapability(ctx, nativePeerEvidence(f.process), f.backend)
	return f
}

func TestNativePeerCapabilityActualIdentityAndDisconnect(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	f := actualNativePeerFixture(t)
	identity := nativePeerEvidence(f.process)
	if identity.ProcessID != windows.GetCurrentProcessId() || identity.ProcessCreation == 0 || identity.SID == "" || identity.SessionID == 0 || (identity.LogonIDHigh == 0 && identity.LogonIDLow == 0) {
		t.Fatal("actual native identity unavailable")
	}
	if cap, err := AuthorizePeer(f.ctx, identity.Peer); err != nil || cap != f.cap {
		t.Fatal("actual connected peer refused", err)
	}
	if err := f.cap.WithPeer(f.ctx, func(got PeerIdentity) error {
		if got != identity {
			return ErrRefused
		}
		return nil
	}); err != nil {
		t.Fatal(err)
	}
	if err := windows.CloseHandle(f.client); err != nil {
		t.Fatal(err)
	}
	f.client = 0
	if !errors.Is(f.cap.Validate(f.ctx), ErrPeerRevoked) {
		t.Fatal("disconnected native caller remained authorized")
	}
	if _, err := AuthorizePeer(f.ctx, identity.Peer); !errors.Is(err, ErrPeerRevoked) {
		t.Fatal("same public identity restored disconnected caller")
	}
}

func TestNativePeerCapabilityProcessAndLogonRefusals(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	for _, scenario := range []string{"closed-process", "creation-mismatch", "pid-reuse", "logon-mismatch", "closed-token", "signaled-handle", "unsolicited-bytes"} {
		t.Run(scenario, func(t *testing.T) {
			f := actualNativePeerFixture(t)
			identity := nativePeerEvidence(f.process)
			if f.cap.Validate(f.ctx) != nil {
				t.Fatal("live native fixture refused")
			}
			switch scenario {
			case "closed-process":
				if err := windows.CloseHandle(f.process.handle); err != nil {
					t.Fatal(err)
				}
				f.process.handle = 0
			case "creation-mismatch":
				f.process.created.LowDateTime ^= 1
			case "pid-reuse":
				f.process.pid++
			case "logon-mismatch":
				f.process.token.logon.LowPart ^= 1
			case "closed-token":
				if err := f.process.retainedToken.Close(); err != nil {
					t.Fatal(err)
				}
				f.process.retainedToken = 0
			case "signaled-handle":
				// The native exit-state branch must reject a signaled retained object before
				// any process/token/image use; this does not claim an actual child execution.
				event, err := windows.CreateEvent(nil, 1, 1, nil)
				if err != nil {
					t.Fatal(err)
				}
				t.Cleanup(func() { windows.CloseHandle(event) })
				dead := *f.process
				dead.handle = event
				f.backend.client = &dead
			case "unsolicited-bytes":
				if _, err := (pipeIO{f.ctx, f.client}).Write([]byte{2}); err != nil {
					t.Fatal(err)
				}
			}
			if !errors.Is(f.cap.Validate(f.ctx), ErrPeerRevoked) {
				t.Fatal("native identity/lifetime change authorized", scenario)
			}
			if _, err := AuthorizePeer(f.ctx, identity.Peer); !errors.Is(err, ErrPeerRevoked) {
				t.Fatal("public metadata restored native authority", scenario)
			}
		})
	}
}
