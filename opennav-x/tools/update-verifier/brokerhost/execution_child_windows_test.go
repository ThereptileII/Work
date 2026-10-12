package brokerhost

import (
	"bytes"
	"context"
	"errors"
	"os"
	"path/filepath"
	"runtime"
	"strings"
	"testing"
	"unsafe"

	"golang.org/x/sys/windows"
)

func nativeExecutionPipeFixture(t *testing.T) (windows.Handle, string, string) {
	t.Helper()
	user, err := windows.GetCurrentProcessToken().GetTokenUser()
	if err != nil {
		t.Fatal(err)
	}
	broker := user.User.Sid.String()
	client := "S-1-5-21-1-2-3-500"
	sd, err := executionChildPipeDescriptor(broker, client)
	if err != nil {
		t.Fatal(err)
	}
	name := `\\.\pipe\Skager.InertExecutionFixture.` + strings.ReplaceAll(filepath.Base(t.TempDir()), " ", "") + "." + strings.ReplaceAll(t.Name(), "/", ".")
	p, err := windows.UTF16PtrFromString(name)
	if err != nil {
		t.Fatal(err)
	}
	sa := windows.SecurityAttributes{Length: uint32(unsafe.Sizeof(windows.SecurityAttributes{})), SecurityDescriptor: sd}
	h, err := windows.CreateNamedPipe(p, windows.PIPE_ACCESS_INBOUND|windows.FILE_FLAG_FIRST_PIPE_INSTANCE|windows.FILE_FLAG_OVERLAPPED, windows.PIPE_REJECT_REMOTE_CLIENTS, 1, 256, 1024, 0, &sa)
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() {
		if err := windows.CloseHandle(h); err != nil {
			t.Error(err)
		}
	})
	return h, name, client
}
func TestNativeExecutionChildHealthACLExcludesClientInstanceControl(t *testing.T) {
	if unsafe.Sizeof(executionTokenStatistics{}) != 56 || unsafe.Offsetof(executionTokenStatistics{}.AuthenticationID) != 8 || unsafe.Offsetof(executionTokenStatistics{}.ModifiedID) != 48 {
		t.Fatal("native token layout")
	}
	h, _, client := nativeExecutionPipeFixture(t)
	sd, err := windows.GetSecurityInfo(h, windows.SE_KERNEL_OBJECT, windows.DACL_SECURITY_INFORMATION)
	if err != nil {
		t.Fatal(err)
	}
	acl, _, err := sd.DACL()
	if err != nil || acl == nil {
		t.Fatal("actual pipe ACL", err)
	}
	found := false
	for index := uint32(0); index < uint32(acl.AceCount); index++ {
		var ace *windows.ACCESS_ALLOWED_ACE
		if err = windows.GetAce(acl, index, &ace); err != nil {
			t.Fatal(err)
		}
		if ace.Header.AceType != windows.ACCESS_ALLOWED_ACE_TYPE {
			t.Fatal("unexpected ACE")
		}
		sid := (*windows.SID)(unsafe.Pointer(&ace.SidStart))
		if sid.String() == client {
			found = true
			if uint32(ace.Mask) != executionChildClientRights || uint32(ace.Mask)&(windows.FILE_APPEND_DATA|windows.GENERIC_WRITE|windows.GENERIC_ALL|windows.WRITE_DAC|windows.WRITE_OWNER|windows.DELETE) != 0 {
				t.Fatalf("actual client rights %08x", ace.Mask)
			}
		}
	}
	if !found {
		t.Fatal("missing specific client grant")
	}
}
func TestNativeExecutionChildOverlappedConnectionReadAndCancelDrain(t *testing.T) {
	t.Run("pending-connect-cancel", func(t *testing.T) {
		h, _, _ := nativeExecutionPipeFixture(t)
		o, err := beginExecutionChildIO(h, func(ov *windows.Overlapped) error { return windows.ConnectNamedPipe(h, ov) })
		if err != nil {
			t.Fatal(err)
		}
		ctx, cancel := context.WithCancel(context.Background())
		cancel()
		if _, err = o.finish(ctx, func() error { return nil }); !errors.Is(err, context.Canceled) {
			t.Fatal(err)
		}
		if o.event != 0 || !o.finished {
			t.Fatal("cancelled connection escaped lifetime")
		}
		if err = o.cancelAndClose(); err != nil {
			t.Fatal("repeat close", err)
		}
	})
	t.Run("actual-client-message-and-read-cancel", func(t *testing.T) {
		h, name, _ := nativeExecutionPipeFixture(t)
		connect, err := beginExecutionChildIO(h, func(ov *windows.Overlapped) error { return windows.ConnectNamedPipe(h, ov) })
		if err != nil {
			t.Fatal(err)
		}
		defer connect.cancelAndClose()
		p, err := windows.UTF16PtrFromString(name)
		if err != nil {
			t.Fatal(err)
		}
		client, err := windows.CreateFile(p, executionChildClientRights, 0, nil, windows.OPEN_EXISTING, windows.SECURITY_SQOS_PRESENT|windows.SECURITY_IDENTIFICATION, 0)
		if err != nil {
			t.Fatal("specific original producer rights", err)
		}
		defer windows.CloseHandle(client)
		if _, err = connect.finish(context.Background(), func() error { return nil }); err != nil {
			t.Fatal(err)
		}
		var pid uint32
		if err = windows.GetNamedPipeClientProcessId(h, &pid); err != nil || pid != windows.GetCurrentProcessId() {
			t.Fatal("actual kernel client PID", pid, err)
		}
		payload := []byte("inert message only; never application health\n")
		var written uint32
		if err = windows.WriteFile(client, payload, &written, nil); err != nil || int(written) != len(payload) {
			t.Fatal("native client write", written, err)
		}
		buffer := make([]byte, 128)
		read, err := beginExecutionChildIO(h, func(ov *windows.Overlapped) error { return windows.ReadFile(h, buffer, nil, ov) })
		if err != nil {
			t.Fatal(err)
		}
		n, err := read.finish(context.Background(), func() error { return nil })
		if err != nil || !bytes.Equal(buffer[:n], payload) {
			t.Fatal("actual message", n, err)
		}
		pending, err := beginExecutionChildIO(h, func(ov *windows.Overlapped) error { return windows.ReadFile(h, buffer, nil, ov) })
		if err != nil {
			t.Fatal(err)
		}
		ctx, cancel := context.WithCancel(context.Background())
		cancel()
		if _, err = pending.finish(ctx, func() error { return nil }); !errors.Is(err, context.Canceled) {
			t.Fatal(err)
		}
		if pending.event != 0 || !pending.finished {
			t.Fatal("read cancellation escaped")
		}
		// A kernel PID/message alone has no public constructor or original app owner.
		unowned := &nativeExecutionChild{pipe: h, connected: true}
		defer func() {
			if err := unowned.child.close(); err != nil {
				t.Error(err)
			}
		}()
		if _, err = unowned.authenticate(context.Background()); err == nil {
			t.Fatal("unowned message became application evidence")
		}
	})
}
func TestNativeExecutionColdProfileOriginalBytesAndReleasedINILock(t *testing.T) {
	directory := filepath.Join(t.TempDir(), "opencpn")
	if err := os.Mkdir(directory, 0700); err != nil {
		t.Fatal(err)
	}
	path := filepath.Join(directory, "opencpn.ini")
	raw := []byte("[Settings]\nConnection=COM8\nUnknownPreserved=1\n")
	if err := os.WriteFile(path, raw, 0600); err != nil {
		t.Fatal(err)
	}
	p, err := captureExecutionProfilePath(context.Background(), path)
	if err != nil {
		t.Fatal(err)
	}
	defer p.Close()
	original, err := p.OriginalBytes(context.Background())
	if err != nil || !bytes.Equal(original, raw) {
		t.Fatal("native original", err)
	}
	if err = p.ValidateBeforeLaunch(context.Background()); err != nil {
		t.Fatal(err)
	}
	renamed := directory + "-renamed"
	if err = os.Rename(directory, renamed); !errors.Is(err, windows.ERROR_SHARING_VIOLATION) && !errors.Is(err, windows.ERROR_ACCESS_DENIED) {
		t.Fatal("retained directory was renameable", err)
	}
	// The application's legitimate normal Flush is possible after capture.
	if err = os.WriteFile(path, []byte("[Settings]\nConnection=COM8\nNormalFlush=1\n"), 0600); err != nil {
		t.Fatal("snapshot leaked INI write exclusion", err)
	}
	if err = p.Validate(context.Background()); err != nil {
		t.Fatal("normal later INI write rejected by ancestry-only check", err)
	}
	if err = p.ValidateBeforeLaunch(context.Background()); err == nil {
		t.Fatal("changed INI accepted before next launch")
	}
	if err = p.Close(); err != nil {
		t.Fatal(err)
	}
	if err = os.Rename(directory, renamed); err != nil {
		t.Fatal("directory pin leaked", err)
	}
}
func TestNativeExecutionColdProfileRefusesReplacementLinksAndUnsafeSources(t *testing.T) {
	for _, mode := range []string{"same-bytes-replacement", "hardlink", "held-writer", "oversize", "cancelled"} {
		t.Run(mode, func(t *testing.T) {
			path := filepath.Join(t.TempDir(), "opencpn.ini")
			raw := []byte("[Settings]\nConnection=COM8\n")
			if err := os.WriteFile(path, raw, 0600); err != nil {
				t.Fatal(err)
			}
			if mode == "same-bytes-replacement" {
				p, err := captureExecutionProfilePath(context.Background(), path)
				if err != nil {
					t.Fatal(err)
				}
				defer p.Close()
				if err = os.Rename(path, path+".old"); err != nil {
					t.Fatal(err)
				}
				if err = os.WriteFile(path, raw, 0600); err != nil {
					t.Fatal(err)
				}
				if err = p.ValidateBeforeLaunch(context.Background()); err == nil {
					t.Fatal("replacement physical identity adopted")
				}
				return
			}
			if mode == "hardlink" {
				if err := os.Link(path, path+".link"); err != nil {
					t.Fatal(err)
				}
			}
			if mode == "held-writer" {
				name, err := windows.UTF16PtrFromString(path)
				if err != nil {
					t.Fatal(err)
				}
				h, err := windows.CreateFile(name, windows.GENERIC_WRITE, windows.FILE_SHARE_READ|windows.FILE_SHARE_WRITE|windows.FILE_SHARE_DELETE, nil, windows.OPEN_EXISTING, 0, 0)
				if err != nil {
					t.Fatal(err)
				}
				defer windows.CloseHandle(h)
			}
			if mode == "oversize" {
				file, err := os.OpenFile(path, os.O_WRONLY, 0600)
				if err != nil {
					t.Fatal(err)
				}
				err = file.Truncate(executionProfileLimit + 1)
				closeErr := file.Close()
				if err != nil || closeErr != nil {
					t.Fatal(err, closeErr)
				}
			}
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			if mode == "cancelled" {
				cancel()
			}
			p, err := captureExecutionProfilePath(ctx, path)
			if err == nil {
				p.Close()
				t.Fatal("unsafe original accepted")
			}
		})
	}
}

// No application or helper entrypoint executes: this fixture creates this test
// PE suspended and only its original creator handles terminate/drain it. A
// service session is intentionally refused, never counted as interactive proof.
func TestNativeExecutionChildRetainsActualProcessAndInteractiveToken(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	var session, used uint32
	if err := windows.GetTokenInformation(windows.GetCurrentProcessToken(), windows.TokenSessionId, (*byte)(unsafe.Pointer(&session)), 4, &used); err != nil || used != 4 {
		t.Fatal(err)
	}
	original, err := openExecutionProcess(windows.GetCurrentProcessId())
	if session == 0 {
		if err == nil {
			original.close()
			t.Fatal("service session became interactive")
		}
		t.Log("session0 native denial passed; interactive token/process qualification not exercised")
		return
	}
	if err != nil {
		t.Fatal("actual interactive process", err)
	}
	defer original.close()
	if err = original.check(); err != nil {
		t.Fatal(err)
	}
	path, err := os.Executable()
	if err != nil {
		t.Fatal(err)
	}
	image, err := windows.UTF16PtrFromString(path)
	if err != nil {
		t.Fatal(err)
	}
	startup := windows.StartupInfo{Cb: uint32(unsafe.Sizeof(windows.StartupInfo{}))}
	var child windows.ProcessInformation
	if err = windows.CreateProcess(image, nil, nil, nil, false, windows.CREATE_SUSPENDED|windows.CREATE_NO_WINDOW, nil, nil, &startup, &child); err != nil {
		t.Fatal("inert suspended child", err)
	}
	defer func() {
		if err := windows.TerminateProcess(child.Process, 0); err != nil {
			t.Error(err)
		}
		status, err := windows.WaitForSingleObject(child.Process, 10000)
		if err != nil || status != windows.WAIT_OBJECT_0 {
			t.Error("drain inert child", status, err)
		}
		if err = windows.CloseHandle(child.Thread); err != nil {
			t.Error(err)
		}
		if err = windows.CloseHandle(child.Process); err != nil {
			t.Error(err)
		}
	}()
	retained, err := openExecutionProcess(child.ProcessId)
	if err != nil {
		t.Fatal("native child query/token access", err)
	}
	defer retained.close()
	if retained.parent != original.pid || retained.created <= original.created || retained.identity != original.identity {
		t.Fatal("actual child birth/parent/session/logon differs")
	}
	if err = retained.check(); err != nil {
		t.Fatal(err)
	}
	if err = retained.close(); err != nil {
		t.Fatal(err)
	}
	if err = retained.check(); err == nil {
		t.Fatal("closed actual child owner survived")
	}
}
