//go:build windows

package main

import (
	"context"
	"errors"
	"path/filepath"
	"testing"
	"time"
	"unsafe"

	"golang.org/x/sys/windows"
)

func installedCodeGuardFixture(t *testing.T, allowPrevious bool) *installedElevationCodePin {
	t.Helper()
	path := filepath.Join(t.TempDir(), installedElevationImage)
	ptr, err := windows.UTF16PtrFromString(path)
	if err != nil {
		t.Fatal(err)
	}
	sd, err := windows.SecurityDescriptorFromString("O:BAG:BAD:P(A;;FA;;;SY)(A;;FA;;;BA)")
	if err != nil {
		t.Fatal(err)
	}
	sa := windows.SecurityAttributes{Length: uint32(unsafe.Sizeof(windows.SecurityAttributes{})), SecurityDescriptor: sd}
	writer, err := windows.CreateFile(ptr, windows.GENERIC_WRITE|windows.READ_CONTROL, 0, &sa, windows.CREATE_NEW, windows.FILE_ATTRIBUTE_NORMAL, 0)
	if err != nil {
		t.Fatal("protected inert image fixture", err)
	}
	var written uint32
	err = windows.WriteFile(writer, []byte("inert original helper bytes"), &written, nil)
	err = errors.Join(err, windows.CloseHandle(writer))
	if err != nil {
		t.Fatal(err)
	}
	h, err := windows.CreateFile(ptr, windows.GENERIC_READ|windows.READ_CONTROL, windows.FILE_SHARE_READ|windows.FILE_SHARE_DELETE, nil, windows.OPEN_EXISTING, windows.FILE_FLAG_OPEN_REPARSE_POINT, 0)
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() {
		if err := windows.CloseHandle(h); err != nil {
			t.Error(err)
		}
	})
	p := &installedElevationCodePin{handle: h, path: path}
	info, id, physical, actual, err := p.observe()
	if err != nil {
		t.Fatal(err)
	}
	p.info, p.id, p.physical, p.descriptor = info, id, physical, actual.String()
	if allowPrevious {
		p.previousPhysical = filepath.Join(filepath.Dir(physical), "helper.previous.exe")
		p.imageSHA256, err = installedElevationImageDigest(h, int64(info.FileSizeLow))
		if err != nil {
			t.Fatal(err)
		}
	}
	if err := p.check(); err != nil {
		t.Fatal(err)
	}
	return p
}

func TestNativeInstalledCodeOriginalGuardOnlyRetainedFixedPreviousAlias(t *testing.T) {
	p := installedCodeGuardFixture(t, true)
	initial := p.id
	ptr, _ := windows.UTF16PtrFromString(p.path)
	writer, err := windows.CreateFile(ptr, windows.GENERIC_WRITE, windows.FILE_SHARE_READ|windows.FILE_SHARE_WRITE|windows.FILE_SHARE_DELETE, nil, windows.OPEN_EXISTING, 0, 0)
	if err == nil {
		_ = windows.CloseHandle(writer)
		t.Fatal("original image admitted a writer")
	}
	if !errors.Is(err, windows.ERROR_SHARING_VIOLATION) {
		t.Fatal("unexpected deny-write result", err)
	}
	previous := filepath.Join(filepath.Dir(p.path), "helper.previous.exe")
	previousPtr, _ := windows.UTF16PtrFromString(previous)
	if err := windows.MoveFileEx(ptr, previousPtr, 0); err != nil {
		t.Fatal(err)
	}
	if err := p.check(); err != nil || p.id != initial {
		t.Fatal("same-original-ID previous move refused", err)
	}
	// A same-name replacement is never adopted as the original object.
	p.id.id[0] ^= 1
	if err := p.check(); err == nil {
		t.Fatal("different full file ID accepted")
	}
	p.id = initial
	foreign, _ := windows.UTF16PtrFromString(filepath.Join(filepath.Dir(p.path), "foreign.exe"))
	if err := windows.MoveFileEx(previousPtr, foreign, 0); err != nil {
		t.Fatal(err)
	}
	if err := p.check(); err == nil {
		t.Fatal("arbitrary same-object alias accepted")
	}
}

func TestNativeInstalledCodeOtherMaintenanceRemainsFixedOnly(t *testing.T) {
	p := installedCodeGuardFixture(t, false)
	source, _ := windows.UTF16PtrFromString(p.path)
	target, _ := windows.UTF16PtrFromString(filepath.Join(filepath.Dir(p.path), "helper.previous.exe"))
	if err := windows.MoveFileEx(source, target, 0); err != nil {
		t.Fatal(err)
	}
	if err := p.check(); err == nil {
		t.Fatal("ordinary maintenance accepted previous image")
	}
}

func TestNativeInstalledCodeElevatedParentBindingAndOpaqueLaunch(t *testing.T) {
	parent := installedElevationIdentity{pid: 16, created: 123450, sid: "S-1-5-21-1-2-3-1001", session: 2, elevated: true}
	self := parent
	self.pid++
	self.created++
	q := installedElevationRequest{action: codeUpgradeCurrent, child: true, pid: parent.pid, created: parent.created}
	if err := installedCodeParentBinding(q, self, parent); err != nil {
		t.Fatal(err)
	}
	for _, fault := range []string{"SID", "session", "birth", "same process", "unelevated parent", "unelevated child", "old parent", "wrong action"} {
		s, p, request := self, parent, q
		switch fault {
		case "SID":
			s.sid += "0"
		case "session":
			s.session++
		case "birth":
			p.created++
		case "same process":
			s.pid = p.pid
		case "unelevated parent":
			p.elevated = false
		case "unelevated child":
			s.elevated = false
		case "old parent":
			s.created += uint64((installedCodeUpgradeLimit + time.Second) / (100 * time.Nanosecond))
		case "wrong action":
			request.action = uninstallCurrent
		}
		if err := installedCodeParentBinding(request, s, p); err == nil {
			t.Fatal("wrong elevated parent accepted", fault)
		}
	}
	// A copied snapshot/fake owner supplies no launch authority and cannot reach
	// fixed paths, UAC, CreateProcess or any installed mutation.
	if child, err := launchInstalledCodeSuccessor(context.Background(), &codeFixtureLaunch{}, installedCodeSnapshot{path: "C:\\arbitrary.exe"}); child != nil || err == nil {
		t.Fatal("opaque admission bypass", child, err)
	}
}
