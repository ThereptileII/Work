//go:build windows

package brokerenroll

import (
	"bytes"
	"context"
	"errors"
	"os"
	"path/filepath"
	"runtime"
	"strings"
	"syscall"
	"testing"
	"unsafe"

	"golang.org/x/sys/windows"
)

// Inert native fixtures only. No SCM, fixed installed tree or real user menu is
// changed, no product is executed, and success is not installed-flow acceptance.
func TestNativeStartupShellLinkPersistsFixedProtectedArguments(t *testing.T) {
	target := filepath.Join(t.TempDir(), "skager-start.exe")
	if err := os.WriteFile(target, []byte("INERT-NOT-EXECUTABLE"), 0600); err != nil {
		t.Fatal(err)
	}
	arguments := "--broker-start=" + strings.Repeat("b", 32)
	raw, err := startupLinkBytes(target, arguments, filepath.Dir(target))
	if err != nil {
		t.Fatal(err)
	}
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if err := windows.CoInitializeEx(0, 2); err != nil && !errors.Is(err, syscall.Errno(1)) {
		t.Fatal(err)
	}
	defer startupUninitialize.Call()
	clsid, _ := windows.GUIDFromString("{00021401-0000-0000-C000-000000000046}")
	classIID, _ := windows.GUIDFromString("{00000001-0000-0000-C000-000000000046}")
	linkIID, _ := windows.GUIDFromString("{000214F9-0000-0000-C000-000000000046}")
	persistIID, _ := windows.GUIDFromString("{00000109-0000-0000-C000-000000000046}")
	var factory, link, persist, stream *startupCOM
	hr, _, _ := startupShellFactory.Call(uintptr(unsafe.Pointer(&clsid)), uintptr(unsafe.Pointer(&classIID)), uintptr(unsafe.Pointer(&factory)))
	if int32(hr) < 0 || factory == nil {
		t.Fatalf("actual system ShellLink factory HRESULT %#x", uint32(hr))
	}
	defer startupCOMRelease(factory)
	if err := startupCOMCall(factory, 3, 0, uintptr(unsafe.Pointer(&linkIID)), uintptr(unsafe.Pointer(&link))); err != nil {
		t.Fatal(err)
	}
	defer startupCOMRelease(link)
	if err := startupCOMCall(link, 0, uintptr(unsafe.Pointer(&persistIID)), uintptr(unsafe.Pointer(&persist))); err != nil {
		t.Fatal(err)
	}
	defer startupCOMRelease(persist)
	hr, _, _ = startupMemoryStream.Call(0, 1, uintptr(unsafe.Pointer(&stream)))
	if int32(hr) < 0 || stream == nil {
		t.Fatal("actual native stream unavailable")
	}
	defer startupCOMRelease(stream)
	var used uint32
	if err := startupCOMCall(stream, 4, uintptr(unsafe.Pointer(&raw[0])), uintptr(len(raw)), uintptr(unsafe.Pointer(&used))); err != nil || used != uint32(len(raw)) {
		t.Fatalf("actual stream write %d %v", used, err)
	}
	if err := startupStreamRewind(stream); err != nil {
		t.Fatal(err)
	}
	if err := startupCOMCall(persist, 5, uintptr(unsafe.Pointer(stream))); err != nil {
		t.Fatal(err)
	}
	for _, field := range []struct {
		name   string
		method int
		want   string
	}{{"target", 3, target}, {"arguments", 10, arguments}, {"working-directory", 8, filepath.Dir(target)}} {
		// COM methods run on this retained original apartment thread, before t.Run
		// starts an assertion-only goroutine.
		buffer := make([]uint16, 32768)
		args := []uintptr{uintptr(unsafe.Pointer(&buffer[0])), uintptr(len(buffer))}
		if field.method == 3 {
			args = append(args, 0, 4)
		}
		err := startupCOMCall(link, field.method, args...)
		got := windows.UTF16ToString(buffer)
		t.Run(field.name, func(t *testing.T) {
			if err != nil {
				t.Fatal(err)
			}
			if got != field.want {
				t.Fatalf("actual persisted %s differs got=%q want=%q", field.name, got, field.want)
			}
		})
	}

}

func startupNativeFixture(t *testing.T) (*nativeStartupPublication, *enrollmentPin, *enrollmentPin, *windows.SECURITY_DESCRIPTOR) {
	t.Helper()
	if !windows.GetCurrentProcessToken().IsElevated() {
		t.Skip("inert fresh protected-directory fixture requires elevation; not a pass")
	}
	user, err := windows.GetCurrentProcessToken().GetTokenUser()
	if err != nil {
		t.Fatal(err)
	}
	sid := user.User.Sid.String()
	files := &nativeEnrollmentFiles{}
	t.Cleanup(func() {
		if err := files.Close(); err != nil {
			t.Errorf("native owners close: %v", err)
		}
	})
	path := t.TempDir()
	parent, err := files.pinDirectory(path, "", path, nil, false, sid)
	if err != nil {
		t.Fatal(err)
	}
	sd, err := windows.SecurityDescriptorFromString("O:BAD:P(A;OICI;FA;;;SY)(A;OICI;FA;;;BA)(A;OICI;FRFX;;;" + sid + ")")
	if err != nil {
		t.Fatal(err)
	}
	menu, err := files.newDirectory(parent, "menu", sd, sid)
	if err != nil {
		t.Fatal(err)
	}
	recovery, err := files.newDirectory(parent, "recovery", sd, sid)
	if err != nil {
		t.Fatal(err)
	}
	return &nativeStartupPublication{files: files, clientSID: sid}, menu, recovery, sd
}
func TestNativeStartupPublicationPreservesOriginalIdentityAndOtherLinks(t *testing.T) {
	owner, menu, recovery, sd := startupNativeFixture(t)
	old := []byte("ORIGINAL-NORMAL-LINK-EXACT-BYTES")
	next := []byte("NEW-INERT-LINK-EXACT-BYTES")
	var err error
	owner.original, err = startupWriteFile(context.Background(), owner.files, menu, startupShortcutName, old, sd, true)
	if err != nil {
		t.Fatal(err)
	}
	owner.originalSHA = digest(old)
	originalID := owner.original.id
	legacy, err := startupWriteFile(context.Background(), owner.files, menu, "OpenCPN Legacy.lnk", []byte("LEGACY-UNCHANGED"), sd, false)
	if err != nil {
		t.Fatal(err)
	}
	safe, err := startupWriteFile(context.Background(), owner.files, menu, "Skager Safe Mode.lnk", []byte("SAFE-UNCHANGED"), sd, false)
	if err != nil {
		t.Fatal(err)
	}
	legacyID, safeID := legacy.id, safe.id
	if _, err := startupWriteFile(context.Background(), owner.files, recovery, "original.lnk.bytes", old, sd, false); err != nil {
		t.Fatal(err)
	}
	owner.link, err = startupWriteFile(context.Background(), owner.files, recovery, "pending.lnk", next, sd, true)
	if err != nil {
		t.Fatal(err)
	}
	owner.linkSHA = digest(next)
	if err := startupPublishRenames(context.Background(), owner, menu, recovery); err != nil {
		t.Fatal(err)
	}
	for _, check := range []struct {
		name string
		pin  *enrollmentPin
		sha  string
	}{{"new-normal", owner.link, digest(next)}, {"original-backup", owner.original, digest(old)}, {"legacy", legacy, digest([]byte("LEGACY-UNCHANGED"))}, {"safe", safe, digest([]byte("SAFE-UNCHANGED"))}} {
		t.Run(check.name, func(t *testing.T) {
			if err := startupHashPin(context.Background(), check.pin, check.sha); err != nil {
				t.Fatal(err)
			}
		})
	}
	if owner.original.id != originalID || legacy.id != legacyID || safe.id != safeID {
		t.Fatal("actual original physical IDs changed")
	}
	if _, err := os.Stat(filepath.Join(recovery.dos, "pending.lnk")); !os.IsNotExist(err) {
		t.Fatal("staging link still exists", err)
	}
	if _, err := owner.files.newDirectory(owner.files.pins[0], "recovery", sd, owner.clientSID); err == nil {
		t.Fatal("existing recovery adopted")
	}
}
func TestNativeStartupPublicationRefusesUnknownDestinationAndCancellation(t *testing.T) {
	for _, mode := range []string{"unknown-destination", "canceled"} {
		t.Run(mode, func(t *testing.T) {
			owner, menu, recovery, sd := startupNativeFixture(t)
			old := []byte("UNKNOWN-ORIGINAL-ENTRY")
			next := []byte("NEW-INERT-LINK")
			var err error
			owner.link, err = startupWriteFile(context.Background(), owner.files, recovery, "pending.lnk", next, sd, true)
			if err != nil {
				t.Fatal(err)
			}
			existing, err := startupWriteFile(context.Background(), owner.files, menu, startupShortcutName, old, sd, true)
			if err != nil {
				t.Fatal(err)
			}
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			if mode == "canceled" {
				owner.original = existing
				owner.originalSHA = digest(old)
				cancel()
			}
			if err := startupPublishRenames(ctx, owner, menu, recovery); err == nil {
				t.Fatal("unexpected publication success")
			}
			raw, err := os.ReadFile(filepath.Join(menu.dos, startupShortcutName))
			if err != nil || !bytes.Equal(raw, old) {
				t.Fatalf("original entry changed %q %v", raw, err)
			}
			if err := startupHashPin(context.Background(), owner.link, digest(next)); err != nil {
				t.Fatal("pending bytes not retained", err)
			}
		})
	}
}
