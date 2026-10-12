package brokerhost

import (
	"bytes"
	"errors"
	"io"
	"os"
	"path/filepath"
	"testing"

	"golang.org/x/sys/windows"
)

// Exercise an existing writable mapping of a private disposable file. A new
// deny-write handle is not assumed to revoke views created before observation.
// No application or installed file is opened, mapped, or executed by this test.
func TestNativeStockExistingMappingIntegrity(t *testing.T) {
	name := filepath.Join(t.TempDir(), "stock-integrity.dat")
	original := bytes.Repeat([]byte{0x41}, 4096)
	if err := os.WriteFile(name, original, 0600); err != nil {
		t.Fatal(err)
	}
	p, err := windows.UTF16PtrFromString(name)
	if err != nil {
		t.Fatal(err)
	}
	writer, err := windows.CreateFile(p, windows.GENERIC_READ|windows.GENERIC_WRITE,
		windows.FILE_SHARE_READ|windows.FILE_SHARE_WRITE|windows.FILE_SHARE_DELETE,
		nil, windows.OPEN_EXISTING, windows.FILE_ATTRIBUTE_NORMAL, 0)
	if err != nil {
		t.Fatal(err)
	}
	mapping, err := windows.CreateFileMapping(writer, nil, windows.PAGE_READWRITE, 0, 0, nil)
	if err != nil {
		_ = windows.CloseHandle(writer)
		t.Fatal(err)
	}
	defer func() {
		if err := windows.CloseHandle(mapping); err != nil {
			t.Error(err)
		}
	}()
	view, err := windows.MapViewOfFile(mapping, windows.FILE_MAP_WRITE, 0, 0, uintptr(len(original)))
	if err != nil {
		_ = windows.CloseHandle(writer)
		t.Fatal(err)
	}
	defer func() {
		if err := windows.UnmapViewOfFile(view); err != nil {
			t.Error(err)
		}
	}()
	if err = windows.CloseHandle(writer); err != nil {
		t.Fatal(err)
	}
	h, err := openStock(name, false)
	if errors.Is(err, windows.ERROR_SHARING_VIOLATION) {
		t.Log("existing writable mapping prevents initial stock custody")
		return
	}
	if err != nil {
		t.Fatal(err)
	}
	owner := &nativeStock{file: os.NewFile(uintptr(h), "stock-integrity-fixture"), path: name}
	defer func() {
		if err := owner.Close(); err != nil {
			t.Error(err)
		}
	}()
	if err = windows.GetFileInformationByHandle(h, &owner.info); err != nil {
		t.Fatal(err)
	}
	owner.physical, err = stockPhysical(h)
	if err != nil {
		t.Fatal(err)
	}
	if err = owner.validate(); err != nil {
		t.Fatalf("unchanged fixture refused: %v", err)
	}
	probe, err := windows.CreateFile(p, windows.GENERIC_WRITE,
		windows.FILE_SHARE_READ|windows.FILE_SHARE_WRITE|windows.FILE_SHARE_DELETE,
		nil, windows.OPEN_EXISTING, windows.FILE_ATTRIBUTE_NORMAL, 0)
	if err == nil {
		_ = windows.CloseHandle(probe)
		t.Fatal("retained stock unexpectedly permits a new writer")
	}
	if !errors.Is(err, windows.ERROR_SHARING_VIOLATION) {
		t.Fatalf("new writer refusal not corroborated: %v", err)
	}
	if err = os.Remove(name); !errors.Is(err, windows.ERROR_SHARING_VIOLATION) {
		t.Fatalf("held stock delete: %v", err)
	}
	// Mutate the same retained writable mapping through the native address API,
	// without converting a stored uintptr into a Go pointer. No process is opened:
	// the pseudo-handle addresses only this fixture's own mapped memory.
	replacement := byte(0x42)
	var written uintptr
	if err = windows.WriteProcessMemory(windows.CurrentProcess(), view, &replacement, 1, &written); err != nil || written != 1 {
		t.Fatalf("retained mapped byte mutation failed: bytes=%d err=%v", written, err)
	}
	if err = windows.FlushViewOfFile(view, uintptr(len(original))); err != nil {
		t.Fatal(err)
	}
	actual := make([]byte, len(original))
	if n, err := owner.file.ReadAt(actual, 0); err != nil && !errors.Is(err, io.EOF) || n != len(actual) || actual[0] != 0x42 {
		t.Fatalf("mapped byte change was not observed through held stock: bytes=%d err=%v", n, err)
	}
	var info windows.ByHandleFileInformation
	if err = windows.GetFileInformationByHandle(h, &info); err != nil {
		t.Fatal(err)
	}
	t.Logf("mapped change keeps identity=%v size=%v timestamp=%v", sameStock(owner.info, info),
		owner.info.FileSizeHigh == info.FileSizeHigh && owner.info.FileSizeLow == info.FileSizeLow,
		owner.info.LastWriteTime == info.LastWriteTime)
	if err := owner.validate(); err == nil {
		t.Fatal("changed mapped stock bytes validated as unchanged")
	}
}

// This exercises the actual stock directory-open helper on disposable paths;
// it does not qualify service provisioning or complete stock namespace custody.
func TestNativeStockDirectoryRenamePin(t *testing.T) {
	path := filepath.Join(t.TempDir(), "stock-parent")
	if err := os.Mkdir(path, 0700); err != nil {
		t.Fatal(err)
	}
	h, err := openStock(path, true)
	if err != nil {
		t.Fatal(err)
	}
	defer func() {
		if h != windows.InvalidHandle {
			if err := windows.CloseHandle(h); err != nil {
				t.Error(err)
			}
		}
	}()
	moved := path + "-moved"
	if err := os.Rename(path, moved); !errors.Is(err, windows.ERROR_SHARING_VIOLATION) {
		t.Fatalf("held stock directory rename: got %v, want sharing violation", err)
	}
	if err := windows.CloseHandle(h); err != nil {
		t.Fatal(err)
	}
	h = windows.InvalidHandle
	if err := os.Rename(path, moved); err != nil {
		t.Fatalf("stock directory rename after close: %v", err)
	}
}

// Actual os.File cleanup failure is retained and never reported successful on a
// second Close. This fixture uses an already closed local temporary file only.
func TestNativeStockCloseFailureIsRetained(t *testing.T) {
	f, err := os.CreateTemp(t.TempDir(), "stock-owner-")
	if err != nil {
		t.Fatal(err)
	}
	if err = f.Close(); err != nil {
		t.Fatal(err)
	}
	owner := &nativeStock{file: f}
	first := owner.Close()
	if !errors.Is(first, ErrStockCleanup) || !errors.Is(first, os.ErrClosed) {
		t.Fatalf("cleanup failure lost: %v", first)
	}
	if second := owner.Close(); second != first {
		t.Fatalf("cleanup retried or lost: first=%v second=%v", first, second)
	}
	if err := owner.validate(); err == nil {
		t.Fatal("closed uncertain owner validated")
	}
}
