package brokerhost

import (
	"errors"
	"os"
	"path/filepath"
	"testing"

	"golang.org/x/sys/windows"
)

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
