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
