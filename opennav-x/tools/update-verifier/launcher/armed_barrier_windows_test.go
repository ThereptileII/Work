package launcher

import (
	"errors"
	"fmt"
	"os"
	"path/filepath"
	"strings"
	"testing"

	"golang.org/x/sys/windows"
)

func TestNativeArmedBarrierOnlyExactMissingLeafProvesAbsence(t *testing.T) {
	for _, err := range []error{nil, windows.ERROR_PATH_NOT_FOUND, windows.ERROR_ACCESS_DENIED,
		windows.ERROR_SHARING_VIOLATION, os.ErrNotExist, fmt.Errorf("wrapped leaf: %w", windows.ERROR_FILE_NOT_FOUND)} {
		if !errors.Is(nativeArmedLeafResult(err), errArmedUpdateBarrier) {
			t.Fatalf("present or uncertain leaf accepted as absent: %v", err)
		}
	}
	if err := nativeArmedLeafResult(windows.ERROR_FILE_NOT_FOUND); err != nil {
		t.Fatal("exact missing leaf refused: " + err.Error())
	}
}

func TestNativeArmedBarrierPresentTypesAndMissingParentRefuse(t *testing.T) {
	for _, kind := range []string{"empty", "garbage", "directory", "missing parent"} {
		t.Run(kind, func(t *testing.T) {
			root := t.TempDir()
			p := nativePlatform{}
			if err := p.armedUpdateAbsent(root); err != nil {
				t.Fatal("existing ordinary parent with absent marker refused: " + err.Error())
			}
			name := filepath.Join(root, armedUpdateMarker)
			var err error
			switch kind {
			case "directory":
				err = os.Mkdir(name, 0700)
			case "missing parent":
				root = filepath.Join(root, "absent")
			case "empty":
				err = os.WriteFile(name, nil, 0600)
			default:
				err = os.WriteFile(name, []byte("not trusted or parsed"), 0600)
			}
			if err != nil {
				t.Fatal(err)
			}
			if !errors.Is(p.armedUpdateAbsent(root), errArmedUpdateBarrier) {
				t.Fatal("present type or missing parent was treated as absence")
			}
		})
	}
}

func TestNativeArmedBarrierUsesRequestedManagedAppRoot(t *testing.T) {
	blockedRoot, absentRoot := t.TempDir(), t.TempDir()
	if err := os.WriteFile(filepath.Join(blockedRoot, armedUpdateMarker), nil, 0600); err != nil {
		t.Fatal(err)
	}
	app := func(root string) string {
		return filepath.ToSlash(filepath.Join(root, "generations", strings.Repeat("a", 32), "app"))
	}
	p := nativePlatform{}
	if !errors.Is(p.managedArmedUpdateAbsent(app(blockedRoot)), errArmedUpdateBarrier) {
		t.Fatal("requested managed app's root barrier was ignored")
	}
	if err := p.managedArmedUpdateAbsent(app(absentRoot)); err != nil {
		t.Fatal("another installation's barrier changed the requested root: " + err.Error())
	}
}
