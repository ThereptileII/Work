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

func nativeArmedOtherSubstTarget(t *testing.T) string {
	t.Helper()
	other := t.TempDir()
	if err := os.Mkdir(filepath.Join(other, "installation"), 0700); err != nil {
		t.Fatal(err)
	}
	directory, err := os.Open(other)
	if err != nil {
		t.Fatal(err)
	}
	name, err := nativePhysicalName(windows.Handle(directory.Fd()))
	directory.Close()
	if err != nil {
		t.Fatal(err)
	}
	return name
}

func TestNativeArmedBarrierCapturedDriveSurvivesSubstRemap(t *testing.T) {
	requireNativeOrdinarySubst(t)
	aliasRoot := t.TempDir()
	root := filepath.Join(aliasRoot, "installation")
	if err := os.Mkdir(root, 0700); err != nil {
		t.Fatal(err)
	}
	if err := os.WriteFile(filepath.Join(root, armedUpdateMarker), nil, 0600); err != nil {
		t.Fatal(err)
	}
	device, remap := nativeOrdinarySubstMapping(t, aliasRoot)
	drive, err := openNativeArmedDirectory(device + `\`)
	if err != nil {
		t.Fatal(err)
	}
	defer windows.CloseHandle(drive.handle)
	remap(nativeArmedOtherSubstTarget(t))
	requestedRoot := device + "/installation"
	if err := (nativePlatform{}).armedUpdateAbsent(requestedRoot); err != nil {
		t.Fatal("remapped B root must be present and unarmed: " + err.Error())
	}
	if !errors.Is(nativeArmedCapturedDriveAbsent(drive, requestedRoot), errArmedUpdateBarrier) {
		t.Fatal("intra-call DOS remap hid the captured A root's marker")
	}
}

func TestNativeArmedBarrierHeldImageRefusesAfterSubstRemap(t *testing.T) {
	requireNativeOrdinarySubst(t)
	for _, at := range []int{1, 2} {
		t.Run(fmt.Sprint(at), func(t *testing.T) {
			spec, receipt := ordinaryNativeFixture(t)
			aliasRoot := filepath.Dir(spec.directory)
			installation := filepath.Join(aliasRoot, "installation")
			generation := strings.Repeat("a", 32)
			directory := filepath.Join(installation, "generations", generation, "app")
			if err := os.MkdirAll(directory, 0700); err != nil {
				t.Fatal(err)
			}
			bytes, err := os.ReadFile(spec.executable)
			if err != nil {
				t.Fatal(err)
			}
			if err := os.WriteFile(filepath.Join(directory, "opencpn.exe"), bytes, 0600); err != nil {
				t.Fatal(err)
			}
			device, remap := nativeOrdinarySubstMapping(t, aliasRoot)
			spec.directory = device + `\installation\generations\` + generation + `\app`
			spec.executable = spec.directory + `\opencpn.exe`
			bound, err := (nativePlatform{}).acquireUnarmedOrdinaryImage(spec)
			if err != nil {
				t.Fatal(err)
			}
			defer bound.Close()
			owner := bound.(unarmedOrdinaryImage).ordinaryImage.(*nativeOrdinaryImage)
			if strings.EqualFold(owner.application[:2], device) {
				t.Fatal("test requires independently derived application spelling, not request alias")
			}
			otherTarget := nativeArmedOtherSubstTarget(t)
			var child ordinaryChild
			defer func() {
				if child != nil {
					_ = child.abortSuspended()
					_ = child.Close()
				}
			}()
			if at == 2 {
				if err := bound.recheck(); err != nil {
					t.Fatal(err)
				}
				child, err = bound.create(spec)
				if err != nil {
					t.Fatal(err)
				}
				if err := child.corroborate(); err != nil {
					t.Fatal(err)
				}
			}
			if err := os.WriteFile(filepath.Join(installation, armedUpdateMarker), nil, 0600); err != nil {
				t.Fatal(err)
			}
			remap(otherTarget)
			if err := (nativePlatform{}).armedUpdateAbsent(device + "/installation"); err != nil {
				t.Fatal("changed alias must select unarmed B: " + err.Error())
			}
			if err := owner.recheck(); err != nil {
				t.Fatal("request-alias remap must preserve held A image custody: " + err.Error())
			}
			if err := bound.recheck(); !errors.Is(err, errArmedUpdateBarrier) {
				t.Fatalf("armed held A image was not denied after alias remap: %v", err)
			}
			if child != nil {
				if err := child.abortSuspended(); err != nil {
					t.Fatal(err)
				}
				if err := child.Close(); err != nil {
					t.Fatal(err)
				}
				child = nil
			}
			if _, err := os.Stat(receipt); !errors.Is(err, os.ErrNotExist) {
				t.Fatal("armed held A image executed")
			}
		})
	}
}

// An empty directory avoids a child image handle masking an ineffective pin.
func TestNativeArmedDirectoryPinBlocksRenameUntilClosed(t *testing.T) {
	name := filepath.Join(t.TempDir(), "empty")
	if err := os.Mkdir(name, 0700); err != nil {
		t.Fatal(err)
	}
	renamed := name + "-renamed"
	if err := os.Rename(name, renamed); err != nil {
		t.Fatal("baseline rename:", err)
	}
	if err := os.Rename(renamed, name); err != nil {
		t.Fatal("baseline restore:", err)
	}
	pin, err := openNativeArmedDirectory(name)
	if err != nil {
		t.Fatal(err)
	}
	defer func() {
		if pin.handle != windows.InvalidHandle {
			if err := windows.CloseHandle(pin.handle); err != nil {
				t.Error("close directory pin:", err)
			}
		}
	}()
	if err := os.Rename(name, renamed); !errors.Is(err, windows.ERROR_SHARING_VIOLATION) {
		t.Fatalf("empty directory rename while pinned must report sharing violation, got %v", err)
	}
	if err := windows.CloseHandle(pin.handle); err != nil {
		t.Fatal(err)
	}
	pin.handle = windows.InvalidHandle
	if err := os.Rename(name, renamed); err != nil {
		t.Fatal("rename after close:", err)
	}
}
