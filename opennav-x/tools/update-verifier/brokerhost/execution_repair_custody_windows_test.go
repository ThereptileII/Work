//go:build windows

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

	"golang.org/x/sys/windows"
)

func repairCustodyNativeFixture(t *testing.T) (string, *nativeProfileCapture) {
	t.Helper()
	root := t.TempDir()
	for name, raw := range map[string][]byte{"opencpn.ini": []byte("[Connections]\r\nuntouched=1\r\n"), "navobj.db": []byte("inert current navigation bytes")} {
		if err := os.WriteFile(filepath.Join(root, name), raw, 0600); err != nil {
			t.Fatal(err)
		}
	}
	source, err := captureProfileTreePath(context.Background(), root, true)
	if err != nil {
		t.Fatal(err)
	}
	native := source.(*nativeProfileCapture)
	t.Cleanup(func() {
		if err := native.close(); err != nil {
			t.Error(err)
		}
	})
	return root, native
}

// Real NTFS deny-writer/delete handles and recursive notifications, private
// temporary data only. The private policy seam bypasses ONLY fixed ProgramData
// routing; the production factory refuses this temporary path. No app, SCM,
// registration, updater store, trust or real navigation profile is touched.
func TestNativeRepairColdCustodyRetainsWholeTreeUntilOuterOwnerClose(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	root, source := repairCustodyNativeFixture(t)
	ctx, e := context.Background(), &engine{}
	if _, err := captureRepairApplicationCustodyLocked(ctx, e, applicationOwnerTestSID, strings.Repeat("a", 32), source); err == nil {
		t.Fatal("production factory accepted caller-selected temporary profile")
	}
	c, err := captureRepairApplicationCustodyWith(ctx, e, applicationOwnerTestSID, strings.Repeat("a", 32), &nativeRepairFrozenProfile{source})
	if err != nil {
		t.Fatal(err)
	}
	file := filepath.Join(root, "navobj.db")
	before, err := os.ReadFile(file)
	if err != nil {
		t.Fatal(err)
	}
	assertFrozen := func() {
		t.Helper()
		if err := os.WriteFile(file, []byte("must not overwrite current voyage"), 0600); !errors.Is(err, windows.ERROR_SHARING_VIOLATION) {
			t.Fatal("current file admitted writer", err)
		}
		if err := os.Remove(file); !errors.Is(err, windows.ERROR_SHARING_VIOLATION) {
			t.Fatal("current file admitted deletion", err)
		}
		if err := os.Rename(root, root+"-replaced"); !errors.Is(err, windows.ERROR_SHARING_VIOLATION) {
			t.Fatal("retained profile root admitted replacement", err)
		}
	}
	assertFrozen()
	if err = c.validate(ctx); err != nil {
		t.Fatal(err)
	}
	if err = c.finish(ctx); err != nil || len(e.applications.entries) != 0 {
		t.Fatal("cold finish invented or required an old process", err)
	}
	if err = c.close(); err != nil {
		t.Fatal(err)
	}
	assertFrozen() // Revoking custody never releases the borrowed full-tree owner.
	if actual, err := os.ReadFile(file); err != nil || !bytes.Equal(actual, before) {
		t.Fatal("custody changed current data", err)
	}
	if err = source.close(); err != nil {
		t.Fatal(err)
	}
	if err = source.close(); err != nil {
		t.Fatal("outer owner did not drain idempotently", err)
	}
	if err = os.WriteFile(file, []byte("inert post-close fixture write"), 0600); err != nil {
		t.Fatal("borrowed file handle leaked", err)
	}
}

func TestNativeRepairColdCustodyRefusesPreexistingWriteOrDeleteHandle(t *testing.T) {
	for _, item := range []struct {
		name   string
		rights uint32
	}{{"write", windows.GENERIC_WRITE}, {"delete", windows.DELETE}} {
		t.Run(item.name, func(t *testing.T) {
			runtime.LockOSThread()
			defer runtime.UnlockOSThread()
			root := t.TempDir()
			name := filepath.Join(root, "opencpn.ini")
			original := []byte("inert original current profile")
			if err := os.WriteFile(name, original, 0600); err != nil {
				t.Fatal(err)
			}
			ptr, err := windows.UTF16PtrFromString(name)
			if err != nil {
				t.Fatal(err)
			}
			handle, err := windows.CreateFile(ptr, item.rights, windows.FILE_SHARE_READ|windows.FILE_SHARE_WRITE|windows.FILE_SHARE_DELETE, nil, windows.OPEN_EXISTING, 0, 0)
			if err != nil {
				t.Fatal(err)
			}
			_, captureErr := captureProfileTreePath(context.Background(), root, true)
			if err = windows.CloseHandle(handle); err != nil {
				t.Fatal(err)
			}
			if captureErr == nil {
				t.Fatal("preexisting kernel writer/delete owner admitted")
			}
			// Failed acquisition drains its partial retained native owners.
			if err = os.WriteFile(name, original, 0600); err != nil {
				t.Fatal("failed factory leaked profile handle", err)
			}
		})
	}
}

func TestNativeRepairColdCustodyDetectsNamespaceChangeAndNeverErasesIt(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	root, source := repairCustodyNativeFixture(t)
	c, err := captureRepairApplicationCustodyWith(context.Background(), &engine{}, applicationOwnerTestSID, strings.Repeat("a", 32), &nativeRepairFrozenProfile{source})
	if err != nil {
		t.Fatal(err)
	}
	// Directory handles permit adds. The actual recursive watch rejects this
	// namespace change; nothing pretends that it was prevented or rolls it back.
	name, data := filepath.Join(root, "new-customer-navigation.xml"), []byte("new data must survive refusal")
	if err = os.WriteFile(name, data, 0600); err != nil {
		t.Fatal(err)
	}
	if err = c.validate(context.Background()); err == nil {
		t.Fatal("changed namespace retained unchanged-profile custody")
	}
	if err = c.finish(context.Background()); err == nil || c.close() == nil {
		t.Fatal("failed original watch regained authority")
	}
	if actual, err := os.ReadFile(name); err != nil || !bytes.Equal(actual, data) {
		t.Fatal("refusal erased newer customer data", err)
	}
}

func TestNativeRepairColdCustodyRefusesReleasedUnwatchedOrProcessUncheckedSource(t *testing.T) {
	for _, fault := range []string{"closed", "unfrozen", "unwatched", "no-process-check"} {
		t.Run(fault, func(t *testing.T) {
			runtime.LockOSThread()
			defer runtime.UnlockOSThread()
			_, source := repairCustodyNativeFixture(t)
			profile := &nativeRepairFrozenProfile{source}
			c, err := captureRepairApplicationCustodyWith(context.Background(), &engine{}, applicationOwnerTestSID, strings.Repeat("a", 32), profile)
			if err != nil {
				t.Fatal(err)
			}
			switch fault {
			case "closed":
				if err = source.close(); err != nil {
					t.Fatal(err)
				}
			case "unfrozen":
				source.frozen = false
			case "unwatched":
				if err = source.watch.close(false); err != nil {
					t.Fatal(err)
				}
			case "no-process-check":
				source.checkProcesses = false
			}
			if err = c.validate(context.Background()); err == nil {
				t.Fatal("native no-writer custody was weakened", fault)
			}
			if err = c.finish(context.Background()); err == nil {
				t.Fatal("released/uncertain source authorized finish")
			}
		})
	}
}
