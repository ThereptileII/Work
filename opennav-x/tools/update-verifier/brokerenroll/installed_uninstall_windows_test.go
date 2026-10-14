//go:build windows

package brokerenroll

import (
	"bytes"
	"context"
	"errors"
	"os"
	"path/filepath"
	"testing"

	"golang.org/x/sys/windows"
)

// Inert disposable files only. This private fixture bypasses actual installed
// stock/baseline/SCM ownership and grants no installed-product acceptance.
func installedMemberNativeFixture(t *testing.T, name string, body []byte) (startupRestorationFile, *enrollmentPin) {
	t.Helper()
	user, err := windows.GetCurrentProcessToken().GetTokenUser()
	if err != nil {
		t.Fatal(err)
	}
	sid := user.User.Sid.String()
	directory := t.TempDir()
	if err := os.WriteFile(filepath.Join(directory, name), body, 0600); err != nil {
		t.Fatal(err)
	}
	files := &nativeEnrollmentFiles{}
	t.Cleanup(func() {
		if err := files.Close(); err != nil {
			t.Error(err)
		}
	})
	parent, err := files.pinDirectory(directory, "", directory, nil, false, sid)
	if err != nil {
		t.Fatal(err)
	}
	file, err := openInstalledMember(parent, name, maxClientBaselineBytes, sid)
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() {
		if file.pin.handle != 0 {
			if err := windows.CloseHandle(file.pin.handle); err != nil {
				t.Error(err)
			}
			file.pin.handle = 0
		}
	})
	return file, parent
}

func TestInstalledUninstallDecommissionNativeOriginalDeletionDisposition(t *testing.T) {
	body := []byte("INERT approved member; never executable")
	file, parent := installedMemberNativeFixture(t, "owned.bin", body)
	unknown := filepath.Join(parent.dos, "unknown.custom")
	modified := filepath.Join(parent.dos, "modified.bin")
	if err := os.WriteFile(unknown, []byte("unknown preserved"), 0600); err != nil {
		t.Fatal(err)
	}
	if err := os.WriteFile(modified, []byte("modified preserved"), 0600); err != nil {
		t.Fatal(err)
	}
	identity := file.pin.id
	actual, err := installedMemberDigest(context.Background(), file.pin)
	if err != nil || actual != digest(body) {
		t.Fatal("retained original read differed", err)
	}
	member := &nativeInstalledMember{file: file, remove: true}
	if err := member.dispose(context.Background()); err != nil {
		t.Fatal("original native deletion disposition failed", err)
	}
	if !member.pending || member.file.pin.id != identity || member.check() != nil {
		t.Fatal("delete-pending original identity not retained")
	}
	if err := member.dispose(context.Background()); err == nil {
		t.Fatal("deletion disposition replayed")
	}
	if err := windows.CloseHandle(file.pin.handle); err != nil {
		t.Fatal(err)
	}
	file.pin.handle = 0
	if _, err := os.Lstat(file.dos); !os.IsNotExist(err) {
		t.Fatal("original known member remained after handle drain", err)
	}
	for path, want := range map[string]string{unknown: "unknown preserved", modified: "modified preserved"} {
		raw, err := os.ReadFile(path)
		if err != nil || string(raw) != want {
			t.Fatal("unknown/modified member changed", path, err)
		}
	}
}

func TestInstalledUninstallDecommissionNativeCancellationAndReplacementRefusal(t *testing.T) {
	body := []byte("INERT original member")
	file, _ := installedMemberNativeFixture(t, "owned.bin", body)
	if err := os.WriteFile(file.dos, []byte("replacement"), 0600); err == nil {
		t.Fatal("original read/delete custody allowed concurrent write")
	}
	if err := os.Rename(file.dos, file.dos+".replaced"); err == nil {
		t.Fatal("original custody allowed external rename")
	}
	ctx, cancel := context.WithCancel(context.Background())
	cancel()
	member := &nativeInstalledMember{file: file, remove: true}
	if err := member.dispose(ctx); !errors.Is(err, context.Canceled) || member.pending {
		t.Fatal("canceled action set deletion disposition", err)
	}
	actual, err := installedMemberDigest(context.Background(), file.pin)
	if err != nil || actual != digest(body) {
		t.Fatal("refusal changed original bytes", err)
	}
}

func TestInstalledUninstallRetirementNativeStateMovesOriginalAndPreservesCollision(t *testing.T) {
	for _, collision := range []bool{false, true} {
		t.Run(map[bool]string{false: "original-move", true: "collision"}[collision], func(t *testing.T) {
			body := []byte(`{"inert":"original retained state"}`)
			file, parent := installedMemberNativeFixture(t, "state.json", body)
			destination := filepath.Join(parent.dos, "retired-state.json")
			if collision {
				if err := os.WriteFile(destination, []byte("foreign recovery"), 0600); err != nil {
					t.Fatal(err)
				}
			}
			original := file.pin.id
			err := startupRenameRestorationFile(context.Background(), &file, parent, "retired-state.json", digest(body))
			if collision {
				if err == nil || filepath.Base(file.dos) != "state.json" {
					t.Fatal("collision replaced/adopted recovery")
				}
				raw, readErr := os.ReadFile(destination)
				if readErr != nil || string(raw) != "foreign recovery" {
					t.Fatal("collision bytes overwritten", readErr)
				}
			} else {
				if err != nil || file.pin.id != original || file.checkAlias() != nil {
					t.Fatal("state original custody lost on rename", err)
				}
				if err := startupRestorationAbsent(parent, "state.json"); err != nil {
					t.Fatal("active state alias remained", err)
				}
			}
			if err := windows.CloseHandle(file.pin.handle); err != nil {
				t.Fatal(err)
			}
			file.pin.handle = 0
			path := destination
			if collision {
				path = filepath.Join(parent.dos, "state.json")
			}
			raw, err := os.ReadFile(path)
			if err != nil || !bytes.Equal(raw, body) {
				t.Fatal("original state bytes not preserved", err)
			}
		})
	}
}
