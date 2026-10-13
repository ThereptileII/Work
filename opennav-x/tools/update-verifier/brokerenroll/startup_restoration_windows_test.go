//go:build windows

package brokerenroll

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"os"
	"path/filepath"
	"testing"

	"example.com/opennav-update-verifier/brokerregistration"
	"golang.org/x/sys/windows"
)

// Only inert temp-file owners. Production opens token-checked KnownFolders;
// tests use the same identity checks and native handle moves in disposable dirs.
func startupRestorationNativeFixture(t *testing.T) *nativeStartupRestoration {
	n, _ := startupRestorationNativeFixtureMode(t, false)
	return n
}

func startupRestorationNativeFixtureMode(t *testing.T, preparing bool) (*nativeStartupRestoration, *brokerregistration.Request) {
	t.Helper()
	owner, oldMenu, oldRecovery, request := startupCompletedBindingFixture(t)
	var generations *enrollmentPin
	if preparing {
		// Put the publication's actual ORIGINAL launcher in an inert generation
		// before the original publisher records its native identity. The producer
		// closes all of its immovable pins before the consumer opens the anchor.
		parentPath := filepath.Dir(oldRecovery.dos)
		parent, err := owner.files.pinDirectory(parentPath, "", parentPath, nil, false, request.ClientSID)
		if err != nil {
			t.Fatal(err)
		}
		generations, err = owner.files.newDirectory(parent, "generations", oldRecovery.descriptor, request.ClientSID)
		if err != nil {
			t.Fatal(err)
		}
		generation, err := owner.files.newDirectory(generations, request.Installed.Generation, oldRecovery.descriptor, request.ClientSID)
		if err != nil {
			t.Fatal(err)
		}
		app, err := owner.files.newDirectory(generation, "app", oldRecovery.descriptor, request.ClientSID)
		if err != nil {
			t.Fatal(err)
		}
		owner.image, err = startupWriteFile(context.Background(), owner.files, app, "skager-start.exe", []byte("INERT-LAUNCHER"), oldRecovery.descriptor, false)
		if err != nil {
			t.Fatal(err)
		}
	}
	if err := completeStartupPublicationBinding(context.Background(), owner, oldMenu, oldRecovery, request, oldRecovery.descriptor); err != nil {
		t.Fatal(err)
	}
	raw, err := os.ReadFile(filepath.Join(oldRecovery.dos, startupBindingName))
	if err != nil {
		t.Fatal(err)
	}
	approved, err := ParseApprovedStartupPublication(raw, digest(raw), request.OriginalBytes(), request.RequestSHA256)
	if err != nil {
		t.Fatal(err)
	}
	var record startupBindingRecord
	if err := json.Unmarshal(approved.OriginalBytes(), &record); err != nil {
		t.Fatal(err)
	}
	if err := owner.Close(); err != nil {
		t.Fatal(err)
	}
	n := &nativeStartupRestoration{files: &nativeEnrollmentFiles{}, clientSID: request.ClientSID, record: record, raw: raw}
	t.Cleanup(func() {
		if err := n.close(); err != nil {
			t.Error(err)
		}
	})
	n.menu, err = n.files.pinDirectory(oldMenu.dos, oldMenu.physical, oldMenu.dos, nil, false, n.clientSID)
	if err != nil {
		t.Fatal(err)
	}
	n.recovery, err = n.files.pinDirectory(oldRecovery.dos, oldRecovery.physical, oldRecovery.dos, nil, false, n.clientSID)
	if err != nil {
		t.Fatal(err)
	}
	if preparing {
		parent, err := n.files.pinDirectory(generations.dos, generations.physical, generations.dos, nil, false, request.ClientSID)
		if err != nil {
			t.Fatal(err)
		}
		n.anchor, err = openStartupRetirementAnchor(context.Background(), parent, record.Generation, record.Launcher)
		if err != nil {
			t.Fatal(err)
		}
	} else {
		n.image, err = startupOpenFile(n.files, n.recovery, "inert-launcher.exe", false, maxBrokerBytes)
		if err != nil {
			t.Fatal(err)
		}
	}
	n.manifest, err = startupOpenFile(n.files, n.recovery, "publication.json", false, maxStartupBindingBytes)
	if err != nil {
		t.Fatal(err)
	}
	n.receipt, err = startupOpenFile(n.files, n.recovery, startupBindingName, false, maxStartupBindingBytes)
	if err != nil {
		t.Fatal(err)
	}
	for i := range n.links {
		spec, _ := startupShortcutMode(i).spec()
		n.links[i], err = openStartupRestorationFile(n.files, n.menu, spec.name)
		if err != nil {
			t.Fatal(err)
		}
	}
	spec, _ := startupNormal.spec()
	original, err := openStartupRestorationFile(n.files, n.recovery, spec.name)
	if err != nil {
		t.Fatal(err)
	}
	n.original = &original
	if err := n.preflight(context.Background()); err != nil {
		t.Fatal(err)
	}
	return n, request
}

func TestNativeStartupRestorationMovesSameObjectsAndRestoresOriginal(t *testing.T) {
	n := startupRestorationNativeFixture(t)
	ctx := context.Background()
	originalID := n.original.pin.id
	originalBytes, err := startupReadOriginal(ctx, n.original.pin)
	if err != nil {
		t.Fatal(err)
	}
	checks := 0
	if err := n.restore(ctx, func(context.Context) error { checks++; return nil }); err != nil {
		t.Fatal(err)
	}
	if checks != 9 || !n.restored || n.original.pin.id != originalID {
		t.Fatal("same live witness/original identity not preserved")
	}
	if err := n.check(ctx); err != nil {
		t.Fatal(err)
	}
	for i, file := range n.links {
		if !n.retired[i] || file.pin.physical != n.recovery.physical+`\`+startupRetirementName(startupShortcutMode(i)) {
			t.Fatal("original published link not retired")
		}
		info, _, err := enrollmentInfo(file.pin.handle)
		creation := uint64(info.CreationTime.HighDateTime)<<32 | uint64(info.CreationTime.LowDateTime)
		if err != nil || creation != n.record.Links[i].New.Creation {
			t.Fatal("post-move NTFS creation not retained", err)
		}
	}
	if n.original.pin.physical != n.menu.physical+`\Skager.lnk` {
		t.Fatal("original normal shortcut not restored")
	}
	if err := n.restore(ctx, func(context.Context) error { return nil }); err == nil {
		t.Fatal("restoration replayed")
	}
	if err := n.close(); err != nil {
		t.Fatal(err)
	}
	raw, err := os.ReadFile(filepath.Join(n.menu.dos, "Skager.lnk"))
	if err != nil || !bytes.Equal(raw, originalBytes) {
		t.Fatal("original normal bytes differ", err)
	}
	for _, name := range []string{"Skager Legacy.lnk", "Skager Safe Mode.lnk"} {
		if _, err := os.Stat(filepath.Join(n.menu.dos, name)); !errors.Is(err, os.ErrNotExist) {
			t.Fatal("protected mode link still published", name, err)
		}
	}
}

func TestNativeStartupRestorationExcludesForeignSourceRename(t *testing.T) {
	n := startupRestorationNativeFixture(t)
	source, _ := windows.UTF16PtrFromString(n.links[0].dos)
	foreign, _ := windows.UTF16PtrFromString(filepath.Join(n.menu.dos, "foreign.lnk"))
	if err := windows.MoveFileEx(source, foreign, 0); err == nil {
		t.Fatal("retained original DELETE handle did not exclude external rename")
	}
	if err := n.check(context.Background()); err != nil {
		t.Fatal(err)
	}
}

func TestNativeStartupRestorationRefusesCollisionsAndSubstitution(t *testing.T) {
	for _, fault := range []string{"retirement-collision", "same-byte-foreign-id", "creation", "descriptor", "missing-completion"} {
		t.Run(fault, func(t *testing.T) {
			n := startupRestorationNativeFixture(t)
			switch fault {
			case "retirement-collision":
				// Existing directory pins have no expected descriptor pointer.
				// Measure the protected descriptor for fixture-only creation.
				sd, err := windows.GetSecurityInfo(n.recovery.handle, windows.SE_FILE_OBJECT, windows.OWNER_SECURITY_INFORMATION|windows.DACL_SECURITY_INFORMATION)
				if err != nil {
					t.Fatal(err)
				}
				if _, err := startupWriteFile(context.Background(), n.files, n.recovery, startupRetirementName(startupLegacy), []byte("FOREIGN-DO-NOT-REPLACE"), sd, false); err != nil {
					t.Fatal(err)
				}
			case "same-byte-foreign-id":
				raw, err := startupReadOriginal(context.Background(), n.links[0].pin)
				if err != nil {
					t.Fatal(err)
				}
				sd, err := windows.GetSecurityInfo(n.recovery.handle, windows.SE_FILE_OBJECT, windows.OWNER_SECURITY_INFORMATION|windows.DACL_SECURITY_INFORMATION)
				if err != nil {
					t.Fatal(err)
				}
				pin, err := startupWriteFile(context.Background(), n.files, n.recovery, "foreign.lnk", raw, sd, true)
				if err != nil {
					t.Fatal(err)
				}
				foreign, err := startupFileIdentity(context.Background(), pin, digest(raw))
				if err != nil {
					t.Fatal(err)
				}
				// Same bytes cannot substitute for the independently approved
				// original ID, even at the fixed correct published path.
				n.record.Links[0].New = foreign
			case "creation":
				n.record.Links[0].New.Creation ^= 1
			case "descriptor":
				n.record.Links[0].New.DescriptorSHA256 = digest([]byte("foreign descriptor"))
			case "missing-completion":
				n.receipt = nil
			}
			moves := 0
			err := n.restore(context.Background(), func(context.Context) error { moves++; return nil })
			if err == nil || moves != 1 || n.retired != [3]bool{} {
				t.Fatal("foreign/incomplete input permitted mutation", err)
			}
			if _, err := os.Stat(filepath.Join(n.menu.dos, "Skager.lnk")); err != nil {
				t.Fatal("preflight failure removed published normal link", err)
			}
		})
	}
}

func TestNativeStartupRestorationPartialFailurePreservesRecovery(t *testing.T) {
	n := startupRestorationNativeFixture(t)
	checks := 0
	err := n.restore(context.Background(), func(context.Context) error {
		checks++
		if checks == 4 {
			return errors.New("same live broker witness lost before second move")
		}
		return nil
	})
	if !errors.Is(err, ErrStartupRestorationUncertain) || !n.retired[0] || n.retired[1] || n.restored {
		t.Fatal("partial move state lost", err)
	}
	if err := n.check(context.Background()); err != nil {
		t.Fatal("partial original custody lost", err)
	}
	if err := n.restore(context.Background(), func(context.Context) error { return nil }); err == nil {
		t.Fatal("partial moves retried")
	}
	if _, err := os.Stat(filepath.Join(n.recovery.dos, startupRetirementName(startupNormal))); err != nil {
		t.Fatal("retired normal recovery missing", err)
	}
	if _, err := os.Stat(filepath.Join(n.recovery.dos, "Skager.lnk")); err != nil {
		t.Fatal("original normal backup missing", err)
	}
}
