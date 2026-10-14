//go:build windows

package brokerenroll

import (
	"bytes"
	"context"
	"errors"
	"os"
	"path/filepath"
	"strings"
	"testing"

	"golang.org/x/sys/windows"
)

func TestNativeStartupInstalledRetirementPreservesOriginalShortcutCustody(t *testing.T) {
	for _, owned := range []bool{true, false} {
		name := "preserve stock or unknown target"
		if owned {
			name = "retire integration target"
		}
		t.Run(name, func(t *testing.T) {
			ctx := context.Background()
			target, arguments, working := `C:\Inert\app\skager-start.exe`, "--xnav", `C:\Inert\app`
			if !owned {
				target, arguments, working = `C:\Stock\opencpn.exe`, "", `C:\Stock`
			}
			raw, err := startupLinkBytes(target, arguments, working)
			if err != nil {
				t.Fatal(err)
			}
			n, owner, d := nativePreparedStartupMaintenanceFixture(t, raw)
			originalID, originalHandle := n.originals[0].pin.id, n.originals[0].pin.handle
			if err := n.prepareInstalled(ctx, func(gotTarget, gotArgs, gotWorking string) bool {
				if gotTarget != target || gotArgs != arguments || gotWorking != working {
					t.Fatal("original retained shortcut fields changed", gotTarget, gotArgs, gotWorking)
				}
				return owned
			}); err != nil {
				t.Fatal(err)
			}
			snapshot := preparedRetirementSnapshot(t, owner)
			snapshot.Generation = strings.Repeat("c", 32)
			rename := prepareStartupFixtureRetirement(t, n.anchor, snapshot.RequestID)
			if err := restorePreparedFixture(t, owner, snapshot, &d.events); err != nil {
				t.Fatal(err)
			}
			if err := rename(); err != nil {
				t.Fatal(err)
			}
			if err := bindDecommissionSnapshotWith(owner, ctx, &decommissionFenceFixture{events: &d.events}, snapshot.ClientSID, snapshot.RegistrationSHA256, &snapshot); err != nil {
				t.Fatal(err)
			}
			if err := n.retireInstalled(ctx, owner.startupGuard); err != nil {
				t.Fatal(err)
			}
			if n.originalRetired[0] != owned || n.originals[0].pin.id != originalID || n.originals[0].pin.handle != originalHandle || d.disables != 0 {
				t.Fatal("installed retirement lost original identity or touched SCM")
			}
			if err := n.check(ctx); err != nil {
				t.Fatal("original startup custody failed after installed retirement", err)
			}
			path := filepath.Join(n.menu.dos, "Skager.lnk")
			if owned {
				if _, err := os.Stat(path); !errors.Is(err, os.ErrNotExist) {
					t.Fatal("integration shortcut still published", err)
				}
				path = filepath.Join(n.recovery.dos, installedOriginalStartupName)
			}
			// The production owner deliberately holds DELETE access. os.ReadFile's
			// ordinary Windows sharing cannot reopen it until custody is closed.
			retained, err := startupReadOriginal(ctx, n.originals[0].pin)
			if err != nil || !bytes.Equal(retained, raw) {
				t.Fatal("original shortcut bytes were not preserved", err)
			}
			if err := n.retireInstalled(ctx, owner.startupGuard); err == nil {
				t.Fatal("installed shortcut retirement repeated")
			}
			if _, err := owner.Decommission(ctx); err != nil {
				t.Fatal("retained original recovery did not survive SCM sequence", err)
			}
			retained, err = os.ReadFile(path)
			if err != nil || !bytes.Equal(retained, raw) {
				t.Fatal("original shortcut bytes unavailable after owner drain", err)
			}
		})
	}
}

func TestNativeStartupInstalledPreparationRefusesUnclassifiableOriginal(t *testing.T) {
	n, _, d := nativePreparedStartupMaintenanceFixture(t)
	if err := n.prepareInstalled(context.Background(), func(string, string, string) bool { return true }); err == nil {
		t.Fatal("malformed original shortcut acquired integration removal authority")
	}
	if !n.installedFailed || d.disables != 0 || n.attempted {
		t.Fatal("failed classification mutated startup or SCM")
	}
}

func TestNativeStartupInstalledRetirementClassifiesOriginalSafeIndependently(t *testing.T) {
	for _, tc := range []struct {
		name         string
		normal, safe bool
	}{
		{"unknown safe remains restored", false, false},
		{"setup safe retires before SCM", false, true},
		{"setup normal and safe retire before SCM", true, true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			ctx := context.Background()
			target, working := `C:\Inert\app\opencpn.exe`, `C:\Inert\app`
			stock, err := startupLinkBytes(`C:\Stock\opencpn.exe`, "", `C:\Stock`)
			if err != nil {
				t.Fatal(err)
			}
			if tc.normal {
				stock, err = startupLinkBytes(target, "--xnav", working)
				if err != nil {
					t.Fatal(err)
				}
			}
			legacy, err := startupLinkBytes(`C:\User\unknown.exe`, "--custom", `C:\User`)
			if err != nil {
				t.Fatal(err)
			}
			safeTarget := `C:\User\unknown.exe`
			if tc.safe {
				safeTarget = target
			}
			safe, err := startupLinkBytes(safeTarget, "--safe-mode", working)
			if err != nil {
				t.Fatal(err)
			}
			originals := [3][]byte{stock, legacy, safe}
			n, owner, d := nativePreparedStartupMaintenanceFixture(t, originals[:]...)
			ids := [3]enrollmentFileID{}
			handles := [3]windows.Handle{}
			for i, original := range n.originals {
				ids[i], handles[i] = original.pin.id, original.pin.handle
			}
			classifications := 0
			if err := n.prepareInstalled(ctx, func(gotTarget, gotArgs, gotWorking string) bool {
				classifications++
				return gotTarget == target && (gotArgs == "--safe-mode" || gotArgs == "--xnav") && gotWorking == working
			}); err != nil {
				t.Fatal(err)
			}
			if classifications != 3 || n.installedEligible != [3]bool{tc.normal, false, tc.safe} {
				t.Fatal("original roles were not classified independently")
			}
			snapshot := preparedRetirementSnapshot(t, owner)
			rename := prepareStartupFixtureRetirement(t, n.anchor, snapshot.RequestID)
			if err := restorePreparedFixture(t, owner, snapshot, &d.events); err != nil {
				t.Fatal(err)
			}
			if err := rename(); err != nil {
				t.Fatal(err)
			}
			if err := bindDecommissionSnapshotWith(owner, ctx, &decommissionFenceFixture{events: &d.events}, snapshot.ClientSID, snapshot.RegistrationSHA256, &snapshot); err != nil {
				t.Fatal(err)
			}
			if err := n.retireInstalled(ctx, owner.startupGuard); err != nil {
				t.Fatal(err)
			}
			if n.originalRetired != [3]bool{tc.normal, false, tc.safe} || d.disables != 0 {
				t.Fatal("wrong original retired or SCM ran early")
			}
			if err := n.check(ctx); err != nil {
				t.Fatal(err)
			}
			for i, original := range n.originals {
				if original.pin.id != ids[i] || original.pin.handle != handles[i] {
					t.Fatal("original handle/identity changed", i)
				}
				raw, err := startupReadOriginal(ctx, original.pin)
				if err != nil || !bytes.Equal(raw, originals[i]) {
					t.Fatal("original bytes changed", i, err)
				}
			}
			if _, err := owner.Decommission(ctx); err != nil {
				t.Fatal("original custody failed through SCM cleanup", err)
			}
			for i, want := range originals {
				spec, _ := startupShortcutMode(i).spec()
				path := filepath.Join(n.menu.dos, spec.name)
				if n.originalRetired[i] {
					if _, err := os.Stat(path); !errors.Is(err, os.ErrNotExist) {
						t.Fatal("integration shortcut remains published", err)
					}
					path = filepath.Join(n.recovery.dos, installedOriginalStartupRetirementName(startupShortcutMode(i)))
				}
				raw, err := os.ReadFile(path)
				if err != nil || !bytes.Equal(raw, want) {
					t.Fatal("original shortcut lost after drain", i, err)
				}
			}
		})
	}
}

func TestNativeStartupInstalledPreparationRefusesUnclassifiableSafeOriginal(t *testing.T) {
	raw, err := startupLinkBytes(`C:\Stock\opencpn.exe`, "", `C:\Stock`)
	if err != nil {
		t.Fatal(err)
	}
	n, _, d := nativePreparedStartupMaintenanceFixture(t, raw, nil, []byte("UNKNOWN-MALFORMED-SAFE"))
	if err := n.prepareInstalled(context.Background(), func(string, string, string) bool { return false }); err == nil {
		t.Fatal("unclassifiable Safe original accepted")
	}
	if !n.installedFailed || d.disables != 0 || n.attempted {
		t.Fatal("failed Safe classification mutated startup or SCM")
	}
}
