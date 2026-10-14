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
			originalID, originalHandle := n.original.pin.id, n.original.pin.handle
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
			if n.originalRetired != owned || n.original.pin.id != originalID || n.original.pin.handle != originalHandle || d.disables != 0 {
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
			retained, err := startupReadOriginal(ctx, n.original.pin)
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
