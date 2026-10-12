package brokerstore

import (
	"bytes"
	"context"
	"errors"
	"os"
	"runtime"
	"strings"
	"testing"

	"golang.org/x/sys/windows"
)

// Uses existing disposable low-level Windows fixtures under the current owner;
// this does not provision the broker, qualify ProgramData or launch any payload.
func nativeEvidenceCandidate(t *testing.T) (*Candidate, *nativeCandidate) {
	t.Helper()
	native, _ := nativeAcquisitionFixture(t)
	store := &Store{backend: native.store}
	candidate := &Candidate{store: store, backend: native, names: map[string]string{}, fileNames: map[string]bool{}, acquisition: &Acquisition{plan: &AcquisitionPlan{Version: "2.0.0-beta.2", Commit: baselineData(t).identity.Commit}}}
	// Acquisition lifetime belongs to the existing fixture; the Candidate merely
	// accounts for that private namespace during its complete payload inventory.
	t.Cleanup(func() {
		candidate.acquisition = nil
		if err := candidate.Close(); err != nil {
			t.Error(err)
		}
	})
	return candidate, native
}

func TestNativeCandidateEvidenceOriginalCustodyAndSeal(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	c, n := nativeEvidenceCandidate(t)
	f := baselineData(t)
	writeCandidateEvidence(t, c, f)
	if err := c.CompleteRecoveryEvidence(context.Background(), f.identity); err != nil {
		t.Fatal(err)
	}
	for name, expected := range map[string][]byte{"FILE_SHA256.json": f.inventory, "installer/package.json": f.manifest} {
		path := physical(n.directories[""].name + `\` + strings.ReplaceAll(name, "/", `\`))
		raw, err := os.ReadFile(path)
		if err != nil || !bytes.Equal(raw, expected) {
			t.Fatal("original bytes differ", err)
		}
		for _, access := range []uint32{windows.GENERIC_WRITE, windows.DELETE} {
			if err := nativeAttemptFile(t, path, access); !errors.Is(err, windows.ERROR_SHARING_VIOLATION) {
				t.Fatalf("original evidence custody not retained: %v", err)
			}
		}
	}
	if err := c.Seal(context.Background(), []byte("opaque receipt")); err != nil {
		t.Fatal(err)
	}
}

func TestNativeCandidateEvidenceExactResidueNamespace(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	for _, scenario := range []string{"unknown-installer-member", "case-alias", "file-directory", "extra-payload"} {
		t.Run(scenario, func(t *testing.T) {
			c, n := nativeEvidenceCandidate(t)
			f := baselineData(t)
			writeCandidateEvidence(t, c, f)
			if err := c.CompleteRecoveryEvidence(context.Background(), f.identity); err != nil {
				t.Fatal(err)
			}
			root := n.directories[""].name
			switch scenario {
			case "unknown-installer-member":
				if err := os.WriteFile(physical(root+`\installer\unexpected`), []byte("extra"), 0600); err != nil {
					t.Fatal(err)
				}
			case "case-alias":
				if err := os.WriteFile(physical(root+`\installer\Package2.json`), []byte("alias"), 0600); err != nil {
					t.Fatal(err)
				}
			case "file-directory":
				if err := os.Mkdir(physical(root+`\installer\unexpected-dir`), 0700); err != nil {
					t.Fatal(err)
				}
			case "extra-payload":
				if err := os.WriteFile(physical(root+`\app\extra.dll`), []byte("extra"), 0600); err != nil {
					t.Fatal(err)
				}
			}
			if err := c.Seal(context.Background(), []byte("receipt")); err == nil {
				t.Fatal("foreign native namespace sealed")
			}
		})
	}
}

func TestNativeCandidateEvidenceReconciliationPreservesExactOriginalResidue(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	for _, scenario := range []string{"originals", "empty-installer", "unknown-member", "alias-member", "hardlink", "wrong-type"} {
		t.Run(scenario, func(t *testing.T) {
			native, _ := nativeAcquisitionFixture(t)
			writeOriginal := func(name string) {
				file, err := native.create(name)
				if err != nil {
					t.Fatal(err)
				}
				if _, err = file.Write([]byte("partial inert original")); err != nil {
					t.Fatal(err)
				}
				if err = file.Sync(); err != nil {
					t.Fatal(err)
				}
				if err = file.Close(); err != nil {
					t.Fatal(err)
				}
			}
			writeOriginal("FILE_SHA256.json")
			if scenario == "empty-installer" {
				dir, err := native.newDirectory(native.directories[""], "installer")
				if err != nil {
					t.Fatal(err)
				}
				native.directories["installer"] = dir
			} else {
				writeOriginal("installer/package.json")
			}
			root := native.directories[""].name
			switch scenario {
			case "unknown-member":
				if err := os.WriteFile(physical(root+`\installer\unexpected`), []byte("extra"), 0600); err != nil {
					t.Fatal(err)
				}
			case "alias-member":
				if err := os.Rename(physical(root+`\installer\package.json`), physical(root+`\installer\Package.json`)); err != nil {
					t.Fatal(err)
				}
			case "hardlink":
				if err := os.Link(physical(root+`\FILE_SHA256.json`), physical(root+`\original-link`)); err != nil {
					t.Fatal(err)
				}
			case "wrong-type":
				if err := os.Remove(physical(root + `\installer\package.json`)); err != nil {
					t.Fatal(err)
				}
				if err := os.Mkdir(physical(root+`\installer\package.json`), 0700); err != nil {
					t.Fatal(err)
				}
			}
			inventoryErr := native.checkRecoveryEvidenceResidue("FILE_SHA256.json")
			manifestErr := native.checkRecoveryEvidenceResidue("installer")
			if scenario == "originals" || scenario == "empty-installer" {
				if inventoryErr != nil || manifestErr != nil {
					t.Fatal(inventoryErr, manifestErr)
				}
			} else if inventoryErr == nil && manifestErr == nil {
				t.Fatal("unknown or unsafe evidence residue accepted")
			}
			if raw, err := os.ReadFile(physical(root + `\FILE_SHA256.json`)); err != nil || string(raw) != "partial inert original" {
				t.Fatal("reconciliation changed original residue", err)
			}
		})
	}
}
