package brokerstore

import (
	"bytes"
	"context"
	"errors"
	"example.com/opennav-update-verifier/brokerregistration"
	"strings"
	"testing"

	"example.com/opennav-update-verifier/baselineevidence"
)

func writeCandidateEvidence(t *testing.T, c *Candidate, f baselineFixtureData) {
	t.Helper()
	for name, raw := range f.payload {
		var err error
		if baselineevidence.IsEngineHelper(name) || baselineevidence.IsCommissioningHelper(name) {
			err = c.WriteEvidence(context.Background(), name, raw, digest(string(raw)))
		} else {
			err = c.WriteFile(context.Background(), name, bytes.NewReader(raw), int64(len(raw)), digest(string(raw)))
		}
		if err != nil {
			t.Fatal(err)
		}
	}
	for name, raw := range map[string][]byte{"FILE_SHA256.json": f.inventory, "installer/package.json": f.manifest} {
		if err := c.WriteEvidence(context.Background(), name, raw, digest(string(raw))); err != nil {
			t.Fatal(err)
		}
	}
}

func TestCandidateEngineHelpersExactSetAndRetainedHashes(t *testing.T) {
	for _, scenario := range []string{"complete", "missing", "tampered", "extra", "alias", "tamper-after-complete"} {
		t.Run(scenario, func(t *testing.T) {
			_, c, m := testCandidate(t)
			f := baselineEngineData(t)
			if scenario == "missing" {
				delete(f.payload, "installer/engine/UpdateTransaction.ps1")
			}
			if scenario == "tampered" {
				f.payload["installer/engine/Lifecycle.ps1"] = []byte("changed original")
			}
			writeCandidateEvidence(t, c, f)
			if scenario == "extra" || scenario == "alias" {
				name := "installer/engine/extra.ps1"
				if scenario == "alias" {
					name = "installer/engine/lifecycle.ps1"
				}
				if err := c.WriteEvidence(context.Background(), name, []byte("extra"), digest("extra")); err == nil {
					t.Fatal("nonfixed helper accepted")
				}
			}
			err := c.CompleteRecoveryEvidence(context.Background(), f.identity)
			if scenario != "complete" && scenario != "tamper-after-complete" {
				if err == nil || m.files[ReceiptFilename] != nil {
					t.Fatal("incomplete or changed helpers completed")
				}
				return
			}
			if err != nil {
				t.Fatal(err)
			}
			if scenario == "tamper-after-complete" {
				m.files["installer/engine/UpdateSupervisor.ps1"].data[0] ^= 1
			}
			err = c.Seal(context.Background(), []byte("opaque receipt"))
			if (err != nil) != (scenario == "tamper-after-complete") {
				t.Fatal("seal did not recheck exact helper originals", err)
			}
		})
	}
}

func TestCandidateEngineHelperParentCollisionsRefuse(t *testing.T) {
	for _, parent := range []string{"installer", "installer/engine"} {
		for _, kind := range []string{"alias", "file"} {
			t.Run(parent+"/"+kind, func(t *testing.T) {
				_, c, m := testCandidate(t)
				c.names[parent] = parent
				if kind == "alias" {
					c.names[parent] = strings.ToUpper(parent)
				} else {
					c.fileNames[parent] = true
				}
				name := "installer/engine/Lifecycle.ps1"
				if err := c.WriteEvidence(context.Background(), name, []byte("original"), digest("original")); err == nil || len(m.files) != 0 {
					t.Fatal("parent collision reached native creation")
				}
			})
		}
	}
}

func TestCandidateEvidenceRetainsOriginalsAndBindsBeforeSeal(t *testing.T) {
	_, c, m := testCandidate(t)
	f := baselineData(t)
	f.inventory = append([]byte(" \n"), f.inventory...)
	f.identity.InventorySHA256 = digest(string(f.inventory))
	writeCandidateEvidence(t, c, f)
	if err := c.CompleteRecoveryEvidence(context.Background(), f.identity); err != nil {
		t.Fatal(err)
	}
	if !bytes.Equal(m.files["FILE_SHA256.json"].data, f.inventory) || !bytes.Equal(m.files["installer/package.json"].data, f.manifest) {
		t.Fatal("original sidecars reconstructed")
	}
	if err := c.Seal(context.Background(), []byte("opaque receipt")); err != nil {
		t.Fatal(err)
	}
	if m.files[ReceiptFilename] == nil {
		t.Fatal("no sealed receipt")
	}
	for _, name := range []string{"app/OPENNAV_PORTABLE_PREVIEW", "skager-recovery.exe"} {
		if m.files[name] != nil {
			t.Fatal("nonpayload inventory entry staged")
		}
	}
}

func TestCandidateEvidenceRequiresCompleteExactPayloadAndOriginals(t *testing.T) {
	for _, scenario := range []string{"missing-inventory", "missing-manifest", "missing-payload", "extra-payload", "portable-marker", "changed-identity", "changed-declared-payload", "changed-stock", "missing-product", "untracked-native-file", "uncompleted-seal", "wrong-acquisition-plan"} {
		t.Run(scenario, func(t *testing.T) {
			_, c, m := testCandidate(t)
			f := baselineData(t)
			if scenario == "missing-inventory" {
				f.inventory = nil
			}
			if scenario == "missing-manifest" {
				f.manifest = nil
			}
			if scenario == "missing-payload" {
				delete(f.payload, "app/opennav-restart.exe")
			}
			if scenario == "missing-product" {
				delete(f.payload, "docs/PRODUCT_BUILD.json")
			}
			if scenario == "extra-payload" {
				f.payload["app/extra.dll"] = []byte("unsigned")
			}
			if scenario == "portable-marker" {
				f.payload["app/OPENNAV_PORTABLE_PREVIEW"] = []byte("portable marker, excluded")
			}
			if scenario == "changed-identity" {
				f.identity.InventorySHA256 = strings.Repeat("f", 64)
			}
			if scenario == "changed-declared-payload" {
				f.identity.DeclaredPayloadSHA256 = strings.Repeat("f", 64)
			}
			if scenario == "changed-stock" {
				f.manifest = bytes.ReplaceAll(f.manifest, []byte(`"arch":"`+brokerregistration.StockBaseline().Arch+`"`), []byte(`"arch":"unsupported"`))
			}
			for name, raw := range f.payload {
				if err := c.WriteFile(context.Background(), name, bytes.NewReader(raw), int64(len(raw)), digest(string(raw))); err != nil {
					t.Fatal(err)
				}
			}
			for name, raw := range map[string][]byte{"FILE_SHA256.json": f.inventory, "installer/package.json": f.manifest} {
				if raw != nil {
					if err := c.WriteEvidence(context.Background(), name, raw, digest(string(raw))); err != nil {
						t.Fatal(err)
					}
				}
			}
			if scenario == "untracked-native-file" {
				m.files["docs/foreign.txt"] = &memoryFile{data: []byte("foreign")}
			}
			if scenario == "wrong-acquisition-plan" {
				c.acquisition = &Acquisition{plan: &AcquisitionPlan{Version: "other", Commit: f.identity.Commit}}
				defer func() { c.acquisition = nil }()
			}
			var err error
			if scenario == "uncompleted-seal" {
				err = c.Seal(context.Background(), []byte("receipt"))
			} else {
				err = c.CompleteRecoveryEvidence(context.Background(), f.identity)
			}
			if err == nil {
				t.Fatal("incomplete or mismatched recovery evidence accepted")
			}
			if m.files[ReceiptFilename] != nil {
				t.Fatal("receipt before complete verification")
			}
			if err := c.Seal(context.Background(), []byte("retry")); !errors.Is(err, ErrPoisoned) {
				t.Fatalf("failed evidence not poisoned: %v", err)
			}
		})
	}
}

func TestCandidateEvidenceSealRechecksEveryHeldOriginalAndPayload(t *testing.T) {
	for _, name := range []string{"FILE_SHA256.json", "installer/package.json", "docs/PRODUCT_BUILD.json", "app/opencpn.exe"} {
		t.Run(name, func(t *testing.T) {
			_, c, m := testCandidate(t)
			f := baselineData(t)
			writeCandidateEvidence(t, c, f)
			if err := c.CompleteRecoveryEvidence(context.Background(), f.identity); err != nil {
				t.Fatal(err)
			}
			m.files[name].data[0] ^= 1
			if err := c.Seal(context.Background(), []byte("receipt")); err == nil {
				t.Fatal("held original mutation accepted")
			}
			if m.files[ReceiptFilename] != nil {
				t.Fatal("receipt after original mutation")
			}
		})
	}
}

func TestCandidateEvidenceBoundsAliasesAndFaultsPoison(t *testing.T) {
	for _, scenario := range []string{"alias", "other", "payload-path", "empty", "large", "hash", "duplicate", "create", "sync", "custody", "canceled", "nil-context"} {
		t.Run(scenario, func(t *testing.T) {
			_, c, m := testCandidate(t)
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			var callCtx context.Context = ctx
			name, raw := "FILE_SHA256.json", []byte("original")
			hash := digest(string(raw))
			switch scenario {
			case "alias":
				name = "file_sha256.json"
			case "other":
				name = "installer/other.json"
			case "payload-path":
				name = "app/FILE_SHA256.json"
			case "empty":
				raw = nil
			case "large":
				raw = make([]byte, maxCandidateEvidenceBytes+1)
			case "hash":
				hash = strings.Repeat("0", 64)
			case "duplicate":
				if err := c.WriteEvidence(ctx, name, raw, hash); err != nil {
					t.Fatal(err)
				}
			case "create":
				m.createFault = testFault
			case "sync":
				m.configure = func(f *memoryFile) { f.syncFault = testFault }
			case "custody":
				m.configure = func(f *memoryFile) { f.fault = testFault }
			case "canceled":
				cancel()
			case "nil-context":
				callCtx = nil
			}
			if err := c.WriteEvidence(callCtx, name, raw, hash); err == nil {
				t.Fatal("invalid original write accepted")
			}
			if err := write(c, "app/after", "x"); !errors.Is(err, ErrPoisoned) {
				t.Fatalf("evidence fault not poisoned: %v", err)
			}
			if err := c.Close(); err != nil {
				t.Fatal(err)
			}
			for _, file := range m.files {
				if file.closes != 1 {
					t.Fatal("partial evidence handle not closed once")
				}
			}
		})
	}
}

func TestCandidateEvidenceCancellationAndCompletionFreeze(t *testing.T) {
	for _, scenario := range []string{"cancel-read", "late-payload", "late-evidence", "repeat-completion", "native-extra-after-complete", "cleanup-fault"} {
		t.Run(scenario, func(t *testing.T) {
			_, c, m := testCandidate(t)
			f := baselineData(t)
			writeCandidateEvidence(t, c, f)
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			if scenario == "cancel-read" {
				m.files["FILE_SHA256.json"].onRead = cancel
				if err := c.CompleteRecoveryEvidence(ctx, f.identity); !errors.Is(err, context.Canceled) {
					t.Fatal(err)
				}
			} else {
				if err := c.CompleteRecoveryEvidence(ctx, f.identity); err != nil {
					t.Fatal(err)
				}
				var err error
				switch scenario {
				case "late-payload":
					err = write(c, "app/late", "x")
				case "late-evidence":
					err = c.WriteEvidence(ctx, "FILE_SHA256.json", f.inventory, digest(string(f.inventory)))
				case "repeat-completion":
					err = c.CompleteRecoveryEvidence(ctx, f.identity)
				case "native-extra-after-complete":
					m.files["docs/extra"] = &memoryFile{}
					err = c.Seal(ctx, []byte("receipt"))
					delete(m.files, "docs/extra")
				case "cleanup-fault":
					m.files["FILE_SHA256.json"].closeFault = testFault
					err = c.Close()
					if !errors.Is(err, testFault) || c.Close() != err {
						t.Fatal("cleanup uncertainty changed")
					}
					return
				}
				if err == nil {
					t.Fatal("completed evidence changed")
				}
			}
			if m.files[ReceiptFilename] != nil {
				t.Fatal("receipt after evidence failure")
			}
		})
	}
}

func TestCandidateCommissioningHelpersExactSetAndRetainedHashes(t *testing.T) {
	for _, scenario := range []string{"complete", "missing", "tampered", "extra", "alias", "tamper-after-complete"} {
		t.Run(scenario, func(t *testing.T) {
			_, c, m := testCandidate(t)
			f := baselineCommissioningData(t)
			if scenario == "missing" {
				delete(f.payload, "installer/commissioning-v1/RestartDashboardPersistence.ps1")
			}
			if scenario == "tampered" {
				f.payload["installer/commissioning-v1/Commissioning.ps1"] = []byte("changed original")
			}
			writeCandidateEvidence(t, c, f)
			if scenario == "extra" || scenario == "alias" {
				name := "installer/commissioning-v1/extra.ps1"
				if scenario == "alias" {
					name = "installer/commissioning-v1/commissioning.ps1"
				}
				if err := c.WriteEvidence(context.Background(), name, []byte("extra"), digest("extra")); err == nil {
					t.Fatal("nonfixed helper accepted")
				}
			}
			err := c.CompleteRecoveryEvidence(context.Background(), f.identity)
			if scenario != "complete" && scenario != "tamper-after-complete" {
				if err == nil || m.files[ReceiptFilename] != nil {
					t.Fatal("incomplete or changed helpers completed")
				}
				return
			}
			if err != nil {
				t.Fatal(err)
			}
			if scenario == "tamper-after-complete" {
				m.files["installer/commissioning-v1/Preparation.ps1"].data[0] ^= 1
			}
			err = c.Seal(context.Background(), []byte("opaque receipt"))
			if (err != nil) != (scenario == "tamper-after-complete") {
				t.Fatal("seal did not recheck exact helper originals", err)
			}
		})
	}
}
