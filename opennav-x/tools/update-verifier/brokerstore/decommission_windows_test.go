package brokerstore

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"os"
	"path/filepath"
	"runtime"
	"testing"

	"golang.org/x/sys/windows"
)

// These current-user temporary namespaces exercise the actual Win32 store
// custody and terminal file mechanics. They never open the product namespace,
// provision a service, touch profiles, or manufacture client/broker authority.
func nativeDecommissionFixture(t *testing.T) (*nativeStore, DecommissionBinding, []byte) {
	t.Helper()
	n, r := nativeRegistrationFixture(t)
	if err := beginRegistration(context.Background(), r); err != nil {
		t.Fatal(err)
	}
	createNativeTestChannel(t, r)
	raw := []byte(`{"schema":1,"inertDecommissionFixture":true}`)
	if err := r.complete(context.Background(), raw); err != nil {
		t.Fatal(err)
	}
	if err := r.close(); err != nil {
		t.Fatal(err)
	}
	binding := decommissionTestBinding()
	binding.ClientSID = r.client
	binding.RegistrationSHA256 = registrationDigest(raw)
	return n, binding, raw
}
func TestNativeDecommissionExactOriginalReceiptAndPermanentOrdinaryDenial(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	n, binding, raw := nativeDecommissionFixture(t)
	d, err := openDecommissionInventory(context.Background(), n, binding)
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() {
		if err := d.close(); err != nil {
			t.Error(err)
		}
	})
	// The same original completion and registration handles exclude mutation.
	if err = os.WriteFile(filepath.Join(n.rootDOS, "registrations", binding.ClientSID+".json"), []byte("changed"), 0600); !errors.Is(err, windows.ERROR_SHARING_VIOLATION) {
		t.Fatalf("original registration writable: %v", err)
	}
	if err = d.commit(context.Background()); err != nil {
		t.Fatal(err)
	}
	if err = d.check(); err != nil {
		t.Fatal("original terminal custody", err)
	}
	if err = n.check(); !errors.Is(err, ErrDecommissioned) {
		t.Fatalf("ordinary native check: %v", err)
	}
	// Ordinary refusal must not invalidate the retained exact terminal receipt.
	if err = d.check(); err != nil {
		t.Fatal("ordinary denial poisoned receipt", err)
	}
	if err = d.commit(context.Background()); err == nil {
		t.Fatal("terminal native writer retried")
	}
	if err = d.close(); err != nil {
		t.Fatal(err)
	}
	actual, err := os.ReadFile(filepath.Join(n.rootDOS, decommissionFilename))
	if err != nil || !bytes.Equal(actual, decommissionBytes(binding)) {
		t.Fatalf("durable exact binding: %q %v", actual, err)
	}
	if err = os.Remove(filepath.Join(n.rootDOS, decommissionFilename)); err != nil {
		t.Fatal("fixture-only fence removal", err)
	}
	if err = n.check(); !errors.Is(err, ErrDecommissioned) {
		t.Fatal("original store resumed after observed terminal fence", err)
	}
	actual, err = os.ReadFile(filepath.Join(n.rootDOS, "registrations", binding.ClientSID+".json"))
	if err != nil || !bytes.Equal(actual, raw) {
		t.Fatal("original registration changed", err)
	}
}
func TestNativeDecommissionAnyPresenceRefusesWithoutParsing(t *testing.T) {
	for _, kind := range []string{"empty", "malformed", "directory", "held-exclusive"} {
		t.Run(kind, func(t *testing.T) {
			runtime.LockOSThread()
			defer runtime.UnlockOSThread()
			n, _ := nativeRegistrationFixture(t)
			path := filepath.Join(n.rootDOS, decommissionFilename)
			switch kind {
			case "empty":
				if err := os.WriteFile(path, nil, 0600); err != nil {
					t.Fatal(err)
				}
			case "malformed":
				if err := os.WriteFile(path, []byte("partial\x00garbage"), 0600); err != nil {
					t.Fatal(err)
				}
			case "directory":
				if err := os.Mkdir(path, 0700); err != nil {
					t.Fatal(err)
				}
			case "held-exclusive":
				sd, err := descriptor(n.broker, nil, false)
				if err != nil {
					t.Fatal(err)
				}
				held, err := createRegistrationFile(n.root, decommissionFilename, sd)
				if err != nil {
					t.Fatal(err)
				}
				defer held.Close()
			}
			if err := decommissionAbsent(n.root); !errors.Is(err, ErrDecommissioned) {
				t.Fatalf("presence accepted: %v", err)
			}
			if err := n.check(); !errors.Is(err, ErrDecommissioned) {
				t.Fatalf("store resumed: %v", err)
			}
		})
	}
}
func TestNativeDecommissionInventoryRefusesOtherClientPartialAndChangedDigest(t *testing.T) {
	for _, kind := range []string{"extra", "partial", "digest", "missing"} {
		t.Run(kind, func(t *testing.T) {
			runtime.LockOSThread()
			defer runtime.UnlockOSThread()
			n, binding, _ := nativeDecommissionFixture(t)
			registrationRoot := filepath.Join(n.rootDOS, "registrations")
			switch kind {
			case "extra":
				if err := os.WriteFile(filepath.Join(registrationRoot, "unknown.once"), nil, 0600); err != nil {
					t.Fatal(err)
				}
			case "partial":
				if err := os.WriteFile(filepath.Join(registrationRoot, binding.ClientSID+".once"), []byte("started"), 0600); err != nil {
					t.Fatal(err)
				}
			case "digest":
				binding.RegistrationSHA256 = registrationDigest([]byte("wrong"))
			case "missing":
				if err := os.Remove(filepath.Join(registrationRoot, binding.ClientSID+".json")); err != nil {
					t.Fatal(err)
				}
			}
			d, err := openDecommissionInventory(context.Background(), n, binding)
			if d != nil || err == nil {
				if d != nil {
					d.close()
				}
				t.Fatal("unowned inventory accepted", err)
			}
			if err = decommissionAbsent(n.root); err != nil {
				t.Fatal("preflight created terminal evidence", err)
			}
		})
	}
}

func TestNativeDecommissionQualifiedOriginalInventorySurvivesTerminalFence(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	s, previous, view, _, raw := nativeBaselineViewFixture(t)
	baseline, err := s.MaterializeApprovedBaselineReadOnlyFresh(context.Background(), previous, raw, registrationDigest(raw))
	if err != nil {
		t.Fatal(err)
	}
	if err = baseline.Close(); err != nil {
		t.Fatal(err)
	}
	if err = previous.Close(); err != nil {
		t.Fatal(err)
	}
	opener := &nativeGenerationReopenFixtureStore{view}
	generation, evidence, err := opener.openReadOnlyGeneration(context.Background(), view.sid.String(), stageA)
	if err != nil {
		t.Fatal(err)
	}
	record, err := validateGenerationReopenEvidence(evidence, view.sid.String(), stageA)
	if err != nil {
		t.Fatal(err)
	}
	if err = generation.close(); err != nil {
		t.Fatal(err)
	}
	n := view.nativeStore
	journalBackend, err := createNativeExecutionJournal(context.Background(), n, view.sid.String(), stageA)
	if err != nil {
		t.Fatal(err)
	}
	journal := journalBackend.(*nativeExecutionJournal)
	ids, err := journal.identities()
	if err != nil {
		t.Fatal(err)
	}
	origin := executionJournalOrigin{Schema: 1, Purpose: "protected-execution-facts", ExecutionID: stageA, ClientSID: view.sid.String(), Generation: stageA, GenerationPurpose: record.Purpose, OriginalCompletion: evidence.completion, CompletionSHA256: registrationDigest(evidence.completion), Identities: ids}
	fence, err := json.Marshal(origin)
	if err != nil {
		t.Fatal(err)
	}
	facts := append(append([]byte(nil), fence...), '\n')
	if err = journal.initialize(context.Background(), fence, facts); err != nil {
		t.Fatal(err)
	}
	if err = journal.close(); err != nil {
		t.Fatal(err)
	}
	archiveBackend, err := createNativeExecutionProfileBackup(context.Background(), n, view.sid.String(), stageA)
	if err != nil {
		t.Fatal(err)
	}
	archive := archiveBackend.(*nativeExecutionProfileBackup)
	archiveIDs, err := archive.identities()
	if err != nil {
		t.Fatal(err)
	}
	archiveOrigin, err := json.Marshal(executionProfileBackupOrigin{1, "protected-execution-profile-backup", fence, archiveIDs})
	if err != nil {
		t.Fatal(err)
	}
	archiveMarker := append(archiveOrigin, '\n')
	if err = archive.initialize(context.Background(), archiveMarker); err != nil {
		t.Fatal(err)
	}
	nativeCompleteProfileBackup(t, archive, archiveMarker)
	if err = archive.close(); err != nil {
		t.Fatal(err)
	}
	sd, err := descriptor(n.broker, nil, false)
	if err != nil {
		t.Fatal(err)
	}
	registration, err := readCompletedRegistration(n.registrations, view.sid.String(), sd)
	if err != nil {
		t.Fatal(err)
	}
	binding := decommissionTestBinding()
	binding.ClientSID = view.sid.String()
	binding.RegistrationSHA256 = registrationDigest(registration)
	d, err := openDecommissionInventoryWithOpeners(context.Background(), n, binding, decommissionInventoryOpeners{opener.openReadOnlyGeneration, nil})
	if err != nil {
		t.Fatal("actual qualified inventory refused", err)
	}
	defer d.close()
	if len(d.extraFiles) == 0 || len(d.owners) < 2 {
		t.Fatal("known metadata silently tolerated without original readers")
	}
	if err = d.commit(context.Background()); err != nil {
		t.Fatal(err)
	}
	if err = n.check(); !errors.Is(err, ErrDecommissioned) {
		t.Fatal("ordinary access resumed", err)
	}
	if err = d.check(); err != nil {
		t.Fatal("private original qualified custody lost", err)
	}
	// Unknown child in the retained private generation registry is still refused.
	path := physical(n.registrations.name + `\` + view.sid.String() + ".generations" + `\unknown.json`)
	if err = os.WriteFile(path, []byte("unknown"), 0600); err != nil {
		t.Fatal(err)
	}
	if err = d.check(); err == nil {
		t.Fatal("new unknown registry metadata silently tolerated")
	}
}

func TestNativeDecommissionOriginalSignedStageSealInventory(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	_, candidate, wrapper := nativeStageSealFixture(t)
	if err := candidate.Seal(context.Background(), []byte("inert staged receipt\n")); err != nil {
		t.Fatal(err)
	}
	if err := candidate.Close(); err != nil {
		t.Fatal(err)
	}
	n := wrapper.nativeStore
	sd, err := descriptor(n.broker, nil, false)
	if err != nil {
		t.Fatal(err)
	}
	raw, err := readCompletedRegistration(n.registrations, candidate.clientSID, sd)
	if err != nil {
		t.Fatal(err)
	}
	binding := decommissionTestBinding()
	binding.ClientSID = candidate.clientSID
	binding.GenerationID = candidate.stageID
	binding.RegistrationSHA256 = registrationDigest(raw)
	d, err := openDecommissionInventoryWithOpeners(context.Background(), n, binding, decommissionInventoryOpeners{wrapper.openReadOnlyGeneration, wrapper.openStageSeal})
	if err != nil {
		t.Fatal("original signed stage inventory refused", err)
	}
	defer d.close()
	if err = d.commit(context.Background()); err != nil {
		t.Fatal(err)
	}
	if err = d.check(); err != nil {
		t.Fatal("attached original stage custody lost after fence", err)
	}
}
