package brokerstore

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"os"
	"path/filepath"
	"runtime"
	"strings"
	"testing"

	"golang.org/x/sys/windows"
)

// Temporary existing-only factory: production uses actual broker/account and
// fixed Program Files discovery. All reopened original records, physical IDs,
// descriptors, namespace and readonly FILE_OBJECT checks remain native.
type nativeGenerationReopenFixtureStore struct {
	*nativeBaselineViewFixtureStore
}

func (s *nativeGenerationReopenFixtureStore) openReadOnlyGeneration(ctx context.Context, client, stage string) (generationReopenBackend, generationReopenEvidence, error) {
	g, err := prepareNativeInstalledGeneration(s.nativeStore, nil, s.sid, client, stage)
	if err != nil {
		return nil, generationReopenEvidence{}, err
	}
	g.applications, err = openPin(physical(s.applications.name), s.applications.name, s.applications.security)
	if err != nil {
		return nil, generationReopenEvidence{}, errors.Join(err, g.close())
	}
	g.ancestors = append(g.ancestors, g.applications)
	if err = g.pin(g.applications); err != nil {
		return nil, generationReopenEvidence{}, errors.Join(err, g.close())
	}
	return openNativeReadOnlyGeneration(ctx, g, s.sid)
}

// Close the original Store, reacquire its actual lifetime lock and independently
// open the temporary native root/users/registration file objects. No retained
// original generation or baseline handle supplies reopen authority.
func restartNativeGenerationFixture(t *testing.T, s *Store, w *nativeBaselineViewFixtureStore) *Store {
	t.Helper()
	old := w.nativeStore
	if err := s.Close(); err != nil {
		t.Fatal(err)
	}
	n := &nativeStore{broker: old.broker, rootDOS: old.rootDOS}
	t.Cleanup(func() {
		if err := n.close(); err != nil {
			t.Error(err)
		}
	})
	sd, err := descriptor(n.broker, nil, true)
	if err != nil {
		t.Fatal(err)
	}
	open := func(name string) *nativePin {
		p, e := openPin(name, "", sd)
		if e != nil {
			t.Fatal(e)
		}
		n.pins = append(n.pins, p)
		return p
	}
	n.root = open(n.rootDOS)
	n.root.dos = n.rootDOS
	n.users = open(filepath.Join(n.rootDOS, "users"))
	n.registrations = open(filepath.Join(n.rootDOS, "registrations"))
	fileSD, err := descriptor(n.broker, nil, false)
	if err != nil {
		t.Fatal(err)
	}
	n.lock, err = acquireStoreLock(n.root, fileSD)
	if err != nil {
		t.Fatal(err)
	}
	wrapper := &nativeGenerationReopenFixtureStore{nativeBaselineViewFixtureStore: &nativeBaselineViewFixtureStore{nativeGenerationFixtureStore: &nativeGenerationFixtureStore{nativeBaselineFixtureStore: &nativeBaselineFixtureStore{nativeStore: n, sid: w.sid}, candidates: w.candidates, applications: w.applications}}}
	result := &Store{backend: wrapper}
	t.Cleanup(func() {
		if err := result.Close(); err != nil {
			t.Error(err)
		}
	})
	return result
}
func TestNativeGenerationReopenAfterStoreCloseRetainsBothPurposes(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	s, p, w, f, raw := nativeBaselineViewFixture(t)
	baseline, err := s.MaterializeApprovedBaselineReadOnlyFresh(context.Background(), p, raw, registrationDigest(raw))
	if err != nil {
		t.Fatal(err)
	}
	candidate, err := s.MaterializeCandidateReadOnlyFresh(context.Background(), p, nativeHistoryCandidate(t, s, p, f, strings.Repeat("9", 32)))
	if err != nil {
		t.Fatal(err)
	}
	baselineBefore, err := baseline.Completion()
	if err != nil {
		t.Fatal(err)
	}
	candidateBefore, err := candidate.Completion()
	if err != nil {
		t.Fatal(err)
	}
	client := p.record.ClientSID
	reopenedStore := restartNativeGenerationFixture(t, s, w)
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	initial, err := reopenedStore.OpenReadOnlyGeneration(ctx, client, stageA)
	if err != nil {
		t.Fatal("completed initial baseline reopen", err)
	}
	update, err := reopenedStore.OpenReadOnlyGeneration(ctx, client, strings.Repeat("9", 32))
	if err != nil {
		t.Fatal("completed signed candidate reopen", err)
	}
	if initial.Purpose() != GenerationPurposeApprovedBaseline || update.Purpose() != GenerationPurposeSignedCandidate {
		t.Fatal("reopened purpose changed")
	}
	initialView, err := initial.ApprovedBaseline()
	if err != nil {
		t.Fatal(err)
	}
	updateView, err := update.Candidate()
	if err != nil {
		t.Fatal(err)
	}
	if got, e := initialView.Completion(); e != nil || got != baselineBefore {
		t.Fatal("baseline provenance changed", e)
	}
	if got, e := updateView.Completion(); e != nil || got != candidateBefore {
		t.Fatal("candidate provenance changed", e)
	}
	if _, e := initial.Candidate(); e == nil {
		t.Fatal("approved baseline became signed candidate")
	}
	if _, e := update.ApprovedBaseline(); e == nil {
		t.Fatal("signed candidate became baseline")
	}
	member, err := initial.OpenMember("app/opencpn.exe")
	if err != nil {
		t.Fatal(err)
	}
	for _, owner := range []*ReopenedGeneration{initial, update} {
		if err = owner.Validate(); err != nil {
			t.Fatal(err)
		}
		native := owner.owner.generation.backend.(*nativeReopenedGeneration)
		for name, file := range native.members {
			if e := nativeAttemptFile(t, physical(file.pin.name), windows.GENERIC_READ|windows.GENERIC_EXECUTE); e != nil {
				t.Fatal("reopened loader read refused", name, e)
			}
			for _, access := range []uint32{windows.GENERIC_WRITE, windows.DELETE} {
				if e := nativeAttemptFile(t, physical(file.pin.name), access); !errors.Is(e, windows.ERROR_SHARING_VIOLATION) {
					t.Fatal("reopened write/delete custody lost", name, e)
				}
			}
		}
	}
	// Fresh append context must work after the original reopen context expires.
	cancel()
	original, err := reopenedStore.OpenInstalledBaseline(context.Background(), client, stageA, registrationDigest(raw), f.expected)
	if err != nil {
		t.Fatal(err)
	}
	later, err := reopenedStore.MaterializeCandidateReadOnlyFresh(context.Background(), original, nativeHistoryCandidate(t, reopenedStore, original, f, strings.Repeat("8", 32)))
	if err != nil {
		t.Fatal("reopened live history append", err)
	}
	for _, owner := range []*ReopenedGeneration{initial, update} {
		if err = owner.Validate(); err != nil {
			t.Fatal("append invalidated reopened owner", err)
		}
	}
	if err = later.Close(); err != nil {
		t.Fatal(err)
	}
	if err = update.Close(); err != nil {
		t.Fatal(err)
	}
	got := make([]byte, len(f.payload["app/opencpn.exe"]))
	if _, err = member.ReadAt(got, 0); err != nil || !bytes.Equal(got, f.payload["app/opencpn.exe"]) {
		t.Fatal("other Close revoked independent original", err)
	}
	if err = reopenedStore.Close(); err != nil {
		t.Fatal(err)
	}
	if _, err = member.ReadAt(got, 0); !errors.Is(err, ErrClosed) {
		t.Fatal("Store close did not revoke reopened member", err)
	}
}
func TestNativeGenerationReopenRejectsOriginalTamperingAndNamespaceChanges(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	for _, scenario := range []string{"missing-completion", "incomplete-marker", "private-record-bytes", "member-bytes", "member-physical-id", "parent-physical-id", "private-extra", "public-extra", "member-extra", "helper-hardlink", "case-alias"} {
		t.Run(scenario, func(t *testing.T) {
			s, p, w, f, raw := nativeBaselineViewFixture(t)
			owner, err := s.MaterializeApprovedBaselineReadOnlyFresh(context.Background(), p, raw, registrationDigest(raw))
			if err != nil {
				t.Fatal(err)
			}
			n := nativeHistoryOwner(t, owner.owner)
			registry, root, parents := physical(n.registry.name), physical(n.tree.directories[""].name), physical(n.store.registrations.name)
			completion := bytes.Clone(owner.owner.generation.raw)
			client := p.record.ClientSID
			reopenedStore := restartNativeGenerationFixture(t, s, w)
			recordPath := filepath.Join(registry, stageA+".json")
			markerPath := filepath.Join(registry, stageA+".once")
			switch scenario {
			case "missing-completion":
				err = os.Remove(recordPath)
			case "incomplete-marker":
				err = os.WriteFile(markerPath, generationMarker(client, stageA, n.origin, "", false), 0600)
			case "private-record-bytes":
				err = os.WriteFile(recordPath, bytes.Replace(completion, []byte(`"schema":1`), []byte(`"schema":2`), 1), 0600)
			case "member-bytes":
				err = os.WriteFile(filepath.Join(root, "app", "opencpn.exe"), bytes.Repeat([]byte("X"), len(f.payload["app/opencpn.exe"])), 0600)
			case "member-physical-id":
				path := filepath.Join(root, "app", "opencpn.exe")
				if err = os.Remove(path); err == nil {
					parent, e := openPin(filepath.Dir(path), "", n.tree.securityDir)
					if e != nil {
						t.Fatal(e)
					}
					replacement, e := baselineFile(parent, "opencpn.exe", n.tree.securityFile, true)
					if e != nil {
						windows.CloseHandle(parent.handle)
						t.Fatal(e)
					}
					if replacement.identity == owner.owner.generation.record.Members["app/opencpn.exe"].Identity {
						t.Fatal("replacement retained original physical ID")
					}
					_, err = replacement.Write(f.payload["app/opencpn.exe"])
					err = errors.Join(err, replacement.Close(), windows.CloseHandle(parent.handle))
				}
			case "parent-physical-id":
				var record generationParents
				path := filepath.Join(parents, client+".launch-parents.json")
				data, e := os.ReadFile(path)
				if e != nil {
					t.Fatal(e)
				}
				if e = json.Unmarshal(data, &record); e != nil {
					t.Fatal(e)
				}
				record.Directories["@generations"] = baselineTestID(999)
				data, e = json.Marshal(record)
				if e != nil {
					t.Fatal(e)
				}
				if err = os.WriteFile(path, data, 0600); err == nil {
					err = os.WriteFile(filepath.Join(parents, client+".launch-parents.once"), parentMarker(client, registrationDigest(data), true), 0600)
				}
			case "private-extra":
				err = os.WriteFile(filepath.Join(registry, "unknown.json"), []byte("{}"), 0600)
			case "public-extra":
				err = os.Mkdir(filepath.Join(filepath.Dir(root), strings.Repeat("7", 32)), 0700)
			case "member-extra":
				err = os.WriteFile(filepath.Join(root, "app", "extra"), []byte("extra"), 0600)
			case "helper-hardlink":
				err = os.Link(filepath.Join(root, "installer", "engine", "Lifecycle.ps1"), filepath.Join(reopenedStore.backend.(*nativeGenerationReopenFixtureStore).rootDOS, "helper-alias"))
			case "case-alias":
				err = os.Rename(filepath.Join(root, "app", "opencpn.exe"), filepath.Join(root, "app", "OPENCPN.exe"))
			}
			if err != nil {
				t.Fatal("actual fixture mutation failed", err)
			}
			got, err := reopenedStore.OpenReadOnlyGeneration(context.Background(), client, stageA)
			if err == nil || got != nil {
				t.Fatal("changed original accepted", scenario, err)
			}
			// Failed existing-only opens release all private/payload file objects.
			if scenario != "helper-hardlink" {
				if e := nativeAttemptFile(t, filepath.Join(root, "app", "opencpn.exe"), windows.GENERIC_WRITE); e != nil {
					t.Fatal("failed reopen leaked member custody", e)
				}
			}
		})
	}
}
func TestNativeGenerationReopenMissingOrCancelledNeverCreatesEvidence(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	for _, scenario := range []string{"absent-parents", "absent-generation", "cancelled"} {
		t.Run(scenario, func(t *testing.T) {
			s, p, w, _, raw := nativeBaselineViewFixture(t)
			client := p.record.ClientSID
			if scenario == "absent-generation" {
				owner, e := s.MaterializeApprovedBaselineReadOnlyFresh(context.Background(), p, raw, registrationDigest(raw))
				if e != nil {
					t.Fatal(e)
				}
				if e = owner.Close(); e != nil {
					t.Fatal(e)
				}
			}
			if err := p.Close(); err != nil {
				t.Fatal(err)
			}
			s.backend = &nativeGenerationReopenFixtureStore{nativeBaselineViewFixtureStore: w}
			ctx := context.Background()
			if scenario == "cancelled" {
				var cancel context.CancelFunc
				ctx, cancel = context.WithCancel(ctx)
				cancel()
			}
			stage := strings.Repeat("9", 32)
			if _, err := s.OpenReadOnlyGeneration(ctx, client, stage); err == nil {
				t.Fatal("absent/cancelled generation accepted")
			}
			if scenario == "absent-generation" {
				for _, suffix := range []string{".once", ".json"} {
					if _, err := os.Stat(filepath.Join(w.rootDOS, "registrations", client+".generations", stage+suffix)); !errors.Is(err, os.ErrNotExist) {
						t.Fatal("reopen reserved missing generation", err)
					}
				}
			} else {
				for _, name := range []string{client + ".launch-parents.once", client + ".launch-parents.json", client + ".generations"} {
					if _, err := os.Stat(filepath.Join(w.rootDOS, "registrations", name)); !errors.Is(err, os.ErrNotExist) {
						t.Fatal("reopen created evidence", name, err)
					}
				}
			}
		})
	}
}
