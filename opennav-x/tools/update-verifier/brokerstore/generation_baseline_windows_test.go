package brokerstore

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"os"
	"runtime"
	"strings"
	"testing"

	"example.com/opennav-update-verifier/brokerregistration"
	"golang.org/x/sys/windows"
)

// This current-user temporary factory bypasses only production principal and
// fixed Program Files discovery. Real protected registration, original baseline,
// permanent reservations, source IDs, copying and readonly transition are used.
type nativeBaselineViewFixtureStore struct{ *nativeGenerationFixtureStore }

func (s *nativeBaselineViewFixtureStore) prepareBaselineGeneration(ctx context.Context, previous installedBaselineBackend, request brokerregistration.Request) (baselineGenerationBackend, error) {
	p, ok := previous.(*nativeInstalledBaseline)
	if !ok || p == nil {
		return nil, errors.New("native baseline required")
	}
	b, err := prepareNativeBaselineGeneration(ctx, s.nativeStore, p, s.sid, request)
	if err != nil {
		return nil, err
	}
	b.applications, err = openPin(physical(s.applications.name), s.applications.name, s.applications.security)
	if err != nil {
		return nil, errors.Join(err, b.close())
	}
	b.ancestors = append(b.ancestors, b.applications)
	if err = b.pin(b.applications); err != nil {
		return nil, errors.Join(err, b.close())
	}
	return b, nil
}
func nativeBaselineViewFixture(t *testing.T) (*Store, *InstalledBaseline, *nativeBaselineViewFixtureStore, baselineFixtureData, []byte) {
	t.Helper()
	n, registration := nativeRegistrationFixture(t)
	f := baselineEngineData(t)
	raw := baselineViewRequest(t, registration.client, f)
	pin := registrationDigest(raw)
	registration.approved = pin
	if err := beginRegistration(context.Background(), registration); err != nil {
		t.Fatal(err)
	}
	s := &Store{backend: &nativeBaselineFixtureStore{nativeStore: n, sid: n.broker}}
	previous, err := s.BeginInstalledBaselineFresh(context.Background(), registration.client, stageA, pin)
	if err != nil {
		t.Fatal(err)
	}
	writeBaseline(t, previous, f)
	if err = previous.Complete(context.Background(), f.identity); err != nil {
		t.Fatal(err)
	}
	createNativeTestChannel(t, registration)
	if err = registration.complete(context.Background(), baselineViewEnvelope(t, raw, registration.client)); err != nil {
		t.Fatal(err)
	}
	if err = registration.close(); err != nil {
		t.Fatal(err)
	}
	sd, err := descriptor(n.broker, nil, true)
	if err != nil {
		t.Fatal(err)
	}
	container := &nativeCandidate{store: n, securityDir: sd}
	candidates, err := container.newDirectory(n.root, "baseline-view-source-candidates")
	if err != nil {
		t.Fatal(err)
	}
	applications, err := container.newDirectory(n.root, "fixture-baseline-Applications")
	if err != nil {
		t.Fatal(err)
	}
	wrapper := &nativeBaselineViewFixtureStore{nativeGenerationFixtureStore: &nativeGenerationFixtureStore{nativeBaselineFixtureStore: s.backend.(*nativeBaselineFixtureStore), candidates: candidates, applications: applications}}
	s.backend = wrapper
	t.Cleanup(func() {
		s.Close()
		if err := container.close(); err != nil {
			t.Error(err)
		}
	})
	return s, previous, wrapper, f, raw
}
func TestNativeGenerationBaselineCopiesApprovedOriginalsToReadonlyPublicView(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	s, p, _, f, raw := nativeBaselineViewFixture(t)
	owner, err := s.MaterializeApprovedBaselineFresh(context.Background(), p, raw, registrationDigest(raw))
	if err != nil {
		t.Fatal(err)
	}
	n := owner.generation.backend.(*nativeBaselineGeneration)
	completion, err := owner.Completion()
	if err != nil || completion.Identity != f.identity || completion.Purpose != approvedBaselineGenerationPurpose || completion.Generation != stageA {
		t.Fatal("distinct initial baseline provenance differs", err)
	}
	originals := map[string][]byte{"FILE_SHA256.json": f.inventory, "installer/package.json": f.manifest}
	for name, raw := range f.payload {
		originals[name] = raw
	}
	sourceIDs := owner.generation.record.SourceMembers
	for name, want := range originals {
		file := n.members[name]
		if file.identity == sourceIDs[name] {
			t.Fatal("public view adopted private source file", name)
		}
		got, err := readBaselineFile(context.Background(), file, int64(len(want)))
		if err != nil || !bytes.Equal(got, want) {
			t.Fatal("copied original differs", name, err)
		}
	}
	r, err := owner.TransitionReadOnly(context.Background())
	if err != nil {
		t.Fatal(err)
	}
	if err = p.Close(); err != nil {
		t.Fatal(err)
	}
	if err = r.Validate(); err != nil {
		t.Fatal("public loader custody depends on private source owner", err)
	}
	after, err := r.Completion()
	if err != nil || after != completion {
		t.Fatal("readonly baseline provenance changed", err)
	}
	for name, want := range originals {
		h, err := r.OpenMember(name)
		if err != nil {
			t.Fatal(err)
		}
		got := make([]byte, len(want))
		if _, err = h.ReadAt(got, 0); err != nil || !bytes.Equal(got, want) {
			t.Fatal("owned original read differs", name, err)
		}
		h.Close()
		path := physical(n.members[name].pin.name)
		if err = nativeAttemptFile(t, path, windows.GENERIC_READ|windows.GENERIC_EXECUTE); err != nil {
			t.Fatal("readonly loader open refused", name, err)
		}
		for _, access := range []uint32{windows.GENERIC_WRITE, windows.DELETE} {
			if err = nativeAttemptFile(t, path, access); !errors.Is(err, windows.ERROR_SHARING_VIOLATION) {
				t.Fatal("readonly public copy lost custody", name, err)
			}
		}
	}
	if _, err = s.MaterializeApprovedBaselineFresh(context.Background(), p, raw, registrationDigest(raw)); err == nil {
		t.Fatal("closed source reused")
	}
	if err = r.Close(); err != nil {
		t.Fatal(err)
	}
}
func TestNativeGenerationBaselineAndCandidateRegistryCoexistWithStrictPurposes(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	s, p, w, f, raw := nativeBaselineViewFixture(t)
	baseline, err := s.MaterializeApprovedBaselineFresh(context.Background(), p, raw, registrationDigest(raw))
	if err != nil {
		t.Fatal(err)
	}
	before := baseline.generation.backend.(*nativeBaselineGeneration).parentIDs
	readonly, err := baseline.TransitionReadOnly(context.Background())
	if err != nil {
		t.Fatal(err)
	}
	// Source remains held; close only the public view before adding a sibling. The
	// separate history-evolution task must permit both owners to stay live.
	if err = readonly.Close(); err != nil {
		t.Fatal(err)
	}
	c, err := s.Begin(strings.Repeat("9", 32), p.record.ClientSID)
	if err != nil {
		t.Fatal(err)
	}
	acquireGenerationArtifacts(t, c, f)
	candidate, err := s.MaterializeCandidateFresh(context.Background(), p, c)
	if err != nil {
		t.Fatal("candidate refused authenticated approved-baseline sibling", err)
	}
	n := candidate.backend.(*nativeInstalledGeneration)
	if !sameGenerationIDs(before, n.parentIDs) || len(n.siblings) != 1 {
		t.Fatal("mixed-purpose registry did not preserve historical parent IDs")
	}
	// A new owner must still refuse the already used initial generation ID.
	if _, err = s.MaterializeApprovedBaselineFresh(context.Background(), p, raw, registrationDigest(raw)); err == nil {
		t.Fatal("public baseline was adopted/repaired")
	}
	entries, err := boundedEntries(n.registry, 8)
	if err != nil {
		t.Fatal(err)
	}
	if len(entries) != 4 {
		t.Fatal("failed retry changed permanent mixed-purpose registry", len(entries))
	}
	if err = candidate.Close(); err != nil {
		t.Fatal(err)
	}
	_ = w
}
func TestNativeGenerationBaselineRefusesRegistrationMutationAndPartialDestination(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	for _, scenario := range []string{"registration-bytes", "registration-marker", "partial-view", "unknown-purpose", "mixed-purpose"} {
		t.Run(scenario, func(t *testing.T) {
			s, p, w, f, raw := nativeBaselineViewFixture(t)
			if scenario == "registration-bytes" || scenario == "registration-marker" {
				component := p.record.ClientSID + ".json"
				if scenario == "registration-marker" {
					component = p.record.ClientSID + ".once"
				}
				file, err := openAcquisitionFile(w.registrations, component, p.backend.(*nativeInstalledBaseline).privateFileSD, windows.GENERIC_READ|windows.GENERIC_WRITE|windows.READ_CONTROL, 0, windows.OPEN_EXISTING)
				if err != nil {
					t.Fatal(err)
				}
				if _, err = file.File.WriteAt([]byte("X"), 0); err != nil {
					t.Fatal(err)
				}
				if err = file.Close(); err != nil {
					t.Fatal(err)
				}
			} else {
				owner, err := s.MaterializeApprovedBaselineFresh(context.Background(), p, raw, registrationDigest(raw))
				if err != nil {
					t.Fatal(err)
				}
				n := owner.generation.backend.(*nativeBaselineGeneration)
				record := physical(n.record.pin.name)
				marker := physical(n.marker.pin.name)
				if err = owner.Close(); err != nil {
					t.Fatal(err)
				}
				if scenario == "partial-view" {
					if err = os.Remove(record); err != nil {
						t.Fatal(err)
					}
				} else {
					data, err := os.ReadFile(record)
					if err != nil {
						t.Fatal(err)
					}
					var completed generationRecord
					if err = json.Unmarshal(data, &completed); err != nil {
						t.Fatal(err)
					}
					if scenario == "unknown-purpose" {
						completed.Purpose = "unknown-view"
					} else {
						completed.Acquisition = acquisitionPlan()
					}
					changed, _ := json.Marshal(completed)
					if err = os.WriteFile(record, changed, 0600); err != nil {
						t.Fatal(err)
					}
					origin := completed
					origin.Directories = nil
					origin.Members = map[string]baselineMember{}
					originRaw, _ := json.Marshal(origin)
					if err = os.WriteFile(marker, generationMarker(completed.ClientSID, completed.Generation, registrationDigest(originRaw), registrationDigest(changed), true), 0600); err != nil {
						t.Fatal(err)
					}
				}
				c, err := s.Begin(strings.Repeat("9", 32), p.record.ClientSID)
				if err != nil {
					t.Fatal(err)
				}
				acquireGenerationArtifacts(t, c, f)
				if _, err = s.MaterializeCandidateFresh(context.Background(), p, c); err == nil {
					t.Fatal("candidate adopted malformed baseline registry")
				}
				return
			}
			if _, err := s.MaterializeApprovedBaselineFresh(context.Background(), p, raw, registrationDigest(raw)); err == nil {
				t.Fatal("mutated protected registration authorized public view")
			}
			if _, err := os.Stat(physical(w.applications.name) + `\` + p.record.ClientSID); !errors.Is(err, os.ErrNotExist) {
				t.Fatal("refusal exposed new namespace", err)
			}
		})
	}
}
