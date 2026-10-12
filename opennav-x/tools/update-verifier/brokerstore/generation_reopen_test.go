package brokerstore

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"strings"
	"sync"
	"testing"
)

type memoryReopenStore struct {
	storeBackend
	b        *memoryReopenedGeneration
	evidence generationReopenEvidence
	calls    int
	fault    error
}

func (s *memoryReopenStore) openReadOnlyGeneration(context.Context, string, string) (generationReopenBackend, generationReopenEvidence, error) {
	s.calls++
	if s.fault != nil {
		return nil, generationReopenEvidence{}, s.fault
	}
	return s.b, s.evidence, nil
}

type memoryReopenedGeneration struct {
	*memoryGeneration
	originals            map[string][]byte
	record               generationRecord
	openFault, rawWriter bool
	cancel               context.CancelFunc
}

func (b *memoryReopenedGeneration) openReadOnlyMembers(ctx context.Context, m map[string]baselineMember, _ map[string]diskIdentity) (map[string]candidateFile, error) {
	result := map[string]candidateFile{}
	for name := range m {
		f := &memoryFile{data: bytes.Clone(b.originals[name])}
		b.files[name] = f
		if b.rawWriter {
			result[name] = f
		} else {
			result[name] = &generationReadOnlyFile{candidateFile: f, reader: memoryMemberReader{f}}
		}
		if b.openFault {
			return result, testFault
		}
	}
	if b.cancel != nil {
		b.cancel()
		return result, ctx.Err()
	}
	return result, nil
}
func (b *memoryReopenedGeneration) identities(m map[string]baselineMember) (map[string]diskIdentity, map[string]diskIdentity, error) {
	files := map[string]diskIdentity{}
	for name, v := range m {
		files[name] = v.Identity
	}
	return b.record.Directories, files, nil
}
func reopenFixture(t *testing.T, purpose GenerationPurpose) (*Store, *memoryReopenStore, baselineFixtureData) {
	t.Helper()
	s, p, b, f, raw := baselineViewFixture(t)
	owner, err := s.MaterializeApprovedBaselineFresh(context.Background(), p, raw, registrationDigest(raw))
	if err != nil {
		t.Fatal(err)
	}
	record := owner.generation.record
	originals := map[string][]byte{}
	for name, file := range b.files {
		originals[name] = bytes.Clone(file.data)
	}
	lineage := *record.ApprovedBaseline
	baselineRaw := bytes.Clone(record.PreviousCompletion)
	if purpose == GenerationPurposeSignedCandidate {
		record.Purpose = string(purpose)
		record.ApprovedBaseline = nil
		record.Generation = strings.Repeat("9", 32)
		record.Acquisition = acquisitionPlan()
		record.Acquisition.Version = record.Identity.Version
		record.Acquisition.Commit = record.Identity.Commit
	}
	record.Directories = map[string]diskIdentity{"": baselineTestID(190)}
	for name := range record.Members {
		parent, _ := splitReopenName(name)
		for parent != "" {
			record.Directories[parent] = baselineTestID(191 + len(record.Directories))
			parent, _ = splitReopenName(parent)
		}
	}
	completion, err := json.Marshal(record)
	if err != nil {
		t.Fatal(err)
	}
	if err = owner.Close(); err != nil {
		t.Fatal(err)
	}
	if err = p.Close(); err != nil {
		t.Fatal(err)
	}
	backend := &memoryReopenedGeneration{memoryGeneration: &memoryGeneration{memoryCandidate: &memoryCandidate{files: map[string]*memoryFile{}}, completed: true}, originals: originals, record: record}
	wrapper := &memoryReopenStore{storeBackend: s.backend, b: backend, evidence: generationReopenEvidence{completion: completion, baselineCompletion: baselineRaw, lineage: lineage}}
	s.backend = wrapper
	return s, wrapper, f
}
func TestGenerationReopenRetainsTypedOriginalBytesAndRevocation(t *testing.T) {
	for _, purpose := range []GenerationPurpose{GenerationPurposeSignedCandidate, GenerationPurposeApprovedBaseline} {
		t.Run(string(purpose), func(t *testing.T) {
			s, b, f := reopenFixture(t, purpose)
			owner, err := s.OpenReadOnlyGeneration(context.Background(), registrationTestSID, b.b.record.Generation)
			if err != nil {
				t.Fatal(err)
			}
			if owner.Purpose() != purpose {
				t.Fatal("purpose changed")
			}
			if purpose == GenerationPurposeSignedCandidate {
				candidate, e := owner.Candidate()
				if e != nil {
					t.Fatal(e)
				}
				if _, e = candidate.Completion(); e != nil {
					t.Fatal(e)
				}
				if _, e = owner.ApprovedBaseline(); e == nil {
					t.Fatal("candidate presented as baseline")
				}
			} else {
				baseline, e := owner.ApprovedBaseline()
				if e != nil {
					t.Fatal(e)
				}
				if _, e = baseline.Completion(); e != nil {
					t.Fatal(e)
				}
				if _, e = owner.Candidate(); e == nil {
					t.Fatal("baseline presented as candidate")
				}
			}
			b.evidence.completion[0] ^= 1
			if err = owner.Validate(); err != nil {
				t.Fatal("transport bytes not owned", err)
			}
			h, err := owner.OpenMember("app/opencpn.exe")
			if err != nil {
				t.Fatal(err)
			}
			got := make([]byte, len(f.payload["app/opencpn.exe"]))
			if _, err = h.ReadAt(got, 0); err != nil || !bytes.Equal(got, f.payload["app/opencpn.exe"]) {
				t.Fatal("original read differs", err)
			}
			if err = s.Close(); err != nil {
				t.Fatal(err)
			}
			if _, err = h.ReadAt(got, 0); !errors.Is(err, ErrClosed) {
				t.Fatal("store close did not revoke", err)
			}
			if _, err = owner.Candidate(); !errors.Is(err, ErrClosed) {
				t.Fatal("typed closed owner survived", err)
			}
			for _, file := range b.b.files {
				if file.closes != 1 {
					t.Fatal("member not drained once", file.closes)
				}
			}
			if b.b.closes != 1 {
				t.Fatal("backend not drained once")
			}
		})
	}
}
func TestGenerationReopenRefusesMalformedOriginalEvidence(t *testing.T) {
	for _, scenario := range []string{"unknown-field", "duplicate-field", "case-field", "different-client", "different-generation", "unapproved-lineage", "different-baseline", "original-registration-id", "mixed-purpose", "candidate-identity", "extra-dir", "case-directory", "missing-dir", "missing-source", "portable-marker", "path-alias", "member-overflow", "member-collision"} {
		t.Run(scenario, func(t *testing.T) {
			s, b, _ := reopenFixture(t, GenerationPurposeSignedCandidate)
			r := b.b.record
			switch scenario {
			case "unknown-field":
				b.evidence.completion = append([]byte(`{"unknown":1,`), b.evidence.completion[1:]...)
			case "duplicate-field":
				b.evidence.completion = append([]byte(`{"schema":1,`), b.evidence.completion[1:]...)
			case "case-field":
				b.evidence.completion = bytes.Replace(b.evidence.completion, []byte(`"schema"`), []byte(`"Schema"`), 1)
			case "different-client":
				r.ClientSID = "S-1-5-21-1-2-3-500"
			case "different-generation":
				r.Generation = strings.Repeat("8", 32)
			case "unapproved-lineage":
				b.evidence.lineage.ApprovedRequestSHA256 = strings.Repeat("a", 64)
			case "different-baseline":
				b.evidence.baselineCompletion = append(b.evidence.baselineCompletion, ' ')
			case "original-registration-id":
				b.evidence.lineage.RegistrationIdentity = diskIdentity{}
			case "mixed-purpose":
				r.ApprovedBaseline = &b.evidence.lineage
			case "candidate-identity":
				r.Acquisition.Version = "3.0.0"
			case "extra-dir":
				r.Directories["app/extra"] = baselineTestID(900)
			case "case-directory":
				r.Directories["App"] = r.Directories["app"]
			case "missing-dir":
				delete(r.Directories, "app")
			case "missing-source":
				delete(r.SourceMembers, "app/opencpn.exe")
			case "portable-marker":
				r.Members["app/OPENNAV_PORTABLE_PREVIEW"] = r.Members["app/opencpn.exe"]
				r.SourceMembers["app/OPENNAV_PORTABLE_PREVIEW"] = baselineTestID(999)
			case "path-alias":
				r.Members["app/../evil"] = r.Members["app/opencpn.exe"]
				r.SourceMembers["app/../evil"] = baselineTestID(999)
			case "member-overflow":
				m := r.Members["app/opencpn.exe"]
				m.Length = MaxFileBytes + 1
				r.Members["app/opencpn.exe"] = m
			case "member-collision":
				r.Members["app/OPENCPN.exe"] = r.Members["app/opencpn.exe"]
				r.SourceMembers["app/OPENCPN.exe"] = baselineTestID(999)
			}
			if scenario != "unknown-field" && scenario != "duplicate-field" && scenario != "case-field" {
				b.evidence.completion, _ = json.Marshal(r)
			}
			if owner, err := s.OpenReadOnlyGeneration(context.Background(), registrationTestSID, b.b.record.Generation); err == nil || owner != nil {
				t.Fatal("malformed original accepted", err)
			}
			if b.b.closes != 1 || len(b.b.files) != 0 {
				t.Fatal("invalid evidence opened payload or leaked owner")
			}
		})
	}
}
func TestGenerationReopenFailureDrainsPartialReadersAndNeverReserves(t *testing.T) {
	for _, scenario := range []string{"factory", "partial-open", "cancel", "raw-writer", "changed-payload", "cleanup"} {
		t.Run(scenario, func(t *testing.T) {
			s, b, _ := reopenFixture(t, GenerationPurposeSignedCandidate)
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			switch scenario {
			case "factory":
				b.fault = testFault
			case "partial-open":
				b.b.openFault = true
			case "cancel":
				b.b.cancel = cancel
			case "raw-writer":
				b.b.rawWriter = true
			case "changed-payload":
				b.b.originals["app/opencpn.exe"][0] ^= 1
			case "cleanup":
				b.b.openFault = true
				b.b.closeFault = testFault
			}
			owner, err := s.OpenReadOnlyGeneration(ctx, registrationTestSID, b.b.record.Generation)
			if err == nil || owner != nil {
				t.Fatal("failure accepted")
			}
			if scenario == "factory" {
				if b.b.closes != 0 {
					t.Fatal("unreturned factory owner closed")
				}
				return
			}
			if b.b.closes != 1 {
				t.Fatal("backend not drained once")
			}
			for _, f := range b.b.files {
				if f.closes != 1 {
					t.Fatal("partial reader leak", f.closes)
				}
			}
			if b.b.reserved || b.b.tree || len(s.generations) != 1 {
				t.Fatal("reopening mutated reservation or published failed owner")
			}
			if scenario == "cleanup" && (!errors.Is(err, ErrInstalledGenerationUncertain) || !errors.Is(err, testFault)) {
				t.Fatal("cleanup uncertainty lost", err)
			}
		})
	}
}
func TestGenerationReopenRefusesLiveWriterPoisonAndBadArgumentsBeforeFactory(t *testing.T) {
	for _, scenario := range []string{"writer", "poison", "nil-context", "cancelled-context", "bad-client", "bad-stage", "closed"} {
		t.Run(scenario, func(t *testing.T) {
			s, b, _ := reopenFixture(t, GenerationPurposeSignedCandidate)
			ctx := context.Background()
			client, stage := registrationTestSID, b.b.record.Generation
			switch scenario {
			case "writer":
				s.generations[0].closed = false
			case "poison":
				s.generations[0].failure = testFault
			case "nil-context":
				ctx = nil
			case "cancelled-context":
				var cancel context.CancelFunc
				ctx, cancel = context.WithCancel(ctx)
				cancel()
			case "bad-client":
				client = "S-1-5-18"
			case "bad-stage":
				stage = "../x"
			case "closed":
				s.closed.Store(true)
			}
			if _, err := s.OpenReadOnlyGeneration(ctx, client, stage); err == nil {
				t.Fatal("invalid owner/arguments accepted")
			}
			if b.calls != 0 {
				t.Fatal("refusal reached native factory")
			}
			s.generations[0].closed = true
			s.generations[0].failure = nil
			s.closed.Store(false)
		})
	}
}
func TestGenerationReopenConcurrentViewsSerializeWithClose(t *testing.T) {
	s, b, _ := reopenFixture(t, GenerationPurposeSignedCandidate)
	owner, err := s.OpenReadOnlyGeneration(context.Background(), registrationTestSID, b.b.record.Generation)
	if err != nil {
		t.Fatal(err)
	}
	var wg sync.WaitGroup
	for i := 0; i < 8; i++ {
		wg.Add(1)
		go func() {
			defer wg.Done()
			for j := 0; j < 10; j++ {
				h, e := owner.OpenMember("app/opencpn.exe")
				if e == nil {
					_, _ = h.ReadAt(make([]byte, 1), 0)
					h.Close()
				}
				_ = owner.Validate()
				_, _ = owner.Candidate()
			}
		}()
	}
	if err = owner.Close(); err != nil {
		t.Fatal(err)
	}
	wg.Wait()
	for _, f := range b.b.files {
		if f.closes != 1 {
			t.Fatal("concurrent close leaked")
		}
	}
}
