package brokerstore

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"sort"
	"strings"
	"testing"
)

// Portable native seams exercise real copy/hash/reopen logic. They do not prove
// Windows physical IDs, ownership descriptors or customer/native acceptance.
type memoryRepairStore struct {
	*memoryReopenStore
	source                  *memoryRepairSource
	current, original       generationRecord
	currentRaw, originalRaw []byte
}

func (b *memoryRepairStore) openRepairSource(context.Context, string, string) (repairSourceBackend, generationRecord, []byte, generationRecord, []byte, error) {
	return b.source, b.current, b.currentRaw, b.original, b.originalRaw, nil
}

type memoryRepairSource struct {
	members         map[string]candidateFile
	destination     *memoryRepairGeneration
	failure         error
	creates, closes int
}

func (b *memoryRepairSource) check() error                    { return b.failure }
func (b *memoryRepairSource) files() map[string]candidateFile { return b.members }
func (b *memoryRepairSource) create(context.Context, string) (generationBackend, error) {
	b.creates++
	return b.destination, nil
}
func (b *memoryRepairSource) close() error { b.closes++; return nil }

type memoryRepairGeneration struct{ *memoryGeneration }

func (b *memoryRepairGeneration) identities(m map[string]baselineMember) (map[string]diskIdentity, map[string]diskIdentity, error) {
	dirs, files, err := b.memoryGeneration.identities(m)
	if err != nil {
		return nil, nil, err
	}
	names := []string{}
	for name := range m {
		names = append(names, name)
	}
	sort.Strings(names)
	for _, name := range names {
		parent, _ := splitReopenName(name)
		for parent != "" {
			if _, ok := dirs[parent]; !ok {
				dirs[parent] = baselineTestID(1000 + len(dirs))
			}
			parent, _ = splitReopenName(parent)
		}
	}
	return dirs, files, nil
}
func repairFixture(t *testing.T, purpose GenerationPurpose) (*Store, *memoryRepairStore) {
	t.Helper()
	s, previous, _ := reopenFixture(t, purpose)
	original := previous.b.record
	source := &memoryRepairSource{members: map[string]candidateFile{}, destination: &memoryRepairGeneration{&memoryGeneration{memoryCandidate: &memoryCandidate{files: map[string]*memoryFile{}}}}}
	for name, raw := range previous.b.originals {
		source.members[name] = &memoryFile{data: bytes.Clone(raw)}
	}
	b := &memoryRepairStore{memoryReopenStore: previous, source: source, current: original, original: original, currentRaw: bytes.Clone(previous.evidence.completion), originalRaw: bytes.Clone(previous.evidence.completion)}
	s.backend = b
	return s, b
}
func TestRepairRestoresExactSelectedBytesAfterDamageWithoutAdoptingOrOverwriting(t *testing.T) {
	for _, purpose := range []GenerationPurpose{GenerationPurposeApprovedBaseline, GenerationPurposeSignedCandidate} {
		t.Run(string(purpose), func(t *testing.T) {
			s, b := repairFixture(t, purpose)
			ctx := context.Background()
			// Corrupt one public member, delete another. Private source remains original.
			b.b.originals["app/opencpn.exe"] = []byte("damaged")
			delete(b.b.originals, "docs/PRODUCT_BUILD.json")
			unknown := []byte("customer addition stays in damaged original tree")
			b.b.originals["app/customer-notes.txt"] = bytes.Clone(unknown)
			if _, err := s.OpenReadOnlyGeneration(ctx, registrationTestSID, b.current.Generation); err == nil {
				t.Fatal("ordinary damaged generation accepted")
			}
			// The original native seam remains metadata-only; public reopening stays strict.
			source, err := s.OpenRepairSource(ctx, registrationTestSID, b.current.Generation)
			if err != nil {
				t.Fatal(err)
			}
			gen, err := s.MaterializeRepairFresh(ctx, source, strings.Repeat("4", 32), strings.Repeat("5", 64))
			if err != nil {
				t.Fatal(err)
			}
			c, err := gen.Completion()
			if err != nil || c.Identity != b.original.Identity || c.Generation == b.original.Generation {
				t.Fatal("repair changed code identity", err)
			}
			if gen.record.Purpose != string(GenerationPurposeRepaired) || gen.record.Repair == nil || gen.record.Repair.SourceCompletionSHA256 != registrationDigest(b.originalRaw) {
				t.Fatal("missing explicit repair provenance")
			}
			if !bytes.Equal(b.b.originals["app/customer-notes.txt"], unknown) || string(b.b.originals["app/opencpn.exe"]) != "damaged" {
				t.Fatal("original tree changed")
			}
			for name, m := range b.original.Members {
				raw := b.source.destination.files[name].data
				if int64(len(raw)) != m.Length || registrationDigest(raw) != m.SHA256 {
					t.Fatal("restored bytes differ", name)
				}
			}
			if len(b.source.destination.files) != len(b.original.Members) {
				t.Fatal("adopted unknown public additions")
			}
			// Fresh strict public reopening now verifies ALL restored members and IDs.
			restored := map[string][]byte{}
			for name, f := range b.source.destination.files {
				restored[name] = bytes.Clone(f.data)
			}
			record := gen.record
			raw := bytes.Clone(gen.raw)
			if err = gen.Close(); err != nil {
				t.Fatal(err)
			}
			b.memoryReopenStore.b = &memoryReopenedGeneration{memoryGeneration: &memoryGeneration{memoryCandidate: &memoryCandidate{files: map[string]*memoryFile{}}, completed: true}, originals: restored, record: record}
			b.memoryReopenStore.evidence.completion = raw
			reopened, err := s.OpenReadOnlyGeneration(ctx, registrationTestSID, record.Generation)
			if err != nil {
				t.Fatal(err)
			}
			if _, err = reopened.Candidate(); err == nil {
				t.Fatal("repair laundered as signed candidate")
			}
			if _, err = reopened.ApprovedBaseline(); err == nil {
				t.Fatal("repair laundered as approved original baseline")
			}
			if _, err = reopened.Repaired(); err != nil {
				t.Fatal(err)
			}
			origin, err := reopened.RepairOrigin(ctx)
			if err != nil || !bytes.Equal(origin.SourceCompletion, b.originalRaw) {
				t.Fatal("source identity lost", err)
			}
			if err = reopened.Close(); err != nil {
				t.Fatal(err)
			}
			if _, err = s.MaterializeRepairFresh(ctx, source, strings.Repeat("6", 32), strings.Repeat("5", 64)); err == nil || b.source.creates != 1 {
				t.Fatal("one source retried")
			}
		})
	}
}
func TestRepairRejectsSourceTamperingBeforeReservationAndPreservesFailedAttempt(t *testing.T) {
	for _, fault := range []string{"source-bytes", "source-missing", "source-completion", "source-lineage", "native-custody", "copy", "publish", "late-source"} {
		t.Run(fault, func(t *testing.T) {
			s, b := repairFixture(t, GenerationPurposeSignedCandidate)
			ctx := context.Background()
			switch fault {
			case "source-bytes":
				b.source.members["app/opencpn.exe"].(*memoryFile).data[0] ^= 1
			case "source-missing":
				delete(b.source.members, "app/opencpn.exe")
			case "source-completion":
				b.originalRaw = append(b.originalRaw, ' ')
			case "source-lineage":
				b.current.Identity.Commit = strings.Repeat("f", 40)
			case "native-custody":
				b.source.failure = testFault
			}
			source, err := s.OpenRepairSource(ctx, registrationTestSID, b.current.Generation)
			if strings.HasPrefix(fault, "source-") || fault == "native-custody" {
				if err == nil || b.source.creates != 0 {
					t.Fatal("invalid source created namespace")
				}
				return
			}
			if err != nil {
				t.Fatal(err)
			}
			if fault == "copy" || fault == "publish" {
				b.source.destination.faultAt = fault
			}
			if fault == "late-source" {
				b.source.destination.onReserve = func() { b.source.members["app/opencpn.exe"].(*memoryFile).data[0] ^= 1 }
			}
			if _, err = s.MaterializeRepairFresh(ctx, source, strings.Repeat("4", 32), strings.Repeat("5", 64)); err == nil {
				t.Fatal("failed transaction succeeded")
			}
			if !b.source.destination.reserved {
				t.Fatal("failure lost permanent reservation")
			}
			if _, err = s.MaterializeRepairFresh(ctx, source, strings.Repeat("6", 32), strings.Repeat("5", 64)); err == nil || b.source.creates != 1 {
				t.Fatal("failed source retried")
			}
		})
	}
}
func TestRepairProvenanceCannotChangeCodeOrNestHistoricalRepair(t *testing.T) {
	s, b := repairFixture(t, GenerationPurposeApprovedBaseline)
	source, err := s.OpenRepairSource(context.Background(), registrationTestSID, b.current.Generation)
	if err != nil {
		t.Fatal(err)
	}
	gen, err := s.MaterializeRepairFresh(context.Background(), source, strings.Repeat("4", 32), strings.Repeat("5", 64))
	if err != nil {
		t.Fatal(err)
	}
	for _, fault := range []string{"code", "member", "source", "predecessor", "nested"} {
		t.Run(fault, func(t *testing.T) {
			raw, _ := json.Marshal(gen.record)
			var changed generationRecord
			json.Unmarshal(raw, &changed)
			switch fault {
			case "code":
				changed.Identity.Commit = strings.Repeat("f", 40)
			case "member":
				m := changed.Members["app/opencpn.exe"]
				m.SHA256 = strings.Repeat("f", 64)
				changed.Members["app/opencpn.exe"] = m
			case "source":
				changed.SourceMembers["app/opencpn.exe"] = baselineTestID(9999)
			case "predecessor":
				changed.Repair.PreviousGeneration = changed.Generation
			case "nested":
				changed.Repair.SourceCompletion = raw
				changed.Repair.SourceCompletionSHA256 = registrationDigest(raw)
			}
			if !errors.Is(validateRepairOrigin(changed), ErrInstalledGenerationUncertain) {
				t.Fatal("invalid repair provenance accepted")
			}
		})
	}
}

type memoryBootstrapRepairSource struct {
	*memoryRepairSource
	public            []byte
	member            baselineMember
	prepared, matches int
}

func (b *memoryBootstrapRepairSource) prepareBaselineExecutor(ctx context.Context, s *Store, m baselineMember, _ map[string]diskIdentity) error {
	b.prepared++
	b.member = m
	return verifyFile(ctx, s, &memoryFile{data: bytes.Clone(b.public)}, m.Length, m.SHA256)
}
func (b *memoryBootstrapRepairSource) matchBaselineExecutor(ctx context.Context, s *Store, _ uint32, _ uint64) error {
	b.matches++
	return verifyFile(ctx, s, &memoryFile{data: bytes.Clone(b.public)}, b.member.Length, b.member.SHA256)
}
func TestRepairBootstrapRequiresOriginalBaselineLauncherAndRetainedSourceCustody(t *testing.T) {
	for _, fault := range []string{"", "signed-source", "damaged-launcher", "changed-launcher", "closed-source", "changed-private-source"} {
		t.Run(fault, func(t *testing.T) {
			purpose := GenerationPurposeApprovedBaseline
			if fault == "signed-source" {
				purpose = GenerationPurposeSignedCandidate
			}
			s, b := repairFixture(t, purpose)
			ctx := context.Background()
			source, err := s.OpenRepairSource(ctx, registrationTestSID, b.current.Generation)
			if err != nil {
				t.Fatal(err)
			}
			native := &memoryBootstrapRepairSource{memoryRepairSource: b.source, public: bytes.Clone(b.b.originals[generationExecutorMember])}
			source.backend = native
			if fault == "damaged-launcher" {
				native.public[0] ^= 1
			}
			image, err := source.OriginalBaselineExecutorImage(ctx)
			if fault == "signed-source" || fault == "damaged-launcher" {
				if err == nil {
					t.Fatal("non-original/damaged bootstrap admitted")
				}
				return
			}
			if err != nil {
				t.Fatal(err)
			}
			switch fault {
			case "changed-launcher":
				native.public[0] ^= 1
			case "closed-source":
				source.Close()
			case "changed-private-source":
				b.source.members["app/opencpn.exe"].(*memoryFile).data[0] ^= 1
			}
			err = image.MatchesProcess(ctx, 17, 23)
			if (fault == "") != (err == nil) {
				t.Fatal("bootstrap custody boundary incorrect", err)
			}
			if fault == "" && native.matches != 1 {
				t.Fatal("did not require original process match")
			}
			if fault == "closed-source" && native.matches != 0 {
				t.Fatal("reopened released source authority")
			}
		})
	}
}

func TestCompletedRepairWithoutCommittedJournalBlocksFreshAttempt(t *testing.T) {
	s, b := repairFixture(t, GenerationPurposeSignedCandidate)
	source, err := s.OpenRepairSource(context.Background(), registrationTestSID, b.current.Generation)
	if err != nil {
		t.Fatal(err)
	}
	gen, err := s.MaterializeRepairFresh(context.Background(), source, strings.Repeat("4", 32), strings.Repeat("5", 64))
	if err != nil {
		t.Fatal(err)
	}
	originals := [][]byte{b.originalRaw, gen.raw}
	if err = validateRepairHistoryRecords(originals, map[string]string{}); err == nil {
		t.Fatal("completed orphan repair silently admitted as retryable stage")
	}
	if err = validateRepairHistoryRecords(originals, map[string]string{gen.record.Generation: registrationDigest(gen.raw)}); err != nil {
		t.Fatal("actual completed repair history refused", err)
	}
	if err = validateRepairHistoryRecords(originals, map[string]string{gen.record.Generation: strings.Repeat("f", 64)}); err == nil {
		t.Fatal("mismatched repair history admitted")
	}
}
