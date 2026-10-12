package brokerstore

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"io"
	"strings"
	"testing"

	"example.com/opennav-update-verifier/baselineevidence"
)

type memoryApplicationBackend struct {
	*memoryExecutorBackend
	applications int
}

func (b *memoryApplicationBackend) matchApplicationProcess(ctx context.Context, pid uint32, created uint64) error {
	b.applications++
	return b.matchExecutorProcess(ctx, pid, created)
}
func applicationFixture(t *testing.T) (*Store, *ReadOnlyGeneration, *memoryApplicationBackend, *GenerationApplicationImage) {
	t.Helper()
	s, r, b := executorFixture(t)
	backend := &memoryApplicationBackend{memoryExecutorBackend: b}
	r.generation.backend = backend
	image, err := r.ApplicationImage(context.Background())
	if err != nil {
		t.Fatal(err)
	}
	return s, r, backend, image
}
func TestGenerationApplicationRequiresLiveFixedOriginalAndNativeProcess(t *testing.T) {
	_, r, b, image := applicationFixture(t)
	identity, err := image.Identity(context.Background())
	if err != nil {
		t.Fatal(err)
	}
	if identity.ClientSID != r.generation.record.ClientSID || identity.Generation != r.generation.record.Generation || identity.Commit != r.generation.record.Identity.Commit || identity.ExecutableSHA256 != r.generation.record.Members[generationApplicationMember].SHA256 {
		t.Fatal("image identity was declared independently of original")
	}
	for _, pair := range [][2]uint64{{0, 444}, {333, 0}, {334, 444}, {333, 445}} {
		if err = image.MatchesProcess(context.Background(), uint32(pair[0]), pair[1]); err == nil {
			t.Fatal("wrong native process accepted")
		}
	}
	if err = image.MatchesProcess(context.Background(), 333, 444); err != nil {
		t.Fatal("mismatch poisoned sound owner", err)
	}
	if b.applications != 3 {
		t.Fatal("invalid zero metadata reached native seam")
	}
	if _, err = json.Marshal(image); err == nil {
		t.Fatal("image serialized as authority")
	}
	if err = (&GenerationApplicationImage{}).MatchesProcess(context.Background(), 333, 444); !errors.Is(err, ErrClosed) {
		t.Fatal(err)
	}
}
func TestGenerationApplicationOriginalCustodyAndLifetimeRevokes(t *testing.T) {
	for _, fault := range []string{"store-close", "owner-close", "before-match", "during-match", "cancelled"} {
		t.Run(fault, func(t *testing.T) {
			s, r, b, image := applicationFixture(t)
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			switch fault {
			case "store-close":
				if err := s.Close(); err != nil {
					t.Fatal(err)
				}
			case "owner-close":
				if err := r.Close(); err != nil {
					t.Fatal(err)
				}
			case "before-match":
				b.readers[generationApplicationMember].data[0] ^= 1
			case "during-match":
				b.after = func() { b.readers[generationApplicationMember].data[0] ^= 1 }
			case "cancelled":
				cancel()
			}
			if err := image.MatchesProcess(ctx, 333, 444); err == nil {
				t.Fatal("revoked/changed original accepted")
			}
			if fault != "during-match" && b.applications != 0 {
				t.Fatal("invalid custody reached native image check")
			}
			if _, err := image.Identity(ctx); err == nil {
				t.Fatal("revoked image identity remained usable")
			}
		})
	}
}
func TestGenerationApplicationNonNativeCannotMintAndBaselineSharesOwner(t *testing.T) {
	_, g, _ := readOnlyFixture(t)
	r, err := g.TransitionReadOnly(context.Background())
	if err != nil {
		t.Fatal(err)
	}
	if _, err = r.ApplicationImage(context.Background()); !errors.Is(err, ErrNativeRequired) {
		t.Fatal("non-native minted image", err)
	}
	_, r, _, _ = applicationFixture(t)
	baseline := &ReadOnlyApprovedBaselineGeneration{owner: r}
	image, err := baseline.ApplicationImage(context.Background())
	if err != nil {
		t.Fatal(err)
	}
	if err = baseline.Close(); err != nil {
		t.Fatal(err)
	}
	if _, err = image.Identity(context.Background()); !errors.Is(err, ErrClosed) {
		t.Fatal("baseline wrapper copied custody", err)
	}
}

// Build a genuine materialized original-member set; no supplied sidecar alone
// can mint GenerationApplicationImage or bypass complete inventory validation.
func qualificationApplicationFixture(t *testing.T, raw []byte) (*Store, *ReadOnlyGeneration, *memoryApplicationBackend, *GenerationApplicationImage) {
	t.Helper()
	f := baselineEngineData(t)
	f.payload[generationQualificationMember] = bytes.Clone(raw)
	var manifest map[string]any
	if err := json.Unmarshal(f.manifest, &manifest); err != nil {
		t.Fatal(err)
	}
	manifest["files"] = append(manifest["files"].([]any), map[string]string{"path": generationQualificationMember, "sha256": digest(string(raw))})
	var err error
	f.manifest, err = json.Marshal(manifest)
	if err != nil {
		t.Fatal(err)
	}
	var inventory map[string]string
	if err := json.Unmarshal(f.inventory, &inventory); err != nil {
		t.Fatal(err)
	}
	inventory[generationQualificationMember], inventory["installer/package.json"] = digest(string(raw)), digest(string(f.manifest))
	f.inventory, err = json.Marshal(inventory)
	if err != nil {
		t.Fatal(err)
	}
	f.expected.InventorySHA256, f.expected.PackageSHA256 = digest(string(f.inventory)), digest(string(f.manifest))
	observed := map[string]string{}
	for name, content := range f.payload {
		observed[name] = digest(string(content))
	}
	f.identity, _, err = baselineevidence.Validate(context.Background(), f.inventory, f.manifest, f.payload["docs/PRODUCT_BUILD.json"], observed, f.expected)
	if err != nil {
		t.Fatal(err)
	}
	s, previous, _, _ := baselineOwner(t)
	t.Cleanup(func() { s.Close() })
	writeBaseline(t, previous, f)
	if err := previous.Complete(context.Background(), f.identity); err != nil {
		t.Fatal(err)
	}
	b := &memoryGeneration{memoryCandidate: &memoryCandidate{files: map[string]*memoryFile{}}}
	s.backend = &generationMemoryStore{baselineMemoryStore: s.backend.(*baselineMemoryStore), destination: b, root: t.TempDir()}
	c, err := s.Begin(strings.Repeat("9", 32), registrationTestSID)
	if err != nil {
		t.Fatal(err)
	}
	acquireGenerationArtifacts(t, c, f)
	g, err := s.MaterializeCandidateFresh(context.Background(), previous, c)
	if err != nil {
		t.Fatal(err)
	}
	readonly := &readOnlyMemoryBackend{memoryGeneration: b, old: map[string]*memoryFile{}, readers: map[string]*memoryFile{}}
	for name, file := range b.files {
		readonly.old[name] = file
	}
	backend := &memoryApplicationBackend{memoryExecutorBackend: &memoryExecutorBackend{readOnlyMemoryBackend: readonly}}
	g.backend = backend
	r, err := g.TransitionReadOnly(context.Background())
	if err != nil {
		t.Fatal(err)
	}
	image, err := r.ApplicationImage(context.Background())
	if err != nil {
		t.Fatal(err)
	}
	return s, r, backend, image
}

type qualificationMemberFault struct {
	reader io.ReaderAt
	after  func()
	err    error
	short  bool
}

func (r qualificationMemberFault) ReadAt(p []byte, off int64) (int, error) {
	n, err := r.reader.ReadAt(p, off)
	if r.after != nil {
		r.after()
	}
	if r.err != nil {
		return n, r.err
	}
	if r.short && n > 0 {
		return n - 1, io.EOF
	}
	return n, err
}

func TestGenerationApplicationQualificationReadsOnlyOwnedOriginalAndCleansViews(t *testing.T) {
	raw := []byte("{\"original\":\"retained compact bytes\"}")
	_, r, _, image := qualificationApplicationFixture(t, raw)
	for i := 0; i < 3; i++ {
		got, err := image.QualificationBytes(context.Background())
		if err != nil || !bytes.Equal(got, raw) || len(r.children) != 0 {
			t.Fatal("original bytes or view cleanup differs", string(got), err, len(r.children))
		}
		got[0] ^= 1
	}
	// A sidecar is data, not a constructor or path-based native capability.
	if raw, err := (&GenerationApplicationImage{}).QualificationBytes(context.Background()); !errors.Is(err, ErrClosed) || raw != nil {
		t.Fatal("zero image read sidecar authority", raw, err)
	}
	_, _, _, old := applicationFixture(t)
	if raw, err := old.QualificationBytes(context.Background()); err == nil || raw != nil {
		t.Fatal("old producer without sidecar qualified", raw, err)
	}
}

func TestGenerationApplicationQualificationBoundsRevocationMutationAndReadFailures(t *testing.T) {
	for _, fault := range []string{"oversized", "cancel-before", "cancel-during", "store-close", "owner-close", "mutate-before", "mutate-during", "read-failure", "short-read"} {
		t.Run(fault, func(t *testing.T) {
			raw := []byte("{\"original\":true}")
			if fault == "oversized" {
				raw = bytes.Repeat([]byte("x"), maxGenerationQualificationBytes+1)
			}
			s, r, b, image := qualificationApplicationFixture(t, raw)
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			member := r.generation.files[generationQualificationMember].(*generationReadOnlyFile)
			switch fault {
			case "cancel-before":
				cancel()
			case "cancel-during":
				member.reader = qualificationMemberFault{reader: member.reader, after: cancel}
			case "store-close":
				if err := s.Close(); err != nil {
					t.Fatal(err)
				}
			case "owner-close":
				if err := r.Close(); err != nil {
					t.Fatal(err)
				}
			case "mutate-before":
				b.readers[generationQualificationMember].data[0] ^= 1
			case "mutate-during":
				member.reader = qualificationMemberFault{reader: member.reader, after: func() { b.readers[generationQualificationMember].data[0] ^= 1 }}
			case "read-failure":
				member.reader = qualificationMemberFault{reader: member.reader, err: testFault}
			case "short-read":
				member.reader = qualificationMemberFault{reader: member.reader, short: true}
			}
			got, err := image.QualificationBytes(ctx)
			if err == nil || got != nil || len(r.children) != 0 {
				t.Fatal("invalid original accepted or view leaked", got, err, len(r.children))
			}
			if fault == "read-failure" && !errors.Is(err, testFault) {
				t.Fatal("original read failure lost", err)
			}
			if strings.HasPrefix(fault, "cancel-") && !errors.Is(err, context.Canceled) {
				t.Fatal("cancellation lost", err)
			}
		})
	}
}
