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

// These portable seams prove ordering, original-byte and lifetime behavior;
// native physical identities, descriptors and historical reopen are separate.
type generationMemoryStore struct {
	*baselineMemoryStore
	destination *memoryGeneration
	root        string
	prepares    int
}

func (s *generationMemoryStore) begin(_, _ string) (candidateBackend, error) {
	return &acquisitionCandidateFixture{memoryCandidate: memoryCandidate{files: map[string]*memoryFile{}}, lease: &acquisitionFixture{root: s.root}}, nil
}
func (s *generationMemoryStore) prepareInstalledGeneration(_ installedBaselineBackend, _ candidateBackend, _ *Acquisition, _, _ string) (generationBackend, error) {
	s.prepares++
	return s.destination, nil
}

type memoryGeneration struct {
	*memoryCandidate
	events                    []string
	faultAt                   string
	reserved, tree, completed bool
	origin, raw               []byte
	publishErr                error
	onReserve, onPublish      func()
	changedSource             bool
}

func (g *memoryGeneration) step(name string) error {
	g.events = append(g.events, name)
	if g.faultAt == name {
		return testFault
	}
	return nil
}
func (g *memoryGeneration) reserve(ctx context.Context, raw []byte) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	if err := g.step("reserve"); err != nil {
		return err
	}
	g.reserved = true
	g.origin = bytes.Clone(raw)
	if g.onReserve != nil {
		g.onReserve()
	}
	return nil
}
func (g *memoryGeneration) createTree() error {
	if !g.reserved {
		panic("generation tree before reservation")
	}
	if err := g.step("tree"); err != nil {
		return err
	}
	g.tree = true
	return nil
}
func (g *memoryGeneration) create(name string) (candidateFile, error) {
	if !g.tree {
		panic("file before tree")
	}
	if err := g.step("copy"); err != nil {
		return nil, err
	}
	return g.memoryCandidate.create(name)
}
func (g *memoryGeneration) inventory(names []string, completed bool) error {
	if err := g.step("inventory"); err != nil {
		return err
	}
	if completed != g.completed {
		return testFault
	}
	return g.memoryCandidate.candidateInventory(names, false)
}
func (g *memoryGeneration) sourceIdentities(m map[string]fileRecord) (map[string]diskIdentity, map[string]diskIdentity, error) {
	if err := g.step("source"); err != nil {
		return nil, nil, err
	}
	names := []string{}
	for name := range m {
		names = append(names, name)
	}
	sort.Strings(names)
	files := map[string]diskIdentity{}
	for i, name := range names {
		files[name] = baselineTestID(i + 100)
	}
	if g.changedSource && g.tree {
		files[names[0]] = baselineTestID(999)
	}
	return map[string]diskIdentity{"": baselineTestID(90)}, files, nil
}
func (g *memoryGeneration) identities(m map[string]baselineMember) (map[string]diskIdentity, map[string]diskIdentity, error) {
	if err := g.step("identities"); err != nil {
		return nil, nil, err
	}
	names := []string{}
	for name := range m {
		names = append(names, name)
	}
	sort.Strings(names)
	files := map[string]diskIdentity{}
	for i, name := range names {
		files[name] = baselineTestID(i + 200)
	}
	return map[string]diskIdentity{"": baselineTestID(190)}, files, nil
}
func (g *memoryGeneration) publish(ctx context.Context, raw []byte) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	if err := g.step("publish"); err != nil {
		return err
	}
	g.raw = bytes.Clone(raw)
	g.completed = true
	if g.onPublish != nil {
		g.onPublish()
	}
	return g.publishErr
}

func acquireGenerationArtifacts(t *testing.T, c *Candidate, f baselineFixtureData) {
	t.Helper()
	a, err := c.BeginAcquisition()
	if err != nil {
		t.Fatal(err)
	}
	p := acquisitionPlan()
	p.Version = f.identity.Version
	p.Commit = f.identity.Commit
	if err = a.Bind(p); err != nil {
		t.Fatal(err)
	}
	for _, pair := range []struct {
		slot ArtifactSlot
		raw  string
	}{{RecoverySlot, "zip"}, {InstallerSlot, "exe"}} {
		artifact, err := a.Create(pair.slot)
		if err != nil {
			t.Fatal(err)
		}
		if _, err = artifact.Write([]byte(pair.raw)); err != nil {
			t.Fatal(err)
		}
		if _, err = artifact.SealReadOnly(); err != nil {
			t.Fatal(err)
		}
	}
	writeCandidateEvidence(t, c, f)
	if err = c.CompleteRecoveryEvidence(context.Background(), f.identity); err != nil {
		t.Fatal(err)
	}
	if err = a.Close(); err != nil {
		t.Fatal(err)
	}
}
func generationFixture(t *testing.T) (*Store, *InstalledBaseline, *Candidate, *memoryGeneration, baselineFixtureData) {
	t.Helper()
	s, previous, _, _ := baselineOwner(t)
	f := baselineEngineData(t)
	writeBaseline(t, previous, f)
	if err := previous.Complete(context.Background(), f.identity); err != nil {
		t.Fatal(err)
	}
	g := &memoryGeneration{memoryCandidate: &memoryCandidate{files: map[string]*memoryFile{}}}
	s.backend = &generationMemoryStore{baselineMemoryStore: s.backend.(*baselineMemoryStore), destination: g, root: t.TempDir()}
	c, err := s.Begin(strings.Repeat("9", 32), registrationTestSID)
	if err != nil {
		t.Fatal(err)
	}
	acquireGenerationArtifacts(t, c, f)
	t.Cleanup(func() { s.Close() })
	return s, previous, c, g, f
}
func TestGenerationCopiesExactOriginalsAndSeparateCandidateOrigin(t *testing.T) {
	s, p, c, b, f := generationFixture(t)
	g, err := s.MaterializeCandidateFresh(context.Background(), p, c)
	if err != nil {
		t.Fatal(err)
	}
	original := map[string][]byte{"FILE_SHA256.json": f.inventory, "installer/package.json": f.manifest}
	for name, raw := range f.payload {
		original[name] = raw
	}
	if len(b.files) != len(original) {
		t.Fatal("incomplete copy")
	}
	for name, raw := range original {
		if !bytes.Equal(b.files[name].data, raw) {
			t.Fatal("original changed", name)
		}
	}
	completion, err := g.Completion()
	if err != nil {
		t.Fatal(err)
	}
	if completion.Schema != 1 || completion.Generation != c.stageID || completion.PreviousGeneration != p.record.Generation || completion.Identity != f.identity || completion.PolicySHA256 != c.acquisition.plan.PolicySHA256 || completion.CompletionSHA256 != registrationDigest(b.raw) {
		t.Fatal("completion binding differs")
	}
	var record generationRecord
	if err = json.Unmarshal(b.raw, &record); err != nil {
		t.Fatal(err)
	}
	if bytes.Contains(b.raw, []byte(`"approvedRequestSha256":"`+completion.PolicySHA256)) {
		t.Fatal("policy hash masquerades as operator approval")
	}
	if registrationDigest(record.PreviousCompletion) != record.PreviousCompletionSHA256 {
		t.Fatal("original baseline lineage unbound")
	}
	for i, event := range b.events {
		if event == "tree" && (i == 0 || b.events[i-1] != "reserve") {
			t.Fatal("creation before reservation")
		}
	}
	if _, err = s.MaterializeCandidateFresh(context.Background(), p, c); err == nil {
		t.Fatal("materialization retry accepted")
	}
	if err = g.Close(); err != nil {
		t.Fatal(err)
	}
	if err = g.Close(); err != nil {
		t.Fatal(err)
	}
	if b.closes != 1 {
		t.Fatal("close repeated")
	}
}
func TestGenerationRefusesInvalidLiveAuthorityBeforeReservation(t *testing.T) {
	for _, scenario := range []string{"other-store", "cancelled", "uncompleted-baseline", "sealed", "no-acquisition", "unclean-acquisition", "wrong-slot", "wrong-artifact", "wrong-version", "missing-helper", "tampered-source"} {
		t.Run(scenario, func(t *testing.T) {
			s, p, c, b, _ := generationFixture(t)
			ctx := context.Background()
			switch scenario {
			case "other-store":
				c.store = &Store{}
			case "cancelled":
				var cancel context.CancelFunc
				ctx, cancel = context.WithCancel(ctx)
				cancel()
			case "uncompleted-baseline":
				p.completed = false
			case "sealed":
				c.sealed = true
			case "no-acquisition":
				c.acquisition = nil
			case "unclean-acquisition":
				c.acquisition.closeErr = testFault
			case "wrong-slot":
				c.acquisition.artifacts[1].slot = RecoverySlot
			case "wrong-artifact":
				c.acquisition.artifacts[0].spec.SHA256 = digest("different")
			case "wrong-version":
				c.acquisition.plan.Version = "different"
			case "missing-helper":
				delete(c.recoveryMembers, "installer/engine/Lifecycle.ps1")
			case "tampered-source":
				c.recoveryMembers["app/opencpn.exe"].file.(*memoryFile).data[0] ^= 1
			}
			if _, err := s.MaterializeCandidateFresh(ctx, p, c); err == nil {
				t.Fatal("invalid authority accepted")
			}
			if b.reserved || len(b.files) != 0 {
				t.Fatal("authority failure reached creation")
			}
		})
	}
}
func TestGenerationFailurePreservesReservationAndNoRetry(t *testing.T) {
	for _, point := range []string{"reserve", "tree", "copy", "identities", "inventory", "publish", "source-changed", "copy-sync"} {
		t.Run(point, func(t *testing.T) {
			s, p, c, b, _ := generationFixture(t)
			if point == "source-changed" {
				b.changedSource = true
			} else if point == "copy-sync" {
				b.configure = func(f *memoryFile) { f.syncFault = testFault }
			} else {
				b.faultAt = point
			}
			if _, err := s.MaterializeCandidateFresh(context.Background(), p, c); err == nil {
				t.Fatal("injected failure passed")
			}
			if b.completed {
				t.Fatal("precommit failure completed")
			}
			if _, err := s.MaterializeCandidateFresh(context.Background(), p, c); err == nil {
				t.Fatal("failed source retried")
			}
			if b.closes != 1 {
				t.Fatal("failed owner not drained once")
			}
		})
	}
}
func TestGenerationCancellationCommitAndUncertainty(t *testing.T) {
	for _, point := range []string{"after-reserve", "after-commit", "uncertain"} {
		t.Run(point, func(t *testing.T) {
			s, p, c, b, _ := generationFixture(t)
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			if point == "after-reserve" {
				b.onReserve = cancel
			} else {
				b.onPublish = cancel
			}
			if point == "uncertain" {
				b.publishErr = errors.Join(ErrInstalledGenerationUncertain, testFault)
			}
			g, err := s.MaterializeCandidateFresh(ctx, p, c)
			switch point {
			case "after-reserve":
				if !errors.Is(err, context.Canceled) || b.tree || b.completed {
					t.Fatal("cancelled precommit created tree", err)
				}
			case "after-commit":
				if err != nil || g == nil || g.failure != nil {
					t.Fatal("durable commit reported cancelled", err)
				}
				if err = g.Validate(); err != nil {
					t.Fatal(err)
				}
			case "uncertain":
				if !errors.Is(err, ErrInstalledGenerationUncertain) || !b.completed {
					t.Fatal("durable uncertainty misclassified", err)
				}
			}
		})
	}
}
func TestGenerationOwnerRevalidatesBytesAndStoreDrainsOnce(t *testing.T) {
	for _, scenario := range []string{"tamper", "extra", "close-failure", "store-close"} {
		t.Run(scenario, func(t *testing.T) {
			s, p, c, b, _ := generationFixture(t)
			g, err := s.MaterializeCandidateFresh(context.Background(), p, c)
			if err != nil {
				t.Fatal(err)
			}
			switch scenario {
			case "tamper":
				b.files["installer/engine/Lifecycle.ps1"].data[0] ^= 1
				if err = g.Validate(); err == nil {
					t.Fatal("tamper accepted")
				}
			case "extra":
				b.files["app/extra"] = &memoryFile{}
				if _, err = g.Completion(); err == nil {
					t.Fatal("unknown member accepted")
				}
			case "close-failure":
				b.files["app/opencpn.exe"].closeFault = testFault
				first := g.Close()
				if !errors.Is(first, testFault) || !errors.Is(first, ErrInstalledGenerationUncertain) || g.Close() != first {
					t.Fatal("completed close uncertainty lost", first)
				}
			case "store-close":
				if err = s.Close(); err != nil {
					t.Fatal(err)
				}
				if !g.closed || b.closes != 1 {
					t.Fatal("store did not drain generation")
				}
				if err = g.Validate(); !errors.Is(err, ErrClosed) {
					t.Fatal("closed owner usable", err)
				}
			}
		})
	}
}
