package brokerstore

import (
	"bytes"
	"context"
	"errors"
	"io"
	"strings"
	"sync"
	"testing"
	"time"

	"example.com/opennav-update-verifier/brokerregistration"
)

// This seam gives inventory checks an evolving shared namespace. A retained
// owner must explicitly acquire a completed readonly sibling before its exact
// namespace can validate; publication alone never changes its expected set.
type historyMemoryStore struct {
	*generationMemoryStore
	baseline                     *memoryBaselineView
	entries                      map[string]*historyMemoryGeneration
	next                         func(*historyMemoryGeneration)
	prepared, installed, drained int
}

type historyMemoryGeneration struct {
	*readOnlyMemoryBackend
	store                                *historyMemoryStore
	stage                                string
	known                                map[string]bool
	readonly                             bool
	prepareFault, drainFault, finalFault bool
	reservationGate                      func()
	onCheck                              func()
}

type historyMemoryBaseline struct {
	*historyMemoryGeneration
	baseline *memoryBaselineView
}

func (b *historyMemoryGeneration) check() error {
	if b.onCheck != nil {
		call := b.onCheck
		b.onCheck = nil
		call()
	}
	return b.readOnlyMemoryBackend.check()
}

func (b *historyMemoryBaseline) baselineSource(m map[string]baselineMember) (map[string]diskIdentity, map[string]diskIdentity, generationBaselineOrigin, error) {
	return b.baseline.baselineSource(m)
}
func (s *historyMemoryStore) newGeneration(stage string, inner *readOnlyMemoryBackend) *historyMemoryGeneration {
	b := &historyMemoryGeneration{readOnlyMemoryBackend: inner, store: s, stage: stage, known: map[string]bool{}}
	if s.next != nil {
		s.next(b)
		s.next = nil
	}
	return b
}
func (s *historyMemoryStore) prepareInstalledGeneration(_ installedBaselineBackend, _ candidateBackend, _ *Acquisition, _, stage string) (generationBackend, error) {
	s.prepares++
	b := &readOnlyMemoryBackend{memoryGeneration: &memoryGeneration{memoryCandidate: &memoryCandidate{files: map[string]*memoryFile{}}}, old: map[string]*memoryFile{}, readers: map[string]*memoryFile{}}
	return s.newGeneration(stage, b), nil
}
func (s *historyMemoryStore) prepareBaselineGeneration(_ context.Context, _ installedBaselineBackend, request brokerregistration.Request) (baselineGenerationBackend, error) {
	s.prepares++
	return &historyMemoryBaseline{historyMemoryGeneration: s.newGeneration(request.Installed.Generation, s.baseline.readOnlyMemoryBackend), baseline: s.baseline}, nil
}

func (b *historyMemoryGeneration) reserve(ctx context.Context, raw []byte) error {
	for stage := range b.store.entries {
		if stage == b.stage {
			return testFault
		}
		b.known[stage] = true
	}
	if err := b.readOnlyMemoryBackend.reserve(ctx, raw); err != nil {
		return err
	}
	b.store.entries[b.stage] = b
	if b.reservationGate != nil {
		b.reservationGate()
	}
	return nil
}
func (b *historyMemoryGeneration) inventory(names []string, completed bool) error {
	if err := b.readOnlyMemoryBackend.inventory(names, completed); err != nil {
		return err
	}
	for stage := range b.store.entries {
		if stage != b.stage && !b.known[stage] {
			return errors.New("unexplained shared namespace")
		}
	}
	if b.finalFault && b.store.installed > 0 {
		return testFault
	}
	return nil
}
func (b *historyMemoryGeneration) transitionReadOnly(ctx context.Context, files map[string]candidateFile, members map[string]baselineMember) error {
	for name, file := range b.files {
		b.old[name] = file
	}
	if err := b.readOnlyMemoryBackend.transitionReadOnly(ctx, files, members); err != nil {
		return err
	}
	b.readonly = true
	return nil
}
func (b *historyMemoryGeneration) historyMemory() *historyMemoryGeneration { return b }
func (b *historyMemoryGeneration) prepareSibling(ctx context.Context, successor generationBackend) (generationSiblingAppend, error) {
	b.store.prepared++
	proof, ok := successor.(interface {
		historyMemory() *historyMemoryGeneration
	})
	if !ok {
		return nil, testFault
	}
	source := proof.historyMemory()
	if source == b || source.store != b.store || !source.readonly || !source.completed || b.known[source.stage] {
		return nil, testFault
	}
	a := &historyMemoryAppend{owner: b, source: source}
	if b.prepareFault {
		return a, testFault
	}
	return a, ctx.Err()
}

type historyMemoryAppend struct {
	owner, source *historyMemoryGeneration
	installed     bool
}

func (a *historyMemoryAppend) install() {
	a.owner.known[a.source.stage] = true
	a.owner.store.installed++
	a.installed = true
}
func (a *historyMemoryAppend) close() error {
	if a.installed {
		return nil
	}
	a.owner.store.drained++
	if a.owner.drainFault {
		return errors.New("prepared history cleanup failed")
	}
	return nil
}

func historyFixture(t *testing.T) (*Store, *InstalledBaseline, *historyMemoryStore, baselineFixtureData, []byte) {
	t.Helper()
	s, p, baseline, f, raw := baselineViewFixture(t)
	h := &historyMemoryStore{generationMemoryStore: &generationMemoryStore{baselineMemoryStore: s.backend.(*memoryBaselineViewStore).baselineMemoryStore, root: t.TempDir()}, baseline: baseline, entries: map[string]*historyMemoryGeneration{}}
	s.backend = h
	return s, p, h, f, raw
}
func historyCandidate(t *testing.T, s *Store, p *InstalledBaseline, f baselineFixtureData, stage string) *Candidate {
	t.Helper()
	c, err := s.Begin(stage, p.record.ClientSID)
	if err != nil {
		t.Fatal(err)
	}
	acquireGenerationArtifacts(t, c, f)
	return c
}
func historyReadOnly(t *testing.T, s *Store, p *InstalledBaseline, f baselineFixtureData, stage string) *ReadOnlyGeneration {
	t.Helper()
	r, err := s.MaterializeCandidateReadOnlyFresh(context.Background(), p, historyCandidate(t, s, p, f, stage))
	if err != nil {
		t.Fatal(err)
	}
	return r
}
func TestGenerationHistoryAtomicMixedPublicationsPreserveOwnersAndMembers(t *testing.T) {
	for _, order := range []string{"baseline-first", "candidate-first"} {
		t.Run(order, func(t *testing.T) {
			s, p, h, f, raw := historyFixture(t)
			var first *ReadOnlyGeneration
			var baseline *ReadOnlyApprovedBaselineGeneration
			var err error
			if order == "baseline-first" {
				baseline, err = s.MaterializeApprovedBaselineReadOnlyFresh(context.Background(), p, raw, registrationDigest(raw))
				if err != nil {
					t.Fatal(err)
				}
				first = baseline.owner
			} else {
				first = historyReadOnly(t, s, p, f, strings.Repeat("9", 32))
			}
			original := bytes.Clone(first.generation.raw)
			member, err := first.OpenMember("app/opencpn.exe")
			if err != nil {
				t.Fatal(err)
			}
			var second *ReadOnlyGeneration
			if order == "baseline-first" {
				second = historyReadOnly(t, s, p, f, strings.Repeat("9", 32))
			} else {
				baseline, err = s.MaterializeApprovedBaselineReadOnlyFresh(context.Background(), p, raw, registrationDigest(raw))
				if err != nil {
					t.Fatal(err)
				}
				second = baseline.owner
			}
			third := historyReadOnly(t, s, p, f, strings.Repeat("8", 32))
			for _, owner := range []*ReadOnlyGeneration{first, second, third} {
				if err := owner.Validate(); err != nil {
					t.Fatal(err)
				}
			}
			if !bytes.Equal(original, first.generation.raw) || h.installed != 3 {
				t.Fatal("history rewrote completion or missed atomic appends", h.installed)
			}
			if _, err := baseline.Completion(); err != nil {
				t.Fatal(err)
			}
			if err := third.Close(); err != nil {
				t.Fatal(err)
			}
			if err := second.Close(); err != nil {
				t.Fatal(err)
			}
			got := make([]byte, len(f.payload["app/opencpn.exe"])+1)
			n, err := member.ReadAt(got, 0)
			if !errors.Is(err, io.EOF) || !bytes.Equal(got[:n], f.payload["app/opencpn.exe"]) {
				t.Fatal("successor Close revoked prior member", err)
			}
			if err := first.Validate(); err != nil {
				t.Fatal("successor Close revoked prior history", err)
			}
		})
	}
}

func TestGenerationHistoryWriterRefusalAndPreMutationFailures(t *testing.T) {
	for _, scenario := range []string{"writer-live", "writer-candidate-api", "writer-baseline-api", "cancelled", "cancel-during-prevalidation", "nil-context", "bad-new-source", "bad-baseline-pin"} {
		t.Run(scenario, func(t *testing.T) {
			s, p, h, f, raw := historyFixture(t)
			firstCandidate := historyCandidate(t, s, p, f, strings.Repeat("9", 32))
			var first *InstalledGeneration
			if scenario == "writer-live" {
				var err error
				first, err = s.MaterializeCandidateFresh(context.Background(), p, firstCandidate)
				if err != nil {
					t.Fatal(err)
				}
			} else {
				r, err := s.MaterializeCandidateReadOnlyFresh(context.Background(), p, firstCandidate)
				if err != nil {
					t.Fatal(err)
				}
				first = r.generation
			}
			c := historyCandidate(t, s, p, f, strings.Repeat("8", 32))
			before := h.prepares
			ctx := context.Background()
			var err error
			switch scenario {
			case "writer-candidate-api":
				_, err = s.MaterializeCandidateFresh(ctx, p, c)
			case "writer-baseline-api":
				_, err = s.MaterializeApprovedBaselineFresh(ctx, p, raw, registrationDigest(raw))
			case "bad-baseline-pin":
				_, err = s.MaterializeApprovedBaselineReadOnlyFresh(ctx, p, raw, strings.Repeat("0", 64))
			default:
				if scenario == "cancelled" {
					var cancel context.CancelFunc
					ctx, cancel = context.WithCancel(ctx)
					cancel()
				}
				if scenario == "nil-context" {
					ctx = nil
				}
				if scenario == "cancel-during-prevalidation" {
					var cancel context.CancelFunc
					ctx, cancel = context.WithCancel(ctx)
					defer cancel()
					h.entries[first.record.Generation].onCheck = cancel
				}
				if scenario == "bad-new-source" {
					c.acquisition.closeErr = testFault
				}
				_, err = s.MaterializeCandidateReadOnlyFresh(ctx, p, c)
			}
			if err == nil || h.prepares != before || c.materialized || len(h.entries) != 1 || first.failure != nil {
				t.Fatal("pre-mutation refusal consumed source or poisoned existing owner", err)
			}
			if strings.HasPrefix(scenario, "writer-") && !errors.Is(err, ErrGenerationHistoryRequiresReadOnly) {
				t.Fatal(err)
			}
			if scenario == "cancel-during-prevalidation" {
				if !errors.Is(err, context.Canceled) {
					t.Fatal("prevalidation cancellation lost", err)
				}
				if err := (&ReadOnlyGeneration{generation: first}).Validate(); err != nil {
					t.Fatal("prevalidation cancellation revoked existing owner", err)
				}
			}

		})
	}
}

func TestGenerationHistoryUncertainMutationPoisonsAllAffectedOwners(t *testing.T) {
	for _, scenario := range []string{"reserve", "copy", "publish", "transition", "cancel-after-publish", "prepare-second", "prepare-cleanup", "final-validation", "candidate-source-cleanup", "baseline-source-cleanup", "baseline-transition", "baseline-prepare-second"} {
		t.Run(scenario, func(t *testing.T) {
			s, p, h, f, raw := historyFixture(t)
			first := historyReadOnly(t, s, p, f, strings.Repeat("9", 32))
			second := historyReadOnly(t, s, p, f, strings.Repeat("8", 32))
			before := h.installed
			member, err := first.OpenMember("app/opencpn.exe")
			if err != nil {
				t.Fatal(err)
			}
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			h.next = func(b *historyMemoryGeneration) {
				switch scenario {
				case "reserve", "copy", "publish":
					b.faultAt = scenario
				case "transition", "baseline-transition":
					b.fault = "after-reopen"
				case "candidate-source-cleanup":
					b.changedSource = true
					b.closeFault = io.ErrClosedPipe
				case "baseline-source-cleanup":
					h.baseline.sourceChanged = true
					b.closeFault = io.ErrClosedPipe
				case "cancel-after-publish":
					b.onPublish = cancel
				}
			}
			if scenario == "prepare-second" || scenario == "prepare-cleanup" || scenario == "baseline-prepare-second" {
				h.entries[strings.Repeat("8", 32)].prepareFault = true
				if scenario == "prepare-cleanup" {
					h.entries[strings.Repeat("9", 32)].drainFault = true
				}
			}
			if scenario == "final-validation" {
				h.entries[strings.Repeat("9", 32)].finalFault = true
				h.installed = 0
				before = 0
			}
			c := historyCandidate(t, s, p, f, strings.Repeat("7", 32))
			var r *ReadOnlyGeneration
			if strings.HasPrefix(scenario, "baseline-") {
				var baseline *ReadOnlyApprovedBaselineGeneration
				baseline, err = s.MaterializeApprovedBaselineReadOnlyFresh(ctx, p, raw, registrationDigest(raw))
				if baseline != nil {
					r = baseline.owner
				}
			} else {
				r, err = s.MaterializeCandidateReadOnlyFresh(ctx, p, c)
			}
			if r != nil || !errors.Is(err, ErrInstalledGenerationUncertain) || (!strings.HasPrefix(scenario, "baseline-") && !c.materialized) {
				t.Fatal("mutation failure lost uncertainty", err)
			}
			for _, owner := range []*ReadOnlyGeneration{first, second} {
				if !errors.Is(owner.Validate(), ErrPoisoned) {
					t.Fatal("affected owner remained usable")
				}
			}
			if _, err = member.ReadAt(make([]byte, 1), 0); !errors.Is(err, ErrPoisoned) {
				t.Fatal("affected member remained usable", err)
			}
			if scenario == "prepare-second" || scenario == "prepare-cleanup" || scenario == "baseline-prepare-second" {
				if h.installed != before || h.drained != 2 {
					t.Fatal("preparation failure partially installed or leaked handles", h.installed, h.drained)
				}
			}
			if strings.HasSuffix(scenario, "source-cleanup") && !errors.Is(err, io.ErrClosedPipe) {
				t.Fatal("source failure cleanup lost", err)
			}
			if scenario == "prepare-cleanup" && !strings.Contains(err.Error(), "prepared history cleanup failed") {
				t.Fatal("cleanup error lost", err)
			}
			if _, err = s.MaterializeCandidateReadOnlyFresh(context.Background(), p, c); err == nil {
				t.Fatal("uncertain mutation retried")
			}
		})
	}
}

func TestGenerationHistoryPublicOperationsWaitForAtomicPublication(t *testing.T) {
	for _, operation := range []string{"validate", "completion", "open", "read", "identity", "member-close", "owner-close", "writer-close", "store-close"} {
		t.Run(operation, func(t *testing.T) {
			s, p, h, f, _ := historyFixture(t)
			first := historyReadOnly(t, s, p, f, strings.Repeat("9", 32))
			member, err := first.OpenMember("app/opencpn.exe")
			if err != nil {
				t.Fatal(err)
			}
			c := historyCandidate(t, s, p, f, strings.Repeat("8", 32))
			reserved, release := make(chan struct{}), make(chan struct{})
			var once sync.Once
			unblock := func() { once.Do(func() { close(release) }) }
			defer unblock()
			h.next = func(b *historyMemoryGeneration) { b.reservationGate = func() { close(reserved); <-release } }
			published := make(chan error, 1)
			go func() { _, err := s.MaterializeCandidateReadOnlyFresh(context.Background(), p, c); published <- err }()
			select {
			case <-reserved:
			case <-time.After(5 * time.Second):
				t.Fatal("publication did not reserve")
			}
			if s.mu.TryLock() {
				s.mu.Unlock()
				t.Fatal("publication released Store lease")
			}
			started, done := make(chan struct{}), make(chan error, 1)
			go func() {
				close(started)
				var err error
				switch operation {
				case "validate":
					err = first.Validate()
				case "completion":
					_, err = first.Completion()
				case "open":
					var next *GenerationReadHandle
					next, err = first.OpenMember("app/opencpn.exe")
					if next != nil {
						err = errors.Join(err, next.Close())
					}
				case "read":
					_, err = member.ReadAt(make([]byte, 1), 0)
				case "identity":
					_, err = member.Identity()
				case "member-close":
					err = member.Close()
				case "owner-close":
					err = first.Close()
				case "writer-close":
					err = first.generation.Close()
				case "store-close":
					err = s.Close()
				}
				done <- err
			}()
			<-started
			select {
			case err := <-done:
				t.Fatal("operation observed intermediate namespace", err)
			case <-time.After(30 * time.Millisecond):
			}
			unblock()
			for _, result := range []<-chan error{published, done} {
				select {
				case err := <-result:
					if err != nil {
						t.Fatal(err)
					}
				case <-time.After(5 * time.Second):
					t.Fatal("Store/generation lock deadlock")
				}
			}
		})
	}
}
