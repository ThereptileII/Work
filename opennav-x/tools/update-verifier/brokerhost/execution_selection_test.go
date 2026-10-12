package brokerhost

import (
	"context"
	"encoding/json"
	"errors"
	"strings"
	"testing"

	verifier "example.com/opennav-update-verifier"
	"example.com/opennav-update-verifier/brokerpipe"
	"example.com/opennav-update-verifier/brokerstore"
	"example.com/opennav-update-verifier/releasepolicy"
)

func selectionBytes(t *testing.T, value any) []byte {
	t.Helper()
	b, err := json.Marshal(value)
	if err != nil {
		t.Fatal(err)
	}
	return b
}

func selectionFixture(t *testing.T, previous *executionSelectionAttempt, rollback bool) (executionSelectionAttempt, []executionSelectionFact) {
	t.Helper()
	generation, executionID := strings.Repeat("a", 32), strings.Repeat("b", 32)
	purpose := string(brokerstore.GenerationPurposeApprovedBaseline)
	b := executionSelectionBegin{Schema: 1, Purpose: "broker-selected-transaction-v1", ExecutionID: executionID, ClientSID: testSID, Generation: generation, Version: "1.0.0-beta.1", Commit: strings.Repeat("a", 40), CompletionSHA256: strings.Repeat("a", 64)}
	if previous != nil {
		generation, executionID, purpose = strings.Repeat("c", 32), strings.Repeat("d", 32), string(brokerstore.GenerationPurposeSignedCandidate)
		b.ExecutionID, b.Generation, b.Version = executionID, generation, "1.0.0-beta.2"
		b.Commit, b.CompletionSHA256 = strings.Repeat("c", 40), strings.Repeat("c", 64)
		b.PreviousGeneration, b.PreviousHeadSHA256 = previous.begin.Generation, previous.head
	}
	facts := []executionSelectionFact{{brokerstore.ExecutionPreparation, selectionBytes(t, b)}}
	add := func(name string, kind brokerstore.ExecutionFactKind, target string) {
		facts = append(facts, executionSelectionFact{kind, selectionBytes(t, executionSelectionStep{1, "broker-selected-step-v1", executionID, name, target, strings.Repeat("e", 64)})})
	}
	add("commissioned", brokerstore.ExecutionCommissioning, generation)
	add("spawn-consumed", brokerstore.ExecutionSpawnIntent, generation)
	if rollback {
		add("candidate-closed", brokerstore.ExecutionTeardown, generation)
		add("profile-restored", brokerstore.ExecutionRestoration, generation)
		add("fallback-commissioned", brokerstore.ExecutionCommissioning, b.PreviousGeneration)
		add("fallback-spawn-consumed", brokerstore.ExecutionSpawnIntent, b.PreviousGeneration)
		add("fallback-healthy", brokerstore.ExecutionHealthObserved, b.PreviousGeneration)
		add("rolled-back", brokerstore.ExecutionRestoration, b.PreviousGeneration)
	} else {
		add("healthy", brokerstore.ExecutionHealthObserved, generation)
		add("selected", brokerstore.ExecutionHealthObserved, generation)
	}
	// Test receipt only. Native callers use the original full journal digest.
	originals := make([]string, len(facts))
	for i, f := range facts {
		originals[i] = string(f.original)
	}
	digest := executionOriginalDigest(selectionBytes(t, originals))
	a, err := parseExecutionSelectionAttempt(testSID, executionID, generation, b.CompletionSHA256, purpose, digest, facts)
	if err != nil {
		t.Fatal(err)
	}
	return a, facts
}

func TestExecutionSelectionRequiresCommissionedConsumedHealthyOrder(t *testing.T) {
	baseline, facts := selectionFixture(t, nil, false)
	parse := func(f []executionSelectionFact) (executionSelectionAttempt, error) {
		return parseExecutionSelectionAttempt(testSID, baseline.begin.ExecutionID, baseline.begin.Generation, baseline.begin.CompletionSHA256, string(brokerstore.GenerationPurposeApprovedBaseline), baseline.head, f)
	}
	for i := 1; i < len(facts); i++ {
		prefix, err := parse(facts[:i])
		if err != nil || prefix.terminal != "" {
			t.Fatalf("prefix %d acquired selection: %v %v", i, prefix, err)
		}
		if _, pending, err := foldExecutionSelection([]executionSelectionAttempt{prefix}, baseline.begin.Generation); err == nil || len(pending) != 1 {
			t.Fatal("unfinished attempt was dropped", pending, err)
		}
	}
	for _, tc := range []struct {
		name   string
		mutate func([]executionSelectionFact) []executionSelectionFact
	}{
		{"health label alone", func(f []executionSelectionFact) []executionSelectionFact {
			f[3].original = []byte(`{"healthy":true}`)
			return f
		}},
		{"health before consumed create", func(f []executionSelectionFact) []executionSelectionFact { return append(f[:2], f[3:]...) }},
		{"second spawn", func(f []executionSelectionFact) []executionSelectionFact {
			return append(append(f[:3:3], f[2]), f[3:]...)
		}},
		{"trailing step after selection", func(f []executionSelectionFact) []executionSelectionFact { return append(f, f[3]) }},
		{"wrong storage fact kind", func(f []executionSelectionFact) []executionSelectionFact {
			f[3].kind = brokerstore.ExecutionPreparation
			return f
		}},
		{"unknown JSON key", func(f []executionSelectionFact) []executionSelectionFact {
			f[0].original = append(f[0].original[:len(f[0].original)-1:len(f[0].original)-1], []byte(`,"extra":true}`)...)
			return f
		}},
		{"duplicate JSON key", func(f []executionSelectionFact) []executionSelectionFact {
			f[1].original = append(f[1].original[:len(f[1].original)-1:len(f[1].original)-1], []byte(`,"schema":1}`)...)
			return f
		}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			f := append([]executionSelectionFact(nil), facts...)
			if _, err := parse(tc.mutate(f)); err == nil {
				t.Fatal("unsafe sequence accepted")
			}
		})
	}
}

func TestExecutionSelectionResolvesActualPredecessorAndRollback(t *testing.T) {
	root, _ := selectionFixture(t, nil, false)
	next, _ := selectionFixture(t, &root, false)
	selected, pending, err := foldExecutionSelection([]executionSelectionAttempt{next, root}, root.begin.Generation)
	if err != nil || len(pending) != 0 || selected.generation != next.begin.Generation || selected.installed.Commit != next.begin.Commit || selected.head != next.head {
		t.Fatal(selected, pending, err)
	}
	failed, _ := selectionFixture(t, &root, true)
	selected, pending, err = foldExecutionSelection([]executionSelectionAttempt{failed, root}, root.begin.Generation)
	if err != nil || len(pending) != 0 || selected.generation != root.begin.Generation || selected.completion != root.begin.CompletionSHA256 || selected.head != failed.head || selected.installed.Commit != root.begin.Commit {
		t.Fatal(selected, pending, err)
	}
}

func TestExecutionSelectionRefusesForksMissingRootsAndPendingAttempts(t *testing.T) {
	root, _ := selectionFixture(t, nil, false)
	next, _ := selectionFixture(t, &root, false)
	fork := next
	fork.begin.ExecutionID = strings.Repeat("e", 32)
	fork.head = strings.Repeat("f", 64)
	unrelated := next
	unrelated.begin.PreviousHeadSHA256 = strings.Repeat("9", 64)
	wrongPrevious := next
	wrongPrevious.begin.PreviousGeneration = strings.Repeat("9", 32)
	pending := next
	pending.terminal = ""
	for _, tc := range []struct {
		name     string
		attempts []executionSelectionAttempt
	}{
		{"empty", nil}, {"missing baseline", []executionSelectionAttempt{next}},
		{"duplicate", []executionSelectionAttempt{root, root}}, {"fork", []executionSelectionAttempt{root, next, fork}},
		{"unrelated head", []executionSelectionAttempt{root, unrelated}}, {"wrong actual predecessor", []executionSelectionAttempt{root, wrongPrevious}},
		{"pending behind healthy baseline", []executionSelectionAttempt{root, pending}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			if _, _, err := foldExecutionSelection(tc.attempts, root.begin.Generation); err == nil {
				t.Fatal("uncertain history selected")
			}
		})
	}
}

type selectedStageFixture struct {
	version                releasepolicy.Installed
	checks, closes, failAt int
	closeErr               error
}

func (s *selectedStageFixture) installed() releasepolicy.Installed { return s.version }
func (s *selectedStageFixture) validate() error {
	s.checks++
	if s.checks == s.failAt {
		return errors.New("selected history changed")
	}
	return nil
}
func (s *selectedStageFixture) Close() error { s.closes++; return s.closeErr }

func selectedFixtureEngine(t *testing.T, f *stageFixture, s *selectedStageFixture) *engine {
	t.Helper()
	e := fixtureEngine(f)
	e.selected = func(context.Context, registration) (selectedExecutionOwner, error) { return s, nil }
	e.prepare = func(_ context.Context, c verifier.PrepareConfig, o releasepolicy.OpenCpn, i verifier.ConsentIdentity, stage candidate) (acquisition, error) {
		reserved, ok := stage.(fakeStage)
		if !ok || reserved.f != f || c.Installed != s.version || s.closes != 0 || c.StateDirectory != "protected-state" || c.ArtifactDirectory != "protected-artifacts" || c.ArtifactOrigin != "https://updates.invalid" || o != stockBaseline {
			t.Fatal("preparation lost protected selected version or retained source", c.Installed, s.closes)
		}
		if err := f.record("prepare"); err != nil {
			return nil, err
		}
		return fakeAcquisition{f, testRecovery(i)}, nil
	}
	return e
}

func TestStagingUsesProtectedSelectedVersionAndRetainsHistory(t *testing.T) {
	f := &stageFixture{registration: testRegistration(), reserved: map[string]bool{}}
	s := &selectedStageFixture{version: releasepolicy.Installed{Version: "0.1.99-beta.2", Commit: strings.Repeat("8", 40)}}
	e := selectedFixtureEngine(t, f, s)
	result, err := e.handle(context.Background(), testPeer(), testRequest())
	if err != nil || result.State != brokerpipe.Staged || s.checks != 4 || s.closes != 1 {
		t.Fatal(result, err, s.checks, s.closes)
	}
}

func TestStagingSelectionUncertaintyWithholdsSuccess(t *testing.T) {
	for _, tc := range []struct {
		name     string
		openErr  bool
		failAt   int
		closeErr bool
	}{
		{"unqualified or pending", true, 0, false}, {"changed before effects", false, 1, false},
		{"changed before reservation", false, 2, false}, {"changed before materialization", false, 3, false},
		{"changed after durable stage", false, 4, false}, {"cleanup uncertain", false, 0, true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			f := &stageFixture{registration: testRegistration(), reserved: map[string]bool{}}
			s := &selectedStageFixture{version: releasepolicy.Installed{Version: "0.1.99-beta.2", Commit: strings.Repeat("8", 40)}, failAt: tc.failAt}
			e := selectedFixtureEngine(t, f, s)
			if tc.closeErr {
				s.closeErr = errors.New("retained journal close failed")
			}
			e.selected = func(context.Context, registration) (selectedExecutionOwner, error) {
				if tc.openErr {
					return s, errExecutionSelection
				}
				return s, nil
			}
			result, err := e.handle(context.Background(), testPeer(), testRequest())
			if result != (brokerpipe.Result{}) || !errors.Is(err, errStageRefused) || s.closes != 1 {
				t.Fatal(result, err, s.closes)
			}
			if (tc.openErr || tc.failAt == 1 || tc.failAt == 2) && (contains(f.events, "reserve") || contains(f.events, "prepare")) {
				t.Fatal("unqualified selection caused durable/download effects", f.events)
			}
			if (tc.failAt == 4 || tc.closeErr) && (!contains(f.events, "seal") || s.checks != 4) {
				t.Fatal("late refusal did not reach its intended durable boundary", f.events, s.checks)
			}
		})
	}
}

func TestExecutionSelectionInventoryDetectsNewAttemptsAndPartials(t *testing.T) {
	entry := func(id string) brokerstore.ExecutionJournalEntry {
		p := testSID + ".execution." + id
		return brokerstore.ExecutionJournalEntry{ExecutionID: id, EntryNames: []string{p + ".once", p + ".journal"}, Readable: true}
	}
	a, b := entry(strings.Repeat("a", 32)), entry(strings.Repeat("b", 32))
	original, err := executionSelectionInventoryDigest(testSID, []brokerstore.ExecutionJournalEntry{a})
	if err != nil {
		t.Fatal(err)
	}
	added, err := executionSelectionInventoryDigest(testSID, []brokerstore.ExecutionJournalEntry{a, b})
	if err != nil || added == original {
		t.Fatal("new attempt missed", err)
	}
	reordered, err := executionSelectionInventoryDigest(testSID, []brokerstore.ExecutionJournalEntry{b, a})
	if err != nil || added != reordered || !strings.HasSuffix(a.EntryNames[0], ".once") {
		t.Fatal("order or caller ownership", err)
	}
	for _, tc := range []struct {
		name   string
		mutate func(*brokerstore.ExecutionJournalEntry)
	}{
		{"partial", func(e *brokerstore.ExecutionJournalEntry) { e.EntryNames = e.EntryNames[:1] }},
		{"unreadable", func(e *brokerstore.ExecutionJournalEntry) { e.Readable = false }},
		{"problem", func(e *brokerstore.ExecutionJournalEntry) { e.Problem = "uncertain" }},
		{"alias", func(e *brokerstore.ExecutionJournalEntry) { e.EntryNames[0] = strings.ToUpper(e.EntryNames[0]) }},
		{"suffix fork", func(e *brokerstore.ExecutionJournalEntry) { e.EntryNames[0] += ".extra" }},
	} {
		t.Run(tc.name, func(t *testing.T) {
			bad := entry(a.ExecutionID)
			tc.mutate(&bad)
			if _, err := executionSelectionInventoryDigest(testSID, []brokerstore.ExecutionJournalEntry{bad}); err == nil {
				t.Fatal("uncertain namespace accepted")
			}
		})
	}
	if _, err := executionSelectionInventoryDigest(testSID, []brokerstore.ExecutionJournalEntry{a, a}); err == nil {
		t.Fatal("duplicate attempt accepted")
	}
}
