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

// These fixtures model retained native facts only. They do not establish cold
// process state, profile restoration, or authority to author a recovery.
func selectionPrefix(t *testing.T, attempt executionSelectionAttempt, facts []executionSelectionFact, count int) executionSelectionAttempt {
	t.Helper()
	purpose := string(brokerstore.GenerationPurposeSignedCandidate)
	if attempt.begin.PreviousHeadSHA256 == "" {
		purpose = string(brokerstore.GenerationPurposeApprovedBaseline)
	}
	originals := make([]string, count)
	for i := range originals {
		originals[i] = string(facts[i].original)
	}
	a, err := parseExecutionSelectionAttempt(testSID, attempt.begin.ExecutionID, attempt.begin.Generation, attempt.begin.CompletionSHA256, purpose, executionOriginalDigest(selectionBytes(t, originals)), facts[:count])
	if err != nil {
		t.Fatal(err)
	}
	return a
}

func selectionRecoveryFixture(t *testing.T, prior executionSelectionAttempt, count int) (executionSelectionAttempt, []executionSelectionFact) {
	t.Helper()
	id := strings.Repeat("e", 32)
	if prior.begin.ExecutionID == id {
		id = strings.Repeat("f", 32)
	}
	r := executionSelectionRecovery{
		Schema: 1, Purpose: "broker-selected-recovery-v1", ExecutionID: id, ClientSID: prior.begin.ClientSID,
		Generation: prior.begin.Generation, CompletionSHA256: prior.begin.CompletionSHA256,
		PreviousGeneration: prior.begin.PreviousGeneration, PreviousHeadSHA256: prior.begin.PreviousHeadSHA256,
		PriorExecutionID: prior.begin.ExecutionID, PriorJournalSHA256: prior.head,
	}
	facts := []executionSelectionFact{{brokerstore.ExecutionPreparation, selectionBytes(t, r)}}
	for _, step := range []struct {
		name string
		kind brokerstore.ExecutionFactKind
	}{
		{"cold-no-spawn", brokerstore.ExecutionTeardown}, {"profile-restored", brokerstore.ExecutionRestoration},
		{"restored-no-spawn", brokerstore.ExecutionRestoration},
	} {
		facts = append(facts, executionSelectionFact{step.kind, selectionBytes(t, executionSelectionStep{1, "broker-selected-recovery-step-v1", id, step.name, r.Generation, strings.Repeat("9", 64)})})
	}
	facts = facts[:count]
	purpose := string(brokerstore.GenerationPurposeSignedCandidate)
	if r.PreviousHeadSHA256 == "" {
		purpose = string(brokerstore.GenerationPurposeApprovedBaseline)
	}
	originals := make([]string, len(facts))
	for i, f := range facts {
		originals[i] = string(f.original)
	}
	a, err := parseExecutionSelectionAttempt(testSID, id, r.Generation, r.CompletionSHA256, purpose, executionOriginalDigest(selectionBytes(t, originals)), facts)
	if err != nil {
		t.Fatal(err)
	}
	return a, facts
}

func TestExecutionSelectionRecoveryPreservesSelectedBytesAndAdvancesOnlyHistory(t *testing.T) {
	root, _ := selectionFixture(t, nil, false)
	candidate, facts := selectionFixture(t, &root, false)
	for _, count := range []int{1, 2} {
		t.Run([]string{"", "before commissioning", "after commissioning"}[count], func(t *testing.T) {
			prior := selectionPrefix(t, candidate, facts, count)
			originalHead := prior.head
			recovery, _ := selectionRecoveryFixture(t, prior, 4)
			selected, pending, err := foldExecutionSelection([]executionSelectionAttempt{recovery, root, prior}, root.begin.Generation)
			if err != nil || len(pending) != 0 || selected.generation != root.begin.Generation || selected.completion != root.begin.CompletionSHA256 ||
				selected.installed.Version != root.begin.Version || selected.installed.Commit != root.begin.Commit || selected.head != recovery.head || prior.head != originalHead || prior.terminal != "" {
				t.Fatal("recovery changed selected bytes or original journal", selected, pending, err)
			}
			// A successor binds the recovery's history head, not the abandoned
			// candidate's journal or the old healthy head.
			successor, successorFacts := selectionFixture(t, &root, false)
			var begin executionSelectionBegin
			if err := json.Unmarshal(successorFacts[0].original, &begin); err != nil {
				t.Fatal(err)
			}
			begin.ExecutionID, begin.Generation = strings.Repeat("7", 32), strings.Repeat("8", 32)
			begin.PreviousHeadSHA256 = recovery.head
			successorFacts[0].original = selectionBytes(t, begin)
			for i := 1; i < len(successorFacts); i++ {
				var step executionSelectionStep
				if err := json.Unmarshal(successorFacts[i].original, &step); err != nil {
					t.Fatal(err)
				}
				step.ExecutionID, step.Generation = begin.ExecutionID, begin.Generation
				successorFacts[i].original = selectionBytes(t, step)
			}
			successor.begin = begin
			successor = selectionPrefix(t, successor, successorFacts, len(successorFacts))
			selected, pending, err = foldExecutionSelection([]executionSelectionAttempt{prior, successor, recovery, root}, root.begin.Generation)
			if err != nil || len(pending) != 0 || selected.generation != begin.Generation || selected.head != successor.head {
				t.Fatal("fresh independently healthy successor unavailable", selected, pending, err)
			}
		})
	}
}

func TestExecutionSelectionRecoveryRootRequiresFreshHealthyQualification(t *testing.T) {
	qualified, facts := selectionFixture(t, nil, false)
	prior := selectionPrefix(t, qualified, facts, 2)
	recovery, _ := selectionRecoveryFixture(t, prior, 4)
	selected, pending, err := foldExecutionSelection([]executionSelectionAttempt{prior, recovery}, prior.begin.Generation)
	if !errors.Is(err, errExecutionSelection) || len(pending) != 0 || selected != (executionSelectedVersion{}) {
		t.Fatal("root recovery invented qualification", selected, pending, err)
	}
	// A fresh root identity must independently repeat commissioning, consumed
	// spawn and healthy selection. Recovery occupies no root selection slot.
	freshFacts := append([]executionSelectionFact(nil), facts...)
	var begin executionSelectionBegin
	if err := json.Unmarshal(freshFacts[0].original, &begin); err != nil {
		t.Fatal(err)
	}
	begin.ExecutionID = strings.Repeat("6", 32)
	freshFacts[0].original = selectionBytes(t, begin)
	for i := 1; i < len(freshFacts); i++ {
		var step executionSelectionStep
		if err := json.Unmarshal(freshFacts[i].original, &step); err != nil {
			t.Fatal(err)
		}
		step.ExecutionID = begin.ExecutionID
		freshFacts[i].original = selectionBytes(t, step)
	}
	fresh := qualified
	fresh.begin = begin
	for count := 1; count <= len(freshFacts); count++ {
		a := selectionPrefix(t, fresh, freshFacts, count)
		selected, pending, err = foldExecutionSelection([]executionSelectionAttempt{recovery, a, prior}, prior.begin.Generation)
		if count < len(freshFacts) {
			if err == nil || len(pending) != 1 || selected.generation != "" {
				t.Fatal("retry root prefix invented qualification", count, selected, pending, err)
			}
		} else if err != nil || len(pending) != 0 || selected.generation != prior.begin.Generation || selected.head != a.head {
			t.Fatal("fresh actual qualification unavailable", selected, pending, err)
		}
	}
}

func TestExecutionSelectionRecoveryPartialRemainsPending(t *testing.T) {
	root, _ := selectionFixture(t, nil, false)
	candidate, facts := selectionFixture(t, &root, false)
	prior := selectionPrefix(t, candidate, facts, 2)
	for count := 1; count < 4; count++ {
		recovery, _ := selectionRecoveryFixture(t, prior, count)
		selected, pending, err := foldExecutionSelection([]executionSelectionAttempt{root, recovery, prior}, root.begin.Generation)
		if !errors.Is(err, errExecutionSelection) || len(pending) != 2 || selected.generation != root.begin.Generation || selected.head != root.head {
			t.Fatal("incomplete recovery hid original pending attempt", count, selected, pending, err)
		}
	}
}

func TestExecutionSelectionRecoveryRefusesConsumedOrMismatchedLinks(t *testing.T) {
	root, _ := selectionFixture(t, nil, false)
	candidate, facts := selectionFixture(t, &root, false)
	prior := selectionPrefix(t, candidate, facts, 2)
	recovery, _ := selectionRecoveryFixture(t, prior, 4)
	for _, tc := range []struct {
		name   string
		change func(*executionSelectionAttempt, *executionSelectionAttempt)
	}{
		{"unknown prior", func(_ *executionSelectionAttempt, r *executionSelectionAttempt) {
			r.recovery.PriorExecutionID = strings.Repeat("9", 32)
		}},
		{"wrong original digest", func(_ *executionSelectionAttempt, r *executionSelectionAttempt) {
			r.recovery.PriorJournalSHA256 = strings.Repeat("0", 64)
		}},
		{"wrong sid", func(_ *executionSelectionAttempt, r *executionSelectionAttempt) { r.recovery.ClientSID += "-1" }},
		{"wrong generation", func(_ *executionSelectionAttempt, r *executionSelectionAttempt) {
			r.recovery.Generation = strings.Repeat("9", 32)
		}},
		{"wrong completion", func(_ *executionSelectionAttempt, r *executionSelectionAttempt) {
			r.recovery.CompletionSHA256 = strings.Repeat("9", 64)
		}},
		{"wrong previous generation", func(_ *executionSelectionAttempt, r *executionSelectionAttempt) {
			r.recovery.PreviousGeneration = strings.Repeat("9", 32)
		}},
		{"wrong selected head", func(_ *executionSelectionAttempt, r *executionSelectionAttempt) {
			r.recovery.PreviousHeadSHA256 = strings.Repeat("9", 64)
		}},
		{"consumed flag", func(p *executionSelectionAttempt, _ *executionSelectionAttempt) { p.consumed = true }},
		{"nested recovery", func(p *executionSelectionAttempt, _ *executionSelectionAttempt) {
			p.recovery = &executionSelectionRecovery{}
		}},
		{"terminal prior", func(p *executionSelectionAttempt, _ *executionSelectionAttempt) { p.terminal = "selected" }},
	} {
		t.Run(tc.name, func(t *testing.T) {
			p, r := prior, recovery
			copy := *recovery.recovery
			r.recovery = &copy
			tc.change(&p, &r)
			if _, _, err := foldExecutionSelection([]executionSelectionAttempt{root, p, r}, root.begin.Generation); err == nil {
				t.Fatal("unsafe recovery linked")
			}
		})
	}
	// Every valid ordinary state at or after consumed create is excluded,
	// including healthy uncertainty and each rollback/fallback prefix.
	for _, rollback := range []bool{false, true} {
		a, f := selectionFixture(t, &root, rollback)
		for count := 3; count <= len(f); count++ {
			p := selectionPrefix(t, a, f, count)
			r, _ := selectionRecoveryFixture(t, p, 4)
			if _, _, err := foldExecutionSelection([]executionSelectionAttempt{root, p, r}, root.begin.Generation); err == nil {
				t.Fatal("consumed ordinary chain became no-spawn abort", rollback, count, p.state)
			}
		}
	}
	fork := candidate
	fork.begin.ExecutionID, fork.head = strings.Repeat("8", 32), strings.Repeat("8", 64)
	for _, tc := range []struct {
		name     string
		attempts []executionSelectionAttempt
	}{
		{"missing prior", []executionSelectionAttempt{root, recovery}},
		{"duplicate recovery", []executionSelectionAttempt{root, prior, recovery, recovery}},
		{"old healthy head fork", []executionSelectionAttempt{root, prior, recovery, fork}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			if _, _, err := foldExecutionSelection(tc.attempts, root.begin.Generation); err == nil {
				t.Fatal("forked recovery history accepted")
			}
		})
	}
	other := recovery
	copy := *recovery.recovery
	other.recovery = &copy
	other.begin.ExecutionID, other.head = strings.Repeat("f", 32), strings.Repeat("f", 64)
	other.recovery.ExecutionID = other.begin.ExecutionID
	if _, _, err := foldExecutionSelection([]executionSelectionAttempt{root, prior, recovery, other}, root.begin.Generation); err == nil {
		t.Fatal("two fresh recovery journals targeted the same original")
	}
	partial, _ := selectionRecoveryFixture(t, prior, 2)
	if _, _, err := foldExecutionSelection([]executionSelectionAttempt{root, prior, partial, other}, root.begin.Generation); err == nil {
		t.Fatal("partial recovery was superseded by another recovery")
	}
	otherPrior := prior
	otherPrior.begin.ExecutionID, otherPrior.head = strings.Repeat("7", 32), strings.Repeat("7", 64)
	other.recovery.PriorExecutionID, other.recovery.PriorJournalSHA256 = otherPrior.begin.ExecutionID, otherPrior.head
	if _, _, err := foldExecutionSelection([]executionSelectionAttempt{root, prior, recovery, otherPrior, other}, root.begin.Generation); err == nil {
		t.Fatal("overlapping recoveries forked the same selected head")
	}
	unreachablePrior := prior
	unreachablePrior.begin.PreviousHeadSHA256 = strings.Repeat("6", 64)
	unreachableRecovery, _ := selectionRecoveryFixture(t, unreachablePrior, 4)
	if _, _, err := foldExecutionSelection([]executionSelectionAttempt{root, unreachablePrior, unreachableRecovery}, root.begin.Generation); err == nil {
		t.Fatal("unreachable recovery history accepted")
	}
}

func TestExecutionSelectionRecoveryExactClosedSchemaAndOrder(t *testing.T) {
	root, _ := selectionFixture(t, nil, false)
	candidate, candidateFacts := selectionFixture(t, &root, false)
	prior := selectionPrefix(t, candidate, candidateFacts, 1)
	recovery, facts := selectionRecoveryFixture(t, prior, 4)
	parse := func(f []executionSelectionFact) error {
		_, err := parseExecutionSelectionAttempt(testSID, recovery.begin.ExecutionID, recovery.begin.Generation, recovery.begin.CompletionSHA256,
			string(brokerstore.GenerationPurposeSignedCandidate), recovery.head, f)
		return err
	}
	for _, tc := range []struct {
		name   string
		change func([]executionSelectionFact) []executionSelectionFact
	}{
		{"missing cold proof", func(f []executionSelectionFact) []executionSelectionFact { return append(f[:1:1], f[2:]...) }},
		{"restore before cold", func(f []executionSelectionFact) []executionSelectionFact { f[1], f[2] = f[2], f[1]; return f }},
		{"duplicate step", func(f []executionSelectionFact) []executionSelectionFact { return append(f[:2:2], f[1:]...) }},
		{"step after terminal", func(f []executionSelectionFact) []executionSelectionFact { return append(f, f[3]) }},
		{"wrong native fact kind", func(f []executionSelectionFact) []executionSelectionFact {
			f[1].kind = brokerstore.ExecutionHealthObserved
			return f
		}},
		{"health key in origin", func(f []executionSelectionFact) []executionSelectionFact {
			f[0].original = append(append([]byte(nil), f[0].original[:len(f[0].original)-1]...), []byte(",\"healthy\":true}")...)
			return f
		}},
		{"duplicate origin key", func(f []executionSelectionFact) []executionSelectionFact {
			f[0].original = append(append([]byte(nil), f[0].original[:len(f[0].original)-1]...), []byte(",\"schema\":1}")...)
			return f
		}},
		{"transaction step purpose", func(f []executionSelectionFact) []executionSelectionFact {
			f[1].original = []byte(strings.ReplaceAll(string(f[1].original), "broker-selected-recovery-step-v1", "broker-selected-step-v1"))
			return f
		}},
		{"wrong target", func(f []executionSelectionFact) []executionSelectionFact {
			var step executionSelectionStep
			_ = json.Unmarshal(f[2].original, &step)
			step.Generation = root.begin.Generation
			f[2].original = selectionBytes(t, step)
			return f
		}},
		{"bad evidence hash", func(f []executionSelectionFact) []executionSelectionFact {
			var step executionSelectionStep
			_ = json.Unmarshal(f[2].original, &step)
			step.EvidenceSHA256 = ""
			f[2].original = selectionBytes(t, step)
			return f
		}},
		{"self recovery", func(f []executionSelectionFact) []executionSelectionFact {
			var r executionSelectionRecovery
			_ = json.Unmarshal(f[0].original, &r)
			r.PriorExecutionID = r.ExecutionID
			f[0].original = selectionBytes(t, r)
			return f
		}},
		{"missing original hash", func(f []executionSelectionFact) []executionSelectionFact {
			var r executionSelectionRecovery
			_ = json.Unmarshal(f[0].original, &r)
			r.PriorJournalSHA256 = ""
			f[0].original = selectionBytes(t, r)
			return f
		}},
		{"mixed root lineage", func(f []executionSelectionFact) []executionSelectionFact {
			var r executionSelectionRecovery
			_ = json.Unmarshal(f[0].original, &r)
			r.PreviousGeneration = ""
			f[0].original = selectionBytes(t, r)
			return f
		}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			if err := parse(tc.change(append([]executionSelectionFact(nil), facts...))); !errors.Is(err, errExecutionSelection) {
				t.Fatal("unsafe schema/order accepted", err)
			}
		})
	}
}
