package brokerhost

import (
	"context"
	"errors"
	"strings"
	"testing"

	"example.com/opennav-update-verifier/brokerstore"
)

func manualRollbackFixture(t *testing.T) (executionSelectionAttempt, executionSelectionAttempt, executionManualRollbackBegin, []executionSelectionFact) {
	t.Helper()
	root, _ := selectionFixture(t, nil, false)
	current, _ := selectionFixture(t, &root, false)
	b := executionManualRollbackBegin{executionSelectionBegin: root.begin,
		QualifiedHeadSHA256: root.head, QualificationSHA256: root.health, QualificationClosedSHA256: root.closed,
		CurrentProfileSHA256: strings.Repeat("6", 64), CurrentProfileIdentitySHA256: strings.Repeat("7", 64)}
	b.Purpose, b.ExecutionID = "broker-manual-rollback-v1", strings.Repeat("1", 32)
	b.PreviousGeneration, b.PreviousHeadSHA256 = current.begin.Generation, current.head
	facts := []executionSelectionFact{{brokerstore.ExecutionPreparation, selectionBytes(t, b)}}
	steps := []struct {
		name, evidence string
		kind           brokerstore.ExecutionFactKind
	}{
		{"current-profile-frozen", b.CurrentProfileIdentitySHA256, brokerstore.ExecutionCommissioning},
		{"qualified-ancestor", b.QualificationSHA256, brokerstore.ExecutionHealthObserved},
		{"ancestor-qualification-closed", b.QualificationClosedSHA256, brokerstore.ExecutionTeardown},
		{"manual-selected", b.QualifiedHeadSHA256, brokerstore.ExecutionHealthObserved},
	}
	for _, s := range steps {
		facts = append(facts, executionSelectionFact{s.kind, selectionBytes(t, executionManualRollbackStep{
			1, "broker-manual-rollback-step-v1", b.ExecutionID, s.name, b.Generation, b.QualifiedHeadSHA256,
			s.evidence, b.CurrentProfileSHA256, b.CurrentProfileIdentitySHA256})})
	}
	return root, current, b, facts
}

func parseManualFixture(t *testing.T, b executionManualRollbackBegin, facts []executionSelectionFact) executionSelectionAttempt {
	t.Helper()
	a, err := parseExecutionSelectionAttempt(testSID, b.ExecutionID, b.Generation, b.CompletionSHA256,
		string(brokerstore.GenerationPurposeApprovedBaseline), strings.Repeat("8", 64), facts)
	if err != nil {
		t.Fatal(err)
	}
	return a
}

func TestExecutionManualRollbackPreservesHistoricalQualificationAndCurrentProfileBinding(t *testing.T) {
	root, current, b, facts := manualRollbackFixture(t)
	a := parseManualFixture(t, b, facts)
	selected, pending, err := foldExecutionSelection([]executionSelectionAttempt{current, a, root}, root.begin.Generation)
	if err != nil || len(pending) != 0 || selected.generation != root.begin.Generation || selected.head != a.head ||
		selected.completion != root.begin.CompletionSHA256 || selected.installed != ancestorInstalled(root) || a.consumed ||
		a.manual.CurrentProfileSHA256 != b.CurrentProfileSHA256 || a.manual.QualifiedHeadSHA256 != root.head {
		t.Fatal("manual ancestor transition differs", selected, pending, err)
	}
	// Every durable prefix is pending. It never resolves by copying historical
	// profile bytes, interpreting a failed-start rollback, or assuming no PID.
	for i := 1; i < len(facts); i++ {
		prefix := parseManualFixture(t, b, facts[:i])
		s, p, err := foldExecutionSelection([]executionSelectionAttempt{root, current, prefix}, root.begin.Generation)
		if err == nil || len(p) != 1 || s.generation != current.begin.Generation || prefix.terminal != "" {
			t.Fatal("manual prefix changed selection", i, s, p, err)
		}
	}
}

func TestExecutionManualRollbackStrictFactsRefuseChangedEvidenceAndAliases(t *testing.T) {
	_, _, b, original := manualRollbackFixture(t)
	for _, tc := range []struct {
		name   string
		mutate func([]executionSelectionFact) []executionSelectionFact
	}{
		{"missing profile freeze", func(f []executionSelectionFact) []executionSelectionFact { return append(f[:1:1], f[2:]...) }},
		{"missing original close", func(f []executionSelectionFact) []executionSelectionFact { return append(f[:3:3], f[4:]...) }},
		{"duplicate health", func(f []executionSelectionFact) []executionSelectionFact { return append(f[:3:3], f[2], f[3]) }},
		{"trailing effect", func(f []executionSelectionFact) []executionSelectionFact { return append(f, f[4]) }},
		{"wrong kind", func(f []executionSelectionFact) []executionSelectionFact {
			f[3].kind = brokerstore.ExecutionRestoration
			return f
		}},
		{"unknown begin field", func(f []executionSelectionFact) []executionSelectionFact {
			f[0].original = append(append([]byte(nil), f[0].original[:len(f[0].original)-1]...), []byte(`,"healthy":true}`)...)
			return f
		}},
		{"duplicate field", func(f []executionSelectionFact) []executionSelectionFact {
			f[1].original = append(append([]byte(nil), f[1].original[:len(f[1].original)-1]...), []byte(`,"schema":1}`)...)
			return f
		}},
		{"case alias", func(f []executionSelectionFact) []executionSelectionFact {
			f[1].original = []byte(strings.Replace(string(f[1].original), `"schema"`, `"Schema"`, 1))
			return f
		}},
		{"changed current bytes", func(f []executionSelectionFact) []executionSelectionFact {
			f[2].original = []byte(strings.ReplaceAll(string(f[2].original), b.CurrentProfileSHA256, strings.Repeat("0", 64)))
			return f
		}},
		{"changed current IDs", func(f []executionSelectionFact) []executionSelectionFact {
			f[3].original = []byte(strings.ReplaceAll(string(f[3].original), b.CurrentProfileIdentitySHA256, strings.Repeat("0", 64)))
			return f
		}},
		{"changed health evidence", func(f []executionSelectionFact) []executionSelectionFact {
			f[2].original = []byte(strings.ReplaceAll(string(f[2].original), b.QualificationSHA256, strings.Repeat("0", 64)))
			return f
		}},
		{"old recovery purpose", func(f []executionSelectionFact) []executionSelectionFact {
			f[0].original = []byte(strings.ReplaceAll(string(f[0].original), "broker-manual-rollback-v1", "broker-selected-recovery-v1"))
			return f
		}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			f := tc.mutate(append([]executionSelectionFact(nil), original...))
			if _, err := parseExecutionSelectionAttempt(testSID, b.ExecutionID, b.Generation, b.CompletionSHA256,
				string(brokerstore.GenerationPurposeApprovedBaseline), strings.Repeat("8", 64), f); err == nil {
				t.Fatal("invalid manual evidence accepted")
			}
		})
	}
}

func TestExecutionManualRollbackRefusesUnqualifiedForkedReplayedAndWrongHead(t *testing.T) {
	root, current, b, facts := manualRollbackFixture(t)
	manual := parseManualFixture(t, b, facts)
	for _, tc := range []struct {
		name   string
		mutate func(*executionSelectionAttempt)
	}{
		{"wrong current head", func(a *executionSelectionAttempt) { a.begin.PreviousHeadSHA256 = strings.Repeat("0", 64) }},
		{"wrong predecessor", func(a *executionSelectionAttempt) { a.begin.PreviousGeneration = strings.Repeat("0", 32) }},
		{"unknown qualifier", func(a *executionSelectionAttempt) { a.manual.QualifiedHeadSHA256 = strings.Repeat("0", 64) }},
		{"wrong target completion", func(a *executionSelectionAttempt) { a.manual.CompletionSHA256 = strings.Repeat("0", 64) }},
		{"wrong target version", func(a *executionSelectionAttempt) { a.manual.Version = "0.0.1" }},
		{"wrong target commit", func(a *executionSelectionAttempt) { a.manual.Commit = strings.Repeat("0", 40) }},
		{"foreign client", func(a *executionSelectionAttempt) { a.manual.ClientSID = "S-1-5-21-10-20-30-1002" }},
		{"unknown original close", func(a *executionSelectionAttempt) { a.manual.QualificationClosedSHA256 = strings.Repeat("0", 64) }},
	} {
		t.Run(tc.name, func(t *testing.T) {
			a := manual
			copied := *a.manual
			a.manual = &copied
			tc.mutate(&a)
			if _, _, err := foldExecutionSelection([]executionSelectionAttempt{root, current, a}, root.begin.Generation); err == nil {
				t.Fatal("changed manual lineage accepted")
			}
		})
	}
	if _, _, err := foldExecutionSelection([]executionSelectionAttempt{root, current, manual, manual}, root.begin.Generation); err == nil {
		t.Fatal("replayed manual selection")
	}
	fork := manual
	fork.begin.ExecutionID, fork.head = strings.Repeat("2", 32), strings.Repeat("2", 64)
	if _, _, err := foldExecutionSelection([]executionSelectionAttempt{root, current, manual, fork}, root.begin.Generation); err == nil {
		t.Fatal("forked manual selection")
	}
	q := ManualRollbackRequest{b.ExecutionID, current.head, root.begin.Generation, root.begin.CompletionSHA256}
	if target, err := manualRollbackAncestor([]executionSelectionAttempt{root, current}, root.begin.Generation, q); err != nil || target.head != root.head {
		t.Fatal(target, err)
	}
	q.RequestID = current.begin.ExecutionID
	if _, err := manualRollbackAncestor([]executionSelectionAttempt{root, current}, root.begin.Generation, q); err == nil {
		t.Fatal("replayed request admitted")
	}
	q.RequestID, q.TargetGeneration, q.TargetCompletionSHA256 = b.ExecutionID, current.begin.Generation, current.begin.CompletionSHA256
	if _, err := manualRollbackAncestor([]executionSelectionAttempt{root, current}, root.begin.Generation, q); err == nil {
		t.Fatal("current target admitted")
	}
	failed, _ := selectionFixture(t, &root, true)
	q.ExpectedSelectedHeadSHA256 = failed.head
	if _, err := manualRollbackAncestor([]executionSelectionAttempt{root, failed}, root.begin.Generation, q); err == nil {
		t.Fatal("failed-start target became qualified")
	}
}

type manualRollbackRunFixture struct {
	run                                                 executionManualRollbackRun
	facts                                               []executionSelectionFact
	events                                              []string
	checks, profileChecks, profileCloses, journalCloses int
	failCheck, failProfile, failAppend                  int
	resolveFault, profileCloseFault, journalCloseFault  bool
	profile                                             manualRollbackProfileBinding
}

func newManualRollbackRunFixture(t *testing.T) *manualRollbackRunFixture {
	t.Helper()
	_, _, b, _ := manualRollbackFixture(t)
	f := &manualRollbackRunFixture{profile: manualRollbackProfileBinding{b.CurrentProfileSHA256, b.CurrentProfileIdentitySHA256}}
	f.run = executionManualRollbackRun{begin: b, journal: f, profile: f, qualification: b.QualificationSHA256, qualificationClosed: b.QualificationClosedSHA256,
		validate: func(ctx context.Context) error {
			f.checks++
			f.events = append(f.events, "history")
			if f.checks == f.failCheck {
				return errors.New("original history changed")
			}
			return ctx.Err()
		},
		resolve: func(context.Context) error {
			f.events = append(f.events, "resolve")
			if f.resolveFault {
				return errors.New("native selection reopen failed")
			}
			return nil
		}}
	return f
}
func (f *manualRollbackRunFixture) validate(ctx context.Context) (manualRollbackProfileBinding, error) {
	f.profileChecks++
	f.events = append(f.events, "profile")
	if f.profileChecks == f.failProfile {
		return manualRollbackProfileBinding{}, errors.New("current tree changed")
	}
	return f.profile, ctx.Err()
}
func (f *manualRollbackRunFixture) close() error {
	f.profileCloses++
	f.events = append(f.events, "close-profile")
	if f.profileCloseFault {
		return errors.New("profile cleanup uncertain")
	}
	return nil
}
func (f *manualRollbackRunFixture) AppendFact(ctx context.Context, kind brokerstore.ExecutionFactKind, raw []byte) error {
	f.events = append(f.events, "append")
	f.facts = append(f.facts, executionSelectionFact{kind, append([]byte(nil), raw...)})
	if len(f.facts) == f.failAppend {
		return errors.New("durable append uncertain")
	}
	return ctx.Err()
}
func (f *manualRollbackRunFixture) Close() error {
	f.journalCloses++
	f.events = append(f.events, "close-journal")
	if f.journalCloseFault {
		return errors.New("journal cleanup uncertain")
	}
	return nil
}

func TestExecutionManualRollbackWriterCommitsOnlyExactRetainedProof(t *testing.T) {
	f := newManualRollbackRunFixture(t)
	before := f.profile
	if err := f.run.execute(context.Background()); err != nil {
		t.Fatal(err)
	}
	if len(f.facts) != 5 || f.profile != before || f.profileCloses != 1 || f.journalCloses != 1 || f.checks != 7 || f.profileChecks != 7 {
		t.Fatal("wrong writer effects", f.events)
	}
	a := parseManualFixture(t, f.run.begin, f.facts)
	root, current, _, _ := manualRollbackFixture(t)
	if s, _, err := foldExecutionSelection([]executionSelectionAttempt{root, current, a}, root.begin.Generation); err != nil || s.generation != root.begin.Generation {
		t.Fatal(s, err)
	}
	for i, e := range f.events {
		if e == "append" && (i < 2 || f.events[i-1] != "profile" || f.events[i-2] != "history") {
			t.Fatal("effect before original history/current profile checks", f.events)
		}
	}
	count := len(f.events)
	if f.run.execute(context.Background()) == nil || len(f.events) != count {
		t.Fatal("manual operation retried")
	}
}

func TestExecutionManualRollbackWriterFaultsNeverRestoreRetryOrResolvePrefixes(t *testing.T) {
	for _, kind := range []string{"history", "profile", "append"} {
		limit := 7
		if kind == "append" {
			limit = 5
		}
		for at := 1; at <= limit; at++ {
			t.Run(kind+string(rune('0'+at)), func(t *testing.T) {
				f := newManualRollbackRunFixture(t)
				switch kind {
				case "history":
					f.failCheck = at
				case "profile":
					f.failProfile = at
				case "append":
					f.failAppend = at
				}
				err := f.run.execute(context.Background())
				if err == nil || f.profileCloses != 1 || f.journalCloses != 1 {
					t.Fatal("failure lost retained owners", f.events, err)
				}
				if len(f.facts) > 0 && !errors.Is(err, ErrManualRollbackUncertain) {
					t.Fatal("mutation ambiguity lost", err)
				}
				if len(f.facts) < 5 {
					for _, e := range f.events {
						if e == "resolve" {
							t.Fatal("prefix resolved selection")
						}
					}
				}
				count := len(f.events)
				if f.run.execute(context.Background()) == nil || len(f.events) != count {
					t.Fatal("failed operation retried")
				}
			})
		}
	}
	for _, kind := range []string{"resolve", "profile-close", "journal-close", "both-close"} {
		t.Run(kind, func(t *testing.T) {
			f := newManualRollbackRunFixture(t)
			switch kind {
			case "resolve":
				f.resolveFault = true
			case "profile-close":
				f.profileCloseFault = true
			case "journal-close":
				f.journalCloseFault = true
			case "both-close":
				f.profileCloseFault = true
				f.journalCloseFault = true
			}
			if err := f.run.execute(context.Background()); !errors.Is(err, ErrManualRollbackUncertain) || f.profileCloses != 1 || f.journalCloses != 1 {
				t.Fatal(err, f.events)
			}
		})
	}
	for _, kind := range []string{"nil-context", "cancelled", "changed-bytes", "changed-IDs", "bad-begin"} {
		t.Run(kind, func(t *testing.T) {
			f := newManualRollbackRunFixture(t)
			ctx := context.Background()
			switch kind {
			case "nil-context":
				ctx = nil
			case "cancelled":
				var cancel context.CancelFunc
				ctx, cancel = context.WithCancel(ctx)
				cancel()
			case "changed-bytes":
				f.profile.contents = strings.Repeat("0", 64)
			case "changed-IDs":
				f.profile.identity = strings.Repeat("0", 64)
			case "bad-begin":
				f.run.begin.ClientSID = "current-user"
			}
			if f.run.execute(ctx) == nil || len(f.facts) != 0 {
				t.Fatal("unapproved input mutated journal", f.events)
			}
		})
	}
}
