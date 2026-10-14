package brokerhost

import (
	"strings"
	"testing"

	"example.com/opennav-update-verifier/brokerstore"
)

func repairContinuationFixture(t *testing.T, prior executionSelectionAttempt, id string, count int) (executionSelectionAttempt, []executionSelectionFact) {
	t.Helper()
	b := *prior.repair
	b.Purpose = "broker-selected-repair-recovery-v1"
	b.ExecutionID = id
	b.PriorExecutionID = prior.begin.ExecutionID
	b.PriorJournalSHA256 = prior.head
	b.CurrentProfileSHA256 = strings.Repeat("a", 64)
	b.CurrentProfileIdentitySHA256 = strings.Repeat("b", 64)
	if prior.repairEmpty {
		b.SourceQualifiedHeadSHA256 = strings.Repeat("c", 64)
	}
	facts := []executionSelectionFact{{brokerstore.ExecutionPreparation, selectionBytes(t, b)}}
	for _, step := range []string{"payload-restored", "repair-selected"} {
		facts = append(facts, executionSelectionFact{brokerstore.ExecutionRestoration, selectionBytes(t, executionSelectionStep{1, "broker-selected-repair-step-v1", id, step, b.Generation, b.CompletionSHA256})})
	}
	facts = facts[:count]
	head := executionOriginalDigest(selectionBytes(t, []string{id, string(facts[0].original)}))
	a, err := parseExecutionSelectionAttempt(testSID, id, b.Generation, b.CompletionSHA256, string(brokerstore.GenerationPurposeRepaired), head, facts)
	if err != nil {
		t.Fatal(err)
	}
	return a, facts
}
func TestInterruptedRepairContinuesOnlyWithFreshExactLinkedJournal(t *testing.T) {
	baseline, _ := selectionFixture(t, nil, false)
	complete, facts := repairSelectionFixture(t, baseline, baseline, strings.Repeat("6", 32))
	for prefix := 1; prefix <= 2; prefix++ {
		t.Run(string(rune('0'+prefix)), func(t *testing.T) {
			pending, err := parseExecutionSelectionAttempt(testSID, complete.begin.ExecutionID, complete.begin.Generation, complete.begin.CompletionSHA256, string(brokerstore.GenerationPurposeRepaired), strings.Repeat("7", 64), facts[:prefix])
			if err != nil {
				t.Fatal(err)
			}
			attempts := []executionSelectionAttempt{baseline, pending}
			selected, interrupted, err := foldExecutionRepairSelection(attempts, baseline.begin.Generation)
			if err != nil || interrupted == nil || interrupted.begin.ExecutionID != pending.begin.ExecutionID || selected.generation != baseline.begin.Generation {
				t.Fatal("explicit repair cannot inspect exact pending copy", err)
			}
			if _, _, err = foldExecutionSelection(attempts, baseline.begin.Generation); err == nil {
				t.Fatal("ordinary startup admitted pending repair")
			}
			continued, _ := repairContinuationFixture(t, pending, strings.Repeat("8", 32), 3)
			selected, remaining, err := foldExecutionSelection(append(attempts, continued), baseline.begin.Generation)
			if err != nil || len(remaining) != 0 || selected.generation != pending.begin.Generation || selected.installed != ancestorInstalled(baseline) || continued.health != "" || continued.closed != "" {
				t.Fatal("exact continuation not selected without fabricated health", err)
			}
			if len(remaining) != 0 {
				t.Fatal("prior journal still pending after exact terminal continuation")
			}
			// Interruption during the continuation is itself one exact pending chain,
			// with old selection intact and no reopened append into either old journal.
			midway, _ := repairContinuationFixture(t, pending, strings.Repeat("9", 32), 1)
			chain := append(append([]executionSelectionAttempt{}, attempts...), midway)
			selected, latest, err := foldExecutionRepairSelection(chain, baseline.begin.Generation)
			if err != nil || latest == nil || latest.begin.ExecutionID != midway.begin.ExecutionID || selected.generation != baseline.begin.Generation {
				t.Fatal("interrupted continuation cannot be inspected", err)
			}
			last, _ := repairContinuationFixture(t, midway, strings.Repeat("a", 32), 3)
			if final, _, err := foldExecutionSelection(append(chain, last), baseline.begin.Generation); err != nil || final.generation != pending.begin.Generation {
				t.Fatal("bounded exact continuation chain failed", err)
			}
		})
	}
}
func TestRepairRecoveryRejectsForkMismatchAndOrdinaryPendingTransactions(t *testing.T) {
	baseline, _ := selectionFixture(t, nil, false)
	repair, facts := repairSelectionFixture(t, baseline, baseline, strings.Repeat("6", 32))
	pending, err := parseExecutionSelectionAttempt(testSID, repair.begin.ExecutionID, repair.begin.Generation, repair.begin.CompletionSHA256, string(brokerstore.GenerationPurposeRepaired), strings.Repeat("7", 64), facts[:1])
	if err != nil {
		t.Fatal(err)
	}
	recovery, _ := repairContinuationFixture(t, pending, strings.Repeat("8", 32), 3)
	for _, fault := range []string{"journal-hash", "target-completion", "source-completion", "predecessor", "fork", "cycle"} {
		t.Run(fault, func(t *testing.T) {
			a := recovery
			b := *recovery.repair
			a.repair = &b
			a.begin = b.executionSelectionBegin
			attempts := []executionSelectionAttempt{baseline, pending}
			switch fault {
			case "journal-hash":
				b.PriorJournalSHA256 = strings.Repeat("f", 64)
			case "target-completion":
				b.CompletionSHA256 = strings.Repeat("f", 64)
				a.begin = b.executionSelectionBegin
			case "source-completion":
				b.SourceCompletionSHA256 = strings.Repeat("f", 64)
			case "predecessor":
				b.PreviousCompletionSHA256 = strings.Repeat("f", 64)
			case "fork":
				other, _ := repairContinuationFixture(t, pending, strings.Repeat("9", 32), 3)
				attempts = append(attempts, other)
			case "cycle":
				changed := pending
				c := *pending.repair
				changed.repair = &c
				c.PriorExecutionID = a.begin.ExecutionID
				c.PriorJournalSHA256 = a.head
				attempts[1] = changed
			}
			if _, _, err := foldExecutionSelection(append(attempts, a), baseline.begin.Generation); err == nil {
				t.Fatal("ambiguous or mismatched recovery selected")
			}
		})
	}
	ordinary, normalFacts := selectionFixture(t, &baseline, false)
	ordinary, err = parseExecutionSelectionAttempt(testSID, ordinary.begin.ExecutionID, ordinary.begin.Generation, ordinary.begin.CompletionSHA256, string(brokerstore.GenerationPurposeSignedCandidate), ordinary.head, normalFacts[:1])
	if err != nil {
		t.Fatal(err)
	}
	if _, _, err = foldExecutionRepairSelection([]executionSelectionAttempt{baseline, ordinary}, baseline.begin.Generation); err == nil {
		t.Fatal("Repair reinterpreted interrupted Upgrade as its own no-launch copy")
	}
}
func TestRepairEmptyOriginIsPendingProvenanceAndNeverHealth(t *testing.T) {
	baseline, _ := selectionFixture(t, nil, false)
	repair, _ := repairSelectionFixture(t, baseline, baseline, strings.Repeat("6", 32))
	empty := repair
	b := *repair.repair
	empty.repair = &b
	empty.terminal = ""
	empty.state = "repair-empty"
	empty.repairEmpty = true
	b.Purpose = "broker-empty-repair-v1"
	b.CurrentProfileSHA256 = ""
	b.CurrentProfileIdentitySHA256 = ""
	b.SourceQualifiedHeadSHA256 = ""
	selected, pending, err := foldExecutionRepairSelection([]executionSelectionAttempt{baseline, empty}, baseline.begin.Generation)
	if err != nil || pending == nil || selected.generation != baseline.begin.Generation {
		t.Fatal("authenticated empty origin cannot be inspected", err)
	}
	continued, _ := repairContinuationFixture(t, empty, strings.Repeat("8", 32), 3)
	continued.repair.SourceQualifiedHeadSHA256 = baseline.head // derived reachable ORIGINAL qualifier
	if final, _, err := foldExecutionSelection([]executionSelectionAttempt{baseline, empty, continued}, baseline.begin.Generation); err != nil || final.generation != empty.begin.Generation {
		t.Fatal("empty-origin continuation failed", err)
	}
}

func TestRepairContinuationEmptyBornJournalSupersedesOnlyItsExactPrior(t *testing.T) {
	baseline, _ := selectionFixture(t, nil, false)
	complete, facts := repairSelectionFixture(t, baseline, baseline, strings.Repeat("6", 32))
	original, err := parseExecutionSelectionAttempt(testSID, complete.begin.ExecutionID, complete.begin.Generation, complete.begin.CompletionSHA256, string(brokerstore.GenerationPurposeRepaired), strings.Repeat("7", 64), facts[:1])
	if err != nil {
		t.Fatal(err)
	}
	// Fields below come from the authentic initial fence in production, not a
	// preparation fact; storage/NTFS tests exercise actual birth/read propagation.
	empty, _ := repairContinuationFixture(t, original, strings.Repeat("8", 32), 1)
	b := *empty.repair
	empty.repair = &b
	empty.repairEmpty = true
	empty.state = "repair-empty"
	b.Purpose = "broker-empty-repair-v1"
	b.CurrentProfileSHA256 = ""
	b.CurrentProfileIdentitySHA256 = ""
	b.SourceQualifiedHeadSHA256 = ""
	attempts := []executionSelectionAttempt{baseline, original, empty}
	selected, latest, err := foldExecutionRepairSelection(attempts, baseline.begin.Generation)
	if err != nil || latest == nil || latest.begin.ExecutionID != empty.begin.ExecutionID || selected.generation != baseline.begin.Generation {
		t.Fatal("crash after recovery origin left two permanently pending journals", err)
	}
	continued, _ := repairContinuationFixture(t, empty, strings.Repeat("9", 32), 3)
	continued.repair.SourceQualifiedHeadSHA256 = baseline.head
	if final, _, err := foldExecutionSelection(append(attempts, continued), baseline.begin.Generation); err != nil || final.generation != original.begin.Generation {
		t.Fatal("exact empty-origin chain could not finish", err)
	}
	unlinked := empty
	c := *empty.repair
	unlinked.repair = &c
	c.PriorExecutionID = ""
	c.PriorJournalSHA256 = ""
	if _, _, err := foldExecutionRepairSelection([]executionSelectionAttempt{baseline, original, unlinked}, baseline.begin.Generation); err == nil {
		t.Fatal("two unlinked origins silently ordered/adopted")
	}
}
