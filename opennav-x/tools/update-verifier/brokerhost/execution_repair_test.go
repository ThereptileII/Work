package brokerhost

import (
	"strings"
	"testing"

	"example.com/opennav-update-verifier/brokerpipe"
	"example.com/opennav-update-verifier/brokerstore"
)

func repairSelectionFixture(t *testing.T, selected, source executionSelectionAttempt, gen string) (executionSelectionAttempt, []executionSelectionFact) {
	t.Helper()
	b := executionRepairBegin{executionSelectionBegin: executionSelectionBegin{Schema: 1, Purpose: "broker-selected-repair-v1", ClientSID: testSID, ExecutionID: gen, Generation: gen, Version: source.begin.Version, Commit: source.begin.Commit, CompletionSHA256: strings.Repeat("7", 64), PreviousGeneration: selected.begin.Generation, PreviousHeadSHA256: selected.head}, PreviousCompletionSHA256: selected.begin.CompletionSHA256, SourceGeneration: source.begin.Generation, SourceCompletionSHA256: source.begin.CompletionSHA256, SourceQualifiedHeadSHA256: source.head, CurrentProfileSHA256: strings.Repeat("8", 64), CurrentProfileIdentitySHA256: strings.Repeat("9", 64)}
	facts := []executionSelectionFact{{brokerstore.ExecutionPreparation, selectionBytes(t, b)}}
	for _, step := range []string{"payload-restored", "repair-selected"} {
		facts = append(facts, executionSelectionFact{brokerstore.ExecutionRestoration, selectionBytes(t, executionSelectionStep{1, "broker-selected-repair-step-v1", gen, step, gen, b.CompletionSHA256})})
	}
	head := executionOriginalDigest(selectionBytes(t, []string{string(facts[0].original), string(facts[1].original), string(facts[2].original)}))
	a, err := parseExecutionSelectionAttempt(testSID, gen, gen, b.CompletionSHA256, string(brokerstore.GenerationPurposeRepaired), head, facts)
	if err != nil {
		t.Fatal(err)
	}
	return a, facts
}
func TestRepairSelectionStartupHistoryAndUninstallRemainDistinctFromHealth(t *testing.T) {
	baseline, _ := selectionFixture(t, nil, false)
	release, _ := selectionFixture(t, &baseline, false)
	repair, _ := repairSelectionFixture(t, release, release, strings.Repeat("6", 32))
	attempts := []executionSelectionAttempt{repair, baseline, release}
	selected, pending, err := foldExecutionSelection(attempts, baseline.begin.Generation)
	if err != nil || len(pending) != 0 || selected.generation != repair.begin.Generation || selected.installed != ancestorInstalled(release) {
		t.Fatal("restored exact code unavailable for ordinary current startup", err)
	}
	if repair.health != "" || repair.closed != "" || repair.terminal == "selected" {
		t.Fatal("fabricated health qualification")
	}
	page, err := selectedHistoryPage(attempts, baseline.begin.Generation, selected, strings.Repeat("a", 64), strings.Repeat("b", 32), 0)
	p := 0
	q := brokerpipe.MaintenanceRequest{Schema: 1, Operation: brokerpipe.SelectedHistory, RequestID: strings.Repeat("b", 32), Page: &p}
	if err != nil || !brokerpipe.ValidSelectedHistoryResult(page, q) || page.QualificationSHA256 != "" || page.QualificationClosedSHA256 != "" || page.RepairSourceQualification == nil || page.RepairSourceQualification.Generation != release.begin.Generation {
		t.Fatal("history misrepresented source health", err)
	}
	target, err := previousQualified(attempts, baseline.begin.Generation, selected, strings.Repeat("c", 32))
	if err != nil || target.Previous.Generation != baseline.begin.Generation {
		t.Fatal("repair replica became a previous different release", err)
	}
	lineage, err := retirementLineage(attempts, baseline.begin.Generation, selected)
	if err != nil || len(lineage) != 3 {
		t.Fatal("uninstall omitted damaged original or repaired copy", err)
	}
	// Repeated repair refers directly to ORIGINAL qualification, never recursively
	// treats the preceding repair as freshly qualified.
	again, _ := repairSelectionFixture(t, repair, release, strings.Repeat("5", 32))
	attempts = append(attempts, again)
	if next, _, err := foldExecutionSelection(attempts, baseline.begin.Generation); err != nil || next.generation != again.begin.Generation {
		t.Fatal("bounded repeated original-source repair refused", err)
	}
	forged := again
	copy := *again.repair
	copy.SourceQualifiedHeadSHA256 = repair.head
	copy.SourceGeneration = repair.begin.Generation
	copy.SourceCompletionSHA256 = repair.begin.CompletionSHA256
	forged.repair = &copy
	if _, _, err = foldExecutionSelection(append(append([]executionSelectionAttempt{}, attempts[:3]...), forged), baseline.begin.Generation); err == nil {
		t.Fatal("unqualified repaired copy became qualification")
	}
	// Automatic failed-update fallback preserves the repaired selected bytes and
	// still needs all real closed/restore/fallback-healthy transaction facts.
	failed, facts := selectionFixture(t, &again, true)
	for i := range facts {
		facts[i].original = []byte(strings.ReplaceAll(strings.ReplaceAll(string(facts[i].original), strings.Repeat("c", 32), strings.Repeat("e", 32)), strings.Repeat("d", 32), strings.Repeat("f", 32)))
	}
	var errParse error
	failed, errParse = parseExecutionSelectionAttempt(testSID, strings.Repeat("f", 32), strings.Repeat("e", 32), strings.ReplaceAll(failed.begin.CompletionSHA256, strings.Repeat("c", 32), strings.Repeat("e", 32)), string(brokerstore.GenerationPurposeSignedCandidate), strings.Repeat("0", 64), facts)
	if errParse != nil {
		t.Fatal(errParse)
	}
	if after, _, err := foldExecutionSelection(append(attempts, failed), baseline.begin.Generation); err != nil || after.generation != again.begin.Generation || after.head != failed.head {
		t.Fatal("automatic fallback lost repaired predecessor", err)
	}
	for _, f := range facts {
		if strings.Contains(string(f.original), `"step":"fallback-healthy"`) {
			return
		}
	}
	t.Fatal("fixture failed to require genuine fallback health")
}
func TestRepairIncompleteOrMismatchedAuthorityCannotSelect(t *testing.T) {
	baseline, _ := selectionFixture(t, nil, false)
	repair, facts := repairSelectionFixture(t, baseline, baseline, strings.Repeat("6", 32))
	for n := 1; n < len(facts); n++ {
		a, err := parseExecutionSelectionAttempt(testSID, repair.begin.ExecutionID, repair.begin.Generation, repair.begin.CompletionSHA256, string(brokerstore.GenerationPurposeRepaired), repair.head, facts[:n])
		if err != nil {
			t.Fatal(err)
		}
		if _, pending, err := foldExecutionSelection([]executionSelectionAttempt{baseline, a}, baseline.begin.Generation); err == nil || len(pending) != 1 {
			t.Fatal("partial repair selected")
		}
	}
	for _, fault := range []string{"unreachable-qualification", "source-generation", "source-completion", "version", "commit", "parent"} {
		t.Run(fault, func(t *testing.T) {
			a := repair
			b := *repair.repair
			a.repair = &b
			switch fault {
			case "unreachable-qualification":
				b.SourceQualifiedHeadSHA256 = strings.Repeat("f", 64)
			case "source-generation":
				b.SourceGeneration = strings.Repeat("f", 32)
			case "source-completion":
				b.SourceCompletionSHA256 = strings.Repeat("f", 64)
			case "version":
				b.Version = "9.0.0"
			case "commit":
				b.Commit = strings.Repeat("f", 40)
			case "parent":
				a.begin.PreviousHeadSHA256 = strings.Repeat("f", 64)
			}
			if _, _, err := foldExecutionSelection([]executionSelectionAttempt{baseline, a}, baseline.begin.Generation); err == nil {
				t.Fatal("invalid repair authority selected")
			}
		})
	}
	facts[1].kind = brokerstore.ExecutionHealthObserved
	if _, err := parseExecutionSelectionAttempt(testSID, repair.begin.ExecutionID, repair.begin.Generation, repair.begin.CompletionSHA256, string(brokerstore.GenerationPurposeRepaired), repair.head, facts); err == nil {
		t.Fatal("health label accepted as copy restoration")
	}
}

func TestPreviousQualifiedRefusesHistoricalRepairWithoutInventingQualification(t *testing.T) {
	baseline, _ := selectionFixture(t, nil, false)
	release, _ := selectionFixture(t, &baseline, false)
	repair, _ := repairSelectionFixture(t, release, release, strings.Repeat("6", 32))
	next := executionSelectionAttempt{begin: executionSelectionBegin{Schema: 1, Purpose: "broker-selected-transaction-v1", ClientSID: testSID, ExecutionID: strings.Repeat("1", 32), Generation: strings.Repeat("2", 32), Version: "1.0.0-beta.3", Commit: strings.Repeat("3", 40), CompletionSHA256: strings.Repeat("4", 64), PreviousGeneration: repair.begin.Generation, PreviousHeadSHA256: repair.head}, head: strings.Repeat("5", 64), terminal: "selected", state: "selected", health: strings.Repeat("7", 64), closed: strings.Repeat("8", 64)}
	attempts := []executionSelectionAttempt{baseline, release, repair, next}
	selected, _, err := foldExecutionSelection(attempts, baseline.begin.Generation)
	if err != nil {
		t.Fatal("strict later selected transaction cannot follow repair", err)
	}
	if _, err = previousQualified(attempts, baseline.begin.Generation, selected, strings.Repeat("9", 32)); err == nil {
		t.Fatal("unqualified historical repair advertised as ordinary healthy target")
	}
}
