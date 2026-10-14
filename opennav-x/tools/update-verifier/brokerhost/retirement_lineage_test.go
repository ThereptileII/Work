package brokerhost

import (
	"reflect"
	"strings"
	"testing"

	"example.com/opennav-update-verifier/brokerstore"
)

func TestRetirementLineageIncludesUpdatedAndRolledBackRoots(t *testing.T) {
	baseline, _ := selectionFixture(t, nil, false)
	updated, _ := selectionFixture(t, &baseline, false)
	rolledBack, _ := selectionFixture(t, &baseline, true)
	_, _, manualBegin, manualFacts := manualRollbackFixture(t)
	manual := parseManualFixture(t, manualBegin, manualFacts)
	pending := updated
	pending.state, pending.terminal, pending.consumed = "commissioned", "", false
	recovered, _ := selectionRecoveryFixture(t, pending, 4)
	for _, tc := range []struct {
		name     string
		attempts []executionSelectionAttempt
		selected string
	}{
		{"baseline", []executionSelectionAttempt{baseline}, baseline.begin.Generation},
		{"updated", []executionSelectionAttempt{updated, baseline}, updated.begin.Generation},
		{"automatic rollback", []executionSelectionAttempt{rolledBack, baseline}, baseline.begin.Generation},
		{"manual rollback", []executionSelectionAttempt{manual, updated, baseline}, baseline.begin.Generation},
		{"resolved no spawn", []executionSelectionAttempt{recovered, baseline, pending}, baseline.begin.Generation},
	} {
		t.Run(tc.name, func(t *testing.T) {
			selected, _, err := foldExecutionSelection(tc.attempts, baseline.begin.Generation)
			if err != nil || selected.generation != tc.selected {
				t.Fatal("invalid source fixture", selected, err)
			}
			want := []brokerstore.RetirementGeneration{{GenerationID: baseline.begin.Generation, CompletionSHA256: baseline.begin.CompletionSHA256}}
			if len(tc.attempts) > 1 {
				want = append(want, brokerstore.RetirementGeneration{GenerationID: updated.begin.Generation, CompletionSHA256: updated.begin.CompletionSHA256})
			}
			got, err := retirementLineage(tc.attempts, baseline.begin.Generation, selected)
			if err != nil || !reflect.DeepEqual(got, want) {
				t.Fatal("original lineage omitted or changed retained root", got, want, err)
			}
		})
	}
}

func TestRetirementLineageRefusesUnqualifiedOrChangedHistory(t *testing.T) {
	baseline, _ := selectionFixture(t, nil, false)
	updated, _ := selectionFixture(t, &baseline, false)
	selected, _, err := foldExecutionSelection([]executionSelectionAttempt{baseline, updated}, baseline.begin.Generation)
	if err != nil {
		t.Fatal(err)
	}
	for _, fault := range []string{"empty", "pending", "fork", "different head", "different completion", "different selected", "different version", "invalid completion", "conflicting root bytes"} {
		t.Run(fault, func(t *testing.T) {
			attempts := []executionSelectionAttempt{baseline, updated}
			expected := selected
			switch fault {
			case "empty":
				attempts = nil
			case "pending":
				attempts[1].terminal = ""
			case "fork":
				fork := updated
				fork.begin.ExecutionID, fork.head = strings.Repeat("f", 32), strings.Repeat("f", 64)
				attempts = append(attempts, fork)
			case "different head":
				expected.head = strings.Repeat("f", 64)
			case "different completion":
				expected.completion = strings.Repeat("f", 64)
			case "different selected":
				expected.generation = baseline.begin.Generation
			case "different version":
				expected.installed.Version = "9.0.0"
			case "invalid completion":
				attempts[1].begin.CompletionSHA256 = "invalid"
				expected.completion = "invalid"
			case "conflicting root bytes":
				// A resolved baseline retry must still identify the same bytes.
				prior := baseline
				prior.begin.ExecutionID, prior.head = strings.Repeat("7", 32), strings.Repeat("7", 64)
				prior.begin.CompletionSHA256 = strings.Repeat("8", 64)
				prior.state, prior.terminal, prior.consumed = "begin", "", false
				recovery, _ := selectionRecoveryFixture(t, prior, 4)
				attempts = append(attempts, prior, recovery)
			}
			if lineage, err := retirementLineage(attempts, baseline.begin.Generation, expected); err == nil || lineage != nil {
				t.Fatal("uncertain history acquired retirement comparison", lineage, err)
			}
		})
	}
}
