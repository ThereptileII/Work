package brokerhost

import (
	"fmt"
	"strings"
	"testing"
)

func customerRollbackLineage() []executionSelectionAttempt {
	var attempts []executionSelectionAttempt
	for i, digit := range []string{"a", "f", "c"} {
		a := executionSelectionAttempt{begin: executionSelectionBegin{Schema: 1, Purpose: "broker-selected-transaction-v1", ClientSID: testSID,
			ExecutionID: fmt.Sprintf("%032x", i+1), Generation: strings.Repeat(digit, 32), Version: fmt.Sprintf("1.0.%d", i), Commit: strings.Repeat(digit, 40), CompletionSHA256: strings.Repeat(digit, 64)},
			head: fmt.Sprintf("%064x", i+10), terminal: "selected", health: strings.Repeat("d", 64), closed: strings.Repeat("e", 64)}
		if i > 0 {
			a.begin.PreviousGeneration, a.begin.PreviousHeadSHA256 = attempts[i-1].begin.Generation, attempts[i-1].head
		}
		attempts = append(attempts, a)
	}
	return attempts
}

func TestPreviousQualifiedWalksOriginalLineageNotLexicalGeneration(t *testing.T) {
	attempts := customerRollbackLineage()
	baseline := attempts[0].begin.Generation
	selected, _, err := foldExecutionSelection(attempts, baseline)
	if err != nil {
		t.Fatal(err)
	}
	// Original store enumeration order is deliberately not chronological.
	r, err := previousQualified([]executionSelectionAttempt{attempts[2], attempts[0], attempts[1]}, baseline, selected, strings.Repeat("1", 32))
	if err != nil || r.Previous == nil || r.Previous.Generation != attempts[1].begin.Generation || r.Previous.Version != "1.0.1" || r.Previous.CompletionSHA256 != attempts[1].begin.CompletionSHA256 {
		t.Fatal(r, err)
	}
	// A failed candidate advances HEAD without qualifying its attempted payload.
	failed := attempts[2]
	failed.begin.ExecutionID = strings.Repeat("4", 32)
	failed.begin.Generation = strings.Repeat("b", 32)
	failed.begin.PreviousGeneration, failed.begin.PreviousHeadSHA256 = selected.generation, selected.head
	failed.head, failed.terminal = strings.Repeat("5", 64), "rolled-back"
	attempts = append(attempts, failed)
	selected, _, err = foldExecutionSelection(attempts, baseline)
	if err != nil {
		t.Fatal(err)
	}
	r, err = previousQualified(attempts, baseline, selected, strings.Repeat("2", 32))
	if err != nil || r.Previous == nil || r.Previous.Generation != attempts[1].begin.Generation {
		t.Fatal("failed candidate became rollback target", r, err)
	}
}

func TestPreviousQualifiedAfterManualSelectionUsesPreviousActualRelease(t *testing.T) {
	root, current, b, facts := manualRollbackFixture(t)
	manual := parseManualFixture(t, b, facts)
	attempts := []executionSelectionAttempt{manual, root, current}
	selected, _, err := foldExecutionSelection(attempts, root.begin.Generation)
	if err != nil {
		t.Fatal(err)
	}
	r, err := previousQualified(attempts, root.begin.Generation, selected, strings.Repeat("2", 32))
	if err != nil || r.Previous == nil || r.Previous.Generation != current.begin.Generation || r.Previous.Version != current.begin.Version {
		t.Fatal(r, err)
	}
}

func TestPreviousQualifiedRefusesMissingAmbiguousOrUnqualifiedHistory(t *testing.T) {
	for _, fault := range []string{"baseline-only", "pending", "fork", "missing-close", "wrong-head", "duplicate-generation"} {
		t.Run(fault, func(t *testing.T) {
			attempts := customerRollbackLineage()
			baseline := attempts[0].begin.Generation
			selected, _, err := foldExecutionSelection(attempts, baseline)
			if err != nil {
				t.Fatal(err)
			}
			switch fault {
			case "baseline-only":
				attempts = attempts[:1]
				selected, _, _ = foldExecutionSelection(attempts, baseline)
			case "pending":
				a := attempts[2]
				a.begin.ExecutionID = strings.Repeat("4", 32)
				a.begin.PreviousHeadSHA256 = selected.head
				a.begin.PreviousGeneration = selected.generation
				a.head = strings.Repeat("5", 64)
				a.terminal = ""
				a.state = "begin"
				attempts = append(attempts, a)
			case "fork":
				a := attempts[1]
				a.begin.ExecutionID = strings.Repeat("4", 32)
				a.head = strings.Repeat("5", 64)
				attempts = append(attempts, a)
			case "missing-close":
				attempts[1].closed = ""
			case "wrong-head":
				selected.head = strings.Repeat("5", 64)
			case "duplicate-generation":
				attempts[1].begin.Generation = baseline
				attempts[2].begin.PreviousGeneration = baseline
			}
			r, err := previousQualified(attempts, baseline, selected, strings.Repeat("2", 32))
			if err == nil || r.Previous != nil || r.Generation != "" {
				t.Fatal("unverified target disclosed", r, err)
			}
		})
	}
}
