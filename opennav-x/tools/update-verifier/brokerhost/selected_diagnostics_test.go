package brokerhost

import (
	"fmt"
	"strings"
	"testing"

	"example.com/opennav-update-verifier/releasepolicy"
)

func TestSelectedHistoryPagesOriginalQualificationWithoutMutation(t *testing.T) {
	var attempts []executionSelectionAttempt
	previousGeneration, previousHead := "", ""
	for i := 1; i <= 19; i++ {
		generation := fmt.Sprintf("%032x", i)
		head := fmt.Sprintf("%064x", i)
		completion := fmt.Sprintf("%064x", i+100)
		attempts = append(attempts, executionSelectionAttempt{begin: executionSelectionBegin{
			Schema: 1, Purpose: "broker-selected-transaction-v1", ExecutionID: fmt.Sprintf("%032x", i+200),
			ClientSID: testSID, Generation: generation, Version: fmt.Sprintf("1.0.%d", i), Commit: strings.Repeat("a", 40),
			CompletionSHA256: completion, PreviousGeneration: previousGeneration, PreviousHeadSHA256: previousHead},
			head: head, terminal: "selected", health: strings.Repeat("b", 64), closed: strings.Repeat("c", 64)})
		previousGeneration, previousHead = generation, head
	}
	selected := executionSelectedVersion{generation: previousGeneration, completion: attempts[18].begin.CompletionSHA256,
		head: previousHead, installed: releasepolicy.Installed{Version: attempts[18].begin.Version, Commit: attempts[18].begin.Commit}}
	for page, count := range []int{16, 2} {
		r, err := selectedHistoryPage(attempts, attempts[0].begin.Generation, selected, strings.Repeat("d", 64), strings.Repeat("e", 32), page)
		if err != nil || r.Ancestors == nil || len(*r.Ancestors) != count || r.QualificationSHA256 != strings.Repeat("b", 64) || r.QualificationClosedSHA256 != strings.Repeat("c", 64) {
			t.Fatal(page, r, err)
		}
		if (page == 0 && (r.NextPage == nil || *r.NextPage != 1)) || (page == 1 && r.NextPage != nil) {
			t.Fatal("incorrect bounded continuation", page, r.NextPage)
		}
	}
	if _, err := selectedHistoryPage(attempts, attempts[0].begin.Generation, selected, strings.Repeat("d", 64), strings.Repeat("e", 32), 2); err == nil {
		t.Fatal("out-of-range page disclosed a partial result")
	}
	selected.head = strings.Repeat("f", 64)
	if _, err := selectedHistoryPage(attempts, attempts[0].begin.Generation, selected, strings.Repeat("d", 64), strings.Repeat("e", 32), 0); err == nil {
		t.Fatal("changed selected head accepted")
	}
	selected.head = previousHead
	pending := executionSelectionAttempt{begin: executionSelectionBegin{ExecutionID: strings.Repeat("f", 32), Generation: strings.Repeat("f", 32),
		PreviousGeneration: selected.generation, PreviousHeadSHA256: selected.head}, head: strings.Repeat("f", 64), state: "begin"}
	if _, err := selectedHistoryPage(append(append([]executionSelectionAttempt{}, attempts...), pending), attempts[0].begin.Generation, selected, strings.Repeat("d", 64), strings.Repeat("e", 32), 0); err == nil {
		t.Fatal("pending protected history disclosed a selected result")
	}
	changed := append([]executionSelectionAttempt{}, attempts...)
	changed[0].closed = ""
	if _, err := selectedHistoryPage(changed, attempts[0].begin.Generation, selected, strings.Repeat("d", 64), strings.Repeat("e", 32), 0); err == nil {
		t.Fatal("missing original qualification closure accepted")
	}
}
