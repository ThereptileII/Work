package brokerhost

import (
	"strings"
	"testing"

	"example.com/opennav-update-verifier/brokerstore"
)

// A durable restoration intent may precede effects but never resolves selection
// by itself. Only actual closure/restoration/fallback health can author a terminal.
func TestExecutionSelectionRestoreIntentRequiresClosedCandidate(t *testing.T) {
	root, _ := selectionFixture(t, nil, false)
	attempt, facts := selectionFixture(t, &root, true)
	intent := executionSelectionFact{brokerstore.ExecutionRestoration, selectionBytes(t, executionSelectionStep{
		1, "broker-selected-step-v1", attempt.begin.ExecutionID, "profile-restore-intent", attempt.begin.Generation, strings.Repeat("9", 64)})}
	parse := func(f []executionSelectionFact) (executionSelectionAttempt, error) {
		return parseExecutionSelectionAttempt(testSID, attempt.begin.ExecutionID, attempt.begin.Generation,
			attempt.begin.CompletionSHA256, string(brokerstore.GenerationPurposeSignedCandidate), attempt.head, f)
	}
	t.Run("intent alone remains pending", func(t *testing.T) {
		prefix := append(append([]executionSelectionFact(nil), facts[:4]...), intent)
		pending, err := parse(prefix)
		if err != nil || pending.terminal != "" || pending.state != "profile-restore-intent" {
			t.Fatal("intent acquired completion", pending, err)
		}
		if _, unresolved, err := foldExecutionSelection([]executionSelectionAttempt{root, pending}, root.begin.Generation); err == nil || len(unresolved) != 1 {
			t.Fatal("intent bypassed pending recovery", unresolved, err)
		}
	})
	t.Run("complete native recovery sequence", func(t *testing.T) {
		sequence := append(append(append([]executionSelectionFact(nil), facts[:4]...), intent), facts[4:]...)
		resolved, err := parse(sequence)
		if err != nil || resolved.terminal != "rolled-back" {
			t.Fatal("valid intent-bound recovery rejected", resolved, err)
		}
		selected, pending, err := foldExecutionSelection([]executionSelectionAttempt{root, resolved}, root.begin.Generation)
		if err != nil || len(pending) != 0 || selected.generation != root.begin.Generation {
			t.Fatal("wrong recovery selection", selected, pending, err)
		}
	})
	for _, before := range []int{1, 2, 3} {
		t.Run("intent before closure "+string(rune('0'+before)), func(t *testing.T) {
			sequence := append(append([]executionSelectionFact(nil), facts[:before]...), intent)
			if _, err := parse(sequence); err == nil {
				t.Fatal("restoration permitted without native candidate closure")
			}
		})
	}
	t.Run("duplicate intent refuses retry", func(t *testing.T) {
		sequence := append(append([]executionSelectionFact(nil), facts[:4]...), intent, intent)
		if _, err := parse(sequence); err == nil {
			t.Fatal("restoration intent retried")
		}
	})
}
