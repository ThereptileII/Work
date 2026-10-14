package brokerhost

import (
	"example.com/opennav-update-verifier/brokerstore"
	"example.com/opennav-update-verifier/releasepolicy"
)

// Repair is an exact-code copy selection, not an ordinary qualification. The
// source qualification must already be reachable in the original selected chain.
type executionRepairBegin struct {
	executionSelectionBegin
	PreviousCompletionSHA256     string `json:"previousCompletionSha256"`
	SourceGeneration             string `json:"sourceGeneration"`
	SourceCompletionSHA256       string `json:"sourceCompletionSha256"`
	SourceQualifiedHeadSHA256    string `json:"sourceQualifiedHeadSha256"`
	CurrentProfileSHA256         string `json:"currentProfileSha256"`
	CurrentProfileIdentitySHA256 string `json:"currentProfileIdentitySha256"`
}

func repairQualified(a executionSelectionAttempt, b *executionRepairBegin) bool {
	return b != nil && a.repair == nil && a.manual == nil && a.recovery == nil && a.terminal == "selected" &&
		a.head == b.SourceQualifiedHeadSHA256 && a.begin.Generation == b.SourceGeneration && a.begin.CompletionSHA256 == b.SourceCompletionSHA256 &&
		a.begin.ClientSID == b.ClientSID && a.begin.Version == b.Version && a.begin.Commit == b.Commit && hashPattern.MatchString(a.health) && hashPattern.MatchString(a.closed)
}
func parseExecutionRepair(sid, id, generation, completion, purpose, head string, facts []executionSelectionFact) (executionSelectionAttempt, error) {
	var b executionRepairBegin
	if purpose != string(brokerstore.GenerationPurposeRepaired) || exactExecutionRecord(facts[0].original, &b) != nil ||
		b.Schema != 1 || b.Purpose != "broker-selected-repair-v1" || b.ClientSID != sid || b.ExecutionID != id || b.Generation != generation || b.CompletionSHA256 != completion ||
		!requestPattern.MatchString(b.PreviousGeneration) || b.PreviousGeneration == generation || !hashPattern.MatchString(b.PreviousHeadSHA256) ||
		!hashPattern.MatchString(b.PreviousCompletionSHA256) || !requestPattern.MatchString(b.SourceGeneration) || b.SourceGeneration == generation || !hashPattern.MatchString(b.SourceCompletionSHA256) ||
		!hashPattern.MatchString(b.SourceQualifiedHeadSHA256) || !hashPattern.MatchString(b.CurrentProfileSHA256) || !hashPattern.MatchString(b.CurrentProfileIdentitySHA256) ||
		!commitPattern.MatchString(b.Commit) {
		return executionSelectionAttempt{}, errExecutionSelection
	}
	if _, err := releasepolicy.ParseSemVer(b.Version); err != nil || len(b.Version) > 128 {
		return executionSelectionAttempt{}, errExecutionSelection
	}
	a := executionSelectionAttempt{begin: b.executionSelectionBegin, head: head, state: "repair-begin", repair: &b}
	if len(facts) > 3 {
		return executionSelectionAttempt{}, errExecutionSelection
	}
	for i, f := range facts[1:] {
		var step executionSelectionStep
		expected := "payload-restored"
		if i == 1 {
			expected = "repair-selected"
		}
		if exactExecutionRecord(f.original, &step) != nil || step.Schema != 1 || step.Purpose != "broker-selected-repair-step-v1" || step.ExecutionID != id || step.Generation != generation || step.Step != expected || step.EvidenceSHA256 != completion || f.kind != brokerstore.ExecutionRestoration {
			return executionSelectionAttempt{}, errExecutionSelection
		}
		a.state = step.Step
	}
	if a.state == "repair-selected" {
		a.terminal = a.state
	}
	return a, nil
}
