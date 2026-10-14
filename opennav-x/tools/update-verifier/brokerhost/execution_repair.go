package brokerhost

import (
	"example.com/opennav-update-verifier/brokerstore"
	"example.com/opennav-update-verifier/releasepolicy"
)

// Repair is an exact-code copy selection, not an ordinary qualification. The
// source qualification must already be reachable in the original selected chain.
type executionRepairBegin struct {
	executionSelectionBegin
	PriorExecutionID             string `json:"priorExecutionId,omitempty"`
	PriorJournalSHA256           string `json:"priorJournalSha256,omitempty"`
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
		b.Schema != 1 || (b.Purpose != "broker-selected-repair-v1" && b.Purpose != "broker-selected-repair-recovery-v1") || b.ClientSID != sid || b.ExecutionID != id || b.Generation != generation || b.CompletionSHA256 != completion ||
		!requestPattern.MatchString(b.PreviousGeneration) || b.PreviousGeneration == generation || !hashPattern.MatchString(b.PreviousHeadSHA256) ||
		!hashPattern.MatchString(b.PreviousCompletionSHA256) || !requestPattern.MatchString(b.SourceGeneration) || b.SourceGeneration == generation || !hashPattern.MatchString(b.SourceCompletionSHA256) ||
		!hashPattern.MatchString(b.SourceQualifiedHeadSHA256) || !hashPattern.MatchString(b.CurrentProfileSHA256) || !hashPattern.MatchString(b.CurrentProfileIdentitySHA256) ||
		!commitPattern.MatchString(b.Commit) {
		return executionSelectionAttempt{}, errExecutionSelection
	}
	if b.Purpose == "broker-selected-repair-recovery-v1" {
		if !requestPattern.MatchString(b.PriorExecutionID) || b.PriorExecutionID == id || !hashPattern.MatchString(b.PriorJournalSHA256) {
			return executionSelectionAttempt{}, errExecutionSelection
		}
	} else if b.PriorExecutionID != "" || b.PriorJournalSHA256 != "" {
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

// An empty original repair journal has durable creation provenance but no fact
// assertion. Its repair-empty label grants neither selection nor profile/health.
func parseExecutionRepairEmpty(sid, id, generation, completion, head string, original []byte) (executionSelectionAttempt, error) {
	c, err := brokerstore.DescribeRepairCompletion(original)
	if err != nil || c.ClientSID != sid || c.Generation != generation || c.CompletionSHA256 != completion || !requestPattern.MatchString(id) || !hashPattern.MatchString(head) {
		return executionSelectionAttempt{}, errExecutionSelection
	}
	b := executionRepairBegin{executionSelectionBegin: executionSelectionBegin{Schema: 1, Purpose: "broker-empty-repair-v1", ExecutionID: id, ClientSID: sid, Generation: generation, Version: c.Identity.Version, Commit: c.Identity.Commit, CompletionSHA256: completion, PreviousGeneration: c.Origin.PreviousGeneration, PreviousHeadSHA256: c.Origin.PreviousSelectedHeadSHA256}, PreviousCompletionSHA256: c.Origin.PreviousCompletionSHA256, SourceGeneration: c.SourceGeneration, SourceCompletionSHA256: c.Origin.SourceCompletionSHA256}
	return executionSelectionAttempt{begin: b.executionSelectionBegin, head: head, state: "repair-empty", repair: &b, repairEmpty: true}, nil
}
func repairRecoveryMatches(prior executionSelectionAttempt, b *executionRepairBegin) bool {
	if b == nil || prior.repair == nil || prior.terminal != "" || prior.consumed || prior.head != b.PriorJournalSHA256 || prior.begin.ExecutionID != b.PriorExecutionID || (prior.state != "repair-empty" && prior.state != "repair-begin" && prior.state != "payload-restored") {
		return false
	}
	p := prior.repair
	return prior.begin.ClientSID == b.ClientSID && prior.begin.Generation == b.Generation && prior.begin.CompletionSHA256 == b.CompletionSHA256 && prior.begin.Version == b.Version && prior.begin.Commit == b.Commit && prior.begin.PreviousGeneration == b.PreviousGeneration && prior.begin.PreviousHeadSHA256 == b.PreviousHeadSHA256 && p.PreviousCompletionSHA256 == b.PreviousCompletionSHA256 && p.SourceGeneration == b.SourceGeneration && p.SourceCompletionSHA256 == b.SourceCompletionSHA256 && (prior.repairEmpty || b.Purpose == "broker-empty-repair-v1" || p.SourceQualifiedHeadSHA256 == b.SourceQualifiedHeadSHA256)
}

// Only explicit Repair may inspect one authenticated interrupted no-launch
// repair. Ordinary selection remains unavailable until a new terminal journal.
func foldExecutionRepairSelection(attempts []executionSelectionAttempt, baseline string) (executionSelectedVersion, *executionSelectionAttempt, error) {
	selected, pending, err := foldExecutionSelection(attempts, baseline)
	if err == nil {
		return selected, nil, nil
	}
	if selected.generation == "" || len(pending) != 1 {
		return executionSelectedVersion{}, nil, errExecutionSelection
	}
	for i := range attempts {
		a := &attempts[i]
		if a.begin.ExecutionID == pending[0].executionID && a.repair != nil && a.terminal == "" && a.begin.PreviousGeneration == selected.generation && a.begin.PreviousHeadSHA256 == selected.head && a.repair.PreviousCompletionSHA256 == selected.completion {
			return selected, a, nil
		}
	}
	return executionSelectedVersion{}, nil, errExecutionSelection
}
