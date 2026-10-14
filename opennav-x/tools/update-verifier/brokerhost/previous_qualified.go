package brokerhost

import "example.com/opennav-update-verifier/brokerpipe"

// Walk original selected HEAD linkage, never sorted generation IDs or timestamps.
// Failed-start recovery advances history but does not create a qualification.
func previousQualified(attempts []executionSelectionAttempt, baseline string, expected executionSelectedVersion, requestID string) (brokerpipe.MaintenanceResult, error) {
	current, pending, err := foldExecutionSelection(attempts, baseline)
	if err != nil || len(pending) != 0 || current != expected || !requestPattern.MatchString(requestID) {
		return brokerpipe.MaintenanceResult{}, errExecutionSelection
	}
	byHead := make(map[string]executionSelectionAttempt, len(attempts))
	qualified := make(map[string]executionSelectionAttempt)
	for _, a := range attempts {
		byHead[a.head] = a
		if a.terminal == "selected" && a.manual == nil && a.recovery == nil {
			if _, duplicate := qualified[a.begin.Generation]; duplicate || !hashPattern.MatchString(a.health) || !hashPattern.MatchString(a.closed) {
				return brokerpipe.MaintenanceResult{}, errExecutionSelection
			}
			qualified[a.begin.Generation] = a
		}
	}
	for head, count := current.head, 0; head != "" && count < 128; count++ {
		a, ok := byHead[head]
		if !ok {
			return brokerpipe.MaintenanceResult{}, errExecutionSelection
		}
		generation := a.begin.Generation
		if a.terminal == "rolled-back" || a.terminal == "restored-no-spawn" {
			generation = a.begin.PreviousGeneration
		}
		if generation != current.generation {
			target, ok := qualified[generation]
			if !ok {
				return brokerpipe.MaintenanceResult{}, errExecutionSelection
			}
			r := brokerpipe.MaintenanceResult{Schema: 1, RequestID: requestID, State: brokerpipe.PreviousQualified,
				Generation: current.generation, CompletionSHA256: current.completion, SelectedHeadSHA256: current.head,
				Version: current.installed.Version, Commit: current.installed.Commit,
				Previous: &brokerpipe.RollbackTarget{Generation: target.begin.Generation, CompletionSHA256: target.begin.CompletionSHA256, Version: target.begin.Version, Commit: target.begin.Commit}}
			q := brokerpipe.MaintenanceRequest{Schema: 1, Operation: brokerpipe.PreviousQualified, RequestID: requestID}
			if !brokerpipe.ValidPreviousQualifiedResult(r, q) {
				return brokerpipe.MaintenanceResult{}, errExecutionSelection
			}
			return r, nil
		}
		head = a.begin.PreviousHeadSHA256
	}
	return brokerpipe.MaintenanceResult{}, errExecutionSelection
}
