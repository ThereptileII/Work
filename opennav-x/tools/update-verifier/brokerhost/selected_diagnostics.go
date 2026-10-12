package brokerhost

import (
	"sort"

	"example.com/opennav-update-verifier/brokerpipe"
)

// selectedHistoryPage is a report over already authenticated original facts.
// It neither opens paths nor grants authority for a later rollback request.
func selectedHistoryPage(attempts []executionSelectionAttempt, baseline string, selected executionSelectedVersion, inventory, requestID string, page int) (brokerpipe.MaintenanceResult, error) {
	refuse := func() (brokerpipe.MaintenanceResult, error) {
		return brokerpipe.MaintenanceResult{}, errExecutionSelection
	}
	current, pending, err := foldExecutionSelection(attempts, baseline)
	if err != nil || len(pending) != 0 || current != selected || !hashPattern.MatchString(inventory) || !requestPattern.MatchString(requestID) || page < 0 || page > 7 {
		return refuse()
	}
	var qualified *executionSelectionAttempt
	ancestors := make([]brokerpipe.MaintenanceAncestor, 0, len(attempts))
	seen := map[string]bool{}
	for i := range attempts {
		a := &attempts[i]
		if a.manual != nil || a.recovery != nil || a.terminal != "selected" {
			continue
		}
		if !hashPattern.MatchString(a.health) || !hashPattern.MatchString(a.closed) {
			return refuse()
		}
		if seen[a.begin.Generation] {
			return refuse() // Ambiguous generation is not an operator target.
		}
		seen[a.begin.Generation] = true
		if a.begin.Generation == selected.generation {
			if a.begin.CompletionSHA256 != selected.completion || qualified != nil || a.begin.Version != selected.installed.Version || a.begin.Commit != selected.installed.Commit {
				return refuse()
			}
			qualified = a
			continue
		}
		ancestors = append(ancestors, brokerpipe.MaintenanceAncestor{Generation: a.begin.Generation, CompletionSHA256: a.begin.CompletionSHA256})
	}
	if qualified == nil || len(ancestors) > 127 {
		return refuse()
	}
	sort.Slice(ancestors, func(i, j int) bool { return ancestors[i].Generation < ancestors[j].Generation })
	const size = 16 // Keep each result within the fixed 4096-byte pipe frame.
	start := page * size
	if start > len(ancestors) || (page != 0 && start == len(ancestors)) {
		return refuse()
	}
	end := start + size
	if end > len(ancestors) {
		end = len(ancestors)
	}
	items := append([]brokerpipe.MaintenanceAncestor{}, ancestors[start:end]...)
	result := brokerpipe.MaintenanceResult{Schema: 1, RequestID: requestID, State: brokerpipe.SelectedHistory,
		Generation: selected.generation, CompletionSHA256: selected.completion, SelectedHeadSHA256: selected.head,
		Version: selected.installed.Version, Commit: selected.installed.Commit,
		QualificationSHA256: qualified.health, QualificationClosedSHA256: qualified.closed,
		InventorySHA256: inventory, Page: &page, Ancestors: &items}
	if end < len(ancestors) {
		next := page + 1
		result.NextPage = &next
	}
	return result, nil
}
