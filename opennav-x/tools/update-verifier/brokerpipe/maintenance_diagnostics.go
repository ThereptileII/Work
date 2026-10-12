package brokerpipe

// ValidSelectedHistoryResult validates a data-only response using the same
// strict wire rules as the fixed native transport. It grants no authority to
// mutate selected history or use an ancestor in another operation.
func ValidSelectedHistoryResult(r MaintenanceResult, q MaintenanceRequest) bool {
	return q.Operation == SelectedHistory && validMaintenanceResult(r, q)
}
