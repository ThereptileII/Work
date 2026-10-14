package brokerpipe

// ValidSelectedHistoryResult validates a data-only response using the same
// strict wire rules as the fixed native transport. It grants no authority to
// mutate selected history or use an ancestor in another operation.
func ValidSelectedHistoryResult(r MaintenanceResult, q MaintenanceRequest) bool {
	return q.Operation == SelectedHistory && validMaintenanceResult(r, q)
}

// ValidMaintenanceClientRequest limits the shared native client to the explicit
// ordinary maintenance and read-only target conversations, using complete wire
// validation.
// Decommission/uninstall retain their separate witness-owning conversation.
func ValidMaintenanceClientRequest(q MaintenanceRequest) bool {
	return (q.Operation == ManualRollback || q.Operation == SelectedHistory || q.Operation == PreviousQualified) && validMaintenanceRequest(q)
}

func ValidMaintenanceClientResult(r MaintenanceResult, q MaintenanceRequest) bool {
	return ValidMaintenanceClientRequest(q) && validMaintenanceResult(r, q)
}
