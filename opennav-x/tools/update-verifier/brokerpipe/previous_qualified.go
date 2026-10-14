package brokerpipe

import "example.com/opennav-update-verifier/releasepolicy"

// The optional query never changes the original selected-history/manual wire.
// An old broker refuses the unknown operation before any mutation.
func ValidPreviousQualifiedResult(r MaintenanceResult, q MaintenanceRequest) bool {
	return q.Operation == PreviousQualified && validMaintenanceResult(r, q)
}
func validPreviousQualifiedResult(r MaintenanceResult) bool {
	p := r.Previous
	if p == nil || !emptyDecommissionResult(r) || r.State != PreviousQualified ||
		!id32.MatchString(r.Generation) || !hex64.MatchString(r.CompletionSHA256) || !hex64.MatchString(r.SelectedHeadSHA256) ||
		!maintenanceCommit.MatchString(r.Commit) || len(r.Version) > 128 ||
		!id32.MatchString(p.Generation) || p.Generation == r.Generation || !hex64.MatchString(p.CompletionSHA256) ||
		!maintenanceCommit.MatchString(p.Commit) || len(p.Version) > 128 ||
		r.QualificationSHA256 != "" || r.QualificationClosedSHA256 != "" || r.InventorySHA256 != "" ||
		r.Page != nil || r.NextPage != nil || r.Ancestors != nil {
		return false
	}
	_, currentErr := releasepolicy.ParseSemVer(r.Version)
	_, previousErr := releasepolicy.ParseSemVer(p.Version)
	return currentErr == nil && previousErr == nil
}
