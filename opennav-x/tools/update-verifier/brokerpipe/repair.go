package brokerpipe

import "example.com/opennav-update-verifier/releasepolicy"

// Explicit historical source evidence, never health of a repaired generation.
type RepairQualification struct {
	Generation          string `json:"generation"`
	CompletionSHA256    string `json:"completionSha256"`
	QualifiedHeadSHA256 string `json:"qualifiedHeadSha256"`
	HealthSHA256        string `json:"healthSha256"`
	ClosedSHA256        string `json:"closedSha256"`
}

func validHistoryQualification(r MaintenanceResult) bool {
	if r.RepairSourceQualification == nil {
		return hex64.MatchString(r.QualificationSHA256) && hex64.MatchString(r.QualificationClosedSHA256)
	}
	p := r.RepairSourceQualification
	return r.QualificationSHA256 == "" && r.QualificationClosedSHA256 == "" && id32.MatchString(p.Generation) && p.Generation != r.Generation && hex64.MatchString(p.CompletionSHA256) && hex64.MatchString(p.QualifiedHeadSHA256) && hex64.MatchString(p.HealthSHA256) && hex64.MatchString(p.ClosedSHA256)
}
func validRepairResult(r MaintenanceResult, q MaintenanceRequest) bool {
	if !emptyDecommissionResult(r) || r.Previous != nil || r.RepairSourceQualification != nil || !id32.MatchString(r.Generation) || !hex64.MatchString(r.CompletionSHA256) || !hex64.MatchString(r.SelectedHeadSHA256) || !maintenanceCommit.MatchString(r.Commit) || len(r.Version) > 128 || r.QualificationSHA256 != "" || r.QualificationClosedSHA256 != "" || r.InventorySHA256 != "" || r.Page != nil || r.NextPage != nil || r.Ancestors != nil {
		return false
	}
	if _, err := releasepolicy.ParseSemVer(r.Version); err != nil {
		return false
	}
	if q.Operation == RepairInspect {
		return r.State == RepairInspect
	}
	return r.State == "repair-selected" && r.SelectedHeadSHA256 != q.ExpectedSelectedHeadSHA256
}
