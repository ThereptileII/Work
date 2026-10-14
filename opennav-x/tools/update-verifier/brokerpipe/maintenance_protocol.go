package brokerpipe

import (
	"context"
	"encoding/json"
	"regexp"

	"example.com/opennav-update-verifier/releasepolicy"
)

// Maintenance is a separate explicit elevated-operator conversation. Its data
// cannot authorize a mutation: mutating host operations must retain registration,
// selected history, original application exit and current profile independently.
const MaintenancePipeName = `\\.\pipe\SkagerUpdateBroker.maintenance.v1`
const ManualRollback = "manual-rollback"
const SelectedHistory = "selected-history"
const RepairInspect = "repair-inspect"
const RepairCurrent = "repair-current"
const PreviousQualified = "previous-qualified"
const PrepareDecommission = "prepare-decommission"
const PrepareUninstall = "prepare-uninstall"

const selectedHistoryPageSize = 16

var maintenanceCommit = regexp.MustCompile(`^[0-9a-f]{40}$`)

type MaintenanceRequest struct {
	Schema                     int    `json:"schema"`
	Operation                  string `json:"operation"`
	RequestID                  string `json:"requestId"`
	ExpectedSelectedHeadSHA256 string `json:"expectedSelectedHeadSha256,omitempty"`
	TargetGeneration           string `json:"targetGeneration,omitempty"`
	TargetCompletionSHA256     string `json:"targetCompletionSha256,omitempty"`
	Page                       *int   `json:"page,omitempty"`
	InventorySHA256            string `json:"inventorySha256,omitempty"`
}
type MaintenanceAncestor struct {
	Generation       string `json:"generation"`
	CompletionSHA256 string `json:"completionSha256"`
}
type RollbackTarget struct {
	Generation       string `json:"generation"`
	CompletionSHA256 string `json:"completionSha256"`
	Version          string `json:"version"`
	Commit           string `json:"commit"`
}
type MaintenanceResult struct {
	RepairSourceQualification    *RepairQualification   `json:"repairSourceQualification,omitempty"`
	Previous                     *RollbackTarget        `json:"previous,omitempty"`
	ClientSID                    string                 `json:"clientSid,omitempty"`
	RegistrationSHA256           string                 `json:"registrationSha256,omitempty"`
	CurrentProfileSHA256         string                 `json:"currentProfileSha256,omitempty"`
	CurrentProfileIdentitySHA256 string                 `json:"currentProfileIdentitySha256,omitempty"`
	FenceSHA256                  string                 `json:"fenceSha256,omitempty"`
	RetirementSHA256             string                 `json:"retirementSha256,omitempty"`
	Schema                       int                    `json:"schema"`
	RequestID                    string                 `json:"requestId"`
	State                        string                 `json:"state"`
	Generation                   string                 `json:"generation"`
	CompletionSHA256             string                 `json:"completionSha256"`
	SelectedHeadSHA256           string                 `json:"selectedHeadSha256"`
	Version                      string                 `json:"version,omitempty"`
	Commit                       string                 `json:"commit,omitempty"`
	QualificationSHA256          string                 `json:"qualificationSha256,omitempty"`
	QualificationClosedSHA256    string                 `json:"qualificationClosedSha256,omitempty"`
	InventorySHA256              string                 `json:"inventorySha256,omitempty"`
	Page                         *int                   `json:"page,omitempty"`
	NextPage                     *int                   `json:"nextPage,omitempty"`
	Ancestors                    *[]MaintenanceAncestor `json:"ancestors,omitempty"`
}
type MaintenanceHandler func(context.Context, Peer, MaintenanceRequest) (MaintenanceResult, error)

// ProtectedUninstallHandler retains its original host owners across one pipe
// conversation. publishFence is one-shot and waits for the same operator's
// exact continuation before returning; only then may the handler retire files.
type ProtectedUninstallHandler func(context.Context, Peer, MaintenanceRequest, func(context.Context, MaintenanceResult) error) (MaintenanceResult, error)

func validMaintenanceRequest(q MaintenanceRequest) bool {
	if q.Schema != 1 || !id32.MatchString(q.RequestID) {
		return false
	}
	switch q.Operation {
	case PreviousQualified, RepairInspect:
		return q.Page == nil && q.InventorySHA256 == "" && q.ExpectedSelectedHeadSHA256 == "" && q.TargetGeneration == "" && q.TargetCompletionSHA256 == ""
	case PrepareDecommission, PrepareUninstall, RepairCurrent:
		return q.Page == nil && q.InventorySHA256 == "" && q.TargetGeneration == "" && q.TargetCompletionSHA256 == "" && hex64.MatchString(q.ExpectedSelectedHeadSHA256)
	case ManualRollback:
		return q.Page == nil && q.InventorySHA256 == "" && hex64.MatchString(q.ExpectedSelectedHeadSHA256) && id32.MatchString(q.TargetGeneration) && hex64.MatchString(q.TargetCompletionSHA256)
	case SelectedHistory:
		if q.Page == nil || *q.Page < 0 || *q.Page > 7 || q.TargetGeneration != "" || q.TargetCompletionSHA256 != "" {
			return false
		}
		if *q.Page == 0 {
			return q.ExpectedSelectedHeadSHA256 == "" && q.InventorySHA256 == ""
		}
		return hex64.MatchString(q.ExpectedSelectedHeadSHA256) && hex64.MatchString(q.InventorySHA256)
	default:
		return false
	}
}
func encodeMaintenanceRequest(q MaintenanceRequest) ([]byte, error) {
	if !validMaintenanceRequest(q) {
		return nil, ErrRefused
	}
	return json.Marshal(q)
}
func decodeMaintenanceRequest(raw []byte) (q MaintenanceRequest, err error) {
	if len(raw) > maxFrame {
		return q, ErrRefused
	}
	var probe struct {
		Operation string `json:"operation"`
	}
	if json.Unmarshal(raw, &probe) != nil {
		return q, ErrRefused
	}
	fields := []string{"schema", "operation", "requestId", "expectedSelectedHeadSha256", "targetGeneration", "targetCompletionSha256"}
	if probe.Operation == PreviousQualified || probe.Operation == RepairInspect {
		fields = []string{"schema", "operation", "requestId"}
	}
	if probe.Operation == PrepareDecommission || probe.Operation == PrepareUninstall || probe.Operation == RepairCurrent {
		fields = []string{"schema", "operation", "requestId", "expectedSelectedHeadSha256"}
	}
	if probe.Operation == SelectedHistory {
		fields = []string{"schema", "operation", "requestId", "page"}
		var continuation struct {
			Page *int `json:"page"`
		}
		if json.Unmarshal(raw, &continuation) != nil || continuation.Page == nil {
			return q, ErrRefused
		}
		if *continuation.Page > 0 {
			fields = append(fields, "expectedSelectedHeadSha256", "inventorySha256")
		}
	}
	if _, err = exactObject(raw, fields, false); err != nil {
		return q, ErrRefused
	}
	if json.Unmarshal(raw, &q) != nil || !validMaintenanceRequest(q) {
		return MaintenanceRequest{}, ErrRefused
	}
	return q, nil
}
func validMaintenanceResult(r MaintenanceResult, q MaintenanceRequest) bool {
	if !validMaintenanceRequest(q) || r.Schema != 1 || r.RequestID != q.RequestID || (q.Operation != PreviousQualified && r.Previous != nil) {
		return false
	}
	if q.Operation != SelectedHistory && r.RepairSourceQualification != nil {
		return false
	}
	if q.Operation == RepairInspect || q.Operation == RepairCurrent {
		return validRepairResult(r, q)
	}
	if q.Operation == PreviousQualified {
		return validPreviousQualifiedResult(r)
	}
	if q.Operation == PrepareDecommission || q.Operation == PrepareUninstall {
		return validDecommissionResult(r, q)
	}
	if !emptyDecommissionResult(r) {
		return false
	}
	if q.Operation == ManualRollback {
		return r.State == "manual-selected" && r.Generation == q.TargetGeneration && r.CompletionSHA256 == q.TargetCompletionSHA256 && hex64.MatchString(r.SelectedHeadSHA256) && r.SelectedHeadSHA256 != q.ExpectedSelectedHeadSHA256 && r.Version == "" && r.Commit == "" && r.QualificationSHA256 == "" && r.QualificationClosedSHA256 == "" && r.InventorySHA256 == "" && r.Page == nil && r.NextPage == nil && r.Ancestors == nil
	}
	if r.State != SelectedHistory || !id32.MatchString(r.Generation) || !hex64.MatchString(r.CompletionSHA256) || !hex64.MatchString(r.SelectedHeadSHA256) || !hex64.MatchString(r.InventorySHA256) || !maintenanceCommit.MatchString(r.Commit) || !validHistoryQualification(r) || r.Page == nil || *r.Page != *q.Page || r.Ancestors == nil || len(*r.Ancestors) > selectedHistoryPageSize || len(r.Version) > 128 {
		return false
	}
	if _, err := releasepolicy.ParseSemVer(r.Version); err != nil {
		return false
	}
	if *q.Page > 0 && (q.ExpectedSelectedHeadSHA256 != r.SelectedHeadSHA256 || q.InventorySHA256 != r.InventorySHA256) {
		return false
	}
	if r.NextPage != nil && (*r.NextPage != *r.Page+1 || *r.NextPage > 7 || len(*r.Ancestors) != selectedHistoryPageSize) {
		return false
	}
	seen := map[string]bool{}
	for _, a := range *r.Ancestors {
		if !id32.MatchString(a.Generation) || !hex64.MatchString(a.CompletionSHA256) || a.Generation == r.Generation || seen[a.Generation] {
			return false
		}
		seen[a.Generation] = true
	}
	return true
}
func decodeMaintenanceResult(raw []byte, q MaintenanceRequest) (r MaintenanceResult, err error) {
	if len(raw) > maxFrame {
		return r, ErrRefused
	}
	fields := []string{"schema", "requestId", "state", "generation", "completionSha256", "selectedHeadSha256"}
	if q.Operation == RepairInspect || q.Operation == RepairCurrent {
		fields = append(fields, "version", "commit")
	}
	if q.Operation == PreviousQualified {
		fields = append(fields, "version", "commit", "previous")
	}
	if q.Operation == PrepareDecommission || q.Operation == PrepareUninstall {
		fields = append(fields, "clientSid", "registrationSha256", "currentProfileSha256", "currentProfileIdentitySha256", "fenceSha256")
	}
	if q.Operation == PrepareUninstall {
		fields = append(fields, "retirementSha256")
	}
	if q.Operation == SelectedHistory {
		fields = append(fields, "version", "commit", "inventorySha256", "page", "ancestors")
		var repair struct {
			Source *RepairQualification `json:"repairSourceQualification"`
		}
		if json.Unmarshal(raw, &repair) != nil {
			return r, ErrRefused
		}
		if repair.Source != nil {
			fields = append(fields, "repairSourceQualification")
		} else {
			fields = append(fields, "qualificationSha256", "qualificationClosedSha256")
		}
		var probe struct {
			NextPage *int `json:"nextPage"`
		}
		if json.Unmarshal(raw, &probe) != nil {
			return r, ErrRefused
		}
		if probe.NextPage != nil {
			fields = append(fields, "nextPage")
		}
	}
	object, err := exactObject(raw, fields, false)
	if err != nil {
		return r, ErrRefused
	}
	if q.Operation == PreviousQualified {
		if _, err := exactObject(object["previous"], []string{"generation", "completionSha256", "version", "commit"}, false); err != nil {
			return r, ErrRefused
		}
	}
	if q.Operation == SelectedHistory {
		if original, ok := object["repairSourceQualification"]; ok {
			if _, err := exactObject(original, []string{"generation", "completionSha256", "qualifiedHeadSha256", "healthSha256", "closedSha256"}, false); err != nil {
				return r, ErrRefused
			}
		}
		var items []json.RawMessage
		if json.Unmarshal(object["ancestors"], &items) != nil || items == nil || len(items) > selectedHistoryPageSize {
			return r, ErrRefused
		}
		for _, item := range items {
			if _, err := exactObject(item, []string{"generation", "completionSha256"}, false); err != nil {
				return r, ErrRefused
			}
		}
	}
	if json.Unmarshal(raw, &r) != nil || !validMaintenanceResult(r, q) {
		return MaintenanceResult{}, ErrRefused
	}
	return r, nil
}
func maintenanceAck(q MaintenanceRequest) []byte {
	b, _ := json.Marshal(struct {
		Schema    int    `json:"schema"`
		Kind      string `json:"kind"`
		RequestID string `json:"requestId"`
	}{1, "maintenance-ack", q.RequestID})
	return b
}
func validMaintenanceAck(raw []byte, q MaintenanceRequest) bool {
	if _, err := exactObject(raw, []string{"schema", "kind", "requestId"}, false); err != nil {
		return false
	}
	var a struct {
		Schema    int    `json:"schema"`
		Kind      string `json:"kind"`
		RequestID string `json:"requestId"`
	}
	return json.Unmarshal(raw, &a) == nil && a.Schema == 1 && a.Kind == "maintenance-ack" && a.RequestID == q.RequestID
}
