package brokerpipe

import (
	"context"
	"encoding/json"
)

// Maintenance is a separate explicit elevated-operator conversation. Its data
// cannot authorize a mutation: the host must retain registration, selected
// history, original application exit and current profile custody independently.
const MaintenancePipeName = `\\.\pipe\SkagerUpdateBroker.maintenance.v1`
const ManualRollback = "manual-rollback"

type MaintenanceRequest struct {
	Schema                     int    `json:"schema"`
	Operation                  string `json:"operation"`
	RequestID                  string `json:"requestId"`
	ExpectedSelectedHeadSHA256 string `json:"expectedSelectedHeadSha256"`
	TargetGeneration           string `json:"targetGeneration"`
	TargetCompletionSHA256     string `json:"targetCompletionSha256"`
}
type MaintenanceResult struct {
	Schema             int    `json:"schema"`
	RequestID          string `json:"requestId"`
	State              string `json:"state"`
	Generation         string `json:"generation"`
	CompletionSHA256   string `json:"completionSha256"`
	SelectedHeadSHA256 string `json:"selectedHeadSha256"`
}
type MaintenanceHandler func(context.Context, Peer, MaintenanceRequest) (MaintenanceResult, error)

func validMaintenanceRequest(q MaintenanceRequest) bool {
	return q.Schema == 1 && q.Operation == ManualRollback && id32.MatchString(q.RequestID) && hex64.MatchString(q.ExpectedSelectedHeadSHA256) && id32.MatchString(q.TargetGeneration) && hex64.MatchString(q.TargetCompletionSHA256)
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
	if _, err = exactObject(raw, []string{"schema", "operation", "requestId", "expectedSelectedHeadSha256", "targetGeneration", "targetCompletionSha256"}, false); err != nil {
		return q, ErrRefused
	}
	if json.Unmarshal(raw, &q) != nil || !validMaintenanceRequest(q) {
		return MaintenanceRequest{}, ErrRefused
	}
	return q, nil
}
func validMaintenanceResult(r MaintenanceResult, q MaintenanceRequest) bool {
	return validMaintenanceRequest(q) && r.Schema == 1 && r.RequestID == q.RequestID && r.State == "manual-selected" && r.Generation == q.TargetGeneration && r.CompletionSHA256 == q.TargetCompletionSHA256 && hex64.MatchString(r.SelectedHeadSHA256) && r.SelectedHeadSHA256 != q.ExpectedSelectedHeadSHA256
}
func decodeMaintenanceResult(raw []byte, q MaintenanceRequest) (r MaintenanceResult, err error) {
	if len(raw) > maxFrame {
		return r, ErrRefused
	}
	if _, err = exactObject(raw, []string{"schema", "requestId", "state", "generation", "completionSha256", "selectedHeadSha256"}, false); err != nil {
		return r, ErrRefused
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
