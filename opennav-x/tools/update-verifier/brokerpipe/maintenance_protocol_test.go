package brokerpipe

import (
	"encoding/json"
	"strings"
	"testing"
)

func maintenanceFixture() MaintenanceRequest {
	return MaintenanceRequest{Schema: 1, Operation: ManualRollback, RequestID: strings.Repeat("a", 32), ExpectedSelectedHeadSHA256: strings.Repeat("b", 64), TargetGeneration: strings.Repeat("c", 32), TargetCompletionSHA256: strings.Repeat("d", 64)}
}
func TestMaintenanceProtocolStrictBindings(t *testing.T) {
	q := maintenanceFixture()
	raw, err := encodeMaintenanceRequest(q)
	if err != nil {
		t.Fatal(err)
	}
	parsed, err := decodeMaintenanceRequest(raw)
	if err != nil || parsed != q {
		t.Fatal(parsed, err)
	}
	for _, bad := range []string{strings.Replace(string(raw), `"schema":1`, `"schema":1,"schema":1`, 1), strings.Replace(string(raw), `"schema":1`, `"Schema":1`, 1), strings.Replace(string(raw), `"manual-rollback"`, `"repair"`, 1), strings.Replace(string(raw), q.TargetGeneration, `C:\arbitrary`, 1), string(raw) + `{}`, strings.Repeat(" ", maxFrame+1), `null`} {
		if _, err := decodeMaintenanceRequest([]byte(bad)); err == nil {
			t.Fatal("unbound maintenance request accepted")
		}
	}
	r := MaintenanceResult{Schema: 1, RequestID: q.RequestID, State: "manual-selected", Generation: q.TargetGeneration, CompletionSHA256: q.TargetCompletionSHA256, SelectedHeadSHA256: strings.Repeat("e", 64)}
	b, _ := json.Marshal(r)
	if _, err := decodeMaintenanceResult(b, q); err != nil {
		t.Fatal(err)
	}
	for _, field := range []string{"request", "state", "generation", "completion", "head"} {
		t.Run(field, func(t *testing.T) {
			bad := r
			switch field {
			case "request":
				bad.RequestID = strings.Repeat("f", 32)
			case "state":
				bad.State = "healthy"
			case "generation":
				bad.Generation = strings.Repeat("f", 32)
			case "completion":
				bad.CompletionSHA256 = strings.Repeat("f", 64)
			case "head":
				bad.SelectedHeadSHA256 = q.ExpectedSelectedHeadSHA256
			}
			b, _ := json.Marshal(bad)
			if _, err := decodeMaintenanceResult(b, q); err == nil {
				t.Fatal("mismatched maintenance completion accepted")
			}
		})
	}
	if !validMaintenanceAck(maintenanceAck(q), q) {
		t.Fatal("bound acknowledgement refused")
	}
	other := q
	other.RequestID = strings.Repeat("f", 32)
	if validMaintenanceAck(maintenanceAck(other), q) {
		t.Fatal("other request acknowledgement accepted")
	}
}
