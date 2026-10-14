package brokerpipe

import (
	"encoding/json"
	"strings"
	"testing"
)

func previousQualifiedFixture() (MaintenanceRequest, MaintenanceResult) {
	q := MaintenanceRequest{Schema: 1, Operation: PreviousQualified, RequestID: strings.Repeat("1", 32)}
	r := MaintenanceResult{Schema: 1, RequestID: q.RequestID, State: PreviousQualified, Generation: strings.Repeat("2", 32), CompletionSHA256: strings.Repeat("3", 64), SelectedHeadSHA256: strings.Repeat("4", 64), Version: "1.0.1", Commit: strings.Repeat("5", 40),
		Previous: &RollbackTarget{Generation: strings.Repeat("6", 32), CompletionSHA256: strings.Repeat("7", 64), Version: "1.0.0", Commit: strings.Repeat("8", 40)}}
	return q, r
}

func TestPreviousQualifiedStrictOptionalProtocolAndOldWireIsolation(t *testing.T) {
	q, r := previousQualifiedFixture()
	request, err := encodeMaintenanceRequest(q)
	if err != nil {
		t.Fatal(err)
	}
	parsed, err := decodeMaintenanceRequest(request)
	if err != nil || parsed != q {
		t.Fatal(parsed, err)
	}
	response, _ := json.Marshal(r)
	if len(response) > maxFrame || !ValidPreviousQualifiedResult(r, q) {
		t.Fatal("unbounded or invalid response")
	}
	if _, err := decodeMaintenanceResult(response, q); err != nil {
		t.Fatal(err)
	}
	for _, bad := range []string{
		strings.Replace(string(request), `"schema":1`, `"schema":1,"schema":1`, 1),
		string(request[:len(request)-1]) + `,"page":0}`,
		string(request[:len(request)-1]) + `,"targetGeneration":"` + strings.Repeat("a", 32) + `"}`,
		strings.Replace(string(response), `"previous":{`, `"previous":{"path":"C:/arbitrary",`, 1),
		strings.Replace(string(response), `"previous":{`, `"previous":{"generation":"`+r.Previous.Generation+`",`, 1),
	} {
		if strings.Contains(bad, `"state"`) {
			if _, err := decodeMaintenanceResult([]byte(bad), q); err == nil {
				t.Fatal("nested fields accepted")
			}
		} else if _, err := decodeMaintenanceRequest([]byte(bad)); err == nil {
			t.Fatal("caller target/unknown fields accepted")
		}
	}
	for _, mutate := range []func(*MaintenanceResult){
		func(r *MaintenanceResult) { r.Previous = nil }, func(r *MaintenanceResult) { p := *r.Previous; p.Generation = r.Generation; r.Previous = &p },
		func(r *MaintenanceResult) { p := *r.Previous; p.Version = "not a version"; r.Previous = &p }, func(r *MaintenanceResult) { r.RequestID = strings.Repeat("f", 32) },
		func(r *MaintenanceResult) { r.QualificationSHA256 = strings.Repeat("f", 64) },
	} {
		bad := r
		mutate(&bad)
		if ValidPreviousQualifiedResult(bad, q) {
			t.Fatal("unbound target accepted", bad)
		}
	}
	// An old selected-history/manual response is not the new capability, and the
	// new field must never leak into those unchanged operation schemas.
	oldQ, oldR := selectedDiagnosticsFixture(0)
	if ValidPreviousQualifiedResult(oldR, q) {
		t.Fatal("old broker response mistaken for target support")
	}
	oldR.Previous = r.Previous
	if validMaintenanceResult(oldR, oldQ) {
		t.Fatal("new field accepted on original history operation")
	}
	oldQ = maintenanceFixture()
	oldR = MaintenanceResult{Schema: 1, RequestID: oldQ.RequestID, State: "manual-selected", Generation: oldQ.TargetGeneration, CompletionSHA256: oldQ.TargetCompletionSHA256, SelectedHeadSHA256: strings.Repeat("e", 64)}
	oldWire, _ := json.Marshal(oldR)
	if strings.Contains(string(oldWire), "previous") {
		t.Fatal("original manual wire changed")
	}
	oldR.Previous = r.Previous
	if validMaintenanceResult(oldR, oldQ) {
		t.Fatal("new field accepted on manual completion")
	}
}
