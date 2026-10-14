package brokerpipe

import (
	"encoding/json"
	"strings"
	"testing"
)

func TestRepairWireRejectsCallerAuthorityAndFalseHealth(t *testing.T) {
	for _, operation := range []string{RepairInspect, RepairCurrent} {
		q := MaintenanceRequest{Schema: 1, Operation: operation, RequestID: strings.Repeat("a", 32)}
		if operation == RepairCurrent {
			q.ExpectedSelectedHeadSHA256 = strings.Repeat("b", 64)
		}
		raw, err := encodeMaintenanceRequest(q)
		if err != nil {
			t.Fatal(err)
		}
		if _, err = decodeMaintenanceRequest(raw); err != nil {
			t.Fatal(err)
		}
		for _, field := range []string{`"source":"C:\\source"`, `"targetGeneration":"` + strings.Repeat("c", 32) + `"`, `"clientSid":"S-1-5-21-1-2-3-1001"`, `"page":0`} {
			injected := append(append([]byte{}, raw[:len(raw)-1]...), []byte(","+field+"}")...)
			if _, err = decodeMaintenanceRequest(injected); err == nil {
				t.Fatal("caller authority accepted", operation, field)
			}
		}
		r := MaintenanceResult{Schema: 1, RequestID: q.RequestID, State: RepairInspect, Generation: strings.Repeat("c", 32), CompletionSHA256: strings.Repeat("d", 64), SelectedHeadSHA256: strings.Repeat("e", 64), Version: "1.0.1", Commit: strings.Repeat("f", 40)}
		if operation == RepairCurrent {
			r.State = "repair-selected"
			r.Generation = q.RequestID
		}
		data, _ := json.Marshal(r)
		if _, err = decodeMaintenanceResult(data, q); err != nil {
			t.Fatal(err)
		}
		r.QualificationSHA256 = strings.Repeat("0", 64)
		data, _ = json.Marshal(r)
		if _, err = decodeMaintenanceResult(data, q); err == nil {
			t.Fatal("repair falsely claimed qualification")
		}
	}
}
func TestRepairedHistorySeparatesOriginalQualificationAndClosesSchema(t *testing.T) {
	page := 0
	ancestors := []MaintenanceAncestor{}
	q := MaintenanceRequest{Schema: 1, Operation: SelectedHistory, RequestID: strings.Repeat("a", 32), Page: &page}
	r := MaintenanceResult{Schema: 1, RequestID: q.RequestID, State: SelectedHistory, Generation: strings.Repeat("b", 32), CompletionSHA256: strings.Repeat("c", 64), SelectedHeadSHA256: strings.Repeat("d", 64), Version: "1.0.1", Commit: strings.Repeat("e", 40), InventorySHA256: strings.Repeat("f", 64), Page: &page, Ancestors: &ancestors, RepairSourceQualification: &RepairQualification{Generation: strings.Repeat("0", 32), CompletionSHA256: strings.Repeat("1", 64), QualifiedHeadSHA256: strings.Repeat("2", 64), HealthSHA256: strings.Repeat("3", 64), ClosedSHA256: strings.Repeat("4", 64)}}
	data, _ := json.Marshal(r)
	if _, err := decodeMaintenanceResult(data, q); err != nil {
		t.Fatal("explicit source qualification refused", err)
	}
	r.QualificationSHA256 = strings.Repeat("5", 64)
	r.QualificationClosedSHA256 = strings.Repeat("6", 64)
	data, _ = json.Marshal(r)
	if _, err := decodeMaintenanceResult(data, q); err == nil {
		t.Fatal("source health also advertised as repaired health")
	}
	r.QualificationSHA256 = ""
	r.QualificationClosedSHA256 = ""
	r.RepairSourceQualification.Generation = r.Generation
	data, _ = json.Marshal(r)
	if _, err := decodeMaintenanceResult(data, q); err == nil {
		t.Fatal("repaired generation became its own original qualifier")
	}
}
