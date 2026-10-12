package brokerpipe

import (
	"encoding/json"
	"fmt"
	"strings"
	"testing"
)

func selectedDiagnosticsFixture(page int) (MaintenanceRequest, MaintenanceResult) {
	q := MaintenanceRequest{Schema: 1, Operation: SelectedHistory, RequestID: strings.Repeat("1", 32), Page: &page}
	items := []MaintenanceAncestor{{Generation: strings.Repeat("2", 32), CompletionSHA256: strings.Repeat("3", 64)}}
	r := MaintenanceResult{Schema: 1, RequestID: q.RequestID, State: SelectedHistory,
		Generation: strings.Repeat("4", 32), CompletionSHA256: strings.Repeat("5", 64), SelectedHeadSHA256: strings.Repeat("6", 64),
		Version: "1.0.0-beta.2", Commit: strings.Repeat("7", 40), QualificationSHA256: strings.Repeat("8", 64),
		QualificationClosedSHA256: strings.Repeat("9", 64), InventorySHA256: strings.Repeat("a", 64), Page: &page, Ancestors: &items}
	if page > 0 {
		q.ExpectedSelectedHeadSHA256, q.InventorySHA256 = r.SelectedHeadSHA256, r.InventorySHA256
	}
	return q, r
}

func TestSelectedHistoryStrictPagesAndNoManualWireChange(t *testing.T) {
	for _, page := range []int{0, 1} {
		q, r := selectedDiagnosticsFixture(page)
		wire, err := encodeMaintenanceRequest(q)
		if err != nil {
			t.Fatal(err)
		}
		parsed, err := decodeMaintenanceRequest(wire)
		if err != nil || parsed.Operation != q.Operation || parsed.Page == nil || *parsed.Page != page || parsed.ExpectedSelectedHeadSHA256 != q.ExpectedSelectedHeadSHA256 || parsed.InventorySHA256 != q.InventorySHA256 {
			t.Fatal(parsed, err)
		}
		response, _ := json.Marshal(r)
		if _, err := decodeMaintenanceResult(response, q); err != nil {
			t.Fatal(err)
		}
		for _, bad := range []string{strings.Replace(string(wire), `"page":`, `"Page":`, 1), strings.Replace(string(wire), `"schema":1`, `"schema":1,"schema":1`, 1), string(wire[:len(wire)-1]) + `,"path":"C:/private"}`} {
			if _, err := decodeMaintenanceRequest([]byte(bad)); err == nil {
				t.Fatal("noncanonical diagnostic accepted", bad)
			}
		}
		bad := r
		wrong := page + 1
		bad.Page = &wrong
		wire, _ = json.Marshal(bad)
		if _, err := decodeMaintenanceResult(wire, q); err == nil {
			t.Fatal("wrong diagnostic page accepted")
		}
	}
	q, r := selectedDiagnosticsFixture(1)
	q.InventorySHA256 = strings.Repeat("b", 64)
	if ValidSelectedHistoryResult(r, q) {
		t.Fatal("mixed original inventory accepted")
	}
	q, r = selectedDiagnosticsFixture(0)
	r.Ancestors = nil
	if ValidSelectedHistoryResult(r, q) {
		t.Fatal("missing bounded ancestor list accepted")
	}
	q, r = selectedDiagnosticsFixture(0)
	response, _ := json.Marshal(r)
	response = []byte(strings.Replace(string(response), `"completionSha256":"`+strings.Repeat("3", 64)+`"`, `"completionSha256":"`+strings.Repeat("3", 64)+`","path":"C:/private"`, 1))
	if _, err := decodeMaintenanceResult(response, q); err == nil {
		t.Fatal("nested unapproved ancestor field accepted")
	}
	q, r = selectedDiagnosticsFixture(0)
	items := make([]MaintenanceAncestor, selectedHistoryPageSize)
	for i := range items {
		items[i] = MaintenanceAncestor{Generation: fmt.Sprintf("%032x", i+1), CompletionSHA256: strings.Repeat("c", 64)}
	}
	r.Ancestors = &items
	next := 1
	r.NextPage = &next
	response, _ = json.Marshal(r)
	if len(response) > maxFrame || !ValidSelectedHistoryResult(r, q) {
		t.Fatal("full selected-history page exceeded fixed frame or failed strict validation", len(response))
	}
	manual := maintenanceFixture()
	raw, err := encodeMaintenanceRequest(manual)
	if err != nil || strings.Contains(string(raw), `"page"`) || strings.Contains(string(raw), `"inventorySha256"`) {
		t.Fatal("manual rollback request wire changed", err, string(raw))
	}
}
