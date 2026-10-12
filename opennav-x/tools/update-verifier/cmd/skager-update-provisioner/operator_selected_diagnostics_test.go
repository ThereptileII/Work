package main

import (
	"bytes"
	"context"
	"errors"
	"strings"
	"testing"

	"example.com/opennav-update-verifier/brokerpipe"
)

func selectedDiagnosticArgs() []string {
	return []string{"--maintenance=selected-history", "--request-id=" + strings.Repeat("1", 32), "--page=0"}
}

func selectedDiagnosticResponse(q brokerpipe.MaintenanceRequest) brokerpipe.MaintenanceResult {
	items := []brokerpipe.MaintenanceAncestor{{Generation: strings.Repeat("2", 32), CompletionSHA256: strings.Repeat("3", 64)}}
	return brokerpipe.MaintenanceResult{Schema: 1, RequestID: q.RequestID, State: brokerpipe.SelectedHistory,
		Generation: strings.Repeat("4", 32), CompletionSHA256: strings.Repeat("5", 64), SelectedHeadSHA256: strings.Repeat("6", 64),
		Version: "1.0.0-beta.2", Commit: strings.Repeat("7", 40), QualificationSHA256: strings.Repeat("8", 64),
		QualificationClosedSHA256: strings.Repeat("9", 64), InventorySHA256: strings.Repeat("a", 64), Page: q.Page, Ancestors: &items}
}

func TestSelectedHistoryCLIReadsOneBoundedPageAndPinsContinuation(t *testing.T) {
	var out bytes.Buffer
	calls := 0
	call := func(_ context.Context, q brokerpipe.MaintenanceRequest) (brokerpipe.MaintenanceResult, error) {
		calls++
		return selectedDiagnosticResponse(q), nil
	}
	if err := runSelectedHistory(context.Background(), selectedDiagnosticArgs(), call, &out); err != nil || calls != 1 || !bytes.Contains(out.Bytes(), []byte(`"state":"selected-history"`)) {
		t.Fatal(err, calls, out.String())
	}
	out.Reset()
	args := append([]string{}, selectedDiagnosticArgs()...)
	args[2] = "--page=1"
	args = append(args, "--expected-selected-head-sha256="+strings.Repeat("6", 64), "--inventory-sha256="+strings.Repeat("a", 64))
	if err := runSelectedHistory(context.Background(), args, call, &out); err != nil || calls != 2 {
		t.Fatal(err, calls)
	}
	for _, bad := range [][]string{
		{"--maintenance=selected-history", "--request-id=" + strings.Repeat("1", 32)},
		append(selectedDiagnosticArgs(), "--root=C:/private"),
		append(selectedDiagnosticArgs(), "--page=0"),
		{"--maintenance=selected-history", "--request-id=" + strings.Repeat("1", 32), "--page=1"},
	} {
		before := calls
		out.Reset()
		if err := runSelectedHistory(context.Background(), bad, call, &out); err == nil || calls != before || out.Len() != 0 {
			t.Fatal("invalid diagnostic had transport/output", bad, err)
		}
	}
	out.Reset()
	if err := runSelectedHistory(context.Background(), selectedDiagnosticArgs(), func(_ context.Context, q brokerpipe.MaintenanceRequest) (brokerpipe.MaintenanceResult, error) {
		return selectedDiagnosticResponse(q), errors.New("private path")
	}, &out); err == nil || strings.Contains(err.Error(), "private") || out.Len() != 0 {
		t.Fatal("unverified diagnostic disclosed response", err, out.String())
	}
}
