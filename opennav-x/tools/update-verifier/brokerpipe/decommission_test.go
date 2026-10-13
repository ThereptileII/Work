package brokerpipe

import (
	"context"
	"encoding/json"
	"errors"
	"reflect"
	"strings"
	"testing"
)

type decommissionWitnessFixture struct {
	checks, closes    int
	failure, closeErr error
}

func (f *decommissionWitnessFixture) check(context.Context) error { f.checks++; return f.failure }
func (f *decommissionWitnessFixture) close() error                { f.closes++; return f.closeErr }
func decommissionFixture() (MaintenanceRequest, MaintenanceResult) {
	q := MaintenanceRequest{Schema: 1, Operation: PrepareDecommission, RequestID: strings.Repeat("1", 32), ExpectedSelectedHeadSHA256: strings.Repeat("2", 64)}
	r := MaintenanceResult{Schema: 1, RequestID: q.RequestID, State: "decommission-prepared", Generation: strings.Repeat("3", 32), CompletionSHA256: strings.Repeat("4", 64), SelectedHeadSHA256: q.ExpectedSelectedHeadSHA256, ClientSID: "S-1-5-21-1-2-3-1001", RegistrationSHA256: strings.Repeat("5", 64), CurrentProfileSHA256: strings.Repeat("6", 64), CurrentProfileIdentitySHA256: strings.Repeat("7", 64), FenceSHA256: strings.Repeat("8", 64)}
	return q, r
}
func TestDecommissionProtocolExactRequestAndResponse(t *testing.T) {
	q, r := decommissionFixture()
	raw, err := encodeMaintenanceRequest(q)
	if err != nil {
		t.Fatal(err)
	}
	if _, err = decodeMaintenanceRequest(raw); err != nil {
		t.Fatal(err)
	}
	response, _ := json.Marshal(r)
	if _, err = decodeMaintenanceResult(response, q); err != nil {
		t.Fatal(err)
	}
	for _, bad := range []string{strings.Replace(string(raw), "\"requestId\":", "\"requestId\":\"bad\",\"requestId\":", 1), strings.TrimSuffix(string(raw), "}") + `,"targetGeneration":"` + strings.Repeat("a", 32) + `"}`, strings.TrimSuffix(string(raw), "}") + `,"page":0}`} {
		if _, err := decodeMaintenanceRequest([]byte(bad)); err == nil {
			t.Fatalf("accepted %s", bad)
		}
	}
	for _, change := range []func(*MaintenanceResult){func(r *MaintenanceResult) { r.RequestID = strings.Repeat("9", 32) }, func(r *MaintenanceResult) { r.SelectedHeadSHA256 = strings.Repeat("9", 64) }, func(r *MaintenanceResult) { r.ClientSID = "S-1-5-18" }, func(r *MaintenanceResult) { r.FenceSHA256 = "" }, func(r *MaintenanceResult) { r.Version = "1.0.0" }} {
		bad := r
		change(&bad)
		b, _ := json.Marshal(bad)
		if _, err := decodeMaintenanceResult(b, q); err == nil {
			t.Fatal("accepted changed provenance")
		}
	}
	for _, bad := range []string{strings.TrimSuffix(string(response), "}") + `,"unknown":1}`, strings.Replace(string(response), `"fenceSha256":`, `"fenceSha256":"bad","fenceSha256":`, 1)} {
		if _, err := decodeMaintenanceResult([]byte(bad), q); err == nil {
			t.Fatal("accepted extra/duplicate")
		}
	}
	manual := MaintenanceRequest{Schema: 1, Operation: ManualRollback, RequestID: q.RequestID, ExpectedSelectedHeadSHA256: q.ExpectedSelectedHeadSHA256, TargetGeneration: r.Generation, TargetCompletionSHA256: r.CompletionSHA256}
	r.State = "manual-selected"
	r.SelectedHeadSHA256 = strings.Repeat("9", 64)
	if validMaintenanceResult(r, manual) {
		t.Fatal("decommission fields escaped into ordinary result")
	}
}
func TestDecommissionWitnessOriginalCustodyAndCleanup(t *testing.T) {
	q, r := decommissionFixture()
	ctx, cancel := context.WithCancel(context.Background())
	f := &decommissionWitnessFixture{}
	w, err := newDecommissionWitness(ctx, q, r, 11, 12, f)
	if err != nil {
		t.Fatal(err)
	}
	if err = w.Check(ctx, 11, 12); err != nil {
		t.Fatal(err)
	}
	snapshot := w.Snapshot()
	snapshot.ClientSID = "changed"
	if w.Snapshot().ClientSID != r.ClientSID {
		t.Fatal("aliased snapshot")
	}
	copyW := reflect.New(reflect.TypeOf(w).Elem())
	copyW.Elem().Set(reflect.ValueOf(w).Elem())
	copied := copyW.Interface().(*DecommissionWitness)
	for _, bad := range []*DecommissionWitness{nil, {}, copied} {
		if bad.Check(ctx, 11, 12) == nil || bad.Snapshot() != (DecommissionSnapshot{}) {
			t.Fatal("copied/zero authority")
		}
		if bad != nil && bad.Close() == nil {
			t.Fatal("copied/zero drained")
		}
	}
	if f.closes != 0 {
		t.Fatal("copy drained original")
	}
	if _, err = json.Marshal(w); err == nil {
		t.Fatal("serialized witness")
	}
	if json.Unmarshal([]byte(`{}`), w) == nil {
		t.Fatal("decoded witness")
	}
	if w.Check(ctx, 11, 13) == nil {
		t.Fatal("wrong original birth")
	}
	cancel()
	if w.Check(context.Background(), 11, 12) == nil {
		t.Fatal("canceled original accepted")
	}
	failure := errors.New("native close failure")
	f.closeErr = failure
	if !errors.Is(w.Close(), failure) || !errors.Is(w.Close(), failure) || f.closes != 1 {
		t.Fatal("closure error lost/repeated")
	}
}
func TestDecommissionWitnessFailureSticky(t *testing.T) {
	q, r := decommissionFixture()
	f := &decommissionWitnessFixture{failure: errors.New("changed owner")}
	w, err := newDecommissionWitness(context.Background(), q, r, 11, 12, f)
	if err != nil {
		t.Fatal(err)
	}
	if w.Check(context.Background(), 11, 12) == nil {
		t.Fatal("changed owner accepted")
	}
	f.failure = nil
	if w.Check(context.Background(), 11, 12) == nil || f.checks != 1 {
		t.Fatal("poisoned custody recovered")
	}
	if w.Close() != nil || f.closes != 1 {
		t.Fatal("poisoned owner cleanup")
	}
}

func TestProtectedUninstallProtocolCannotDowngradeToFenceOnly(t *testing.T) {
	q, r := decommissionFixture()
	q.Operation = PrepareUninstall
	r.State = "uninstall-prepared"
	r.RetirementSHA256 = strings.Repeat("9", 64)
	request, err := encodeMaintenanceRequest(q)
	if err != nil {
		t.Fatal(err)
	}
	if _, err = decodeMaintenanceRequest(request); err != nil {
		t.Fatal(err)
	}
	response, _ := json.Marshal(r)
	if _, err = decodeMaintenanceResult(response, q); err != nil {
		t.Fatal(err)
	}
	backend := &decommissionWitnessFixture{}
	witness, err := newDecommissionWitness(context.Background(), q, r, 11, 12, backend)
	if err != nil || witness.Snapshot().RetirementSHA256 != r.RetirementSHA256 {
		t.Fatal("retirement omitted from sealed custody", err)
	}
	if err = witness.Check(context.Background(), 11, 12); err != nil {
		t.Fatal(err)
	}
	if err = witness.Close(); err != nil {
		t.Fatal(err)
	}
	for _, mutate := range []func(*MaintenanceResult){
		func(r *MaintenanceResult) { r.RetirementSHA256 = "" },
		func(r *MaintenanceResult) { r.RetirementSHA256 = "invalid" },
		func(r *MaintenanceResult) { r.State = "decommission-prepared" },
		func(r *MaintenanceResult) { r.FenceSHA256 = "" },
	} {
		bad := r
		mutate(&bad)
		raw, _ := json.Marshal(bad)
		if _, err := decodeMaintenanceResult(raw, q); err == nil {
			t.Fatal("uninstall accepted incomplete retirement")
		}
		if _, err := newDecommissionWitness(context.Background(), q, bad, 11, 12, &decommissionWitnessFixture{}); err == nil {
			t.Fatal("incomplete retirement constructed witness")
		}
	}
	plain, _ := decommissionFixture()
	if validMaintenanceResult(r, plain) {
		t.Fatal("uninstall response accepted for service-only request")
	}
	if _, err := decodeMaintenanceResult(response, plain); err == nil {
		t.Fatal("service-only schema expanded")
	}
	q.Operation = ManualRollback
	if validMaintenanceResult(r, q) {
		t.Fatal("uninstall response accepted by ordinary maintenance")
	}
}
