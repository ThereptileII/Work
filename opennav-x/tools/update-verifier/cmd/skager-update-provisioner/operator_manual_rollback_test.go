package main

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"io"
	"strings"
	"testing"

	"example.com/opennav-update-verifier/brokerenroll"
	"example.com/opennav-update-verifier/brokerpipe"
)

func manualRollbackArgs() []string {
	return []string{"--maintenance=manual-rollback", "--request-id=" + strings.Repeat("1", 32), "--expected-selected-head-sha256=" + strings.Repeat("2", 64), "--target-generation=" + strings.Repeat("3", 32), "--target-completion-sha256=" + strings.Repeat("4", 64)}
}
func manualRollbackResult(q brokerpipe.MaintenanceRequest) brokerpipe.MaintenanceResult {
	return brokerpipe.MaintenanceResult{Schema: 1, RequestID: q.RequestID, State: "manual-selected", Generation: q.TargetGeneration, CompletionSHA256: q.TargetCompletionSHA256, SelectedHeadSHA256: strings.Repeat("5", 64)}
}
func TestOperatorManualRollbackCallsOnceAndPrintsBoundReadback(t *testing.T) {
	for _, separate := range []bool{false, true} {
		args := manualRollbackArgs()
		if separate {
			args = nil
			for _, arg := range manualRollbackArgs() {
				name, value, _ := strings.Cut(arg, "=")
				args = append(args, name, value)
			}
		}
		var output bytes.Buffer
		calls := 0
		var expected brokerpipe.MaintenanceResult
		err := runManualRollback(context.Background(), args, func(_ context.Context, q brokerpipe.MaintenanceRequest) (brokerpipe.MaintenanceResult, error) {
			calls++
			if q.Schema != 1 || q.Operation != brokerpipe.ManualRollback || q.RequestID != strings.Repeat("1", 32) || q.ExpectedSelectedHeadSHA256 != strings.Repeat("2", 64) || q.TargetGeneration != strings.Repeat("3", 32) || q.TargetCompletionSHA256 != strings.Repeat("4", 64) {
				t.Fatalf("unexpected request: %+v", q)
			}
			expected = manualRollbackResult(q)
			return expected, nil
		}, &output)
		if err != nil || calls != 1 {
			t.Fatalf("err=%v calls=%d", err, calls)
		}
		var actual brokerpipe.MaintenanceResult
		if json.Unmarshal(output.Bytes(), &actual) != nil || actual != expected {
			t.Fatalf("readback=%s", output.String())
		}
	}
}
func TestOperatorManualRollbackInvalidInputHasNoTransportEffect(t *testing.T) {
	mutations := map[string]func([]string) []string{
		"missing":            func(a []string) []string { return a[:4] },
		"duplicate":          func(a []string) []string { return append(a, a[0]) },
		"conflicting-action": func(a []string) []string { return append(a, "--maintenance=quiesce-maintenance") },
		"arbitrary-root":     func(a []string) []string { return append(a, "--root=C:/private") },
		"supplied-sid":       func(a []string) []string { return append(a, "--client-sid=S-1-5-18") },
		"enrollment":         func(a []string) []string { return append(a, "--client-registration-request=C:/private") },
		"positional":         func(a []string) []string { return append(a, "private") },
		"single-dash":        func(a []string) []string { a[1] = "-request-id=" + strings.Repeat("1", 32); return a },
		"empty-id":           func(a []string) []string { a[1] = "--request-id="; return a },
		"upper-id":           func(a []string) []string { a[1] = "--request-id=" + strings.Repeat("A", 32); return a },
		"short-id":           func(a []string) []string { a[1] = "--request-id=1234"; return a },
		"bad-head": func(a []string) []string {
			a[2] = "--expected-selected-head-sha256=" + strings.Repeat("A", 64)
			return a
		},
		"path-generation": func(a []string) []string { a[3] = "--target-generation=C:/private"; return a },
		"bad-completion":  func(a []string) []string { a[4] = "--target-completion-sha256=" + strings.Repeat("g", 64); return a },
		"missing-value":   func(a []string) []string { a[1] = "--request-id"; return a },
		"other-action":    func(a []string) []string { a[0] = "--maintenance=uninstall"; return a },
	}
	for name, mutate := range mutations {
		t.Run(name, func(t *testing.T) {
			var output bytes.Buffer
			calls := 0
			err := runManualRollback(context.Background(), mutate(manualRollbackArgs()), func(context.Context, brokerpipe.MaintenanceRequest) (brokerpipe.MaintenanceResult, error) {
				calls++
				return brokerpipe.MaintenanceResult{}, nil
			}, &output)
			if err == nil || calls != 0 || output.Len() != 0 || strings.Contains(err.Error(), "private") {
				t.Fatalf("err=%v calls=%d output=%s", err, calls, output.String())
			}
		})
	}
}
func TestOperatorManualRollbackSuppressesUnverifiedResultsWithoutRetry(t *testing.T) {
	mutations := map[string]func(*brokerpipe.MaintenanceResult){
		"schema":          func(r *brokerpipe.MaintenanceResult) { r.Schema = 2 },
		"request":         func(r *brokerpipe.MaintenanceResult) { r.RequestID = strings.Repeat("6", 32) },
		"state":           func(r *brokerpipe.MaintenanceResult) { r.State = "C:/private" },
		"generation":      func(r *brokerpipe.MaintenanceResult) { r.Generation = "C:/private" },
		"completion":      func(r *brokerpipe.MaintenanceResult) { r.CompletionSHA256 = strings.Repeat("6", 64) },
		"head":            func(r *brokerpipe.MaintenanceResult) { r.SelectedHeadSHA256 = "C:/private" },
		"unchanged-head":  func(r *brokerpipe.MaintenanceResult) { r.SelectedHeadSHA256 = strings.Repeat("2", 64) },
		"transport-error": func(*brokerpipe.MaintenanceResult) {},
		"cancellation":    func(*brokerpipe.MaintenanceResult) {},
	}
	for name, mutate := range mutations {
		t.Run(name, func(t *testing.T) {
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			var output bytes.Buffer
			calls := 0
			err := runManualRollback(ctx, manualRollbackArgs(), func(_ context.Context, q brokerpipe.MaintenanceRequest) (brokerpipe.MaintenanceResult, error) {
				calls++
				r := manualRollbackResult(q)
				mutate(&r)
				if name == "cancellation" {
					cancel()
				}
				if name == "transport-error" {
					return r, errors.New("C:/private secret")
				}
				return r, nil
			}, &output)
			if err == nil || calls != 1 || output.Len() != 0 || strings.Contains(err.Error(), "private") || strings.Contains(err.Error(), "secret") {
				t.Fatalf("err=%v calls=%d output=%s", err, calls, output.String())
			}
		})
	}
}

type manualRollbackFailWriter struct{ calls int }

func (w *manualRollbackFailWriter) Write([]byte) (int, error) {
	w.calls++
	return 0, errors.New("private writer failure")
}
func TestOperatorManualRollbackOutputFailureDoesNotRetry(t *testing.T) {
	calls := 0
	output := &manualRollbackFailWriter{}
	err := runManualRollback(context.Background(), manualRollbackArgs(), func(_ context.Context, q brokerpipe.MaintenanceRequest) (brokerpipe.MaintenanceResult, error) {
		calls++
		return manualRollbackResult(q), nil
	}, output)
	if err == nil || calls != 1 || output.calls != 1 || strings.Contains(err.Error(), "private") {
		t.Fatalf("err=%v calls=%d writes=%d", err, calls, output.calls)
	}
}
func TestOperatorManualRollbackRequiresLiveDependencies(t *testing.T) {
	canceled, cancel := context.WithCancel(context.Background())
	cancel()
	for _, ctx := range []context.Context{nil, canceled} {
		calls := 0
		err := runManualRollback(ctx, manualRollbackArgs(), func(context.Context, brokerpipe.MaintenanceRequest) (brokerpipe.MaintenanceResult, error) {
			calls++
			return brokerpipe.MaintenanceResult{}, nil
		}, io.Discard)
		if err == nil || calls != 0 {
			t.Fatalf("err=%v calls=%d", err, calls)
		}
	}
	if runManualRollback(context.Background(), manualRollbackArgs(), nil, io.Discard) == nil {
		t.Fatal("nil caller accepted")
	}
	if runManualRollback(context.Background(), manualRollbackArgs(), func(context.Context, brokerpipe.MaintenanceRequest) (brokerpipe.MaintenanceResult, error) {
		t.Fatal("called without output")
		return brokerpipe.MaintenanceResult{}, nil
	}, nil) == nil {
		t.Fatal("nil output accepted")
	}
}
func TestOperatorManualRollbackExplicitDispatch(t *testing.T) {
	for _, args := range [][]string{manualRollbackArgs(), {"--maintenance", "manual-rollback"}, {"--maintenance=quiesce-maintenance", "--maintenance=manual-rollback"}} {
		if !manualRollbackRequested(args) {
			t.Fatalf("manual action missed: %q", args)
		}
	}
	for _, args := range [][]string{nil, {"--maintenance=quiesce-maintenance"}, {"--maintenance=uninstall"}, {"--maintenance-other=manual-rollback"}, {"--maintenance=manual-rollback-extra"}, {"private", "manual-rollback"}} {
		if manualRollbackRequested(args) {
			t.Fatalf("nonmanual action selected: %q", args)
		}
	}
	// Malformed explicit manual dispatch cannot read local inputs or enter the
	// quiesce opener. It fails before reaching the native transport.
	maintenance, err := runOperator(context.Background(), []string{"--maintenance=manual-rollback"}, func(string) ([]byte, error) { t.Fatal("manual dispatch read local input"); return nil, nil }, func(context.Context, brokerenroll.Request) error { t.Fatal("manual dispatch enrolled"); return nil }, func(context.Context, brokerenroll.Request) error {
		t.Fatal("manual dispatch activated startup")
		return nil
	}, func(context.Context, brokerenroll.MaintenanceRequest) (maintenanceOwner, error) {
		t.Fatal("manual dispatch quiesced broker")
		return nil, nil
	})
	if err == nil || !maintenance {
		t.Fatal("incomplete manual dispatch accepted or not isolated")
	}
}
