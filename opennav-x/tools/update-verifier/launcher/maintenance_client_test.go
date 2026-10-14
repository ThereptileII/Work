package launcher

import (
	"context"
	"errors"
	"reflect"
	"strings"
	"testing"

	"example.com/opennav-update-verifier/brokerpipe"
)

func maintenanceClientTestRequest() brokerpipe.MaintenanceRequest {
	return brokerpipe.MaintenanceRequest{Schema: 1, Operation: brokerpipe.ManualRollback, RequestID: strings.Repeat("1", 32), ExpectedSelectedHeadSHA256: strings.Repeat("2", 64), TargetGeneration: strings.Repeat("3", 32), TargetCompletionSHA256: strings.Repeat("4", 64)}
}
func maintenanceClientTestResult(q brokerpipe.MaintenanceRequest) brokerpipe.MaintenanceResult {
	return brokerpipe.MaintenanceResult{Schema: 1, RequestID: q.RequestID, State: "manual-selected", Generation: q.TargetGeneration, CompletionSHA256: q.TargetCompletionSHA256, SelectedHeadSHA256: strings.Repeat("5", 64)}
}

type maintenanceClientFixtureOwner struct {
	name      string
	events    *[]string
	checks    int
	failures  map[string]error
	closeHook func()
}

func (o *maintenanceClientFixtureOwner) Validate(context.Context) error {
	o.checks++
	name := o.name + "-before"
	if o.checks > 1 {
		name = o.name + "-after"
	}
	*o.events = append(*o.events, name)
	return o.failures[name]
}
func (o *maintenanceClientFixtureOwner) Close() error {
	name := o.name + "-close"
	*o.events = append(*o.events, name)
	if o.closeHook != nil {
		o.closeHook()
	}
	return o.failures[name]
}
func TestMaintenanceClientCustodyOrderingAndFailures(t *testing.T) {
	faults := []string{"", "query-prepare", "query-partial", "query-nil", "folder-prepare", "folder-partial", "folder-nil", "query-before", "folder-before", "call", "query-after", "folder-after", "folder-close", "query-close", "cancel-call", "cancel-folder-close", "mismatch"}
	for _, fault := range faults {
		t.Run(fault, func(t *testing.T) {
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			var events []string
			sentinel := errors.New("retained ownership uncertain")
			failures := map[string]error{fault: sentinel}
			query := &maintenanceClientFixtureOwner{name: "query", events: &events, failures: failures}
			folder := &maintenanceClientFixtureOwner{name: "folder", events: &events, failures: failures}
			if fault == "cancel-folder-close" {
				folder.closeHook = cancel
			}
			q := maintenanceClientTestRequest()
			calls := 0
			result, err := callMaintenanceWithCustody(ctx, q, func(context.Context) (maintenanceClientCustody, error) {
				events = append(events, "query-prepare")
				switch fault {
				case "query-prepare":
					return nil, sentinel
				case "query-partial":
					return query, sentinel
				case "query-nil":
					return nil, nil
				}
				return query, nil
			}, func(context.Context) (maintenanceClientCustody, error) {
				events = append(events, "folder-prepare")
				switch fault {
				case "folder-prepare":
					return nil, sentinel
				case "folder-partial":
					return folder, sentinel
				case "folder-nil":
					return nil, nil
				}
				return folder, nil
			}, func(_ context.Context, actual brokerpipe.MaintenanceRequest) (brokerpipe.MaintenanceResult, error) {
				calls++
				events = append(events, "call")
				if actual != q {
					t.Fatal("request changed")
				}
				if fault == "cancel-call" {
					cancel()
				}
				result := maintenanceClientTestResult(q)
				if fault == "mismatch" {
					result.Generation = "private"
				}
				return result, failures["call"]
			})
			expected := []string{"query-prepare", "folder-prepare", "query-before", "folder-before", "call", "query-after", "folder-after", "folder-close", "query-close"}
			switch fault {
			case "query-prepare", "query-nil":
				expected = []string{"query-prepare"}
			case "query-partial":
				expected = []string{"query-prepare", "query-close"}
			case "folder-prepare", "folder-nil":
				expected = []string{"query-prepare", "folder-prepare", "query-close"}
			case "folder-partial":
				expected = []string{"query-prepare", "folder-prepare", "folder-close", "query-close"}
			case "query-before", "folder-before":
				expected = []string{"query-prepare", "folder-prepare", "query-before", "folder-before", "folder-close", "query-close"}
			}
			if !reflect.DeepEqual(events, expected) {
				t.Fatalf("events=%q want=%q", events, expected)
			}
			if calls > 1 {
				t.Fatal("retried")
			}
			if fault == "" {
				if err != nil || result != maintenanceClientTestResult(q) || calls != 1 {
					t.Fatalf("result=%+v err=%v calls=%d", result, err, calls)
				}
			} else {
				if err == nil || result != (brokerpipe.MaintenanceResult{}) {
					t.Fatalf("uncertainty returned result=%+v err=%v", result, err)
				}
				if !strings.HasPrefix(fault, "cancel-") && fault != "mismatch" && fault != "query-nil" && fault != "folder-nil" && !errors.Is(err, sentinel) {
					t.Fatalf("lost retained error: %v", err)
				}
			}
		})
	}
}
func TestMaintenanceClientJoinsIndependentCleanupErrors(t *testing.T) {
	var events []string
	queryErr, folderErr, callErr := errors.New("query close"), errors.New("folder close"), errors.New("transport")
	failures := map[string]error{"query-close": queryErr, "folder-close": folderErr}
	query := &maintenanceClientFixtureOwner{name: "query", events: &events, failures: failures}
	folder := &maintenanceClientFixtureOwner{name: "folder", events: &events, failures: failures}
	result, err := callMaintenanceWithCustody(context.Background(), maintenanceClientTestRequest(), func(context.Context) (maintenanceClientCustody, error) { return query, nil }, func(context.Context) (maintenanceClientCustody, error) { return folder, nil }, func(_ context.Context, q brokerpipe.MaintenanceRequest) (brokerpipe.MaintenanceResult, error) {
		return maintenanceClientTestResult(q), callErr
	})
	if result != (brokerpipe.MaintenanceResult{}) || !errors.Is(err, queryErr) || !errors.Is(err, folderErr) || !errors.Is(err, callErr) {
		t.Fatalf("result=%+v err=%v", result, err)
	}
}
func TestMaintenanceClientPreflightHasNoGrantOrTransport(t *testing.T) {
	prepare := func(context.Context) (maintenanceClientCustody, error) {
		t.Fatal("invalid request prepared grant")
		return nil, nil
	}
	call := func(context.Context, brokerpipe.MaintenanceRequest) (brokerpipe.MaintenanceResult, error) {
		t.Fatal("invalid request transmitted")
		return brokerpipe.MaintenanceResult{}, nil
	}
	mutations := []func(*brokerpipe.MaintenanceRequest){func(q *brokerpipe.MaintenanceRequest) { q.Schema = 2 }, func(q *brokerpipe.MaintenanceRequest) { q.Operation = "quiesce-maintenance" }, func(q *brokerpipe.MaintenanceRequest) { q.RequestID = strings.Repeat("A", 32) }, func(q *brokerpipe.MaintenanceRequest) { q.ExpectedSelectedHeadSHA256 = "private" }, func(q *brokerpipe.MaintenanceRequest) { q.TargetGeneration = "private" }, func(q *brokerpipe.MaintenanceRequest) { q.TargetCompletionSHA256 = "private" }}
	for _, mutate := range mutations {
		q := maintenanceClientTestRequest()
		mutate(&q)
		r, e := callMaintenanceWithCustody(context.Background(), q, prepare, prepare, call)
		if e == nil || r != (brokerpipe.MaintenanceResult{}) {
			t.Fatalf("result=%+v err=%v", r, e)
		}
	}
	canceled, cancel := context.WithCancel(context.Background())
	cancel()
	for _, ctx := range []context.Context{nil, canceled} {
		if _, e := callMaintenanceWithCustody(ctx, maintenanceClientTestRequest(), prepare, prepare, call); e == nil {
			t.Fatal("non-live context accepted")
		}
	}
	if _, e := callMaintenanceWithCustody(context.Background(), maintenanceClientTestRequest(), nil, prepare, call); e == nil {
		t.Fatal("nil query preparation accepted")
	}
	if _, e := callMaintenanceWithCustody(context.Background(), maintenanceClientTestRequest(), prepare, nil, call); e == nil {
		t.Fatal("nil folder preparation accepted")
	}
	if _, e := callMaintenanceWithCustody(context.Background(), maintenanceClientTestRequest(), prepare, prepare, nil); e == nil {
		t.Fatal("nil transport accepted")
	}
}

func maintenanceClientHistoryFixture(page int) (brokerpipe.MaintenanceRequest, brokerpipe.MaintenanceResult) {
	q := brokerpipe.MaintenanceRequest{Schema: 1, Operation: brokerpipe.SelectedHistory, RequestID: strings.Repeat("1", 32), Page: &page}
	if page > 0 {
		q.ExpectedSelectedHeadSHA256, q.InventorySHA256 = strings.Repeat("2", 64), strings.Repeat("6", 64)
	}
	ancestors := []brokerpipe.MaintenanceAncestor{{Generation: strings.Repeat("7", 32), CompletionSHA256: strings.Repeat("8", 64)}}
	r := brokerpipe.MaintenanceResult{Schema: 1, RequestID: q.RequestID, State: brokerpipe.SelectedHistory,
		Generation: strings.Repeat("3", 32), CompletionSHA256: strings.Repeat("4", 64), SelectedHeadSHA256: strings.Repeat("2", 64),
		Version: "1.0.0", Commit: strings.Repeat("a", 40), QualificationSHA256: strings.Repeat("b", 64), QualificationClosedSHA256: strings.Repeat("c", 64),
		InventorySHA256: strings.Repeat("6", 64), Page: &page, Ancestors: &ancestors}
	return q, r
}

func TestSelectedHistoryUsesOriginalNativeCustodyAndNeverRetries(t *testing.T) {
	for _, page := range []int{0, 1} {
		for _, fault := range []string{"", "transport", "wrong-head", "wrong-inventory", "missing-qualification", "query-after", "folder-close", "query-close"} {
			t.Run(string(rune('0'+page))+"/"+fault, func(t *testing.T) {
				q, expected := maintenanceClientHistoryFixture(page)
				var events []string
				failures := map[string]error{fault: errors.New("uncertain original owner")}
				query := &maintenanceClientFixtureOwner{name: "query", events: &events, failures: failures}
				folder := &maintenanceClientFixtureOwner{name: "folder", events: &events, failures: failures}
				calls := 0
				r, err := callMaintenanceWithCustody(context.Background(), q,
					func(context.Context) (maintenanceClientCustody, error) { return query, nil },
					func(context.Context) (maintenanceClientCustody, error) { return folder, nil },
					func(_ context.Context, actual brokerpipe.MaintenanceRequest) (brokerpipe.MaintenanceResult, error) {
						calls++
						if actual != q {
							t.Fatal("history request changed")
						}
						result := expected
						switch fault {
						case "wrong-head":
							result.SelectedHeadSHA256 = "unbound"
							if page > 0 {
								result.SelectedHeadSHA256 = strings.Repeat("f", 64)
							}
						case "wrong-inventory":
							result.InventorySHA256 = "unbound"
							if page > 0 {
								result.InventorySHA256 = strings.Repeat("f", 64)
							}
						case "missing-qualification":
							result.QualificationClosedSHA256 = ""
						}
						return result, failures["transport"]
					})
				if calls != 1 || !reflect.DeepEqual(events, []string{"query-before", "folder-before", "query-after", "folder-after", "folder-close", "query-close"}) {
					t.Fatal("history bypassed custody or retried", calls, events)
				}
				if fault == "" {
					if err != nil || !reflect.DeepEqual(r, expected) {
						t.Fatal(r, err)
					}
				} else if err == nil || r != (brokerpipe.MaintenanceResult{}) {
					t.Fatal("unverified history escaped", r, err)
				}
			})
		}
	}
}

func TestSharedMaintenanceClientRefusesCrossOperationFieldsBeforeGrant(t *testing.T) {
	first, _ := maintenanceClientHistoryFixture(0)
	next, _ := maintenanceClientHistoryFixture(1)
	manual := maintenanceClientTestRequest()
	page := 0
	bad := []brokerpipe.MaintenanceRequest{first, first, first, next, next, manual, manual, manual, manual}
	bad[0].Page = nil
	bad[1].ExpectedSelectedHeadSHA256 = strings.Repeat("2", 64)
	bad[2].TargetGeneration = strings.Repeat("3", 32)
	bad[3].InventorySHA256 = ""
	bad[4].ExpectedSelectedHeadSHA256 = strings.Repeat("f", 63)
	bad[5].Page = &page
	bad[6].InventorySHA256 = strings.Repeat("6", 64)
	bad[7].Operation = brokerpipe.PrepareDecommission
	bad[8].Operation = brokerpipe.PrepareUninstall
	for _, q := range bad {
		prepare := func(context.Context) (maintenanceClientCustody, error) {
			t.Fatal("invalid request prepared grant", q)
			return nil, nil
		}
		call := func(context.Context, brokerpipe.MaintenanceRequest) (brokerpipe.MaintenanceResult, error) {
			t.Fatal("invalid request transmitted", q)
			return brokerpipe.MaintenanceResult{}, nil
		}
		r, err := callMaintenanceWithCustody(context.Background(), q, prepare, prepare, call)
		if err == nil || r != (brokerpipe.MaintenanceResult{}) {
			t.Fatal("cross-operation request accepted", q)
		}
	}
	q := maintenanceClientTestRequest()
	for _, mutate := range []func(*brokerpipe.MaintenanceResult){
		func(r *brokerpipe.MaintenanceResult) { r.Version = "1.0.0" },
		func(r *brokerpipe.MaintenanceResult) { r.Page = &page },
		func(r *brokerpipe.MaintenanceResult) { r.InventorySHA256 = strings.Repeat("6", 64) },
	} {
		r := maintenanceClientTestResult(q)
		mutate(&r)
		if maintenanceClientResult(r, q) {
			t.Fatal("manual rollback accepted history response fields", r)
		}
	}
}
