package main

import (
	"bytes"
	"context"
	"errors"
	"fmt"
	"os"
	"path/filepath"
	"reflect"
	"strings"
	"testing"

	"example.com/opennav-update-verifier/brokerpipe"
)

type rollbackFixtureOwner struct {
	closed     bool
	failure    bool
	closeError bool
}

func (o *rollbackFixtureOwner) Check(ctx context.Context) error {
	if o.closed || o.failure {
		return errors.New("original installation owner lost")
	}
	return ctx.Err()
}
func (o *rollbackFixtureOwner) Close() error {
	o.closed = true
	if o.closeError {
		return errors.New("original owner drain uncertain")
	}
	return nil
}

func TestCustomerRollbackConfirmsOneOriginalTargetThenReadsBack(t *testing.T) {
	for _, fault := range []string{"", "decline", "old-broker", "no-target", "confirm-error", "cancel-confirm", "owner-after-confirm", "duplicate-id", "mutation-error", "mutation-mismatch", "readback-error", "readback-mismatch", "readback-version", "close-error", "confirmation-alias"} {
		t.Run(fault, func(t *testing.T) {
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			owner := &rollbackFixtureOwner{closeError: fault == "close-error"}
			target := brokerpipe.RollbackTarget{Generation: strings.Repeat("a", 32), CompletionSHA256: strings.Repeat("b", 64), Version: "1.0.0", Commit: strings.Repeat("c", 40)}
			var operations []string
			mutations, confirmed, ids := 0, false, 0
			output := &bytes.Buffer{}
			op := customerRollbackOperations{
				open: func(context.Context) (customerRollbackOwner, error) { return owner, nil },
				newID: func() (string, error) {
					ids++
					if fault == "duplicate-id" {
						return strings.Repeat("1", 32), nil
					}
					return fmt.Sprintf("%032x", ids), nil
				},
				confirm: func(_ context.Context, p brokerpipe.MaintenanceResult) (bool, error) {
					confirmed = true
					if p.Previous == nil || *p.Previous != target || p.Version != "1.0.1" {
						t.Fatal("wrong displayed release", p)
					}
					if fault == "confirmation-alias" {
						p.Previous.Generation = strings.Repeat("f", 32)
					}
					if fault == "cancel-confirm" {
						cancel()
					}
					if fault == "owner-after-confirm" {
						owner.failure = true
					}
					if fault == "confirm-error" {
						return false, errors.New("dialog failed")
					}
					return fault != "decline", nil
				},
				call: func(_ context.Context, q brokerpipe.MaintenanceRequest) (brokerpipe.MaintenanceResult, error) {
					operations = append(operations, q.Operation)
					switch q.Operation {
					case brokerpipe.PreviousQualified:
						if fault == "old-broker" {
							return brokerpipe.MaintenanceResult{}, brokerpipe.ErrRefused
						}
						r := brokerpipe.MaintenanceResult{Schema: 1, RequestID: q.RequestID, State: brokerpipe.PreviousQualified, Generation: strings.Repeat("d", 32), CompletionSHA256: strings.Repeat("e", 64), SelectedHeadSHA256: strings.Repeat("f", 64), Version: "1.0.1", Commit: strings.Repeat("d", 40), Previous: &brokerpipe.RollbackTarget{Generation: target.Generation, CompletionSHA256: target.CompletionSHA256, Version: target.Version, Commit: target.Commit}}
						if fault == "no-target" {
							r.Previous = nil
						}
						return r, nil
					case brokerpipe.ManualRollback:
						mutations++
						if !confirmed || q.ExpectedSelectedHeadSHA256 != strings.Repeat("f", 64) || q.TargetGeneration != target.Generation || q.TargetCompletionSHA256 != target.CompletionSHA256 {
							t.Fatal("caller/UI replaced original target", q)
						}
						if fault == "mutation-error" {
							return brokerpipe.MaintenanceResult{}, errors.New("committed reply lost")
						}
						r := brokerpipe.MaintenanceResult{Schema: 1, RequestID: q.RequestID, State: "manual-selected", Generation: target.Generation, CompletionSHA256: target.CompletionSHA256, SelectedHeadSHA256: strings.Repeat("1", 64)}
						if fault == "mutation-mismatch" {
							r.Generation = strings.Repeat("f", 32)
						}
						return r, nil
					case brokerpipe.SelectedHistory:
						if mutations != 1 {
							t.Fatal("readback before actual request")
						}
						if fault == "readback-error" {
							return brokerpipe.MaintenanceResult{}, errors.New("readback lost")
						}
						ancestors := []brokerpipe.MaintenanceAncestor{}
						page := 0
						r := brokerpipe.MaintenanceResult{Schema: 1, RequestID: q.RequestID, State: brokerpipe.SelectedHistory, Generation: target.Generation, CompletionSHA256: target.CompletionSHA256, SelectedHeadSHA256: strings.Repeat("1", 64), Version: target.Version, Commit: target.Commit, QualificationSHA256: strings.Repeat("2", 64), QualificationClosedSHA256: strings.Repeat("3", 64), InventorySHA256: strings.Repeat("4", 64), Page: &page, Ancestors: &ancestors}
						if fault == "readback-mismatch" {
							r.SelectedHeadSHA256 = strings.Repeat("5", 64)
						}
						if fault == "readback-version" {
							r.Version = "1.0.9"
						}
						return r, nil
					default:
						t.Fatal("unexpected operation", q.Operation)
						return brokerpipe.MaintenanceResult{}, nil
					}
				},
			}
			err := runCustomerRollbackWith(ctx, output, op)
			if !owner.closed || mutations > 1 {
				t.Fatal("lost original owner or retried mutation", mutations)
			}
			switch fault {
			case "", "confirmation-alias":
				if err != nil || !strings.Contains(output.String(), "verified-customer-rollback-selected") || !reflect.DeepEqual(operations, []string{brokerpipe.PreviousQualified, brokerpipe.ManualRollback, brokerpipe.SelectedHistory}) {
					t.Fatal(output.String(), operations, err)
				}
			case "decline":
				if err != nil || mutations != 0 || !strings.Contains(output.String(), "rollback-cancelled") {
					t.Fatal(output.String(), mutations, err)
				}
			default:
				if err == nil || output.Len() != 0 {
					t.Fatal("unverified success escaped", output.String(), err)
				}
			}
			if fault == "mutation-error" || fault == "mutation-mismatch" {
				if len(operations) != 2 {
					t.Fatal("ambiguous mutation caused another request", operations)
				}
			}
			if fault == "old-broker" || fault == "no-target" {
				if confirmed || mutations != 0 {
					t.Fatal("unsupported bootstrap reached confirmation or mutation")
				}
			}
		})
	}
}

func TestCustomerRollbackElevationFixedActionsAndOriginalOwners(t *testing.T) {
	for _, args := range [][]string{{"--maintenance=rollback-current"}, {"--maintenance", "rollback-current"}, {"--maintenance=rollback-current-elevated", "--elevation-parent-pid=16", "--elevation-parent-created=123450"}} {
		q, handled, err := parseInstalledElevation(args)
		if !handled || err != nil || q.action != rollbackCurrent {
			t.Fatal(q, handled, err)
		}
	}
	for _, args := range [][]string{{"--maintenance=rollback-current", "--target-generation=" + strings.Repeat("a", 32)}, {"--maintenance=rollback-current", "--sid=other"}, {"--maintenance=rollback-current-elevated", "--elevation-parent-pid=016", "--elevation-parent-created=123450"}} {
		if _, handled, err := parseInstalledElevation(args); !handled || err == nil {
			t.Fatal("caller authority accepted", args)
		}
	}
	op, self, _, child := elevationFixture(t, false)
	op.launch = func(_ context.Context, id installedElevationIdentity, args []string) (installedElevationChild, error) {
		want := []string{"--maintenance=rollback-current-elevated", "--elevation-parent-pid=17", "--elevation-parent-created=123456"}
		if !reflect.DeepEqual(args, want) || id.elevated {
			t.Fatal(args, id)
		}
		return child, nil
	}
	handled, err := installedElevationDispatchWith(context.Background(), []string{"--maintenance=rollback-current"}, func(context.Context) error { t.Fatal("parent entered privileged action"); return nil }, op)
	if !handled || err != nil || !child.waited || !self.closed || !child.closed {
		t.Fatal(handled, err)
	}
	op, self, parent, _ := elevationFixture(t, true)
	called := false
	handled, err = installedElevationDispatchWith(context.Background(), []string{"--maintenance=rollback-current-elevated", "--elevation-parent-pid=16", "--elevation-parent-created=123450"}, func(context.Context) error { called = true; return nil }, op)
	if !handled || err != nil || !called || !self.closed || !parent.closed {
		t.Fatal(handled, err)
	}
}

func TestCustomerRollbackNSISRoutesProtectedActionWithoutLegacyFallback(t *testing.T) {
	raw, err := os.ReadFile(filepath.Join("..", "..", "..", "..", "installer", "windows", "AlphaSetup.nsi"))
	if err != nil {
		t.Fatal(err)
	}
	source := string(raw)
	start := strings.Index(source, "protected_rollback:")
	if start < 0 {
		t.Fatal("protected customer rollback route missing")
	}
	end := strings.Index(source[start:], "  ${EndIf}")
	if end < 0 {
		t.Fatal("protected customer rollback route missing")
	}
	route := source[start : start+end]
	if !strings.Contains(route, "--maintenance=rollback-current") || !strings.Contains(route, "Goto maintenance_result") || strings.Contains(route, "Lifecycle.ps1") {
		t.Fatal("protected rollback fell into legacy engine", route)
	}
	if !strings.Contains(source, "protected_uninstall_missing legacy_maintenance") {
		t.Fatal("partial protected namespace lost fail-closed route")
	}
}
