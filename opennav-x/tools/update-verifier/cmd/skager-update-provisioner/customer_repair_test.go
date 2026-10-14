package main

import (
	"bytes"
	"context"
	"errors"
	"fmt"
	"os"
	"path/filepath"
	"strings"
	"testing"

	"example.com/opennav-update-verifier/brokerpipe"
)

func TestCustomerRepairOneMutationAndStrictFreshReadback(t *testing.T) {
	for _, fault := range []string{"", "decline", "old-broker", "lost-parent", "ambiguous-mutation", "changed-version", "same-generation", "readback-health", "readback-head", "close"} {
		t.Run(fault, func(t *testing.T) {
			owner := &rollbackFixtureOwner{closeError: fault == "close"}
			out := &bytes.Buffer{}
			ids, mutations, queries := 0, 0, 0
			newGeneration := ""
			version, commit := "1.0.1", strings.Repeat("a", 40)
			op := customerRollbackOperations{open: func(context.Context) (customerRollbackOwner, error) { return owner, nil }, newID: func() (string, error) { ids++; return fmt.Sprintf("%032x", ids), nil }, confirm: func(_ context.Context, p brokerpipe.MaintenanceResult) (bool, error) {
				if p.Version != version || p.Commit != commit {
					t.Fatal("wrong customer confirmation")
				}
				if fault == "lost-parent" {
					owner.failure = true
				}
				return fault != "decline", nil
			}, call: func(_ context.Context, q brokerpipe.MaintenanceRequest) (brokerpipe.MaintenanceResult, error) {
				queries++
				r := brokerpipe.MaintenanceResult{Schema: 1, RequestID: q.RequestID, Generation: strings.Repeat("b", 32), CompletionSHA256: strings.Repeat("c", 64), SelectedHeadSHA256: strings.Repeat("d", 64), Version: version, Commit: commit}
				switch q.Operation {
				case brokerpipe.RepairInspect:
					r.State = brokerpipe.RepairInspect
					if fault == "old-broker" {
						return brokerpipe.MaintenanceResult{}, errors.New("unknown operation")
					}
				case brokerpipe.RepairCurrent:
					mutations++
					if q.ExpectedSelectedHeadSHA256 != strings.Repeat("d", 64) || q.TargetGeneration != "" || q.TargetCompletionSHA256 != "" {
						t.Fatal("not fixed broker-owned repair")
					}
					if fault == "ambiguous-mutation" {
						return brokerpipe.MaintenanceResult{}, errors.New("committed reply lost")
					}
					r.State = "repair-selected"
					newGeneration = q.RequestID
					r.Generation = newGeneration
					r.CompletionSHA256 = strings.Repeat("f", 64)
					r.SelectedHeadSHA256 = strings.Repeat("0", 64)
					if fault == "changed-version" {
						r.Version = "9.0.0"
					}
					if fault == "same-generation" {
						r.Generation = strings.Repeat("b", 32)
					}
				case brokerpipe.SelectedHistory:
					if mutations != 1 {
						t.Fatal("readback without mutation")
					}
					r.State = brokerpipe.SelectedHistory
					r.Generation = newGeneration
					r.CompletionSHA256 = strings.Repeat("f", 64)
					r.SelectedHeadSHA256 = strings.Repeat("0", 64)
					r.InventorySHA256 = strings.Repeat("1", 64)
					page := 0
					items := []brokerpipe.MaintenanceAncestor{}
					r.Page = &page
					r.Ancestors = &items
					r.RepairSourceQualification = &brokerpipe.RepairQualification{Generation: strings.Repeat("b", 32), CompletionSHA256: strings.Repeat("c", 64), QualifiedHeadSHA256: strings.Repeat("d", 64), HealthSHA256: strings.Repeat("2", 64), ClosedSHA256: strings.Repeat("3", 64)}
					if fault == "readback-health" {
						r.RepairSourceQualification = nil
						r.QualificationSHA256 = strings.Repeat("2", 64)
						r.QualificationClosedSHA256 = strings.Repeat("3", 64)
					}
					if fault == "readback-head" {
						r.SelectedHeadSHA256 = strings.Repeat("9", 64)
					}
				default:
					t.Fatal("unexpected operation", q.Operation)
				}
				return r, nil
			}}
			err := runCustomerRepairWith(context.Background(), out, op)
			if mutations > 1 || !owner.closed {
				t.Fatal("retry or leaked original owner")
			}
			success := fault == "" || fault == "decline"
			if success != (err == nil) {
				t.Fatalf("completion mismatch: %v", err)
			}
			if !success && out.Len() != 0 {
				t.Fatal("failure reported completion")
			}
			if fault == "ambiguous-mutation" && (queries != 2 || mutations != 1) {
				t.Fatal("ambiguous mutation retried/readback as success")
			}
			if fault == "old-broker" && mutations != 0 {
				t.Fatal("old broker mutation attempted")
			}
		})
	}
}
func TestCustomerRepairFixedSameAccountElevationAndNSISJoin(t *testing.T) {
	for _, args := range [][]string{{"--maintenance=repair-current"}, {"--maintenance", "repair-current"}, {"--maintenance=repair-current-elevated", "--elevation-parent-pid=17", "--elevation-parent-created=23"}} {
		q, handled, err := parseInstalledElevation(args)
		if !handled || err != nil || q.action != repairCurrent {
			t.Fatal("fixed repair elevation missing", err)
		}
	}
	for _, arg := range []string{"--source=C:\\payload", "--client-sid=S-1-5-21-1-2-3-1001", "--target-generation=" + strings.Repeat("a", 32)} {
		if _, handled, err := parseInstalledElevation([]string{"--maintenance=repair-current", arg}); !handled || err == nil {
			t.Fatal("caller authority accepted")
		}
	}
	raw, err := os.ReadFile(filepath.Join("..", "..", "..", "..", "installer", "windows", "AlphaSetup.nsi"))
	if err != nil {
		t.Fatal(err)
	}
	s := string(raw)
	start := strings.Index(s, `Section "Uninstall"`)
	s = s[start:]
	a := strings.Index(s, `${If} $Action == "Repair"`)
	b := strings.Index(s, `legacy_maintenance:`)
	if a < 0 || b < a || !strings.Contains(s[a:b], "--maintenance=repair-current") || !strings.Contains(s[a:b], "protected_uninstall_missing legacy_maintenance") {
		t.Fatal("Maintain Repair bypassed fixed protected helper")
	}
}
