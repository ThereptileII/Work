package brokerhost

import (
	"context"
	"errors"
	"example.com/opennav-update-verifier/brokerpipe"
	"example.com/opennav-update-verifier/brokerstore"
	"strings"
	"testing"
)

type decommissionReceiptFixture struct {
	events *[]string
	fail   string
}

func (f decommissionReceiptFixture) event(name string) error {
	*f.events = append(*f.events, name)
	if f.fail == name {
		return errors.New(name)
	}
	return nil
}
func (f decommissionReceiptFixture) Validate(context.Context) error { return f.event("receipt") }
func (f decommissionReceiptFixture) CloseExecutionJournalReader(context.Context, *brokerstore.ExecutionJournalReader) error {
	return f.event("reader")
}
func (f decommissionReceiptFixture) EvidenceSHA256(context.Context) (string, error) {
	return strings.Repeat("a", 64), f.event("evidence")
}
func TestDecommissionPreparationFaultsPreserveTerminalUncertainty(t *testing.T) {
	for _, fault := range []string{"", "reader", "receipt", "original", "evidence", "cancel"} {
		t.Run(fault, func(t *testing.T) {
			var events []string
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			receipt := decommissionReceiptFixture{&events, fault}
			standalone := func(context.Context) error {
				events = append(events, "original")
				if fault == "cancel" {
					cancel()
				}
				if fault == "original" {
					return errors.New("original changed")
				}
				return nil
			}
			digest, err := finishDecommissionPreparation(ctx, receipt, []*brokerstore.ExecutionJournalReader{{}}, standalone)
			if fault == "" {
				if err != nil || len(digest) != 64 || strings.Join(events, ",") != "reader,receipt,original,evidence" {
					t.Fatalf("%s %v %v", digest, err, events)
				}
			} else {
				if !errors.Is(err, brokerstore.ErrDecommissionUncertain) || digest != "" {
					t.Fatalf("success escaped fault %s: %s %v", fault, digest, err)
				}
				if fault != "evidence" && strings.Contains(strings.Join(events, ","), "evidence") {
					t.Fatal("accepted completion after failure")
				}
			}
		})
	}
}
func TestDecommissionRequestClosedForm(t *testing.T) {
	q := brokerpipe.MaintenanceRequest{Schema: 1, Operation: brokerpipe.PrepareDecommission, RequestID: strings.Repeat("a", 32), ExpectedSelectedHeadSHA256: strings.Repeat("b", 64)}
	if !validDecommissionRequest(q) {
		t.Fatal("exact request refused")
	}
	for _, mutation := range []func(*brokerpipe.MaintenanceRequest){func(q *brokerpipe.MaintenanceRequest) { q.TargetGeneration = strings.Repeat("c", 32) }, func(q *brokerpipe.MaintenanceRequest) { p := 0; q.Page = &p }, func(q *brokerpipe.MaintenanceRequest) { q.Operation = brokerpipe.ManualRollback }, func(q *brokerpipe.MaintenanceRequest) { q.ExpectedSelectedHeadSHA256 = "" }} {
		bad := q
		mutation(&bad)
		if validDecommissionRequest(bad) {
			t.Fatal("expanded request accepted")
		}
	}
}

type retirementReceiptFixture struct {
	events *[]string
	fault  string
}

func (r retirementReceiptFixture) event(name string) error {
	*r.events = append(*r.events, name)
	if r.fault == name {
		return errors.New(name)
	}
	return nil
}
func (r retirementReceiptFixture) Validate(context.Context) error { return r.event("retired") }
func (r retirementReceiptFixture) FenceSHA256(context.Context) (string, error) {
	return strings.Repeat("a", 64), r.event("fence")
}
func (r retirementReceiptFixture) EvidenceSHA256(context.Context) (string, error) {
	return strings.Repeat("b", 64), r.event("retirement")
}
func TestProtectedUninstallHandoffAndCompletionFaults(t *testing.T) {
	for _, fault := range []string{"", "release", "cancel-release", "commit", "nil-receipt", "retired", "standalone", "fence", "retirement"} {
		t.Run(fault, func(t *testing.T) {
			var events []string
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			fence, retired, err := commitPreparedRetirement(ctx, func() error {
				events = append(events, "release")
				if fault == "cancel-release" {
					cancel()
				}
				if fault == "release" {
					return errors.New("selected close uncertain")
				}
				return nil
			}, func(context.Context) (retirementReceipt, error) {
				events = append(events, "commit")
				if fault == "commit" {
					return nil, errors.New("partial retirement")
				}
				if fault == "nil-receipt" {
					return nil, nil
				}
				return retirementReceiptFixture{&events, fault}, nil
			}, func(context.Context) error {
				events = append(events, "standalone")
				if fault == "standalone" {
					return errors.New("profile changed")
				}
				return nil
			})
			if fault == "" {
				if err != nil || len(fence) != 64 || len(retired) != 64 || strings.Join(events, ",") != "release,commit,retired,standalone,fence,retirement,retired,standalone" {
					t.Fatalf("%v %v", events, err)
				}
			} else {
				if !errors.Is(err, brokerstore.ErrDecommissionUncertain) || fence != "" || retired != "" {
					t.Fatalf("retirement success escaped %s: %v", fault, err)
				}
				if (fault == "release" || fault == "cancel-release") && strings.Contains(strings.Join(events, ","), "commit") {
					t.Fatal("mutation after failed original reader release")
				}
			}
		})
	}
}

func TestProtectedUninstallStartupDrainGatesFinalWitness(t *testing.T) {
	for _, fault := range []string{"", "release", "missing-drain", "duplicate-drain", "invalid-fence", "publish", "cancel-publish", "post-drain-guard", "commit", "different-fence", "retired"} {
		t.Run(fault, func(t *testing.T) {
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			var events []string
			published, guardCalls := 0, 0
			fence, retired, err := commitPreparedStartupRetirement(ctx, func() error {
				events = append(events, "release")
				if fault == "release" {
					return errors.New("uncertain release")
				}
				return nil
			}, func(ctx context.Context, drain func(context.Context, string) error) (retirementReceipt, error) {
				events = append(events, "commit")
				if fault != "missing-drain" {
					digest := strings.Repeat("a", 64)
					if fault == "invalid-fence" {
						digest = "invalid"
					}
					if fault == "different-fence" {
						digest = strings.Repeat("c", 64)
					}
					// Deliberately ignore a callback refusal: the host must not turn
					// a later receipt into authority after a failed/duplicate drain.
					_ = drain(ctx, digest)
					if fault == "duplicate-drain" {
						_ = drain(ctx, digest)
					}
				}
				if fault == "commit" {
					return nil, errors.New("rename failed")
				}
				return retirementReceiptFixture{&events, fault}, nil
			}, func(context.Context) error {
				guardCalls++
				events = append(events, "guard")
				if fault == "post-drain-guard" && guardCalls == 2 {
					return errors.New("original owner changed")
				}
				return nil
			}, func(context.Context, string) error {
				published++
				events = append(events, "publish-drain")
				if fault == "publish" {
					return errors.New("operator did not drain")
				}
				if fault == "cancel-publish" {
					cancel()
				}
				return nil
			})
			if fault == "" {
				if err != nil || len(fence) != 64 || len(retired) != 64 || published != 1 || strings.Join(events, ",") != "release,commit,guard,publish-drain,guard,retired,guard,fence,retirement,retired,guard" {
					t.Fatalf("complete transaction: %v %v", events, err)
				}
			} else if !errors.Is(err, brokerstore.ErrDecommissionUncertain) || fence != "" || retired != "" {
				t.Fatalf("final witness escaped %s: %v", fault, err)
			}
			if published > 1 {
				t.Fatal("operator was asked to mutate twice")
			}
			if (fault == "release" || fault == "missing-drain" || fault == "invalid-fence") && published != 0 {
				t.Fatal("publication without original preparation")
			}
			if (fault == "publish" || fault == "duplicate-drain" || fault == "cancel-publish" || fault == "post-drain-guard" || fault == "missing-drain") && strings.Contains(strings.Join(events, ","), "retirement") {
				t.Fatal("accepted receipt after uncertain drainage")
			}
		})
	}
}
