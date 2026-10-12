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
