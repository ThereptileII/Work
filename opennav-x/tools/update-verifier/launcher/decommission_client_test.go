package launcher

import (
	"context"
	"errors"
	"example.com/opennav-update-verifier/brokerpipe"
	"strings"
	"testing"
)

type decommissionClientFixture struct {
	name   string
	events *[]string
	fault  string
}

func (f *decommissionClientFixture) Validate(context.Context) error {
	*f.events = append(*f.events, f.name+".validate")
	if f.fault == f.name+".validate" {
		return errors.New(f.fault)
	}
	return nil
}
func (f *decommissionClientFixture) Close() error {
	*f.events = append(*f.events, f.name+".close")
	if f.fault == f.name+".close" {
		return errors.New(f.fault)
	}
	return nil
}
func TestDecommissionClientRestoresTemporaryGrantBeforeQueryCleanup(t *testing.T) {
	q := brokerpipe.MaintenanceRequest{Schema: 1, Operation: brokerpipe.PrepareDecommission, RequestID: strings.Repeat("1", 32), ExpectedSelectedHeadSHA256: strings.Repeat("2", 64)}
	for _, operation := range []string{brokerpipe.PrepareDecommission, brokerpipe.PrepareUninstall} {
		q.Operation = operation
		for _, fault := range []string{"", "query.prepare", "folder.prepare", "query.validate", "folder.validate", "folder.close", "query.close"} {
			t.Run(operation+"/"+fault, func(t *testing.T) {
				var events []string
				prepare := func(name string) maintenanceClientPrepare {
					return func(context.Context) (maintenanceClientCustody, error) {
						events = append(events, name+".prepare")
						owner := &decommissionClientFixture{name, &events, fault}
						if fault == name+".prepare" {
							return owner, errors.New(fault)
						}
						return owner, nil
					}
				}
				calls := 0
				witness, err := callDecommissionWithCustody(context.Background(), q, prepare("query"), prepare("folder"), func(context.Context, brokerpipe.MaintenanceRequest) (*brokerpipe.DecommissionWitness, error) {
					calls++
					events = append(events, "call")
					return nil, errors.New("lost response may mean fenced")
				})
				if err == nil || witness != nil || calls > 1 {
					t.Fatal("uncertain action accepted/repeated")
				}
				joined := strings.Join(events, ",")
				if !strings.HasSuffix(joined, "query.close") {
					t.Fatalf("query owner not drained: %s", joined)
				}
				if strings.Contains(joined, "folder.prepare") && !strings.Contains(joined, "folder.close,query.close") {
					t.Fatalf("wrong restore order: %s", joined)
				}
				if strings.Contains(fault, "prepare") || strings.Contains(fault, "validate") {
					if calls != 0 {
						t.Fatal("called after preparation refusal")
					}
				} else if calls != 1 {
					t.Fatal("did not perform exactly one call")
				}
			})
		}
	}

}
