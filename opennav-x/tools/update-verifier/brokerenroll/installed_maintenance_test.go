package brokerenroll

import (
	"context"
	"errors"
	"strings"
	"testing"

	"example.com/opennav-update-verifier/brokerregistration"
)

type installedMaintenanceCompletionFixture struct {
	checks, closes  int
	fail, closeFail bool
}

func (c *installedMaintenanceCompletionFixture) check(ctx context.Context) error {
	c.checks++
	if c.fail {
		return errors.New("original completion custody changed")
	}
	return ctx.Err()
}
func (c *installedMaintenanceCompletionFixture) close() error {
	c.closes++
	if c.closeFail {
		return errors.New("completion close uncertain")
	}
	return nil
}

func TestInstalledMaintenanceDecommissionCompletionRetainedAcrossServiceLifetime(t *testing.T) {
	f := newMaintenanceFixture(t)
	c := &installedMaintenanceCompletionFixture{}
	client, err := brokerregistration.ParseRetainedApprovedRequest(f.req.ClientRegistrationRequest, f.req.ApprovedClientRequestSHA256)
	if err != nil {
		t.Fatal(err)
	}
	m, err := openMaintenanceAuthorityWith(context.Background(), maintenanceAuthority{brokerSHA: strings.Repeat("a", 64), client: client}, f.op, false, c)
	if err != nil {
		t.Fatal(err)
	}
	if c.checks == 0 || c.closes != 0 {
		t.Fatal("completion was not retained")
	}
	if err := m.Check(context.Background()); err != nil {
		t.Fatal(err)
	}
	c.fail = true
	if err := m.Check(context.Background()); err == nil {
		t.Fatal("lost completion custody accepted")
	}
	c.fail = false
	if err := m.Quiesce(context.Background()); err == nil || f.stops != 0 {
		t.Fatal("revoked completion reached STOP", err, f.stops)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil || c.closes != 1 {
		t.Fatal("completion owner not drained once", err, c.closes)
	}
}

func TestInstalledMaintenanceDecommissionCompletionFailureClosesBeforeSCM(t *testing.T) {
	for _, fault := range []string{"completion", "service", "close"} {
		t.Run(fault, func(t *testing.T) {
			f := newMaintenanceFixture(t)
			c := &installedMaintenanceCompletionFixture{fail: fault == "completion", closeFail: fault == "close"}
			if fault != "completion" {
				f.fault = "open"
			}
			client, err := brokerregistration.ParseRetainedApprovedRequest(f.req.ClientRegistrationRequest, f.req.ApprovedClientRequestSHA256)
			if err != nil {
				t.Fatal(err)
			}
			m, err := openMaintenanceAuthorityWith(context.Background(), maintenanceAuthority{brokerSHA: strings.Repeat("a", 64), client: client}, f.op, false, c)
			if err == nil || m != nil || c.closes != 1 || f.stops != 0 {
				t.Fatal("partial completion leaked or reached SCM", err, c.closes, f.stops)
			}
			if fault == "completion" && len(f.events) != 0 {
				t.Fatal("completion failed after service open", f.events)
			}
			if fault == "close" && !errors.Is(err, ErrCleanup) {
				t.Fatal("close uncertainty lost", err)
			}
		})
	}
}

func TestInstalledMaintenanceDecommissionMissingOriginalNeverFallsBack(t *testing.T) {
	c := &installedMaintenanceCompletionFixture{}
	f := newMaintenanceFixture(t)
	called := false
	m, err := openInstalledMaintenanceWith(context.Background(), func(context.Context) (installedMaintenanceInputs, error) {
		return installedMaintenanceInputs{completion: c}, nil
	}, f.op, func(context.Context, *MaintenanceOwner, ApprovedClientBaseline) error { called = true; return nil })
	if err == nil || m != nil || called || c.closes != 1 || len(f.events) != 0 {
		t.Fatal("incomplete installation was adopted", err, called, c.closes, f.events)
	}
}

func TestInstalledMaintenanceDecommissionCancellationAtOwnershipTransferClosesOriginal(t *testing.T) {
	f := newMaintenanceFixture(t)
	c := &installedMaintenanceCompletionFixture{}
	ctx, cancel := context.WithCancel(context.Background())
	cancel()
	m, err := openMaintenanceAuthorityWith(ctx, maintenanceAuthority{}, f.op, true, c)
	if err == nil || m != nil || c.closes != 1 || len(f.events) != 0 {
		t.Fatal("canceled transfer leaked or opened SCM", err, c.closes, f.events)
	}
}
