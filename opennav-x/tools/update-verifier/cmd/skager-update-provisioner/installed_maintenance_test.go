package main

import (
	"bytes"
	"context"
	"errors"
	"strings"
	"testing"

	"example.com/opennav-update-verifier/brokerenroll"
	"example.com/opennav-update-verifier/brokerpipe"
)

type currentInstalledFixture struct {
	fault                                  string
	checks, drains, binds, removes, closes int
}

func (f *currentInstalledFixture) Check(context.Context) error {
	f.checks++
	if f.fault == "check" {
		return errors.New("lost native custody")
	}
	return nil
}
func (f *currentInstalledFixture) RestoreStartupAndDrain(context.Context, *brokerpipe.UninstallFenceWitness) error {
	f.drains++
	if f.fault == "drain" {
		return errors.New("original drain refused")
	}
	return nil
}
func (f *currentInstalledFixture) BindDecommission(context.Context, *brokerpipe.DecommissionWitness) error {
	f.binds++
	if f.fault == "bind" {
		return errors.New("changed original peer")
	}
	return nil
}
func (f *currentInstalledFixture) DecommissionInstalled(context.Context) (brokerenroll.InstalledUninstallResult, error) {
	f.removes++
	if f.fault == "remove" {
		return brokerenroll.InstalledUninstallResult{}, errors.New("partial removal")
	}
	r := brokerenroll.InstalledUninstallResult{Service: brokerenroll.ServiceMarkedForRemoval, Integration: "verified-owned-members-removed", Registration: "unregistered", Recovery: "preserved"}
	if f.fault == "result" {
		r.Recovery = ""
	}
	return r, nil
}
func (f *currentInstalledFixture) Close() error {
	f.closes++
	if f.fault == "close" {
		return errors.New("close uncertain")
	}
	return nil
}

func currentInstalledTestOps(t *testing.T, f *currentInstalledFixture) (currentInstalledOperations, *int) {
	t.Helper()
	calls := 0
	ids := 0
	return currentInstalledOperations{
		open: func(context.Context) (currentInstalledOwner, error) {
			calls++
			if f.fault == "open" {
				return f, errors.New("partial native open")
			}
			return f, nil
		},
		newID: func() (string, error) {
			ids++
			if f.fault == "random" {
				return "", errors.New("random failed")
			}
			return strings.Repeat(string(rune('1'+ids)), 32), nil
		},
		history: func(_ context.Context, q brokerpipe.MaintenanceRequest) (brokerpipe.MaintenanceResult, error) {
			calls++
			r := selectedDiagnosticResponse(q)
			if f.fault == "history" {
				r.SelectedHeadSHA256 = ""
			}
			return r, nil
		},
		uninstall: func(ctx context.Context, q brokerpipe.MaintenanceRequest, drain func(context.Context, *brokerpipe.UninstallFenceWitness) error) (*brokerpipe.DecommissionWitness, error) {
			calls++
			if q.Operation != brokerpipe.PrepareUninstall || q.ExpectedSelectedHeadSHA256 != strings.Repeat("6", 64) || q.RequestID != strings.Repeat("3", 32) {
				t.Fatal("customer lost exact observed head", q)
			}
			if err := drain(ctx, nil); err != nil {
				return nil, err
			}
			if f.fault == "witness" {
				return nil, errors.New("retirement unverified")
			}
			return &brokerpipe.DecommissionWitness{}, nil
		},
		closeWitness: func(*brokerpipe.DecommissionWitness) error {
			if f.fault == "witness-close" {
				return errors.New("witness close uncertain")
			}
			return nil
		},
	}, &calls
}

func TestInstalledMaintenanceDecommissionCLINoCallerAuthorityAndSameOwner(t *testing.T) {
	f := &currentInstalledFixture{}
	op, calls := currentInstalledTestOps(t, f)
	var out bytes.Buffer
	if err := runCurrentInstalledWith(context.Background(), []string{"--maintenance=uninstall-current"}, &out, op); err != nil || *calls != 3 || f.drains != 1 || f.binds != 1 || f.removes != 1 || !bytes.Contains(out.Bytes(), []byte(`"state":"verified-installed-integration-uninstalled"`)) {
		t.Fatal(err, *calls, f, out.String())
	}
	for _, args := range [][]string{{"--maintenance=uninstall-current", "--client-sid=other"}, {"--maintenance=uninstall-current", "--broker-sha256=" + strings.Repeat("a", 64)}, {"--maintenance=uninstall-current", "--installed-baseline-artifact=C:/other"}, {"--maintenance=uninstall-current", "--maintenance=uninstall-current"}, {"--maintenance=uninstall-installed"}} {
		out.Reset()
		before := *calls
		if err := runCurrentInstalledWith(context.Background(), args, &out, op); err == nil || *calls != before || out.Len() != 0 {
			t.Fatal("caller gained authority", args, err, *calls)
		}
	}
}

func TestInstalledMaintenanceDecommissionCLIFaultsSuppressCompletionWithoutRetry(t *testing.T) {
	for _, fault := range []string{"open", "check", "random", "history", "drain", "witness", "bind", "remove", "result", "close", "witness-close"} {
		t.Run(fault, func(t *testing.T) {
			f := &currentInstalledFixture{fault: fault}
			op, calls := currentInstalledTestOps(t, f)
			var out bytes.Buffer
			err := runCurrentInstalledWith(context.Background(), []string{"--maintenance=uninstall-current"}, &out, op)
			if err == nil || out.Len() != 0 || *calls > 3 || f.drains > 1 || f.binds > 1 || f.removes > 1 || f.closes == 0 {
				t.Fatal("failure claimed completion or retried", err, *calls, f, out.String())
			}
			if (fault == "open" || fault == "check" || fault == "random" || fault == "history" || fault == "drain" || fault == "witness" || fault == "bind") && f.removes != 0 {
				t.Fatal("removal after prerequisite refusal", f)
			}
		})
	}
}
