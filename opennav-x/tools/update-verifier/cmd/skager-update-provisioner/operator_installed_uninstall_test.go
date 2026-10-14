package main

import (
	"bytes"
	"context"
	"encoding/json"
	"reflect"
	"testing"

	"example.com/opennav-update-verifier/brokerenroll"
	"example.com/opennav-update-verifier/brokerpipe"
)

func TestOperatorInstalledUninstallDecommissionStaysInOneOwner(t *testing.T) {
	f := &startupDecommissionFixture{t: t, installed: true}
	var output bytes.Buffer
	err := f.run([]byte("request"), []byte("original publication"), &output)
	want := []string{"read-request", "read-publication", "parse", "open", "check", "prepare-startup", "prepare-installed", "fence", "restore-and-drain", "retirement", "bind-fence", "bind-retirement", "decommission-installed", "close-owner", "close-witness"}
	var result map[string]any
	if err != nil || !reflect.DeepEqual(f.events, want) || json.Unmarshal(output.Bytes(), &result) != nil || result["state"] != "verified-installed-integration-uninstalled" || result["registration"] != "unregistered" || result["recovery"] != "preserved" {
		t.Fatal("installed owner sequence/result differs", f.events, output.String(), err)
	}
}

func TestOperatorInstalledUninstallDecommissionFaultsCannotDowngrade(t *testing.T) {
	for _, fault := range []string{"prepare-installed", "fence", "restore-and-drain", "retirement", "bind-fence", "bind-retirement", "decommission-installed", "partial-installed-result", "close-owner", "close-witness", "cancel-close"} {
		t.Run(fault, func(t *testing.T) {
			f := &startupDecommissionFixture{t: t, installed: true, fault: fault}
			var output bytes.Buffer
			if err := f.run([]byte("request"), []byte("original publication"), &output); err == nil || output.Len() != 0 || !f.closed {
				t.Fatal("failed installed operation reported success or leaked owner", f.events, err)
			}
			counts := map[string]int{}
			for _, event := range f.events {
				counts[event]++
				if event == "decommission" || counts[event] > 1 {
					t.Fatal("installed operation downgraded/retried", f.events)
				}
			}
			if fault == "prepare-installed" && counts["fence"] != 0 {
				t.Fatal("failed preflight still fenced original service", f.events)
			}
		})
	}
}

func TestOperatorInstalledUninstallDecommissionArtifactIsRequiredAndActionBound(t *testing.T) {
	raw, publication := []byte("request"), []byte("publication")
	base := decommissionPublicationArguments(raw, publication)
	for _, tc := range []struct {
		action, artifact string
		present          bool
	}{
		{uninstallInstalled, "", false}, {uninstallInstalled, "", true},
		{uninstallProtected, "original-baseline-artifact", true}, {decommissionService, "original-baseline-artifact", true},
	} {
		args := append([]string{}, base...)
		args[0] = "--maintenance=" + tc.action
		if tc.present {
			args = append(args, "--installed-baseline-artifact="+tc.artifact)
		}
		var output bytes.Buffer
		calls := 0
		err := runDecommission(context.Background(), args, func(string) ([]byte, error) { calls++; return raw, nil },
			func(context.Context, brokerenroll.MaintenanceRequest) (decommissionOwner, error) {
				calls++
				return nil, nil
			},
			func(context.Context, brokerpipe.MaintenanceRequest) (*brokerpipe.DecommissionWitness, error) {
				calls++
				return nil, nil
			}, &output)
		if err == nil || calls != 0 || output.Len() != 0 {
			t.Fatal("wrong-scope/missing installed input was consumed", tc, calls, err)
		}
	}
}
