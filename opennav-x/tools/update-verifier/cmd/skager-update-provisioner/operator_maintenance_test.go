package main

import (
	"context"
	"crypto/sha256"
	"encoding/hex"
	"errors"
	"reflect"
	"strings"
	"testing"

	"example.com/opennav-update-verifier/brokerenroll"
	"example.com/opennav-update-verifier/brokerregistration"
)

var maintenanceFault = errors.New("maintenance fixture failure")

type operatorMaintenanceFixture struct {
	events []string
	fault  string
	checks int
}

func (f *operatorMaintenanceFixture) Check(context.Context) error {
	f.checks++
	name := "before"
	if f.checks == 2 {
		name = "after"
	}
	f.events = append(f.events, name)
	if f.fault == name {
		return maintenanceFault
	}
	return nil
}
func (f *operatorMaintenanceFixture) Quiesce(context.Context) error {
	f.events = append(f.events, "stop")
	if f.fault == "stop" {
		return maintenanceFault
	}
	return nil
}
func (f *operatorMaintenanceFixture) Close() error {
	f.events = append(f.events, "close")
	if f.fault == "close" {
		return maintenanceFault
	}
	return nil
}

func maintenanceArgs(raw []byte) []string {
	s := sha256.Sum256(raw)
	return []string{"--maintenance=quiesce-maintenance", "--broker-sha256", strings.Repeat("a", 64),
		"--client-registration-request", "approved-client.json", "--client-registration-request-sha256", hex.EncodeToString(s[:])}
}

func TestOperatorMaintenanceCannotFallBackToEnrollmentOrRetry(t *testing.T) {
	raw := []byte("original approved request validated by native API")
	for _, fault := range []string{"", "open", "before", "stop", "after", "close"} {
		t.Run(fault, func(t *testing.T) {
			f := &operatorMaintenanceFixture{fault: fault}
			opens, enrolls := 0, 0
			maintenance, err := runOperator(context.Background(), maintenanceArgs(raw), func(string) ([]byte, error) { return raw, nil },
				func(context.Context, brokerenroll.Request) error { enrolls++; return nil },
				func(context.Context, brokerenroll.Request) error { enrolls++; return nil },
				func(_ context.Context, req brokerenroll.MaintenanceRequest) (maintenanceOwner, error) {
					opens++
					if req.ApprovedBrokerSHA256 != strings.Repeat("a", 64) || string(req.ClientRegistrationRequest) != string(raw) {
						t.Fatal("lost independent request/pin")
					}
					if fault == "open" {
						return f, maintenanceFault
					}
					return f, nil
				})
			want := []string{"before", "stop", "after", "close"}
			switch fault {
			case "open":
				want = []string{"close"}
			case "before":
				want = []string{"before", "close"}
			case "stop":
				want = []string{"before", "stop", "close"}
			}
			if !maintenance || opens != 1 || enrolls != 0 || !reflect.DeepEqual(f.events, want) || (fault == "" && err != nil) || (fault != "" && !errors.Is(err, maintenanceFault)) {
				t.Fatalf("maintenance=%v opens=%d enrolls=%d events=%v err=%v", maintenance, opens, enrolls, f.events, err)
			}
		})
	}
}

func TestMaintenanceAmbiguityNeverReadsOrStopsService(t *testing.T) {
	base := maintenanceArgs([]byte("request"))
	for _, args := range [][]string{
		nil, base[:len(base)-2], append(append([]string{}, base...), "--activate-startup=true"),
		append(append([]string{}, base...), "--maintenance=quiesce-maintenance"),
		append([]string{"--maintenance=uninstall"}, base[1:]...),
		append([]string{"--maintenance=quiesce-maintenance", "--broker-sha256", strings.Repeat("A", 64)}, base[3:]...),
		append(append([]string{}, base...), "unexpected"), {"--maintenance"},
	} {
		calls := 0
		err := runMaintenance(context.Background(), args, func(string) ([]byte, error) { calls++; return nil, nil }, func(context.Context, brokerenroll.MaintenanceRequest) (maintenanceOwner, error) {
			calls++
			return nil, nil
		})
		if err == nil || calls != 0 {
			t.Fatalf("invalid maintenance had effects: args=%v err=%v calls=%d", args, err, calls)
		}
	}
}

func TestMaintenanceMissingChangedOversizeInputNeverAcquiresService(t *testing.T) {
	for _, fault := range []string{"read", "empty", "changed", "oversize", "nil-owner"} {
		t.Run(fault, func(t *testing.T) {
			opens := 0
			raw := []byte("approved")
			err := runMaintenance(context.Background(), maintenanceArgs(raw), func(string) ([]byte, error) {
				switch fault {
				case "read":
					return nil, errors.New("private input path")
				case "empty":
					return nil, nil
				case "changed":
					return []byte("wrong"), nil
				case "oversize":
					return make([]byte, brokerregistration.MaxRequestBytes+1), nil
				}
				return raw, nil
			}, func(context.Context, brokerenroll.MaintenanceRequest) (maintenanceOwner, error) {
				opens++
				return nil, nil
			})
			want := 0
			if fault == "nil-owner" {
				want = 1
			}
			if err == nil || opens != want || strings.Contains(err.Error(), "private input path") {
				t.Fatalf("opens=%d err=%v", opens, err)
			}
		})
	}
}
