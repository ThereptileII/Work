package main

import (
	"bytes"
	"context"
	"crypto/sha256"
	"encoding/hex"
	"errors"
	"strings"
	"testing"

	"example.com/opennav-update-verifier/brokerenroll"
	"example.com/opennav-update-verifier/brokerpipe"
)

func decommissionArguments(raw []byte) []string {
	h := sha256.Sum256(raw)
	return []string{"--maintenance=decommission-service", "--broker-sha256=" + strings.Repeat("a", 64),
		"--client-registration-request=original-private-request", "--client-registration-request-sha256=" + hex.EncodeToString(h[:]),
		"--request-id=" + strings.Repeat("1", 32), "--expected-selected-head-sha256=" + strings.Repeat("2", 64)}
}

type refusedDecommissionOwner struct {
	checks, binds, effects, closes int
	checkErr, closeErr             error
}

func (f *refusedDecommissionOwner) Check(context.Context) error { f.checks++; return f.checkErr }
func (f *refusedDecommissionOwner) Close() error                { f.closes++; return f.closeErr }
func (f *refusedDecommissionOwner) BindDecommission(context.Context, *brokerpipe.DecommissionWitness) error {
	f.binds++
	return errors.New("private invalid original witness")
}
func (f *refusedDecommissionOwner) Decommission(context.Context) (brokerenroll.DecommissionResult, error) {
	f.effects++
	return brokerenroll.DecommissionResult{}, errors.New("unexpected decommission")
}

func TestOperatorDecommissionInvalidInputsHaveNoEffects(t *testing.T) {
	raw := []byte("original request")
	base := decommissionArguments(raw)
	for _, args := range [][]string{nil, base[:5], append(append([]string{}, base...), base[0]),
		append(append([]string{}, base...), "--root=C:/private"),
		append(append([]string{}, base...), "--client-sid=S-1-5-18"),
		append(append([]string{}, base...), "--maintenance=manual-rollback"),
		append([]string{"--maintenance=uninstall"}, base[1:]...),
		append(base[:4:4], "--request-id=ABC", base[5]),
		append(base[:5:5], "--expected-selected-head-sha256=bad"),
		append(append([]string{}, base...), "positional")} {
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
		if err == nil || calls != 0 || output.Len() != 0 || strings.Contains(err.Error(), "C:/private") {
			t.Fatalf("invalid action had effects: calls=%d output=%q err=%v", calls, output.String(), err)
		}
	}
}

func TestOperatorDecommissionRequiresOriginalServiceBeforeFenceRequest(t *testing.T) {
	raw := []byte("original request")
	for _, fault := range []string{"changed-request", "open", "check", "transport", "missing-witness", "zero-witness"} {
		t.Run(fault, func(t *testing.T) {
			owner, calls := &refusedDecommissionOwner{}, 0
			var output bytes.Buffer
			if fault == "check" {
				owner.checkErr = errors.New("private original service changed")
			}
			err := runDecommission(context.Background(), decommissionArguments(raw), func(string) ([]byte, error) {
				if fault == "changed-request" {
					return []byte("different"), nil
				}
				return raw, nil
			}, func(_ context.Context, request brokerenroll.MaintenanceRequest) (decommissionOwner, error) {
				if !bytes.Equal(request.ClientRegistrationRequest, raw) {
					t.Fatal("request changed")
				}
				if fault == "open" {
					return owner, errors.New("private open refused")
				}
				return owner, nil
			}, func(_ context.Context, q brokerpipe.MaintenanceRequest) (*brokerpipe.DecommissionWitness, error) {
				calls++
				if owner.checks != 1 || q.Schema != 1 || q.Operation != brokerpipe.PrepareDecommission || q.RequestID != strings.Repeat("1", 32) || q.ExpectedSelectedHeadSHA256 != strings.Repeat("2", 64) {
					t.Fatal("missing original service check or wrong closed request")
				}
				if fault == "transport" {
					return nil, errors.New("private transport refused")
				}
				if fault == "zero-witness" {
					return &brokerpipe.DecommissionWitness{}, nil
				}
				return nil, nil
			}, &output)
			wantCalls := 0
			if fault == "transport" || fault == "missing-witness" || fault == "zero-witness" {
				wantCalls = 1
			}
			if err == nil || calls != wantCalls || owner.effects != 0 || output.Len() != 0 || strings.Contains(err.Error(), "private") {
				t.Fatalf("calls=%d effects=%d output=%q err=%v", calls, owner.effects, output.String(), err)
			}
			if fault != "changed-request" && owner.closes != 1 {
				t.Fatalf("original owner not closed: %d", owner.closes)
			}
		})
	}
}

func TestDecommissionDispatchSelectsOnlyExplicitAction(t *testing.T) {
	for _, args := range [][]string{{"--maintenance=decommission-service"}, {"--maintenance", "decommission-service"}} {
		if !decommissionRequested(args) {
			t.Fatal("explicit action ignored")
		}
	}
	for _, args := range [][]string{nil, {"--maintenance=uninstall"}, {"--maintenance=decommission-service-extra"}, {"--maintenance-other=decommission-service"}} {
		if decommissionRequested(args) {
			t.Fatal("unrelated action selected")
		}
	}
}
