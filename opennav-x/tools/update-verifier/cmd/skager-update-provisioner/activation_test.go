package main

import (
	"context"
	"errors"
	"strings"
	"testing"

	"example.com/opennav-update-verifier/brokerenroll"
)

func activationArgs() []string {
	return append(operatorArgs(), "--client-registration-request", "approved-client.json", "--client-registration-request-sha256", strings.Repeat("d", 64), "--activate-startup=true")
}

func TestExplicitActivationDoesNotRetryStoppedEnrollment(t *testing.T) {
	fault := errors.New("activation incomplete")
	stopped, active := 0, 0
	err := runWithActivation(context.Background(), activationArgs(), func(path string) ([]byte, error) { return []byte(path), nil },
		func(context.Context, brokerenroll.Request) error { stopped++; return nil },
		func(_ context.Context, r brokerenroll.Request) error {
			active++
			if string(r.ClientRegistrationRequest) != "approved-client.json" || r.ApprovedClientRequestSHA256 != strings.Repeat("d", 64) {
				t.Fatal("activation lost exact separate approval")
			}
			return fault
		})
	if !errors.Is(err, fault) || stopped != 0 || active != 1 {
		t.Fatalf("activation error=%v stopped=%d active=%d", err, stopped, active)
	}
}

func TestActivationRequiresExplicitApprovedClientBeforeEffects(t *testing.T) {
	for _, args := range [][]string{
		append(operatorArgs(), "--activate-startup=true"),
		append(activationArgs(), "--activate-startup=true"),
		append(activationArgs()[:len(activationArgs())-1], "--activate-startup=false"),
		append(activationArgs()[:len(activationArgs())-1], "--activate-startup=TRUE"),
	} {
		calls := 0
		err := runWithActivation(context.Background(), args, func(string) ([]byte, error) { calls++; return nil, nil },
			func(context.Context, brokerenroll.Request) error { calls++; return nil },
			func(context.Context, brokerenroll.Request) error { calls++; return nil })
		if err == nil || calls != 0 {
			t.Fatalf("ambiguous/unapproved activation had effects: %v %d", err, calls)
		}
	}
}

func TestDefaultOperatorEnrollmentRemainsStoppedOnly(t *testing.T) {
	stopped, active := 0, 0
	err := runWithActivation(context.Background(), operatorArgs(), func(path string) ([]byte, error) { return []byte(path), nil },
		func(context.Context, brokerenroll.Request) error { stopped++; return nil },
		func(context.Context, brokerenroll.Request) error { active++; return nil })
	if err != nil || stopped != 1 || active != 0 {
		t.Fatalf("default changed: %v stopped=%d active=%d", err, stopped, active)
	}
}
