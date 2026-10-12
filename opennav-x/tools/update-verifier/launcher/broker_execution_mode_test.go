package launcher

import (
	"context"
	"strings"
	"testing"

	"example.com/opennav-update-verifier/brokerpipe"
)

func TestBrokerExecutionModeSoleCanonicalIntent(t *testing.T) {
	id := strings.Repeat("a", 32)
	for _, prefix := range []string{"--broker-start", "--broker-current"} {
		for _, mode := range []brokerpipe.StartupMode{brokerpipe.StartupXNav, brokerpipe.StartupLegacy, brokerpipe.StartupSafe} {
			intent, err := brokerModeIntent(id, mode)
			if err != nil {
				t.Fatal(err)
			}
			got, matched, err := brokerIntentArgument([]string{prefix + "=" + intent}, prefix)
			if err != nil || !matched || got != intent {
				t.Fatal(got, matched, err)
			}
			generation, parsed, err := brokerStartupMode(got)
			if err != nil || generation != id || parsed != mode {
				t.Fatal(generation, parsed, err)
			}
		}
	}
	for _, value := range []string{id + ":", id + ":xnav", id + ":Legacy", id + ":safe-mode", id + ":--legacy", id + ":legacy:safe", id + ":legacy ", id + ":C:/app", "../app:legacy"} {
		for _, prefix := range []string{"--broker-start", "--broker-current", "--broker-offer", "--broker-execute"} {
			if _, matched, err := brokerIntentArgument([]string{prefix + "=" + value}, prefix); !matched || err == nil {
				t.Fatalf("ambiguous mode accepted %s=%s", prefix, value)
			}
		}
	}
	for _, prefix := range []string{"--broker-start", "--broker-current"} {
		if _, matched, err := brokerIntentArgument([]string{prefix + "=" + id + ":legacy", "--safe-mode"}, prefix); !matched || err == nil {
			t.Fatal("extra process argument accepted")
		}
	}
	for _, prefix := range []string{"--broker-offer", "--broker-execute"} {
		if _, _, err := brokerIntentArgument([]string{prefix + "=" + id + ":legacy"}, prefix); err == nil {
			t.Fatal("offer/qualification mode accepted")
		}
	}
}
func TestBrokerExecutionModePreservesAdmissionQualificationAndCurrent(t *testing.T) {
	for _, mode := range []brokerpipe.StartupMode{brokerpipe.StartupLegacy, brokerpipe.StartupSafe} {
		for _, initial := range []bool{false, true} {
			a, b := brokerExecutionFixture()
			a.request.Operation = "start-selected"
			a.request.StartupMode = mode
			if initial {
				for _, phase := range []brokerpipe.ExecutionPhase{brokerpipe.SpawnCandidate, brokerpipe.CloseCandidate} {
					c := brokerExecutionCommand(a, phase)
					if _, err := a.handle(context.Background(), c); err != nil {
						t.Fatal("qualification must stay XNav", err)
					}
				}
			}
			c := brokerExecutionCommand(a, brokerpipe.RelaySelectedExecutor)
			c.StartupMode = mode
			if _, err := a.handle(context.Background(), c); err != nil {
				t.Fatal(err)
			}
			if b.commands[len(b.commands)-1].StartupMode != mode {
				t.Fatal("relay lost mode")
			}
			if _, err := a.handle(context.Background(), brokerExecutionCommand(a, brokerpipe.FinishExecution)); err != nil {
				t.Fatal(err)
			}
			a.close()
		}
	}
	for _, mode := range []brokerpipe.StartupMode{brokerpipe.StartupXNav, brokerpipe.StartupLegacy, brokerpipe.StartupSafe} {
		a, b := brokerExecutionFixture()
		a.request.Operation = "start-current"
		a.request.StartupMode = mode
		for _, phase := range []brokerpipe.ExecutionPhase{brokerpipe.SpawnCurrent, brokerpipe.ObserveCurrent, brokerpipe.CloseCurrent} {
			c := brokerExecutionCommand(a, phase)
			c.StartupMode = mode
			if _, err := a.handle(context.Background(), c); err != nil {
				t.Fatal(err)
			}
		}
		if len(b.commands) != 1 || b.commands[0].StartupMode != mode {
			t.Fatal("ordinary creator lost mode")
		}
		a.close()
	}
}
func TestBrokerExecutionModeSubstitutionNeverCreatesOrRetries(t *testing.T) {
	for _, phase := range []brokerpipe.ExecutionPhase{brokerpipe.SpawnCurrent, brokerpipe.RelaySelectedExecutor, brokerpipe.RelayStartupExecutor, brokerpipe.SpawnCandidate} {
		a, b := brokerExecutionFixture()
		a.request.Operation = "start-current"
		a.request.StartupMode = brokerpipe.StartupLegacy
		if phase != brokerpipe.SpawnCurrent {
			a.request.Operation = "start-selected"
		}
		c := brokerExecutionCommand(a, phase)
		if phase == brokerpipe.SpawnCandidate || phase == brokerpipe.RelayStartupExecutor {
			c.StartupMode = brokerpipe.StartupLegacy
		}
		if _, err := a.handle(context.Background(), c); err == nil || len(b.commands) != 0 {
			t.Fatalf("mode substitution created %s", phase)
		}
		c.StartupMode = brokerpipe.StartupLegacy
		if _, err := a.handle(context.Background(), c); err == nil || len(b.commands) != 0 {
			t.Fatal("failed mode retried")
		}
		a.close()
	}
	a, b := brokerExecutionFixture()
	a.request.Operation = "start-current"
	a.request.StartupMode = brokerpipe.StartupLegacy
	c := brokerExecutionCommand(a, brokerpipe.SpawnCurrent)
	c.StartupMode = brokerpipe.StartupLegacy
	if _, err := a.handle(context.Background(), c); err != nil {
		t.Fatal(err)
	}
	c.Phase = brokerpipe.ObserveCurrent
	c.StartupMode = brokerpipe.StartupSafe
	if _, err := a.handle(context.Background(), c); err == nil || len(b.commands) != 1 {
		t.Fatal("observe switched original mode")
	}
	a.close()
}
func TestBrokerExecutionModeRunnerRequestAndQualificationRestriction(t *testing.T) {
	for _, mode := range []brokerpipe.StartupMode{brokerpipe.StartupLegacy, brokerpipe.StartupSafe} {
		_, backend := brokerExecutionFixture()
		calls := 0
		err := runBrokerExecutionModeIntent(context.Background(), strings.Repeat("a", 32), "start-selected", mode, backend, func(_ context.Context, q brokerpipe.ExecutionRequest, _ brokerpipe.ExecutionClientHandler) error {
			calls++
			if q.StartupMode != mode || q.Operation != "start-selected" {
				t.Fatal("lost request mode")
			}
			return nil
		})
		if err != nil || calls != 1 {
			t.Fatal(err, calls)
		}
		err = runBrokerExecutionModeIntent(context.Background(), strings.Repeat("a", 32), "execute-session", mode, backend, func(context.Context, brokerpipe.ExecutionRequest, brokerpipe.ExecutionClientHandler) error {
			t.Fatal("mode reached qualification request")
			return nil
		})
		if err == nil {
			t.Fatal("non-XNav qualification accepted")
		}
	}
}

func TestBrokerExecutionRecoveryModeCannotEnterOfferPrompt(t *testing.T) {
	for _, mode := range []brokerpipe.StartupMode{brokerpipe.StartupLegacy, brokerpipe.StartupSafe} {
		c := brokerpipe.ExecutionCommand{Phase: brokerpipe.OfferUpdate, ApplicationGeneration: strings.Repeat("a", 32), OfferVersion: "1.2.3", OfferCommit: strings.Repeat("b", 40), OfferPolicySHA256: strings.Repeat("c", 64), PromptBytes: 1, PromptSHA256: strings.Repeat("d", 64)}
		if !validBrokerOffer(c, c.ApplicationGeneration) {
			t.Fatal("fixture not otherwise valid")
		}
		c.StartupMode = mode
		if validBrokerOffer(c, c.ApplicationGeneration) {
			t.Fatal("recovery mode reached update prompt")
		}
	}
}
