package launcher

import (
	"context"
	"errors"
	"io"
	"strings"
	"testing"

	"example.com/opennav-update-verifier/brokerpipe"
)

func TestBrokerStartupAdmissionSoleCanonicalPrivateArgument(t *testing.T) {
	id := strings.Repeat("a", 32)
	for _, args := range [][]string{{"--broker-start"}, {"--broker-start="}, {"--broker-start=" + strings.Repeat("A", 32)}, {"--broker-start=../app"}, {"--broker-start=" + id, "--xnav"}, {"--broker-start=" + id, "--broker-start=" + id}, {"--broker-start=" + id, "--broker-current=" + id}, {"--broker-start-more=" + id}} {
		if _, matched, err := brokerStartupArgument(args); !matched || err == nil {
			t.Fatal("malformed startup admitted", args, err)
		}
		if err := Run(context.Background(), args); err == nil {
			t.Fatal("malformed startup fell into legacy run", args)
		}
	}
	if got, matched, err := brokerStartupArgument([]string{"--broker-start=" + id}); got != id || !matched || err != nil {
		t.Fatal(got, matched, err)
	}
	for _, ordinary := range []string{"--xnav", "--legacy", "--safe-mode", "--initialize-trust"} {
		if _, matched, err := brokerStartupArgument([]string{ordinary}); matched || err != nil {
			t.Fatal("ordinary mode captured", ordinary, err)
		}
	}
}

func TestBrokerStartupAdmissionAllowsOnlyQualificationOrTerminalRelay(t *testing.T) {
	for _, qualify := range []bool{false, true} {
		name := "selected-relay"
		if qualify {
			name = "initial-qualification-relay"
		}
		t.Run(name, func(t *testing.T) {
			a, b := brokerExecutionFixture()
			a.request.Operation = "start-selected"
			defer a.close()
			if qualify {
				for _, phase := range []brokerpipe.ExecutionPhase{brokerpipe.SpawnCandidate, brokerpipe.CloseCandidate} {
					if _, err := a.handle(context.Background(), brokerExecutionCommand(a, phase)); err != nil {
						t.Fatal(err)
					}
				}
			}
			if _, err := a.handle(context.Background(), brokerExecutionCommand(a, brokerpipe.RelaySelectedExecutor)); err != nil {
				t.Fatal(err)
			}
			if a.current.consumed || !a.relay.consumed || a.candidate.consumed != qualify {
				t.Fatal("startup role mixed")
			}
			if _, err := a.handle(context.Background(), brokerExecutionCommand(a, brokerpipe.FinishExecution)); err != nil {
				t.Fatal(err)
			}
			relay := b.children[len(b.commands)-1]
			if relay.exited || relay.requests != 0 || relay.closes != 1 {
				t.Fatal("terminal finish killed relay or leaked handles")
			}
		})
	}
	for _, phase := range []brokerpipe.ExecutionPhase{brokerpipe.SpawnCurrent, brokerpipe.ObserveCurrent, brokerpipe.CloseCurrent} {
		t.Run("deny-"+string(phase), func(t *testing.T) {
			a, b := brokerExecutionFixture()
			a.request.Operation = "start-selected"
			defer a.close()
			if _, err := a.handle(context.Background(), brokerExecutionCommand(a, phase)); err == nil || len(b.commands) != 0 {
				t.Fatal("admission created ordinary current", phase, err)
			}
		})
	}
}

func TestBrokerStartupAdmissionCannotSwitchOrRepeatAfterEffects(t *testing.T) {
	for _, phase := range []brokerpipe.ExecutionPhase{brokerpipe.SpawnCandidate, brokerpipe.SpawnFallback, brokerpipe.SpawnCurrent, brokerpipe.RelaySelectedExecutor} {
		t.Run("after-relay-"+string(phase), func(t *testing.T) {
			a, b := brokerExecutionFixture()
			a.request.Operation = "start-selected"
			defer a.close()
			if _, err := a.handle(context.Background(), brokerExecutionCommand(a, brokerpipe.RelaySelectedExecutor)); err != nil {
				t.Fatal(err)
			}
			if _, err := a.handle(context.Background(), brokerExecutionCommand(a, phase)); err == nil || len(b.commands) != 1 {
				t.Fatal("terminal relay reused", phase, err)
			}
		})
	}
	t.Run("live-qualification-cannot-relay", func(t *testing.T) {
		a, b := brokerExecutionFixture()
		a.request.Operation = "start-selected"
		defer a.close()
		if _, err := a.handle(context.Background(), brokerExecutionCommand(a, brokerpipe.SpawnCandidate)); err != nil {
			t.Fatal(err)
		}
		if _, err := a.handle(context.Background(), brokerExecutionCommand(a, brokerpipe.RelaySelectedExecutor)); err == nil || len(b.commands) != 1 {
			t.Fatal("relay preceded candidate teardown", err)
		}
	})
	t.Run("failed-birth-cannot-retry-or-switch", func(t *testing.T) {
		a, b := brokerExecutionFixture()
		a.request.Operation = "start-selected"
		defer a.close()
		b.spawnErr = io.ErrUnexpectedEOF
		if _, err := a.handle(context.Background(), brokerExecutionCommand(a, brokerpipe.SpawnCandidate)); !errors.Is(err, io.ErrUnexpectedEOF) {
			t.Fatal(err)
		}
		b.spawnErr = nil
		for _, phase := range []brokerpipe.ExecutionPhase{brokerpipe.SpawnCandidate, brokerpipe.SpawnCurrent, brokerpipe.RelaySelectedExecutor} {
			if _, err := a.handle(context.Background(), brokerExecutionCommand(a, phase)); err == nil || len(b.commands) != 1 {
				t.Fatal("failed birth retried or reinterpreted", phase, err)
			}
		}
	})
}

func TestBrokerStartupAdmissionPreservesFreshIntentAndTransportFailure(t *testing.T) {
	id := strings.Repeat("a", 32)
	_, b := brokerExecutionFixture()
	calls := 0
	err := runBrokerExecutionIntent(context.Background(), id, "start-selected", b, func(_ context.Context, q brokerpipe.ExecutionRequest, _ brokerpipe.ExecutionClientHandler) error {
		calls++
		if q.Operation != "start-selected" || q.StageID != id || !generationPattern.MatchString(q.RequestID) || q.RequestID == id {
			t.Fatal("lost exact fresh admission intent", q)
		}
		return io.ErrUnexpectedEOF
	})
	if !errors.Is(err, io.ErrUnexpectedEOF) || calls != 1 || len(b.commands) != 0 {
		t.Fatal("transport failure retried or entered ordinary startup", err, calls)
	}
}
