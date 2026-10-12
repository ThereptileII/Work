package launcher

import (
	"context"
	"errors"
	"io"
	"strings"
	"sync"
	"testing"

	"example.com/opennav-update-verifier/brokerpipe"
)

func TestBrokerExecutionRelayFreshIntentAndFixedLookup(t *testing.T) {
	id := strings.Repeat("a", 32)
	for _, args := range [][]string{{"--broker-current"}, {"--broker-current="}, {"--broker-current=" + strings.Repeat("A", 32)}, {"--broker-current=../app"}, {"--broker-current=" + id, "--broker-execute=" + id}, {"--broker-current-more=" + id}} {
		if _, matched, err := brokerCurrentArgument(args); !matched || err == nil {
			t.Fatal("invalid explicit current route", args, err)
		}
		if err := Run(context.Background(), args); err == nil {
			t.Fatal("invalid current route fell through", args)
		}
	}
	if got, matched, err := brokerCurrentArgument([]string{"--broker-current=" + id}); got != id || !matched || err != nil {
		t.Fatal(got, matched, err)
	}
	if _, matched, err := brokerCurrentArgument([]string{"--legacy"}); matched || err != nil {
		t.Fatal("captured ordinary argument")
	}
	name, err := brokerExecutionLauncher(`C:/Program Files (x86)`, "S-1-5-21-1-2-3-500", id)
	if err != nil || name != `C:/Program Files (x86)/SKAGER/Applications/S-1-5-21-1-2-3-500/generations/`+id+`/app/skager-start.exe` {
		t.Fatal(name, err)
	}
	for _, bad := range []string{"", strings.ToUpper(id), "../app"} {
		if _, err = brokerExecutionLauncher(`C:/Program Files (x86)`, "S-1-5-21-1-2-3-500", bad); err == nil {
			t.Fatal("unsafe relay lookup")
		}
	}
	_, b := brokerExecutionFixture()
	err = runBrokerExecutionIntent(context.Background(), id, "start-current", b, func(_ context.Context, q brokerpipe.ExecutionRequest, _ brokerpipe.ExecutionClientHandler) error {
		if q.Operation != "start-current" || q.StageID != id || q.RequestID == id {
			t.Fatal("lost fresh explicit intent", q)
		}
		return io.EOF
	})
	if !errors.Is(err, io.EOF) || len(b.commands) != 0 {
		t.Fatal("transport failure retried or entered ordinary startup", err)
	}
}
func TestBrokerExecutionRelayAfterClosedQualificationIsTerminal(t *testing.T) {
	for _, operation := range []string{"execute-session", "start-current"} {
		t.Run(operation, func(t *testing.T) {
			a, b := brokerExecutionFixture()
			a.request.Operation = operation
			defer a.close()
			if operation == "execute-session" {
				for _, phase := range []brokerpipe.ExecutionPhase{brokerpipe.SpawnCandidate, brokerpipe.CloseCandidate} {
					if _, err := a.handle(context.Background(), brokerExecutionCommand(a, phase)); err != nil {
						t.Fatal(err)
					}
				}
			}
			if _, err := a.handle(context.Background(), brokerExecutionCommand(a, brokerpipe.RelaySelectedExecutor)); err != nil {
				t.Fatal(err)
			}
			if !a.relay.consumed || a.current.consumed {
				t.Fatal("relay reused application role")
			}
			child := b.children[len(b.commands)-1]
			if _, err := a.handle(context.Background(), brokerExecutionCommand(a, brokerpipe.FinishExecution)); err != nil {
				t.Fatal(err)
			}
			if child.exited || child.requests != 0 || child.closes != 1 {
				t.Fatal("finish terminated fresh relay or leaked creator")
			}
		})
	}
	for _, next := range []brokerpipe.ExecutionPhase{brokerpipe.RelaySelectedExecutor, brokerpipe.SpawnCandidate, brokerpipe.SpawnCurrent, brokerpipe.ObserveCurrent, brokerpipe.CloseCurrent} {
		t.Run("terminal-"+string(next), func(t *testing.T) {
			a, b := brokerExecutionFixture()
			defer a.close()
			if _, err := a.handle(context.Background(), brokerExecutionCommand(a, brokerpipe.RelaySelectedExecutor)); err != nil {
				t.Fatal(err)
			}
			if _, err := a.handle(context.Background(), brokerExecutionCommand(a, next)); err == nil || len(b.commands) != 1 {
				t.Fatal("relay conversation reused")
			}
		})
	}
}
func TestBrokerExecutionRelayRefusesLiveChildrenAndWrongIntent(t *testing.T) {
	for _, phase := range []brokerpipe.ExecutionPhase{brokerpipe.SpawnCandidate, brokerpipe.SpawnFallback, brokerpipe.SpawnCurrent} {
		t.Run(string(phase), func(t *testing.T) {
			a, b := brokerExecutionFixture()
			defer a.close()
			if phase == brokerpipe.SpawnCurrent {
				a.request.Operation = "start-current"
			}
			if _, err := a.handle(context.Background(), brokerExecutionCommand(a, phase)); err != nil {
				t.Fatal(err)
			}
			if _, err := a.handle(context.Background(), brokerExecutionCommand(a, brokerpipe.RelaySelectedExecutor)); err == nil || len(b.commands) != 1 {
				t.Fatal("live child overlapped relay")
			}
		})
	}
	for _, operation := range []string{"execute-session", "start-current"} {
		t.Run(operation, func(t *testing.T) {
			a, b := brokerExecutionFixture()
			defer a.close()
			a.request.Operation = operation
			phase := brokerpipe.SpawnCurrent
			if operation == "start-current" {
				phase = brokerpipe.SpawnCandidate
			}
			if _, err := a.handle(context.Background(), brokerExecutionCommand(a, phase)); err == nil || len(b.commands) != 0 {
				t.Fatal("wrong intent reached creator")
			}
		})
	}
}
func TestBrokerExecutionRelayFailureConsumedBeforeCreatorAndNoRetry(t *testing.T) {
	for _, mode := range []string{"creator", "observe", "cleanup", "cancel"} {
		t.Run(mode, func(t *testing.T) {
			a, b := brokerExecutionFixture()
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			b.onSpawn = func() {
				if !a.relay.consumed {
					t.Fatal("creator before consumption")
				}
				if mode == "cancel" {
					cancel()
				}
			}
			boom := errors.New(mode)
			switch mode {
			case "creator":
				b.spawnErr = boom
			case "observe":
				b.children[0].observeErr = boom
			case "cleanup":
				b.children[0].closeErr = boom
			}
			_, err := a.handle(ctx, brokerExecutionCommand(a, brokerpipe.RelaySelectedExecutor))
			if mode != "cleanup" && !errors.Is(err, errBrokerExecutionUncertain) {
				t.Fatal("failure not uncertain", err)
			}
			if _, err = a.handle(context.Background(), brokerExecutionCommand(a, brokerpipe.RelaySelectedExecutor)); err == nil {
				t.Fatal("retry after relay attempt")
			}
			closeErr := a.close()
			if mode == "cleanup" && !errors.Is(closeErr, errBrokerExecutionUncertain) {
				t.Fatal("cleanup uncertainty lost", closeErr)
			}
			if len(b.commands) != 1 || b.children[0].exited || b.children[0].requests != 0 || b.children[0].closes != 1 {
				t.Fatal("failure retried/killed/leaked")
			}
		})
	}
}
func TestBrokerExecutionRelayConcurrentConsumption(t *testing.T) {
	a, b := brokerExecutionFixture()
	defer a.close()
	var wg sync.WaitGroup
	for i := 0; i < 8; i++ {
		wg.Add(1)
		go func() {
			defer wg.Done()
			_, _ = a.handle(context.Background(), brokerExecutionCommand(a, brokerpipe.RelaySelectedExecutor))
		}()
	}
	wg.Wait()
	if len(b.commands) != 1 {
		t.Fatal("repeated relay creator")
	}
}
