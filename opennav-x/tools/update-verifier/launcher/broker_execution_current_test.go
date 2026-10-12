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

func TestBrokerExecutionCurrentNormalFlowLeavesApplicationAlive(t *testing.T) {
	a, backend := brokerExecutionFixture()
	a.request.Operation = "start-current"
	defer a.close()
	for _, phase := range []brokerpipe.ExecutionPhase{brokerpipe.SpawnCurrent, brokerpipe.ObserveCurrent, brokerpipe.FinishExecution} {
		report, err := a.handle(context.Background(), brokerExecutionCommand(a, phase))
		if err != nil || report.Status != "reported-complete" {
			t.Fatal(phase, report, err)
		}
		if phase != brokerpipe.FinishExecution && (report.ProcessID != backend.children[0].info.pid || report.ProcessCreation != backend.children[0].info.created) {
			t.Fatal("lost original creator observation")
		}
	}
	if len(backend.commands) != 1 || backend.commands[0].ApplicationGeneration == a.request.StageID || backend.commands[0].HealthPipe != "" || backend.commands[0].HealthChallenge != "" {
		t.Fatal("normal directive acquired qualification or caller-stage authority")
	}
	child := backend.children[0]
	if child.exited || child.requests != 0 || !child.handlesClosed || child.closes != 1 {
		t.Fatal("normal finish signaled application or leaked handles")
	}
}
func TestBrokerExecutionCurrentAndQualificationCannotMixEvenAfterClose(t *testing.T) {
	for _, qualification := range []brokerpipe.ExecutionPhase{brokerpipe.SpawnCandidate, brokerpipe.SpawnFallback} {
		for _, normalFirst := range []bool{true, false} {
			for _, closeFirst := range []bool{false, true} {
				name := string(qualification)
				if normalFirst {
					name += "/normal-first"
				} else {
					name += "/qualification-first"
				}
				if closeFirst {
					name += "/closed"
				} else {
					name += "/live"
				}
				t.Run(name, func(t *testing.T) {
					a, backend := brokerExecutionFixture()
					a.request.Operation = "start-current"
					defer a.close()
					first, next, closePhase := brokerpipe.SpawnCurrent, qualification, brokerpipe.CloseCurrent
					if !normalFirst {
						a.request.Operation = "execute-session"
						first, next, closePhase = qualification, brokerpipe.SpawnCurrent, brokerpipe.CloseCandidate
						if qualification == brokerpipe.SpawnFallback {
							closePhase = brokerpipe.CloseFallback
						}
					}
					if _, err := a.handle(context.Background(), brokerExecutionCommand(a, first)); err != nil {
						t.Fatal(err)
					}
					if closeFirst {
						if _, err := a.handle(context.Background(), brokerExecutionCommand(a, closePhase)); err != nil {
							t.Fatal(err)
						}
					}
					if _, err := a.handle(context.Background(), brokerExecutionCommand(a, next)); err == nil {
						t.Fatal("normal and qualification combined")
					}
					if len(backend.commands) != 1 {
						t.Fatal("mixed second native creator")
					}
				})
			}
		}
	}
}
func TestBrokerExecutionCurrentCloseConsumesRoleAndRequiresOriginalBinding(t *testing.T) {
	for _, mode := range []string{"repeated-spawn", "repeated-observe", "wrong-observe-generation", "wrong-close-generation", "repeated-close"} {
		t.Run(mode, func(t *testing.T) {
			a, backend := brokerExecutionFixture()
			a.request.Operation = "start-current"
			defer a.close()
			if _, err := a.handle(context.Background(), brokerExecutionCommand(a, brokerpipe.SpawnCurrent)); err != nil {
				t.Fatal(err)
			}
			next := brokerExecutionCommand(a, brokerpipe.SpawnCurrent)
			switch mode {
			case "repeated-spawn", "repeated-close":
				if _, err := a.handle(context.Background(), brokerExecutionCommand(a, brokerpipe.CloseCurrent)); err != nil {
					t.Fatal(err)
				}
				if backend.children[0].requests != 1 || !backend.children[0].exited || backend.children[0].closes != 1 {
					t.Fatal("owned normal close not confirmed")
				}
				if mode == "repeated-close" {
					next = brokerExecutionCommand(a, brokerpipe.CloseCurrent)
				}
			case "repeated-observe":
				if _, err := a.handle(context.Background(), brokerExecutionCommand(a, brokerpipe.ObserveCurrent)); err != nil {
					t.Fatal(err)
				}
				next = brokerExecutionCommand(a, brokerpipe.ObserveCurrent)
			case "wrong-observe-generation":
				next = brokerExecutionCommand(a, brokerpipe.ObserveCurrent)
				next.ApplicationGeneration = strings.Repeat("9", 32)
			case "wrong-close-generation":
				next = brokerExecutionCommand(a, brokerpipe.CloseCurrent)
				next.ApplicationGeneration = strings.Repeat("9", 32)
			}
			observes, requests := backend.children[0].observes, backend.children[0].requests
			if _, err := a.handle(context.Background(), next); err == nil {
				t.Fatal("replayed or unrelated normal role accepted")
			}
			if len(backend.commands) != 1 || backend.children[0].observes != observes || backend.children[0].requests != requests {
				t.Fatal("refused role performed effects")
			}
		})
	}
}
func TestBrokerExecutionCurrentRefusesQualificationOrUnsafePayloadBeforeCreator(t *testing.T) {
	for _, mode := range []string{"no-generation", "uppercase", "path", "health-pipe", "health-challenge", "both-health"} {
		t.Run(mode, func(t *testing.T) {
			a, backend := brokerExecutionFixture()
			a.request.Operation = "start-current"
			defer a.close()
			command := brokerExecutionCommand(a, brokerpipe.SpawnCurrent)
			switch mode {
			case "no-generation":
				command.ApplicationGeneration = ""
			case "uppercase":
				command.ApplicationGeneration = strings.Repeat("A", 32)
			case "path":
				command.ApplicationGeneration = "../generations/current"
			case "health-pipe":
				command.HealthPipe = "Skager.Update." + strings.Repeat("6", 32)
			case "health-challenge":
				command.HealthChallenge = strings.Repeat("7", 64)
			case "both-health":
				command.HealthPipe = "Skager.Update." + strings.Repeat("6", 32)
				command.HealthChallenge = strings.Repeat("7", 64)
			}
			if _, err := a.handle(context.Background(), command); err == nil {
				t.Fatal("unsafe normal launch accepted")
			}
			if len(backend.commands) != 0 || a.current.consumed {
				t.Fatal("malformed normal payload reached creator")
			}
		})
	}
}
func TestBrokerExecutionCurrentConsumesFailuresAndLostConversationNeverFallback(t *testing.T) {
	for _, mode := range []string{"create", "observe", "close"} {
		t.Run(mode, func(t *testing.T) {
			a, backend := brokerExecutionFixture()
			a.request.Operation = "start-current"
			defer a.close()
			fault := errors.New(mode)
			backend.onSpawn = func() {
				if !a.current.consumed {
					t.Fatal("normal attempt not consumed before native entry")
				}
			}
			if mode == "create" {
				backend.spawnErr = fault
			}
			if mode == "observe" {
				backend.children[0].observeErr = fault
			}
			_, firstErr := a.handle(context.Background(), brokerExecutionCommand(a, brokerpipe.SpawnCurrent))
			if mode == "close" {
				if firstErr != nil {
					t.Fatal(firstErr)
				}
				backend.children[0].requestCloseErr = fault
				_, firstErr = a.handle(context.Background(), brokerExecutionCommand(a, brokerpipe.CloseCurrent))
			}
			if !errors.Is(firstErr, errBrokerExecutionUncertain) || !errors.Is(firstErr, fault) {
				t.Fatal("lost normal failure", firstErr)
			}
			for _, phase := range []brokerpipe.ExecutionPhase{brokerpipe.SpawnCurrent, brokerpipe.SpawnCandidate, brokerpipe.SpawnFallback} {
				if _, err := a.handle(context.Background(), brokerExecutionCommand(a, phase)); err == nil {
					t.Fatal("retry/fallback after normal uncertainty")
				}
			}
			if err := a.close(); err != nil {
				t.Fatal(err)
			}
			if len(backend.commands) != 1 || !backend.children[0].handlesClosed || backend.children[0].exited {
				t.Fatal("normal cleanup retried/signaled")
			}
		})
	}
	_, backend := brokerExecutionFixture()
	err := runBrokerExecutionIntent(context.Background(), strings.Repeat("2", 32), "start-current", backend, func(ctx context.Context, q brokerpipe.ExecutionRequest, h brokerpipe.ExecutionClientHandler) error {
		a := &brokerExecutionActions{request: q}
		if _, err := h(ctx, brokerExecutionCommand(a, brokerpipe.SpawnCurrent)); err != nil {
			t.Fatal(err)
		}
		return io.EOF
	})
	if !errors.Is(err, io.EOF) || len(backend.commands) != 1 || backend.children[0].exited || !backend.children[0].handlesClosed {
		t.Fatal("lost normal conversation entered fallback", err)
	}
}
func TestBrokerExecutionCurrentConcurrentSpawnCreatesOnce(t *testing.T) {
	a, backend := brokerExecutionFixture()
	a.request.Operation = "start-current"
	defer a.close()
	var group sync.WaitGroup
	for i := 0; i < 8; i++ {
		group.Add(1)
		go func() {
			defer group.Done()
			_, _ = a.handle(context.Background(), brokerExecutionCommand(a, brokerpipe.SpawnCurrent))
		}()
	}
	group.Wait()
	if len(backend.commands) != 1 {
		t.Fatal("concurrent normal spawn repeated native creator")
	}
}
