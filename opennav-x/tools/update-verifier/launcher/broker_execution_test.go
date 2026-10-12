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

type memoryBrokerExecutionChild struct {
	info                                  brokerExecutionChildInfo
	observeErr, requestCloseErr, closeErr error
	observes, requests, closes            int
	exited, handlesClosed                 bool
}

func (c *memoryBrokerExecutionChild) observe(ctx context.Context) (brokerExecutionChildInfo, error) {
	c.observes++
	if c.handlesClosed || c.exited {
		return brokerExecutionChildInfo{}, errBrokerExecutionRefused
	}
	return c.info, errors.Join(ctx.Err(), c.observeErr)
}
func (c *memoryBrokerExecutionChild) closeApplication(ctx context.Context) (brokerExecutionChildInfo, error) {
	c.requests++
	if err := errors.Join(ctx.Err(), c.requestCloseErr); err != nil {
		return brokerExecutionChildInfo{}, err
	}
	c.exited = true
	return c.info, nil
}
func (c *memoryBrokerExecutionChild) close() error {
	if c.handlesClosed {
		return c.closeErr
	}
	c.handlesClosed = true
	c.closes++
	return c.closeErr
}

type memoryBrokerExecutionBackend struct {
	children []*memoryBrokerExecutionChild
	commands []brokerpipe.ExecutionCommand
	spawnErr error
	onSpawn  func()
}

func (b *memoryBrokerExecutionBackend) spawn(ctx context.Context, c brokerpipe.ExecutionCommand) (brokerExecutionChild, error) {
	b.commands = append(b.commands, c)
	if b.onSpawn != nil {
		b.onSpawn()
	}
	if len(b.children) < len(b.commands) {
		return nil, errors.Join(ctx.Err(), b.spawnErr, errBrokerExecutionRefused)
	}
	return b.children[len(b.commands)-1], errors.Join(ctx.Err(), b.spawnErr)
}
func brokerExecutionFixture() (*brokerExecutionActions, *memoryBrokerExecutionBackend) {
	children := []*memoryBrokerExecutionChild{{info: brokerExecutionChildInfo{7, 123, strings.Repeat("a", 64)}}, {info: brokerExecutionChildInfo{8, 124, strings.Repeat("b", 64)}}}
	backend := &memoryBrokerExecutionBackend{children: children}
	return &brokerExecutionActions{request: brokerpipe.ExecutionRequest{Schema: 1, Operation: "execute-session", RequestID: strings.Repeat("1", 32), StageID: strings.Repeat("2", 32)}, backend: backend}, backend
}
func brokerExecutionCommand(a *brokerExecutionActions, phase brokerpipe.ExecutionPhase) brokerpipe.ExecutionCommand {
	c := brokerpipe.ExecutionCommand{Schema: 1, Kind: "command", RequestID: a.request.RequestID, StageID: a.request.StageID, SessionID: strings.Repeat("3", 32), Sequence: 1, Challenge: strings.Repeat("4", 32), Phase: phase}
	switch phase {
	case brokerpipe.SpawnCandidate, brokerpipe.ObserveCandidate, brokerpipe.CloseCandidate:
		c.ApplicationGeneration = a.request.StageID
	case brokerpipe.SpawnFallback, brokerpipe.ObserveFallback, brokerpipe.CloseFallback:
		c.ApplicationGeneration = strings.Repeat("5", 32)
	}
	switch phase {
	case brokerpipe.SpawnCandidate, brokerpipe.ObserveCandidate, brokerpipe.SpawnFallback, brokerpipe.ObserveFallback:
		c.HealthPipe = "Skager.Update." + strings.Repeat("6", 32)
		c.HealthChallenge = strings.Repeat("7", 64)
	}
	return c
}
func TestBrokerExecutionOwnedRoleFlowAndFinishKeepsApplicationAlive(t *testing.T) {
	a, b := brokerExecutionFixture()
	defer a.close()
	ctx := context.Background()
	for _, phase := range []brokerpipe.ExecutionPhase{brokerpipe.SpawnCandidate, brokerpipe.ObserveCandidate, brokerpipe.CloseCandidate, brokerpipe.SpawnFallback, brokerpipe.ObserveFallback, brokerpipe.FinishExecution} {
		report, err := a.handle(ctx, brokerExecutionCommand(a, phase))
		if err != nil {
			t.Fatalf("%s: %v", phase, err)
		}
		if report.Status != "reported-complete" {
			t.Fatal(report)
		}
		if phase != brokerpipe.FinishExecution && (report.ProcessID == 0 || report.ProcessCreation == 0 || report.EvidenceSHA256 == "") {
			t.Fatal("lost actual owned observation", report)
		}
	}
	if len(b.commands) != 2 || !b.children[0].exited || b.children[0].requests != 1 || b.children[1].exited || b.children[1].requests != 0 {
		t.Fatal("implicit signal/fallback/extra creator")
	}
	for _, child := range b.children {
		if child.closes != 1 || !child.handlesClosed {
			t.Fatal("creator handle cleanup")
		}
	}
	if _, err := a.handle(ctx, brokerExecutionCommand(a, brokerpipe.SpawnCandidate)); err == nil {
		t.Fatal("finished action resurrected")
	}
}
func TestBrokerExecutionUnsupportedActualPhasesRefuseWithoutEffects(t *testing.T) {
	for _, phase := range []brokerpipe.ExecutionPhase{brokerpipe.InspectPrevious, brokerpipe.RestorePrevious, brokerpipe.CommissionCandidate, brokerpipe.CommissionFallback, brokerpipe.RestoreProfile, "run-command"} {
		t.Run(string(phase), func(t *testing.T) {
			a, b := brokerExecutionFixture()
			defer a.close()
			if report, err := a.handle(context.Background(), brokerExecutionCommand(a, phase)); err == nil || report.Status != "" {
				t.Fatal("unimplemented effect reported complete", report, err)
			}
			if len(b.commands) != 0 {
				t.Fatal("unsupported phase created a child")
			}
		})
	}
}
func TestBrokerExecutionConsumesBeforeNativeAttemptAndNeverRetriesUncertainty(t *testing.T) {
	for _, mode := range []string{"create-error", "observe-error", "resume-uncertain", "close-error"} {
		t.Run(mode, func(t *testing.T) {
			a, b := brokerExecutionFixture()
			defer a.close()
			b.onSpawn = func() {
				if !a.candidate.consumed {
					t.Fatal("attempt not consumed before creator")
				}
			}
			fault := errors.New(mode)
			switch mode {
			case "create-error", "resume-uncertain":
				b.spawnErr = fault
			case "observe-error":
				b.children[0].observeErr = fault
			}
			_, firstErr := a.handle(context.Background(), brokerExecutionCommand(a, brokerpipe.SpawnCandidate))
			if mode == "close-error" {
				if firstErr != nil {
					t.Fatal(firstErr)
				}
				b.children[0].requestCloseErr = fault
				_, firstErr = a.handle(context.Background(), brokerExecutionCommand(a, brokerpipe.CloseCandidate))
			}
			if !errors.Is(firstErr, errBrokerExecutionUncertain) || !errors.Is(firstErr, fault) {
				t.Fatal("lost uncertainty/cause", firstErr)
			}
			if _, err := a.handle(context.Background(), brokerExecutionCommand(a, brokerpipe.SpawnCandidate)); err == nil {
				t.Fatal("retried native uncertainty")
			}
			if _, err := a.handle(context.Background(), brokerExecutionCommand(a, brokerpipe.SpawnFallback)); err == nil {
				t.Fatal("fallback after uncertain effect")
			}
			if len(b.commands) != 1 {
				t.Fatal("extra native create")
			}
			if err := a.close(); err != nil {
				t.Fatal(err)
			}
			if b.children[0].exited && mode != "close-error" {
				t.Fatal("cleanup signalled live child")
			}
		})
	}
}
func TestBrokerExecutionRefusesRepeatedRolesOrConcurrentWriters(t *testing.T) {
	for _, phase := range []brokerpipe.ExecutionPhase{brokerpipe.SpawnCandidate, brokerpipe.SpawnFallback, brokerpipe.ObserveFallback, brokerpipe.CloseFallback} {
		t.Run(string(phase), func(t *testing.T) {
			a, b := brokerExecutionFixture()
			defer a.close()
			if _, err := a.handle(context.Background(), brokerExecutionCommand(a, brokerpipe.SpawnCandidate)); err != nil {
				t.Fatal(err)
			}
			if _, err := a.handle(context.Background(), brokerExecutionCommand(a, phase)); err == nil {
				t.Fatal("repeated or unowned role accepted")
			}
			if len(b.commands) != 1 {
				t.Fatal("simultaneous real-profile child")
			}
		})
	}
}
func TestBrokerExecutionObserveRequiresExactOriginalHealthBinding(t *testing.T) {
	for _, field := range []string{"generation", "pipe", "challenge", "request", "stage"} {
		t.Run(field, func(t *testing.T) {
			a, b := brokerExecutionFixture()
			defer a.close()
			if _, err := a.handle(context.Background(), brokerExecutionCommand(a, brokerpipe.SpawnCandidate)); err != nil {
				t.Fatal(err)
			}
			c := brokerExecutionCommand(a, brokerpipe.ObserveCandidate)
			switch field {
			case "generation":
				c.ApplicationGeneration = strings.Repeat("f", 32)
			case "pipe":
				c.HealthPipe = "Skager.Update." + strings.Repeat("f", 32)
			case "challenge":
				c.HealthChallenge = strings.Repeat("f", 64)
			case "request":
				c.RequestID = strings.Repeat("f", 32)
			case "stage":
				c.StageID = strings.Repeat("f", 32)
			}
			if _, err := a.handle(context.Background(), c); err == nil {
				t.Fatal("different original binding")
			}
			if b.children[0].observes != 1 {
				t.Fatal("invalid phase reached owned child")
			}
		})
	}
}
func TestBrokerExecutionMalformedDirectiveNeverReachesCreator(t *testing.T) {
	for _, mode := range []string{"path-generation", "uppercase", "foreign-candidate", "candidate-as-fallback", "path-pipe", "short-challenge", "health-on-close", "health-on-finish"} {
		t.Run(mode, func(t *testing.T) {
			a, b := brokerExecutionFixture()
			defer a.close()
			c := brokerExecutionCommand(a, brokerpipe.SpawnCandidate)
			switch mode {
			case "path-generation":
				c.ApplicationGeneration = "../app"
			case "uppercase":
				c.ApplicationGeneration = strings.Repeat("A", 32)
			case "foreign-candidate":
				c.ApplicationGeneration = strings.Repeat("f", 32)
			case "candidate-as-fallback":
				c.Phase = brokerpipe.SpawnFallback
			case "path-pipe":
				c.HealthPipe = `\\.\pipe\Skager.Update.` + strings.Repeat("6", 32)
			case "short-challenge":
				c.HealthChallenge = "a"
			case "health-on-close":
				c.Phase = brokerpipe.CloseCandidate
			case "health-on-finish":
				c.Phase = brokerpipe.FinishExecution
			}
			if _, err := a.handle(context.Background(), c); err == nil {
				t.Fatal("unsafe server payload")
			}
			if len(b.commands) != 0 {
				t.Fatal("unsafe payload reached creator")
			}
		})
	}
}
func TestBrokerExecutionConversationLossClosesHandlesWithoutOrdinaryFallback(t *testing.T) {
	_, b := brokerExecutionFixture()
	var requests []brokerpipe.ExecutionRequest
	err := runBrokerExecution(context.Background(), strings.Repeat("2", 32), b, func(ctx context.Context, q brokerpipe.ExecutionRequest, handle brokerpipe.ExecutionClientHandler) error {
		requests = append(requests, q)
		a := &brokerExecutionActions{request: q}
		_, err := handle(ctx, brokerExecutionCommand(a, brokerpipe.SpawnCandidate))
		if err != nil {
			t.Fatal(err)
		}
		return io.EOF
	})
	if !errors.Is(err, io.EOF) || len(b.commands) != 1 || !b.children[0].handlesClosed || b.children[0].exited || b.children[0].requests != 0 {
		t.Fatal("lost conversation caused retry/termination", err)
	}
	if len(requests) != 1 || !generationPattern.MatchString(requests[0].RequestID) {
		t.Fatal("request not freshly bounded")
	}
}
func TestBrokerExecutionFinishIsOnlyTransportCompletionAndCleanupFailurePersists(t *testing.T) {
	a, b := brokerExecutionFixture()
	report, err := a.handle(context.Background(), brokerExecutionCommand(a, brokerpipe.FinishExecution))
	if err != nil || report.ProcessID != 0 || len(b.commands) != 0 {
		t.Fatal("finish invented process/health", report, err)
	}
	a, b = brokerExecutionFixture()
	if _, err = a.handle(context.Background(), brokerExecutionCommand(a, brokerpipe.SpawnCandidate)); err != nil {
		t.Fatal(err)
	}
	fault := errors.New("native handle cleanup")
	b.children[0].closeErr = fault
	_, err = a.handle(context.Background(), brokerExecutionCommand(a, brokerpipe.FinishExecution))
	if !errors.Is(err, errBrokerExecutionUncertain) || !errors.Is(err, fault) {
		t.Fatal(err)
	}
	if err = a.close(); !errors.Is(err, fault) {
		t.Fatal("cleanup uncertainty lost", err)
	}
	if b.children[0].exited {
		t.Fatal("finish terminated app")
	}
}
func TestBrokerExecutionConcurrentRepeatedSpawnCallsOneCreator(t *testing.T) {
	a, b := brokerExecutionFixture()
	defer a.close()
	var group sync.WaitGroup
	for i := 0; i < 8; i++ {
		group.Add(1)
		go func() {
			defer group.Done()
			_, _ = a.handle(context.Background(), brokerExecutionCommand(a, brokerpipe.SpawnCandidate))
		}()
	}
	group.Wait()
	if len(b.commands) != 1 {
		t.Fatal("concurrent repeat created more than one child")
	}
}
func TestBrokerExecutionCancelledRequestCannotCreate(t *testing.T) {
	a, b := brokerExecutionFixture()
	defer a.close()
	ctx, cancel := context.WithCancel(context.Background())
	cancel()
	if _, err := a.handle(ctx, brokerExecutionCommand(a, brokerpipe.SpawnCandidate)); !errors.Is(err, context.Canceled) {
		t.Fatal(err)
	}
	if len(b.commands) != 0 {
		t.Fatal("cancelled creator")
	}
}
func TestBrokerExecutionArgumentAndFixedApplicationNamespace(t *testing.T) {
	id := strings.Repeat("a", 32)
	for _, args := range [][]string{{"--broker-execute"}, {"--broker-execute="}, {"--broker-execute=" + strings.Repeat("A", 32)}, {"--broker-execute=../app"}, {"--broker-execute=" + id, "--legacy"}, {"--legacy", "--broker-execute=" + id}, {"--broker-execute-more=" + id}} {
		if _, matched, err := brokerExecutionArgument(args); !matched || err == nil {
			t.Fatal("unsafe early dispatch", args)
		}
	}
	if got, matched, err := brokerExecutionArgument([]string{"--broker-execute=" + id}); got != id || !matched || err != nil {
		t.Fatal(got, matched, err)
	}
	if _, matched, err := brokerExecutionArgument([]string{"--legacy"}); matched || err != nil {
		t.Fatal("ordinary dispatch changed")
	}
	name, err := brokerExecutionApplication(`C:\Program Files (x86)`, "S-1-5-21-1-2-3-500", id)
	if err != nil || name != "C:/Program Files (x86)/SKAGER/Applications/S-1-5-21-1-2-3-500/generations/"+id+"/app/opencpn.exe" {
		t.Fatal(name, err)
	}
	for _, sid := range []string{"S-1-5-18", "S-1-5-21-01-2-3-500", "../user"} {
		if _, err := brokerExecutionApplication(`C:\Program Files (x86)`, sid, id); err == nil {
			t.Fatal("unsafe client root", sid)
		}
	}
}
