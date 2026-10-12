package brokerpipe

import (
	"context"
	"encoding/json"
	"errors"
	"sync"
	"testing"
	"time"

	"example.com/opennav-update-verifier/brokerstore"
)

// Private test authorities never substitute for native SCM/token/image checks.
type fakeExecutionAuthority struct {
	bound                           bool
	uses, closes                    int
	failBefore, failAfter, closeErr error
}

func (a *fakeExecutionAuthority) bind(context.Context, *brokerstore.GenerationExecutorImage) error {
	a.bound = true
	return nil
}
func (a *fakeExecutionAuthority) use(ctx context.Context, f func(PeerIdentity) error) error {
	if !a.bound || a.failBefore != nil {
		return errors.Join(ErrRefused, a.failBefore)
	}
	a.uses++
	return errors.Join(f(PeerIdentity{}), a.failAfter, ctx.Err())
}
func (a *fakeExecutionAuthority) close() error { a.closes++; return a.closeErr }

type fakeExecutionConnection struct {
	commands []ExecutionCommand
	response []byte
	mutate   func([]byte) []byte
	queued   bool
	started  chan struct{}
	block    bool
}

func (c *fakeExecutionConnection) write(ctx context.Context, raw []byte) error {
	command, err := decodeExecutionCommand(raw, "command")
	if err != nil {
		return err
	}
	c.commands = append(c.commands, command)
	observation := executionTestObservation()
	if command.Phase == FinishExecution {
		observation.Status = "reported-failed"
		observation.EvidenceSHA256 = ""
	}
	c.response, err = encodeExecutionReport(command, observation)
	if c.mutate != nil {
		c.response = c.mutate(c.response)
	}
	return err
}
func (c *fakeExecutionConnection) read(ctx context.Context) ([]byte, error) {
	if c.started != nil {
		close(c.started)
		c.started = nil
	}
	if c.block {
		<-ctx.Done()
		return nil, ctx.Err()
	}
	return c.response, nil
}
func (c *fakeExecutionConnection) idle() error {
	if c.queued {
		return ErrRefused
	}
	return nil
}
func executionTestSession(t *testing.T) (*ExecutionSession, *fakeExecutionAuthority, *fakeExecutionConnection) {
	t.Helper()
	ctx, cancel := context.WithCancel(context.Background())
	a, c := &fakeExecutionAuthority{}, &fakeExecutionConnection{}
	s, err := newExecutionSession(ctx, cancel, executionTestRequest(), a, c)
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { s.Close() })
	return s, a, c
}
func bindExecutionTestSession(t *testing.T, s *ExecutionSession) {
	t.Helper()
	if err := s.BindExecutorImage(context.Background(), &brokerstore.GenerationExecutorImage{}); err != nil {
		t.Fatal(err)
	}
}

func TestExecutionSessionNativeMintAndBindingBoundary(t *testing.T) {
	for _, s := range []*ExecutionSession{nil, {}} {
		if _, err := s.Exchange(context.Background(), InspectPrevious); !errors.Is(err, ErrRefused) {
			t.Fatal("zero session accepted")
		}
		if s.Close() != nil {
			t.Fatal("zero close failed")
		}
	}
	s, a, c := executionTestSession(t)
	if _, err := json.Marshal(s); err == nil {
		t.Fatal("serialized capability")
	}
	if json.Unmarshal([]byte(`{}`), s) == nil {
		t.Fatal("deserialized capability")
	}
	if s.BindExecutorImage(context.Background(), nil) == nil {
		t.Fatal("nil image accepted")
	}
	if _, err := s.ExchangeCommand(context.Background(), executionTestDirective(SpawnCandidate)); !errors.Is(err, ErrRefused) {
		t.Fatal("unbound execution accepted")
	}
	if len(c.commands) != 0 || a.closes != 1 {
		t.Fatal("unbound request escaped or owner not drained")
	}
	if s.BindExecutorImage(context.Background(), &brokerstore.GenerationExecutorImage{}) == nil {
		t.Fatal("failed session reused")
	}
	copy := *s
	if copy.Close() != nil || s.Close() != nil || a.closes != 1 {
		t.Fatal("copied session cleanup repeated")
	}
}
func TestExecutionSessionExplicitPhasesAndDataOnlyFinish(t *testing.T) {
	s, a, c := executionTestSession(t)
	bindExecutionTestSession(t, s)
	for _, phase := range []ExecutionPhase{InspectPrevious, CommissionCandidate, CommissionCandidate, ObserveCandidate, CloseCandidate, CommissionFallback, ObserveFallback, RestoreProfile, FinishExecution} {
		observation, err := s.ExchangeCommand(context.Background(), executionTestDirective(phase))
		if err != nil {
			t.Fatal(phase, err)
		}
		if phase == FinishExecution && observation.Status != "reported-failed" {
			t.Fatal("finish was transformed into transaction success")
		}
	}
	for i, command := range c.commands {
		if command.Sequence != uint32(i+1) || command.SessionID != c.commands[0].SessionID {
			t.Fatal("bad envelope")
		}
		for j := 0; j < i; j++ {
			if command.Challenge == c.commands[j].Challenge {
				t.Fatal("challenge reused")
			}
		}
	}
	if a.uses != len(c.commands) {
		t.Fatal("phase omitted owner validation")
	}
	if s.finishHandlerScope() != nil {
		t.Fatal("explicit transport finish refused")
	}
	if _, err := s.ExchangeCommand(context.Background(), executionTestDirective(SpawnCandidate)); err == nil {
		t.Fatal("phase after finish accepted")
	}
	copy := *s
	copy.Close()
	s.Close()
	if a.closes != 1 {
		t.Fatal("cleanup was repeated")
	}
}
func TestExecutionSessionFaultsConsumeAndDrain(t *testing.T) {
	for _, name := range []string{"unrequested-report", "trailing-report", "identity-before", "identity-after", "missing-finish", "unknown-phase", "limit", "canceled-owner", "cleanup-failure"} {
		t.Run(name, func(t *testing.T) {
			s, a, c := executionTestSession(t)
			bindExecutionTestSession(t, s)
			sentinel := errors.New("retained native fault")
			phase := InspectPrevious
			switch name {
			case "unrequested-report":
				c.mutate = func(raw []byte) []byte {
					var r executionReport
					json.Unmarshal(raw, &r)
					r.Sequence++
					b, _ := json.Marshal(r)
					return b
				}
			case "trailing-report":
				c.mutate = func(raw []byte) []byte { c.queued = true; return raw }
			case "identity-before":
				a.failBefore = sentinel
			case "identity-after":
				a.failAfter = sentinel
			case "missing-finish":
				if s.finishHandlerScope() == nil {
					t.Fatal("handler return implied finish")
				}
				s.Close()
				return
			case "unknown-phase":
				phase = "execute-path"
			case "limit":
				s.state.sequence = maxExecutionPhases
			case "canceled-owner":
				s.state.cancel()
			case "cleanup-failure":
				a.closeErr = sentinel
				a.failAfter = sentinel
			}
			if _, err := s.Exchange(context.Background(), phase); err == nil {
				t.Fatal("fault accepted")
			}
			before := len(c.commands)
			if _, err := s.Exchange(context.Background(), InspectPrevious); err == nil {
				t.Fatal("fault retry accepted")
			}
			if len(c.commands) != before {
				t.Fatal("retry sent another command")
			}
			first := s.Close()
			second := s.Close()
			if first != second || a.closes != 1 {
				t.Fatal("cleanup not retained exactly once")
			}
			if name == "cleanup-failure" && !errors.Is(first, sentinel) {
				t.Fatal("cleanup fault discarded")
			}
		})
	}
}
func TestExecutionSessionCloseCancelsAndJoinsPhase(t *testing.T) {
	s, a, c := executionTestSession(t)
	bindExecutionTestSession(t, s)
	started := make(chan struct{})
	c.started = started
	c.block = true
	phaseDone := make(chan error, 1)
	go func() {
		_, err := s.ExchangeCommand(context.Background(), executionTestDirective(ObserveCandidate))
		phaseDone <- err
	}()
	select {
	case <-started:
	case <-time.After(time.Second):
		t.Fatal("phase not entered")
	}
	// Concurrent server phases refuse rather than queuing behind an old challenge.
	if _, err := s.ExchangeCommand(context.Background(), executionTestDirective(SpawnCandidate)); err == nil {
		t.Fatal("concurrent phase accepted")
	}
	closeDone := make(chan error, 1)
	go func() { closeDone <- s.Close() }()
	select {
	case err := <-phaseDone:
		if !errors.Is(err, context.Canceled) {
			t.Fatal("cancellation lost", err)
		}
	case <-time.After(time.Second):
		t.Fatal("phase did not drain")
	}
	select {
	case <-closeDone:
	case <-time.After(time.Second):
		t.Fatal("close did not join phase")
	}
	if a.closes != 1 {
		t.Fatal("authority cleanup repeated")
	}
}
func TestExecutionSessionConcurrentCopiesCannotExtendLifetime(t *testing.T) {
	s, a, _ := executionTestSession(t)
	copy := *s
	var wg sync.WaitGroup
	for i := 0; i < 8; i++ {
		wg.Add(1)
		go func() { defer wg.Done(); copy.Close() }()
	}
	wg.Wait()
	if a.closes != 1 {
		t.Fatal("multiple cleanup owners")
	}
	if _, err := s.Exchange(context.Background(), InspectPrevious); err == nil {
		t.Fatal("copy extended revoked lifetime")
	}
}
func TestExecutionReplayIsBoundedAndFailurePermanent(t *testing.T) {
	var replay executionRequests
	q := executionTestRequest()
	p := Peer{"S-1-5-21-1-2-3-1001", 1, 99}
	if replay.reserve(p, q) != nil || replay.reserve(p, q) == nil {
		t.Fatal("request replay allowed")
	}
	q.RequestID = executionTestCommand().SessionID
	if replay.reserve(p, q) != nil {
		t.Fatal("fresh request refused")
	}
	replay.seen = map[string]bool{}
	for i := 0; i < maxRequests; i++ {
		replay.seen[string(rune(i))] = true
	}
	if replay.reserve(p, q) == nil {
		t.Fatal("bounded set evicted into reuse")
	}
}

func TestExecutionHandlerReturnAndPanicRevokeBeforeCleanup(t *testing.T) {
	for _, mode := range []string{"error", "panic", "missing-finish", "explicit-finish"} {
		t.Run(mode, func(t *testing.T) {
			s, a, _ := executionTestSession(t)
			bindExecutionTestSession(t, s)
			handler := func(ctx context.Context, _ Peer, _ ExecutionRequest, session *ExecutionSession) error {
				switch mode {
				case "error":
					return ErrRefused
				case "panic":
					panic("handler fixture")
				case "explicit-finish":
					_, err := session.Exchange(ctx, FinishExecution)
					return err
				}
				return nil
			}
			var err error
			var panicked bool
			func() {
				defer func() { panicked = recover() != nil }()
				err = invokeExecutionHandler(context.Background(), Peer{}, executionTestRequest(), handler, s)
			}()
			if (mode == "panic") != panicked {
				t.Fatal("panic semantics changed")
			}
			if mode == "explicit-finish" {
				if err != nil {
					t.Fatal(err)
				}
			} else if mode != "panic" && err == nil {
				t.Fatal("handler implied finish")
			}
			if a.closes != 1 {
				t.Fatal("capability not revoked immediately at handler boundary")
			}
			if _, err := s.ExchangeCommand(context.Background(), executionTestDirective(SpawnCandidate)); err == nil {
				t.Fatal("escaped session remained live")
			}
		})
	}
}

func TestExecutionSpawnObserveRefuseBarePhase(t *testing.T) {
	for _, phase := range []ExecutionPhase{SpawnCandidate, ObserveCandidate, SpawnFallback, ObserveFallback} {
		t.Run(string(phase), func(t *testing.T) {
			s, a, c := executionTestSession(t)
			bindExecutionTestSession(t, s)
			if _, err := s.Exchange(context.Background(), phase); err == nil {
				t.Fatal("bare spawn/observe accepted")
			}
			if len(c.commands) != 0 || a.closes != 1 {
				t.Fatal("invalid directive escaped native boundary")
			}
		})
	}
}
