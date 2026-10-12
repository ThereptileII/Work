package brokerhost

import (
	"context"
	"errors"
	"io"
	"sync"
	"time"

	"example.com/opennav-update-verifier/brokerpipe"
	"example.com/opennav-update-verifier/brokerstore"
)

var ErrExecutionChildRefused = errors.New("native execution child observation refused")
var ErrExecutionChildCancelled = errors.New("application startup navigation warning cancelled")
var ErrExecutionChildCleanup = errors.New("execution child observation cleanup uncertain")

const executionStartupTimeout = 90 * time.Second
const executionHumanTimeout = 300 * time.Second
const executionObservationTimeout = 8 * time.Minute
const executionMinimumReadyAge = 30 * time.Second

type executionChildAuthority interface {
	withExecutor(context.Context, func(brokerpipe.PeerIdentity) error) error
}
type executionPeerAuthority struct{ peer *brokerpipe.PeerCapability }

func (p executionPeerAuthority) withExecutor(ctx context.Context, use func(brokerpipe.PeerIdentity) error) error {
	return p.peer.WithExecutor(ctx, use)
}

type executionChildBackend interface {
	check(context.Context) error
	connect(context.Context) error
	authenticate(context.Context) (time.Duration, error)
	read(context.Context, []byte) (int, error)
	validate(context.Context) error
	close() error
}

// ExecutionChildObserver owns one native local health pipe and retained actual
// executor/child process+token observations. It neither creates nor signals a
// process. Its private native factory accepts no caller PID, path or receipt.
// Selection, consumed spawn and real profile commissioning remain host gates.
type ExecutionChildObserver struct{ state *executionChildState }

// ObservedExecutionChild can only result from this live observer's authenticated
// native pipe/process/image evidence and strict producer READY+EOF. It is not a
// durable permit, selection or authority to spawn, close, terminate or fallback.
// Closing either observer or original generation/peer revokes it.
type ObservedExecutionChild struct{ state *executionChildState }
type executionChildState struct {
	mu                         sync.Mutex
	owner                      context.Context
	cancel                     context.CancelFunc
	authority                  executionChildAuthority
	expected                   brokerpipe.PeerIdentity
	identity                   brokerstore.GenerationApplicationIdentity
	backend                    executionChildBackend
	profile                    *ExecutionColdProfile
	prepared                   time.Time
	environment                map[string]string
	now                        func() time.Time // private per-owner seam; production time.Now
	attempted, healthy, closed bool
	failure, closeErr          error
}

// PrepareExecutionChildNative arms ConnectNamedPipe before the interactive
// caller may create its child. The peer must already be bound to the protected
// selected predecessor launcher. Application image may be another same-client
// generation; this API does not validate selected-chain or commissioning policy.
func PrepareExecutionChildNative(ctx context.Context, executor *brokerpipe.PeerCapability, image *brokerstore.GenerationApplicationImage, profile *ExecutionColdProfile) (*ExecutionChildObserver, error) {
	if ctx == nil || executor == nil || image == nil || profile == nil {
		return nil, ErrExecutionChildRefused
	}
	if err := profile.ValidateBeforeLaunch(ctx); err != nil {
		return nil, err
	}
	identity, err := validateExecutionCapability(ctx, image)
	if err != nil {
		return nil, err
	}
	return prepareExecutionChildNative(ctx, executionPeerAuthority{executor}, image, identity, profile)
}
func newExecutionChildObserver(ctx context.Context, authority executionChildAuthority, expected brokerpipe.PeerIdentity, identity brokerstore.GenerationApplicationIdentity, backend executionChildBackend, profile *ExecutionColdProfile, prepared time.Time, environment map[string]string) *ExecutionChildObserver {
	owner, cancel := context.WithTimeout(ctx, executionObservationTimeout)
	return &ExecutionChildObserver{state: &executionChildState{owner: owner, cancel: cancel, authority: authority, expected: expected, identity: identity, backend: backend, profile: profile, prepared: prepared, environment: environment, now: time.Now}}
}
func (s *executionChildState) ready(ctx context.Context) error {
	if s == nil || s.closed || s.owner == nil || s.cancel == nil || s.authority == nil || s.backend == nil || s.profile == nil {
		return ErrExecutionChildRefused
	}
	if s.failure != nil {
		return errors.Join(ErrExecutionChildRefused, s.failure)
	}
	if ctx == nil {
		return ErrExecutionChildRefused
	}
	return errors.Join(ctx.Err(), s.owner.Err())
}
func (s *executionChildState) validate(ctx context.Context) error {
	if err := s.ready(ctx); err != nil {
		return err
	}
	if err := s.profile.Validate(ctx); err != nil {
		return err
	}
	return s.authority.withExecutor(ctx, func(actual brokerpipe.PeerIdentity) error {
		if actual != s.expected || actual.SID != s.identity.ClientSID {
			return ErrExecutionChildRefused
		}
		return s.backend.validate(ctx)
	})
}

// Environment returns only owned copies of the four generated fields consumed
// and cleared by the original application. It rechecks the live executor/session
// each time and never returns the broker's or interactive user's environment.
func (o *ExecutionChildObserver) Environment(ctx context.Context) (map[string]string, error) {
	if o == nil || o.state == nil {
		return nil, ErrExecutionChildRefused
	}
	s := o.state
	s.mu.Lock()
	defer s.mu.Unlock()
	if s.attempted {
		return nil, ErrExecutionChildRefused
	}
	if err := s.profile.ValidateBeforeLaunch(ctx); err != nil {
		s.failure = errors.Join(s.failure, err)
		return nil, err
	}
	if err := s.validate(ctx); err != nil {
		s.failure = errors.Join(s.failure, err)
		return nil, err
	}
	values := map[string]string{}
	for name, value := range s.environment {
		values[name] = value
	}
	return values, nil
}

// Observe consumes this one observation attempt. PID comes only from the actual
// pipe client. READY alone is insufficient: exact ordered phases, bounded EOF,
// original application image, actual live process/token/session/logon/direct
// parent and the live image-bound executor must all corroborate it.
func (o *ExecutionChildObserver) Observe(ctx context.Context) (result *ObservedExecutionChild, resultErr error) {
	if o == nil || o.state == nil {
		return nil, ErrExecutionChildRefused
	}
	s := o.state
	s.mu.Lock()
	defer s.mu.Unlock()
	if err := s.ready(ctx); err != nil {
		return nil, err
	}
	if s.attempted {
		return nil, ErrExecutionChildRefused
	}
	s.attempted = true
	defer func() {
		if resultErr != nil {
			s.failure = errors.Join(s.failure, resultErr)
			resultErr = errors.Join(resultErr, s.closeLocked())
		}
	}()
	operation, cancel := context.WithCancel(s.owner)
	defer cancel()
	stop := context.AfterFunc(ctx, cancel)
	defer stop()
	if err := s.validate(operation); err != nil {
		return nil, err
	}
	deadline := s.prepared.Add(executionStartupTimeout)
	connect, stopConnect := context.WithDeadline(operation, deadline)
	err := s.backend.connect(connect)
	stopConnect()
	if err != nil {
		return nil, err
	}
	authCtx, stopAuth := context.WithDeadline(operation, deadline)
	readyAfter, err := s.backend.authenticate(authCtx)
	stopAuth()
	if err != nil || readyAfter < executionMinimumReadyAge {
		return nil, errors.Join(ErrExecutionChildRefused, err)
	}
	configuration, err := s.profile.configuration(operation)
	if err != nil {
		return nil, err
	}
	protocol := executionChildProtocol{configuration: configuration, identity: s.identity.Generation + " " + s.identity.Commit + " " + s.environment["SKAGER_UPDATE_CHALLENGE"] + "\n"}
	buffer := make([]byte, 513)
	for {
		if err := s.ready(operation); err != nil {
			return nil, err
		}
		if !s.now().Before(deadline) {
			return nil, context.DeadlineExceeded
		}
		readCtx, stopRead := context.WithDeadline(operation, deadline)
		n, readErr := s.backend.read(readCtx, buffer)
		stopRead()
		if n < 0 || n > len(buffer) {
			return nil, ErrExecutionChildRefused
		}
		for _, value := range buffer[:n] {
			phase, err := protocol.feed(value)
			if err != nil {
				return nil, err
			}
			if phase != "" {
				if err = s.validate(operation); err != nil {
					return nil, err
				}
				now := s.now()
				if !now.Before(deadline) {
					return nil, context.DeadlineExceeded
				}
				switch phase {
				case "wait":
					deadline = now.Add(executionHumanTimeout)
				case "continue":
					deadline = now.Add(executionStartupTimeout)
				case "ready":
					if now.Sub(s.prepared) < readyAfter {
						return nil, ErrExecutionChildRefused
					}
				}
			}
		}
		if readErr != nil && !errors.Is(readErr, io.EOF) {
			return nil, readErr
		}
		if n == 0 && readErr == nil {
			return nil, io.ErrNoProgress
		}
		if errors.Is(readErr, io.EOF) {
			if protocol.used != 0 || protocol.terminal == "" {
				return nil, ErrExecutionChildRefused
			}
			if protocol.terminal == "cancel" {
				return nil, ErrExecutionChildCancelled
			}
			if err = s.validate(operation); err != nil {
				return nil, err
			}
			if !s.now().Before(deadline) {
				return nil, context.DeadlineExceeded
			}
			s.healthy = true
			return &ObservedExecutionChild{state: s}, nil
		}
	}
}
func (c *ObservedExecutionChild) Validate(ctx context.Context) error {
	if c == nil || c.state == nil {
		return ErrExecutionChildRefused
	}
	s := c.state
	s.mu.Lock()
	defer s.mu.Unlock()
	if !s.healthy {
		return ErrExecutionChildRefused
	}
	if err := s.validate(ctx); err != nil {
		s.failure = errors.Join(s.failure, err)
		return err
	}
	return nil
}
func (s *executionChildState) closeLocked() error {
	if s.closed {
		return s.closeErr
	}
	s.closed = true
	s.healthy = false
	s.cancel()
	if err := s.backend.close(); err != nil {
		s.closeErr = errors.Join(ErrExecutionChildCleanup, err)
	}
	return s.closeErr
}
func (o *ExecutionChildObserver) Close() error {
	if o == nil || o.state == nil {
		return nil
	}
	s := o.state
	// Cancellation is thread-safe and occurs before taking the lifetime lock, so
	// Close interrupts a blocked native connect/read and joins its drained IO.
	s.cancel()
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.closeLocked()
}
func (c *ObservedExecutionChild) Close() error {
	if c == nil {
		return nil
	}
	return (&ExecutionChildObserver{state: c.state}).Close()
}
func (*ExecutionChildObserver) MarshalJSON() ([]byte, error) { return nil, ErrExecutionChildRefused }
func (*ExecutionChildObserver) UnmarshalJSON([]byte) error   { return ErrExecutionChildRefused }
func (*ObservedExecutionChild) MarshalJSON() ([]byte, error) { return nil, ErrExecutionChildRefused }
func (*ObservedExecutionChild) UnmarshalJSON([]byte) error   { return ErrExecutionChildRefused }

type executionChildProtocol struct {
	identity, configuration string
	record                  [512]byte
	used, records           int
	phase, terminal         string
}

func (p *executionChildProtocol) feed(value byte) (string, error) {
	if p.identity == "" || p.configuration == "" || p.terminal != "" || value > 127 || p.used >= len(p.record) {
		return "", ErrExecutionChildRefused
	}
	p.record[p.used] = value
	p.used++
	if value != '\n' {
		return "", nil
	}
	p.records++
	if p.records > 4 {
		return "", ErrExecutionChildRefused
	}
	frame := string(p.record[:p.used])
	p.used = 0
	switch {
	case frame == "SKAGER-UPDATE-CONFIG/1 "+p.identity[:len(p.identity)-1]+" "+p.configuration+"\n" && p.phase == "" && p.records == 1:
		p.phase = "config"
	case frame == "SKAGER-UPDATE-WAIT/1 "+p.identity && p.phase == "config" && p.records == 2:
		p.phase = "wait"
	case frame == "SKAGER-UPDATE-CONTINUE/1 "+p.identity && p.phase == "wait" && p.records == 3:
		p.phase = "continue"
	case frame == "SKAGER-UPDATE-CANCEL/1 "+p.identity && p.phase == "wait" && p.records == 3:
		p.phase = "cancel"
		p.terminal = "cancel"
	case frame == "SKAGER-UPDATE-READY/1 "+p.identity && (p.phase == "config" || p.phase == "continue"):
		p.phase = "ready"
		p.terminal = "ready"
	default:
		return "", ErrExecutionChildRefused
	}
	return p.phase, nil
}
