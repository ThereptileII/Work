package brokerpipe

import (
	"context"
	"crypto/rand"
	"encoding/hex"
	"encoding/json"
	"errors"
	"sync"
	"sync/atomic"
	"time"

	"example.com/opennav-update-verifier/brokerstore"
)

const executionPhaseTimeout = 5 * time.Minute

type executionAuthority interface {
	bind(context.Context, *brokerstore.GenerationExecutorImage) error
	use(context.Context, func(PeerIdentity) error) error
	close() error
}
type peerExecutionAuthority struct{ peer *PeerCapability }

func (p peerExecutionAuthority) bind(ctx context.Context, image *brokerstore.GenerationExecutorImage) error {
	return p.peer.BindExecutorImage(ctx, image)
}
func (p peerExecutionAuthority) use(ctx context.Context, f func(PeerIdentity) error) error {
	return p.peer.WithExecutor(ctx, f)
}
func (p peerExecutionAuthority) close() error { return p.peer.Close() }

type executionConnection interface {
	read(context.Context) ([]byte, error)
	write(context.Context, []byte) error
	idle() error
}

// ExecutionSession is a native-minted, revocable live conversation. Copies share
// one state. It cannot be reconstructed from JSON/identifiers; its native serving
// owner retains physical connection handles and joins the watcher before cleanup.
// Phase responses and clean transport finish grant no transaction authority.
type ExecutionSession struct{ state *executionSessionState }
type executionSessionState struct {
	mu                      sync.Mutex
	owner                   context.Context
	cancel                  context.CancelFunc
	request                 ExecutionRequest
	sessionID               string
	authority               executionAuthority
	connection              executionConnection
	incoming                atomic.Bool
	sequence                uint32
	terminal                ExecutionCommand
	bound, finished, closed bool
	failure, closeErr       error
}

func executionNonce() (string, error) {
	var b [16]byte
	if _, err := rand.Read(b[:]); err != nil {
		return "", err
	}
	return hex.EncodeToString(b[:]), nil
}
func newExecutionSession(ctx context.Context, cancel context.CancelFunc, q ExecutionRequest, authority executionAuthority, connection executionConnection) (*ExecutionSession, error) {
	if ctx == nil || cancel == nil || !validExecutionRequest(q) || authority == nil || connection == nil {
		return nil, ErrRefused
	}
	nonce, err := executionNonce()
	if err != nil {
		return nil, err
	}
	return &ExecutionSession{state: &executionSessionState{owner: ctx, cancel: cancel, request: q, sessionID: nonce, authority: authority, connection: connection}}, nil
}
func (s *executionSessionState) ready(ctx context.Context) error {
	if s.closed || s.finished || s.failure != nil {
		return errors.Join(ErrRefused, s.failure)
	}
	if ctx == nil || s.owner == nil || s.authority == nil || s.connection == nil || s.cancel == nil {
		return ErrRefused
	}
	return errors.Join(ctx.Err(), s.owner.Err())
}
func (s *executionSessionState) fail(err error) error {
	s.failure = errors.Join(ErrRefused, s.failure, err)
	s.incoming.Store(false)
	s.cancel()
	if !s.closed {
		s.closed = true
		s.closeErr = errors.Join(s.closeErr, s.authority.close())
	}
	return errors.Join(s.failure, s.closeErr)
}

// BindExecutorImage must succeed once before any business phase. It accepts only
// the sealed owner of fixed app/skager-start.exe, never a client path/hash claim.
func (s *ExecutionSession) BindExecutorImage(ctx context.Context, image *brokerstore.GenerationExecutorImage) error {
	if s == nil || s.state == nil || image == nil {
		return ErrRefused
	}
	p := s.state
	if !p.mu.TryLock() {
		return ErrRefused
	}
	defer p.mu.Unlock()
	if err := p.ready(ctx); err != nil {
		return err
	}
	if p.sequence != 0 || p.bound {
		return ErrRefused
	}
	if err := p.authority.bind(ctx, image); err != nil {
		return p.fail(err)
	}
	p.bound = true
	return nil
}

// Exchange issues exactly one finite server-selected phase with a fresh challenge.
// A report must match the entire envelope, and unsolicited/trailing bytes refuse.
// Report status is data even for finish: native host corroboration and durable
// one-use spawn/selection/restore/health decisions remain separate obligations.
func (s *ExecutionSession) Exchange(ctx context.Context, phase ExecutionPhase) (observation ExecutionObservation, resultErr error) {
	if s == nil || s.state == nil {
		return observation, ErrRefused
	}
	p := s.state
	if !p.mu.TryLock() {
		return observation, ErrRefused
	}
	defer p.mu.Unlock()
	if err := p.ready(ctx); err != nil {
		return observation, err
	}
	if !p.bound || !validExecutionPhase(phase) || p.sequence >= maxExecutionPhases {
		return observation, p.fail(ErrRefused)
	}
	phaseCtx, cancel := context.WithTimeout(ctx, executionPhaseTimeout)
	defer cancel()
	stopOwner := context.AfterFunc(p.owner, cancel)
	defer stopOwner()
	resultErr = p.authority.use(phaseCtx, func(PeerIdentity) error {
		if err := p.connection.idle(); err != nil {
			return err
		}
		challenge, err := executionNonce()
		if err != nil {
			return err
		}
		p.sequence++
		command := ExecutionCommand{Schema: 1, Kind: "command", RequestID: p.request.RequestID, StageID: p.request.StageID, SessionID: p.sessionID, Sequence: p.sequence, Challenge: challenge, Phase: phase}
		raw, err := json.Marshal(command)
		if err != nil {
			return err
		}
		// Arming before write avoids rejecting a prompt legitimate reply. The new
		// unpredictable challenge prevents a pre-sent guess from becoming a report.
		p.incoming.Store(true)
		defer p.incoming.Store(false)
		if err := p.connection.write(phaseCtx, raw); err != nil {
			return err
		}
		raw, err = p.connection.read(phaseCtx)
		if err != nil {
			return err
		}
		observation, err = decodeExecutionReport(raw, command)
		if err != nil {
			return err
		}
		p.incoming.Store(false)
		if err := p.connection.idle(); err != nil {
			return err
		}
		if phase == FinishExecution {
			p.terminal = command
		}
		return nil
	})
	if resultErr != nil {
		return ExecutionObservation{}, p.fail(errors.Join(resultErr, phaseCtx.Err()))
	}
	if phase == FinishExecution {
		p.finished = true
	}
	return observation, nil
}

// finishHandlerScope revokes business authority before any final transport ack.
// Successful finish here means only that an explicit matching finish report was
// consumed. It deliberately does not inspect/accept any transaction result status.
func (s *ExecutionSession) finishHandlerScope() error {
	if s == nil || s.state == nil {
		return ErrRefused
	}
	p := s.state
	p.mu.Lock()
	defer p.mu.Unlock()
	if p.closed || !p.finished || p.failure != nil || p.owner.Err() != nil {
		return errors.Join(ErrRefused, p.failure, p.owner.Err())
	}
	p.closed = true
	p.closeErr = errors.Join(p.closeErr, p.authority.close())
	return p.closeErr
}

// Close aborts pending I/O via the original owner before waiting for its lifetime
// lock. Physical handles stay with ServeExecution until watcher/IO cleanup joins.
func (s *ExecutionSession) Close() error {
	if s == nil || s.state == nil {
		return nil
	}
	p := s.state
	if p.cancel != nil {
		p.cancel()
	}
	p.mu.Lock()
	defer p.mu.Unlock()
	if !p.closed {
		p.closed = true
		p.incoming.Store(false)
		p.closeErr = errors.Join(p.closeErr, p.authority.close())
	}
	return p.closeErr
}
func (*ExecutionSession) MarshalJSON() ([]byte, error) { return nil, ErrRefused }
func (*ExecutionSession) UnmarshalJSON([]byte) error   { return ErrRefused }

// Process-local replay defense is bounded and never evicts failures into reuse.
// The host must additionally persist a one-use reservation before durable effects;
// this set alone cannot survive SCM restart and is never transaction authority.
type executionRequests struct{ seen map[string]bool }

func (r *executionRequests) reserve(peer Peer, q ExecutionRequest) error {
	if !validExecutionRequest(q) || peer.SID == "" || peer.ProcessID == 0 || peer.SessionID == 0 {
		return ErrRefused
	}
	if r.seen == nil {
		r.seen = map[string]bool{}
	}
	key := peer.SID + ":" + q.RequestID
	if r.seen[key] || len(r.seen) >= maxRequests {
		return ErrRefused
	}
	r.seen[key] = true
	return nil
}

// Revoke immediately at the handler boundary, before joining a possibly busy
// watcher or unwinding physical cleanup. Panic/error cannot leak a still-live
// capability to a goroutine that kept the handler context/session wrapper.
func invokeExecutionHandler(ctx context.Context, peer Peer, request ExecutionRequest, handler ExecutionHandler, session *ExecutionSession) (err error) {
	finished := false
	defer func() {
		if !finished {
			_ = session.Close()
		}
	}()
	if err = handler(ctx, peer, request, session); err != nil {
		return err
	}
	if err = session.finishHandlerScope(); err != nil {
		return err
	}
	finished = true
	return nil
}
