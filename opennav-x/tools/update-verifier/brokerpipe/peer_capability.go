package brokerpipe

import (
	"context"
	"errors"
	"example.com/opennav-update-verifier/brokerstore"
	"sync"
)

// ErrPeerRevoked means the native connection capability is permanently inert.
// Reopening a pipe or repeating public Peer fields cannot restore it.
var ErrPeerRevoked = errors.New("authenticated pipe peer capability revoked")

// PeerIdentity is native-derived evidence within a live WithPeer use. It cannot
// establish authority independently of its originating PeerCapability.
type PeerIdentity struct {
	Peer
	ProcessCreation uint64
	LogonIDHigh     int32
	LogonIDLow      uint32
}

func (PeerIdentity) MarshalJSON() ([]byte, error) { return nil, ErrRefused }
func (*PeerIdentity) UnmarshalJSON([]byte) error  { return ErrRefused }

type peerContextKey struct{}

// Only the native serving boundary installs a backend. Test fakes remain private
// to this package; there is no public constructor or caller-supplied checker.
type peerCapabilityBackend interface{ check() error }

// PeerCapability binds one live native pipe, retained process creation and token
// logon identity. It is not serializable and is revoked when its handler returns,
// its operation ends, or a native check fails. The serving connection owns all
// physical handles and joins its watcher before closing them.
//
// This authenticates a caller/session, not its executable or update permission.
// Protected registration, selected generation/image binding, explicit user
// consent and launch/health authority remain independent execution preconditions.
type PeerCapability struct{ state *peerCapabilityState }

// A copied public wrapper shares the same revocation state; copying a mutex or
// native backend into a second independently live authority is impossible.
type peerCapabilityState struct {
	mu       sync.Mutex
	owner    context.Context
	identity PeerIdentity
	backend  peerCapabilityBackend
	image    *brokerstore.GenerationExecutorImage
	failure  error
	closed   bool
}

func withPeerCapability(ctx context.Context, identity PeerIdentity, backend peerCapabilityBackend) (context.Context, *PeerCapability) {
	capability := &PeerCapability{state: &peerCapabilityState{owner: ctx, identity: identity, backend: backend}}
	return context.WithValue(ctx, peerContextKey{}, capability), capability
}

// AuthorizePeer requires the actual handler context and exact public peer
// metadata. Peer or context JSON, context.Background and zero-value capabilities
// cannot authorize anything. Derived contexts may shorten the native lifetime,
// never extend it. Each execution boundary must revalidate the returned owner.
func AuthorizePeer(ctx context.Context, expected Peer) (*PeerCapability, error) {
	if ctx == nil {
		return nil, ErrRefused
	}
	capability, ok := ctx.Value(peerContextKey{}).(*PeerCapability)
	if !ok || capability == nil || capability.state == nil {
		return nil, ErrRefused
	}
	capability.state.mu.Lock()
	defer capability.state.mu.Unlock()
	if capability.state.identity.Peer != expected {
		return nil, ErrRefused
	}
	if err := capability.state.checkLocked(ctx); err != nil {
		return nil, err
	}
	return capability, nil
}

func (p *peerCapabilityState) checkLocked(ctx context.Context) error {
	if p.closed || p.failure != nil {
		return errors.Join(ErrPeerRevoked, p.failure)
	}
	if p.owner == nil || p.backend == nil || p.identity.SID == "" || p.identity.SessionID == 0 || p.identity.ProcessID == 0 || p.identity.ProcessCreation == 0 || (p.identity.LogonIDHigh == 0 && p.identity.LogonIDLow == 0) {
		return ErrRefused
	}
	if err := p.owner.Err(); err != nil {
		p.failure = errors.Join(ErrPeerRevoked, err)
		return p.failure
	}
	if ctx == nil {
		return ErrRefused
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	if err := p.backend.check(); err != nil {
		p.failure = errors.Join(ErrPeerRevoked, err)
		return p.failure
	}
	if p.image != nil {
		if err := p.image.MatchesProcess(ctx, p.identity.ProcessID, p.identity.ProcessCreation); err != nil {
			p.failure = errors.Join(ErrPeerRevoked, err)
			return p.failure
		}
		// Image validation may inspect a large retained generation. Recheck the
		// original pipe/process/token after it, not only before opening the image.
		if err := p.backend.check(); err != nil {
			p.failure = errors.Join(ErrPeerRevoked, err)
			return p.failure
		}
	}
	// Cancellation may race a native check. Never grant a new use after it.
	if err := p.owner.Err(); err != nil {
		p.failure = errors.Join(ErrPeerRevoked, err)
		return p.failure
	}
	return ctx.Err()
}

// Validate checks retained native identity and connection state afresh. A failed
// native identity check permanently revokes this owner, even if later public
// identifiers or connection state appear to match again.
func (p *PeerCapability) Validate(ctx context.Context) error {
	if p == nil || p.state == nil {
		return ErrRefused
	}
	p.state.mu.Lock()
	defer p.state.mu.Unlock()
	return p.state.checkLocked(ctx)
}

// WithPeer runs one bounded session boundary while revocation/Close is excluded,
// checking actual identity before and after it. The callback must obey ctx and
// must not call this capability's methods (they use the same lifetime lock).
// Public Peer fields passed to it remain informational outside this live use.
// A callback with durable effects owns their commit/uncertainty semantics: a
// later check failure does not roll back or authorize automatic repetition.
// This API grants no executable/image or launch authority.
func (p *PeerCapability) WithPeer(ctx context.Context, use func(PeerIdentity) error) error {
	return p.withPeer(ctx, use, false)
}

// WithExecutor requires a successful live BindExecutorImage before entering a
// bounded session boundary. Native caller and exact protected launcher image are
// revalidated before/after use. Selected-chain state, consent and the callback's
// actual launch/commit/health checks remain independent obligations. Like WithPeer,
// callbacks must obey ctx and must not reenter this capability.
func (p *PeerCapability) WithExecutor(ctx context.Context, use func(PeerIdentity) error) error {
	return p.withPeer(ctx, use, true)
}

func (p *PeerCapability) withPeer(ctx context.Context, use func(PeerIdentity) error, requireImage bool) error {
	if p == nil || p.state == nil || use == nil {
		return ErrRefused
	}
	p.state.mu.Lock()
	defer p.state.mu.Unlock()
	if requireImage && p.state.image == nil {
		return ErrRefused
	}
	if err := p.state.checkLocked(ctx); err != nil {
		return err
	}
	useErr := use(p.state.identity)
	return errors.Join(useErr, p.state.checkLocked(ctx))
}

// Close permanently revokes the view. The native serving owner retains physical
// handles until its joined connection cleanup; callers cannot close/reuse them.
func (p *PeerCapability) Close() error {
	if p == nil || p.state == nil {
		return nil
	}
	p.state.mu.Lock()
	defer p.state.mu.Unlock()
	p.state.closed = true
	return nil
}
func (*PeerCapability) MarshalJSON() ([]byte, error) { return nil, ErrRefused }
func (*PeerCapability) UnmarshalJSON([]byte) error   { return ErrRefused }

// invokeAuthenticatedHandler keeps revocation tied to the actual handler scope,
// including refusal or panic. Public protocol invocation itself remains unchanged.
func invokeAuthenticatedHandler(ctx context.Context, capability *PeerCapability, replay *requests, handler Handler, peer Peer, request Request) (Result, error) {
	defer capability.Close()
	if _, err := AuthorizePeer(ctx, peer); err != nil {
		return Result{}, err
	}
	return replay.invoke(ctx, handler, peer, request)
}

// BindExecutorImage binds this caller once to the sealed live owner's fixed
// app/skager-start.exe member. No path, hash, PID declaration or replaceable
// image-auth callback is accepted. Validate/WithPeer subsequently check the same
// image owner on every use. The caller retains/owns image through this capability's
// lifetime; closing it revokes later use. Binding proves image identity only, not
// selection, update permission, consent or launch/health authority.
// Lock order is peer capability then generation/store owner: callers must not
// already hold a store/generation lifetime lock when entering this API.
func (p *PeerCapability) BindExecutorImage(ctx context.Context, image *brokerstore.GenerationExecutorImage) error {
	if p == nil || p.state == nil || image == nil {
		return ErrRefused
	}
	p.state.mu.Lock()
	defer p.state.mu.Unlock()
	if err := p.state.checkLocked(ctx); err != nil {
		return err
	}
	if p.state.image != nil {
		return ErrRefused
	}
	p.state.image = image // Consume the one attempt; mismatch permanently revokes it.
	return p.state.checkLocked(ctx)
}
