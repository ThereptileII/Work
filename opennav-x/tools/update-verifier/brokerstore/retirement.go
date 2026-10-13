package brokerstore

import (
	"context"
	"errors"
	"runtime"
	"sync"
)

// ErrRetirementUncertain means the terminal marker or one of the original
// objects may already have moved. Neither a retry nor adoption is safe.
var ErrRetirementUncertain = errors.New("original protected retirement uncertain; preserve all recovery objects")

type retirementStoreBackend interface {
	prepareRetirement(context.Context, DecommissionBinding) (retirementPreparationBackend, error)
}
type retirementPreparationBackend interface {
	check() error
	commit(context.Context) (retirementBackend, error)
	close() error
}
type retirementBackend interface {
	check() error
	fenceSHA256() (string, error)
	evidenceSHA256() (string, error)
	close() error
}

// RetirementPreparation is a non-serializable original-object handoff. Prepare
// runs while the host's selected generation and journal readers still validate.
// It retains their original physical IDs and protected parents. The host must
// close those readers successfully before Commit acquires DELETE-capable handles
// matching these IDs. No terminal marker or namespace mutation occurs in Prepare.
type RetirementPreparation struct {
	self                      *RetirementPreparation
	mu                        sync.Mutex
	store                     *Store
	backend                   retirementPreparationBackend
	binding                   DecommissionBinding
	committed, closed, failed bool
	closeErr                  error
}

// PrepareRetiringDecommission is separate from service-only decommission. The
// original guard must validate the same selected host owner while it is still
// live. It runs outside Store locks and cannot manufacture native file custody.
func (s *Store) PrepareRetiringDecommission(ctx context.Context, binding DecommissionBinding, originalGuard func(context.Context) error) (result *RetirementPreparation, resultErr error) {
	if s == nil || ctx == nil || originalGuard == nil || !binding.valid() {
		return nil, errors.New("original protected retirement binding required")
	}
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	s.mu.Lock()
	if s.closed.Load() || s.decommissionAttempted {
		s.mu.Unlock()
		return nil, errors.New("original terminal attempt unavailable")
	}
	if _, ok := s.backend.(retirementStoreBackend); !ok {
		s.mu.Unlock()
		return nil, ErrNativeRequired
	}
	if err := s.backend.check(); err != nil {
		s.mu.Unlock()
		return nil, err
	}
	s.decommissionAttempted = true
	s.mu.Unlock()
	if err := errors.Join(ctx.Err(), originalGuard(ctx)); err != nil {
		return nil, err
	}
	s.mu.Lock()
	defer s.mu.Unlock()
	if s.closed.Load() || ctx.Err() != nil {
		return nil, ErrClosed
	}
	if err := s.backend.check(); err != nil {
		return nil, err
	}
	b, err := s.backend.(retirementStoreBackend).prepareRetirement(ctx, binding)
	if err != nil || b == nil {
		return nil, errors.Join(errors.New("original protected retirement owner unavailable"), err)
	}
	p := &RetirementPreparation{store: s, backend: b, binding: binding}
	p.self = p
	s.retirementPreparations = append(s.retirementPreparations, p)
	return p, nil
}

func (p *RetirementPreparation) Binding() DecommissionBinding {
	if p == nil || p.self != p {
		return DecommissionBinding{}
	}
	return p.binding
}
func (p *RetirementPreparation) Validate(ctx context.Context) error {
	if p == nil || p.self != p || ctx == nil {
		return ErrNativeRequired
	}
	p.mu.Lock()
	defer p.mu.Unlock()
	if p.closed || p.failed || p.committed || p.store.closed.Load() {
		return ErrClosed
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	if err := p.backend.check(); err != nil {
		p.failed = true
		return err
	}
	return nil
}

// Commit consumes one attempt after the selected host owner has closed. The
// post-selection guard may check stock/profile/operator/application custody but
// cannot call ordinary Store methods after the terminal marker is published.
// A native failure after publication leaves a durable started retirement record.
func (p *RetirementPreparation) Commit(ctx context.Context, postSelectionGuard func(context.Context) error) (result *Retirement, resultErr error) {
	if p == nil || p.self != p || ctx == nil || postSelectionGuard == nil {
		return nil, ErrNativeRequired
	}
	p.mu.Lock()
	defer p.mu.Unlock()
	if p.closed || p.failed || p.committed || p.store.closed.Load() {
		return nil, ErrClosed
	}
	p.committed = true
	if err := errors.Join(ctx.Err(), postSelectionGuard(ctx), p.backend.check()); err != nil {
		p.failed = true
		return nil, err
	}
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	b, err := p.backend.commit(ctx)
	if b != nil {
		r := &Retirement{store: p.store, backend: b, binding: p.binding, poisoned: err != nil}
		r.self = r
		p.store.mu.Lock()
		p.store.retirements = append(p.store.retirements, r)
		p.store.mu.Unlock()
		if err == nil {
			result = r
		}
	}
	if err != nil || b == nil {
		p.failed = true
		if err == nil {
			return nil, errors.New("original retirement backend returned no owner")
		}
		return nil, err
	}
	return result, nil
}

func (p *RetirementPreparation) Close() error {
	if p == nil {
		return nil
	}
	if p.self != p {
		return ErrNativeRequired
	}
	p.mu.Lock()
	defer p.mu.Unlock()
	if !p.closed {
		p.closed = true
		p.closeErr = p.backend.close()
	}
	return p.closeErr
}

// Retirement is the completed original broker-owned recovery and terminal
// record. Its hashes are comparison evidence only, never path or approval data.
type Retirement struct {
	self             *Retirement
	mu               sync.Mutex
	store            *Store
	backend          retirementBackend
	binding          DecommissionBinding
	poisoned, closed bool
	closeErr         error
}

func (r *Retirement) Binding() DecommissionBinding {
	if r == nil || r.self != r {
		return DecommissionBinding{}
	}
	return r.binding
}
func (r *Retirement) Validate(ctx context.Context) error {
	if r == nil || r.self != r || ctx == nil {
		return ErrNativeRequired
	}
	r.mu.Lock()
	defer r.mu.Unlock()
	if r.closed || r.poisoned || r.store.closed.Load() {
		return ErrClosed
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	if err := r.backend.check(); err != nil {
		r.poisoned = true
		return err
	}
	return nil
}
func (r *Retirement) hashes(ctx context.Context) (string, string, error) {
	if err := r.Validate(ctx); err != nil {
		return "", "", err
	}
	r.mu.Lock()
	defer r.mu.Unlock()
	fence, err := r.backend.fenceSHA256()
	if err != nil || !validHex(fence, 64) {
		r.poisoned = true
		return "", "", errors.Join(ErrRetirementUncertain, err)
	}
	evidence, err := r.backend.evidenceSHA256()
	if err != nil || !validHex(evidence, 64) {
		r.poisoned = true
		return "", "", errors.Join(ErrRetirementUncertain, err)
	}
	return fence, evidence, nil
}
func (r *Retirement) FenceSHA256(ctx context.Context) (string, error) {
	a, _, e := r.hashes(ctx)
	return a, e
}
func (r *Retirement) EvidenceSHA256(ctx context.Context) (string, error) {
	_, b, e := r.hashes(ctx)
	return b, e
}
func (r *Retirement) Close() error {
	if r == nil {
		return nil
	}
	if r.self != r {
		return ErrNativeRequired
	}
	r.mu.Lock()
	defer r.mu.Unlock()
	if !r.closed {
		r.closed = true
		r.closeErr = r.backend.close()
	}
	return r.closeErr
}
func (*Retirement) MarshalJSON() ([]byte, error) { return nil, ErrNativeRequired }
func (*Retirement) UnmarshalJSON([]byte) error   { return ErrNativeRequired }
