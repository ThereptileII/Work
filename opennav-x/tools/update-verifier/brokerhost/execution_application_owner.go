package brokerhost

import (
	"context"
	"errors"

	"example.com/opennav-update-verifier/brokerregistration"
)

var errExecutionApplicationOwner = errors.New("original ordinary application custody unavailable or uncertain")
var errExecutionApplicationLive = errors.New("original ordinary application is still live")

const maxExecutionApplicationOwners = 64

type executionApplicationState uint8

const (
	executionApplicationUnknown executionApplicationState = iota
	executionApplicationLive
	executionApplicationExited
)

// Private native process/token custody, transferred from the corroborated birth
// owner. Neither its provenance nor an observed exit grants cold-profile,
// selection, maintenance, hardware, or process-signalling authority.
type executionApplicationOwner interface {
	clientSID() string
	generation() string
	observe(context.Context) (executionApplicationState, error)
	Close() error
}

type executionApplicationEntry struct {
	generation string
	owner      executionApplicationOwner
	cleanupErr error
}

// All access, including native observations and cleanup, requires engine.busy.
// An entry without an owner is a consumed/uncertain creator, not proof of absence.
// There is no serialization or PID recovery after the service loses custody.
type executionApplicationOwners struct {
	entries  map[string]*executionApplicationEntry
	closed   bool
	closeErr error
}

type executionApplicationStart struct {
	owners *executionApplicationOwners
	sid    string
	entry  *executionApplicationEntry
}

func (owners *executionApplicationOwners) reserve(ctx context.Context, sid, generation string) (*executionApplicationStart, error) {
	if owners == nil || owners.closed || ctx == nil || ctx.Err() != nil || !brokerregistration.IsCanonicalUserSID(sid) || !requestPattern.MatchString(generation) {
		return nil, errors.Join(errExecutionApplicationOwner, contextError(ctx))
	}
	if _, exists := owners.entries[sid]; exists {
		if err := owners.release(ctx, sid); err != nil {
			return nil, err
		}
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if len(owners.entries) >= maxExecutionApplicationOwners {
		return nil, errExecutionApplicationOwner
	}
	if owners.entries == nil {
		owners.entries = make(map[string]*executionApplicationEntry)
	}
	entry := &executionApplicationEntry{generation: generation}
	owners.entries[sid] = entry
	return &executionApplicationStart{owners: owners, sid: sid, entry: entry}, nil
}

// Ownership transfers only on nil. The caller must drain a rejected owner.
func (start *executionApplicationStart) retain(ctx context.Context, owner executionApplicationOwner) error {
	if start == nil || start.owners == nil || start.entry == nil || start.owners.closed || ctx == nil || ctx.Err() != nil || owner == nil || start.owners.entries[start.sid] != start.entry || start.entry.owner != nil || start.entry.cleanupErr != nil || owner.clientSID() != start.sid || owner.generation() != start.entry.generation {
		return errors.Join(errExecutionApplicationOwner, contextError(ctx))
	}
	start.entry.owner = owner
	return nil
}

func (start *executionApplicationStart) finish(consumed bool) {
	if start == nil || start.owners == nil || start.entry == nil || start.owners.entries[start.sid] != start.entry || start.entry.owner != nil {
		return
	}
	if !consumed {
		delete(start.owners.entries, start.sid)
	}
	// A possibly created child without transferred original custody stays unknown.
}

func (owners *executionApplicationOwners) observe(ctx context.Context, sid string) (executionApplicationState, error) {
	if owners == nil || owners.closed || ctx == nil || ctx.Err() != nil || !brokerregistration.IsCanonicalUserSID(sid) {
		return executionApplicationUnknown, errors.Join(errExecutionApplicationOwner, contextError(ctx))
	}
	entry := owners.entries[sid]
	if entry == nil || entry.owner == nil {
		return executionApplicationUnknown, errExecutionApplicationOwner
	}
	if entry.cleanupErr != nil {
		return executionApplicationUnknown, errors.Join(errExecutionApplicationOwner, entry.cleanupErr)
	}
	state, err := entry.owner.observe(ctx)
	if err != nil || ctx.Err() != nil {
		return executionApplicationUnknown, errors.Join(errExecutionApplicationOwner, err, ctx.Err())
	}
	switch state {
	case executionApplicationLive:
		return state, errExecutionApplicationLive
	case executionApplicationExited:
		// Keep the exact owner until maintenance has consumed this proof, or a new
		// start drains its exited predecessor. Observation alone does not release it.
		return state, nil
	default:
		return executionApplicationUnknown, errExecutionApplicationOwner
	}
}

// Release requires a fresh observation of the same retained original process.
// A cleanup failure leaves a refusal tombstone rather than permission to retry.
func (owners *executionApplicationOwners) release(ctx context.Context, sid string) error {
	state, err := owners.observe(ctx, sid)
	if err != nil || state != executionApplicationExited {
		return errors.Join(errExecutionApplicationOwner, err)
	}
	entry := owners.entries[sid]
	entry.cleanupErr = entry.owner.Close()
	if entry.cleanupErr != nil {
		return errors.Join(errExecutionApplicationOwner, entry.cleanupErr)
	}
	delete(owners.entries, sid)
	return nil
}

func (owners *executionApplicationOwners) close() error {
	if owners == nil {
		return nil
	}
	if owners.closed {
		return owners.closeErr
	}
	owners.closed = true
	for sid, entry := range owners.entries {
		if entry.owner != nil {
			owners.closeErr = errors.Join(owners.closeErr, entry.owner.Close())
		}
		owners.closeErr = errors.Join(owners.closeErr, entry.cleanupErr)
		delete(owners.entries, sid)
	}
	return owners.closeErr
}

// Caller must already hold e.busy. Missing custody (including service restart)
// refuses; exit describes only this exact original process, never a cold profile.
func (e *engine) observeApplicationOwnerLocked(ctx context.Context, sid string) (executionApplicationOwner, executionApplicationState, error) {
	if e == nil {
		return nil, executionApplicationUnknown, errExecutionApplicationOwner
	}
	state, err := e.applications.observe(ctx, sid)
	entry := e.applications.entries[sid]
	if entry == nil {
		return nil, state, err
	}
	return entry.owner, state, err
}

// Caller holds e.busy through observation, maintenance's use of that proof, and
// explicit release. This does not acquire cold-profile or selection authority.
func (e *engine) releaseApplicationOwnerLocked(ctx context.Context, sid string) error {
	if e == nil {
		return errExecutionApplicationOwner
	}
	return e.applications.release(ctx, sid)
}

// Shutdown must first drain handlers. TryLock refuses concurrent release; Close
// only releases original query handles and leaves every ordinary app alive.
func (e *engine) closeApplicationOwners() error {
	if e == nil || !e.busy.TryLock() {
		return errExecutionApplicationOwner
	}
	defer e.busy.Unlock()
	return e.applications.close()
}
