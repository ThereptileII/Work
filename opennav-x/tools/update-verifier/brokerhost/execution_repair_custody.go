package brokerhost

import (
	"context"
	"errors"

	"example.com/opennav-update-verifier/brokerregistration"
)

var errExecutionRepairCustody = errors.New("original Repair application/profile custody unavailable or uncertain")

// Only the Windows factory can supply production frozen-profile custody. This
// private seam permits policy fault tests; it is not serialized/native authority.
type repairFrozenProfile interface {
	validateFrozen(context.Context) error
}

// Caller holds engine.busy through capture, every validation, finish and close.
// This Repair-only owner BORROWS the full frozen profile. It never inserts an
// application keeper, infers an original process exit, launches, or closes that
// borrowed profile. Manual Rollback and Uninstall retain their existing rules.
type executionRepairApplicationCustody struct {
	self             *executionRepairApplicationCustody
	ctx              context.Context
	engine           *engine
	sid, generation  string
	entry            *executionApplicationEntry
	original         executionApplicationOwner
	profile          repairFrozenProfile
	finished, closed bool
	failure          error
}

func captureRepairApplicationCustodyWith(ctx context.Context, e *engine, sid, generation string, profile repairFrozenProfile) (*executionRepairApplicationCustody, error) {
	if ctx == nil || ctx.Err() != nil || e == nil || e.applications.closed ||
		!brokerregistration.IsCanonicalUserSID(sid) || !requestPattern.MatchString(generation) || profile == nil {
		return nil, errExecutionRepairCustody
	}
	c := &executionRepairApplicationCustody{ctx: ctx, engine: e, sid: sid, generation: generation, profile: profile}
	c.self = c
	entry, exists := e.applications.entries[sid]
	if exists {
		// A consumed creator, nil owner or cleanup tombstone is uncertainty,
		// never the cold branch. Capture of the profile cannot repair that proof.
		if entry == nil || entry.owner == nil || entry.cleanupErr != nil || entry.generation != generation ||
			entry.owner.clientSID() != sid || entry.owner.generation() != generation {
			return nil, errExecutionRepairCustody
		}
		c.entry, c.original = entry, entry.owner
	}
	if err := c.validate(ctx); err != nil {
		return nil, err
	}
	return c, nil
}

func (c *executionRepairApplicationCustody) checkApplication(ctx context.Context) error {
	if c.engine.applications.closed {
		return errExecutionRepairCustody
	}
	entry, exists := c.engine.applications.entries[c.sid]
	if c.entry == nil {
		if exists { // Even a present nil map value revokes the cold branch.
			return errExecutionRepairCustody
		}
		return nil
	}
	if !exists || entry != c.entry || entry.owner != c.original || entry.cleanupErr != nil ||
		entry.generation != c.generation || c.original.clientSID() != c.sid || c.original.generation() != c.generation {
		return errExecutionRepairCustody
	}
	owner, state, err := c.engine.observeApplicationOwnerLocked(ctx, c.sid)
	if err != nil || owner != c.original || state != executionApplicationExited {
		return errors.Join(errExecutionRepairCustody, err)
	}
	return nil
}

func (c *executionRepairApplicationCustody) validate(ctx context.Context) (resultErr error) {
	if c == nil || c.self != c || c.closed || c.finished {
		return errExecutionRepairCustody
	}
	if c.failure != nil {
		return c.failure
	}
	defer func() {
		if resultErr != nil {
			c.failure = errors.Join(errExecutionRepairCustody, resultErr)
			resultErr = c.failure
		}
	}()
	if c.engine == nil || c.profile == nil || ctx == nil || c.ctx == nil {
		return errExecutionRepairCustody
	}
	if err := errors.Join(ctx.Err(), c.ctx.Err(), c.checkApplication(ctx)); err != nil {
		return err
	}
	// Entry absence alone grants nothing. Native validation retains all existing
	// file deny-write/delete handles and the recursive namespace watch. Namespace
	// changes are detected/refused, not physically prevented by directory handles.
	if err := c.profile.validateFrozen(ctx); err != nil {
		return err
	}
	return errors.Join(c.checkApplication(ctx), ctx.Err(), c.ctx.Err())
}

func (c *executionRepairApplicationCustody) finish(ctx context.Context) error {
	if err := c.validate(ctx); err != nil {
		return err
	}
	// Consume only a genuinely retained warm owner, after committed readback.
	// Cold repair never creates, releases or reopens an application keeper.
	c.finished = true
	if c.original != nil {
		if err := c.engine.releaseApplicationOwnerLocked(ctx, c.sid); err != nil {
			c.failure = errors.Join(errExecutionRepairCustody, err)
			return c.failure
		}
	}
	return nil
}

func (c *executionRepairApplicationCustody) close() error {
	if c == nil {
		return nil
	}
	if c.self != c {
		return errExecutionRepairCustody
	}
	c.closed = true
	return c.failure // No borrowed native profile or engine keeper is closed here.
}
