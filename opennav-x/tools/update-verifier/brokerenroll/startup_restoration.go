package brokerenroll

import (
	"context"
	"encoding/json"
	"errors"
)

// ErrStartupRestorationUncertain means a retained shortcut rename was attempted.
// Preserve every menu/recovery object; never retry, adopt or clean up the partial
// transaction. This error does not authorize profile or application mutation.
var ErrStartupRestorationUncertain = errors.New("original shortcut restoration uncertain; preserve recovery")

type startupRestorationBackend interface {
	check(context.Context) error
	restore(context.Context, func(context.Context) error) error
	close() error
}

type startupRestorationOpener func(context.Context, maintenanceAuthority, startupBindingRecord, []byte) (startupRestorationBackend, error)

// BindStartupRestoration binds independently approved ORIGINAL publication
// evidence to this original decommission owner and its already-bound live broker
// witness. It only opens/checks native owners. Decommission performs the moves
// before disabling or stopping the broker, checking the same live witness around
// every move. The witness proves terminal fencing and the broker's earlier cold
// observation; this operation does not claim continuous profile custody.
//
// The approved record alone is historical comparison data. No caller-selected
// path, replacement shortcut, serialized native owner or adjacent hash can supply
// restoration authority. A refused binding revokes this decommission owner.
func (m *MaintenanceOwner) BindStartupRestoration(ctx context.Context, approved *ApprovedStartupPublication) error {
	return bindStartupRestorationWith(m, ctx, approved, openStartupRestorationNative)
}

func bindStartupRestorationWith(m *MaintenanceOwner, ctx context.Context, approved *ApprovedStartupPublication, open startupRestorationOpener) (resultErr error) {
	if m == nil || m.self != m {
		return errors.New("original decommission owner required")
	}
	m.mu.Lock()
	defer m.mu.Unlock()
	if !m.decommission || m.closed || m.failed || m.disableAttempted || m.stopAttempted || m.startupBound {
		return errors.New("fresh fenced original shortcut restoration binding required")
	}
	// Consume before validating/opening: a failed binding cannot be omitted to
	// fall back to the service-only path on this same owner.
	m.startupBound = true
	defer func() {
		if resultErr != nil {
			m.failed = true
		}
	}()
	if m.fence == nil {
		return errors.New("already-bound original live broker witness required")
	}
	if approved == nil || open == nil {
		return errors.New("independently approved original shortcut publication required")
	}
	parsed, err := ParseApprovedStartupPublication(approved.OriginalBytes(), approved.SHA256(), m.authority.client.OriginalBytes(), m.authority.client.RequestSHA256)
	if err != nil {
		return err
	}
	var record startupBindingRecord
	if err := json.Unmarshal(parsed.OriginalBytes(), &record); err != nil {
		return err
	}
	if err := m.startupGuard(ctx); err != nil {
		return err
	}
	m.startup, err = open(ctx, m.authority, record, parsed.OriginalBytes())
	if err != nil || m.startup == nil {
		return errors.Join(errors.New("original native shortcut owners required"), err)
	}
	if err := m.startup.check(ctx); err != nil {
		return err
	}
	return m.startupGuard(ctx)
}

// m.mu is held; the witness must remain live, so this guard is used only before
// disable/STOP. Its sealed native provenance is established by BindDecommission.
func (m *MaintenanceOwner) startupGuard(ctx context.Context) error {
	if err := m.check(ctx); err != nil {
		return err
	}
	birth, ok := m.process.(maintenanceProcessBirth)
	if !ok || birth.birth() == 0 || m.fence == nil {
		return errors.New("same original live broker witness required")
	}
	return m.fence.Check(ctx, m.pid, birth.birth())
}

// Every destination is checked before any move. Native restore rechecks complete
// custody and the live witness before AND after each non-replacing handle move.
func startupRestoreSequence(ctx context.Context, original bool, preflight func() error, move func(int) error) error {
	if ctx == nil || preflight == nil || move == nil {
		return errors.New("original native restoration sequence required")
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	if err := preflight(); err != nil {
		return err
	}
	count := 3
	if original {
		count++
	}
	for step := 0; step < count; step++ {
		if err := ctx.Err(); err != nil {
			return errors.Join(ErrStartupRestorationUncertain, err)
		}
		if err := move(step); err != nil {
			return errors.Join(ErrStartupRestorationUncertain, err)
		}
	}
	return nil
}
