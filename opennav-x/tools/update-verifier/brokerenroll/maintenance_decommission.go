package brokerenroll

import (
	"context"
	"errors"
)

// DecommissionResult reports the bounded SCM effect after every retained owner
// closes successfully. Windows DeleteService marks a service for removal; it
// does not prove the entry is already gone. This result says nothing about
// profiles, files, shortcuts, registration, floors or full uninstall.
type DecommissionResult struct {
	Service string
}

const ServiceMarkedForRemoval = "marked-for-removal"

// The native public binding accepts only brokerpipe's sealed witness. This
// private seam exists for fault tests; a caller cannot implement a witness and
// pass it through the public API or reconstruct one from serialized bytes.
type decommissionFence interface {
	Check(context.Context, uint32, uint64) error
	Close() error
}

type decommissionBackend interface {
	maintenanceBackend
	disable(context.Context, uint32) error
	markRemoval(context.Context) error
}

type maintenanceProcessBirth interface {
	birth() uint64
}

// OpenDecommissionMaintenance obtains the original owner with CHANGE_CONFIG
// and DELETE service rights. These rights are never requested by OpenMaintenance.
// No SCM change occurs until a sealed original broker fence is bound and checked.
func OpenDecommissionMaintenance(ctx context.Context, req MaintenanceRequest) (*MaintenanceOwner, error) {
	return openMaintenanceModeWith(ctx, req, maintenanceOperations{
		authenticate:     maintenanceOperatorNative,
		openDecommission: openDecommissionNative,
		poll:             servicePoll,
		limit:            maintenanceStopLimit,
	}, true)
}

func bindDecommissionFenceWith(m *MaintenanceOwner, ctx context.Context, fence decommissionFence, clientSID string) (resultErr error) {
	if m == nil || m.self != m {
		return errors.New("original decommission owner required")
	}
	m.mu.Lock()
	defer m.mu.Unlock()
	if !m.decommission || m.closed || m.failed || m.fence != nil || m.disableAttempted || fence == nil {
		return errors.New("fresh original decommission fence binding required")
	}
	// Ownership transfers before validation so a rejected witness cannot leave
	// a native peer handle outside the owner's deterministic close path.
	m.fence = fence
	defer func() {
		if resultErr != nil {
			m.failed = true
		}
	}()
	if clientSID != m.authority.client.ClientSID {
		return errors.New("broker fence client differs from approved original request")
	}
	if err := m.check(ctx); err != nil {
		return err
	}
	birth, ok := m.process.(maintenanceProcessBirth)
	if !ok || birth.birth() == 0 {
		return errors.New("original retained broker creation time required")
	}
	if err := m.fence.Check(ctx, m.pid, birth.birth()); err != nil {
		return err
	}
	return m.check(ctx)
}

// Decommission consumes one disable, one clean shutdown, and one deletion mark
// in that order on the same retained SCM/process owner. Any uncertain effect
// permanently revokes this owner; no retry, repair or cleanup is attempted.
func (m *MaintenanceOwner) Decommission(ctx context.Context) (result DecommissionResult, resultErr error) {
	if m == nil || m.self != m {
		return result, errors.New("original decommission owner required")
	}
	m.mu.Lock()
	defer m.mu.Unlock()
	if !m.decommission || m.closed || m.failed || m.fence == nil || m.disableAttempted || m.stopAttempted || m.deleteAttempted {
		return result, errors.New("fresh bound decommission attempt required")
	}
	defer func() {
		if resultErr != nil {
			m.failed = true
			result = DecommissionResult{}
			if m.dispatched {
				resultErr = errors.Join(ErrMaintenanceUncertain, resultErr)
			}
		}
	}()
	backend, ok := m.backend.(decommissionBackend)
	if !ok {
		return result, errors.New("original native decommission service required")
	}
	birth, ok := m.process.(maintenanceProcessBirth)
	if !ok || birth.birth() == 0 {
		return result, errors.New("original retained broker creation time required")
	}
	if err := m.check(ctx); err != nil {
		return result, err
	}
	if err := m.fence.Check(ctx, m.pid, birth.birth()); err != nil {
		return result, err
	}
	// Set dispatched before the first native call. A failure may occur after the
	// SCM applied the start-type change; it is never safe to infer no effect.
	m.disableAttempted, m.dispatched = true, true
	if err := backend.disable(ctx, m.pid); err != nil {
		return result, err
	}
	if err := m.check(ctx); err != nil {
		return result, err
	}
	if err := m.fence.Check(ctx, m.pid, birth.birth()); err != nil {
		return result, err
	}
	if err := m.quiesceLocked(ctx); err != nil {
		return result, err
	}
	// The broker is now cleanly exited. The original retained process handle,
	// token, image, service and disabled configuration remain under m.check.
	if err := m.check(ctx); err != nil {
		return result, err
	}
	m.deleteAttempted = true
	if err := backend.markRemoval(ctx); err != nil {
		return result, err
	}
	if err := m.closeLocked(); err != nil {
		return result, err
	}
	return DecommissionResult{Service: ServiceMarkedForRemoval}, nil
}
