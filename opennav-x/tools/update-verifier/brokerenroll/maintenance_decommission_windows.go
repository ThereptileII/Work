//go:build windows

package brokerenroll

import (
	"context"
	"errors"
	"runtime"

	"golang.org/x/sys/windows"
)

func (p *nativeMaintenanceProcess) birth() uint64 {
	return uint64(p.created.HighDateTime)<<32 | uint64(p.created.LowDateTime)
}

// disable changes only the start type of the original retained SCM service.
// The owner consumes the attempt before entering this method: an API failure
// cannot be interpreted as proof that the change did not take effect.
func (n *nativeMaintenanceBackend) disable(ctx context.Context, pid uint32) error {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if !n.decommission || n.bound == nil || n.bound.pid != pid {
		return errors.New("original decommission service and broker required")
	}
	if err := n.check(ctx); err != nil {
		return err
	}
	if err := n.bound.live(); err != nil {
		return err
	}
	state, err := n.status()
	if err != nil || !maintenanceRunning(state, pid) {
		return errors.Join(errors.New("original broker changed before disable"), err)
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	if err := windows.ChangeServiceConfig(n.service.service, windows.SERVICE_NO_CHANGE,
		windows.SERVICE_DISABLED, windows.SERVICE_NO_CHANGE, nil, nil, nil, nil, nil, nil, nil); err != nil {
		return err
	}
	n.service.expectedStart = windows.SERVICE_DISABLED
	if err := n.check(ctx); err != nil {
		return err
	}
	state, err = n.status()
	if err != nil || !maintenanceRunning(state, pid) {
		return errors.Join(errors.New("original broker changed after disable"), err)
	}
	return n.bound.live()
}

// markRemoval marks only the already-disabled, cleanly stopped original service.
// DeleteService success is not proof that the SCM entry has disappeared while
// handles exist; callers must report marked-for-removal, never full uninstall.
func (n *nativeMaintenanceBackend) markRemoval(ctx context.Context) error {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if !n.decommission || n.bound == nil {
		return errors.New("original decommission service required")
	}
	if err := n.check(ctx); err != nil {
		return err
	}
	state, err := n.status()
	if err != nil || !serviceCompleted(state) {
		return errors.Join(errors.New("original service must be stopped before deletion mark"), err)
	}
	exited, code, err := n.bound.exit()
	if err != nil || !exited || code != 0 {
		return errors.Join(errors.New("original broker zero exit required before deletion mark"), err)
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	return windows.DeleteService(n.service.service)
}
