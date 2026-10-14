package brokerenroll

import (
	"context"
	"errors"
)

// InstalledUninstallResult is an observation after every original owner drains,
// never a reusable cleanup handoff. Recovery, modified and unapproved members
// remain; DeleteService reports a removal mark rather than immediate absence.
type InstalledUninstallResult struct {
	Service, Integration, Registration, Recovery string
}

type installedUninstallBackend interface {
	check(context.Context) error
	cleanup(context.Context, func(context.Context) error) error
	ownsStartupLink(target, arguments, working string) bool
	close() error
}

type installedStartupRestorationBackend interface {
	prepareInstalled(context.Context, func(target, arguments, working string) bool) error
	retireInstalled(context.Context, func(context.Context) error) error
}

type installedUninstallOpener func(context.Context, maintenanceAuthority, string) (installedUninstallBackend, error)

// PrepareInstalledUninstall retains the actual original user's fixed installed
// integration before fencing. artifactDirectory supplies only the original
// approved baseline ZIP; it cannot select a cleanup target. Original request
// approval, archive contents, native custody and the later live terminal broker
// witness are independent mandatory inputs. Startup must already be prepared.
func (m *MaintenanceOwner) PrepareInstalledUninstall(ctx context.Context, artifactDirectory string) error {
	return prepareInstalledUninstallWith(m, ctx, artifactDirectory, openInstalledUninstallNative)
}

func prepareInstalledUninstallWith(m *MaintenanceOwner, ctx context.Context, artifactDirectory string, open installedUninstallOpener) (resultErr error) {
	if m == nil || m.self != m {
		return errors.New("original installed decommission owner required")
	}
	m.mu.Lock()
	defer m.mu.Unlock()
	if !m.decommission || m.closed || m.failed || m.installedAttempted || m.fence != nil || m.disableAttempted || m.stopAttempted || !m.startupPrepared || m.startupDrainAttempted {
		return errors.New("fresh original prepared startup and installed owner required")
	}
	m.installedAttempted = true // A refused preparation cannot fall back to SCM-only removal.
	defer func() {
		if resultErr != nil {
			m.failed = true
		}
	}()
	startup, ok := m.startup.(installedStartupRestorationBackend)
	if !ok || open == nil || artifactDirectory == "" {
		return errors.New("native installed startup join and approved baseline input required")
	}
	if err := m.check(ctx); err != nil {
		return err
	}
	var err error
	m.installed, err = open(ctx, m.authority, artifactDirectory)
	if err != nil || m.installed == nil {
		return errors.Join(errors.New("original native installed integration custody required"), err)
	}
	if err := errors.Join(m.installed.check(ctx), m.startup.check(ctx), m.check(ctx)); err != nil {
		return err
	}
	if err := startup.prepareInstalled(ctx, m.installed.ownsStartupLink); err != nil {
		return err
	}
	return errors.Join(m.installed.check(ctx), m.startup.check(ctx), m.check(ctx))
}

// DecommissionInstalled is the only terminal path after installed preparation.
// It consumes the same live final broker witness, retires the restored original
// integration link, disables/stops and proves original broker exit, then removes
// only pre-retained independently authenticated installed members and the same
// registry key. It never calls old Lifecycle or reconstructs authority from JSON.
func (m *MaintenanceOwner) DecommissionInstalled(ctx context.Context) (InstalledUninstallResult, error) {
	result, err := m.decommissionMode(ctx, true)
	if err != nil {
		return InstalledUninstallResult{}, err
	}
	return InstalledUninstallResult{Service: result.Service, Integration: "verified-owned-members-removed", Registration: "unregistered", Recovery: "preserved"}, nil
}

// Called only with m.mu held AFTER the original final witness was validated,
// disable/STOP dispatched once, and quiesceLocked proved this original process
// exited zero. An expired live-process witness must never be checked after exit.
func (m *MaintenanceOwner) installedPostStopGuard(ctx context.Context) error {
	if !m.installedAttempted || !m.installedBound || m.installed == nil || m.startup == nil || !m.disableAttempted || !m.stopAttempted || !m.quiescent || m.deleteAttempted {
		return errors.New("same original installed post-stop custody required")
	}
	return errors.Join(m.check(ctx), m.startup.check(ctx), m.installed.check(ctx))
}
