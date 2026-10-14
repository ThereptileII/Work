package brokerenroll

import (
	"bytes"
	"context"
	"errors"
	"sync"
	"time"

	"example.com/opennav-update-verifier/brokerpipe"
	"example.com/opennav-update-verifier/brokerregistration"
)

// ErrMaintenanceUncertain means original shortcut restoration or a single SCM
// effect may have taken effect, but completion/custody was not established.
// Never retry, restart, recreate enrollment or reset protected state in response.
var ErrMaintenanceUncertain = errors.New("original broker maintenance shutdown uncertain")

const maintenanceStopLimit = 60 * time.Second

// MaintenanceRequest is an independently authenticated operator handoff, not
// an application's declaration. Before OpenMaintenance the operator must
// authenticate this provisioner and its loader chain and separately approve the
// installed broker hash and original client request bytes/hash. UAC, a self-hash,
// adjacent JSON or the current user's SID do not establish those prerequisites.
// No caller-selected machine path, replacement executable or trust floor exists.
type MaintenanceRequest struct {
	ApprovedBrokerSHA256        string
	ClientRegistrationRequest   []byte
	ApprovedClientRequestSHA256 string
}

type maintenanceAuthority struct {
	brokerSHA string
	client    brokerregistration.Request
}

// The private seams describe native observations, never caller-supplied proof.
type maintenanceBackend interface {
	check(context.Context) error
	status() (serviceState, error)
	bindProcess(uint32) (maintenanceProcess, error)
	stop(uint32) error
	close() error
}
type maintenanceProcess interface {
	live() error
	exit() (exited bool, exitCode uint32, err error)
	close() error
}
type maintenanceOperations struct {
	authenticate     func(string) error
	open             func(context.Context, maintenanceAuthority) (maintenanceBackend, error)
	openDecommission func(context.Context, maintenanceAuthority) (maintenanceBackend, error)
	poll             func(context.Context) error
	limit            time.Duration
}

// MaintenanceOwner retains original SCM, process/token, protected image and
// approved request custody. It is not serializable mutation authority. A clean
// Quiesce proves only this broker drained and exited: it does not prove an
// ordinary application exited, its profile is cold, or armed/pending operations
// and protected history permit repair, rollback or uninstall. Those operations
// still need their own actual child/profile/history/store-lock handoff.
//
// Check is a bounded observation, not exclusion of another privileged operator
// or a subsequent SCM start. Unexpected restart permanently revokes this owner.
// No service database lock is claimed: it has no effect since Windows Vista.
// https://learn.microsoft.com/en-us/windows/win32/api/winsvc/nf-winsvc-lockservicedatabase
type MaintenanceOwner struct {
	self                               *MaintenanceOwner
	mu                                 sync.Mutex
	authority                          maintenanceAuthority
	op                                 maintenanceOperations
	backend                            maintenanceBackend
	process                            maintenanceProcess
	pid                                uint32
	stopAttempted                      bool
	dispatched                         bool
	quiescent                          bool
	failed                             bool
	closed                             bool
	closeErr                           error
	decommission                       bool
	disableAttempted                   bool
	deleteAttempted                    bool
	fence                              decommissionFence
	startupBound                       bool
	startupPrepared                    bool
	startupDrainAttempted              bool
	startupRestored, startupDrained    bool
	startupFenceSnapshot               *brokerpipe.DecommissionSnapshot
	startup                            startupRestorationBackend
	installedAttempted, installedBound bool
	installed                          installedUninstallBackend
	completion                         installedMaintenanceCompletion
}

// OpenMaintenance observes only the exact existing normal-running broker as
// the elevated non-impersonated approved current client. Initially STOPPED or
// pending services refuse: SCM cannot supply a valid original process identity
// in those states. It never starts a service to manufacture missing evidence.
// Call Quiesce explicitly, retain the owner and recheck it at later boundaries.
func OpenMaintenance(ctx context.Context, req MaintenanceRequest) (*MaintenanceOwner, error) {
	return openMaintenanceWith(ctx, req, maintenanceOperations{
		authenticate: maintenanceOperatorNative, open: openMaintenanceNative,
		poll: servicePoll, limit: maintenanceStopLimit,
	})
}

func openMaintenanceWith(ctx context.Context, req MaintenanceRequest, op maintenanceOperations) (result *MaintenanceOwner, resultErr error) {
	return openMaintenanceModeWith(ctx, req, op, false)
}

func openMaintenanceModeWith(ctx context.Context, req MaintenanceRequest, op maintenanceOperations, decommission bool) (result *MaintenanceOwner, resultErr error) {
	open := op.open
	if decommission {
		open = op.openDecommission
	}
	if ctx == nil || ctx.Err() != nil || !validPin(req.ApprovedBrokerSHA256) || op.authenticate == nil || open == nil || op.poll == nil || op.limit <= 0 || op.limit > maintenanceStopLimit {
		return nil, errors.New("live independently approved native maintenance inputs required")
	}
	// This is a new independently approved operator invocation. An expired
	// original bootstrap anchor refuses until a separate completed broker-owned
	// maintenance handoff exists; the retained-registration parser cannot turn
	// caller-supplied bytes into that missing authority or reset rotated trust.
	client, err := brokerregistration.ParseApprovedRequest(bytes.Clone(req.ClientRegistrationRequest), req.ApprovedClientRequestSHA256, time.Now().UTC())
	if err != nil {
		return nil, err
	}
	if err := op.authenticate(client.ClientSID); err != nil {
		return nil, err
	}
	return openMaintenanceAuthorityWith(ctx, maintenanceAuthority{brokerSHA: req.ApprovedBrokerSHA256, client: client}, op, decommission, nil)
}

// The retained identity entry is private. Only the fixed native customer opener
// may supply a completion from the original protected installation transaction;
// the public operator entry above still validates fresh bootstrap trust.
func openMaintenanceAuthorityWith(ctx context.Context, authority maintenanceAuthority, op maintenanceOperations, decommission bool, completion installedMaintenanceCompletion) (result *MaintenanceOwner, resultErr error) {
	open := op.open
	if decommission {
		open = op.openDecommission
	}
	if ctx == nil || ctx.Err() != nil || open == nil || op.authenticate == nil || op.poll == nil || op.limit <= 0 || op.limit > maintenanceStopLimit {
		err := errors.New("live native maintenance authority required")
		if completion != nil {
			err = errors.Join(err, completion.close())
		}
		return nil, err
	}
	m := &MaintenanceOwner{authority: authority, op: op, decommission: decommission, completion: completion}
	m.self = m
	defer func() {
		if resultErr != nil {
			resultErr = errors.Join(resultErr, m.Close())
		}
	}()
	var err error
	if completion != nil {
		if err := completion.check(ctx); err != nil {
			return nil, err
		}
	}
	m.backend, err = open(ctx, m.authority)
	if err != nil || m.backend == nil {
		return nil, errors.Join(errors.New("retained original maintenance service required"), err)
	}
	if err := m.custody(ctx); err != nil {
		return nil, err
	}
	state, err := m.backend.status()
	if err != nil || !maintenanceRunning(state, state.processID) {
		return nil, errors.Join(errors.New("original running broker required; stopped/pending state lacks drain proof"), err)
	}
	m.pid = state.processID
	m.process, err = m.backend.bindProcess(m.pid)
	if err != nil || m.process == nil {
		return nil, errors.Join(errors.New("original native broker process binding refused"), err)
	}
	if err := m.check(ctx); err != nil {
		return nil, err
	}
	return m, nil
}

func maintenanceRunning(s serviceState, pid uint32) bool {
	return pid != 0 && s.state == 4 && s.processID == pid && s.win32Exit == 0 && s.serviceExit == 0 && s.controls == 5 // STOP | SHUTDOWN
}

func (m *MaintenanceOwner) custody(ctx context.Context) error {
	if ctx == nil || ctx.Err() != nil {
		if ctx != nil {
			return ctx.Err()
		}
		return errors.New("live maintenance context required")
	}
	if m.closed || m.failed || m.backend == nil {
		return errors.New("original maintenance owner revoked or closed")
	}
	if err := m.op.authenticate(m.authority.client.ClientSID); err != nil {
		return err
	}
	if m.completion != nil {
		if err := m.completion.check(ctx); err != nil {
			return err
		}
	}
	return m.backend.check(ctx)
}

func (m *MaintenanceOwner) check(ctx context.Context) error {
	if err := m.custody(ctx); err != nil {
		return err
	}
	if m.process == nil {
		return errors.New("original retained maintenance process required")
	}
	state, err := m.backend.status()
	if err != nil {
		return err
	}
	if !m.quiescent {
		if !maintenanceRunning(state, m.pid) {
			return errors.New("original running maintenance binding changed")
		}
		return m.process.live()
	}
	if !serviceCompleted(state) {
		return errors.New("quiescent broker restarted or service state changed")
	}
	exited, code, err := m.process.exit()
	if err != nil || !exited || code != 0 {
		return errors.Join(errors.New("original broker clean process exit required"), err)
	}
	return nil
}

func (m *MaintenanceOwner) Check(ctx context.Context) error {
	if m == nil || m.self != m {
		return errors.New("original maintenance owner required")
	}
	m.mu.Lock()
	defer m.mu.Unlock()
	err := m.check(ctx)
	if err == nil && (m.startupBound || m.startupPrepared) {
		if m.startup == nil {
			err = errors.New("bound original shortcut owners missing")
		} else {
			err = m.startup.check(ctx)
		}
	}
	if err == nil && m.installedAttempted {
		if m.installed == nil {
			err = errors.New("prepared original installed owners missing")
		} else {
			err = m.installed.check(ctx)
		}
	}
	if err != nil {
		m.failed = true
		if m.dispatched {
			err = errors.Join(ErrMaintenanceUncertain, err)
		}
	}
	return err
}

// Quiesce consumes one attempt and dispatches at most one STOP to the retained,
// independently checked service. It neither kills a process nor restarts a
// service. SCM STOPPED alone never proves exit or drained updater handlers.
func (m *MaintenanceOwner) Quiesce(ctx context.Context) (resultErr error) {
	if m == nil || m.self != m {
		return errors.New("original maintenance owner required")
	}
	m.mu.Lock()
	defer m.mu.Unlock()
	if m.decommission {
		return errors.New("decommission requires disable and original fence before STOP")
	}
	return m.quiesceLocked(ctx)
}

func (m *MaintenanceOwner) quiesceLocked(ctx context.Context) (resultErr error) {
	if m.stopAttempted || m.closed || m.failed {
		return errors.New("maintenance stop already consumed or owner revoked")
	}
	m.stopAttempted = true
	defer func() {
		if resultErr != nil {
			m.failed = true
			if m.dispatched {
				resultErr = errors.Join(ErrMaintenanceUncertain, resultErr)
			}
		}
	}()
	if err := m.check(ctx); err != nil {
		return err
	}
	waitCtx, cancel := context.WithTimeout(ctx, m.op.limit)
	defer cancel()
	m.dispatched = true
	if err := m.backend.stop(m.pid); err != nil {
		return err
	}
	for {
		if err := m.custody(waitCtx); err != nil {
			return err
		}
		state, err := m.backend.status()
		if err != nil {
			return err
		}
		stopped := serviceCompleted(state)
		pending := state.state == serviceStopPending && state.controls == 0 && state.win32Exit == 0 && state.serviceExit == 0
		if !stopped && !pending && !maintenanceRunning(state, m.pid) {
			return errors.New("original broker stop state changed or shutdown failed")
		}
		exited, code, err := m.process.exit()
		if err != nil || exited && code != 0 {
			return errors.Join(errors.New("original process shutdown failed or identity uncertain"), err)
		}
		if stopped && exited {
			m.quiescent = true
			return m.check(waitCtx)
		}
		if err := m.op.poll(waitCtx); err != nil {
			return err
		}
	}
}

// Close releases owners only. It never repairs, deletes, republishes, retries
// or restarts anything. A close failure remains an error on every later Close.
func (m *MaintenanceOwner) Close() error {
	if m == nil {
		return nil
	}
	if m.self != m {
		return errors.New("original maintenance owner cannot be copied")
	}
	m.mu.Lock()
	defer m.mu.Unlock()
	return m.closeLocked()
}

func (m *MaintenanceOwner) closeLocked() error {
	if m.closed {
		return m.closeErr
	}
	m.closed = true
	if m.installed != nil {
		m.closeErr = errors.Join(m.closeErr, m.installed.close())
		m.installed = nil
	}
	if m.startup != nil {
		m.closeErr = errors.Join(m.closeErr, m.startup.close())
		m.startup = nil
	}
	if m.fence != nil {
		m.closeErr = errors.Join(m.closeErr, m.fence.Close())
		m.fence = nil
	}
	if m.process != nil {
		m.closeErr = errors.Join(m.closeErr, m.process.close())
		m.process = nil
	}
	if m.backend != nil {
		m.closeErr = errors.Join(m.closeErr, m.backend.close())
		m.backend = nil
	}
	if m.completion != nil {
		m.closeErr = errors.Join(m.closeErr, m.completion.close())
		m.completion = nil
	}
	if m.closeErr != nil {
		m.closeErr = errors.Join(ErrCleanup, m.closeErr)
		if m.dispatched {
			m.closeErr = errors.Join(ErrMaintenanceUncertain, m.closeErr)
		}
	}
	return m.closeErr
}

func (*MaintenanceOwner) MarshalJSON() ([]byte, error) {
	return nil, errors.New("native maintenance owner cannot be serialized")
}
func (*MaintenanceOwner) UnmarshalJSON([]byte) error {
	return errors.New("native maintenance owner cannot be reconstructed")
}
