//go:build windows

package brokerenroll

import (
	"bytes"
	"context"
	"errors"
	"path/filepath"
	"runtime"
	"unsafe"

	"example.com/opennav-update-verifier/launcher"
	"example.com/opennav-update-verifier/releasepolicy"
	"golang.org/x/sys/windows"
)

// Kernel-enforced observer rights: finalization cannot configure/start/stop or
// delete a service even if a future call accidentally reaches such an API.
const codeFinalizeServiceRights = windows.SERVICE_QUERY_CONFIG | windows.SERVICE_QUERY_STATUS | windows.READ_CONTROL

type nativeCodeFinalizeBackend struct {
	service   nativeServiceBackend
	original  *nativeInstalledMaintenanceAuthority // borrowed from publication owner
	image     *enrollmentPin                       // borrowed; process observer closes before original custody
	brokerSID string
}

func (n *nativeCodeFinalizeBackend) check(ctx context.Context) error {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if n == nil || n.original == nil || ctx == nil || ctx.Err() != nil {
		return errors.New("live original finalization observation required")
	}
	return errors.Join(n.original.check(ctx), maintenanceOperatorNative(n.original.clientSID), n.service.check(n.service.image), maintenanceNoRecovery(n.service.service))
}
func (n *nativeCodeFinalizeBackend) status() (serviceState, error) { return n.service.status() }
func (n *nativeCodeFinalizeBackend) bindProcess(pid uint32) (maintenanceProcess, error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if err := n.check(context.Background()); err != nil {
		return nil, err
	}
	state, err := n.status()
	if err != nil || n.service.expectedStart != windows.SERVICE_AUTO_START || !maintenanceRunning(state, pid) || n.image == nil {
		return nil, errors.Join(errors.New("actual same normally running successor required"), err)
	}
	return openMaintenanceProcess(pid, n.image, n.brokerSID, 0)
}
func (n *nativeCodeFinalizeBackend) close() error {
	if n == nil {
		return nil
	}
	return n.service.close()
}

func openCodeFinalizeObserverNative(ctx context.Context, c *nativeInstalledMaintenanceAuthority, brokerSID string, startCapability ...bool) (result *nativeCodeFinalizeBackend, resultErr error) {
	n := &nativeCodeFinalizeBackend{original: c, brokerSID: brokerSID}
	defer func() {
		if resultErr != nil {
			resultErr = errors.Join(resultErr, n.close())
		}
	}()
	if err := errors.Join(c.check(ctx), maintenanceOperatorNative(c.clientSID)); err != nil {
		return nil, err
	}
	var err error
	n.service.scm, err = windows.OpenSCManager(nil, nil, windows.SC_MANAGER_CONNECT|windows.READ_CONTROL)
	if err != nil {
		return nil, err
	}
	for _, path := range []string{`SYSTEM`, `SYSTEM\Select`, `SYSTEM\CurrentControlSet`, `SYSTEM\CurrentControlSet\Services`, `SYSTEM\CurrentControlSet\Services\` + serviceName} {
		if err = n.service.holdRegistry(path); err != nil {
			return nil, err
		}
	}
	name, _ := windows.UTF16PtrFromString(serviceName)
	rights := uint32(codeFinalizeServiceRights)
	if len(startCapability) == 1 && startCapability[0] {
		rights |= windows.SERVICE_START
	}
	n.service.service, err = windows.OpenService(n.service.scm, name, rights)
	if err != nil {
		return nil, err
	}
	n.service.expected, err = windows.SecurityDescriptorFromString("D:P(A;;0xf01ff;;;SY)(A;;0xf01ff;;;BA)(A;;0x20005;;;AU)(A;;0x20005;;;" + brokerSID + ")")
	if err != nil {
		return nil, err
	}
	n.service.image = filepath.Join(c.codeParent.dos, enrollmentImageName)
	// Observe the actual start type on this SAME query-only handle, then validate
	// every config/account/image/descriptor/registry fact with that exact type.
	var needed uint32
	if err = windows.QueryServiceConfig(n.service.service, nil, 0, &needed); !errors.Is(err, windows.ERROR_INSUFFICIENT_BUFFER) || needed < uint32(unsafe.Sizeof(windows.QUERY_SERVICE_CONFIG{})) || needed > 8192 {
		return nil, errors.Join(errors.New("bounded finalization service configuration required"), err)
	}
	raw := make([]byte, needed)
	config := (*windows.QUERY_SERVICE_CONFIG)(unsafe.Pointer(&raw[0]))
	if err = windows.QueryServiceConfig(n.service.service, config, uint32(len(raw)), &needed); err != nil {
		return nil, err
	}
	n.service.expectedStart = config.StartType
	if config.StartType != windows.SERVICE_DISABLED && config.StartType != windows.SERVICE_AUTO_START {
		return nil, errors.New("resume accepts only exact disabled recovery or running automatic finalization")
	}
	return n, n.check(ctx)
}

// The signed-purpose/source checks have already passed before this private
// entry. Native observations select the phase; no caller-selected phase or
// present ACL can manufacture missing completion/pair/readiness authority.
func openCodeResumeNative(ctx context.Context, p releasepolicy.BrokerCodeUpgradePolicy, raw, trust []byte, originalSHA string) (result codeUpgradeResumeOwner, resultErr error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	in, err := readInstalledCompletionModeNative(ctx, false, true)
	if err != nil {
		return nil, err
	}
	c, ok := in.completion.(*nativeInstalledMaintenanceAuthority)
	if !ok {
		return nil, errors.Join(errors.New("native original resume completion required"), in.completion.close())
	}
	var observer *nativeCodeFinalizeBackend
	transferred := false
	defer func() {
		if !transferred {
			if observer != nil {
				resultErr = errors.Join(resultErr, observer.close())
			}
			resultErr = errors.Join(resultErr, c.close())
		}
	}()
	if digest(c.originalRaw) != originalSHA || !bytes.Equal(in.authority.client.TrustConfiguration, trust) || p.Bootstrap.TrustConfigSHA256 != in.authority.client.TrustConfigSHA256 || p.Bootstrap.Channel != in.authority.client.Trust().Channel || p.PreviousBrokerSHA256 != c.record.Broker.SHA256 || p.PreviousHelperSHA256 != c.record.MaintenanceHelper.SHA256 {
		return nil, errors.New("resume differs from independently approved original tuple")
	}
	broker, err := enrollmentBrokerSID()
	if err != nil {
		return nil, err
	}
	observer, err = openCodeFinalizeObserverNative(ctx, c, broker.String())
	if err != nil {
		return nil, err
	}
	state, err := observer.status()
	if err != nil {
		return nil, err
	}
	if observer.service.expectedStart == windows.SERVICE_AUTO_START && maintenanceRunning(state, state.processID) &&
		startupRestorationAbsent(c.requestParent, c.clientSID+codeDrainSuffix) == nil &&
		startupRestorationAbsent(c.requestParent, c.clientSID+codeSuccessorSuffix) == nil {
		// The early opener independently requires unchanged fixed original IDs,
		// exact completed staging receipts and absence of every later boundary.
		transferred = true // Opener/owner takes original and observer cleanup.
		owner, err := openCodeEarlyAbortNative(ctx, c, observer, p, raw)
		if err != nil {
			return nil, errors.Join(err, observer.close(), c.close())
		}
		return owner, nil
	}
	if observer.service.expectedStart == windows.SERVICE_DISABLED {
		if startupRestorationAbsent(c.requestParent, c.clientSID+codeStartAttemptSuffix) != nil {
			return nil, errors.New("start attempt forbids disabled recovery")
		}
		if !serviceCompleted(state) {
			return nil, errors.New("disabled recovery still requires original clean stopped state")
		}
		// Broad recovery is reached only from this exact observed phase. Its original
		// opener independently reacquires/revalidates DISABLED/STOPPED and drain proof.
		if err = errors.Join(observer.close(), c.close()); err != nil {
			return nil, errors.Join(ErrCleanup, err)
		}
		transferred = true
		return resumeCodeUpgradeNative(ctx, p, raw, trust, originalSHA)
	}
	stopped := serviceCompleted(state)
	if !stopped && !maintenanceRunning(state, state.processID) {
		return nil, errors.New("automatic resume requires clean stopped or normally running successor")
	}
	recordSD, err := clientRequestDescriptor(broker.String(), false)
	if err != nil {
		return nil, err
	}
	imageSD, err := enrollmentCodeDescriptor(broker, false)
	if err != nil {
		return nil, err
	}
	image, readyRaw, err := openCodeFinalizePairNative(ctx, c, p, raw, recordSD, imageSD)
	if err != nil {
		return nil, err
	}
	observer.image = image
	publication := &nativeCodeFinalizePublication{original: c, files: &nativeEnrollmentFiles{}, parent: c.requestParent, name: c.clientSID + codeSuccessorMarkerSuffix, descriptor: recordSD, raw: readyRaw}
	if stopped {
		if err = startupRestorationAbsent(c.requestParent, c.clientSID+codeStartAttemptSuffix); err != nil {
			return nil, errors.New("existing start attempt forbids another start")
		}
		if err = observer.close(); err != nil {
			return nil, err
		}
		observer, err = openCodeFinalizeObserverNative(ctx, c, broker.String(), true)
		if err != nil {
			return nil, err
		}
		observer.image = image
		start := &nativeCodeStartBackend{nativeCodeFinalizeBackend: observer, publication: publication}
		owner := &codeUpgradeStarter{backend: start, publication: publication, call: launcher.CallMaintenanceNative}
		owner.self = owner
		transferred = true
		if err = owner.check(ctx); err != nil {
			return nil, errors.Join(err, owner.close())
		}
		return owner, nil
	}
	transferred = true // openCodeFinalizerWith owns cleanup, including failure.
	return openCodeFinalizerWith(ctx, observer, publication, launcher.CallMaintenanceNative)
}

type nativeCodeFinalizePublication struct {
	original          *nativeInstalledMaintenanceAuthority
	files             *nativeEnrollmentFiles
	parent            *enrollmentPin
	name              string
	descriptor        *windows.SECURITY_DESCRIPTOR
	raw               []byte
	pin               *enrollmentPin
	attempted, closed bool
	closeErr          error
}

func (p *nativeCodeFinalizePublication) check(ctx context.Context) error {
	if p == nil || p.closed || p.original == nil || p.files == nil || ctx == nil || ctx.Err() != nil || len(p.raw) == 0 {
		return errors.New("live exact finalization publication custody required")
	}
	if err := errors.Join(p.original.check(ctx), p.files.Check(), p.parent.check()); err != nil {
		return err
	}
	if p.pin != nil {
		return startupHashPin(ctx, p.pin, digest(p.raw))
	}
	if !p.attempted {
		return startupRestorationAbsent(p.parent, p.name)
	}
	return nil // One in-flight CREATE_NEW publication, never reusable authority.
}
func (p *nativeCodeFinalizePublication) publish(ctx context.Context, guard func(context.Context) error) (resultErr error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if p == nil || p.attempted || p.closed || guard == nil {
		return errors.New("single native ready publication required")
	}
	if err := errors.Join(p.check(ctx), guard(ctx)); err != nil {
		return err
	}
	// Recheck the complete retained process/service/tuple immediately before the
	// exclusive creation. Any unexpected destination presence must still refuse.
	if err := startupRestorationAbsent(p.parent, p.name); err != nil {
		return err
	}
	if err := guard(ctx); err != nil {
		return err
	}
	p.attempted = true
	var err error
	p.pin, err = writeCodeUpgradeFileNative(ctx, p.files, p.parent, p.name, p.descriptor, p.raw)
	if err != nil {
		return errors.Join(ErrCodeUpgradeUncertain, err)
	}
	// Sync/identity/hash readback happened in the shared writer. Re-observe the
	// same live process and complete native custody after those actual effects.
	return errors.Join(p.check(ctx), guard(ctx))
}
func (p *nativeCodeFinalizePublication) close() error {
	if p == nil {
		return nil
	}
	if p.closed {
		return p.closeErr
	}
	p.closed = true
	if p.files != nil {
		p.closeErr = errors.Join(p.closeErr, p.files.Close())
	}
	if p.original != nil {
		p.closeErr = errors.Join(p.closeErr, p.original.close())
	}
	return p.closeErr
}

// This closure is shared by the authenticated opener and inert native fixtures.
// It grants no service/process or publication capability on its own.
func openCodeFinalizePairNative(ctx context.Context, c *nativeInstalledMaintenanceAuthority, p releasepolicy.BrokerCodeUpgradePolicy, raw []byte, recordSD, imageSD *windows.SECURITY_DESCRIPTOR) (*enrollmentPin, []byte, error) {
	// Public installed readers still require ready. Only this authenticated native
	// path accepts an exact completed pair with a genuinely ABSENT ready marker.
	selected, err := readCodeSuccessorNative(ctx, c, c.record, c.originalRaw, recordSD, imageSD, true)
	if err != nil {
		return nil, nil, err
	}
	if selected.Broker.SHA256 != p.Bootstrap.Broker.SHA256 || selected.MaintenanceHelper.SHA256 != p.Bootstrap.Provisioner.SHA256 {
		return nil, nil, errors.New("complete exact successor pair required for finalization")
	}
	receipt, err := maintenanceHoldFile(c.files, c.requestParent, c.clientSID+codeSuccessorSuffix, recordSD, maxCodeUpgradeRecordBytes)
	if err != nil {
		return nil, nil, err
	}
	receiptRaw, err := installedReadPin(ctx, receipt, maxCodeUpgradeRecordBytes)
	if err != nil {
		return nil, nil, err
	}
	successor, err := parseCodeUpgradeRecord(receiptRaw)
	if err != nil || !bytes.Equal(successor.Policy, raw) || successor.PolicySHA256 != digest(raw) {
		return nil, nil, errors.Join(errors.New("finalization differs from exact independently approved policy"), err)
	}
	for _, item := range []struct {
		name string
		want startupFileBinding
	}{{enrollmentImageName, selected.Broker}, {maintenanceImageName, selected.MaintenanceHelper}} {
		pin, e := maintenanceHoldFile(c.files, c.codeParent, item.name, imageSD, maxBrokerBytes)
		if e != nil {
			return nil, nil, e
		}
		if e = installedMatchFile(ctx, pin, item.want); e != nil {
			return nil, nil, e
		}
		if item.name == enrollmentImageName {
			c.brokerImage = pin
		} else {
			c.helperImage = pin
		}
	}
	return c.brokerImage, codeSuccessorMarker(digest(receiptRaw)), nil
}
