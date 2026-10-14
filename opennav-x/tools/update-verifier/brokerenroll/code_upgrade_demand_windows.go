//go:build windows

package brokerenroll

import (
	"context"
	"errors"
	"path/filepath"
	"runtime"

	"example.com/opennav-update-verifier/releasepolicy"
	"golang.org/x/sys/windows"
)

// This handle cannot start or stop a service. AUTO is the sole configuration
// effect, after the protected complete-pair authority and exclusive attempt.
const codeDemandServiceRights = codeFinalizeServiceRights | windows.SERVICE_CHANGE_CONFIG

type nativeCodeDemandBackend struct {
	*nativeCodeFinalizeBackend
	publication          *nativeCodeFinalizePublication
	attemptRaw           []byte
	attemptPin           *enrollmentPin
	fencing, configuring bool
}

// Optional canonical attempts are compatible with legacy AUTO transactions.
// Call only after authenticating the complete protected pair, using its exact
// original/policy/successor/pair digests. Presence is never valid DEMAND retry
// authority; the DEMAND opener separately requires genuine absence.
func readCodeDemandAttemptNative(ctx context.Context, c *nativeInstalledMaintenanceAuthority, sd *windows.SECURITY_DESCRIPTOR, original, policy, successor, pair string) (bool, error) {
	if err := startupRestorationAbsent(c.requestParent, c.clientSID+codeDemandAttemptSuffix); err == nil {
		return false, nil
	}
	pin, err := maintenanceHoldFile(c.files, c.requestParent, c.clientSID+codeDemandAttemptSuffix, sd, maxCodeUpgradeRecordBytes)
	if err != nil {
		return true, err
	}
	raw, err := installedReadPin(ctx, pin, maxCodeUpgradeRecordBytes)
	if err != nil {
		return true, err
	}
	return true, matchCodeDemandAttempt(raw, codeDemandAttemptBytes(original, policy, successor, pair))
}

func (n *nativeCodeDemandBackend) check(ctx context.Context) error {
	if n == nil || n.nativeCodeFinalizeBackend == nil || n.publication == nil || len(n.attemptRaw) == 0 ||
		(n.service.expectedStart != windows.SERVICE_DEMAND_START && n.service.expectedStart != windows.SERVICE_AUTO_START) {
		return errors.New("exact demand transition configuration required")
	}
	if err := errors.Join(n.nativeCodeFinalizeBackend.check(ctx), n.publication.check(ctx),
		startupRestorationAbsent(n.original.requestParent, n.original.clientSID+codeStartAttemptSuffix)); err != nil {
		return err
	}
	if n.attemptPin != nil {
		return startupHashPin(ctx, n.attemptPin, digest(n.attemptRaw))
	}
	if n.fencing {
		return errors.New("demand attempt publication outcome uncertain")
	}
	return startupRestorationAbsent(n.original.requestParent, n.original.clientSID+codeDemandAttemptSuffix)
}

func (n *nativeCodeDemandBackend) fence(ctx context.Context) error {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if n.fencing || n.configuring || n.service.expectedStart != windows.SERVICE_DEMAND_START {
		return errors.New("demand configuration attempt already consumed")
	}
	if err := n.check(ctx); err != nil {
		return err
	}
	n.fencing = true
	var err error
	n.attemptPin, err = writeCodeUpgradeFileNative(ctx, n.publication.files, n.original.requestParent,
		n.original.clientSID+codeDemandAttemptSuffix, n.publication.descriptor, n.attemptRaw)
	return err
}

func (n *nativeCodeDemandBackend) automatic(ctx context.Context) error {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if n.configuring || !n.fencing || n.attemptPin == nil || n.service.expectedStart != windows.SERVICE_DEMAND_START {
		return errors.New("single durably fenced demand configuration required")
	}
	if err := n.check(ctx); err != nil {
		return err
	}
	state, err := n.status()
	if err != nil || !serviceCompleted(state) {
		return errors.Join(errors.New("clean stopped demand service required before configuration"), err)
	}
	n.configuring = true // An error may still have changed SCM; no retry/rollback.
	if err = windows.ChangeServiceConfig(n.service.service, windows.SERVICE_NO_CHANGE, windows.SERVICE_AUTO_START,
		windows.SERVICE_NO_CHANGE, nil, nil, nil, nil, nil, nil, nil); err != nil {
		return err
	}
	n.service.expectedStart = windows.SERVICE_AUTO_START
	return n.check(ctx)
}

// Takes original-completion custody on every outcome. The caller must close its
// query-only observer before transferring c, and the callback must re-open the
// existing authenticated AUTO dispatcher after this capability has been closed.
// This entry is reached only after the existing signed-purpose/source/tuple
// checks; it independently reauthenticates complete-pair policy and native files.
func openCodeDemandResumeNative(ctx context.Context, c *nativeInstalledMaintenanceAuthority, p releasepolicy.BrokerCodeUpgradePolicy, raw []byte, brokerSID string, openAutomatic func(context.Context) (codeUpgradeResumeOwner, error)) (result codeUpgradeResumeOwner, resultErr error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	publication := &nativeCodeFinalizePublication{original: c, files: &nativeEnrollmentFiles{}}
	backend := &nativeCodeDemandBackend{nativeCodeFinalizeBackend: &nativeCodeFinalizeBackend{original: c, brokerSID: brokerSID}, publication: publication}
	owner := &codeUpgradeDemandTransition{backend: backend, publication: publication, openAutomatic: openAutomatic}
	owner.self = owner
	defer func() {
		if resultErr != nil {
			resultErr = errors.Join(resultErr, owner.close())
		}
	}()
	if c == nil || ctx == nil || ctx.Err() != nil || openAutomatic == nil {
		return nil, errors.New("live authenticated demand resume required")
	}
	if err := errors.Join(c.check(ctx), maintenanceOperatorNative(c.clientSID)); err != nil {
		return nil, err
	}
	recordSD, err := clientRequestDescriptor(brokerSID, false)
	if err != nil {
		return nil, err
	}
	broker, err := windows.StringToSid(brokerSID)
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
	publication.parent, publication.name, publication.descriptor, publication.raw = c.requestParent, c.clientSID+codeSuccessorMarkerSuffix, recordSD, readyRaw
	backend.image = image
	if err = errors.Join(startupRestorationAbsent(c.requestParent, c.clientSID+codeDemandAttemptSuffix),
		startupRestorationAbsent(c.requestParent, c.clientSID+codeStartAttemptSuffix)); err != nil {
		return nil, errors.Join(errors.New("existing attempt forbids demand configuration"), err)
	}
	receipt, err := maintenanceHoldFile(c.files, c.requestParent, c.clientSID+codeSuccessorSuffix, recordSD, maxCodeUpgradeRecordBytes)
	if err != nil {
		return nil, err
	}
	receiptRaw, err := installedReadPin(ctx, receipt, maxCodeUpgradeRecordBytes)
	if err != nil {
		return nil, err
	}
	backend.attemptRaw = codeDemandAttemptBytes(digest(c.originalRaw), digest(raw), digest(receiptRaw), digest(readyRaw))
	backend.service.scm, err = windows.OpenSCManager(nil, nil, windows.SC_MANAGER_CONNECT|windows.READ_CONTROL)
	if err != nil {
		return nil, err
	}
	for _, path := range []string{`SYSTEM`, `SYSTEM\Select`, `SYSTEM\CurrentControlSet`, `SYSTEM\CurrentControlSet\Services`, `SYSTEM\CurrentControlSet\Services\` + serviceName} {
		if err = backend.service.holdRegistry(path); err != nil {
			return nil, err
		}
	}
	name, _ := windows.UTF16PtrFromString(serviceName)
	backend.service.service, err = windows.OpenService(backend.service.scm, name, codeDemandServiceRights)
	if err != nil {
		return nil, err
	}
	backend.service.expected, err = windows.SecurityDescriptorFromString("D:P(A;;0xf01ff;;;SY)(A;;0xf01ff;;;BA)(A;;0x20005;;;AU)(A;;0x20005;;;" + brokerSID + ")")
	if err != nil {
		return nil, err
	}
	backend.service.expectedStart = windows.SERVICE_DEMAND_START
	backend.service.image = filepath.Join(c.codeParent.dos, enrollmentImageName)
	if err = owner.check(ctx); err != nil {
		return nil, err
	}
	state, err := backend.status()
	if err != nil || !serviceCompleted(state) {
		return nil, errors.Join(errors.New("validated demand successor must be clean stopped"), err)
	}
	return owner, nil
}
