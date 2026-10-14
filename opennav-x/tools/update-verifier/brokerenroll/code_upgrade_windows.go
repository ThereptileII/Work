//go:build windows

package brokerenroll

import (
	"bytes"
	"context"
	"crypto/rand"
	"encoding/hex"
	"encoding/json"
	"errors"
	"io"
	"path/filepath"
	"runtime"
	"time"

	"example.com/opennav-update-verifier/brokerpipe"
	"example.com/opennav-update-verifier/brokerregistration"
	"example.com/opennav-update-verifier/launcher"
	"example.com/opennav-update-verifier/releasepolicy"
	"example.com/opennav-update-verifier/repository"
	"golang.org/x/sys/windows"
)

// These capabilities exist only after signed-purpose validation, actual running
// new-provisioner source authentication and exact original completion approval.
type nativeCodeUpgrade struct {
	inputs                                                                                  installedMaintenanceInputs
	original                                                                                *nativeInstalledMaintenanceAuthority
	maintenance                                                                             *MaintenanceOwner
	backend                                                                                 *nativeMaintenanceBackend
	policy                                                                                  releasepolicy.BrokerCodeUpgradePolicy
	record                                                                                  codeUpgradeRecord
	rawPolicy, reservationRaw                                                               []byte
	files                                                                                   *nativeEnrollmentFiles
	broker, helper, oldBroker, oldHelper                                                    startupRestorationFile
	staged, reserved, disabled, stopped, replaced, validated, activated, readied, completed bool
	successor                                                                               *enrollmentPin
	startUncertain                                                                          bool
	recovering                                                                              bool
	closed                                                                                  bool
	closeErr                                                                                error
}

func openCodeUpgradeNativeMode(ctx context.Context, p releasepolicy.BrokerCodeUpgradePolicy, raw, trust []byte, originalSHA string, resume bool) (result codeUpgradeOwner, resultErr error) {
	if resume {
		return resumeCodeUpgradeNative(ctx, p, raw, trust, originalSHA)
	}
	// This private native entry is called only by upgradeEnrolledCodeWith after
	// independently approved target verification and actual running source binding.
	in, err := readInstalledCompletionModeNative(ctx, false)
	if err != nil {
		return nil, err
	}
	n := &nativeCodeUpgrade{inputs: in, policy: p, rawPolicy: bytes.Clone(raw), files: &nativeEnrollmentFiles{}}
	n.original, _ = in.completion.(*nativeInstalledMaintenanceAuthority)
	defer func() {
		if resultErr != nil {
			resultErr = errors.Join(resultErr, n.close())
		}
	}()
	if n.original == nil || digest(n.original.originalRaw) != originalSHA || !bytes.Equal(in.authority.client.TrustConfiguration, trust) || in.authority.client.TrustConfigSHA256 != p.Bootstrap.TrustConfigSHA256 || in.authority.client.Trust().Channel != p.Bootstrap.Channel || n.original.record.Broker.SHA256 != p.PreviousBrokerSHA256 || n.original.record.MaintenanceHelper.SHA256 != p.PreviousHelperSHA256 {
		return nil, errors.New("upgrade differs from exact original enrollment or unchanged trust")
	}
	n.record = codeUpgradeRecord{Schema: 1, Purpose: codeUpgradeRecordPurpose, OriginalCompletionSHA256: originalSHA, ClientRequestSHA256: in.authority.client.RequestSHA256, Policy: bytes.Clone(raw), PolicySHA256: digest(raw), PreviousBroker: n.original.record.Broker, PreviousHelper: n.original.record.MaintenanceHelper}
	n.record.CodeParent, err = startupDirectoryIdentity(n.original.codeParent)
	if err != nil {
		return nil, err
	}
	for _, name := range []string{stagedBrokerName, stagedHelperName, previousBrokerName, previousHelperName} {
		if err = startupRestorationAbsent(n.original.codeParent, name); err != nil {
			return nil, err
		}
	}
	// Original completion ownership transfers to the exact same service/process
	// owner. Upgrade rights are obtained only through this private opener.
	n.maintenance, err = openMaintenanceAuthorityWith(ctx, in.authority, maintenanceOperations{authenticate: maintenanceOperatorNative, open: func(ctx context.Context, a maintenanceAuthority) (maintenanceBackend, error) {
		return openMaintenanceNativeMode(ctx, a, false, true)
	}, poll: servicePoll, limit: maintenanceStopLimit}, false, in.completion)
	if err != nil {
		n.original = nil
		return nil, err
	}
	n.backend, _ = n.maintenance.backend.(*nativeMaintenanceBackend)
	if n.backend == nil {
		return nil, errors.New("native upgrade service required")
	}
	return n, n.check(ctx)
}
func (n *nativeCodeUpgrade) check(ctx context.Context) error {
	if n == nil || n.closed || ctx == nil || ctx.Err() != nil || n.original == nil || n.backend == nil {
		return errors.New("live retained code upgrade required")
	}
	if !n.replaced && n.recovering {
		return n.checkDisabledStopped(ctx)
	}
	if !n.replaced {
		return errors.Join(n.maintenance.Check(ctx), n.files.Check(), n.checkMovable(ctx))
	}
	if err := errors.Join(n.original.check(ctx), n.backend.check(ctx), n.files.Check(), n.checkMovable(ctx)); err != nil {
		return err
	}
	state, err := n.backend.status()
	if err != nil {
		return err
	}
	if !n.activated && !serviceCompleted(state) {
		return errors.New("replaced code requires original stopped service")
	}
	if n.activated {
		if !maintenanceRunning(state, n.maintenance.pid) || n.maintenance.process == nil {
			return errors.New("successor service running identity changed")
		}
		return n.maintenance.process.live()
	}
	return nil
}
func (n *nativeCodeUpgrade) checkMovable(ctx context.Context) error {
	for _, f := range []startupRestorationFile{n.broker, n.helper, n.oldBroker, n.oldHelper} {
		if f.pin != nil {
			if err := errors.Join(f.checkAlias(), startupHashPin(ctx, f.pin, fingerprintForCodeFile(n, f))); err != nil {
				return err
			}
		}
	}
	return nil
}
func fingerprintForCodeFile(n *nativeCodeUpgrade, f startupRestorationFile) string {
	switch f.pin {
	case n.broker.pin:
		return n.policy.Bootstrap.Broker.SHA256
	case n.helper.pin:
		return n.policy.Bootstrap.Provisioner.SHA256
	case n.oldBroker.pin:
		return n.policy.PreviousBrokerSHA256
	default:
		return n.policy.PreviousHelperSHA256
	}
}
func (n *nativeCodeUpgrade) stage(ctx context.Context, broker io.Reader, helper maintenanceImageSource) error {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if n.staged || n.reserved {
		return errors.New("code staging already consumed")
	}
	n.staged = true
	if err := n.check(ctx); err != nil {
		return err
	}
	// Persist exact authenticated intent BEFORE first staging mutation. Partial
	// staging therefore cannot be adopted or mistaken for a fresh installation.
	intent, err := json.Marshal(n.record)
	if err != nil {
		return err
	}
	sd, err := clientRequestDescriptor(n.backend.brokerSID, false)
	if err != nil {
		return err
	}
	if _, err = n.write(ctx, n.original.requestParent, n.original.clientSID+codeIntentSuffix, sd, intent); err != nil {
		return err
	}
	imageSD, err := enrollmentCodeDescriptor(mustCodeBrokerSID(n.backend.brokerSID), false)
	if err != nil {
		return err
	}
	n.broker, err = n.stageImage(ctx, stagedBrokerName, broker, n.policy.Bootstrap.Broker, imageSD, false)
	if err != nil {
		return err
	}
	n.helper, err = n.stageImage(ctx, stagedHelperName, helper, n.policy.Bootstrap.Provisioner, imageSD, true)
	if err != nil {
		return err
	}
	n.record.Broker, err = startupFileIdentity(ctx, n.broker.pin, n.policy.Bootstrap.Broker.SHA256)
	if err != nil {
		return err
	}
	n.record.Helper, err = startupFileIdentity(ctx, n.helper.pin, n.policy.Bootstrap.Provisioner.SHA256)
	return err
}
func mustCodeBrokerSID(s string) *windows.SID { sid, _ := windows.StringToSid(s); return sid }
func (n *nativeCodeUpgrade) write(ctx context.Context, parent *enrollmentPin, name string, sd *windows.SECURITY_DESCRIPTOR, raw []byte) (*enrollmentPin, error) {
	return writeCodeUpgradeFileNative(ctx, n.files, parent, name, sd, raw)
}
func writeCodeUpgradeFileNative(ctx context.Context, files *nativeEnrollmentFiles, parent *enrollmentPin, name string, sd *windows.SECURITY_DESCRIPTOR, raw []byte) (pin *enrollmentPin, resultErr error) {
	file, pin, err := files.newFile(parent, name, sd)
	if err != nil {
		return nil, err
	}
	open := true
	defer func() {
		if open {
			resultErr = errors.Join(resultErr, enrollmentCleanup(file.Close()))
		}
	}()
	if ctx.Err() != nil {
		return nil, ctx.Err()
	}
	if _, err = file.Write(raw); err != nil {
		return nil, err
	}
	pin.length = int64(len(raw))
	if err = file.Sync(); err != nil {
		return nil, err
	}
	if err = startupHashPin(ctx, pin, digest(raw)); err != nil {
		return nil, err
	}
	err = enrollmentCleanup(file.Close())
	open = false
	if err != nil {
		return nil, err
	}
	if err = reopenClientRequest(pin); err != nil {
		return nil, err
	}
	pin.dos = filepath.Join(parent.dos, name)
	files.pins = append(files.pins, pin)
	return pin, errors.Join(pin.check(), startupHashPin(ctx, pin, digest(raw)))
}
func (n *nativeCodeUpgrade) stageImage(ctx context.Context, name string, source io.Reader, a releasepolicy.Artifact, sd *windows.SECURITY_DESCRIPTOR, helper bool) (f startupRestorationFile, resultErr error) {
	writer, pin, err := n.files.newFile(n.original.codeParent, name, sd)
	if err != nil {
		return f, err
	}
	if helper {
		err = copyMaintenanceImage(ctx, writer, source, a)
	} else {
		err = copyEnrollmentImage(ctx, writer, source, a)
	}
	if err == nil {
		pin.length = a.Bytes
		err = writer.Sync()
	}
	if err == nil {
		err = startupHashPin(ctx, pin, a.SHA256)
	}
	closeErr := enrollmentCleanup(writer.Close())
	if err != nil || closeErr != nil {
		return f, errors.Join(err, closeErr)
	}
	f, err = openCodeMoveFile(n.files, n.original.codeParent, name, sd)
	if err != nil {
		return f, err
	}
	if f.pin.id != pin.id || f.pin.info.CreationTime != pin.info.CreationTime {
		return f, errors.New("staged code identity changed")
	}
	return f, errors.Join(f.checkAlias(), startupHashPin(ctx, f.pin, a.SHA256))
}
func (n *nativeCodeUpgrade) reserve(ctx context.Context) error {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if !n.staged || n.reserved {
		return errors.New("complete pair staging required")
	}
	n.reserved = true
	raw, err := marshalCodeUpgradeRecord(n.record)
	if err != nil {
		return err
	}
	sd, err := clientRequestDescriptor(n.backend.brokerSID, false)
	if err != nil {
		return err
	}
	_, err = n.write(ctx, n.original.requestParent, n.original.clientSID+codeReservationSuffix, sd, raw)
	if err == nil {
		n.reservationRaw = raw
	}
	return err
}
func (n *nativeCodeUpgrade) quiesce(ctx context.Context) error {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if !n.reserved || n.disabled {
		return errors.New("single reserved code shutdown required")
	}
	if err := n.check(ctx); err != nil {
		return err
	}
	n.disabled = true // consume before an SCM effect, including ambiguous failure
	if err := n.setStart(ctx, windows.SERVICE_DISABLED); err != nil {
		return err
	}
	if err := n.maintenance.Quiesce(ctx); err != nil {
		return err
	}
	n.stopped = true
	drain := codeDrainRecord{Schema: 1, ReservationSHA256: digest(n.reservationRaw), PID: n.maintenance.pid, Birth: n.backend.bound.birth(), ExitCode: 0}
	raw, err := json.Marshal(drain)
	if err != nil {
		return err
	}
	sd, err := clientRequestDescriptor(n.backend.brokerSID, false)
	if err != nil {
		return err
	}
	if _, err = n.write(ctx, n.original.requestParent, n.original.clientSID+codeDrainSuffix, sd, raw); err != nil {
		return err
	}
	return n.check(ctx)
}

// Release a completed read owner only after actual original process exit. The
// replacement owner reopens DELETE access and compares every original identity;
// protected ancestry/ACLs remain held through this deliberate handoff.
func codeReleasePin(files *nativeEnrollmentFiles, pin *enrollmentPin) error {
	for i, p := range files.pins {
		if p == pin {
			err := enrollmentCleanup(windows.CloseHandle(p.handle))
			if err != nil {
				return err
			}
			files.pins = append(files.pins[:i], files.pins[i+1:]...)
			return nil
		}
	}
	return errors.New("owned code pin handoff required")
}
func openCodeMoveFile(files *nativeEnrollmentFiles, parent *enrollmentPin, name string, sd *windows.SECURITY_DESCRIPTOR) (f startupRestorationFile, resultErr error) {
	physical := parent.physical + `\` + name
	pointer, err := windows.UTF16PtrFromString(enrollmentPhysical(physical))
	if err != nil {
		return f, err
	}
	h, err := windows.CreateFile(pointer, windows.GENERIC_READ|windows.READ_CONTROL|windows.DELETE, windows.FILE_SHARE_READ, nil, windows.OPEN_EXISTING, windows.FILE_FLAG_OPEN_REPARSE_POINT, 0)
	if err != nil {
		return f, err
	}
	p := &enrollmentPin{handle: h, physical: physical, descriptor: sd, file: true, broker: parent.broker}
	files.pins = append(files.pins, p)
	f = startupRestorationFile{p, filepath.Join(parent.dos, name)}
	p.info, p.id, err = enrollmentInfo(h)
	if err != nil {
		return f, err
	}
	p.length = int64(uint64(p.info.FileSizeHigh)<<32 | uint64(p.info.FileSizeLow))
	if p.length < 1 || p.length > maxBrokerBytes {
		return f, errors.New("bounded code move owner required")
	}
	return f, f.checkAlias()
}
func (n *nativeCodeUpgrade) replace(ctx context.Context) (resultErr error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if n.recovering {
		return n.resumeReplacement(ctx)
	}
	if !n.stopped || n.replaced {
		return errors.New("clean original process exit required before code replacement")
	}
	if err := n.check(ctx); err != nil {
		return err
	}
	if err := n.maintenance.process.close(); err != nil {
		return err
	}
	n.maintenance.process = nil
	if err := codeReleasePin(n.backend.files, n.backend.image); err != nil {
		return err
	}
	for _, p := range []*enrollmentPin{n.original.brokerImage, n.original.helperImage} {
		if err := codeReleasePin(n.original.files, p); err != nil {
			return err
		}
	}
	sd, err := enrollmentCodeDescriptor(mustCodeBrokerSID(n.backend.brokerSID), false)
	if err != nil {
		return err
	}
	n.oldBroker, err = openCodeMoveFile(n.files, n.original.codeParent, enrollmentImageName, sd)
	if err != nil {
		return err
	}
	n.oldHelper, err = openCodeMoveFile(n.files, n.original.codeParent, maintenanceImageName, sd)
	if err != nil {
		return err
	}
	if err = errors.Join(installedMatchFile(ctx, n.oldBroker.pin, n.original.record.Broker), installedMatchFile(ctx, n.oldHelper.pin, n.original.record.MaintenanceHelper)); err != nil {
		return err
	}
	restoreOwned := true
	defer func() {
		if resultErr != nil && restoreOwned {
			recovery, cancel := context.WithTimeout(context.Background(), serviceInitializationLimit)
			defer cancel()
			if e := n.checkDisabledStopped(recovery); e != nil {
				resultErr = errors.Join(resultErr, e)
				return
			}
			resultErr = errors.Join(resultErr, restoreOwnedCodePair(recovery, n.original.codeParent, &n.oldBroker, &n.oldHelper, &n.broker, &n.helper, n.policy))
		}
	}()
	// Each move uses the same DELETE-capable handle, retained destination parent
	// and collision refusal. Original bytes are retained before publishing either
	// successor. Crash/partial state keeps the service disabled and intent intact.
	for _, move := range []struct {
		f         *startupRestorationFile
		name, sha string
	}{{&n.oldBroker, previousBrokerName, n.policy.PreviousBrokerSHA256}, {&n.oldHelper, previousHelperName, n.policy.PreviousHelperSHA256}, {&n.broker, enrollmentImageName, n.policy.Bootstrap.Broker.SHA256}, {&n.helper, maintenanceImageName, n.policy.Bootstrap.Provisioner.SHA256}} {
		if err = n.checkDisabledStopped(ctx); err != nil {
			return err
		}
		if err = startupRenameRestorationFile(ctx, move.f, n.original.codeParent, move.name, move.sha); err != nil {
			return err
		}
	}
	restoreOwned = false
	return n.publishSuccessor(ctx)
}
func (n *nativeCodeUpgrade) publishSuccessor(ctx context.Context) error {
	sd, err := enrollmentCodeDescriptor(mustCodeBrokerSID(n.backend.brokerSID), false)
	if err != nil {
		return err
	}
	n.record.PreviousBroker, err = startupFileIdentity(ctx, n.oldBroker.pin, n.policy.PreviousBrokerSHA256)
	if err != nil {
		return err
	}
	n.record.PreviousHelper, err = startupFileIdentity(ctx, n.oldHelper.pin, n.policy.PreviousHelperSHA256)
	if err != nil {
		return err
	}
	n.record.Broker, err = startupFileIdentity(ctx, n.broker.pin, n.policy.Bootstrap.Broker.SHA256)
	if err != nil {
		return err
	}
	n.record.Helper, err = startupFileIdentity(ctx, n.helper.pin, n.policy.Bootstrap.Provisioner.SHA256)
	if err != nil {
		return err
	}
	n.record.ReservationSHA256 = digest(n.reservationRaw)
	raw, err := marshalCodeUpgradeRecord(n.record)
	if err != nil {
		return err
	}
	requestSD, err := clientRequestDescriptor(n.backend.brokerSID, false)
	if err != nil {
		return err
	}
	n.successor, err = n.writeRecoveryMatch(ctx, n.original.requestParent, n.original.clientSID+codeSuccessorSuffix, requestSD, raw)
	if err != nil {
		return err
	}

	// Downgrade each mutation handle only after the complete pair/receipt is
	// durable, comparing the original full identity across the protected handoff.
	// Process image observers require the ordinary non-DELETE read owner.
	for _, f := range []*startupRestorationFile{&n.broker, &n.helper} {
		want, e := startupFileIdentity(ctx, f.pin, fingerprintForCodeFile(n, *f))
		if e != nil {
			return e
		}
		name := filepath.Base(f.dos)
		if e = codeReleasePin(n.files, f.pin); e != nil {
			return e
		}
		pin, e := maintenanceHoldFile(n.files, n.original.codeParent, name, sd, maxBrokerBytes)
		if e != nil {
			return e
		}
		if e = installedMatchFile(ctx, pin, want); e != nil {
			return e
		}
		f.pin = pin
	}
	n.backend.image = n.broker.pin
	n.original.brokerImage = n.broker.pin
	n.original.helperImage = n.helper.pin
	n.replaced = true
	return n.check(ctx)
}
func (n *nativeCodeUpgrade) checkDisabledStopped(ctx context.Context) error {
	if ctx == nil || ctx.Err() != nil || !n.disabled || !n.stopped {
		return errors.New("disabled original service required")
	}
	if err := errors.Join(n.backend.service.check(n.backend.service.image), maintenanceNoRecovery(n.backend.service.service), n.original.check(ctx), n.files.Check(), n.checkMovable(ctx)); err != nil {
		return err
	}
	s, e := n.backend.status()
	if e != nil || !serviceCompleted(s) {
		return errors.Join(errors.New("original disabled service must remain stopped"), e)
	}
	return nil
}
func (n *nativeCodeUpgrade) setStart(ctx context.Context, start uint32) error {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if n.startUncertain {
		return errors.New("prior SCM start-type effect uncertain; no automatic retry")
	}
	if n.backend == nil {
		return errors.New("retained original upgrade service required")
	}
	if err := n.backend.check(ctx); err != nil {
		return err
	}
	if ctx.Err() != nil {
		return ctx.Err()
	}
	n.startUncertain = true
	if err := windows.ChangeServiceConfig(n.backend.service.service, windows.SERVICE_NO_CHANGE, start, windows.SERVICE_NO_CHANGE, nil, nil, nil, nil, nil, nil, nil); err != nil {
		return err
	}
	n.backend.service.expectedStart = start
	n.startUncertain = false
	return n.backend.service.check(n.backend.service.image)
}
func (n *nativeCodeUpgrade) validateStore(ctx context.Context) (resultErr error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if !n.replaced || n.validated {
		return errors.New("complete successor pair required for one store validation")
	}
	if err := n.check(ctx); err != nil {
		return err
	}
	// verify-store runs as the original virtual account. Never initialize-store
	// or register-client, never open/reset histories or floors as administrator.
	if err := n.setStart(ctx, windows.SERVICE_DEMAND_START); err != nil {
		return err
	}
	defer func() {
		if resultErr != nil {
			resultErr = errors.Join(resultErr, n.setStart(context.WithoutCancel(ctx), windows.SERVICE_DISABLED))
		}
	}()
	if err := n.backend.service.start("verify-store"); err != nil {
		return err
	}
	wait, cancel := context.WithTimeout(ctx, serviceInitializationLimit)
	defer cancel()
	if err := waitServiceStopped(wait, &n.backend.service, func() error {
		return errors.Join(wait.Err(), n.original.check(wait), n.files.Check(), n.backend.service.check(n.backend.service.image), n.checkMovable(wait))
	}, servicePoll); err != nil {
		return err
	}
	// This durable pair selection precedes restoring AUTO_START. Health remains
	// separately incomplete until the registered-client readiness query succeeds.
	raw, e := installedReadPin(ctx, n.successor, maxCodeUpgradeRecordBytes)
	if e != nil {
		return e
	}
	sd, e := clientRequestDescriptor(n.backend.brokerSID, false)
	if e != nil {
		return e
	}
	if _, e = n.writeRecoveryMatch(ctx, n.original.requestParent, n.original.clientSID+codeSuccessorPairSuffix, sd, codeSuccessorMarker(digest(raw))); e != nil {
		return e
	}
	n.validated = true
	return n.check(ctx)
}
func (n *nativeCodeUpgrade) activate(ctx context.Context) (resultErr error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if !n.validated || n.activated {
		return errors.New("validated stopped successor required")
	}
	if err := n.check(ctx); err != nil {
		return err
	}
	defer func() {
		if resultErr != nil {
			resultErr = errors.Join(resultErr, n.setStart(context.WithoutCancel(ctx), windows.SERVICE_DISABLED))
		}
	}()
	if err := n.setStart(ctx, windows.SERVICE_AUTO_START); err != nil {
		return err
	}
	if err := windows.StartService(n.backend.service.service, 0, nil); err != nil {
		return err
	}
	wait, cancel := context.WithTimeout(ctx, serviceInitializationLimit)
	defer cancel()
	for {
		if err := errors.Join(wait.Err(), n.original.check(wait), n.files.Check(), n.backend.service.check(n.backend.service.image), n.checkMovable(wait)); err != nil {
			return err
		}
		s, err := n.backend.status()
		if err != nil {
			return err
		}
		if maintenanceRunning(s, s.processID) {
			process, err := openMaintenanceProcess(s.processID, n.broker.pin, n.backend.brokerSID, 0)
			if err != nil {
				return err
			}
			// Retain the authenticated successor process through ready/completion.
			n.maintenance.process = process
			n.maintenance.pid = s.processID
			n.activated = true
			return n.check(ctx)
		}
		if s.state != serviceStartPending || s.controls != 0 || s.win32Exit != 0 || s.serviceExit != 0 {
			return errors.New("successor did not become normally running")
		}
		if err := servicePoll(wait); err != nil {
			return err
		}
	}
}
func (n *nativeCodeUpgrade) ready(ctx context.Context) (resultErr error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if !n.activated || n.readied {
		return errors.New("single successor readiness required")
	}
	defer func() {
		if resultErr != nil {
			resultErr = errors.Join(resultErr, n.setStart(context.WithoutCancel(ctx), windows.SERVICE_DISABLED))
		}
	}()
	if err := n.check(ctx); err != nil {
		return err
	}
	var nonce [16]byte
	if _, err := rand.Read(nonce[:]); err != nil {
		return err
	}
	page := 0
	q := brokerpipe.MaintenanceRequest{Schema: 1, Operation: brokerpipe.SelectedHistory, RequestID: hex.EncodeToString(nonce[:]), Page: &page}
	// Authenticated pipe/server + actual original client registration/store read.
	// This does not launch/qualify the application, repair a payload or touch profile.
	r, err := launcher.CallMaintenanceNative(ctx, q)
	if err != nil || !brokerpipe.ValidMaintenanceClientResult(r, q) {
		return errors.Join(errors.New("successor registered-client history readiness refused"), err)
	}
	n.readied = true
	return n.check(ctx)
}
func (n *nativeCodeUpgrade) complete(ctx context.Context) error {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if !n.readied || n.completed || n.successor == nil {
		return errors.New("actual successor readiness required before completion")
	}
	if err := n.check(ctx); err != nil {
		return err
	}
	raw, err := installedReadPin(ctx, n.successor, maxCodeUpgradeRecordBytes)
	if err != nil {
		return err
	}
	sd, err := clientRequestDescriptor(n.backend.brokerSID, false)
	if err != nil {
		return err
	}
	if _, err = n.write(ctx, n.original.requestParent, n.original.clientSID+codeSuccessorMarkerSuffix, sd, codeSuccessorMarker(digest(raw))); err != nil {
		return err
	}
	n.completed = true
	return n.check(ctx)
}
func (n *nativeCodeUpgrade) close() error {
	if n == nil {
		return nil
	}
	if n.closed {
		return n.closeErr
	}
	n.closed = true
	var err error
	if n.disabled && !n.completed && n.backend != nil {
		ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
		err = n.setStart(ctx, windows.SERVICE_DISABLED)
		cancel()
	}
	if n.maintenance != nil {
		err = errors.Join(err, n.maintenance.Close())
	} else if n.inputs.completion != nil {
		err = errors.Join(err, n.inputs.completion.close())
	}
	n.closeErr = errors.Join(err, n.files.Close())
	return n.closeErr
}

// Original completion stays immutable. Exact absence selects original code;
// ANY intent/reservation/successor presence requires the completed exact tuple.
// Malformed, partial, mixed and unknown state never falls back to old code.
func readCodeSuccessorNative(ctx context.Context, c *nativeInstalledMaintenanceAuthority, original installedCompletionRecord, originalRaw []byte, recordSD, imageSD *windows.SECURITY_DESCRIPTOR, pendingReady ...bool) (installedCompletionRecord, error) {
	parent := c.requestParent
	sid := c.clientSID
	intentAbsent := startupRestorationAbsent(parent, sid+codeIntentSuffix) == nil
	if intentAbsent {
		for _, suffix := range []string{codeDrainSuffix, codeReservationSuffix, codeSuccessorSuffix, codeSuccessorPairSuffix, codeSuccessorMarkerSuffix} {
			if startupRestorationAbsent(parent, sid+suffix) != nil {
				return original, errors.New("partial code successor state")
			}
		}
		return original, nil
	}
	intent, err := maintenanceHoldFile(c.files, parent, sid+codeIntentSuffix, recordSD, maxCodeUpgradeRecordBytes)
	if err != nil {
		return original, err
	}
	intentRaw, err := installedReadPin(ctx, intent, maxCodeUpgradeRecordBytes)
	if err != nil {
		return original, err
	}
	reservation, err := maintenanceHoldFile(c.files, parent, sid+codeReservationSuffix, recordSD, maxCodeUpgradeRecordBytes)
	if err != nil {
		return original, err
	}
	reservedRaw, err := installedReadPin(ctx, reservation, maxCodeUpgradeRecordBytes)
	if err != nil {
		return original, err
	}
	reserved, err := parseCodeUpgradeRecord(reservedRaw)
	if err != nil {
		return original, err
	}
	drain, err := maintenanceHoldFile(c.files, parent, sid+codeDrainSuffix, recordSD, maxCodeUpgradeRecordBytes)
	if err != nil {
		return original, err
	}
	drainRaw, err := installedReadPin(ctx, drain, maxCodeUpgradeRecordBytes)
	if err != nil {
		return original, err
	}
	if _, err = parseCodeDrainRecord(drainRaw, digest(reservedRaw)); err != nil {
		return original, err
	}
	expectedIntent := reserved
	expectedIntent.Broker = startupFileBinding{}
	expectedIntent.Helper = startupFileBinding{}
	expectedIntentRaw, err := json.Marshal(expectedIntent)
	if err != nil || !bytes.Equal(intentRaw, expectedIntentRaw) {
		return original, errors.New("code reservation differs from exact original intent")
	}
	receipt, err := maintenanceHoldFile(c.files, parent, sid+codeSuccessorSuffix, recordSD, maxCodeUpgradeRecordBytes)
	if err != nil {
		return original, err
	}
	raw, err := installedReadPin(ctx, receipt, maxCodeUpgradeRecordBytes)
	if err != nil {
		return original, err
	}
	r, err := parseCodeUpgradeRecord(raw)
	if err != nil {
		return original, err
	}
	pair, err := maintenanceHoldFile(c.files, parent, sid+codeSuccessorPairSuffix, recordSD, maxInstalledCompletionMarkerBytes)
	if err != nil {
		return original, err
	}
	pairRaw, err := installedReadPin(ctx, pair, maxInstalledCompletionMarkerBytes)
	if err != nil || !bytes.Equal(pairRaw, codeSuccessorMarker(digest(raw))) {
		return original, errors.New("code pair selection is incomplete")
	}
	// Only independently authenticated running finalization may request the
	// pending-ready closure. Any ready namespace presence still refuses it.
	if len(pendingReady) == 1 && pendingReady[0] {
		if err := startupRestorationAbsent(parent, sid+codeSuccessorMarkerSuffix); err != nil {
			return original, errors.New("ready marker must be absent for finalization")
		}
	} else {
		marker, err := maintenanceHoldFile(c.files, parent, sid+codeSuccessorMarkerSuffix, recordSD, maxInstalledCompletionMarkerBytes)
		if err != nil {
			return original, err
		}
		markerRaw, err := installedReadPin(ctx, marker, maxInstalledCompletionMarkerBytes)
		if err != nil || !bytes.Equal(markerRaw, codeSuccessorMarker(digest(raw))) {
			return original, errors.New("code successor is incomplete")
		}

	}
	codeParent, err := startupDirectoryIdentity(c.codeParent)
	if err != nil {
		return original, err
	}
	if r.OriginalCompletionSHA256 != digest(originalRaw) || r.ClientRequestSHA256 != original.ClientRequestSHA256 || r.ReservationSHA256 != digest(reservedRaw) || reserved.ReservationSHA256 != "" || reserved.OriginalCompletionSHA256 != digest(originalRaw) || reserved.ClientRequestSHA256 != original.ClientRequestSHA256 || r.PolicySHA256 != reserved.PolicySHA256 || !bytes.Equal(r.Policy, reserved.Policy) || r.CodeParent != codeParent || reserved.CodeParent != codeParent || reserved.PreviousBroker != original.Broker || reserved.PreviousHelper != original.MaintenanceHelper || !sameCodeFile(r.PreviousBroker, original.Broker) || !sameCodeFile(r.PreviousHelper, original.MaintenanceHelper) || !sameCodeFile(r.Broker, reserved.Broker) || !sameCodeFile(r.Helper, reserved.Helper) {
		return original, errors.New("successor differs from original exact reserved transaction")
	}
	p, err := releasepolicy.ParseBrokerCodeUpgrade(r.Policy)
	if err != nil {
		return original, err
	}
	if p.Bootstrap.TrustConfigSHA256 != c.recordTrustSHA() {
		return original, errors.New("successor trust differs from original")
	}
	for _, item := range []struct {
		name string
		want startupFileBinding
	}{{previousBrokerName, r.PreviousBroker}, {previousHelperName, r.PreviousHelper}} {
		pin, e := maintenanceHoldFile(c.files, c.codeParent, item.name, imageSD, maxBrokerBytes)
		if e != nil {
			return original, e
		}
		if e = installedMatchFile(ctx, pin, item.want); e != nil {
			return original, e
		}
	}
	for _, name := range []string{stagedBrokerName, stagedHelperName} {
		if startupRestorationAbsent(c.codeParent, name) != nil {
			return original, errors.New("unknown staged code remains")
		}
	}
	original.Broker, original.MaintenanceHelper = r.Broker, r.Helper
	return original, nil
}
func (c *nativeInstalledMaintenanceAuthority) recordTrustSHA() string {
	r, e := parseInstalledCompletion(c.originalRaw)
	if e != nil {
		return ""
	}
	client, e := brokerregistration.ParseRetainedApprovedRequest(r.ClientRequest, r.ClientRequestSHA256)
	if e != nil {
		return ""
	}
	return client.TrustConfigSHA256
}

// Bootstrap-anchor expiry is exempted only after this genuine native original
// completion check. Current TUF metadata/root expiry and durable floors remain
// enforced by VerifyBrokerCodeUpgrade; fresh enrollment still uses ValidateTrust.
func codeUpgradeTrustNativeMode(ctx context.Context, raw []byte, originalSHA string, resume bool) (trust repository.Trust, resultErr error) {
	in, err := readInstalledCompletionModeNative(ctx, false, resume)
	if err != nil {
		return trust, err
	}
	defer func() { resultErr = errors.Join(resultErr, in.completion.close()) }()
	c, ok := in.completion.(*nativeInstalledMaintenanceAuthority)
	if !ok || digest(c.originalRaw) != originalSHA || !bytes.Equal(in.authority.client.TrustConfiguration, raw) {
		return trust, errors.New("original installed trust/completion differs from independent approval")
	}
	return repository.ValidateTrustIdentity(raw)
}

// Recovery is deliberately limited to the SAME live owner and original clean
// process-exit proof. It reconciles only known positions of those original
// handles; foreign/ambiguous paths refuse. An interrupted process/reboot does not
// reconstruct this capability from a journal, reset a store or enroll again.
func restoreOwnedCodePair(ctx context.Context, parent *enrollmentPin, oldBroker, oldHelper, newBroker, newHelper *startupRestorationFile, p releasepolicy.BrokerCodeUpgradePolicy) error {
	for _, pair := range []struct {
		old, next                               *startupRestorationFile
		fixed, staged, previous, oldSHA, newSHA string
	}{
		{oldBroker, newBroker, enrollmentImageName, stagedBrokerName, previousBrokerName, p.PreviousBrokerSHA256, p.Bootstrap.Broker.SHA256},
		{oldHelper, newHelper, maintenanceImageName, stagedHelperName, previousHelperName, p.PreviousHelperSHA256, p.Bootstrap.Provisioner.SHA256},
	} {
		if pair.old == nil || pair.next == nil || pair.old.pin == nil || pair.next.pin == nil {
			return errors.New("same original pair recovery owners required")
		}
		if err := errors.Join(pair.old.checkAlias(), pair.next.checkAlias(), startupHashPin(ctx, pair.old.pin, pair.oldSHA), startupHashPin(ctx, pair.next.pin, pair.newSHA)); err != nil {
			return err
		}
		fixed := filepath.Join(parent.dos, pair.fixed)
		previous := filepath.Join(parent.dos, pair.previous)
		staged := filepath.Join(parent.dos, pair.staged)
		if pair.old.dos != fixed && pair.old.dos != previous || pair.next.dos != fixed && pair.next.dos != staged || pair.old.dos == pair.next.dos {
			return errors.New("unknown code recovery position")
		}
		if pair.next.dos == fixed {
			if pair.old.dos != previous {
				return errors.New("original backup required before removing successor")
			}
			if err := startupRenameRestorationFile(ctx, pair.next, parent, pair.staged, pair.newSHA); err != nil {
				return err
			}
		}
		if pair.old.dos == previous {
			if err := startupRenameRestorationFile(ctx, pair.old, parent, pair.fixed, pair.oldSHA); err != nil {
				return err
			}
		}
		if pair.old.dos != fixed || pair.next.dos != staged {
			return errors.New("restored original code positions differ")
		}
		if err := errors.Join(pair.old.checkAlias(), pair.next.checkAlias(), startupHashPin(ctx, pair.old.pin, pair.oldSHA), startupHashPin(ctx, pair.next.pin, pair.newSHA)); err != nil {
			return err
		}
	}
	return nil
}

// Resume requires the exact authenticated ORIGINAL completion and the original
// transaction's immutable intent/reservation/clean-exit proof. It never treats
// SCM STOPPED, a current pathname or an adjacent self hash as that proof.
func resumeCodeUpgradeNative(ctx context.Context, p releasepolicy.BrokerCodeUpgradePolicy, raw, trust []byte, originalSHA string) (result *nativeCodeUpgrade, resultErr error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	in, err := readInstalledCompletionModeNative(ctx, false, true)
	if err != nil {
		return nil, err
	}
	c, ok := in.completion.(*nativeInstalledMaintenanceAuthority)
	if !ok {
		return nil, errors.New("native original recovery completion required")
	}
	n := &nativeCodeUpgrade{inputs: in, original: c, policy: p, rawPolicy: bytes.Clone(raw), files: &nativeEnrollmentFiles{}, recovering: true, staged: true, reserved: true, disabled: true, stopped: true}
	defer func() {
		if resultErr != nil {
			resultErr = errors.Join(resultErr, n.close())
		}
	}()
	if digest(c.originalRaw) != originalSHA || !bytes.Equal(in.authority.client.TrustConfiguration, trust) || p.Bootstrap.TrustConfigSHA256 != in.authority.client.TrustConfigSHA256 || p.Bootstrap.Channel != in.authority.client.Trust().Channel {
		return nil, errors.New("resume differs from independently approved original enrollment")
	}
	broker, err := enrollmentBrokerSID()
	if err != nil {
		return nil, err
	}
	sd, err := clientRequestDescriptor(broker.String(), false)
	if err != nil {
		return nil, err
	}
	read := func(suffix string) ([]byte, error) {
		pin, e := maintenanceHoldFile(n.files, c.requestParent, c.clientSID+suffix, sd, maxCodeUpgradeRecordBytes)
		if e != nil {
			return nil, e
		}
		return installedReadPin(ctx, pin, maxCodeUpgradeRecordBytes)
	}
	reservationRaw, err := read(codeReservationSuffix)
	if err != nil {
		return nil, err
	}
	r, err := parseCodeUpgradeRecord(reservationRaw)
	if err != nil {
		return nil, err
	}
	parent, err := startupDirectoryIdentity(c.codeParent)
	if err != nil {
		return nil, err
	}
	if r.ReservationSHA256 != "" || r.OriginalCompletionSHA256 != originalSHA || r.ClientRequestSHA256 != in.authority.client.RequestSHA256 || r.PolicySHA256 != digest(raw) || !bytes.Equal(r.Policy, raw) || r.PreviousBroker != c.record.Broker || r.PreviousHelper != c.record.MaintenanceHelper || r.CodeParent != parent {
		return nil, errors.New("resume differs from original exact reserved pair")
	}
	intent, err := read(codeIntentSuffix)
	if err != nil {
		return nil, err
	}
	wantIntent := r
	wantIntent.Broker = startupFileBinding{}
	wantIntent.Helper = startupFileBinding{}
	wantRaw, _ := json.Marshal(wantIntent)
	if !bytes.Equal(intent, wantRaw) {
		return nil, errors.New("resume intent differs from original reservation")
	}
	drainRaw, err := read(codeDrainSuffix)
	if err != nil {
		return nil, err
	}
	drain, err := parseCodeDrainRecord(drainRaw, digest(reservationRaw))
	if err != nil {
		return nil, err
	}
	if err = codeDrainedProcessAbsent(drain); err != nil {
		return nil, err
	}
	n.record = r
	n.reservationRaw = reservationRaw
	n.backend, err = openStoppedCodeUpgradeService(ctx, c, broker.String())
	if err != nil {
		return nil, err
	}
	m := &MaintenanceOwner{authority: in.authority, backend: n.backend, completion: in.completion, op: maintenanceOperations{authenticate: maintenanceOperatorNative}, quiescent: true, stopAttempted: true}
	m.self = m
	n.maintenance = m
	imageSD, err := enrollmentCodeDescriptor(broker, false)
	if err != nil {
		return nil, err
	}
	n.oldBroker, n.broker, err = readReservedCodePositions(ctx, n.files, c.codeParent, enrollmentImageName, previousBrokerName, stagedBrokerName, r.PreviousBroker, r.Broker, imageSD)
	if err != nil {
		return nil, err
	}
	n.oldHelper, n.helper, err = readReservedCodePositions(ctx, n.files, c.codeParent, maintenanceImageName, previousHelperName, stagedHelperName, r.PreviousHelper, r.Helper, imageSD)
	if err != nil {
		return nil, err
	}
	return n, n.check(ctx)
}
func codeDrainedProcessAbsent(d codeDrainRecord) (resultErr error) {
	h, err := windows.OpenProcess(windows.PROCESS_QUERY_LIMITED_INFORMATION|windows.SYNCHRONIZE, false, d.PID)
	if errors.Is(err, windows.ERROR_INVALID_PARAMETER) {
		return nil
	}
	if err != nil {
		return err
	}
	defer func() { resultErr = errors.Join(resultErr, enrollmentCleanup(windows.CloseHandle(h))) }()
	var birth, exit, kernel, user windows.Filetime
	if err = windows.GetProcessTimes(h, &birth, &exit, &kernel, &user); err != nil {
		return err
	}
	if uint64(birth.HighDateTime)<<32|uint64(birth.LowDateTime) != d.Birth {
		return nil
	} // PID reuse is not original process survival.
	state, err := windows.WaitForSingleObject(h, 0)
	if err != nil || state != windows.WAIT_OBJECT_0 {
		return errors.Join(errors.New("drained original broker unexpectedly live"), err)
	}
	var code uint32
	if err = windows.GetExitCodeProcess(h, &code); err != nil || code != 0 {
		return errors.Join(errors.New("original zero-exit drain proof contradicted"), err)
	}
	return nil
}
func openStoppedCodeUpgradeService(ctx context.Context, c *nativeInstalledMaintenanceAuthority, brokerSID string) (result *nativeMaintenanceBackend, resultErr error) {
	n := &nativeMaintenanceBackend{files: &nativeEnrollmentFiles{}, clientSID: c.clientSID, brokerSID: brokerSID}
	defer func() {
		if resultErr != nil {
			resultErr = errors.Join(resultErr, n.close())
		}
	}()
	if err := maintenanceOperatorNative(c.clientSID); err != nil {
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
	n.service.service, err = windows.OpenService(n.service.scm, name, windows.SERVICE_QUERY_CONFIG|windows.SERVICE_QUERY_STATUS|windows.SERVICE_CHANGE_CONFIG|windows.SERVICE_START|windows.SERVICE_STOP|windows.READ_CONTROL)
	if err != nil {
		return nil, err
	}
	n.service.expected, err = windows.SecurityDescriptorFromString("D:P(A;;0xf01ff;;;SY)(A;;0xf01ff;;;BA)(A;;0x20005;;;AU)(A;;0x20005;;;" + brokerSID + ")")
	if err != nil {
		return nil, err
	}
	n.service.expectedStart = windows.SERVICE_DISABLED
	n.service.image = filepath.Join(c.codeParent.dos, enrollmentImageName)
	if err = n.check(ctx); err != nil {
		return nil, err
	}
	state, err := n.status()
	if err != nil || !serviceCompleted(state) {
		return nil, errors.Join(errors.New("recovery requires the original disabled stopped service"), err)
	}
	return n, nil
}
func readReservedCodePositions(ctx context.Context, files *nativeEnrollmentFiles, parent *enrollmentPin, fixed, previous, staged string, old, next startupFileBinding, sd *windows.SECURITY_DESCRIPTOR) (oldFile, nextFile startupRestorationFile, resultErr error) {
	for _, name := range []string{fixed, previous, staged} {
		if startupRestorationAbsent(parent, name) == nil {
			continue
		}
		f, err := openCodeMoveFile(files, parent, name, sd)
		if err != nil {
			return oldFile, nextFile, err
		}
		var want startupFileBinding
		switch f.pin.id.ID {
		case old.FileID:
			if oldFile.pin != nil || name == staged {
				return oldFile, nextFile, errors.New("duplicate or misplaced original recovery code")
			}
			want = old
			oldFile = f
		case next.FileID:
			if nextFile.pin != nil || name == previous {
				return oldFile, nextFile, errors.New("duplicate or misplaced successor recovery code")
			}
			want = next
			nextFile = f
		default:
			return oldFile, nextFile, errors.New("foreign code occupies reserved recovery namespace")
		}
		actual, err := startupFileIdentity(ctx, f.pin, want.SHA256)
		if err != nil || !sameCodeFile(actual, want) {
			return oldFile, nextFile, errors.Join(errors.New("recovery code differs from full reserved identity"), err)
		}
		// NTFS tunneling may change creation time only at a vacated fixed name.
		if name != fixed && actual.Creation != want.Creation {
			return oldFile, nextFile, errors.New("unmoved recovery code creation changed")
		}
	}
	if oldFile.pin == nil || nextFile.pin == nil {
		return oldFile, nextFile, errors.New("complete exact reserved code pair required")
	}
	return oldFile, nextFile, nil
}
func (n *nativeCodeUpgrade) resumeReplacement(ctx context.Context) error {
	if n.replaced {
		return errors.New("code resume already consumed")
	}
	if err := n.checkDisabledStopped(ctx); err != nil {
		return err
	}
	if err := resumeOwnedCodePair(ctx, n.checkDisabledStopped, n.original.codeParent, &n.oldBroker, &n.oldHelper, &n.broker, &n.helper, n.policy); err != nil {
		return err
	}
	return n.publishSuccessor(ctx)
}
func (n *nativeCodeUpgrade) writeRecoveryMatch(ctx context.Context, parent *enrollmentPin, name string, sd *windows.SECURITY_DESCRIPTOR, raw []byte) (*enrollmentPin, error) {
	if !n.recovering || startupRestorationAbsent(parent, name) == nil {
		return n.write(ctx, parent, name, sd, raw)
	}
	pin, err := maintenanceHoldFile(n.files, parent, name, sd, maxCodeUpgradeRecordBytes)
	if err != nil {
		return nil, err
	}
	actual, err := installedReadPin(ctx, pin, maxCodeUpgradeRecordBytes)
	if err != nil || !bytes.Equal(actual, raw) {
		return nil, errors.Join(errors.New("existing recovery publication differs from the same transaction"), err)
	}
	return pin, nil
}

func resumeOwnedCodePair(ctx context.Context, guard func(context.Context) error, parent *enrollmentPin, oldBroker, oldHelper, nextBroker, nextHelper *startupRestorationFile, p releasepolicy.BrokerCodeUpgradePolicy) error {
	if guard == nil {
		return errors.New("retained service recovery guard required")
	}
	if err := guard(ctx); err != nil {
		return err
	}
	for _, pair := range []struct {
		old, next       *startupRestorationFile
		fixed, previous string
		oldSHA, newSHA  string
	}{
		{oldBroker, nextBroker, enrollmentImageName, previousBrokerName, p.PreviousBrokerSHA256, p.Bootstrap.Broker.SHA256},
		{oldHelper, nextHelper, maintenanceImageName, previousHelperName, p.PreviousHelperSHA256, p.Bootstrap.Provisioner.SHA256},
	} {
		if pair.old.dos == filepath.Join(parent.dos, pair.fixed) {
			if err := guard(ctx); err != nil {
				return err
			}
			if err := startupRenameRestorationFile(ctx, pair.old, parent, pair.previous, pair.oldSHA); err != nil {
				return err
			}
		}
		if pair.next.dos != filepath.Join(parent.dos, pair.fixed) {
			if err := guard(ctx); err != nil {
				return err
			}
			if err := startupRenameRestorationFile(ctx, pair.next, parent, pair.fixed, pair.newSHA); err != nil {
				return err
			}
		}
		if err := guard(ctx); err != nil {
			return err
		}
	}
	return nil
}

// The disabled recovery sequence is unchanged. Native phase observation and
// authentication choose this owner; a running finalizer never obtains it.
func (n *nativeCodeUpgrade) resume(ctx context.Context) error {
	for _, step := range []func(context.Context) error{n.replace, n.validateStore, n.activate, n.ready, n.complete} {
		if err := n.check(ctx); err != nil {
			return err
		}
		if err := step(ctx); err != nil {
			return err
		}
	}
	return n.check(ctx)
}
