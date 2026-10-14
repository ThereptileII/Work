//go:build windows

package brokerenroll

import (
	"bytes"
	"context"
	"errors"
	"path/filepath"
	"runtime"
	"unsafe"

	"example.com/opennav-update-verifier/brokerregistration"
	"golang.org/x/sys/windows"
)

func maintenanceOperatorNative(sid string) error {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if err := startupCurrentSID(sid); err != nil {
		return err
	}
	return validateClientSIDNative(sid)
}

type nativeMaintenanceBackend struct {
	service      nativeServiceBackend
	files        *nativeEnrollmentFiles
	image        *enrollmentPin
	clientSID    string
	brokerSID    string
	bound        *nativeMaintenanceProcess
	decommission bool
}

func openMaintenanceNative(ctx context.Context, authority maintenanceAuthority) (result maintenanceBackend, resultErr error) {
	return openMaintenanceNativeMode(ctx, authority, false, false)
}

func openDecommissionNative(ctx context.Context, authority maintenanceAuthority) (maintenanceBackend, error) {
	return openMaintenanceNativeMode(ctx, authority, true, false)
}

func openMaintenanceNativeMode(ctx context.Context, authority maintenanceAuthority, decommission, codeUpgrade bool) (result maintenanceBackend, resultErr error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if err := maintenanceOperatorNative(authority.client.ClientSID); err != nil {
		return nil, err
	}
	n := &nativeMaintenanceBackend{files: &nativeEnrollmentFiles{}, clientSID: authority.client.ClientSID, decommission: decommission}
	defer func() {
		if resultErr != nil {
			resultErr = errors.Join(resultErr, n.close())
		}
	}()
	broker, err := enrollmentBrokerSID()
	if err != nil {
		return nil, err
	}
	n.brokerSID = broker.String()
	// No CREATE/START/WRITE_DAC service rights, privilege enablement or broad
	// filesystem ACLs are acquired. Ordinary maintenance remains STOP-only.
	n.service.scm, err = windows.OpenSCManager(nil, nil, windows.SC_MANAGER_CONNECT|windows.READ_CONTROL)
	if err != nil {
		return nil, err
	}
	for _, path := range []string{`SYSTEM`, `SYSTEM\Select`, `SYSTEM\CurrentControlSet`, `SYSTEM\CurrentControlSet\Services`, `SYSTEM\CurrentControlSet\Services\` + serviceName} {
		if err := n.service.holdRegistry(path); err != nil {
			return nil, err
		}
	}
	name, _ := windows.UTF16PtrFromString(serviceName)
	rights := uint32(windows.SERVICE_QUERY_CONFIG | windows.SERVICE_QUERY_STATUS | windows.SERVICE_STOP | windows.READ_CONTROL)
	if decommission {
		rights |= windows.SERVICE_CHANGE_CONFIG | windows.DELETE
	}
	if codeUpgrade {
		rights |= windows.SERVICE_CHANGE_CONFIG | windows.SERVICE_START
	}
	n.service.service, err = windows.OpenService(n.service.scm, name, rights)
	if err != nil {
		return nil, err
	}
	n.service.expected, err = windows.SecurityDescriptorFromString("D:P(A;;0xf01ff;;;SY)(A;;0xf01ff;;;BA)(A;;0x20005;;;AU)(A;;0x20005;;;" + n.brokerSID + ")")
	if err != nil {
		return nil, err
	}
	n.service.expectedStart = windows.SERVICE_AUTO_START
	programFiles, err := windows.KnownFolderPath(windows.FOLDERID_ProgramFilesX86, 0)
	if err != nil {
		return nil, err
	}
	n.service.image = filepath.Join(programFiles, "SKAGER", "UpdateBroker", enrollmentImageName)
	if err := n.service.check(n.service.image); err != nil {
		return nil, err
	}
	codeSD, err := enrollmentCodeDescriptor(broker, true)
	if err != nil {
		return nil, err
	}
	fileSD, err := enrollmentCodeDescriptor(broker, false)
	if err != nil {
		return nil, err
	}
	parent, err := n.files.holdAncestry(programFiles, false)
	if err != nil {
		return nil, err
	}
	for _, component := range []string{"SKAGER", "UpdateBroker"} {
		parent, err = maintenanceHoldChild(n.files, parent, component, codeSD, n.brokerSID)
		if err != nil {
			return nil, err
		}
	}
	n.image, err = maintenanceHoldFile(n.files, parent, enrollmentImageName, fileSD, maxBrokerBytes)
	if err != nil {
		return nil, err
	}
	if err := startupHashPin(ctx, n.image, authority.brokerSHA); err != nil {
		return nil, errors.Join(errors.New("installed broker differs from separately approved hash"), err)
	}
	programData, err := windows.KnownFolderPath(windows.FOLDERID_ProgramData, 0)
	if err != nil {
		return nil, err
	}
	parent, err = n.files.holdAncestry(programData, true)
	if err != nil {
		return nil, err
	}
	dataSD, err := windows.SecurityDescriptorFromString("O:BAD:P(A;OICI;FA;;;SY)(A;OICI;FA;;;BA)(A;OICI;FA;;;" + n.brokerSID + ")")
	if err != nil {
		return nil, err
	}
	parent, err = maintenanceHoldChild(n.files, parent, "SKAGER", dataSD, n.brokerSID)
	if err != nil {
		return nil, err
	}
	fenceSD, err := windows.SecurityDescriptorFromString("O:BAD:P(A;;FA;;;SY)(A;;FA;;;BA)")
	if err != nil {
		return nil, err
	}
	fence, err := maintenanceHoldFile(n.files, parent, enrollmentFenceName, fenceSD, int64(len(filesCompleted)))
	if err != nil {
		return nil, err
	}
	fenceBytes, err := readClientRequestPin(ctx, fence)
	if err != nil || !bytes.Equal(fenceBytes, []byte(filesCompleted)) {
		return nil, errors.Join(errors.New("original completed enrollment fence required"), err)
	}
	requestSD, err := clientRequestDescriptor(n.brokerSID, true)
	if err != nil {
		return nil, err
	}
	parent, err = maintenanceHoldChild(n.files, parent, clientRequestsDirectory, requestSD, n.brokerSID)
	if err != nil {
		return nil, err
	}
	requestSD, err = clientRequestDescriptor(n.brokerSID, false)
	if err != nil {
		return nil, err
	}
	request, err := maintenanceHoldFile(n.files, parent, n.clientSID+".json", requestSD, brokerregistration.MaxRequestBytes)
	if err != nil {
		return nil, err
	}
	raw, err := readClientRequestPin(ctx, request)
	if err != nil || !bytes.Equal(raw, authority.client.OriginalBytes()) || digest(raw) != authority.client.RequestSHA256 {
		return nil, errors.Join(errors.New("retained machine request differs from separately approved original"), err)
	}
	if err := n.check(ctx); err != nil {
		return nil, err
	}
	// The store's broker/SYSTEM-only histories, floors and lifetime lock are
	// deliberately not opened as admin. Current broker shutdown is observable;
	// completed registration/recovery/mutation authority needs a separate actual
	// broker-owned handoff and cannot be reconstructed from these request bytes.
	return n, nil
}

func maintenanceHoldChild(f *nativeEnrollmentFiles, parent *enrollmentPin, component string, sd *windows.SECURITY_DESCRIPTOR, broker string) (*enrollmentPin, error) {
	physical := parent.physical + `\` + component
	child, err := f.pinDirectory(enrollmentPhysical(physical), physical, filepath.Join(parent.dos, component), sd, false, broker)
	if err == nil && child.info.VolumeSerialNumber != parent.info.VolumeSerialNumber {
		return nil, errors.New("maintenance ancestry crossed volume")
	}
	return child, err
}

func maintenanceHoldFile(f *nativeEnrollmentFiles, parent *enrollmentPin, name string, sd *windows.SECURITY_DESCRIPTOR, bound int64) (*enrollmentPin, error) {
	pin, err := startupOpenFile(f, parent, name, false, bound)
	if err != nil {
		return nil, err
	}
	// Exact original at-creation descriptor, not adoption of the current ACL.
	pin.descriptor = sd
	if pin.info.VolumeSerialNumber != parent.info.VolumeSerialNumber {
		return nil, errors.New("maintenance file crossed volume")
	}
	return pin, pin.check()
}

func (n *nativeMaintenanceBackend) check(ctx context.Context) error {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if ctx == nil || ctx.Err() != nil {
		return errors.New("live native maintenance context required")
	}
	if err := maintenanceOperatorNative(n.clientSID); err != nil {
		return err
	}
	if err := n.files.Check(); err != nil {
		return err
	}
	if err := n.service.check(n.service.image); err != nil {
		return err
	}
	return maintenanceNoRecovery(n.service.service)
}

// Original enrollment sets no delayed auto-start, failure actions or triggers.
// Refuse additions instead of clearing them or guessing about queued restart.
func maintenanceNoRecovery(service windows.Handle) error {
	for _, item := range []struct{ level, minimum uint32 }{
		{windows.SERVICE_CONFIG_DELAYED_AUTO_START_INFO, 4},
		{windows.SERVICE_CONFIG_FAILURE_ACTIONS_FLAG, 4},
		{windows.SERVICE_CONFIG_TRIGGER_INFO, 4},
		{windows.SERVICE_CONFIG_FAILURE_ACTIONS, uint32(unsafe.Sizeof(windows.SERVICE_FAILURE_ACTIONS{}))},
	} {
		var used uint32
		if err := windows.QueryServiceConfig2(service, item.level, nil, 0, &used); !errors.Is(err, windows.ERROR_INSUFFICIENT_BUFFER) || used < item.minimum || used > 8192 {
			return errors.Join(errors.New("bounded original service recovery configuration required"), err)
		}
		buffer := make([]byte, used)
		if err := windows.QueryServiceConfig2(service, item.level, &buffer[0], uint32(len(buffer)), &used); err != nil {
			return err
		}
		if item.level == windows.SERVICE_CONFIG_FAILURE_ACTIONS {
			actions := (*windows.SERVICE_FAILURE_ACTIONS)(unsafe.Pointer(&buffer[0]))
			command, commandErr := boundedServiceString(buffer, actions.Command)
			reboot, rebootErr := boundedServiceString(buffer, actions.RebootMsg)
			if actions.ActionsCount != 0 || actions.ResetPeriod != 0 || commandErr != nil || rebootErr != nil || command != "" || reboot != "" {
				return errors.New("service failure/restart actions differ from original enrollment")
			}
		} else if *(*uint32)(unsafe.Pointer(&buffer[0])) != 0 {
			return errors.New("service delayed/recovery/trigger configuration differs from original enrollment")
		}
	}
	return nil
}

func (n *nativeMaintenanceBackend) status() (serviceState, error) { return n.service.status() }
func (n *nativeMaintenanceBackend) bindProcess(pid uint32) (maintenanceProcess, error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if err := maintenanceOperatorNative(n.clientSID); err != nil {
		return nil, err
	}
	process, err := openMaintenanceProcess(pid, n.image, n.brokerSID, 0)
	if process != nil {
		n.bound = process.(*nativeMaintenanceProcess)
	}
	return process, err
}
func (n *nativeMaintenanceBackend) stop(pid uint32) error {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if err := n.check(context.Background()); err != nil {
		return err
	}
	if n.bound == nil || n.bound.pid != pid {
		return errors.New("original retained native process required before STOP")
	}
	if err := n.bound.live(); err != nil {
		return err
	}
	s, err := n.status()
	if err != nil || !maintenanceRunning(s, pid) {
		return errors.Join(errors.New("original bound broker changed before STOP"), err)
	}
	var status windows.SERVICE_STATUS
	return windows.ControlService(n.service.service, windows.SERVICE_CONTROL_STOP, &status)
}
func (n *nativeMaintenanceBackend) close() error {
	return errors.Join(n.service.close(), n.files.Close())
}
