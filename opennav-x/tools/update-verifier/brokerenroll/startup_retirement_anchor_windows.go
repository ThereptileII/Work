//go:build windows

package brokerenroll

import (
	"context"
	"errors"
	"path/filepath"

	"example.com/opennav-update-verifier/brokerpipe"
	"golang.org/x/sys/windows"
)

// This private reader holds the original generation/app/launcher BEFORE the
// broker's one retirement call. Delete sharing permits acquisition of the
// broker's DELETE owner. Descendant handles must drain before tree rename;
// the original root remains held. No write/delete access is granted here.
// Ordinary enrollment pins keep their existing, immovable policy.
type startupRetirementAnchor struct {
	parent                  *enrollmentPin // borrowed fixed generations parent
	generation              string
	launcher                startupFileBinding
	pins                    [3]*enrollmentPin
	dos                     [3]string
	bound, failed, closed   bool
	drainAttempted, drained bool
	closeErr                error
}

func startupRetirementID(value string) bool {
	if len(value) != 32 {
		return false
	}
	for _, c := range value {
		if !(c >= '0' && c <= '9' || c >= 'a' && c <= 'f') {
			return false
		}
	}
	return true
}

func openStartupRetirementAnchor(ctx context.Context, parent *enrollmentPin, generation string, launcher startupFileBinding) (*startupRetirementAnchor, error) {
	a := &startupRetirementAnchor{parent: parent, generation: generation, launcher: launcher}
	if ctx == nil || ctx.Err() != nil || parent == nil || !startupRetirementID(generation) {
		return a, errors.New("original baseline startup anchor required")
	}
	if err := parent.check(); err != nil {
		return a, err
	}
	physical, dos := parent.physical+`\`+generation, filepath.Join(parent.dos, generation)
	for i, suffix := range []string{"", "app", "skager-start.exe"} {
		if suffix != "" {
			physical += `\` + suffix
			dos = filepath.Join(dos, suffix)
		}
		pointer, err := windows.UTF16PtrFromString(enrollmentPhysical(physical))
		if err != nil {
			return a, err
		}
		access := uint32(windows.FILE_LIST_DIRECTORY | windows.FILE_READ_ATTRIBUTES | windows.READ_CONTROL)
		flags := uint32(windows.FILE_FLAG_OPEN_REPARSE_POINT | windows.FILE_FLAG_BACKUP_SEMANTICS)
		if i == 2 {
			access, flags = windows.GENERIC_READ|windows.READ_CONTROL, windows.FILE_FLAG_OPEN_REPARSE_POINT
		}
		handle, err := windows.CreateFile(pointer, access, windows.FILE_SHARE_READ|windows.FILE_SHARE_DELETE, nil, windows.OPEN_EXISTING, flags, 0)
		if err != nil {
			return a, err
		}
		// dos stays empty on these pins: the private alias check below MUST permit
		// the broker's DELETE owner. Generic enrollment alias checks do not.
		pin := &enrollmentPin{handle: handle, physical: physical, broker: parent.broker, file: i == 2}
		a.pins[i], a.dos[i] = pin, dos
		pin.info, pin.id, err = enrollmentInfo(handle)
		if err != nil {
			return a, err
		}
		if pin.file {
			pin.length = int64(uint64(pin.info.FileSizeHigh)<<32 | uint64(pin.info.FileSizeLow))
			if pin.length < 1 || pin.length > maxBrokerBytes {
				return a, errors.New("bounded original startup launcher required")
			}
		}
		if pin.info.VolumeSerialNumber != parent.info.VolumeSerialNumber {
			return a, errors.New("startup anchor crossed original volume")
		}
		if err := enrollmentSecurity(handle, nil, false, pin.broker); err != nil {
			return a, err
		}
		pin.descriptor, err = windows.GetSecurityInfo(handle, windows.SE_FILE_OBJECT, windows.OWNER_SECURITY_INFORMATION|windows.DACL_SECURITY_INFORMATION)
		if err != nil {
			return a, err
		}
	}
	return a, a.check(ctx)
}

func startupAnchorAlias(pin *enrollmentPin, dos string) (resultErr error) {
	if err := pin.check(); err != nil {
		return err
	}
	pointer, err := windows.UTF16PtrFromString(dos)
	if err != nil {
		return err
	}
	access := uint32(windows.FILE_LIST_DIRECTORY | windows.FILE_READ_ATTRIBUTES | windows.READ_CONTROL)
	flags := uint32(windows.FILE_FLAG_OPEN_REPARSE_POINT | windows.FILE_FLAG_BACKUP_SEMANTICS)
	if pin.file {
		access, flags = windows.GENERIC_READ|windows.READ_CONTROL, windows.FILE_FLAG_OPEN_REPARSE_POINT
	}
	handle, err := windows.CreateFile(pointer, access, windows.FILE_SHARE_READ|windows.FILE_SHARE_WRITE|windows.FILE_SHARE_DELETE, nil, windows.OPEN_EXISTING, flags, 0)
	if err != nil {
		return err
	}
	defer func() { resultErr = errors.Join(resultErr, enrollmentCleanup(windows.CloseHandle(handle))) }()
	_, id, err := enrollmentInfo(handle)
	if err != nil || id != pin.id {
		return errors.Join(errors.New("startup anchor DOS identity differs"), err)
	}
	name, err := enrollmentPhysicalPath(handle)
	if err != nil || name != pin.physical {
		return errors.Join(errors.New("startup anchor DOS location differs"), err)
	}
	return nil
}

func (a *startupRetirementAnchor) checkPins(ctx context.Context, pins [3]*enrollmentPin, dos [3]string) error {
	if ctx == nil || ctx.Err() != nil || a.parent == nil {
		return errors.New("live original startup anchor required")
	}
	if err := a.parent.check(); err != nil {
		return err
	}
	for i, pin := range pins {
		if a.drained && i > 0 {
			if pin != nil {
				return errors.New("drained original startup descendants remain held")
			}
			continue
		}
		if pin == nil {
			return errors.New("complete original startup anchor required")
		}
		if err := startupAnchorAlias(pin, dos[i]); err != nil {
			return err
		}
	}
	if !a.drained {
		image, err := startupFileIdentity(ctx, pins[2], a.launcher.SHA256)
		if err != nil || image != a.launcher {
			return errors.Join(errors.New("original startup launcher identity differs"), err)
		}
	}
	return errors.Join(a.parent.check(), ctx.Err())
}

func (a *startupRetirementAnchor) check(ctx context.Context) error {
	if a == nil || a.closed || a.failed {
		return errors.New("original startup anchor revoked or closed")
	}
	if err := a.checkPins(ctx, a.pins, a.dos); err != nil {
		a.failed = true
		return err
	}
	return nil
}

// drain closes only the original app/launcher descendants after their last
// full identity/hash check. The original root and parent stay held throughout.
func (a *startupRetirementAnchor) drain(ctx context.Context, guard func(context.Context) error) (resultErr error) {
	if a == nil || a.closed || a.failed || a.bound || a.drainAttempted || guard == nil {
		return errors.New("fresh original startup descendant drain required")
	}
	a.drainAttempted = true
	defer func() {
		if resultErr != nil {
			a.failed = true
		}
	}()
	if err := errors.Join(guard(ctx), a.check(ctx)); err != nil {
		return err
	}
	for i := len(a.pins) - 1; i > 0; i-- {
		if err := errors.Join(ctx.Err(), guard(ctx), a.parent.check(), startupAnchorAlias(a.pins[0], a.dos[0])); err != nil {
			return err
		}
		if a.pins[i] == nil {
			return errors.New("original startup descendant missing before drain")
		}
		if err := a.pins[i].check(); err != nil {
			return err
		}
		if err := enrollmentCleanup(windows.CloseHandle(a.pins[i].handle)); err != nil {
			return err
		}
		a.pins[i] = nil
		if err := errors.Join(ctx.Err(), guard(ctx), a.parent.check(), startupAnchorAlias(a.pins[0], a.dos[0])); err != nil {
			return err
		}
	}
	a.drained = true
	return errors.Join(guard(ctx), a.check(ctx))
}

// Called only after MaintenanceOwner has checked its sealed native uninstall
// witness against the SAME original broker process and approved registration.
// There is no path fallback or existing retirement receipt import. Rebind only
// the original ROOT; its alias open corroborates it and never supplies bytes.
// Drained descendants are neither reopened nor adopted at the retired path.
func (a *startupRetirementAnchor) bind(ctx context.Context, snapshot brokerpipe.DecommissionSnapshot) (resultErr error) {
	if a == nil || a.closed || a.failed || a.bound || !a.drained || a.parent == nil {
		return errors.New("fresh original startup retirement binding required")
	}
	defer func() {
		if resultErr != nil {
			a.failed = true
		}
	}()
	if !startupRetirementID(snapshot.Generation) || !startupRetirementID(snapshot.RequestID) || !validPin(snapshot.RetirementSHA256) {
		return errors.New("completed original lineage retirement required")
	}
	// The sealed Store witness covers the exact closed original lineage. Its
	// selected generation may be B1; this retained startup root is still B0.
	// Rebind only that original B0 identity at its fixed retirement destination.
	// No selected alternative or caller path can choose a different root.
	name := "retired-" + snapshot.RequestID + "-" + a.generation
	physical, dos := a.parent.physical+`\`+name, filepath.Join(a.parent.dos, name)
	if a.pins[0] == nil || a.pins[1] != nil || a.pins[2] != nil {
		return errors.New("same original root and drained descendants required")
	}
	pin := *a.pins[0]
	pin.physical = physical
	info, id, err := enrollmentInfo(pin.handle)
	if err != nil || id != pin.id {
		return errors.Join(errors.New("retired original generation identity differs"), err)
	}
	// Only the renamed root can tunnel creation time.
	pin.info.CreationTime = info.CreationTime
	rebound, aliases := [3]*enrollmentPin{&pin}, [3]string{dos}
	if err := a.checkPins(ctx, rebound, aliases); err != nil {
		return err
	}
	*a.pins[0] = *rebound[0]
	a.dos, a.bound = aliases, true
	return nil
}

func (a *startupRetirementAnchor) close() error {
	if a == nil {
		return nil
	}
	if !a.closed {
		a.closed = true
		for i := len(a.pins) - 1; i >= 0; i-- {
			if a.pins[i] != nil {
				a.closeErr = errors.Join(a.closeErr, enrollmentCleanup(windows.CloseHandle(a.pins[i].handle)))
			}
		}
	}
	return a.closeErr
}
