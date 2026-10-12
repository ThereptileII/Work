package launcher

import (
	"context"
	"errors"
	"strings"

	"golang.org/x/sys/windows"
)

// MaintenanceBarrier retains the physical LocalAppData ancestry supplied by the
// authenticated operator-token scope. It provides denial-only legacy marker
// observations; it neither authenticates that scope nor authorizes a transaction,
// cold profile, output, launch, or mutation. Callers must serialize its use.
//
// Validate proves exact absence at its checks. There is no notification watch:
// custody cannot prove a marker never briefly existed between those checks.
// Callers must not promote this owner to continuous absence authority.
type MaintenanceBarrier struct {
	location string
	pins     []nativeOrdinaryDirectory
	closed   bool
	failure  error
	closeErr error
}

// CaptureMaintenanceBarrierNative accepts only the actual operator's token-
// derived LocalAppData, supplied inside WithMaintenanceOperatorLocation. A path
// alone has no public operator authority. No service-user token or query-access
// policy is read or changed, and no legacy installation is adopted/repaired.
func CaptureMaintenanceBarrierNative(ctx context.Context, localAppData string) (result *MaintenanceBarrier, resultErr error) {
	if ctx == nil || ctx.Err() != nil {
		return nil, errors.Join(errArmedUpdateBarrier, contextErrorBroker(ctx))
	}
	if err := plainNativePath(localAppData); err != nil {
		return nil, errors.Join(errArmedUpdateBarrier, err)
	}
	original, err := openNativeArmedDirectory(localAppData)
	if err != nil {
		return nil, errors.Join(errArmedUpdateBarrier, err)
	}
	b := &MaintenanceBarrier{location: localAppData, pins: []nativeOrdinaryDirectory{original}}
	defer func() {
		if resultErr != nil {
			resultErr = errors.Join(resultErr, b.Close())
		}
	}()
	_, parents, err := physicalAncestors(original.name + `\sentinel`)
	if err != nil || len(parents) == 0 || len(parents) > 256 {
		return nil, errors.Join(errArmedUpdateBarrier, err)
	}
	// Preserve the original LocalAppData handle throughout capture. Pin ancestors
	// through GLOBALROOT so DOS aliases/SUBST cannot hide any physical ancestor.
	for index, parent := range parents[:len(parents)-1] {
		name := physicalWin32(parent)
		if index == 0 {
			name += `\`
		}
		pin, err := openNativeArmedDirectory(name)
		if err != nil {
			return nil, errors.Join(errArmedUpdateBarrier, err)
		}
		b.pins = append(b.pins, pin)
		if !strings.EqualFold(pin.name, parent) {
			return nil, errArmedUpdateBarrier
		}
	}
	if !strings.EqualFold(parents[len(parents)-1], original.name) {
		return nil, errArmedUpdateBarrier
	}
	if err = b.Validate(ctx); err != nil {
		return nil, err
	}
	return b, nil
}

func (b *MaintenanceBarrier) Validate(ctx context.Context) (resultErr error) {
	if b != nil {
		if b.failure != nil {
			return b.failure
		}
		defer func() {
			if resultErr != nil {
				b.failure = errors.Join(errArmedUpdateBarrier, resultErr)
			}
		}()
	}
	if b == nil || b.closed || len(b.pins) == 0 || ctx == nil || ctx.Err() != nil {
		return errors.Join(errArmedUpdateBarrier, contextErrorBroker(ctx))
	}
	validate := func() error {
		for _, pin := range b.pins {
			var info windows.ByHandleFileInformation
			name, err := nativePhysicalName(pin.handle)
			infoErr := windows.GetFileInformationByHandle(pin.handle, &info)
			if err != nil || infoErr != nil || !sameNativeIdentity(pin.info, info) || info.FileAttributes&(windows.FILE_ATTRIBUTE_DIRECTORY|windows.FILE_ATTRIBUTE_REPARSE_POINT) != windows.FILE_ATTRIBUTE_DIRECTORY || !strings.EqualFold(strings.TrimSuffix(name, `\`), pin.name) {
				return errors.Join(errArmedUpdateBarrier, err, infoErr)
			}
		}
		// DOS spelling can contain an 8.3 alias: compare actual retained native
		// identities/names, never an unresolved DOS suffix with a normalized name.
		if err := plainNativePath(b.location); err != nil {
			return errors.Join(errArmedUpdateBarrier, err)
		}
		current, err := openNativeArmedDirectory(b.location)
		if err != nil {
			return errors.Join(errArmedUpdateBarrier, err)
		}
		original := b.pins[0]
		same := sameNativeIdentity(original.info, current.info) && strings.EqualFold(original.name, current.name)
		closeErr := windows.CloseHandle(current.handle)
		if !same || closeErr != nil {
			return errors.Join(errArmedUpdateBarrier, closeErr)
		}
		return ctx.Err()
	}
	if err := validate(); err != nil {
		return err
	}
	return errors.Join(brokerExecutionOldMarkerAbsent(b.pins[0].name), validate())
}

// Close only drains original read/query directory custody; it never removes or
// repairs the denial marker or legacy installation. Cleanup is idempotent.
func (b *MaintenanceBarrier) Close() error {
	if b == nil {
		return nil
	}
	if b.closed {
		return b.closeErr
	}
	b.closed = true
	for index := len(b.pins) - 1; index >= 0; index-- {
		b.closeErr = errors.Join(b.closeErr, windows.CloseHandle(b.pins[index].handle))
	}
	b.pins = nil
	if b.closeErr != nil {
		b.closeErr = errors.Join(errArmedUpdateBarrier, b.closeErr)
	}
	return b.closeErr
}
