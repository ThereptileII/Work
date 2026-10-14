//go:build windows

package brokerenroll

import (
	"context"
	"errors"
	"runtime"
	"unsafe"

	"golang.org/x/sys/windows"
)

const installedCodeRefreshLockName = ".installed-code-refresh.lock"

// This reusable zero-byte coordination file conveys no approval. Share mode zero
// serializes all customer root selection/New/Refresh using the original cache.
// It is never deleted, reset or retried; process exit releases the native lock.
type nativeInstalledCodeRefresh struct {
	pin      *enrollmentPin
	closed   bool
	closeErr error
}

func openInstalledCodeRefreshNative(ctx context.Context, cache *nativeEnrollmentFiles) (result *nativeInstalledCodeRefresh, resultErr error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if ctx == nil || ctx.Err() != nil || cache == nil || cache.bootstrapCache == nil {
		return nil, errors.New("retained original cache refresh owner required")
	}
	if err := cache.Check(); err != nil {
		return nil, err
	}
	parent := cache.bootstrapCache
	sd, err := windows.SecurityDescriptorFromString("O:BAD:P(A;;FA;;;SY)(A;;FA;;;BA)")
	if err != nil {
		return nil, err
	}
	pointer, err := windows.UTF16PtrFromString(enrollmentPhysical(parent.physical + `\` + installedCodeRefreshLockName))
	if err != nil {
		return nil, err
	}
	sa := windows.SecurityAttributes{Length: uint32(unsafe.Sizeof(windows.SecurityAttributes{})), SecurityDescriptor: sd}
	h, err := windows.CreateFile(pointer, windows.GENERIC_READ|windows.GENERIC_WRITE|windows.READ_CONTROL, 0, &sa, windows.OPEN_ALWAYS, windows.FILE_FLAG_OPEN_REPARSE_POINT, 0)
	if err != nil {
		return nil, errors.New("original installed cache refresh is busy or unavailable")
	}
	n := &nativeInstalledCodeRefresh{pin: &enrollmentPin{handle: h, file: true, descriptor: sd}}
	defer func() {
		if resultErr != nil {
			resultErr = errors.Join(resultErr, n.close())
		}
	}()
	n.pin.info, n.pin.id, err = enrollmentInfo(h)
	if err != nil {
		return nil, err
	}
	n.pin.physical, err = enrollmentPhysicalPath(h)
	if err != nil || n.pin.physical != parent.physical+`\`+installedCodeRefreshLockName || n.pin.info.VolumeSerialNumber != parent.info.VolumeSerialNumber {
		return nil, errors.Join(errors.New("cache refresh lock escaped original directory"), err)
	}
	// Leave dos unset: another pathname open would conflict with this share-zero
	// owner. The retained original parent and handle physical identity bind it.
	if err = errors.Join(n.check(ctx), cache.Check()); err != nil {
		return nil, err
	}
	return n, nil
}
func (n *nativeInstalledCodeRefresh) check(ctx context.Context) error {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if n == nil || n.closed || n.pin == nil || ctx == nil || ctx.Err() != nil {
		return errors.New("live exclusive original cache refresh required")
	}
	return errors.Join(n.pin.check(), ctx.Err())
}
func (n *nativeInstalledCodeRefresh) close() error {
	if n == nil {
		return nil
	}
	if n.closed {
		return n.closeErr
	}
	n.closed = true
	if n.pin != nil {
		n.closeErr = windows.CloseHandle(n.pin.handle)
	}
	return n.closeErr
}
