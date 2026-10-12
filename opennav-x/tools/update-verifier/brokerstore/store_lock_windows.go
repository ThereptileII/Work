package brokerstore

import (
	"errors"
	"unsafe"

	"golang.org/x/sys/windows"
)

const storeLockFilename = "store.lock"

// A surviving lock file is only a synchronization object. Its existence/content
// grants no recovery, candidate, no-child or launch authority. The exclusive
// data-access handle, not the pathname or an abandoned marker, owns the lock.
type nativeStoreLock struct {
	handle   windows.Handle
	parent   *nativePin
	name     string
	info     windows.ByHandleFileInformation
	security *windows.SECURITY_DESCRIPTOR
}

func acquireStoreLock(root *nativePin, security *windows.SECURITY_DESCRIPTOR) (result *nativeStoreLock, resultErr error) {
	if root == nil || root.security == nil || security == nil {
		return nil, errors.New("protected physical broker root required for store lock")
	}
	if err := root.check(); err != nil {
		return nil, err
	}
	name := root.name + `\` + storeLockFilename
	pointer, err := windows.UTF16PtrFromString(physical(name))
	if err != nil {
		return nil, err
	}
	attributes := windows.SecurityAttributes{Length: uint32(unsafe.Sizeof(windows.SecurityAttributes{})), SecurityDescriptor: security}
	access := uint32(windows.GENERIC_READ | windows.GENERIC_WRITE | windows.READ_CONTROL)
	handle, err := windows.CreateFile(pointer, access, 0, &attributes, windows.CREATE_NEW, windows.FILE_FLAG_OPEN_REPARSE_POINT, 0)
	created := err == nil
	if errors.Is(err, windows.ERROR_FILE_EXISTS) || errors.Is(err, windows.ERROR_ALREADY_EXISTS) {
		// Never repair/truncate a pre-existing object. Exact protected metadata
		// is required before this handle can serve as the lifetime lock.
		handle, err = windows.CreateFile(pointer, access, 0, nil, windows.OPEN_EXISTING, windows.FILE_FLAG_OPEN_REPARSE_POINT, 0)
	}
	if err != nil {
		return nil, err
	}
	lock := &nativeStoreLock{handle: handle, parent: root, name: name, security: security}
	ok := false
	defer func() {
		if !ok {
			resultErr = errors.Join(resultErr, windows.CloseHandle(handle))
		}
	}()
	if err = windows.GetFileInformationByHandle(handle, &lock.info); err != nil {
		return nil, err
	}
	if err = lock.check(); err != nil {
		return nil, err
	}
	if created {
		if err = windows.FlushFileBuffers(handle); err != nil {
			return nil, err
		}
	}
	ok = true
	return lock, nil
}

func (lock *nativeStoreLock) check() error {
	if err := lock.parent.check(); err != nil {
		return err
	}
	var info windows.ByHandleFileInformation
	if err := windows.GetFileInformationByHandle(lock.handle, &info); err != nil {
		return err
	}
	kind, err := windows.GetFileType(lock.handle)
	if err != nil {
		return err
	}
	name, err := nativeName(lock.handle)
	if err != nil || name != lock.name || !sameIdentity(lock.info, info) ||
		kind != windows.FILE_TYPE_DISK || info.FileAttributes&(windows.FILE_ATTRIBUTE_DIRECTORY|windows.FILE_ATTRIBUTE_REPARSE_POINT) != 0 ||
		info.NumberOfLinks != 1 || info.FileSizeHigh != 0 || info.FileSizeLow != 0 ||
		info.VolumeSerialNumber != lock.parent.info.VolumeSerialNumber {
		return errors.New("store lock physical identity/type/link/size changed or unsafe")
	}
	return checkSecurity(lock.handle, lock.security)
}
