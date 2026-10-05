package verifier

import (
	"errors"
	"golang.org/x/sys/windows"
	"path/filepath"
	"unsafe"
)

func privateStateSD() (*windows.SECURITY_DESCRIPTOR, error) {
	u, err := windows.GetCurrentProcessToken().GetTokenUser()
	if err != nil {
		return nil, err
	}
	return windows.SecurityDescriptorFromString("D:P(A;OICI;FA;;;" + u.User.Sid.String() + ")(A;OICI;FA;;;SY)")
}
func makePrivateStateDirectory(path string) error {
	sd, err := privateStateSD()
	if err != nil {
		return err
	}
	// Elevated Windows tokens may default new object ownership to the
	// Administrators group. The per-user store must explicitly belong to the
	// current user, under both elevated CI and ordinary unelevated installs.
	u, err := windows.GetCurrentProcessToken().GetTokenUser()
	if err != nil {
		return err
	}
	sd, err = windows.SecurityDescriptorFromString("O:" + u.User.Sid.String() + sd.String())
	if err != nil {
		return err
	}
	p, err := windows.UTF16PtrFromString(path)
	if err != nil {
		return err
	}
	sa := windows.SecurityAttributes{Length: uint32(unsafe.Sizeof(windows.SecurityAttributes{})), SecurityDescriptor: sd}
	if err := windows.CreateDirectory(p, &sa); err != nil {
		return err
	}
	return assertPrivateStateDirectory(path)
}
func assertPrivateStateDirectory(path string) error {
	if err := rejectStateReparse(path); err != nil {
		return err
	}
	expected, err := privateStateSD()
	if err != nil {
		return err
	}
	actual, err := windows.GetNamedSecurityInfo(path, windows.SE_FILE_OBJECT, windows.DACL_SECURITY_INFORMATION)
	if err != nil {
		return err
	}
	if actual.String() != expected.String() {
		return errors.New("update directory permissions changed")
	}
	sd, err := windows.GetNamedSecurityInfo(path, windows.SE_FILE_OBJECT, windows.OWNER_SECURITY_INFORMATION)
	if err != nil {
		return err
	}
	owner, _, err := sd.Owner()
	if err != nil {
		return err
	}
	user, err := windows.GetCurrentProcessToken().GetTokenUser()
	if err != nil {
		return err
	}
	if !owner.Equals(user.User.Sid) {
		return errors.New("update directory owner changed")
	}
	return nil
}
func rejectStateReparse(path string) error {
	p, err := windows.UTF16PtrFromString(path)
	if err != nil {
		return err
	}
	a, err := windows.GetFileAttributes(p)
	if err != nil {
		return err
	}
	if a&windows.FILE_ATTRIBUTE_REPARSE_POINT != 0 {
		return errors.New("update path reparse points are unavailable")
	}
	return nil
}
func lockState(directory string) (func(), error) {
	p, err := windows.UTF16PtrFromString(filepath.Join(directory, "state.lock"))
	if err != nil {
		return nil, err
	}
	h, err := windows.CreateFile(p, windows.GENERIC_READ|windows.GENERIC_WRITE, 0, nil, windows.OPEN_ALWAYS, windows.FILE_FLAG_OPEN_REPARSE_POINT, 0)
	if err != nil {
		return nil, errors.New("update state is busy or unavailable")
	}
	var info windows.ByHandleFileInformation
	if err := windows.GetFileInformationByHandle(h, &info); err != nil || info.FileAttributes&windows.FILE_ATTRIBUTE_REPARSE_POINT != 0 {
		windows.CloseHandle(h)
		return nil, errors.New("invalid update lock")
	}
	return func() { _ = windows.CloseHandle(h) }, nil
}
func replaceStateFile(from, to string) error {
	a, err := windows.UTF16PtrFromString(from)
	if err != nil {
		return err
	}
	b, err := windows.UTF16PtrFromString(to)
	if err != nil {
		return err
	}
	return windows.MoveFileEx(a, b, windows.MOVEFILE_REPLACE_EXISTING|windows.MOVEFILE_WRITE_THROUGH)
}

// Metadata files and the pointer use FlushFileBuffers/WRITE_THROUGH. Windows
// does not expose POSIX directory fsync through an ordinary directory handle.
func syncStateDirectory(string) error { return nil }
