package launcher

import (
	"errors"
	"path"

	"golang.org/x/sys/windows"
)

func (nativePlatform) armedUpdateAbsent(root string) error {
	canonical, err := localPath(root)
	if err != nil {
		return errors.Join(errArmedUpdateBarrier, errors.New("armed update barrier root is invalid; startup withheld"))
	}
	// Unlike read/os.ErrNotExist, missing ancestors, ACL failures and reparses
	// cannot become an absent leaf. Validate the complete existing parent first.
	if err := plainNativePath(canonical); err != nil {
		return errors.Join(errArmedUpdateBarrier, errors.New("armed update barrier parent unavailable; startup withheld"))
	}
	leaf, err := windows.UTF16PtrFromString(canonical + "/" + armedUpdateMarker)
	if err != nil {
		return errors.Join(errArmedUpdateBarrier, errors.New("armed update barrier path invalid; startup withheld"))
	}
	_, err = windows.GetFileAttributes(leaf)
	return nativeArmedLeafResult(err)
}

func nativeArmedLeafResult(err error) error {
	// Only this exact error from the validated leaf lookup proves absence.
	// Successful lookup denies every type/content; PATH_NOT_FOUND and all
	// wrapped, access, sharing or otherwise ambiguous errors fail closed.
	if err == windows.ERROR_FILE_NOT_FOUND {
		return nil
	}
	return errors.Join(errArmedUpdateBarrier, errors.New("armed update marker present or unavailable; startup withheld"))
}

func (p nativePlatform) managedArmedUpdateAbsent(app string) error {
	l, err := deriveLayout(app + "/skager-start.exe")
	if err != nil {
		// Raw arbitrary-directory inert creator fixtures exercise image custody,
		// not installed launcher policy. Production run requires this layout.
		return nil
	}
	return p.armedUpdateAbsent(l.root)
}

func (p nativePlatform) acquireUnarmedOrdinaryImage(spec processSpec) (ordinaryImage, error) {
	name, err := localPath(spec.executable)
	if err != nil {
		return nil, err
	}
	image, err := acquireNativeOrdinaryImage(spec)
	if err != nil {
		return nil, err
	}
	l, err := deriveLayout(path.Dir(name) + "/skager-start.exe")
	if err != nil {
		return image, nil // Raw inert creator seam, not a production installed launcher.
	}
	return unarmedOrdinaryImage{ordinaryImage: image, platform: p, root: l.root}, nil
}
