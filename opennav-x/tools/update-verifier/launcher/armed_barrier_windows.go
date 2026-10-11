package launcher

import (
	"errors"
	"path"
	"strings"

	"golang.org/x/sys/windows"
)

func (nativePlatform) armedUpdateAbsent(root string) error {
	canonical, err := localPath(root)
	if err != nil {
		return errors.Join(errArmedUpdateBarrier, errors.New("armed update barrier root is invalid; startup withheld"))
	}
	// Resolve the DOS drive once. A later remap cannot select a different parent
	// or leaf within this lookup. This is not identity binding across calls.
	drive, err := openNativeArmedDirectory(canonical[:3])
	if err != nil {
		return errors.Join(errArmedUpdateBarrier, err)
	}
	defer windows.CloseHandle(drive.handle)
	return nativeArmedCapturedDriveAbsent(drive, canonical)
}

func nativeArmedCapturedDriveAbsent(drive nativeOrdinaryDirectory, canonical string) error {
	physicalRoot := drive.name
	if len(canonical) > 3 {
		physicalRoot += `\` + strings.ReplaceAll(canonical[3:], "/", `\`)
	}
	_, parents, err := physicalAncestors(physicalRoot + `\` + armedUpdateMarker)
	if err != nil || len(parents) > 256 {
		return errors.Join(errArmedUpdateBarrier, errors.New("armed barrier physical ancestry unavailable or outside bound"))
	}
	pins := make([]nativeOrdinaryDirectory, 0, len(parents))
	defer func() {
		for i := len(pins) - 1; i >= 0; i-- {
			windows.CloseHandle(pins[i].handle)
		}
	}()
	driveMatched := false
	for i, parent := range parents {
		spelling := physicalWin32(parent)
		if i == 0 {
			spelling += `\`
		}
		pin, err := openNativeArmedDirectory(spelling)
		if err != nil {
			return errors.Join(errArmedUpdateBarrier, err)
		}
		pins = append(pins, pin)
		if !strings.EqualFold(pin.name, parent) {
			return errors.Join(errArmedUpdateBarrier, errors.New("armed barrier physical ancestor differs"))
		}
		if strings.EqualFold(parent, drive.name) {
			if !sameNativeIdentity(drive.info, pin.info) {
				return errors.Join(errArmedUpdateBarrier, errors.New("armed barrier captured drive identity differs"))
			}
			driveMatched = true
		}
	}
	if !driveMatched {
		return errors.Join(errArmedUpdateBarrier, errors.New("armed barrier drive is outside physical ancestry"))
	}
	return nativeArmedPhysicalLeafAbsent(physicalRoot)
}

func openNativeArmedDirectory(name string) (nativeOrdinaryDirectory, error) {
	pointer, err := windows.UTF16PtrFromString(name)
	if err != nil {
		return nativeOrdinaryDirectory{}, err
	}
	handle, err := windows.CreateFile(pointer, windows.FILE_READ_ATTRIBUTES,
		windows.FILE_SHARE_READ|windows.FILE_SHARE_WRITE, nil, windows.OPEN_EXISTING,
		windows.FILE_FLAG_BACKUP_SEMANTICS|windows.FILE_FLAG_OPEN_REPARSE_POINT, 0)
	if err != nil {
		return nativeOrdinaryDirectory{}, err
	}
	pin := nativeOrdinaryDirectory{handle: handle}
	pin.name, err = nativePhysicalName(handle)
	if err != nil || windows.GetFileInformationByHandle(handle, &pin.info) != nil ||
		pin.info.FileAttributes&(windows.FILE_ATTRIBUTE_DIRECTORY|windows.FILE_ATTRIBUTE_REPARSE_POINT) != windows.FILE_ATTRIBUTE_DIRECTORY {
		windows.CloseHandle(handle)
		return nativeOrdinaryDirectory{}, errors.New("armed barrier requires an existing plain physical directory")
	}
	pin.name = strings.TrimSuffix(pin.name, `\`)
	return pin, nil
}

func nativeArmedPhysicalLeafAbsent(root string) error {
	leaf, err := windows.UTF16PtrFromString(physicalWin32(root) + `\` + armedUpdateMarker)
	if err != nil {
		return errors.Join(errArmedUpdateBarrier, err)
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
	owner := image.(*nativeOrdinaryImage)
	root, err := managedPhysicalInstallationRoot(owner.name, l.generation)
	if err != nil {
		image.Close()
		return nil, err
	}
	// The exact physical installation root must be among this image's held
	// ancestors. These pins outlive both final checks, create and resume/abort.
	for _, parent := range owner.parents {
		if strings.EqualFold(parent.name, root) {
			return unarmedOrdinaryImage{ordinaryImage: image,
				checkArmed: func() error { return nativeArmedPhysicalLeafAbsent(root) }}, nil
		}
	}
	image.Close()
	return nil, errors.Join(errArmedUpdateBarrier, errors.New("held image lacks physical installation root custody"))
}
