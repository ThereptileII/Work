//go:build windows

package brokerenroll

import (
	"bytes"
	"context"
	"errors"
	"path/filepath"
	"runtime"
	"unsafe"

	"golang.org/x/sys/windows"
)

type startupRestorationFile struct {
	pin *enrollmentPin
	dos string
}

type nativeStartupRestoration struct {
	files                                    *nativeEnrollmentFiles
	clientSID                                string
	record                                   startupBindingRecord
	menu, recovery, image, manifest, receipt *enrollmentPin
	links                                    [3]startupRestorationFile
	original                                 *startupRestorationFile
	retired                                  [3]bool
	restored, attempted                      bool
	raw                                      []byte
}

func openStartupRestorationNative(ctx context.Context, authority maintenanceAuthority, record startupBindingRecord, raw []byte) (result startupRestorationBackend, resultErr error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	n := &nativeStartupRestoration{files: &nativeEnrollmentFiles{}, clientSID: authority.client.ClientSID, record: record, raw: bytes.Clone(raw)}
	// Even a partial open remains owned until deterministic Close. No namespace
	// creation, descriptor repair or path adoption occurs here.
	result = n
	if err := maintenanceOperatorNative(n.clientSID); err != nil {
		return result, err
	}
	if err := validateStartupBinding(record, authority.client); err != nil {
		return result, err
	}
	programs, err := windows.KnownFolderPath(windows.FOLDERID_Programs, 0)
	if err != nil {
		return result, err
	}
	local, err := windows.KnownFolderPath(windows.FOLDERID_LocalAppData, 0)
	if err != nil {
		return result, err
	}
	menuParent, err := startupHoldAncestry(n.files, programs, n.clientSID)
	if err != nil {
		return result, err
	}
	recoveryParent, err := startupHoldAncestry(n.files, local, n.clientSID)
	if err != nil {
		return result, err
	}
	n.menu, err = maintenanceHoldChild(n.files, menuParent, "SKAGER", nil, n.clientSID)
	if err != nil {
		return result, err
	}
	n.recovery, err = maintenanceHoldChild(n.files, recoveryParent, startupRecoveryDirectory, nil, n.clientSID)
	if err != nil {
		return result, err
	}
	programFiles, err := windows.KnownFolderPath(windows.FOLDERID_ProgramFilesX86, 0)
	if err != nil {
		return result, err
	}
	broker, err := enrollmentBrokerSID()
	if err != nil {
		return result, err
	}
	app := filepath.Join(programFiles, "SKAGER", "Applications", n.clientSID, "generations", record.Generation, "app")
	imageParent, err := startupHoldAncestry(n.files, app, broker.String())
	if err != nil {
		return result, err
	}
	n.image, err = startupOpenFile(n.files, imageParent, "skager-start.exe", false, maxBrokerBytes)
	if err != nil {
		return result, err
	}
	n.manifest, err = startupOpenFile(n.files, n.recovery, "publication.json", false, maxStartupBindingBytes)
	if err != nil {
		return result, err
	}
	n.receipt, err = startupOpenFile(n.files, n.recovery, startupBindingName, false, maxStartupBindingBytes)
	if err != nil {
		return result, err
	}
	for i := range n.links {
		spec, _ := startupShortcutMode(i).spec()
		n.links[i], err = openStartupRestorationFile(n.files, n.menu, spec.name)
		if err != nil {
			return result, err
		}
	}
	if record.Links[0].Original != nil {
		spec, _ := startupNormal.spec()
		original, err := openStartupRestorationFile(n.files, n.recovery, spec.name)
		if err != nil {
			return result, err
		}
		n.original = &original
	}
	return result, n.preflight(ctx)
}

// Retain DELETE access from the first validated original open and exclude all
// other delete/rename handles. The rename uses THIS handle, never a path-opened
// substitute. Enrollment's ordinary DOS check omits FILE_SHARE_DELETE, so moved
// pins leave dos empty and use the explicit sharing-compatible alias check below.
func openStartupRestorationFile(files *nativeEnrollmentFiles, parent *enrollmentPin, name string) (startupRestorationFile, error) {
	dos := filepath.Join(parent.dos, name)
	physical := parent.physical + `\` + name
	pointer, err := windows.UTF16PtrFromString(enrollmentPhysical(physical))
	if err != nil {
		return startupRestorationFile{}, err
	}
	handle, err := windows.CreateFile(pointer, windows.GENERIC_READ|windows.READ_CONTROL|windows.DELETE, windows.FILE_SHARE_READ, nil, windows.OPEN_EXISTING, windows.FILE_FLAG_OPEN_REPARSE_POINT, 0)
	if err != nil {
		return startupRestorationFile{}, err
	}
	pin := &enrollmentPin{handle: handle, physical: physical, file: true, broker: parent.broker}
	files.pins = append(files.pins, pin)
	file := startupRestorationFile{pin, dos}
	pin.info, pin.id, err = enrollmentInfo(handle)
	if err != nil {
		return file, err
	}
	pin.length = int64(uint64(pin.info.FileSizeHigh)<<32 | uint64(pin.info.FileSizeLow))
	if pin.length < 1 || pin.length > maxStartupShortcutBytes {
		return file, errors.New("bounded original restoration shortcut required")
	}
	if err := enrollmentSecurity(handle, nil, false, pin.broker); err != nil {
		return file, err
	}
	pin.descriptor, err = windows.GetSecurityInfo(handle, windows.SE_FILE_OBJECT, windows.OWNER_SECURITY_INFORMATION|windows.DACL_SECURITY_INFORMATION)
	if err != nil {
		return file, err
	}
	return file, file.checkAlias()
}

func (file startupRestorationFile) checkAlias() error {
	if file.pin == nil || file.dos == "" {
		return errors.New("retained restoration shortcut required")
	}
	if err := file.pin.check(); err != nil {
		return err
	}
	pointer, err := windows.UTF16PtrFromString(file.dos)
	if err != nil {
		return err
	}
	handle, err := windows.CreateFile(pointer, windows.GENERIC_READ|windows.READ_CONTROL, windows.FILE_SHARE_READ|windows.FILE_SHARE_WRITE|windows.FILE_SHARE_DELETE, nil, windows.OPEN_EXISTING, windows.FILE_FLAG_OPEN_REPARSE_POINT, 0)
	if err != nil {
		return err
	}
	_, id, identityErr := enrollmentInfo(handle)
	physical, nameErr := enrollmentPhysicalPath(handle)
	closeErr := enrollmentCleanup(windows.CloseHandle(handle))
	if identityErr != nil || nameErr != nil || id != file.pin.id || physical != file.pin.physical {
		return errors.Join(errors.New("restoration shortcut DOS identity differs"), identityErr, nameErr, closeErr)
	}
	return closeErr
}

func (n *nativeStartupRestoration) check(ctx context.Context) error {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if ctx == nil || ctx.Err() != nil || n == nil || n.files == nil {
		return errors.New("live original shortcut custody required")
	}
	if (n.original == nil) != (n.record.Links[0].Original == nil) {
		return errors.New("complete original normal backup custody required")
	}
	if err := maintenanceOperatorNative(n.clientSID); err != nil {
		return err
	}
	if err := n.files.Check(); err != nil {
		return err
	}
	for _, item := range []struct {
		pin  *enrollmentPin
		want startupDirectoryBinding
	}{{n.menu, n.record.Menu}, {n.recovery, n.record.Recovery}} {
		actual, err := startupDirectoryIdentity(item.pin)
		if err != nil || actual != item.want {
			return errors.Join(errors.New("original startup directory identity differs"), err)
		}
	}
	image, err := startupFileIdentity(ctx, n.image, n.record.Launcher.SHA256)
	if err != nil || image != n.record.Launcher {
		return errors.Join(errors.New("original startup launcher identity differs"), err)
	}
	if err := startupHashPin(ctx, n.manifest, n.record.PrepublicationSHA256); err != nil {
		return err
	}
	if err := startupHashPin(ctx, n.receipt, digest(n.raw)); err != nil {
		return err
	}
	for i, file := range n.links {
		spec, _ := startupShortcutMode(i).spec()
		parent, name := n.menu, spec.name
		if n.retired[i] {
			parent, name = n.recovery, startupRetirementName(startupShortcutMode(i))
			if i != 0 || !n.restored {
				if err := startupRestorationAbsent(n.menu, spec.name); err != nil {
					return err
				}
			}
		} else if err := startupRestorationAbsent(n.recovery, startupRetirementName(startupShortcutMode(i))); err != nil {
			return err
		}
		if file.pin == nil || file.pin.physical != parent.physical+`\`+name || file.dos != filepath.Join(parent.dos, name) {
			return errors.New("fixed original restoration shortcut location differs")
		}
		if err := file.checkAlias(); err != nil {
			return err
		}
		actual, err := startupFileIdentity(ctx, file.pin, n.record.Links[i].New.SHA256)
		if err != nil || actual != n.record.Links[i].New {
			return errors.Join(errors.New("original published shortcut identity differs"), err)
		}
	}
	if n.original != nil {
		spec, _ := startupNormal.spec()
		parent := n.recovery
		if n.restored {
			parent = n.menu
		}
		if n.original.pin == nil || n.original.pin.physical != parent.physical+`\`+spec.name || n.original.dos != filepath.Join(parent.dos, spec.name) {
			return errors.New("fixed original normal backup location differs")
		}
		if err := n.original.checkAlias(); err != nil {
			return err
		}
		actual, err := startupFileIdentity(ctx, n.original.pin, n.record.Links[0].Original.SHA256)
		if err != nil || actual != *n.record.Links[0].Original {
			return errors.Join(errors.New("original normal backup identity differs"), err)
		}
	}
	if n.original == nil || n.restored {
		spec, _ := startupNormal.spec()
		if err := startupRestorationAbsent(n.recovery, spec.name); err != nil {
			return err
		}
	}
	return ctx.Err()
}

func startupRetirementName(mode startupShortcutMode) string {
	spec, _ := mode.spec()
	return "retired-" + spec.name
}

// Exact FILE_NOT_FOUND only; directories, aliases, inaccessible/foreign files
// and every prior partial retirement refuse. There is no resume/adopt path.
func startupRestorationAbsent(parent *enrollmentPin, name string) error {
	pointer, err := windows.UTF16PtrFromString(enrollmentPhysical(parent.physical + `\` + name))
	if err != nil {
		return err
	}
	_, err = windows.GetFileAttributes(pointer)
	if errors.Is(err, windows.ERROR_FILE_NOT_FOUND) {
		return nil
	}
	return errors.Join(errors.New("original restoration destination is not absent"), err)
}

func (n *nativeStartupRestoration) preflight(ctx context.Context) error {
	return n.check(ctx)
}

// FILE_RENAME_INFORMATION is architecture-aligned on both Win32 and Win64. A zero
// ReplaceIfExists field guarantees collision refusal. RootDirectory is the
// original retained destination directory and FileName is one fixed component.
// Use the native relative-directory operation: the Win32 rename wrapper refused
// this retained-root form with ERROR_INVALID_PARAMETER on Windows Server 2022.
// https://learn.microsoft.com/en-us/windows-hardware/drivers/ddi/ntifs/ns-ntifs-_file_rename_information
func startupRenameRestorationFile(ctx context.Context, file *startupRestorationFile, destination *enrollmentPin, name, sha string) error {
	if file == nil || file.pin == nil || destination == nil || ctx == nil {
		return errors.New("original handle rename custody required")
	}
	if err := file.checkAlias(); err != nil {
		return err
	}
	if err := destination.check(); err != nil {
		return err
	}
	if err := startupRestorationAbsent(destination, name); err != nil {
		return err
	}
	component, err := windows.UTF16FromString(name)
	if err != nil {
		return err
	}
	component = component[:len(component)-1]
	type renameInfo struct {
		ReplaceIfExists uint32
		RootDirectory   windows.Handle
		FileNameLength  uint32
		FileName        [1]uint16
	}
	offset := unsafe.Offsetof(renameInfo{}.FileName)
	buffer := make([]byte, int(unsafe.Sizeof(renameInfo{}))+len(component)*2)
	header := (*renameInfo)(unsafe.Pointer(&buffer[0]))
	header.RootDirectory, header.FileNameLength = destination.handle, uint32(len(component)*2)
	copy(unsafe.Slice((*uint16)(unsafe.Pointer(&buffer[offset])), len(component)), component)
	if err := ctx.Err(); err != nil {
		return err
	}
	var status windows.IO_STATUS_BLOCK
	if err := windows.NtSetInformationFile(file.pin.handle, &status, &buffer[0], uint32(len(buffer)), windows.FileRenameInformation); err != nil {
		return err
	}
	// Only our successful rename may rebind the original handle's path/creation.
	// NTFS can tunnel the vacated normal name's creation time during restoration.
	info, id, err := enrollmentInfo(file.pin.handle)
	if err != nil || id != file.pin.id || info.VolumeSerialNumber != file.pin.info.VolumeSerialNumber {
		return errors.Join(errors.New("renamed original shortcut identity differs"), err)
	}
	rebound := *file.pin
	rebound.physical = destination.physical + `\` + name
	rebound.info.CreationTime = info.CreationTime
	candidate := startupRestorationFile{&rebound, filepath.Join(destination.dos, name)}
	if err := candidate.checkAlias(); err != nil {
		return err
	}
	if err := startupHashPin(ctx, &rebound, sha); err != nil {
		return err
	}
	*file.pin = rebound
	file.dos = candidate.dos
	return nil
}

func (n *nativeStartupRestoration) restore(ctx context.Context, guard func(context.Context) error) error {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if n == nil || n.attempted || guard == nil {
		return errors.New("fresh original shortcut restoration required")
	}
	n.attempted = true
	return startupRestoreSequence(ctx, n.original != nil, func() error {
		if err := guard(ctx); err != nil {
			return err
		}
		return n.preflight(ctx)
	}, func(step int) error {
		if err := guard(ctx); err != nil {
			return err
		}
		if err := n.check(ctx); err != nil {
			return err
		}
		if step < 3 {
			file := &n.links[step]
			if err := startupRenameRestorationFile(ctx, file, n.recovery, startupRetirementName(startupShortcutMode(step)), n.record.Links[step].New.SHA256); err != nil {
				return err
			}
			n.retired[step] = true
			n.record.Links[step].New.Creation = uint64(file.pin.info.CreationTime.HighDateTime)<<32 | uint64(file.pin.info.CreationTime.LowDateTime)
		} else {
			spec, _ := startupNormal.spec()
			if err := startupRenameRestorationFile(ctx, n.original, n.menu, spec.name, n.record.Links[0].Original.SHA256); err != nil {
				return err
			}
			n.restored = true
			n.record.Links[0].Original.Creation = uint64(n.original.pin.info.CreationTime.HighDateTime)<<32 | uint64(n.original.pin.info.CreationTime.LowDateTime)
		}
		if err := n.check(ctx); err != nil {
			return err
		}
		return guard(ctx)
	})
}

func (n *nativeStartupRestoration) close() error {
	if n == nil || n.files == nil {
		return nil
	}
	return n.files.Close()
}
