//go:build windows

package brokerenroll

import (
	"context"
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"errors"
	"io"
	"os"
	"path/filepath"
	"runtime"
	"strings"

	"example.com/opennav-update-verifier/brokerregistration"
	"golang.org/x/sys/windows"
)

type nativeStartupPublication struct {
	files                *nativeEnrollmentFiles
	clientSID            string
	image                *enrollmentPin
	imageSHA             string
	link, original       *enrollmentPin
	linkSHA, originalSHA string
}

func (s *nativeStartupPublication) Check(ctx context.Context) error {
	if ctx == nil || ctx.Err() != nil {
		return errors.New("live startup custody context required")
	}
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if err := startupCurrentSID(s.clientSID); err != nil {
		return err
	}
	if err := s.files.Check(); err != nil {
		return err
	}
	for _, item := range []struct {
		pin *enrollmentPin
		sha string
	}{{s.image, s.imageSHA}, {s.link, s.linkSHA}, {s.original, s.originalSHA}} {
		if item.pin == nil {
			continue
		}
		if err := startupHashPin(ctx, item.pin, item.sha); err != nil {
			return err
		}
	}
	return nil
}
func (s *nativeStartupPublication) Close() error { return s.files.Close() }

// User menu/profile custody pins preserve namespace/physical IDs, not launch or
// selection authority. The current user's writable profile is not a protected
// trust source. Only the approved original baseline and original SCM enrollment
// authorize this explicit publication; authenticated broker dispatch remains a
// separate mandatory check when the shortcut is later invoked.
func startupHoldAncestry(f *nativeEnrollmentFiles, dos, allowedSID string) (*enrollmentPin, error) {
	if len(dos) < 4 || filepath.Clean(dos) != dos || dos[1:3] != `:\` || strings.ContainsAny(dos[3:], ":/\x00") {
		return nil, errors.New("fixed local KnownFolder required")
	}
	parent, err := f.pinDirectory(dos[:3], "", dos[:3], nil, true, allowedSID)
	if err != nil {
		return nil, err
	}
	parts, err := enrollmentPhysicalParts(parent.physical)
	if err != nil || len(parts) != 3 {
		return nil, errors.New("startup KnownFolder drive alias refused")
	}
	var serial, component, flags uint32
	fs := make([]uint16, 32)
	if err := windows.GetVolumeInformationByHandle(parent.handle, nil, 0, &serial, &component, &flags, &fs[0], uint32(len(fs))); err != nil || windows.UTF16ToString(fs) != "NTFS" || serial != parent.info.VolumeSerialNumber {
		return nil, errors.New("native NTFS startup volume required")
	}
	physical, path := parent.physical, dos[:2]
	for _, part := range strings.Split(dos[3:], `\`) {
		if part == "" || part == "." || part == ".." || strings.HasSuffix(part, ".") || strings.HasSuffix(part, " ") {
			return nil, errors.New("ambiguous startup ancestry")
		}
		physical += `\` + part
		path += `\` + part
		parent, err = f.pinDirectory(enrollmentPhysical(physical), physical, path, nil, false, allowedSID)
		if err != nil {
			return nil, err
		}
		if parent.info.VolumeSerialNumber != serial {
			return nil, errors.New("startup ancestry crossed volumes")
		}
	}
	return parent, nil
}

func startupOpenFile(f *nativeEnrollmentFiles, parent *enrollmentPin, name string, shareDelete bool, max int64) (*enrollmentPin, error) {
	physical := parent.physical + `\` + name
	p, err := windows.UTF16PtrFromString(enrollmentPhysical(physical))
	if err != nil {
		return nil, err
	}
	share := uint32(windows.FILE_SHARE_READ)
	if shareDelete {
		share |= windows.FILE_SHARE_DELETE
	}
	h, err := windows.CreateFile(p, windows.GENERIC_READ|windows.READ_CONTROL, share, nil, windows.OPEN_EXISTING, windows.FILE_FLAG_OPEN_REPARSE_POINT, 0)
	if err != nil {
		return nil, err
	}
	pin := &enrollmentPin{handle: h, physical: physical, dos: filepath.Join(parent.dos, name), file: true, broker: parent.broker}
	f.pins = append(f.pins, pin)
	pin.info, pin.id, err = enrollmentInfo(h)
	if err != nil {
		return nil, err
	}
	pin.length = int64(uint64(pin.info.FileSizeHigh)<<32 | uint64(pin.info.FileSizeLow))
	if pin.length < 1 || pin.length > max {
		return nil, errors.New("bounded startup file required")
	}
	pin.descriptor, err = windows.GetSecurityInfo(h, windows.SE_FILE_OBJECT, windows.OWNER_SECURITY_INFORMATION|windows.DACL_SECURITY_INFORMATION)
	if err != nil {
		return nil, err
	}
	// Capture exact descriptor only after checking allowed principals. Original
	// user shortcut bytes are evidence for rollback, never executable authority.
	if err := enrollmentSecurity(h, nil, false, pin.broker); err != nil {
		return nil, err
	}
	return pin, pin.check()
}
func startupHashPin(ctx context.Context, pin *enrollmentPin, want string) (result error) {
	if pin == nil || !validPin(want) {
		return errors.New("retained startup byte identity required")
	}
	if err := pin.check(); err != nil {
		return err
	}
	// ReadAt on the retained native handle; no path reopen supplies trusted bytes.
	file, err := startupDuplicateFile(pin.handle)
	if err != nil {
		return err
	}
	defer func() { result = errors.Join(result, enrollmentCleanup(file.Close())) }()
	hash := sha256.New()
	var off int64
	buffer := make([]byte, 64<<10)
	for off < pin.length {
		if err := ctx.Err(); err != nil {
			return err
		}
		nbytes := int64(len(buffer))
		if pin.length-off < nbytes {
			nbytes = pin.length - off
		}
		n, err := file.ReadAt(buffer[:int(nbytes)], off)
		if n != int(nbytes) || err != nil && !(errors.Is(err, io.EOF) && off+nbytes == pin.length) {
			return errors.Join(errors.New("retained startup bytes unreadable"), err)
		}
		_, _ = hash.Write(buffer[:n])
		off += int64(n)
	}
	if hex.EncodeToString(hash.Sum(nil)) != want {
		return errors.New("retained startup bytes differ from original identity")
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	return pin.check()
}

func startupWriteFile(ctx context.Context, f *nativeEnrollmentFiles, parent *enrollmentPin, name string, raw []byte, sd *windows.SECURITY_DESCRIPTOR, shareDelete bool) (*enrollmentPin, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	file, pin, err := f.newFile(parent, name, sd)
	if err != nil {
		return nil, err
	}
	n, writeErr := file.Write(raw)
	syncErr := file.Sync()
	closeErr := file.Close()
	if writeErr != nil || syncErr != nil || closeErr != nil || n != len(raw) {
		return nil, errors.Join(errors.New("startup evidence write uncertain"), writeErr, syncErr, enrollmentCleanup(closeErr))
	}
	// Fresh recovery parent excludes ordinary writers from birth. Reopen and
	// compare full original IDs before attaching its retained read-only handle.
	reopened, err := startupOpenFile(f, parent, name, shareDelete, maxStartupShortcutBytes)
	if err != nil {
		return nil, err
	}
	if reopened.id != pin.id || reopened.info.CreationTime != pin.info.CreationTime {
		return nil, errors.New("fresh startup evidence identity changed")
	}
	if err := startupHashPin(ctx, reopened, digest(raw)); err != nil {
		return nil, err
	}
	return reopened, nil
}

func publishStartupNative(ctx context.Context, original enrollmentFiles, client *brokerregistration.Request, baseline ApprovedClientBaseline) (publication startupPublication, result error) {
	if ctx == nil || ctx.Err() != nil || client == nil || original == nil {
		return nil, errors.New("original startup enrollment required")
	}
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if err := startupCurrentSID(client.ClientSID); err != nil {
		return nil, err
	}
	if err := original.Check(); err != nil {
		return nil, err
	}
	sha, size, err := startupLauncherIdentity(ctx, client, baseline)
	if err != nil {
		return nil, err
	}
	owner := &nativeStartupPublication{files: &nativeEnrollmentFiles{}, clientSID: client.ClientSID, imageSHA: sha}
	publication = owner // even partial owners must be drained; partial namespace stays.
	broker, err := enrollmentBrokerSID()
	if err != nil {
		return publication, err
	}
	programFiles, err := windows.KnownFolderPath(windows.FOLDERID_ProgramFilesX86, 0)
	if err != nil {
		return publication, err
	}
	app := filepath.Join(programFiles, "SKAGER", "Applications", client.ClientSID, "generations", client.Installed.Generation, "app")
	imageParent, err := startupHoldAncestry(owner.files, app, broker.String())
	if err != nil {
		return publication, err
	}
	owner.image, err = startupOpenFile(owner.files, imageParent, "skager-start.exe", false, maxBrokerBytes)
	if err != nil {
		return publication, err
	}
	if owner.image.length != size {
		return publication, errors.New("original baseline launcher length differs")
	}
	if err := startupHashPin(ctx, owner.image, sha); err != nil {
		return publication, err
	}
	programs, err := windows.KnownFolderPath(windows.FOLDERID_Programs, 0)
	if err != nil {
		return publication, err
	}
	local, err := windows.KnownFolderPath(windows.FOLDERID_LocalAppData, 0)
	if err != nil {
		return publication, err
	}
	menuParent, err := startupHoldAncestry(owner.files, programs, client.ClientSID)
	if err != nil {
		return publication, err
	}
	recoveryParent, err := startupHoldAncestry(owner.files, local, client.ClientSID)
	if err != nil {
		return publication, err
	}
	sd, err := windows.SecurityDescriptorFromString("O:BAD:P(A;OICI;FA;;;SY)(A;OICI;FA;;;BA)(A;OICI;FRFX;;;" + client.ClientSID + ")")
	if err != nil {
		return publication, err
	}
	// Never adopt or replace an unknown recovery namespace, even when empty.
	recovery, err := owner.files.newDirectory(recoveryParent, startupRecoveryDirectory, sd, client.ClientSID)
	if err != nil {
		return publication, err
	}
	menuPhysical := menuParent.physical + `\SKAGER`
	menu, err := owner.files.pinDirectory(enrollmentPhysical(menuPhysical), menuPhysical, filepath.Join(menuParent.dos, "SKAGER"), nil, false, client.ClientSID)
	if errors.Is(err, windows.ERROR_FILE_NOT_FOUND) || errors.Is(err, windows.ERROR_PATH_NOT_FOUND) {
		menu, err = owner.files.newDirectory(menuParent, "SKAGER", sd, client.ClientSID)
	}
	if err != nil {
		return publication, err
	}
	// Preserve original file IDs through same-volume, non-replacing renames.
	if menu.id.Volume != recovery.id.Volume {
		return publication, errors.New("startup menu and recovery must share native volume")
	}
	owner.original, err = startupOpenFile(owner.files, menu, startupShortcutName, true, maxStartupShortcutBytes)
	if errors.Is(err, windows.ERROR_FILE_NOT_FOUND) {
		owner.original = nil
		err = nil
	}
	if err != nil {
		return publication, err
	}
	type originalEvidence struct {
		Schema   int              `json:"schema"`
		Present  bool             `json:"present"`
		ID       enrollmentFileID `json:"originalFileID"`
		Creation windows.Filetime `json:"originalCreation"`
		SHA256   string           `json:"originalSHA256"`
	}
	evidence := originalEvidence{Schema: 1, Present: owner.original != nil}
	if owner.original != nil {
		bytes, err := startupReadOriginal(ctx, owner.original)
		if err != nil {
			return publication, err
		}
		owner.originalSHA = digest(bytes)
		evidence.ID = owner.original.id
		evidence.Creation = owner.original.info.CreationTime
		evidence.SHA256 = owner.originalSHA
		if err := startupHashPin(ctx, owner.original, owner.originalSHA); err != nil {
			return publication, err
		}
		if _, err := startupWriteFile(ctx, owner.files, recovery, "original.lnk.bytes", bytes, sd, false); err != nil {
			return publication, err
		}
	}
	evidenceBytes, err := json.Marshal(evidence)
	if err != nil {
		return publication, err
	}
	if _, err := startupWriteFile(ctx, owner.files, recovery, "original.json", evidenceBytes, sd, false); err != nil {
		return publication, err
	}
	raw, err := startupLinkBytes(owner.image.dos, "--broker-start="+client.Installed.Generation, app)
	if err != nil {
		return publication, err
	}
	owner.linkSHA = digest(raw)
	owner.link, err = startupWriteFile(ctx, owner.files, recovery, "pending.lnk", raw, sd, true)
	if err != nil {
		return publication, err
	}
	if err := owner.files.Check(); err != nil {
		return publication, err
	}
	if err := original.Check(); err != nil {
		return publication, err
	}
	if err := baseline.Validate(); err != nil {
		return publication, err
	}
	if err := startupCurrentSID(client.ClientSID); err != nil {
		return publication, err
	}
	if err := ctx.Err(); err != nil {
		return publication, err
	}
	if err := startupPublishRenames(ctx, owner, menu, recovery); err != nil {
		return publication, err
	}
	return publication, owner.Check(ctx)
}
func startupReadOriginal(ctx context.Context, p *enrollmentPin) (raw []byte, result error) {
	if err := p.check(); err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	raw = make([]byte, int(p.length))
	file, err := startupDuplicateFile(p.handle)
	if err != nil {
		return nil, err
	}
	defer func() { result = errors.Join(result, enrollmentCleanup(file.Close())) }()
	n, err := file.ReadAt(raw, 0)
	if n != len(raw) || err != nil {
		return nil, errors.Join(errors.New("original shortcut unreadable"), err)
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	return raw, p.check()
}

func startupDuplicateFile(handle windows.Handle) (*os.File, error) {
	var duplicate windows.Handle
	if err := windows.DuplicateHandle(windows.CurrentProcess(), handle, windows.CurrentProcess(), &duplicate, 0, false, windows.DUPLICATE_SAME_ACCESS); err != nil {
		return nil, err
	}
	return os.NewFile(uintptr(duplicate), "owned duplicate startup read"), nil
}

// No path or command is supplied by callers of the public enrollment API.
func startupPublishRenames(ctx context.Context, owner *nativeStartupPublication, menu, recovery *enrollmentPin) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	if err := owner.files.Check(); err != nil {
		return err
	}
	destination := menu.physical + `\` + startupShortcutName
	dest, _ := windows.UTF16PtrFromString(enrollmentPhysical(destination))
	pending, _ := windows.UTF16PtrFromString(enrollmentPhysical(owner.link.physical))
	if owner.original != nil {
		backupPhysical := recovery.physical + `\` + startupShortcutName
		backup, _ := windows.UTF16PtrFromString(enrollmentPhysical(backupPhysical))
		// Neither rename replaces a destination. If interrupted after the first,
		// the original native file and a separately synced exact byte copy survive
		// in recovery; a new/unknown menu entry is never overwritten or adopted.
		if err := windows.MoveFileEx(dest, backup, windows.MOVEFILE_WRITE_THROUGH); err != nil {
			return errors.Join(ErrStartupUncertain, err)
		}
		owner.original.physical = backupPhysical
		owner.original.dos = filepath.Join(recovery.dos, startupShortcutName)
		if err := owner.files.Check(); err != nil {
			return errors.Join(ErrStartupUncertain, err)
		}
		if err := startupHashPin(ctx, owner.original, owner.originalSHA); err != nil {
			return errors.Join(ErrStartupUncertain, err)
		}
	}
	if err := ctx.Err(); err != nil {
		return errors.Join(ErrStartupUncertain, err)
	}
	if err := windows.MoveFileEx(pending, dest, windows.MOVEFILE_WRITE_THROUGH); err != nil {
		return errors.Join(ErrStartupUncertain, err)
	}
	owner.link.physical = destination
	owner.link.dos = filepath.Join(menu.dos, startupShortcutName)
	return nil
}
