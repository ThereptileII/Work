package brokerhost

import (
	"context"
	"crypto/sha256"
	"encoding/binary"
	"encoding/hex"
	"errors"
	"io"
	"os"
	"path/filepath"
	"regexp"
	"runtime"
	"sort"
	"strings"
	"unsafe"

	"example.com/opennav-update-verifier/brokerstore"
	"golang.org/x/sys/windows"
)

type nativeProfileCaptureMember struct {
	entry    profileCaptureEntry
	file     *os.File
	physical string
}
type nativeProfileCaptureWatch struct {
	handle, event   windows.Handle
	overlapped      windows.Overlapped
	buffer          [64 << 10]byte
	pending, closed bool
	closeErr        error
	phase           string
}

// Diagnostics remain private: Error never prints profile member names. Only
// inert native fixture tests inspect notifications to identify unexpected I/O.
type nativeProfileWatchNotification struct {
	action uint32
	name   string
}
type nativeProfileWatchDiagnostic struct {
	stage         string
	bytes         uint32
	notifications []nativeProfileWatchNotification
	malformed     bool
	cause         error
}

func (d *nativeProfileWatchDiagnostic) Error() string {
	return "profile changed during frozen native custody"
}
func (d *nativeProfileWatchDiagnostic) Unwrap() error { return d.cause }

func profileWatchNotifications(buffer []byte) (events []nativeProfileWatchNotification, malformed bool) {
	for offset := 0; offset < len(buffer); {
		if len(buffer)-offset < 12 {
			return events, true
		}
		next := binary.LittleEndian.Uint32(buffer[offset:])
		action := binary.LittleEndian.Uint32(buffer[offset+4:])
		nameBytes := binary.LittleEndian.Uint32(buffer[offset+8:])
		if nameBytes%2 != 0 || uint64(nameBytes) > uint64(len(buffer)-offset-12) {
			return events, true
		}
		units := make([]uint16, nameBytes/2)
		for index := range units {
			units[index] = binary.LittleEndian.Uint16(buffer[offset+12+index*2:])
		}
		events = append(events, nativeProfileWatchNotification{action: action, name: windows.UTF16ToString(units)})
		if next == 0 {
			return events, offset+12+int(nameBytes) > len(buffer)
		}
		if next < 12+nameBytes || next%4 != 0 || uint64(next) >= uint64(len(buffer)-offset) {
			return events, true
		}
		offset += int(next)
	}
	return events, true
}

type nativeProfileCapture struct {
	fixed, physical string
	ancestors       []executionProfilePin
	members         []*nativeProfileCaptureMember
	watch           *nativeProfileCaptureWatch
	checkProcesses  bool
	frozen, closed  bool
	closeErr        error
}

var profileApplicationProcess = regexp.MustCompile(`(?i)^(opencpn|oeserver\w*|oexserver\w*|rtl_ais|rtl_fm|aisdecoder|ais-?catcher|hackrf_transfer)\.exe$`)

func profileNoNamedProcesses(ctx context.Context) (resultErr error) {
	h, err := windows.CreateToolhelp32Snapshot(windows.TH32CS_SNAPPROCESS, 0)
	if err != nil {
		return err
	}
	defer func() { resultErr = errors.Join(resultErr, windows.CloseHandle(h)) }()
	var e windows.ProcessEntry32
	e.Size = uint32(unsafe.Sizeof(e))
	err = windows.Process32First(h, &e)
	for err == nil {
		if err := ctx.Err(); err != nil {
			return err
		}
		if profileApplicationProcess.MatchString(windows.UTF16ToString(e.ExeFile[:])) {
			return errors.New("OpenCPN or navigation/plugin helper is still running; close normally before profile custody")
		}
		err = windows.Process32Next(h, &e)
	}
	if !errors.Is(err, windows.ERROR_NO_MORE_FILES) {
		return err
	}
	return nil
}
func captureProfileTreeNative(ctx context.Context) (profileCaptureSource, error) {
	root, err := windows.KnownFolderPath(windows.FOLDERID_ProgramData, 0)
	if err != nil {
		return nil, err
	}
	return captureProfileTreePath(ctx, filepath.Join(root, "opencpn"), true)
}
func captureProfileFile(name string, directory bool) (*os.File, error) {
	return captureProfileFileMode(name, directory, false)
}
func captureProfileFileMode(name string, directory, writable bool) (*os.File, error) {
	p, err := windows.UTF16PtrFromString(name)
	if err != nil {
		return nil, err
	}
	access := uint32(windows.GENERIC_READ | windows.READ_CONTROL)
	share := uint32(windows.FILE_SHARE_READ)
	flags := uint32(windows.FILE_FLAG_OPEN_REPARSE_POINT)
	if directory {
		access = windows.FILE_LIST_DIRECTORY | windows.FILE_READ_ATTRIBUTES | windows.READ_CONTROL
		share |= windows.FILE_SHARE_WRITE
		flags |= windows.FILE_FLAG_BACKUP_SEMANTICS
	}
	if writable {
		access |= windows.FILE_WRITE_ATTRIBUTES
		if !directory {
			access |= windows.GENERIC_WRITE
		}
	}
	h, err := windows.CreateFile(p, access, share, nil, windows.OPEN_EXISTING, flags, 0)
	if err != nil {
		return nil, err
	}
	f := os.NewFile(uintptr(h), "fixed-profile-native-member")
	if f == nil {
		return nil, errors.Join(ErrExecutionProfileCaptureRefused, windows.CloseHandle(h))
	}
	return f, nil
}
func profileDefaultStreams(h windows.Handle, directory bool) error {
	var buf [64 << 10]byte
	err := windows.GetFileInformationByHandleEx(h, 7, &buf[0], uint32(len(buf)))
	if directory && errors.Is(err, windows.ERROR_HANDLE_EOF) {
		return nil
	}
	if err != nil {
		return err
	}
	next := binary.LittleEndian.Uint32(buf[:4])
	length := binary.LittleEndian.Uint32(buf[4:8])
	if next != 0 || length == 0 || length > uint32(len(buf)-24) || length%2 != 0 {
		return errors.New("named/unknown profile streams require review")
	}
	name := unsafe.Slice((*uint16)(unsafe.Pointer(&buf[24])), int(length/2))
	if windows.UTF16ToString(name) != "::$DATA" || (directory && binary.LittleEndian.Uint64(buf[8:16]) != 0) {
		return errors.New("named/unknown profile stream refused")
	}
	return nil
}
func profileMemberMetadata(f *os.File, name string, directory bool) (profileCaptureEntry, error) {
	h := windows.Handle(f.Fd())
	id, err := executionProfileIdentity(h, directory)
	if err != nil {
		return profileCaptureEntry{}, err
	}
	var info windows.ByHandleFileInformation
	if err = windows.GetFileInformationByHandle(h, &info); err != nil {
		return profileCaptureEntry{}, err
	}
	allowed := uint32(windows.FILE_ATTRIBUTE_READONLY | windows.FILE_ATTRIBUTE_HIDDEN | windows.FILE_ATTRIBUTE_SYSTEM | windows.FILE_ATTRIBUTE_ARCHIVE | windows.FILE_ATTRIBUTE_NORMAL | windows.FILE_ATTRIBUTE_NOT_CONTENT_INDEXED)
	if directory {
		allowed |= windows.FILE_ATTRIBUTE_DIRECTORY
	}
	if info.FileAttributes == 0 || info.FileAttributes & ^allowed != 0 {
		return profileCaptureEntry{}, errors.New("specialized profile attributes require review")
	}
	if err = profileDefaultStreams(h, directory); err != nil {
		return profileCaptureEntry{}, err
	}
	sd, err := windows.GetSecurityInfo(h, windows.SE_FILE_OBJECT, windows.OWNER_SECURITY_INFORMATION|windows.GROUP_SECURITY_INFORMATION|windows.DACL_SECURITY_INFORMATION)
	if err != nil {
		return profileCaptureEntry{}, err
	}
	// Read existing access security only. No ACL is changed; audit metadata is not
	// claimed captured and cannot be reconstructed on missing/recreated originals.
	security := sd.String()
	if security == "" {
		return profileCaptureEntry{}, ErrExecutionProfileCaptureRefused
	}
	ticks := func(t windows.Filetime) uint64 { return uint64(t.HighDateTime)<<32 | uint64(t.LowDateTime) }
	size := int64(uint64(info.FileSizeHigh)<<32 | uint64(info.FileSizeLow))
	if directory {
		size = 0
	}
	if size < 0 || size > brokerstore.MaxFileBytes {
		return profileCaptureEntry{}, ErrExecutionProfileCaptureRefused
	}
	return profileCaptureEntry{Path: name, Directory: directory, Bytes: size, Identity: id.wire(), Attributes: info.FileAttributes, Created: ticks(info.CreationTime), LastAccess: ticks(info.LastAccessTime), LastWrite: ticks(info.LastWriteTime), Security: security}, nil
}
func profileFileDigest(ctx context.Context, f *os.File, size int64) (string, error) {
	reader := io.NewSectionReader(f, 0, size+1)
	hash := sha256.New()
	buf := make([]byte, 64<<10)
	var count int64
	for {
		if err := ctx.Err(); err != nil {
			return "", err
		}
		n, err := reader.Read(buf)
		count += int64(n)
		_, _ = hash.Write(buf[:n])
		if count > size {
			return "", ErrExecutionProfileCaptureRefused
		}
		if errors.Is(err, io.EOF) {
			break
		}
		if err != nil {
			return "", err
		}
	}
	if count != size {
		return "", ErrExecutionProfileCaptureRefused
	}
	return hex.EncodeToString(hash.Sum(nil)), nil
}
func startProfileWatch(physical string) (result *nativeProfileCaptureWatch, resultErr error) {
	p, err := windows.UTF16PtrFromString(`\\?\GLOBALROOT` + physical)
	if err != nil {
		return nil, err
	}
	h, err := windows.CreateFile(p, windows.FILE_LIST_DIRECTORY, windows.FILE_SHARE_READ|windows.FILE_SHARE_WRITE, nil, windows.OPEN_EXISTING, windows.FILE_FLAG_OPEN_REPARSE_POINT|windows.FILE_FLAG_BACKUP_SEMANTICS|windows.FILE_FLAG_OVERLAPPED, 0)
	if err != nil {
		return nil, err
	}
	w := &nativeProfileCaptureWatch{handle: h, phase: "watch"}
	defer func() {
		if resultErr != nil {
			resultErr = errors.Join(resultErr, w.close(false))
		}
	}()
	w.event, err = windows.CreateEvent(nil, 1, 0, nil)
	if err != nil {
		return nil, err
	}
	w.overlapped.HEvent = w.event
	mask := uint32(windows.FILE_NOTIFY_CHANGE_FILE_NAME | windows.FILE_NOTIFY_CHANGE_DIR_NAME | windows.FILE_NOTIFY_CHANGE_ATTRIBUTES | windows.FILE_NOTIFY_CHANGE_SIZE | windows.FILE_NOTIFY_CHANGE_LAST_WRITE | windows.FILE_NOTIFY_CHANGE_CREATION | windows.FILE_NOTIFY_CHANGE_SECURITY)
	err = windows.ReadDirectoryChanges(h, &w.buffer[0], uint32(len(w.buffer)), true, mask, nil, &w.overlapped, 0)
	if err != nil && !errors.Is(err, windows.ERROR_IO_PENDING) {
		return nil, err
	}
	w.pending = true
	return w, nil
}
func (w *nativeProfileCaptureWatch) check() error {
	return w.checkAt("check")
}
func (w *nativeProfileCaptureWatch) checkAt(stage string) error {
	if w == nil || w.closed || !w.pending {
		return ErrExecutionProfileCaptureRefused
	}
	var count uint32
	err := windows.GetOverlappedResult(w.handle, &w.overlapped, &count, false)
	if errors.Is(err, windows.ERROR_IO_INCOMPLETE) {
		return nil
	}
	// A notification, overflow, completed zero-byte read or I/O failure all refuse.
	diagnostic := &nativeProfileWatchDiagnostic{stage: w.phase + "/" + stage, bytes: count, cause: err}
	if err == nil && count > 0 {
		if count <= uint32(len(w.buffer)) {
			diagnostic.notifications, diagnostic.malformed = profileWatchNotifications(w.buffer[:count])
		} else {
			diagnostic.malformed = true
		}
	}
	return diagnostic
}
func (w *nativeProfileCaptureWatch) close(strict bool) error {
	if w == nil {
		return nil
	}
	if w.closed {
		return w.closeErr
	}
	w.closed = true
	if w.pending {
		err := windows.CancelIoEx(w.handle, &w.overlapped)
		if err != nil && !errors.Is(err, windows.ERROR_NOT_FOUND) {
			w.closeErr = errors.Join(w.closeErr, err)
		}
		var count uint32
		err = windows.GetOverlappedResult(w.handle, &w.overlapped, &count, true)
		if !errors.Is(err, windows.ERROR_OPERATION_ABORTED) {
			if strict || err != nil {
				w.closeErr = errors.Join(w.closeErr, errors.New("profile watch completed with changes or failed while releasing custody"), err)
			}
		}
		w.pending = false
	}
	if w.event != 0 {
		w.closeErr = errors.Join(w.closeErr, windows.CloseHandle(w.event))
	}
	if w.handle != 0 {
		w.closeErr = errors.Join(w.closeErr, windows.CloseHandle(w.handle))
	}
	return w.closeErr
}
func profileDirectoryNames(member *nativeProfileCaptureMember) (names []string, resultErr error) {
	f, err := captureProfileFile(`\\?\GLOBALROOT`+member.physical, true)
	if err != nil {
		return nil, err
	}
	defer func() { resultErr = errors.Join(resultErr, f.Close()) }()
	id, err := executionProfileIdentity(windows.Handle(f.Fd()), true)
	if err != nil || id.wire() != member.entry.Identity {
		return nil, errors.Join(ErrExecutionProfileCaptureRefused, err)
	}
	entries, err := f.ReadDir(brokerstore.MaxFiles + 1)
	if err != nil && !errors.Is(err, io.EOF) {
		return nil, err
	}
	if len(entries) > brokerstore.MaxFiles {
		return nil, ErrExecutionProfileCaptureRefused
	}
	for _, e := range entries {
		names = append(names, e.Name())
	}
	sort.Strings(names)
	return names, nil
}

// Private path seam is solely for inert native fixtures. Production always uses
// fixed KnownFolder ProgramData/opencpn after same-Store backup reservation.
func captureProfileTreePath(ctx context.Context, path string, checkProcesses bool) (profileCaptureSource, error) {
	return captureProfileTreePathMode(ctx, path, checkProcesses, false)
}
func captureProfileTreePathMode(ctx context.Context, path string, checkProcesses, writable bool) (result profileCaptureSource, resultErr error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if ctx == nil {
		return nil, ErrExecutionProfileCaptureRefused
	}
	if err := executionNoImpersonation(); err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if checkProcesses {
		if err := profileNoNamedProcesses(ctx); err != nil {
			return nil, err
		}
	}
	b := &nativeProfileCapture{fixed: path, checkProcesses: checkProcesses, frozen: true}
	defer func() {
		if resultErr != nil {
			resultErr = errors.Join(resultErr, b.close())
		}
	}()
	root, err := captureProfileFile(path, true)
	if err != nil {
		return nil, err
	}
	physical, err := stockPhysical(windows.Handle(root.Fd()))
	closeErr := root.Close()
	if err != nil || closeErr != nil {
		return nil, errors.Join(err, closeErr)
	}
	parts := strings.Split(strings.TrimRight(physical, `\`), `\`)
	if len(parts) < 4 || !physicalVolume.MatchString(strings.Join(parts[:3], `\`)) {
		return nil, ErrExecutionProfileCaptureRefused
	}
	for count := 3; count <= len(parts); count++ {
		if err = ctx.Err(); err != nil {
			return nil, err
		}
		name := strings.Join(parts[:count], `\`)
		openName := `\\?\GLOBALROOT` + name
		if count == 3 {
			openName += `\`
		}
		h, err := openStock(openName, true)
		if err != nil {
			return nil, err
		}
		b.ancestors = append(b.ancestors, executionProfilePin{handle: h, physical: name})
		pin := &b.ancestors[len(b.ancestors)-1]
		pin.id, err = executionProfileIdentity(h, true)
		if err != nil {
			return nil, err
		}
		actual, err := stockPhysical(h)
		if err != nil || !strings.EqualFold(strings.TrimRight(actual, `\`), name) {
			return nil, errors.Join(ErrExecutionProfileCaptureRefused, err)
		}
	}
	b.physical = strings.TrimRight(physical, `\`)
	var fs [32]uint16
	var serial, component, flags uint32
	if err = windows.GetVolumeInformationByHandle(b.ancestors[len(b.ancestors)-1].handle, nil, 0, &serial, &component, &flags, &fs[0], uint32(len(fs))); err != nil || windows.UTF16ToString(fs[:]) != "NTFS" {
		return nil, errors.Join(errors.New("native profile requires physical NTFS custody"), err)
	}
	pending := []string{"."}
	known := map[string]bool{}
	var total int64
	for len(pending) > 0 {
		relative := pending[0]
		pending = pending[1:]
		if !profileCapturePath(relative) || len(b.members) >= brokerstore.MaxFiles || known[strings.ToLower(relative)] {
			return nil, ErrExecutionProfileCaptureRefused
		}
		known[strings.ToLower(relative)] = true
		name := b.physical
		if relative != "." {
			name += `\` + strings.ReplaceAll(relative, "/", `\`)
		}
		// Resolve type without following reparse points, then require full metadata
		// from the retained ordinary directory/file handle before any recursion/read.
		pointer, err := windows.UTF16PtrFromString(`\\?\GLOBALROOT` + name)
		if err != nil {
			return nil, err
		}
		attributes, err := windows.GetFileAttributes(pointer)
		if err != nil {
			return nil, err
		}
		directory := attributes&windows.FILE_ATTRIBUTE_DIRECTORY != 0
		file, err := captureProfileFileMode(`\\?\GLOBALROOT`+name, directory, writable && (directory || relative == "opencpn.ini" || relative == "opencpn.log"))
		if err != nil {
			return nil, err
		}
		member := &nativeProfileCaptureMember{file: file, physical: name}
		b.members = append(b.members, member)
		member.entry, err = profileMemberMetadata(file, relative, directory)
		if err != nil {
			return nil, err
		}
		actual, err := stockPhysical(windows.Handle(file.Fd()))
		if err != nil || actual != name {
			return nil, errors.Join(ErrExecutionProfileCaptureRefused, err)
		}
		if directory {
			names, err := profileDirectoryNames(member)
			if err != nil {
				return nil, err
			}
			for _, child := range names {
				rel := child
				if relative != "." {
					rel = relative + "/" + child
				}
				pending = append(pending, rel)
			}
		} else {
			if member.entry.Bytes > brokerstore.MaxPayloadBytes-total {
				return nil, ErrExecutionProfileCaptureRefused
			}
			total += member.entry.Bytes
			member.entry.SHA256, err = profileFileDigest(ctx, file, member.entry.Bytes)
			if err != nil {
				return nil, err
			}
		}
	}
	sort.Slice(b.members, func(i, j int) bool {
		if b.members[i].entry.Path == "." || b.members[j].entry.Path == "." {
			return b.members[i].entry.Path == "." && b.members[j].entry.Path != "."
		}
		return b.members[i].entry.Path < b.members[j].entry.Path
	})
	// The first walk is only a candidate snapshot. Opening and enumerating its
	// members can complete pending directory metadata writes; watching that walk
	// makes those writes indistinguishable from external profile changes. Arm the
	// watch after retaining all member handles, then recheck the entire snapshot
	// under the watch before accepting custody. Any event from that point refuses.
	b.watch, err = startProfileWatch(b.physical)
	if err != nil {
		return nil, err
	}
	b.watch.phase = "initial capture"
	if err = b.validate(ctx, true); err != nil {
		return nil, err
	}
	b.watch.phase = "frozen custody"
	return b, nil
}
func (b *nativeProfileCapture) originals() ([]profileCaptureEntry, []string) {
	entries := make([]profileCaptureEntry, 0, len(b.members))
	for _, m := range b.members {
		entries = append(entries, m.entry)
	}
	var parents []string
	for _, p := range b.ancestors {
		parents = append(parents, p.id.wire())
	}
	return entries, parents
}
func (b *nativeProfileCapture) readers() ([]io.Reader, error) {
	if b.closed || !b.frozen {
		return nil, ErrExecutionProfileCaptureRefused
	}
	var readers []io.Reader
	for _, m := range b.members {
		if !m.entry.Directory {
			readers = append(readers, io.NewSectionReader(m.file, 0, m.entry.Bytes))
		}
	}
	return readers, nil
}
func (b *nativeProfileCapture) validate(ctx context.Context, frozen bool) error {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if ctx == nil || b.closed {
		return ErrExecutionProfileCaptureRefused
	}
	if err := executionNoImpersonation(); err != nil {
		return err
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	for _, p := range b.ancestors {
		id, err := executionProfileIdentity(p.handle, true)
		if err != nil || id != p.id {
			return errors.Join(ErrExecutionProfileCaptureRefused, err)
		}
		actual, err := stockPhysical(p.handle)
		if err != nil || !strings.EqualFold(strings.TrimRight(actual, `\`), p.physical) {
			return errors.Join(ErrExecutionProfileCaptureRefused, err)
		}
	}
	if !frozen {
		return ctx.Err()
	}
	if !b.frozen {
		return ErrExecutionProfileCaptureRefused
	}
	if err := b.watch.checkAt("validate begin"); err != nil {
		return err
	}
	if b.checkProcesses {
		if err := profileNoNamedProcesses(ctx); err != nil {
			return err
		}
	}
	expected := map[string]bool{}
	for _, m := range b.members {
		expected[m.entry.Path] = true
	}
	observed := map[string]bool{".": true}
	for _, m := range b.members {
		actual, err := profileMemberMetadata(m.file, m.entry.Path, m.entry.Directory)
		if err != nil {
			return err
		}
		before := m.entry
		before.LastAccess = 0
		actual.LastAccess = 0
		actual.SHA256 = before.SHA256
		if actual != before {
			return errors.New("original profile metadata/identity/security changed")
		}
		if m.entry.Directory {
			names, err := profileDirectoryNames(m)
			if err != nil {
				return err
			}
			for _, name := range names {
				relative := name
				if m.entry.Path != "." {
					relative = m.entry.Path + "/" + name
				}
				if !expected[relative] || observed[relative] {
					return errors.New("profile namespace changed")
				}
				observed[relative] = true
			}
		} else {
			hash, err := profileFileDigest(ctx, m.file, m.entry.Bytes)
			if err != nil || hash != m.entry.SHA256 {
				return errors.Join(errors.New("original entire-profile bytes changed"), err)
			}
		}
	}
	if len(observed) != len(expected) {
		return errors.New("original profile member disappeared")
	}
	return errors.Join(b.watch.checkAt("validate end"), ctx.Err())
}
func (b *nativeProfileCapture) release(ctx context.Context) error {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if err := b.validate(ctx, true); err != nil {
		return err
	}
	if err := b.watch.close(true); err != nil {
		return err
	}
	b.watch = nil
	var err error
	for _, m := range b.members {
		err = errors.Join(err, m.file.Close())
		m.file = nil
	}
	b.frozen = false
	return err
}
func (b *nativeProfileCapture) inspect(ctx context.Context) (profileCaptureSource, error) {
	if b.frozen {
		return nil, ErrExecutionProfileCaptureRefused
	}
	if err := b.validate(ctx, false); err != nil {
		return nil, err
	}
	source, err := captureProfileTreePath(ctx, b.fixed, b.checkProcesses)
	if err != nil {
		return nil, err
	}
	current := source.(*nativeProfileCapture)
	if len(current.ancestors) != len(b.ancestors) {
		return nil, errors.Join(ErrExecutionProfileCaptureRefused, current.close())
	}
	for index, p := range current.ancestors {
		if p.id != b.ancestors[index].id {
			return nil, errors.Join(errors.New("profile fixed ancestry changed before inspection"), current.close())
		}
	}
	return source, nil
}
func (b *nativeProfileCapture) close() error {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if b.closed {
		return b.closeErr
	}
	b.closed = true
	b.closeErr = errors.Join(b.closeErr, b.watch.close(false))
	b.watch = nil
	for _, m := range b.members {
		if m.file != nil {
			b.closeErr = errors.Join(b.closeErr, m.file.Close())
			m.file = nil
		}
	}
	for index := len(b.ancestors) - 1; index >= 0; index-- {
		b.closeErr = errors.Join(b.closeErr, windows.CloseHandle(b.ancestors[index].handle))
	}
	b.ancestors = nil
	return b.closeErr
}
