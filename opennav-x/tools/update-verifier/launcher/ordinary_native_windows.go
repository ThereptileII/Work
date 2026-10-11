package launcher

import (
	"errors"
	"os"
	"path"
	"sort"
	"strings"
	"unicode/utf16"
	"unsafe"

	"golang.org/x/sys/windows"
)

// Only an acquired NTFS handle supplies the physical name. CreateProcess needs
// one compatible DOS application spelling derived from that same held handle.
// Its namespace is not pinned: lookups and the suspended child's native image
// must corroborate the physical owner before resume; there is no path retry.
// QueryFullProcessImageName is corroboration, not a mapped-section file-ID query.
type nativeOrdinaryImage struct {
	file        *os.File
	info        windows.ByHandleFileInformation
	name        string
	application string
	expected    string
	parents     []nativeOrdinaryDirectory
}

type nativeOrdinaryDirectory struct {
	handle windows.Handle
	info   windows.ByHandleFileInformation
	name   string
}

func nativePhysicalName(handle windows.Handle) (string, error) {
	buffer := make([]uint16, 32768)
	n, err := windows.GetFinalPathNameByHandle(handle, &buffer[0], uint32(len(buffer)), 2) // VOLUME_NAME_NT, FILE_NAME_NORMALIZED
	if err != nil || n == 0 || n >= uint32(len(buffer)) {
		return "", errors.New("physical file name unavailable or outside bound")
	}
	name := windows.UTF16ToString(buffer[:n])
	if _, _, err := physicalAncestors(name); err != nil {
		return "", err
	}
	return name, nil
}

// This deliberately supports local HarddiskVolume NT names only. The root is
// always included, followed by every physical directory above the image; a SUBST
// root cannot conceal an ancestor. Root-only trailing slashes are normalized.
func physicalAncestors(name string) (string, []string, error) {
	parts := strings.Split(name, `\`)
	if len(parts) < 3 || parts[0] != "" || !strings.EqualFold(parts[1], "Device") ||
		!strings.HasPrefix(parts[2], "HarddiskVolume") {
		return "", nil, errors.New("local physical NTFS volume name required")
	}
	digits := strings.TrimPrefix(parts[2], "HarddiskVolume")
	if len(digits) == 0 || len(digits) > 10 {
		return "", nil, errors.New("unsupported physical volume name")
	}
	for _, c := range digits {
		if c < '0' || c > '9' {
			return "", nil, errors.New("unsupported physical volume name")
		}
	}
	root := `\Device\` + parts[2]
	if len(parts) == 4 && parts[3] == "" {
		return root, []string{root}, nil
	}
	if len(parts) == 3 {
		return root, []string{root}, nil
	}
	parents := []string{root}
	current := root
	for i, component := range parts[3:] {
		if !safeSegment(component) || len(utf16.Encode([]rune(component))) > 255 {
			return "", nil, errors.New("unsupported physical path component")
		}
		current += `\` + component
		if i < len(parts[3:])-1 {
			parents = append(parents, current)
		}
	}
	return root, parents, nil
}

func physicalWin32(name string) string { return `\\?\GLOBALROOT` + name }

func ordinaryApplicationPath(name, physical string) (string, error) {
	// Strip only the exact extended-path prefix returned by the held handle.
	name = strings.TrimPrefix(name, `\\?\`)
	canonical, err := localPath(name)
	if err != nil {
		return "", errors.New("ordinary application requires a local DOS spelling")
	}
	root, _, err := physicalAncestors(physical)
	canonical = strings.ReplaceAll(canonical, "/", `\`)
	if err != nil || !strings.EqualFold(canonical[2:], physical[len(root):]) {
		return "", errors.New("ordinary DOS application suffix differs from held physical image")
	}
	return canonical, nil
}

func nativeOrdinaryApplication(handle windows.Handle, physical string) (string, error) {
	buffer := make([]uint16, 32768)
	n, err := windows.GetFinalPathNameByHandle(handle, &buffer[0], uint32(len(buffer)), 0) // VOLUME_NAME_DOS, FILE_NAME_NORMALIZED
	if err != nil || n == 0 || n >= uint32(len(buffer)) {
		return "", errors.New("held ordinary DOS application name unavailable or outside bound")
	}
	return ordinaryApplicationPath(windows.UTF16ToString(buffer[:n]), physical)
}

func sameNativeIdentity(a, b windows.ByHandleFileInformation) bool {
	return a.VolumeSerialNumber == b.VolumeSerialNumber && a.FileIndexHigh == b.FileIndexHigh &&
		a.FileIndexLow == b.FileIndexLow && a.CreationTime == b.CreationTime
}

func sameNativeImage(a, b windows.ByHandleFileInformation) bool {
	return sameNativeIdentity(a, b) && a.FileAttributes == b.FileAttributes &&
		b.FileAttributes&(windows.FILE_ATTRIBUTE_DIRECTORY|windows.FILE_ATTRIBUTE_REPARSE_POINT) == 0 &&
		b.NumberOfLinks == 1 && a.NumberOfLinks == b.NumberOfLinks &&
		a.FileSizeHigh == b.FileSizeHigh && a.FileSizeLow == b.FileSizeLow && a.LastWriteTime == b.LastWriteTime
}

func acquireNativeOrdinaryImage(spec processSpec) (ordinaryImage, error) {
	name, err := localPath(spec.executable)
	if err != nil {
		return nil, err
	}
	directory, err := localPath(spec.directory)
	if err != nil || !strings.EqualFold(path.Dir(name), directory) || !strings.EqualFold(path.Base(name), "opencpn.exe") {
		return nil, errors.New("ordinary application image/directory mismatch")
	}
	file, err := openNativeRead(name)
	if err != nil {
		return nil, err
	}
	owner := &nativeOrdinaryImage{file: file, expected: spec.expectedImageHash}
	ok := false
	defer func() {
		if !ok {
			_ = owner.Close()
		}
	}()
	handle := windows.Handle(file.Fd())
	if err := windows.GetFileInformationByHandle(handle, &owner.info); err != nil {
		return nil, err
	}
	owner.name, err = nativePhysicalName(handle)
	if err != nil {
		return nil, err
	}
	_, parents, err := physicalAncestors(owner.name)
	if err != nil || len(parents) > 256 {
		return nil, errors.New("physical ancestor count outside bound")
	}
	for i, parent := range parents {
		spelling := physicalWin32(parent)
		if i == 0 {
			spelling += `\` // CreateFile must open the volume root directory, not the volume device.
		}
		pointer, err := windows.UTF16PtrFromString(spelling)
		if err != nil {
			return nil, err
		}
		h, err := windows.CreateFile(pointer, windows.FILE_READ_ATTRIBUTES,
			windows.FILE_SHARE_READ|windows.FILE_SHARE_WRITE, nil, windows.OPEN_EXISTING,
			windows.FILE_FLAG_BACKUP_SEMANTICS|windows.FILE_FLAG_OPEN_REPARSE_POINT, 0)
		if err != nil {
			return nil, errors.New("physical ancestor could not be pinned")
		}
		entry := nativeOrdinaryDirectory{handle: h, name: parent}
		owner.parents = append(owner.parents, entry) // close even if inspection fails
		if err := windows.GetFileInformationByHandle(h, &entry.info); err != nil {
			return nil, err
		}
		owner.parents[len(owner.parents)-1] = entry
		if entry.info.FileAttributes&windows.FILE_ATTRIBUTE_DIRECTORY == 0 ||
			entry.info.FileAttributes&windows.FILE_ATTRIBUTE_REPARSE_POINT != 0 ||
			entry.info.VolumeSerialNumber != owner.info.VolumeSerialNumber {
			return nil, errors.New("ordinary same-volume physical directory required")
		}
		actual, err := nativePhysicalName(h)
		if err != nil || !strings.EqualFold(strings.TrimSuffix(actual, `\`), parent) {
			return nil, errors.New("physical ancestor name differs")
		}
		if i == 0 {
			var serial, component, flags uint32
			fs := make([]uint16, 32)
			if err := windows.GetVolumeInformationByHandle(h, nil, 0, &serial, &component, &flags, &fs[0], uint32(len(fs))); err != nil ||
				windows.UTF16ToString(fs) != "NTFS" || serial != owner.info.VolumeSerialNumber {
				return nil, errors.New("ordinary application requires a local NTFS volume")
			}
		}
	}
	owner.application, err = nativeOrdinaryApplication(handle, owner.name)
	if err != nil {
		return nil, err
	}
	if err := owner.recheck(); err != nil {
		return nil, err
	}
	ok = true
	return owner, nil
}

func (owner *nativeOrdinaryImage) Close() error {
	var result error
	for i := len(owner.parents) - 1; i >= 0; i-- {
		result = errors.Join(result, windows.CloseHandle(owner.parents[i].handle))
	}
	owner.parents = nil
	if owner.file != nil {
		result = errors.Join(result, owner.file.Close())
		owner.file = nil
	}
	return result
}

func (owner *nativeOrdinaryImage) recheck() error {
	for i, entry := range owner.parents {
		var info windows.ByHandleFileInformation
		name, err := nativePhysicalName(entry.handle)
		if err != nil || windows.GetFileInformationByHandle(entry.handle, &info) != nil ||
			!sameNativeIdentity(entry.info, info) || info.FileAttributes&windows.FILE_ATTRIBUTE_DIRECTORY == 0 ||
			info.FileAttributes&windows.FILE_ATTRIBUTE_REPARSE_POINT != 0 {
			return errors.New("held physical ancestor changed")
		}
		if i == 0 {
			name = strings.TrimSuffix(name, `\`)
		}
		if !strings.EqualFold(name, entry.name) {
			return errors.New("held physical ancestor name changed")
		}
	}
	var info windows.ByHandleFileInformation
	handle := windows.Handle(owner.file.Fd())
	name, err := nativePhysicalName(handle)
	if err != nil || windows.GetFileInformationByHandle(handle, &info) != nil ||
		!sameNativeImage(owner.info, info) || !strings.EqualFold(name, owner.name) {
		return errors.New("held ordinary image metadata changed")
	}
	actual, err := hashFile(owner.file)
	if err != nil || !hashPattern.MatchString(owner.expected) || actual != owner.expected {
		return errors.New("held ordinary image hash differs")
	}
	// Reopen the pinned physical spelling before corroborating the separately
	// selected, mutable DOS application lookup.
	pointer, err := windows.UTF16PtrFromString(physicalWin32(owner.name))
	if err != nil {
		return err
	}
	h, err := windows.CreateFile(pointer, windows.GENERIC_READ, windows.FILE_SHARE_READ, nil, windows.OPEN_EXISTING, windows.FILE_FLAG_OPEN_REPARSE_POINT, 0)
	if err != nil {
		return errors.New("held physical image lookup failed")
	}
	defer windows.CloseHandle(h)
	var reopened windows.ByHandleFileInformation
	if windows.GetFileInformationByHandle(h, &reopened) != nil || !sameNativeImage(owner.info, reopened) {
		return errors.New("physical image lookup identity differs")
	}
	return owner.recheckApplication()
}

func (owner *nativeOrdinaryImage) recheckApplication() error {
	application, err := ordinaryApplicationPath(owner.application, owner.name)
	if err != nil || application != owner.application || len(owner.parents) == 0 {
		return errors.New("selected ordinary DOS application spelling changed")
	}
	root, err := windows.UTF16PtrFromString(application[:3])
	if err != nil {
		return err
	}
	h, err := windows.CreateFile(root, windows.FILE_READ_ATTRIBUTES,
		windows.FILE_SHARE_READ|windows.FILE_SHARE_WRITE, nil, windows.OPEN_EXISTING,
		windows.FILE_FLAG_BACKUP_SEMANTICS|windows.FILE_FLAG_OPEN_REPARSE_POINT, 0)
	if err != nil {
		return errors.New("selected ordinary DOS root lookup failed")
	}
	defer windows.CloseHandle(h)
	var info windows.ByHandleFileInformation
	name, err := nativePhysicalName(h)
	if err != nil || windows.GetFileInformationByHandle(h, &info) != nil ||
		!sameNativeIdentity(owner.parents[0].info, info) ||
		info.FileAttributes&windows.FILE_ATTRIBUTE_DIRECTORY == 0 || info.FileAttributes&windows.FILE_ATTRIBUTE_REPARSE_POINT != 0 ||
		!strings.EqualFold(strings.TrimSuffix(name, `\`), owner.parents[0].name) {
		return errors.New("selected ordinary DOS root differs from held physical root")
	}
	pointer, err := windows.UTF16PtrFromString(application)
	if err != nil {
		return err
	}
	image, err := windows.CreateFile(pointer, windows.GENERIC_READ, windows.FILE_SHARE_READ,
		nil, windows.OPEN_EXISTING, windows.FILE_FLAG_OPEN_REPARSE_POINT, 0)
	if err != nil {
		return errors.New("selected ordinary DOS image lookup failed")
	}
	defer windows.CloseHandle(image)
	name, err = nativePhysicalName(image)
	if err != nil || windows.GetFileInformationByHandle(image, &info) != nil ||
		!sameNativeImage(owner.info, info) || !strings.EqualFold(name, owner.name) {
		return errors.New("selected ordinary DOS image differs from held physical image")
	}
	return nil
}

func ordinaryEnvironment(values []string) ([]uint16, error) {
	kept := make([]string, 0, len(values))
	for _, value := range values {
		key, _, ok := strings.Cut(value, "=")
		if strings.ContainsRune(value, 0) || !ok || (key == "" && !strings.HasPrefix(value, "=")) {
			return nil, errors.New("invalid ordinary application environment")
		}
		if !strings.HasPrefix(strings.ToUpper(key), "SKAGER_UPDATE_") {
			kept = append(kept, value)
		}
	}
	sort.Slice(kept, func(i, j int) bool { return strings.ToUpper(kept[i]) < strings.ToUpper(kept[j]) })
	block := utf16.Encode([]rune(strings.Join(kept, "\x00") + "\x00\x00"))
	if len(block) > 32767 {
		return nil, errors.New("ordinary application environment outside bound")
	}
	return block, nil
}

type nativeOrdinaryChild struct {
	process, thread windows.Handle
	pid, tid        uint32
	creation        windows.Filetime
	image           string
	owned           bool
	resumeAttempted bool
}

func filetimeValue(t windows.Filetime) uint64 {
	return uint64(t.HighDateTime)<<32 | uint64(t.LowDateTime)
}

func (owner *nativeOrdinaryImage) create(spec processSpec) (ordinaryChild, error) {
	physical := physicalWin32(owner.name)
	application, err := windows.UTF16PtrFromString(owner.application)
	if err != nil {
		return nil, err
	}
	command := windows.ComposeCommandLine(append([]string{physical}, spec.arguments...))
	commandLine, err := windows.UTF16FromString(command)
	if err != nil || len(commandLine) > 32767 {
		return nil, errors.New("ordinary command line outside bound")
	}
	directory, err := windows.UTF16PtrFromString(physical[:strings.LastIndex(physical, `\`)])
	if err != nil {
		return nil, err
	}
	environment, err := ordinaryEnvironment(os.Environ())
	if err != nil {
		return nil, err
	}
	startup := windows.StartupInfo{Cb: uint32(unsafe.Sizeof(windows.StartupInfo{})), Flags: windows.STARTF_USESHOWWINDOW, ShowWindow: windows.SW_HIDE}
	var pi windows.ProcessInformation
	// No inherited handles and no implicit application search. Any error is a
	// failed/uncertain native attempt, never permission to try another creator.
	err = windows.CreateProcess(application, &commandLine[0], nil, nil, false,
		windows.CREATE_SUSPENDED|windows.CREATE_UNICODE_ENVIRONMENT, &environment[0], directory, &startup, &pi)
	if err != nil {
		return nil, errors.New("suspended ordinary application creation failed; no retry")
	}
	return &nativeOrdinaryChild{process: pi.Process, thread: pi.Thread, pid: pi.ProcessId, tid: pi.ThreadId, image: owner.name}, nil
}

var ordinaryGetThreadID = windows.NewLazySystemDLL("kernel32.dll").NewProc("GetThreadId")
var ordinaryGetThreadProcessID = windows.NewLazySystemDLL("kernel32.dll").NewProc("GetProcessIdOfThread")

func (child *nativeOrdinaryChild) rawIdentity() (windows.Filetime, error) {
	if child.process == 0 || child.process == windows.InvalidHandle || child.thread == 0 || child.thread == windows.InvalidHandle || child.pid == 0 || child.tid == 0 {
		return windows.Filetime{}, errors.New("ordinary child returned invalid raw identity")
	}
	pid, err := windows.GetProcessId(child.process)
	tid, _, _ := ordinaryGetThreadID.Call(uintptr(child.thread))
	threadPID, _, _ := ordinaryGetThreadProcessID.Call(uintptr(child.thread))
	var created, exited, kernel, user windows.Filetime
	if err != nil || pid != child.pid || uint32(tid) != child.tid || uint32(threadPID) != child.pid ||
		windows.GetProcessTimes(child.process, &created, &exited, &kernel, &user) != nil || filetimeValue(created) == 0 {
		return windows.Filetime{}, errors.New("ordinary child raw identity differs")
	}
	return created, nil
}

func (child *nativeOrdinaryChild) corroborate() error {
	created, err := child.rawIdentity()
	if err != nil {
		return err
	}
	confirmed, err := child.rawIdentity()
	if err != nil || created != confirmed {
		return errors.New("ordinary child creation identity not consistent")
	}
	child.creation, child.owned = created, true
	buffer := make([]uint16, 32768)
	size := uint32(len(buffer))
	if windows.QueryFullProcessImageName(child.process, 1, &buffer[0], &size) != nil || size == 0 || size >= uint32(len(buffer)) ||
		!strings.EqualFold(windows.UTF16ToString(buffer[:size]), child.image) {
		return errors.New("ordinary child native image name differs")
	}
	status, err := windows.WaitForSingleObject(child.process, 0)
	if err != nil || status != uint32(windows.WAIT_TIMEOUT) {
		return errors.New("ordinary child exited before resume")
	}
	return nil
}

func (child *nativeOrdinaryChild) resume() (uint32, error) {
	if !child.owned || child.resumeAttempted {
		return 0, errors.New("ordinary child resume ownership unavailable")
	}
	created, err := child.rawIdentity()
	if err != nil || created != child.creation {
		return 0, errors.New("ordinary child identity changed before resume")
	}
	child.resumeAttempted = true
	return windows.ResumeThread(child.thread)
}

func (child *nativeOrdinaryChild) abortSuspended() error {
	// This object records successful CREATE_SUSPENDED and no ResumeThread attempt.
	// Never terminate by PID/name, after an uncertain create, or after any resume.
	if !child.owned || child.resumeAttempted {
		return errors.New("ordinary child termination withheld: suspension/ownership uncertain")
	}
	created, err := child.rawIdentity()
	if err != nil || created != child.creation {
		return errors.New("ordinary child termination withheld: raw identity changed")
	}
	if err := windows.TerminateProcess(child.process, 1); err != nil {
		return errors.New("owned suspended ordinary child termination failed")
	}
	status, err := windows.WaitForSingleObject(child.process, 5000)
	if err != nil || status != windows.WAIT_OBJECT_0 {
		return errors.New("owned suspended ordinary child exit not confirmed")
	}
	return nil
}

func (child *nativeOrdinaryChild) Close() error {
	return errors.Join(windows.CloseHandle(child.thread), windows.CloseHandle(child.process))
}
