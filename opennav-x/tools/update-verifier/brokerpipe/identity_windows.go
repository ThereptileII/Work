//go:build windows

package brokerpipe

import (
	"errors"
	"os"
	"path/filepath"
	"runtime"
	"strings"
	"unsafe"

	"golang.org/x/sys/windows"
)

const fileDeleteChild = 0x40

const serviceAccount = `NT SERVICE\SkagerUpdateBroker`
const trustedInstaller = "S-1-5-80-956008885-3418522649-1831038044-1853292631-2271478464"

var impersonate = windows.NewLazySystemDLL("advapi32.dll").NewProc("ImpersonateNamedPipeClient")

type tokenIdentity struct {
	sid     string
	session uint32
	logon   windows.LUID
}

// TOKEN_STATISTICS has a native 64-bit LARGE_INTEGER, including Win32. Keep the
// explicit layout, rather than Go int or architecture-dependent pointer fields.
type tokenStatistics struct {
	TokenID            windows.LUID
	AuthenticationID   windows.LUID
	ExpirationTime     int64
	TokenType          uint32
	ImpersonationLevel uint32
	DynamicCharged     uint32
	DynamicAvailable   uint32
	GroupCount         uint32
	PrivilegeCount     uint32
	ModifiedID         windows.LUID
}

func tokenInfo(t windows.Token) (tokenIdentity, error) {
	var out tokenIdentity
	u, e := t.GetTokenUser()
	if e != nil {
		return out, e
	}
	out.sid = u.User.Sid.String()
	var used uint32
	if e = windows.GetTokenInformation(t, windows.TokenSessionId, (*byte)(unsafe.Pointer(&out.session)), 4, &used); e != nil || used != 4 {
		return tokenIdentity{}, ErrRefused
	}
	var stats tokenStatistics
	if e = windows.GetTokenInformation(t, windows.TokenStatistics, (*byte)(unsafe.Pointer(&stats)), uint32(unsafe.Sizeof(stats)), &used); e != nil || used != uint32(unsafe.Sizeof(stats)) {
		return tokenIdentity{}, ErrRefused
	}
	out.logon = stats.AuthenticationID
	return out, nil
}

type processIdentity struct {
	handle  windows.Handle
	pid     uint32
	created windows.Filetime
	token   tokenIdentity
}

func openProcess(pid uint32) (*processIdentity, error) {
	if pid == 0 {
		return nil, ErrRefused
	}
	h, e := windows.OpenProcess(windows.PROCESS_QUERY_LIMITED_INFORMATION|windows.SYNCHRONIZE, false, pid)
	if e != nil {
		return nil, e
	}
	p := &processIdentity{handle: h, pid: pid}
	var exit, kernel, user windows.Filetime
	if e = windows.GetProcessTimes(h, &p.created, &exit, &kernel, &user); e != nil {
		p.close()
		return nil, e
	}
	var token windows.Token
	if e = windows.OpenProcessToken(h, windows.TOKEN_QUERY, &token); e != nil {
		p.close()
		return nil, e
	}
	p.token, e = tokenInfo(token)
	_ = token.Close()
	if e != nil {
		p.close()
		return nil, e
	}
	if e = p.live(); e != nil {
		p.close()
		return nil, e
	}
	return p, nil
}
func (p *processIdentity) close() {
	if p != nil && p.handle != 0 {
		_ = windows.CloseHandle(p.handle)
		p.handle = 0
	}
}
func (p *processIdentity) live() error {
	if p == nil || p.handle == 0 {
		return ErrRefused
	}
	state, e := windows.WaitForSingleObject(p.handle, 0)
	if e != nil || state != uint32(windows.WAIT_TIMEOUT) {
		return ErrRefused
	}
	var c, x, k, u windows.Filetime
	if windows.GetProcessTimes(p.handle, &c, &x, &k, &u) != nil || c != p.created {
		return ErrRefused
	}
	return nil
}
func clientIdentity(pipe windows.Handle) (p *processIdentity, err error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	ok, _, e := impersonate.Call(uintptr(pipe))
	if ok == 0 {
		return nil, errors.Join(ErrRefused, e)
	}
	// Revert before any filesystem/service/handler work, on every token failure.
	// Failure to revert cannot safely return to privileged caller code. This is
	// a fail-stop of this broker process only, never termination of a peer/child.
	defer func() {
		if windows.RevertToSelf() != nil {
			os.Exit(1)
		}
	}()
	thread, e := windows.GetCurrentThread()
	if e != nil {
		return nil, e
	}
	var token windows.Token
	if e = windows.OpenThreadToken(thread, windows.TOKEN_QUERY, true, &token); e != nil {
		return nil, e
	}
	identity, e := tokenInfo(token)
	_ = token.Close()
	if e != nil {
		return nil, e
	}
	if identity.session == 0 || !strings.HasPrefix(identity.sid, "S-1-5-21-") {
		return nil, ErrRefused
	}
	var pid uint32
	if e = windows.GetNamedPipeClientProcessId(pipe, &pid); e != nil {
		return nil, e
	}
	// Open the process only after reversion: the identification token deliberately
	// cannot be used to perform access checks as the user.
	if e = windows.RevertToSelf(); e != nil {
		os.Exit(1)
	}
	p, e = openProcess(pid)
	if e != nil {
		return nil, e
	}
	if p.token != identity {
		p.close()
		return nil, ErrRefused
	}
	return p, nil
}
func brokerSID() (string, error) {
	sid, _, _, e := windows.LookupSID("", serviceAccount)
	if e != nil {
		return "", e
	}
	return sid.String(), nil
}
func expectedImage() (string, error) {
	root, e := windows.KnownFolderPath(windows.FOLDERID_ProgramFilesX86, 0)
	if e != nil {
		return "", e
	}
	if len(root) < 3 || root[1] != ':' || !filepath.IsAbs(root) {
		return "", ErrRefused
	}
	return filepath.Join(root, "SKAGER", "UpdateBroker", "skager-update-broker.exe"), nil
}

type fileID struct {
	Volume uint64
	ID     [16]byte
}
type imagePins struct {
	handles []windows.Handle
	image   fileID
	path    string
}

func (p *imagePins) close() {
	for i := len(p.handles) - 1; i >= 0; i-- {
		_ = windows.CloseHandle(p.handles[i])
	}
	p.handles = nil
}
func trustedOwner(s string) bool {
	return s == "S-1-5-18" || s == "S-1-5-32-544" || s == trustedInstaller
}

// Do not evaluate arbitrary user's group membership and miss a writable ACE.
// Only SYSTEM/admin/TrustedInstaller may mutate code or its directory entries;
// unknown ACE types refuse rather than being silently ignored.
func protectedACL(h windows.Handle, kind windows.SE_OBJECT_TYPE, mutation uint32) error {
	sd, e := windows.GetSecurityInfo(h, kind, windows.OWNER_SECURITY_INFORMATION|windows.DACL_SECURITY_INFORMATION)
	if e != nil {
		return e
	}
	return protectedDescriptor(sd, mutation)
}
func protectedDescriptor(sd *windows.SECURITY_DESCRIPTOR, mutation uint32) error {
	owner, _, e := sd.Owner()
	if e != nil || owner == nil || !trustedOwner(owner.String()) {
		return ErrRefused
	}
	acl, _, e := sd.DACL()
	if e != nil || acl == nil {
		return ErrRefused
	}
	for i := uint32(0); i < uint32(acl.AceCount); i++ {
		var ace *windows.ACCESS_ALLOWED_ACE
		if windows.GetAce(acl, i, &ace) != nil || ace == nil {
			return ErrRefused
		}
		if ace.Header.AceFlags&windows.INHERIT_ONLY_ACE != 0 {
			continue
		}
		if ace.Header.AceType == windows.ACCESS_DENIED_ACE_TYPE {
			continue
		}
		if ace.Header.AceType != windows.ACCESS_ALLOWED_ACE_TYPE {
			return ErrRefused
		}
		sid := (*windows.SID)(unsafe.Pointer(&ace.SidStart))
		if !sid.IsValid() {
			return ErrRefused
		}
		if uint32(ace.Mask)&mutation != 0 && !trustedOwner(sid.String()) {
			return ErrRefused
		}
	}
	return nil
}
func idOf(h windows.Handle) (fileID, error) {
	var id fileID
	e := windows.GetFileInformationByHandleEx(h, windows.FileIdInfo, (*byte)(unsafe.Pointer(&id)), uint32(unsafe.Sizeof(id)))
	return id, e
}

// physicalPath is obtained only from a held handle, never from a request. Only
// local disk volumes are supported; UNC/device aliases fail closed.
func physicalPath(h windows.Handle) (string, error) {
	var buffer [32768]uint16
	n, e := windows.GetFinalPathNameByHandle(h, &buffer[0], uint32(len(buffer)), 2)
	if e != nil || n == 0 || n >= uint32(len(buffer)) {
		return "", ErrRefused
	}
	name := windows.UTF16ToString(buffer[:n])
	if _, e = physicalParents(name); e != nil {
		return "", e
	}
	return name, nil
}
func physicalParents(name string) ([]string, error) {
	parts := strings.Split(name, `\`)
	if len(parts) < 4 || len(parts) > 128 || parts[0] != "" || parts[1] != "Device" || !strings.HasPrefix(parts[2], "HarddiskVolume") {
		return nil, ErrRefused
	}
	number := strings.TrimPrefix(parts[2], "HarddiskVolume")
	if len(number) == 0 || len(number) > 10 {
		return nil, ErrRefused
	}
	for _, c := range number {
		if c < '0' || c > '9' {
			return nil, ErrRefused
		}
	}
	root := `\Device\` + parts[2]
	parents := []string{root}
	current := root
	for i, part := range parts[3:] {
		if part == "" || part == "." || part == ".." || strings.ContainsAny(part, `/:`+"\x00") || strings.HasSuffix(part, ".") || strings.HasSuffix(part, " ") {
			return nil, ErrRefused
		}
		for _, c := range part {
			if c < 32 {
				return nil, ErrRefused
			}
		}
		current += `\` + part
		if i < len(parts[3:])-1 {
			parents = append(parents, current)
		}
	}
	return parents, nil
}
func pinImage(path string) (pins *imagePins, err error) {
	pins = &imagePins{}
	owned := pins
	defer func() {
		if err != nil {
			owned.close()
		}
	}()
	pointer, e := windows.UTF16PtrFromString(path)
	if e != nil {
		return nil, e
	}
	image, e := windows.CreateFile(pointer, windows.GENERIC_READ|windows.READ_CONTROL, windows.FILE_SHARE_READ, nil, windows.OPEN_EXISTING, windows.FILE_FLAG_OPEN_REPARSE_POINT, 0)
	if e != nil {
		return nil, e
	}
	pins.handles = append(pins.handles, image)
	var info windows.ByHandleFileInformation
	if windows.GetFileInformationByHandle(image, &info) != nil || info.FileAttributes&(windows.FILE_ATTRIBUTE_REPARSE_POINT|windows.FILE_ATTRIBUTE_DIRECTORY) != 0 || info.NumberOfLinks != 1 {
		return nil, ErrRefused
	}
	mutation := uint32(windows.DELETE | windows.WRITE_DAC | windows.WRITE_OWNER | windows.GENERIC_ALL | windows.GENERIC_WRITE | windows.FILE_WRITE_ATTRIBUTES | windows.FILE_WRITE_EA)
	if e = protectedACL(image, windows.SE_FILE_OBJECT, mutation|windows.FILE_WRITE_DATA|windows.FILE_APPEND_DATA); e != nil {
		return nil, e
	}
	pins.image, e = idOf(image)
	if e != nil {
		return nil, e
	}
	pins.path, e = physicalPath(image)
	if e != nil {
		return nil, e
	}
	parents, e := physicalParents(pins.path)
	if e != nil {
		return nil, e
	}
	// Pin the real volume root and all physical ancestors root-first. DOS aliases
	// cannot conceal a writable ancestor or redirect this traversal after capture.
	for i, name := range parents {
		spelling := `\\?\GLOBALROOT` + name
		if i == 0 {
			spelling += `\`
		} // Open the root directory, not the volume device.
		ptr, e := windows.UTF16PtrFromString(spelling)
		if e != nil {
			return nil, e
		}
		h, e := windows.CreateFile(ptr, windows.READ_CONTROL|windows.FILE_READ_ATTRIBUTES, windows.FILE_SHARE_READ|windows.FILE_SHARE_WRITE, nil, windows.OPEN_EXISTING, windows.FILE_FLAG_OPEN_REPARSE_POINT|windows.FILE_FLAG_BACKUP_SEMANTICS, 0)
		if e != nil {
			return nil, e
		}
		pins.handles = append(pins.handles, h)
		if windows.GetFileInformationByHandle(h, &info) != nil || info.FileAttributes&windows.FILE_ATTRIBUTE_REPARSE_POINT != 0 || info.FileAttributes&windows.FILE_ATTRIBUTE_DIRECTORY == 0 {
			return nil, ErrRefused
		}
		// A drive root may permit unrelated directory creation. It must not permit
		// deletion/control of the existing ProgramFiles child. Lower ancestors must
		// also refuse namespace additions, including adjacent DLL insertion.
		mask := mutation | fileDeleteChild
		if i != 0 {
			mask |= windows.FILE_WRITE_DATA | windows.FILE_APPEND_DATA
		}
		if e = protectedACL(h, windows.SE_FILE_OBJECT, mask); e != nil {
			return nil, e
		}
		var final [32768]uint16
		n, e := windows.GetFinalPathNameByHandle(h, &final[0], uint32(len(final)), 2)
		if e != nil || n == 0 || n >= uint32(len(final)) || !strings.EqualFold(strings.TrimRight(windows.UTF16ToString(final[:n]), `\`), name) {
			return nil, ErrRefused
		}
	}
	return pins, nil
}
func (p *imagePins) matches(process *processIdentity) error {
	if e := process.live(); e != nil {
		return e
	}
	var image [32768]uint16
	n := uint32(len(image))
	if e := windows.QueryFullProcessImageName(process.handle, windows.PROCESS_NAME_NATIVE, &image[0], &n); e != nil {
		return e
	}
	actual := windows.UTF16ToString(image[:n])
	if !strings.EqualFold(actual, p.path) {
		return ErrRefused
	}
	name, e := windows.UTF16PtrFromString(`\\?\GLOBALROOT` + actual)
	if e != nil {
		return e
	}
	h, e := windows.CreateFile(name, windows.FILE_READ_ATTRIBUTES, windows.FILE_SHARE_READ, nil, windows.OPEN_EXISTING, windows.FILE_FLAG_OPEN_REPARSE_POINT, 0)
	if e != nil {
		return e
	}
	defer windows.CloseHandle(h)
	id, e := idOf(h)
	if e != nil || id != p.image {
		return ErrRefused
	}
	return nil
}
