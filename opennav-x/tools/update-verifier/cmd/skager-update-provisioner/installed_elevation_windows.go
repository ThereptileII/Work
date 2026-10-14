//go:build windows

package main

import (
	"context"
	"errors"
	"path/filepath"
	"runtime"
	"strings"
	"sync"
	"syscall"
	"time"
	"unsafe"

	"golang.org/x/sys/windows"
)

const installedElevationImage = "skager-update-provisioner.exe"
const installedElevationTI = "S-1-5-80-956008885-3418522649-1831038044-1853292631-2271478464"

var installedElevationShellExecute = windows.NewLazySystemDLL("shell32.dll").NewProc("ShellExecuteExW")

type installedElevationFileID struct {
	volume uint64
	id     [16]byte
}
type installedElevationCodePin struct {
	handle                     windows.Handle
	path, physical, descriptor string
	info                       windows.ByHandleFileInformation
	id                         installedElevationFileID
	directory, allowAdds       bool
}
type installedElevationCode struct {
	pins             []*installedElevationCodePin
	image, directory string
	closed           bool
	closeErr         error
}

func installedElevationSecurity(sd *windows.SECURITY_DESCRIPTOR, allowAdds bool) error {
	trusted := func(sid string) bool {
		return sid == "S-1-5-18" || sid == "S-1-5-32-544" || sid == installedElevationTI
	}
	owner, _, err := sd.Owner()
	if err != nil || owner == nil || !trusted(owner.String()) {
		return errors.New("original protected maintenance code owner required")
	}
	acl, _, err := sd.DACL()
	if err != nil || acl == nil {
		return errors.New("original protected maintenance code DACL required")
	}
	mutation := uint32(windows.FILE_WRITE_DATA | windows.FILE_APPEND_DATA | windows.FILE_WRITE_EA | windows.FILE_WRITE_ATTRIBUTES | 0x40 /* FILE_DELETE_CHILD */ | windows.DELETE | windows.WRITE_DAC | windows.WRITE_OWNER | windows.GENERIC_WRITE | windows.GENERIC_ALL)
	if allowAdds {
		mutation &^= windows.FILE_WRITE_DATA | windows.FILE_APPEND_DATA
	}
	for i := uint32(0); i < uint32(acl.AceCount); i++ {
		var ace *windows.ACCESS_ALLOWED_ACE
		if err := windows.GetAce(acl, i, &ace); err != nil {
			return err
		}
		if ace.Header.AceFlags&windows.INHERIT_ONLY_ACE != 0 || ace.Header.AceType == windows.ACCESS_DENIED_ACE_TYPE {
			continue
		}
		if ace.Header.AceType != windows.ACCESS_ALLOWED_ACE_TYPE {
			return errors.New("unsupported maintenance code ACE")
		}
		sid := (*windows.SID)(unsafe.Pointer(&ace.SidStart))
		if !sid.IsValid() || !trusted(sid.String()) && uint32(ace.Mask)&mutation != 0 {
			return errors.New("foreign maintenance code mutation permission")
		}
	}
	return nil
}
func installedElevationPhysical(handle windows.Handle) (string, error) {
	raw := make([]uint16, 32768)
	n, err := windows.GetFinalPathNameByHandle(handle, &raw[0], uint32(len(raw)), 2) // VOLUME_NAME_NT
	if err != nil || n == 0 || n >= uint32(len(raw)) {
		return "", errors.Join(errors.New("original physical maintenance name required"), err)
	}
	return windows.UTF16ToString(raw[:n]), nil
}
func (p *installedElevationCodePin) observe() (windows.ByHandleFileInformation, installedElevationFileID, string, *windows.SECURITY_DESCRIPTOR, error) {
	var info windows.ByHandleFileInformation
	var id installedElevationFileID
	if err := windows.GetFileInformationByHandle(p.handle, &info); err != nil {
		return info, id, "", nil, err
	}
	if err := windows.GetFileInformationByHandleEx(p.handle, windows.FileIdInfo, (*byte)(unsafe.Pointer(&id)), uint32(unsafe.Sizeof(id))); err != nil {
		return info, id, "", nil, err
	}
	name, err := installedElevationPhysical(p.handle)
	if err != nil {
		return info, id, "", nil, err
	}
	sd, err := windows.GetSecurityInfo(p.handle, windows.SE_FILE_OBJECT, windows.OWNER_SECURITY_INFORMATION|windows.DACL_SECURITY_INFORMATION)
	return info, id, name, sd, err
}
func (p *installedElevationCodePin) check() error {
	info, id, name, sd, err := p.observe()
	if err != nil {
		return err
	}
	if id != p.id || info.CreationTime != p.info.CreationTime || name != p.physical || info.FileAttributes&windows.FILE_ATTRIBUTE_REPARSE_POINT != 0 || (info.FileAttributes&windows.FILE_ATTRIBUTE_DIRECTORY != 0) != p.directory || sd.String() != p.descriptor {
		return errors.New("original maintenance code identity changed")
	}
	if !p.directory && (info.NumberOfLinks != 1 || info.FileSizeHigh != p.info.FileSizeHigh || info.FileSizeLow != p.info.FileSizeLow) {
		return errors.New("original maintenance image links/length changed")
	}
	return installedElevationSecurity(sd, p.allowAdds)
}
func (c *installedElevationCode) check(ctx context.Context) error {
	if ctx == nil || ctx.Err() != nil || c == nil || c.closed {
		return errors.New("live original maintenance code required")
	}
	for _, pin := range c.pins {
		if err := pin.check(); err != nil {
			return err
		}
	}
	return ctx.Err()
}
func (c *installedElevationCode) close() error {
	if c == nil {
		return nil
	}
	if c.closed {
		return c.closeErr
	}
	c.closed = true
	for i := len(c.pins) - 1; i >= 0; i-- {
		c.closeErr = errors.Join(c.closeErr, windows.CloseHandle(c.pins[i].handle))
	}
	return c.closeErr
}
func openInstalledElevationCode(ctx context.Context) (c *installedElevationCode, resultErr error) {
	if ctx == nil || ctx.Err() != nil {
		return nil, errors.New("live fixed code custody required")
	}
	base, err := windows.KnownFolderPath(windows.FOLDERID_ProgramFilesX86, 0)
	if err != nil {
		return nil, err
	}
	if len(base) < 4 || filepath.Clean(base) != base || base[1:3] != `:\` || strings.ContainsAny(base[3:], ":/\x00") {
		return nil, errors.New("fixed local ProgramFilesX86 required")
	}
	c = &installedElevationCode{directory: filepath.Join(base, "SKAGER", "UpdateBroker")}
	c.image = filepath.Join(c.directory, installedElevationImage)
	defer func() {
		if resultErr != nil {
			resultErr = errors.Join(resultErr, c.close())
		}
	}()
	paths := []string{base[:3]}
	current := base[:2]
	for _, component := range strings.Split(c.image[3:], `\`) {
		if component == "" || component == "." || component == ".." || strings.HasSuffix(component, ".") || strings.HasSuffix(component, " ") {
			return c, errors.New("canonical fixed code ancestry required")
		}
		current += `\` + component
		paths = append(paths, current)
	}
	for i, path := range paths {
		directory := i != len(paths)-1
		pin := &installedElevationCodePin{path: path, directory: directory, allowAdds: directory && len(path) < len(base)}
		ptr, err := windows.UTF16PtrFromString(path)
		if err != nil {
			return c, err
		}
		rights, flags, share := uint32(windows.FILE_READ_ATTRIBUTES|windows.READ_CONTROL), uint32(windows.FILE_FLAG_OPEN_REPARSE_POINT|windows.FILE_FLAG_BACKUP_SEMANTICS), uint32(windows.FILE_SHARE_READ|windows.FILE_SHARE_WRITE)
		if !directory {
			rights = windows.GENERIC_READ | windows.READ_CONTROL
			flags = windows.FILE_FLAG_OPEN_REPARSE_POINT
			share = windows.FILE_SHARE_READ
		}
		pin.handle, err = windows.CreateFile(ptr, rights, share, nil, windows.OPEN_EXISTING, flags, 0)
		if err != nil {
			return c, err
		}
		c.pins = append(c.pins, pin)
		info, id, physical, sd, err := pin.observe()
		if err != nil {
			return c, err
		}
		pin.info, pin.id, pin.physical, pin.descriptor = info, id, physical, sd.String()
		if i == 0 {
			pieces := strings.Split(strings.TrimSuffix(physical, `\`), `\`)
			if len(pieces) != 3 || pieces[0] != "" || pieces[1] != "Device" || !strings.HasPrefix(pieces[2], "HarddiskVolume") || strings.TrimPrefix(pieces[2], "HarddiskVolume") == "" {
				return c, errors.New("fixed local physical drive root required")
			}
			for _, ch := range strings.TrimPrefix(pieces[2], "HarddiskVolume") {
				if ch < '0' || ch > '9' {
					return c, errors.New("fixed local physical volume required")
				}
			}
			var serial, component, volumeFlags uint32
			fs := make([]uint16, 32)
			if err := windows.GetVolumeInformationByHandle(pin.handle, nil, 0, &serial, &component, &volumeFlags, &fs[0], uint32(len(fs))); err != nil || windows.UTF16ToString(fs) != "NTFS" {
				return c, errors.New("fixed native NTFS maintenance code required")
			}
		} else {
			previous := c.pins[i-1]
			if id.volume != previous.id.volume || !strings.EqualFold(physical, strings.TrimSuffix(previous.physical, `\`)+`\`+filepath.Base(path)) {
				return c, errors.New("fixed code ancestry alias or volume differs")
			}
		}
		if !directory && (info.FileSizeHigh != 0 || info.FileSizeLow == 0 || info.FileSizeLow > 512<<20) {
			return c, errors.New("bounded original maintenance image required")
		}
		if err := pin.check(); err != nil {
			return c, err
		}
	}
	return c, c.check(ctx)
}

type installedElevationTokenStats struct {
	tokenID, logon                                                                 windows.LUID
	expires                                                                        int64
	tokenType, impersonation, dynamicCharged, dynamicAvailable, groups, privileges uint32
	modified                                                                       windows.LUID
}
type installedElevationTokenIdentity struct {
	sid            string
	session        uint32
	tokenID, logon windows.LUID
	elevated       bool
}

func installedElevationToken(token windows.Token) (id installedElevationTokenIdentity, resultErr error) {
	user, err := token.GetTokenUser()
	if err != nil || user == nil || user.User.Sid == nil {
		return id, errors.New("actual native maintenance token user required")
	}
	id.sid = user.User.Sid.String()
	if !strings.HasPrefix(id.sid, "S-1-5-21-") {
		return id, errors.New("actual interactive maintenance user required")
	}
	var used uint32
	if err := windows.GetTokenInformation(token, windows.TokenSessionId, (*byte)(unsafe.Pointer(&id.session)), 4, &used); err != nil || used != 4 || id.session == 0 {
		return id, errors.New("actual interactive maintenance session required")
	}
	var stats installedElevationTokenStats
	if err := windows.GetTokenInformation(token, windows.TokenStatistics, (*byte)(unsafe.Pointer(&stats)), uint32(unsafe.Sizeof(stats)), &used); err != nil || used != uint32(unsafe.Sizeof(stats)) || stats.tokenType != windows.TokenPrimary || stats.tokenID == (windows.LUID{}) || stats.logon == (windows.LUID{}) {
		return id, errors.New("original primary maintenance token required")
	}
	id.tokenID, id.logon, id.elevated = stats.tokenID, stats.logon, token.IsElevated()
	return id, nil
}
func installedElevationNoImpersonation() error {
	var token windows.Token
	err := windows.OpenThreadToken(windows.CurrentThread(), windows.TOKEN_QUERY, true, &token)
	if err == nil {
		return errors.Join(errors.New("impersonated customer maintenance refused"), token.Close())
	}
	if !errors.Is(err, windows.ERROR_NO_TOKEN) {
		return err
	}
	return nil
}

type installedElevationProcess struct {
	mu                sync.Mutex
	handle            windows.Handle
	token             windows.Token
	identity          installedElevationIdentity
	tokenIdentity     installedElevationTokenIdentity
	image             string
	code              *installedElevationCode
	allowExit, closed bool
	closeErr          error
}

func installedElevationImagePath(handle windows.Handle) (string, error) {
	raw := make([]uint16, 32768)
	size := uint32(len(raw))
	if err := windows.QueryFullProcessImageName(handle, 0, &raw[0], &size); err != nil || size == 0 || size >= uint32(len(raw)) {
		return "", errors.New("original native process image required")
	}
	return windows.UTF16ToString(raw[:size]), nil
}
func installedElevationCreated(handle windows.Handle) (uint64, error) {
	var created, exit, kernel, user windows.Filetime
	if err := windows.GetProcessTimes(handle, &created, &exit, &kernel, &user); err != nil {
		return 0, err
	}
	value := uint64(created.HighDateTime)<<32 | uint64(created.LowDateTime)
	if value == 0 {
		return 0, errors.New("original native process creation required")
	}
	return value, nil
}

// A real retained process/token seam used by inert native identity tests. The
// production caller supplies only its held fixed image, never a command path.
func openInstalledElevationProcess(ctx context.Context, pid uint32, image string, allowExit bool) (p *installedElevationProcess, resultErr error) {
	if ctx == nil || ctx.Err() != nil || pid == 0 || image == "" {
		return nil, errors.New("live original native process required")
	}
	p = &installedElevationProcess{image: image, allowExit: allowExit}
	defer func() {
		if resultErr != nil {
			resultErr = errors.Join(resultErr, p.close())
		}
	}()
	var err error
	p.handle, err = windows.OpenProcess(windows.PROCESS_QUERY_LIMITED_INFORMATION|windows.SYNCHRONIZE, false, pid)
	if err != nil {
		return p, err
	}
	if err := windows.OpenProcessToken(p.handle, windows.TOKEN_QUERY, &p.token); err != nil {
		return p, err
	}
	p.tokenIdentity, err = installedElevationToken(p.token)
	if err != nil {
		return p, err
	}
	created, err := installedElevationCreated(p.handle)
	if err != nil {
		return p, err
	}
	p.identity = installedElevationIdentity{pid: pid, created: created, sid: p.tokenIdentity.sid, session: p.tokenIdentity.session, elevated: p.tokenIdentity.elevated}
	return p, p.check(ctx)
}
func (p *installedElevationProcess) check(ctx context.Context) error {
	p.mu.Lock()
	defer p.mu.Unlock()
	return p.checkLocked(ctx)
}
func (p *installedElevationProcess) checkLocked(ctx context.Context) error {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if ctx == nil || ctx.Err() != nil || p.closed {
		return errors.New("original native maintenance process revoked")
	}
	if err := installedElevationNoImpersonation(); err != nil {
		return err
	}
	pid, err := windows.GetProcessId(p.handle)
	if err != nil || pid != p.identity.pid {
		return errors.New("original process handle identity changed")
	}
	created, err := installedElevationCreated(p.handle)
	if err != nil || created != p.identity.created {
		return errors.New("original process creation changed")
	}
	state, err := windows.WaitForSingleObject(p.handle, 0)
	if err != nil {
		return err
	}
	if p.allowExit && state == windows.WAIT_OBJECT_0 {
		// The returned original process handle and creation still identify the
		// exited child. Do not demand a live primary-token/image query after exit.
		return ctx.Err()
	}
	if state != uint32(windows.WAIT_TIMEOUT) {
		return errors.New("original maintenance parent exited or uncertain")
	}
	token, err := installedElevationToken(p.token)
	if err != nil || token != p.tokenIdentity {
		return errors.New("original maintenance token changed")
	}
	// Corroborate the process still uses THIS retained primary token. The
	// temporary query handle is never adopted as the original token owner.
	var observed windows.Token
	if err := windows.OpenProcessToken(p.handle, windows.TOKEN_QUERY, &observed); err != nil {
		return err
	}
	actual, queryErr := installedElevationToken(observed)
	closeErr := observed.Close()
	if queryErr != nil || closeErr != nil || actual != p.tokenIdentity {
		return errors.Join(errors.New("original process primary token changed"), queryErr, closeErr)
	}
	image, err := installedElevationImagePath(p.handle)
	if err != nil || !strings.EqualFold(image, p.image) {
		return errors.New("original fixed process image differs")
	}
	if p.code != nil {
		return p.code.check(ctx)
	}
	return ctx.Err()
}
func (p *installedElevationProcess) close() error {
	p.mu.Lock()
	defer p.mu.Unlock()
	if p.closed {
		return p.closeErr
	}
	p.closed = true
	if p.token != 0 {
		p.closeErr = errors.Join(p.closeErr, p.token.Close())
	}
	if p.handle != 0 {
		p.closeErr = errors.Join(p.closeErr, windows.CloseHandle(p.handle))
	}
	p.closeErr = errors.Join(p.closeErr, p.code.close())
	return p.closeErr
}
func (p *installedElevationProcess) wait(ctx context.Context) (uint32, error) {
	for {
		if err := p.check(ctx); err != nil {
			return 0, err
		}
		state, err := windows.WaitForSingleObject(p.handle, 25)
		if err != nil {
			return 0, err
		}
		if state == windows.WAIT_OBJECT_0 {
			var exit uint32
			if err := windows.GetExitCodeProcess(p.handle, &exit); err != nil {
				return 0, err
			}
			return exit, p.check(ctx)
		}
		if state != uint32(windows.WAIT_TIMEOUT) {
			return 0, errors.New("original elevated process wait uncertain")
		}
	}
}
func installedElevationNativeOperations() installedElevationOperations {
	return installedElevationOperations{openSelf: openInstalledElevationSelf, openParent: openInstalledElevationParent, launch: launchInstalledElevation}
}
func openInstalledElevationSelf(ctx context.Context) (result installedElevationSelf, resultErr error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	// Remove mutable working-directory/PATH DLL searches before loading shell32.
	// Initial executable loading still relies on the original protected code model.
	if err := windows.SetDefaultDllDirectories(windows.LOAD_LIBRARY_SEARCH_SYSTEM32); err != nil {
		return result, err
	}
	if err := installedElevationNoImpersonation(); err != nil {
		return result, err
	}
	code, err := openInstalledElevationCode(ctx)
	if err != nil {
		return result, err
	}
	p, err := openInstalledElevationProcess(ctx, windows.GetCurrentProcessId(), code.image, false)
	if err != nil {
		return result, errors.Join(err, code.close())
	}
	p.code = code
	result = installedElevationSelf{identity: p.identity, owner: p}
	return result, p.check(ctx)
}
func openInstalledElevationParent(ctx context.Context, q installedElevationRequest, self installedElevationIdentity) (installedElevationOwner, error) {
	code, err := openInstalledElevationCode(ctx)
	if err != nil {
		return nil, err
	}
	parent, err := openInstalledElevationProcess(ctx, q.pid, code.image, false)
	if err != nil {
		return nil, errors.Join(err, code.close())
	}
	parent.code = code
	// The native values are comparisons only. The child later derives all actual
	// cleanup authority from its own elevated token and protected completion.
	if err := installedElevationParentBinding(q, self, parent.identity); err != nil {
		return nil, errors.Join(errors.New("original live same-account elevation parent differs"), parent.close())
	}
	return parent, parent.check(ctx)
}

func installedElevationParentBinding(q installedElevationRequest, self, parent installedElevationIdentity) error {
	if !q.child || !self.elevated || q.pid == 0 || q.created == 0 || parent.pid != q.pid || parent.created != q.created || parent.sid == "" || parent.sid != self.sid || parent.session == 0 || parent.session != self.session || parent.elevated || q.pid == self.pid || self.created < q.created || self.created-q.created > uint64(installedElevationLimit/(100*time.Nanosecond)) {
		return errors.New("original live same-account elevation parent differs")
	}
	return nil
}

type installedElevationShellInfo struct {
	size, mask                        uint32
	window                            windows.Handle
	verb, file, parameters, directory *uint16
	show                              int32
	instance                          windows.Handle
	idList                            uintptr
	class                             *uint16
	classKey                          windows.Handle
	hotKey                            uint32
	icon                              windows.Handle
	process                           windows.Handle
}

func launchInstalledElevation(ctx context.Context, parent installedElevationIdentity, args []string) (installedElevationChild, error) {
	if ctx == nil || ctx.Err() != nil {
		return nil, errors.New("live bounded UAC launch required")
	}
	code, err := openInstalledElevationCode(ctx)
	if err != nil {
		return nil, err
	}
	if err := installedElevationShellExecute.Find(); err != nil {
		return nil, errors.Join(err, code.close())
	}
	// ShellExecuteEx has no cancellation API while Windows owns the UAC prompt.
	// Keep separate original code custody in its worker. A timed-out caller can
	// return without killing/retrying; a late child must reject an exited/old parent.
	type launched struct {
		child installedElevationChild
		err   error
	}
	result := make(chan launched)
	go func() {
		runtime.LockOSThread()
		defer runtime.UnlockOSThread()
		var child installedElevationChild
		var launchErr error
		defer func() {
			launchErr = errors.Join(launchErr, code.close())
			select {
			case result <- launched{child, launchErr}:
			case <-ctx.Done():
				if child != nil {
					_ = child.close()
				}
			}
		}()
		if launchErr = windows.CoInitializeEx(0, windows.COINIT_APARTMENTTHREADED|windows.COINIT_DISABLE_OLE1DDE); launchErr != nil && !errors.Is(launchErr, syscall.Errno(1)) {
			return
		}
		launchErr = nil // S_FALSE is success and still requires CoUninitialize.
		defer windows.CoUninitialize()
		if launchErr = code.check(ctx); launchErr != nil {
			return
		}
		verb, _ := windows.UTF16PtrFromString("runas")
		file, _ := windows.UTF16PtrFromString(code.image)
		parameters, _ := windows.UTF16PtrFromString(strings.Join(args, " "))
		directory, _ := windows.UTF16PtrFromString(code.directory)
		info := installedElevationShellInfo{mask: 0x40 | 0x100, verb: verb, file: file, parameters: parameters, directory: directory, show: 1}
		info.size = uint32(unsafe.Sizeof(info))
		ok, _, callErr := installedElevationShellExecute.Call(uintptr(unsafe.Pointer(&info)))
		runtime.KeepAlive(info)
		if ok == 0 || info.process == 0 {
			launchErr = errors.Join(errors.New("native maintenance UAC launch refused"), callErr)
			if info.process != 0 {
				launchErr = errors.Join(launchErr, windows.CloseHandle(info.process))
			}
			return
		}
		// Adopt the returned ORIGINAL child handle, rather than reopen a PID after UAC.
		p := &installedElevationProcess{handle: info.process, image: code.image, allowExit: true}
		child = p
		pid, err := windows.GetProcessId(p.handle)
		if err != nil {
			launchErr = err
			return
		}
		if err := windows.OpenProcessToken(p.handle, windows.TOKEN_QUERY, &p.token); err != nil {
			launchErr = err
			return
		}
		p.tokenIdentity, err = installedElevationToken(p.token)
		if err != nil {
			launchErr = err
			return
		}
		created, err := installedElevationCreated(p.handle)
		if err != nil {
			launchErr = err
			return
		}
		p.identity = installedElevationIdentity{pid: pid, created: created, sid: p.tokenIdentity.sid, session: p.tokenIdentity.session, elevated: p.tokenIdentity.elevated}
		if !p.identity.elevated || p.identity.sid != parent.sid || p.identity.session != parent.session || created <= parent.created {
			launchErr = errors.New("native elevated child account/session differs")
			return
		}
		launchErr = p.check(ctx)
	}()
	select {
	case r := <-result:
		return r.child, r.err
	case <-ctx.Done():
		return nil, ctx.Err()
	}
}
