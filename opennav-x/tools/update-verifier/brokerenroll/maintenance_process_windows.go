//go:build windows

package brokerenroll

import (
	"errors"
	"strings"
	"unsafe"

	"golang.org/x/sys/windows"
)

type maintenanceTokenIdentity struct {
	sid            string
	session        uint32
	tokenID, logon windows.LUID
}
type maintenanceTokenStatistics struct {
	TokenID, AuthenticationID                                                                   windows.LUID
	ExpirationTime                                                                              int64
	TokenType, ImpersonationLevel, DynamicCharged, DynamicAvailable, GroupCount, PrivilegeCount uint32
	ModifiedID                                                                                  windows.LUID
}

func maintenanceToken(token windows.Token) (identity maintenanceTokenIdentity, err error) {
	user, err := token.GetTokenUser()
	if err != nil || user == nil || user.User.Sid == nil {
		return identity, errors.Join(errors.New("actual maintenance process token user required"), err)
	}
	identity.sid = user.User.Sid.String()
	var used uint32
	if err := windows.GetTokenInformation(token, windows.TokenSessionId, (*byte)(unsafe.Pointer(&identity.session)), 4, &used); err != nil || used != 4 {
		return identity, errors.Join(errors.New("actual maintenance token session required"), err)
	}
	var stats maintenanceTokenStatistics
	if err := windows.GetTokenInformation(token, windows.TokenStatistics, (*byte)(unsafe.Pointer(&stats)), uint32(unsafe.Sizeof(stats)), &used); err != nil || used != uint32(unsafe.Sizeof(stats)) || stats.TokenType != 1 {
		return identity, errors.Join(errors.New("actual primary maintenance token identity required"), err)
	}
	identity.tokenID, identity.logon = stats.TokenID, stats.AuthenticationID
	return identity, nil
}

type nativeMaintenanceProcess struct {
	handle   windows.Handle
	token    windows.Token
	pid      uint32
	created  windows.Filetime
	identity maintenanceTokenIdentity
	image    *enrollmentPin
}

func openMaintenanceProcess(pid uint32, image *enrollmentPin, sid string, session uint32) (result maintenanceProcess, resultErr error) {
	if pid == 0 || image == nil || sid == "" {
		return nil, errors.New("original native maintenance process inputs required")
	}
	p := &nativeMaintenanceProcess{pid: pid, image: image}
	defer func() {
		if resultErr != nil {
			resultErr = errors.Join(resultErr, enrollmentCleanup(p.close()))
		}
	}()
	var err error
	p.handle, err = windows.OpenProcess(windows.PROCESS_QUERY_LIMITED_INFORMATION|windows.SYNCHRONIZE, false, pid)
	if err != nil {
		return nil, err
	}
	var exited, kernel, user windows.Filetime
	if err := windows.GetProcessTimes(p.handle, &p.created, &exited, &kernel, &user); err != nil || p.created == (windows.Filetime{}) {
		return nil, errors.Join(errors.New("original maintenance process creation time required"), err)
	}
	if err := windows.OpenProcessToken(p.handle, windows.TOKEN_QUERY, &p.token); err != nil {
		return nil, err
	}
	p.identity, err = maintenanceToken(p.token)
	if err != nil || p.identity.sid != sid || p.identity.session != session {
		return nil, errors.Join(errors.New("maintenance process account or session differs"), err)
	}
	if err := p.live(); err != nil {
		return nil, err
	}
	return p, nil
}

func (p *nativeMaintenanceProcess) original() error {
	if p == nil || p.handle == 0 || p.token == 0 || p.image == nil {
		return errors.New("original maintenance process custody required")
	}
	pid, err := windows.GetProcessId(p.handle)
	if err != nil || pid != p.pid {
		return errors.Join(errors.New("retained maintenance process identity changed"), err)
	}
	var created, exited, kernel, user windows.Filetime
	if err := windows.GetProcessTimes(p.handle, &created, &exited, &kernel, &user); err != nil || created != p.created {
		return errors.Join(errors.New("retained maintenance process creation changed"), err)
	}
	identity, err := maintenanceToken(p.token)
	if err != nil || identity != p.identity {
		return errors.Join(errors.New("retained maintenance token identity changed"), err)
	}
	return p.image.check()
}

func (p *nativeMaintenanceProcess) live() error {
	if err := p.original(); err != nil {
		return err
	}
	state, err := windows.WaitForSingleObject(p.handle, 0)
	if err != nil || state != uint32(windows.WAIT_TIMEOUT) {
		return errors.Join(errors.New("original maintenance process must still be live"), err)
	}
	var token windows.Token
	if err := windows.OpenProcessToken(p.handle, windows.TOKEN_QUERY, &token); err != nil {
		return err
	}
	actual, queryErr := maintenanceToken(token)
	closeErr := enrollmentCleanup(token.Close())
	if queryErr != nil || closeErr != nil || actual != p.identity {
		return errors.Join(errors.New("current maintenance primary token changed"), queryErr, closeErr)
	}
	image := make([]uint16, 32768)
	length := uint32(len(image))
	if err := windows.QueryFullProcessImageName(p.handle, windows.PROCESS_NAME_NATIVE, &image[0], &length); err != nil || length == 0 || length >= uint32(len(image)) {
		return errors.Join(errors.New("actual maintenance process image differs from retained protected image"), err)
	}
	actualImage := windows.UTF16ToString(image[:length])
	if !strings.EqualFold(actualImage, p.image.physical) {
		return errors.New("actual maintenance process image path differs")
	}
	if _, err := enrollmentPhysicalParts(actualImage); err != nil {
		return err
	}
	path, err := windows.UTF16PtrFromString(enrollmentPhysical(actualImage))
	if err != nil {
		return err
	}
	// Corroborate the process-reported path against the originally retained
	// image's full native ID. This reopen supplies no trusted bytes/identity.
	handle, err := windows.CreateFile(path, windows.FILE_READ_ATTRIBUTES|windows.READ_CONTROL, windows.FILE_SHARE_READ, nil, windows.OPEN_EXISTING, windows.FILE_FLAG_OPEN_REPARSE_POINT, 0)
	if err != nil {
		return err
	}
	info, id, queryErr := enrollmentInfo(handle)
	closeErr = enrollmentCleanup(windows.CloseHandle(handle))
	if queryErr != nil || closeErr != nil || id != p.image.id || info.CreationTime != p.image.info.CreationTime || info.FileAttributes&windows.FILE_ATTRIBUTE_REPARSE_POINT != 0 {
		return errors.Join(errors.New("actual process image native identity differs"), queryErr, closeErr)
	}
	state, err = windows.WaitForSingleObject(p.handle, 0)
	if err != nil || state != uint32(windows.WAIT_TIMEOUT) {
		return errors.Join(errors.New("original maintenance process exited during binding"), err)
	}
	return p.original()
}

func (p *nativeMaintenanceProcess) exit() (bool, uint32, error) {
	if err := p.original(); err != nil {
		return false, 0, err
	}
	state, err := windows.WaitForSingleObject(p.handle, 0)
	if err != nil {
		return false, 0, err
	}
	if state == uint32(windows.WAIT_TIMEOUT) {
		// The original retained process may exit immediately after this probe.
		// Do not reopen a PID or turn that normal transition into identity loss.
		return false, 0, nil
	}
	if state != windows.WAIT_OBJECT_0 {
		return false, 0, errors.New("original maintenance process exit wait uncertain")
	}
	var code uint32
	if err := windows.GetExitCodeProcess(p.handle, &code); err != nil {
		return false, 0, err
	}
	return true, code, p.original()
}

func (p *nativeMaintenanceProcess) close() error {
	var result error
	if p.token != 0 {
		result = p.token.Close()
		p.token = 0
	}
	if p.handle != 0 {
		result = errors.Join(result, windows.CloseHandle(p.handle))
		p.handle = 0
	}
	return result
}
