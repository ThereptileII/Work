package launcher

import (
	"context"
	"errors"
	"os"
	"path"
	"runtime"
	"sort"
	"strings"
	"time"
	"unicode/utf16"
	"unsafe"

	"example.com/opennav-update-verifier/brokerpipe"
	"example.com/opennav-update-verifier/brokerregistration"
	"golang.org/x/sys/windows"
)

const brokerExecutionCloseBudget = 45 * time.Second

type brokerExecutionTokenStatistics struct {
	TokenID, AuthenticationID                                                                   windows.LUID
	ExpirationTime                                                                              int64
	TokenType, ImpersonationLevel, DynamicCharged, DynamicAvailable, GroupCount, PrivilegeCount uint32
	ModifiedID                                                                                  windows.LUID
}
type brokerExecutionUser struct {
	sid     string
	session uint32
	logon   windows.LUID
}

func brokerExecutionToken(token windows.Token) (out brokerExecutionUser, err error) {
	user, err := token.GetTokenUser()
	if err != nil {
		return out, err
	}
	out.sid = user.User.Sid.String()
	var used uint32
	if err = windows.GetTokenInformation(token, windows.TokenSessionId, (*byte)(unsafe.Pointer(&out.session)), 4, &used); err != nil || used != 4 {
		return out, errors.Join(errBrokerExecutionRefused, err)
	}
	var stats brokerExecutionTokenStatistics
	if err = windows.GetTokenInformation(token, windows.TokenStatistics, (*byte)(unsafe.Pointer(&stats)), uint32(unsafe.Sizeof(stats)), &used); err != nil || used != uint32(unsafe.Sizeof(stats)) {
		return out, errors.Join(errBrokerExecutionRefused, err)
	}
	out.logon = stats.AuthenticationID
	if !brokerregistration.IsCanonicalUserSID(out.sid) || out.session == 0 || out.logon == (windows.LUID{}) {
		return out, errBrokerExecutionRefused
	}
	return out, nil
}
func brokerExecutionCurrentUser() (brokerExecutionUser, error) {
	var thread windows.Token
	err := windows.OpenThreadToken(windows.CurrentThread(), windows.TOKEN_QUERY, true, &thread)
	if err == nil {
		return brokerExecutionUser{}, errors.Join(errBrokerExecutionRefused, thread.Close())
	}
	if !errors.Is(err, windows.ERROR_NO_TOKEN) {
		return brokerExecutionUser{}, errors.Join(errBrokerExecutionRefused, err)
	}
	return brokerExecutionToken(windows.GetCurrentProcessToken())
}

type nativeBrokerExecutionBackend struct {
	user                       brokerExecutionUser
	programFiles, oldParentDOS string
	oldPins                    []nativeOrdinaryDirectory
	queryAccess                *brokerQueryAccess
	closed                     bool
	closeErr                   error
}

func runBrokerExecutionNative(ctx context.Context, stage string) (resultErr error) {
	return runBrokerIntentNative(ctx, stage, "execute-session")
}
func runBrokerCurrentNative(ctx context.Context, stage string) error {
	return runBrokerIntentNative(ctx, stage, "start-current")
}
func runBrokerIntentNative(ctx context.Context, stage, operation string) (resultErr error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	backend, err := newNativeBrokerExecutionBackend(ctx)
	if err != nil {
		return err
	}
	defer func() { resultErr = errors.Join(resultErr, backend.close()) }()
	return runBrokerExecutionIntent(ctx, stage, operation, backend, brokerpipe.CallExecution)
}
func newNativeBrokerExecutionBackend(ctx context.Context) (result *nativeBrokerExecutionBackend, resultErr error) {
	if ctx == nil || ctx.Err() != nil {
		return nil, errors.Join(errBrokerExecutionRefused, contextErrorBroker(ctx))
	}
	user, err := brokerExecutionCurrentUser()
	if err != nil {
		return nil, err
	}
	programFiles, err := windows.KnownFolderPath(windows.FOLDERID_ProgramFilesX86, 0)
	if err != nil {
		return nil, err
	}
	oldParent, err := windows.KnownFolderPath(windows.FOLDERID_LocalAppData, 0)
	if err != nil {
		return nil, err
	}
	b := &nativeBrokerExecutionBackend{user: user, programFiles: programFiles, oldParentDOS: oldParent}
	defer func() {
		if resultErr != nil {
			resultErr = errors.Join(resultErr, b.close())
		}
	}()
	if err = plainNativePath(oldParent); err != nil {
		return nil, errors.Join(errArmedUpdateBarrier, err)
	}
	original, err := openNativeArmedDirectory(oldParent)
	if err != nil {
		return nil, errors.Join(errArmedUpdateBarrier, err)
	}
	physical := original.name
	closeErr := windows.CloseHandle(original.handle)
	if closeErr != nil {
		return nil, errors.Join(errBrokerExecutionUncertain, closeErr)
	}
	_, parents, err := physicalAncestors(physical + `\sentinel`)
	if err != nil || len(parents) > 256 {
		return nil, errors.Join(errArmedUpdateBarrier, err)
	}
	for index, parent := range parents {
		name := physicalWin32(parent)
		if index == 0 {
			name += `\`
		}
		pin, err := openNativeArmedDirectory(name)
		if err != nil {
			return nil, errors.Join(errArmedUpdateBarrier, err)
		}
		b.oldPins = append(b.oldPins, pin)
		if !strings.EqualFold(pin.name, parent) {
			return nil, errArmedUpdateBarrier
		}
	}
	if err = b.check(ctx); err != nil {
		return nil, err
	}
	// This explicit mode cooperatively permits only the fixed broker observer.
	// Its process/token query policy is independent from application authority.
	b.queryAccess, err = prepareBrokerQueryAccessNative(ctx)
	if err != nil {
		return nil, err
	}
	return b, nil
}
func (b *nativeBrokerExecutionBackend) check(ctx context.Context) error {
	if b == nil || b.closed || ctx == nil || len(b.oldPins) == 0 {
		return errBrokerExecutionRefused
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	if b.queryAccess != nil {
		if err := b.queryAccess.Validate(ctx); err != nil {
			return err
		}
	}
	actual, err := brokerExecutionCurrentUser()
	if err != nil || actual != b.user {
		return errors.Join(errBrokerExecutionRefused, err)
	}
	for _, pin := range b.oldPins {
		var info windows.ByHandleFileInformation
		name, err := nativePhysicalName(pin.handle)
		if err != nil || windows.GetFileInformationByHandle(pin.handle, &info) != nil || !sameNativeIdentity(pin.info, info) || info.FileAttributes&(windows.FILE_ATTRIBUTE_DIRECTORY|windows.FILE_ATTRIBUTE_REPARSE_POINT) != windows.FILE_ATTRIBUTE_DIRECTORY || !strings.EqualFold(strings.TrimSuffix(name, `\`), pin.name) {
			return errors.Join(errArmedUpdateBarrier, err)
		}
	}
	current, err := openNativeArmedDirectory(b.oldParentDOS)
	if err != nil {
		return errors.Join(errArmedUpdateBarrier, err)
	}
	original := b.oldPins[len(b.oldPins)-1]
	same := sameNativeIdentity(original.info, current.info) && strings.EqualFold(original.name, current.name)
	closeErr := windows.CloseHandle(current.handle)
	if !same || closeErr != nil {
		return errors.Join(errArmedUpdateBarrier, closeErr)
	}
	return brokerExecutionOldMarkerAbsent(original.name)
}

// An exact absent old root is acceptable. Existing old roots remain denial-only:
// no state/receipt/ACL/installation is adopted and no bytes are repaired/deleted.
func brokerExecutionOldMarkerAbsent(parent string) error {
	name := parent + `\OpenNavXAlpha1`
	pointer, err := windows.UTF16PtrFromString(physicalWin32(name))
	if err != nil {
		return errors.Join(errArmedUpdateBarrier, err)
	}
	attributes, err := windows.GetFileAttributes(pointer)
	if err == windows.ERROR_FILE_NOT_FOUND {
		return nil
	}
	if err != nil || attributes&(windows.FILE_ATTRIBUTE_DIRECTORY|windows.FILE_ATTRIBUTE_REPARSE_POINT) != windows.FILE_ATTRIBUTE_DIRECTORY {
		return errors.Join(errArmedUpdateBarrier, err)
	}
	pin, err := openNativeArmedDirectory(physicalWin32(name))
	if err != nil {
		return errors.Join(errArmedUpdateBarrier, err)
	}
	var result error
	if !strings.EqualFold(pin.name, name) {
		result = errArmedUpdateBarrier
	} else {
		result = nativeArmedPhysicalLeafAbsent(pin.name)
	}
	return errors.Join(result, windows.CloseHandle(pin.handle))
}
func (b *nativeBrokerExecutionBackend) close() error {
	if b == nil {
		return nil
	}
	if b.closed {
		return b.closeErr
	}
	b.closed = true
	if b.queryAccess != nil {
		b.closeErr = errors.Join(b.closeErr, b.queryAccess.Close())
		b.queryAccess = nil
	}
	for i := len(b.oldPins) - 1; i >= 0; i-- {
		b.closeErr = errors.Join(b.closeErr, windows.CloseHandle(b.oldPins[i].handle))
	}
	b.oldPins = nil
	if b.closeErr != nil {
		b.closeErr = errors.Join(errBrokerExecutionUncertain, b.closeErr)
	}
	return b.closeErr
}
func brokerExecutionEnvironment(values []string, c brokerpipe.ExecutionCommand) ([]uint16, error) {
	if brokerExecutionPayload(c, c.StageID) != nil {
		return nil, errBrokerExecutionRefused
	}
	if c.Phase != brokerpipe.SpawnCandidate && c.Phase != brokerpipe.SpawnFallback {
		return nil, errBrokerExecutionRefused
	}
	// Reuse ordinary validation; ordinary creation itself continues stripping all
	// health variables. Only this separately named creator adds the exact4 fields.
	if _, err := ordinaryEnvironment(values); err != nil {
		return nil, err
	}
	kept := make([]string, 0, len(values)+4)
	for _, value := range values {
		key, _, _ := strings.Cut(value, "=")
		if !strings.HasPrefix(strings.ToUpper(key), "SKAGER_UPDATE_") {
			kept = append(kept, value)
		}
	}
	kept = append(kept, "SKAGER_UPDATE_PIPE="+c.HealthPipe, "SKAGER_UPDATE_GENERATION="+c.ApplicationGeneration, "SKAGER_UPDATE_CHALLENGE="+c.HealthChallenge, "SKAGER_UPDATE_QUALIFY_PROFILE=1")
	sort.Slice(kept, func(i, j int) bool { return strings.ToUpper(kept[i]) < strings.ToUpper(kept[j]) })
	block := utf16.Encode([]rune(strings.Join(kept, "\x00") + "\x00\x00"))
	if len(block) > 32767 {
		return nil, errBrokerExecutionRefused
	}
	return block, nil
}

// This creator is private and called only by an authenticated broker directive.
// It never changes ordinaryEnvironment/create, accepts no application arguments
// and accepts only the separately prepared narrow process security attributes.
// Ordinary creation still uses its unchanged default security policy.
func (image *nativeOrdinaryImage) createBrokerExecution(c brokerpipe.ExecutionCommand, attributes *windows.SecurityAttributes) (*nativeOrdinaryChild, error) {
	environment, err := brokerExecutionEnvironment(os.Environ(), c)
	if err != nil {
		return nil, err
	}
	return image.createBrokerChild(attributes, environment)
}

// Environment is constructed only by the two private normal/qualification
// producers. Application, arguments and working directory remain fixed here.
func (image *nativeOrdinaryImage) createBrokerChild(attributes *windows.SecurityAttributes, environment []uint16) (*nativeOrdinaryChild, error) {
	return image.createBrokerFixedChild(attributes, environment, "--xnav")
}

// argument comes only from fixed private normal/qualification/relay producers.
func (image *nativeOrdinaryImage) createBrokerFixedChild(attributes *windows.SecurityAttributes, environment []uint16, argument string) (*nativeOrdinaryChild, error) {
	physical := physicalWin32(image.name)
	application, err := windows.UTF16PtrFromString(image.application)
	if err != nil {
		return nil, err
	}
	command, err := windows.UTF16FromString(windows.ComposeCommandLine([]string{physical, argument}))
	if err != nil || len(command) > 32767 {
		return nil, errBrokerExecutionRefused
	}
	directory, err := windows.UTF16PtrFromString(physical[:strings.LastIndex(physical, `\`)])
	if err != nil {
		return nil, err
	}
	startup := windows.StartupInfo{Cb: uint32(unsafe.Sizeof(windows.StartupInfo{})), Flags: windows.STARTF_USESHOWWINDOW, ShowWindow: windows.SW_HIDE}
	var process windows.ProcessInformation
	if err = windows.CreateProcess(application, &command[0], attributes, nil, false, windows.CREATE_SUSPENDED|windows.CREATE_UNICODE_ENVIRONMENT, &environment[0], directory, &startup, &process); err != nil {
		return nil, errors.Join(errBrokerExecutionUncertain, err)
	}
	return &nativeOrdinaryChild{process: process.Process, thread: process.Thread, pid: process.ProcessId, tid: process.ThreadId, image: image.name}, nil
}

type nativeBrokerExecutionChild struct {
	backend     *nativeBrokerExecutionBackend
	image       *nativeOrdinaryImage
	child       *nativeOrdinaryChild
	token       windows.Token
	queryAccess *brokerQueryAccess
	created     uint64
	closed      bool
	closeErr    error
}

func (b *nativeBrokerExecutionBackend) spawn(ctx context.Context, c brokerpipe.ExecutionCommand) (result brokerExecutionChild, resultErr error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if err := b.check(ctx); err != nil {
		return nil, err
	}
	if brokerExecutionPayload(c, c.StageID) != nil {
		return nil, errBrokerExecutionRefused
	}
	executable, err := brokerExecutionApplication(b.programFiles, b.user.sid, c.ApplicationGeneration)
	if c.Phase == brokerpipe.RelaySelectedExecutor {
		executable, err = brokerExecutionLauncher(b.programFiles, b.user.sid, c.ApplicationGeneration)
	}
	if err != nil {
		return nil, err
	}
	// This locally observed digest establishes byte continuity only. It is not
	// installed/signed/approved provenance; broker's live ApplicationImage owner
	// independently corroborates this actual process against protected originals.
	initial, err := openNativeRead(executable)
	if err != nil {
		return nil, err
	}
	info, err := initial.Stat()
	if err == nil {
		err = validatePE32(initial, info.Size())
	}
	var measured string
	if err == nil {
		measured, err = hashFile(initial)
	}
	closeErr := initial.Close()
	if err != nil || closeErr != nil {
		return nil, errors.Join(err, closeErr)
	}
	spec := processSpec{executable: executable, directory: path.Dir(executable), expectedImageHash: measured}
	acquire := acquireNativeOrdinaryImage
	if c.Phase == brokerpipe.RelaySelectedExecutor {
		acquire = acquireNativeRelayImage
	}
	acquired, err := acquire(spec)
	if err != nil {
		return nil, err
	}
	owner := &nativeBrokerExecutionChild{backend: b, image: acquired.(*nativeOrdinaryImage)}
	defer func() {
		if resultErr != nil {
			if owner.child != nil && !owner.child.resumeAttempted {
				resultErr = errors.Join(resultErr, owner.child.abortSuspended())
			}
			resultErr = errors.Join(resultErr, owner.close())
		}
	}()
	if err = owner.guard(ctx); err != nil {
		return nil, err
	}
	// Each already-consumed role gets one birth owner. The exact existing executor
	// query ACE is idempotent; failed attributes/bind cannot be retried.
	owner.queryAccess, err = prepareBrokerQueryAccessNative(ctx)
	if err != nil {
		return nil, err
	}
	attributes, err := owner.queryAccess.childProcessAttributes()
	if err != nil || attributes == nil {
		return nil, errors.Join(errBrokerExecutionRefused, err)
	}
	if c.Phase == brokerpipe.RelaySelectedExecutor {
		owner.child, err = owner.image.createBrokerRelay(c, attributes)
	} else if c.Phase == brokerpipe.SpawnCurrent {
		owner.child, err = owner.image.createBrokerCurrent(c, attributes)
	} else {
		owner.child, err = owner.image.createBrokerExecution(c, attributes)
	}
	if err != nil {
		return nil, err
	}
	if err = owner.child.corroborate(); err != nil {
		return nil, err
	}
	if err = owner.queryAccess.bindSuspendedChild(ctx, owner.child); err != nil {
		return nil, err
	}
	owner.created = filetimeValue(owner.child.creation)
	if err = windows.OpenProcessToken(owner.child.process, windows.TOKEN_QUERY, &owner.token); err != nil {
		return nil, err
	}
	if err = owner.liveIdentity(ctx); err != nil {
		return nil, err
	}
	if err = owner.guard(ctx); err != nil {
		return nil, err
	}
	count, err := owner.child.resume()
	if err != nil || count != 1 {
		return nil, errors.Join(errBrokerExecutionUncertain, err)
	}
	return owner, nil
}
func (o *nativeBrokerExecutionChild) guard(ctx context.Context) error {
	if err := o.guardImage(ctx); err != nil {
		return err
	}
	if o.queryAccess != nil {
		return o.queryAccess.Validate(ctx)
	}
	return nil
}

// The image/root/user guard also applies to an exited original child. Its live
// query owner is checked separately before any resumed-process operation.
func (o *nativeBrokerExecutionChild) guardImage(ctx context.Context) error {
	if o == nil || o.closed || o.image == nil {
		return errBrokerExecutionRefused
	}
	if err := o.backend.check(ctx); err != nil {
		return err
	}
	if err := o.image.recheck(); err != nil {
		return err
	}
	root, err := managedPhysicalInstallationRoot(o.image.name, path.Base(path.Dir(path.Dir(strings.ReplaceAll(o.image.application, `\`, "/")))))
	if err != nil {
		return err
	}
	for _, pin := range o.image.parents {
		if strings.EqualFold(pin.name, root) {
			return nativeArmedPhysicalLeafAbsent(root)
		}
	}
	return errArmedUpdateBarrier
}
func (o *nativeBrokerExecutionChild) liveIdentity(ctx context.Context) error {
	if o == nil || o.closed || o.child == nil || o.token == 0 || ctx == nil {
		return errBrokerExecutionRefused
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	created, err := o.child.rawIdentity()
	if err != nil || filetimeValue(created) != o.created {
		return errors.Join(errBrokerExecutionRefused, err)
	}
	if err = o.child.corroborate(); err != nil {
		return err
	}
	original, err := brokerExecutionToken(o.token)
	if err != nil || original != o.backend.user {
		return errors.Join(errBrokerExecutionRefused, err)
	}
	var current windows.Token
	if err = windows.OpenProcessToken(o.child.process, windows.TOKEN_QUERY, &current); err != nil {
		return err
	}
	actual, tokenErr := brokerExecutionToken(current)
	closeErr := current.Close()
	if tokenErr != nil || closeErr != nil || actual != original {
		return errors.Join(errBrokerExecutionRefused, tokenErr, closeErr)
	}
	return ctx.Err()
}
func (o *nativeBrokerExecutionChild) observe(ctx context.Context) (brokerExecutionChildInfo, error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if err := o.guard(ctx); err != nil {
		return brokerExecutionChildInfo{}, err
	}
	if err := o.liveIdentity(ctx); err != nil {
		return brokerExecutionChildInfo{}, err
	}
	return o.information(), nil
}
func (o *nativeBrokerExecutionChild) information() brokerExecutionChildInfo {
	return brokerExecutionChildInfo{o.child.pid, o.created, o.image.expected}
}

var brokerExecutionPostMessage = windows.NewLazySystemDLL("user32.dll").NewProc("PostMessageW")

// Only an original creator handle can establish exit. A PID or reported close
// never supplies this proof; an unowned/changed creator is refused even if its
// handle is signaled. The caller separately revalidates image/root custody.
func brokerExecutionOriginalExited(child *nativeOrdinaryChild, expected uint64) (bool, error) {
	if child == nil || !child.owned || expected == 0 {
		return false, errBrokerExecutionRefused
	}
	created, err := child.rawIdentity()
	if err != nil || filetimeValue(created) != expected {
		return false, errors.Join(errBrokerExecutionUncertain, err)
	}
	status, err := windows.WaitForSingleObject(child.process, 0)
	if err != nil {
		return false, err
	}
	switch status {
	case windows.WAIT_OBJECT_0:
		return true, nil
	case uint32(windows.WAIT_TIMEOUT):
		return false, nil
	}
	return false, errBrokerExecutionUncertain
}

// Exit can race a live token/window check. Revalidate custody and the original
// retained process before accepting closure; never reopen by PID or retry close.
func (o *nativeBrokerExecutionChild) closeFailure(ctx context.Context, cause error) (brokerExecutionChildInfo, error) {
	if err := o.guardImage(ctx); err != nil {
		return brokerExecutionChildInfo{}, errors.Join(cause, err)
	}
	exited, err := brokerExecutionOriginalExited(o.child, o.created)
	if err == nil && exited {
		return o.information(), nil
	}
	return brokerExecutionChildInfo{}, errors.Join(cause, err)
}

func (o *nativeBrokerExecutionChild) closeApplication(ctx context.Context) (brokerExecutionChildInfo, error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if err := o.guardImage(ctx); err != nil {
		return brokerExecutionChildInfo{}, err
	}
	created, err := o.child.rawIdentity()
	if err != nil || filetimeValue(created) != o.created {
		return brokerExecutionChildInfo{}, errors.Join(errBrokerExecutionUncertain, err)
	}
	status, err := windows.WaitForSingleObject(o.child.process, 0)
	if err != nil {
		return brokerExecutionChildInfo{}, err
	}
	if status == windows.WAIT_OBJECT_0 {
		return o.information(), nil
	}
	if o.queryAccess == nil {
		return brokerExecutionChildInfo{}, errBrokerExecutionRefused
	}
	if err = o.queryAccess.Validate(ctx); err != nil {
		return o.closeFailure(ctx, err)
	}
	if err = o.liveIdentity(ctx); err != nil {
		return o.closeFailure(ctx, err)
	}
	found := false
	var callbackErr error
	callback := windows.NewCallback(func(hwnd uintptr, _ uintptr) uintptr {
		var pid uint32
		if _, err := windows.GetWindowThreadProcessId(windows.HWND(hwnd), &pid); err != nil {
			return 1
		}
		if pid != o.child.pid {
			return 1
		}
		if err := o.liveIdentity(ctx); err != nil {
			callbackErr = err
			return 0
		}
		// PID only chooses a window of the still-live retained creator child. It
		// never selects a process to terminate. Requery immediately before WM_CLOSE.
		var current uint32
		if _, err := windows.GetWindowThreadProcessId(windows.HWND(hwnd), &current); err != nil || current != pid {
			callbackErr = errors.Join(errBrokerExecutionUncertain, err)
			return 0
		}
		ok, _, err := brokerExecutionPostMessage.Call(hwnd, 0x0010, 0, 0)
		if ok == 0 {
			callbackErr = errors.Join(errBrokerExecutionUncertain, err)
			return 0
		}
		found = true
		return 1
	})
	enumErr := windows.EnumWindows(callback, nil)
	runtime.KeepAlive(callback)
	if callbackErr != nil {
		return o.closeFailure(ctx, callbackErr)
	}
	if enumErr != nil || !found {
		return o.closeFailure(ctx, errors.Join(errBrokerExecutionUncertain, enumErr))
	}
	deadline := time.Now().Add(brokerExecutionCloseBudget)
	for time.Now().Before(deadline) {
		if err = ctx.Err(); err != nil {
			return brokerExecutionChildInfo{}, err
		}
		status, err = windows.WaitForSingleObject(o.child.process, 100)
		if err != nil {
			return brokerExecutionChildInfo{}, err
		}
		if status == windows.WAIT_OBJECT_0 {
			return o.information(), nil
		}
		if status != uint32(windows.WAIT_TIMEOUT) {
			return brokerExecutionChildInfo{}, errBrokerExecutionUncertain
		}
		if err = o.liveIdentity(ctx); err != nil {
			return o.closeFailure(ctx, err)
		}
	}
	return brokerExecutionChildInfo{}, errBrokerExecutionUncertain
}
func (o *nativeBrokerExecutionChild) close() error {
	if o == nil {
		return nil
	}
	if o.closed {
		return o.closeErr
	}
	o.closed = true
	if o.queryAccess != nil {
		o.closeErr = errors.Join(o.closeErr, o.queryAccess.Close())
		o.queryAccess = nil
	}
	if o.token != 0 {
		o.closeErr = errors.Join(o.closeErr, o.token.Close())
		o.token = 0
	}
	if o.child != nil {
		o.closeErr = errors.Join(o.closeErr, o.child.Close())
		o.child = nil
	}
	if o.image != nil {
		o.closeErr = errors.Join(o.closeErr, o.image.Close())
		o.image = nil
	}
	if o.closeErr != nil {
		o.closeErr = errors.Join(errBrokerExecutionUncertain, o.closeErr)
	}
	return o.closeErr
}
