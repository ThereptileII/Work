package brokerhost

import (
	"context"
	"crypto/rand"
	"encoding/hex"
	"errors"
	"io"
	"os"
	"runtime"
	"time"
	"unsafe"

	"example.com/opennav-update-verifier/brokerpipe"
	"example.com/opennav-update-verifier/brokerregistration"
	"example.com/opennav-update-verifier/brokerstore"
	"golang.org/x/sys/windows"
)

const executionChildClientRights = windows.FILE_WRITE_DATA | windows.READ_CONTROL | windows.SYNCHRONIZE

var executionImpersonate = windows.NewLazySystemDLL("advapi32.dll").NewProc("ImpersonateNamedPipeClient")

type executionTokenIdentity struct {
	sid     string
	session uint32
	logon   windows.LUID
}
type executionTokenStatistics struct {
	TokenID, AuthenticationID                                                                   windows.LUID
	ExpirationTime                                                                              int64
	TokenType, ImpersonationLevel, DynamicCharged, DynamicAvailable, GroupCount, PrivilegeCount uint32
	ModifiedID                                                                                  windows.LUID
}

func executionToken(t windows.Token) (executionTokenIdentity, error) {
	var out executionTokenIdentity
	user, err := t.GetTokenUser()
	if err != nil {
		return out, err
	}
	out.sid = user.User.Sid.String()
	var used uint32
	if err = windows.GetTokenInformation(t, windows.TokenSessionId, (*byte)(unsafe.Pointer(&out.session)), 4, &used); err != nil || used != 4 {
		return out, errors.Join(ErrExecutionChildRefused, err)
	}
	var stats executionTokenStatistics
	if err = windows.GetTokenInformation(t, windows.TokenStatistics, (*byte)(unsafe.Pointer(&stats)), uint32(unsafe.Sizeof(stats)), &used); err != nil || used != uint32(unsafe.Sizeof(stats)) {
		return out, errors.Join(ErrExecutionChildRefused, err)
	}
	out.logon = stats.AuthenticationID
	if !brokerregistration.IsCanonicalUserSID(out.sid) || out.session == 0 || out.logon == (windows.LUID{}) {
		return out, ErrExecutionChildRefused
	}
	return out, nil
}

type executionProcess struct {
	handle   windows.Handle
	token    windows.Token
	pid      uint32
	created  uint64
	identity executionTokenIdentity
	parent   uint32
}

func openExecutionProcess(pid uint32) (result *executionProcess, resultErr error) {
	if pid == 0 {
		return nil, ErrExecutionChildRefused
	}
	// QUERY_INFORMATION is also needed for native parent information. No debug
	// privilege, impersonated file access or mutation right is requested/granted.
	h, err := windows.OpenProcess(windows.PROCESS_QUERY_INFORMATION|windows.SYNCHRONIZE, false, pid)
	if err != nil {
		return nil, err
	}
	p := &executionProcess{handle: h, pid: pid}
	defer func() {
		if resultErr != nil {
			resultErr = errors.Join(resultErr, p.close())
		}
	}()
	var c, x, k, u windows.Filetime
	if err = windows.GetProcessTimes(h, &c, &x, &k, &u); err != nil {
		return nil, err
	}
	p.created = uint64(c.HighDateTime)<<32 | uint64(c.LowDateTime)
	var basic windows.PROCESS_BASIC_INFORMATION
	var used uint32
	if err = windows.NtQueryInformationProcess(h, windows.ProcessBasicInformation, unsafe.Pointer(&basic), uint32(unsafe.Sizeof(basic)), &used); err != nil || used != uint32(unsafe.Sizeof(basic)) || basic.UniqueProcessId != uintptr(pid) || basic.InheritedFromUniqueProcessId > uintptr(^uint32(0)) {
		return nil, errors.Join(ErrExecutionChildRefused, err)
	}
	p.parent = uint32(basic.InheritedFromUniqueProcessId)
	if err = windows.OpenProcessToken(h, windows.TOKEN_QUERY, &p.token); err != nil {
		return nil, err
	}
	p.identity, err = executionToken(p.token)
	if err != nil {
		return nil, err
	}
	if err = p.check(); err != nil {
		return nil, err
	}
	return p, nil
}
func (p *executionProcess) check() error {
	if p == nil || p.handle == 0 || p.token == 0 {
		return ErrExecutionChildRefused
	}
	status, err := windows.WaitForSingleObject(p.handle, 0)
	if err != nil || status != uint32(windows.WAIT_TIMEOUT) {
		return errors.Join(ErrExecutionChildRefused, err)
	}
	var c, x, k, u windows.Filetime
	if err = windows.GetProcessTimes(p.handle, &c, &x, &k, &u); err != nil || uint64(c.HighDateTime)<<32|uint64(c.LowDateTime) != p.created {
		return errors.Join(ErrExecutionChildRefused, err)
	}
	original, err := executionToken(p.token)
	if err != nil || original != p.identity {
		return errors.Join(ErrExecutionChildRefused, err)
	}
	var current windows.Token
	if err = windows.OpenProcessToken(p.handle, windows.TOKEN_QUERY, &current); err != nil {
		return err
	}
	actual, infoErr := executionToken(current)
	closeErr := current.Close()
	if infoErr != nil || closeErr != nil || actual != p.identity {
		return errors.Join(ErrExecutionChildRefused, infoErr, closeErr)
	}
	return nil
}
func (p *executionProcess) close() error {
	if p == nil {
		return nil
	}
	var err error
	if p.token != 0 {
		err = p.token.Close()
		p.token = 0
	}
	if p.handle != 0 {
		err = errors.Join(err, windows.CloseHandle(p.handle))
		p.handle = 0
	}
	return err
}
func executionProcessMatchesPeer(p *executionProcess, peer brokerpipe.PeerIdentity) error {
	if p == nil || p.pid != peer.ProcessID || p.created != peer.ProcessCreation || p.identity.sid != peer.SID || p.identity.session != peer.SessionID || p.identity.logon.HighPart != peer.LogonIDHigh || p.identity.logon.LowPart != peer.LogonIDLow {
		return ErrExecutionChildRefused
	}
	return p.check()
}

type nativeExecutionChild struct {
	pipe              windows.Handle
	connection        *executionChildIO
	parent, child     *executionProcess
	image             *brokerstore.GenerationApplicationImage
	prepared          uint64
	prefetched        []byte
	prefetchedErr     error
	connected, closed bool
	closeErr          error
}

func prepareExecutionChildNative(ctx context.Context, authority executionChildAuthority, image *brokerstore.GenerationApplicationImage, identity brokerstore.GenerationApplicationIdentity, profile *ExecutionColdProfile) (result *ExecutionChildObserver, resultErr error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if err := executionNoImpersonation(); err != nil {
		return nil, err
	}
	var native *nativeExecutionChild
	err := authority.withExecutor(ctx, func(peer brokerpipe.PeerIdentity) error {
		if peer.SID != identity.ClientSID {
			return ErrExecutionChildRefused
		}
		user, err := windows.GetCurrentProcessToken().GetTokenUser()
		if err != nil {
			return err
		}
		var environment map[string]string
		var prepared time.Time
		native, environment, prepared, err = newNativeExecutionChild(ctx, peer, image, user.User.Sid.String())
		if err != nil {
			return err
		}
		environment["SKAGER_UPDATE_GENERATION"] = identity.Generation
		environment["SKAGER_UPDATE_QUALIFY_PROFILE"] = "1"
		result = newExecutionChildObserver(ctx, authority, peer, identity, native, profile, prepared, environment)
		return nil
	})
	if err != nil {
		if result != nil {
			err = errors.Join(err, result.Close())
		} else if native != nil {
			err = errors.Join(err, native.close())
		}
		return nil, err
	}
	return result, nil
}

// The public path obtains peer only from live WithExecutor and image only from
// retained original generation custody. This lower helper permits inert native
// fixtures to test pipe/process mechanics without claiming product execution.
func newNativeExecutionChild(ctx context.Context, peer brokerpipe.PeerIdentity, image *brokerstore.GenerationApplicationImage, brokerSID string) (result *nativeExecutionChild, environment map[string]string, prepared time.Time, resultErr error) {
	b := &nativeExecutionChild{image: image}
	defer func() {
		if resultErr != nil {
			resultErr = errors.Join(resultErr, b.close())
		}
	}()
	if ctx == nil || ctx.Err() != nil || !brokerregistration.IsCanonicalUserSID(peer.SID) {
		return nil, nil, prepared, ErrExecutionChildRefused
	}
	var err error
	b.parent, err = openExecutionProcess(peer.ProcessID)
	if err != nil {
		return nil, nil, prepared, err
	}
	if err = executionProcessMatchesPeer(b.parent, peer); err != nil {
		return nil, nil, prepared, err
	}
	var random [48]byte
	if _, err = rand.Read(random[:]); err != nil {
		return nil, nil, prepared, err
	}
	name := "Skager.Update." + hex.EncodeToString(random[:16])
	challenge := hex.EncodeToString(random[16:])
	pipeName, err := windows.UTF16PtrFromString(`\\.\pipe\` + name)
	if err != nil {
		return nil, nil, prepared, err
	}
	// Exact revised C++ client rights exclude APPEND_DATA/CREATE_PIPE_INSTANCE.
	// Old GENERIC_WRITE producers are intentionally not qualified by this ACL.
	sd, err := executionChildPipeDescriptor(brokerSID, peer.SID)
	if err != nil {
		return nil, nil, prepared, err
	}
	sa := windows.SecurityAttributes{Length: uint32(unsafe.Sizeof(windows.SecurityAttributes{})), SecurityDescriptor: sd}
	b.pipe, err = windows.CreateNamedPipe(pipeName, windows.PIPE_ACCESS_INBOUND|windows.FILE_FLAG_FIRST_PIPE_INSTANCE|windows.FILE_FLAG_OVERLAPPED, windows.PIPE_REJECT_REMOTE_CLIENTS, 1, 256, 1024, 0, &sa)
	runtime.KeepAlive(sd)
	if err != nil {
		b.pipe = 0
		return nil, nil, prepared, err
	}
	var now windows.Filetime
	windows.GetSystemTimeAsFileTime(&now)
	// Take the monotonic anchor after the native wall-clock sample. This is
	// conservative: sampling latency cannot shorten the30s child-age floor.
	prepared = time.Now()
	b.prepared = uint64(now.HighDateTime)<<32 | uint64(now.LowDateTime)
	// Actually arm connection before returning fields to the interactive caller.
	b.connection, err = beginExecutionChildIO(b.pipe, func(ov *windows.Overlapped) error { return windows.ConnectNamedPipe(b.pipe, ov) })
	if err != nil {
		return nil, nil, prepared, err
	}
	return b, map[string]string{"SKAGER_UPDATE_PIPE": name, "SKAGER_UPDATE_CHALLENGE": challenge}, prepared, nil
}
func executionChildPipeDescriptor(broker, client string) (*windows.SECURITY_DESCRIPTOR, error) {
	if broker == "" || !brokerregistration.IsCanonicalUserSID(client) {
		return nil, ErrExecutionChildRefused
	}
	return windows.SecurityDescriptorFromString("O:" + broker + "D:P(A;;GA;;;" + broker + ")(A;;GA;;;SY)(A;;0x00120002;;;" + client + ")")
}
func (b *nativeExecutionChild) check(ctx context.Context) error {
	if ctx == nil || b == nil || b.closed || b.pipe == 0 {
		return ErrExecutionChildRefused
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	if err := b.parent.check(); err != nil {
		return err
	}
	if b.child != nil {
		if err := b.child.check(); err != nil {
			return err
		}
		if b.child.parent != b.parent.pid || b.child.identity != b.parent.identity || b.child.created <= b.parent.created || b.child.created < b.prepared {
			return ErrExecutionChildRefused
		}
	}
	return ctx.Err()
}
func (b *nativeExecutionChild) connect(ctx context.Context) error {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if err := executionNoImpersonation(); err != nil {
		return err
	}
	if b.connected || b.connection == nil {
		return ErrExecutionChildRefused
	}
	_, err := b.connection.finish(ctx, func() error { return b.check(ctx) })
	if err != nil {
		return err
	}
	b.connected = true
	return b.check(ctx)
}
func (b *nativeExecutionChild) authenticate(ctx context.Context) (time.Duration, error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if err := executionNoImpersonation(); err != nil {
		return 0, err
	}
	if !b.connected || b.child != nil {
		return 0, ErrExecutionChildRefused
	}
	var pid uint32
	if err := windows.GetNamedPipeClientProcessId(b.pipe, &pid); err != nil {
		return 0, err
	}
	child, err := openExecutionProcess(pid)
	if err != nil {
		return 0, err
	}
	b.child = child
	if err = b.check(ctx); err != nil {
		return 0, err
	}
	if b.image == nil {
		return 0, ErrExecutionChildRefused
	}
	if err = b.image.MatchesProcess(ctx, child.pid, child.created); err != nil {
		return 0, err
	}
	// Impersonation identity is queried only after a bounded read establishes the
	// last client message. These bytes remain untrusted until native authentication.
	buffer := make([]byte, 513)
	n, readErr := b.readNative(ctx, buffer)
	if n < 1 || n > len(buffer) || (readErr != nil && !errors.Is(readErr, io.EOF)) {
		return 0, errors.Join(ErrExecutionChildRefused, readErr)
	}
	b.prefetched = append([]byte(nil), buffer[:n]...)
	b.prefetchedErr = readErr
	token, err := executionHealthPeerToken(b.pipe)
	if err != nil || token != child.identity {
		return 0, errors.Join(ErrExecutionChildRefused, err)
	}
	if err = b.check(ctx); err != nil {
		return 0, err
	}
	// Anchor the native child birth offset to preparation's monotonic clock. A
	// system-clock forward step can only delay/refuse observation, never shorten
	// the required actual-child lifetime below30s relative to that anchor.
	offset := child.created - b.prepared
	if offset > uint64(executionObservationTimeout/(100*time.Nanosecond)) {
		return 0, ErrExecutionChildRefused
	}
	return time.Duration(offset)*100*time.Nanosecond + executionMinimumReadyAge, nil
}
func executionHealthPeerToken(pipe windows.Handle) (result executionTokenIdentity, resultErr error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	ok, _, err := executionImpersonate.Call(uintptr(pipe))
	if ok == 0 {
		return result, errors.Join(ErrExecutionChildRefused, err)
	}
	// No privileged filesystem/image/process work occurs under this token.
	// Failure to revert is fail-stop of this broker only, never peer termination.
	defer func() {
		if windows.RevertToSelf() != nil {
			os.Exit(1)
		}
	}()
	var token windows.Token
	if err = windows.OpenThreadToken(windows.CurrentThread(), windows.TOKEN_QUERY, true, &token); err != nil {
		return result, err
	}
	result, resultErr = executionToken(token)
	resultErr = errors.Join(resultErr, token.Close())
	return result, resultErr
}
func (b *nativeExecutionChild) read(ctx context.Context, buffer []byte) (int, error) {
	if len(b.prefetched) > 0 {
		n := copy(buffer, b.prefetched)
		b.prefetched = b.prefetched[n:]
		if len(b.prefetched) == 0 {
			err := b.prefetchedErr
			b.prefetchedErr = nil
			return n, err
		}
		return n, nil
	}
	return b.readNative(ctx, buffer)
}
func (b *nativeExecutionChild) readNative(ctx context.Context, buffer []byte) (int, error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if err := executionNoImpersonation(); err != nil {
		return 0, err
	}
	if err := b.check(ctx); err != nil {
		return 0, err
	}
	operation, err := beginExecutionChildIO(b.pipe, func(ov *windows.Overlapped) error { return windows.ReadFile(b.pipe, buffer, nil, ov) })
	if err != nil {
		if errors.Is(err, windows.ERROR_BROKEN_PIPE) || errors.Is(err, windows.ERROR_PIPE_NOT_CONNECTED) {
			return 0, io.EOF
		}
		return 0, err
	}
	n, err := operation.finish(ctx, func() error { return b.check(ctx) })
	runtime.KeepAlive(buffer)
	if errors.Is(err, windows.ERROR_BROKEN_PIPE) || errors.Is(err, windows.ERROR_PIPE_NOT_CONNECTED) {
		return int(n), io.EOF
	}
	return int(n), err
}
func (b *nativeExecutionChild) validate(ctx context.Context) error {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if err := executionNoImpersonation(); err != nil {
		return err
	}
	if err := b.check(ctx); err != nil {
		return err
	}
	if b.image == nil {
		return ErrExecutionChildRefused
	}
	if b.child == nil {
		if _, err := b.image.Identity(ctx); err != nil {
			return err
		}
	} else {
		if err := b.image.MatchesProcess(ctx, b.child.pid, b.child.created); err != nil {
			return err
		}
	}
	return b.check(ctx)
}
func (b *nativeExecutionChild) close() error {
	if b == nil {
		return nil
	}
	if b.closed {
		return b.closeErr
	}
	b.closed = true
	if b.connection != nil {
		b.closeErr = errors.Join(b.closeErr, b.connection.cancelAndClose())
		b.connection = nil
	}
	if b.pipe != 0 {
		err := windows.DisconnectNamedPipe(b.pipe)
		if errors.Is(err, windows.ERROR_PIPE_NOT_CONNECTED) || errors.Is(err, windows.ERROR_NO_DATA) || errors.Is(err, windows.ERROR_BROKEN_PIPE) {
			err = nil
		}
		b.closeErr = errors.Join(b.closeErr, err, windows.CloseHandle(b.pipe))
		b.pipe = 0
	}
	b.closeErr = errors.Join(b.closeErr, b.child.close(), b.parent.close())
	b.prefetched = nil
	return b.closeErr
}

// OVERLAPPED/event/buffer ownership is retained until completion or drained
// cancellation. There is no abandoned IO goroutine or native handle exposure.
type executionChildIO struct {
	pipe, event windows.Handle
	ov          windows.Overlapped
	immediate   bool
	finished    bool
}

func beginExecutionChildIO(pipe windows.Handle, start func(*windows.Overlapped) error) (*executionChildIO, error) {
	event, err := windows.CreateEvent(nil, 1, 0, nil)
	if err != nil {
		return nil, err
	}
	o := &executionChildIO{pipe: pipe, event: event, ov: windows.Overlapped{HEvent: event}}
	err = start(&o.ov)
	if errors.Is(err, windows.ERROR_PIPE_CONNECTED) {
		o.immediate = true
		return o, nil
	}
	if err != nil && !errors.Is(err, windows.ERROR_IO_PENDING) {
		return nil, errors.Join(err, windows.CloseHandle(event))
	}
	return o, nil
}
func (o *executionChildIO) finish(ctx context.Context, check func() error) (n uint32, resultErr error) {
	if o == nil || o.finished {
		return 0, ErrExecutionChildRefused
	}
	defer func() { resultErr = errors.Join(resultErr, o.cancelAndClose()) }()
	if o.immediate {
		o.finished = true
		return 0, ctx.Err()
	}
	for {
		if err := ctx.Err(); err != nil {
			return 0, err
		}
		if err := check(); err != nil {
			return 0, err
		}
		status, err := windows.WaitForSingleObject(o.event, 100)
		if err != nil {
			return 0, err
		}
		if status == windows.WAIT_OBJECT_0 {
			o.finished = true
			err = windows.GetOverlappedResult(o.pipe, &o.ov, &n, false)
			return n, errors.Join(err, ctx.Err())
		}
		if status != uint32(windows.WAIT_TIMEOUT) {
			return 0, ErrExecutionChildRefused
		}
	}
}
func (o *executionChildIO) cancelAndClose() error {
	if o == nil || o.event == 0 {
		return nil
	}
	var err error
	if !o.finished && !o.immediate {
		cancelErr := windows.CancelIoEx(o.pipe, &o.ov)
		if errors.Is(cancelErr, windows.ERROR_NOT_FOUND) {
			cancelErr = nil
		}
		var n uint32
		drainErr := windows.GetOverlappedResult(o.pipe, &o.ov, &n, true)
		if errors.Is(drainErr, windows.ERROR_OPERATION_ABORTED) || errors.Is(drainErr, windows.ERROR_BROKEN_PIPE) || errors.Is(drainErr, windows.ERROR_PIPE_NOT_CONNECTED) {
			drainErr = nil
		}
		err = errors.Join(cancelErr, drainErr)
		o.finished = true
	}
	err = errors.Join(err, windows.CloseHandle(o.event))
	o.event = 0
	return err
}

// This guard is used on a locked OS thread before privileged native file/image/
// process work. Identification-level pipe impersonation is confined to token
// queries and reverted before any such work resumes.
func executionNoImpersonation() error {
	var thread windows.Token
	err := windows.OpenThreadToken(windows.CurrentThread(), windows.TOKEN_QUERY, true, &thread)
	if err == nil {
		return errors.Join(ErrExecutionChildRefused, thread.Close())
	}
	if !errors.Is(err, windows.ERROR_NO_TOKEN) {
		return errors.Join(ErrExecutionChildRefused, err)
	}
	original, err := windows.GetCurrentProcessToken().GetTokenUser()
	if err != nil {
		return err
	}
	effective, err := windows.GetCurrentThreadEffectiveToken().GetTokenUser()
	if err != nil || !effective.User.Sid.Equals(original.User.Sid) {
		return errors.Join(ErrExecutionChildRefused, err)
	}
	return nil
}
