//go:build windows

package brokerenroll

import (
	"context"
	"errors"
	"io"
	"path/filepath"
	"runtime"
	"sync"

	"golang.org/x/sys/windows"
)

type nativeClientBaseline struct {
	mu       sync.Mutex
	ctx      context.Context
	files    *nativeEnrollmentFiles
	pin      *enrollmentPin
	broker   *windows.SID // nil only for the private operator input capability
	size     int64
	closed   bool
	closeErr error
}

func (b *nativeClientBaseline) Size() int64 { return b.size }

func (b *nativeClientBaseline) validate() error {
	if b.closed || b.files == nil || b.pin == nil || b.ctx == nil {
		return errors.New("live retained baseline custody required")
	}
	if err := b.ctx.Err(); err != nil {
		return err
	}
	if b.broker != nil {
		if err := clientRequestBrokerPrincipal(b.broker); err != nil {
			return err
		}
	} else if err := enrollmentOperator(); err != nil {
		return err
	}
	if b.size < 1 || b.size > maxClientBaselineBytes || b.pin.length != b.size {
		return errors.New("bounded retained baseline size required")
	}
	for _, pin := range b.files.pins {
		if err := pin.check(); err != nil {
			return err
		}
	}
	return b.ctx.Err()
}

func (b *nativeClientBaseline) Validate() error {
	b.mu.Lock()
	defer b.mu.Unlock()
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	return b.validate()
}

func (b *nativeClientBaseline) ReadAt(p []byte, off int64) (int, error) {
	b.mu.Lock()
	defer b.mu.Unlock()
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if err := b.validate(); err != nil {
		return 0, err
	}
	n, err := readClientBaselinePinAt(b.pin, p, off)
	if checkErr := b.validate(); checkErr != nil {
		return n, errors.Join(err, checkErr)
	}
	return n, err
}

func (b *nativeClientBaseline) Close() error {
	b.mu.Lock()
	defer b.mu.Unlock()
	if !b.closed {
		b.closed = true
		b.closeErr = b.files.Close()
	}
	return b.closeErr
}

// The owning mutex serializes the synchronous file pointer. No new path or
// handle is used to read bytes; the original no-write/delete-sharing pin owns it.
func readClientBaselinePinAt(pin *enrollmentPin, p []byte, off int64) (int, error) {
	if off < 0 {
		return 0, errors.New("negative baseline read offset")
	}
	if len(p) == 0 {
		return 0, nil
	}
	if off >= pin.length {
		return 0, io.EOF
	}
	high := int32(off >> 32)
	if _, err := windows.SetFilePointer(pin.handle, int32(off), &high, windows.FILE_BEGIN); err != nil {
		return 0, err
	}
	limit := len(p)
	if int64(limit) > pin.length-off {
		limit = int(pin.length - off)
	}
	n := 0
	for n < limit {
		var done uint32
		err := windows.ReadFile(pin.handle, p[n:limit], &done, nil)
		if int(done) > limit-n {
			return n, errors.New("invalid native baseline read length")
		}
		n += int(done)
		if err != nil {
			if errors.Is(err, windows.ERROR_HANDLE_EOF) {
				return n, io.EOF
			}
			return n, err
		}
		if done == 0 {
			return n, io.EOF
		}
	}
	if n < len(p) {
		return n, io.EOF
	}
	return n, nil
}

func holdClientBaselineFile(f *nativeEnrollmentFiles, parent *enrollmentPin, component string, sd *windows.SECURITY_DESCRIPTOR) (*enrollmentPin, error) {
	pin := &enrollmentPin{physical: parent.physical + `\` + component, dos: filepath.Join(parent.dos, component), descriptor: sd, file: true}
	if err := reopenClientRequest(pin); err != nil {
		return nil, err
	}
	f.pins = append(f.pins, pin)
	var err error
	pin.info, pin.id, err = enrollmentInfo(pin.handle)
	if err != nil {
		return nil, err
	}
	pin.length = int64(uint64(pin.info.FileSizeHigh)<<32 | uint64(pin.info.FileSizeLow))
	if pin.length < 1 || pin.length > maxClientBaselineBytes || pin.info.VolumeSerialNumber != parent.info.VolumeSerialNumber {
		return nil, errors.New("bounded same-volume baseline file required")
	}
	return pin, pin.check()
}

// This private source is preexisting operator input, not a newly trusted object.
// Present protected ACLs preserve custody only; ImportApprovedBaseline must
// authenticate all contents against the independently approved original request.
func openClientBaselineInputNative(ctx context.Context, artifactDirectory string) (result ApprovedClientBaseline, resultErr error) {
	if ctx == nil || ctx.Err() != nil {
		return nil, errors.New("live operator baseline input context required")
	}
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if err := enrollmentOperator(); err != nil {
		return nil, err
	}
	f := &nativeEnrollmentFiles{}
	ready := false
	defer func() {
		if !ready {
			resultErr = errors.Join(resultErr, f.Close())
		}
	}()
	parent, err := f.holdAncestry(artifactDirectory, true)
	if err != nil {
		return nil, err
	}
	parent.allowAdds = false
	if err := parent.check(); err != nil {
		return nil, err
	}
	sd, err := windows.GetSecurityInfo(parent.handle, windows.SE_FILE_OBJECT, windows.OWNER_SECURITY_INFORMATION)
	if err != nil {
		return nil, err
	}
	owner, _, err := sd.Owner()
	if err != nil || owner == nil || owner.String() != "S-1-5-18" && owner.String() != "S-1-5-32-544" {
		return nil, errors.New("machine-protected operator baseline parent required")
	}
	pin, err := holdClientBaselineFile(f, parent, clientBaselineInputName, nil)
	if err != nil {
		return nil, err
	}
	b := &nativeClientBaseline{ctx: ctx, files: f, pin: pin, size: pin.length}
	if err := b.validate(); err != nil {
		return nil, err
	}
	ready = true
	return b, nil
}

// OpenApprovedClientBaselineNative opens only the fixed machine SID.baseline.zip
// as the actual non-impersonated broker. It retains all physical NTFS ancestors
// and the original read-only file with no write/delete sharing. It does not
// authenticate contents, adopt partial publication, accept a caller path or
// allow an administrator-console fallback. ImportApprovedBaseline must bind the
// bytes to the separately approved exact client request before any store import.
func OpenApprovedClientBaselineNative(ctx context.Context, sid string) (result ApprovedClientBaseline, resultErr error) {
	if ctx == nil || ctx.Err() != nil || !canonicalClientSID(sid) {
		return nil, errors.New("live canonical client baseline identity required")
	}
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	broker, err := enrollmentBrokerSID()
	if err != nil {
		return nil, err
	}
	if err := clientRequestBrokerPrincipal(broker); err != nil {
		return nil, err
	}
	if err := validateClientSIDNative(sid); err != nil {
		return nil, err
	}
	f := &nativeEnrollmentFiles{}
	ready := false
	defer func() {
		if !ready {
			resultErr = errors.Join(resultErr, f.Close())
		}
	}()
	programData, err := windows.KnownFolderPath(windows.FOLDERID_ProgramData, windows.KF_FLAG_DEFAULT)
	if err != nil {
		return nil, err
	}
	parent, err := f.holdAncestry(programData, true)
	if err != nil {
		return nil, err
	}
	dataSD, err := windows.SecurityDescriptorFromString("O:BAD:P(A;OICI;FA;;;SY)(A;OICI;FA;;;BA)(A;OICI;FA;;;" + broker.String() + ")")
	if err != nil {
		return nil, err
	}
	directorySD, err := clientRequestDescriptor(broker.String(), true)
	if err != nil {
		return nil, err
	}
	fileSD, err := clientRequestDescriptor(broker.String(), false)
	if err != nil {
		return nil, err
	}
	for _, child := range []struct {
		name string
		sd   *windows.SECURITY_DESCRIPTOR
	}{{"SKAGER", dataSD}, {clientRequestsDirectory, directorySD}} {
		name := parent.physical + `\` + child.name
		pin, err := f.pinDirectory(enrollmentPhysical(name), name, filepath.Join(parent.dos, child.name), child.sd, false, broker.String())
		if err != nil {
			return nil, err
		}
		if pin.info.VolumeSerialNumber != parent.info.VolumeSerialNumber {
			return nil, errors.New("client baseline ancestry crossed volume")
		}
		parent = pin
	}
	pin, err := holdClientBaselineFile(f, parent, sid+".baseline.zip", fileSD)
	if err != nil {
		return nil, err
	}
	b := &nativeClientBaseline{ctx: ctx, files: f, pin: pin, broker: broker, size: pin.length}
	if err := b.validate(); err != nil {
		return nil, err
	}
	ready = true
	return b, nil
}

type clientBaselinePinReader struct{ pin *enrollmentPin }

func (r clientBaselinePinReader) ReadAt(p []byte, off int64) (int, error) {
	if err := r.pin.check(); err != nil {
		return 0, err
	}
	n, err := readClientBaselinePinAt(r.pin, p, off)
	return n, errors.Join(err, r.pin.check())
}

func (f *nativeEnrollmentFiles) publishClientBaseline(ctx context.Context, source ApprovedClientBaseline, sid string) (resultErr error) {
	f.mu.Lock()
	defer f.mu.Unlock()
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if ctx == nil || ctx.Err() != nil || source == nil || source.Size() < 1 || source.Size() > maxClientBaselineBytes ||
		f.clientParent == nil || f.clientSID != sid || !canonicalClientSID(sid) || f.baselineAttempted {
		return errors.New("one fresh retained approved client baseline required")
	}
	if err := f.check(); err != nil {
		return err
	}
	if err := source.Validate(); err != nil {
		return err
	}
	broker, err := enrollmentBrokerSID()
	if err != nil {
		return err
	}
	fileSD, err := clientRequestDescriptor(broker.String(), false)
	if err != nil {
		return err
	}
	f.baselineAttempted = true
	component := sid + ".baseline.zip"
	file, pin, err := f.newFile(f.clientParent, component, fileSD)
	if err != nil {
		return err
	}
	writerOpen := true
	defer func() {
		if writerOpen {
			resultErr = errors.Join(resultErr, enrollmentCleanup(file.Close()))
		}
	}()
	originalSHA, err := copyClientBaseline(ctx, file, source)
	if err != nil {
		return err
	}
	pin.length = source.Size()
	if err := file.Sync(); err != nil {
		return err
	}
	readbackSHA, err := copyBaselineBytes(ctx, io.Discard, clientBaselinePinReader{pin}, pin.length)
	if err != nil || readbackSHA != originalSHA {
		return errors.Join(errors.New("durable client baseline readback differs"), err)
	}
	closeErr := enrollmentCleanup(file.Close())
	writerOpen = false
	if closeErr != nil {
		return closeErr
	}
	if err := reopenClientRequest(pin); err != nil {
		return err
	}
	pin.dos = filepath.Join(f.clientParent.dos, component)
	f.pins = append(f.pins, pin)
	readbackSHA, err = copyBaselineBytes(ctx, io.Discard, clientBaselinePinReader{pin}, pin.length)
	if err != nil || readbackSHA != originalSHA {
		return errors.Join(errors.New("held client baseline readback differs"), err)
	}
	if err := source.Validate(); err != nil {
		return err
	}
	if err := f.check(); err != nil {
		return err
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	f.clientBaselineReady = true
	return nil
}
