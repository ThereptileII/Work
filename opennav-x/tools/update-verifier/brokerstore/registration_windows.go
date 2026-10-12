package brokerstore

import (
	"context"
	"errors"
	"io"
	"os"
	"path/filepath"
	"unsafe"

	"golang.org/x/sys/windows"
)

type nativeRegistration struct {
	store                         *nativeStore
	client, channel, approved     string
	directorySD, fileSD           *windows.SECURITY_DESCRIPTOR
	pins                          []*nativePin
	user, trust, artifacts, state *nativePin
	marker, registration          *nativeFile
	markerLength                  int64
	stateDOS, artifactsDOS        string
	closed                        bool
	closeErr                      error
}

func (n *nativeStore) prepareRegistration(client, channel, approved string) (registrationBackend, error) {
	if err := n.check(); err != nil {
		return nil, err
	}
	if n.users == nil || n.registrations == nil {
		return nil, errors.New("protected fixed client store infrastructure required")
	}
	sid, err := clientSID(client, n.broker)
	if err != nil {
		return nil, err
	}
	if (channel != "beta" && channel != "stable") || !validHex(approved, 64) {
		return nil, errors.New("fixed registration identity required")
	}
	directorySD, err := descriptor(n.broker, nil, true)
	if err != nil {
		return nil, err
	}
	fileSD, err := descriptor(n.broker, nil, false)
	if err != nil {
		return nil, err
	}
	return &nativeRegistration{store: n, client: sid.String(), channel: channel, approved: approved, directorySD: directorySD, fileSD: fileSD,
		stateDOS:     filepath.Join(n.rootDOS, "users", sid.String(), "trust", channel),
		artifactsDOS: filepath.Join(n.rootDOS, "users", sid.String(), "artifacts")}, nil
}
func (r *nativeRegistration) checkTree() error {
	if r.closed {
		return ErrClosed
	}
	if err := r.store.check(); err != nil {
		return err
	}
	for _, pin := range r.pins {
		if err := pin.check(); err != nil {
			return err
		}
	}
	return nil
}
func (r *nativeRegistration) check() error {
	if err := r.checkTree(); err != nil {
		return err
	}
	if r.marker != nil {
		return r.marker.check(r.markerLength)
	}
	return nil
}
func (r *nativeRegistration) reserve(ctx context.Context) error {
	if err := r.check(); err != nil {
		return err
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	if r.marker != nil {
		return errors.New("registration reservation already consumed")
	}
	// Refuse historical registration bytes before consuming a fresh reservation.
	// Missing parent or any lookup uncertainty is never treated as absence.
	pointer, err := windows.UTF16PtrFromString(physical(r.store.registrations.name + `\` + r.client + ".json"))
	if err != nil {
		return err
	}
	if _, err = windows.GetFileAttributes(pointer); !errors.Is(err, windows.ERROR_FILE_NOT_FOUND) {
		if err != nil {
			return err
		}
		return errors.New("existing registration refuses fresh creation")
	}
	r.marker, err = createRegistrationFile(r.store.registrations, r.client+".once", r.fileSD)
	if err != nil {
		return err
	}
	started := registrationMarker(r.client, r.channel, r.approved, "", false)
	if err = writeRegistrationBytes(ctx, r.marker, 0, started, r.checkTree); err != nil {
		return err
	}
	r.markerLength = int64(len(started))
	return r.check()
}
func (r *nativeRegistration) newDirectory(parent *nativePin, component string) (*nativePin, error) {
	if r.marker == nil || r.markerLength == 0 {
		return nil, errors.New("durable started client reservation required")
	}
	if !safeComponent(component) {
		return nil, errors.New("fixed client directory component required")
	}
	if err := r.check(); err != nil {
		return nil, err
	}
	if err := parent.check(); err != nil {
		return nil, err
	}
	name := parent.name + `\` + component
	pointer, err := windows.UTF16PtrFromString(physical(name))
	if err != nil {
		return nil, err
	}
	attributes := windows.SecurityAttributes{Length: uint32(unsafe.Sizeof(windows.SecurityAttributes{})), SecurityDescriptor: r.directorySD}
	if err = windows.CreateDirectory(pointer, &attributes); err != nil {
		return nil, err
	}
	// No OPEN_EXISTING fallback: created partial nodes survive failed pinning.
	pin, err := openPin(physical(name), name, r.directorySD)
	if err != nil {
		return nil, err
	}
	r.pins = append(r.pins, pin)
	if pin.info.VolumeSerialNumber != parent.info.VolumeSerialNumber {
		return nil, errors.New("fresh client directory crossed volumes")
	}
	return pin, r.check()
}
func (r *nativeRegistration) createUser() error {
	if r.user != nil {
		return errors.New("client namespace already created")
	}
	pin, err := r.newDirectory(r.store.users, r.client)
	if err != nil {
		return err
	}
	r.user = pin
	return nil
}
func (r *nativeRegistration) createTrust() error {
	if r.user == nil || r.trust != nil {
		return errors.New("fresh client owner required for trust parent")
	}
	pin, err := r.newDirectory(r.user, "trust")
	if err != nil {
		return err
	}
	r.trust = pin
	pin.dos = filepath.Dir(r.stateDOS)
	return r.check()
}
func (r *nativeRegistration) createArtifacts() error {
	if r.user == nil || r.trust == nil || r.artifacts != nil {
		return errors.New("fresh client trust parent required for artifacts")
	}
	pin, err := r.newDirectory(r.user, "artifacts")
	if err != nil {
		return err
	}
	r.artifacts = pin
	pin.dos = r.artifactsDOS
	return r.check()
}
func (r *nativeRegistration) directories() (string, string, error) {
	if err := r.check(); err != nil {
		return "", "", err
	}
	if r.trust == nil || r.artifacts == nil {
		return "", "", errors.New("complete retained client tree required")
	}
	return r.stateDOS, r.artifactsDOS, nil
}
func (r *nativeRegistration) complete(ctx context.Context, raw []byte) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	if err := r.check(); err != nil {
		return err
	}
	if r.trust == nil || r.artifacts == nil || r.registration != nil || r.state != nil || len(raw) == 0 || len(raw) > MaxRegistrationBytes {
		return errors.New("one bounded fresh client registration required")
	}
	// InitializeState, owned by the caller, must create this exact absent channel.
	// No creation or trust reset occurs here; a missing/unsafe channel refuses.
	name := r.trust.name + `\` + r.channel
	state, err := openPin(physical(name), name, r.directorySD)
	if err != nil {
		return err
	}
	r.pins = append(r.pins, state)
	r.state = state
	state.dos = r.stateDOS
	if state.info.VolumeSerialNumber != r.trust.info.VolumeSerialNumber {
		return errors.New("initialized channel crossed volumes")
	}
	if err = r.check(); err != nil {
		return err
	}
	if err = ctx.Err(); err != nil {
		return err
	}
	r.registration, err = createRegistrationFile(r.store.registrations, r.client+".json", r.fileSD)
	if err != nil {
		return err
	}
	if err = writeRegistrationBytes(ctx, r.registration, 0, raw, r.check); err != nil {
		return err
	}
	completed := registrationMarker(r.client, r.channel, r.approved, registrationDigest(raw), true)
	if int64(len(completed)) != r.markerLength {
		return errors.New("registration completion marker width differs")
	}
	// Publish completion only after exact JSON and initialized tree validation.
	// Cancellation is checked immediately before this irrevocable final section.
	// Actual I/O/custody failures thereafter retain an uncertain durable result.
	check := func() error {
		if err := r.check(); err != nil {
			return err
		}
		return r.registration.check(int64(len(raw)))
	}
	return publishRegistrationCompletion(ctx, r.marker, r.markerLength, completed, check)
}
func (r *nativeRegistration) close() error {
	if r.closed {
		return r.closeErr
	}
	r.closed = true
	if r.registration != nil {
		r.closeErr = errors.Join(r.closeErr, r.registration.Close())
		r.registration = nil
	}
	if r.marker != nil {
		r.closeErr = errors.Join(r.closeErr, r.marker.Close())
		r.marker = nil
	}
	for i := len(r.pins) - 1; i >= 0; i-- {
		r.closeErr = errors.Join(r.closeErr, windows.CloseHandle(r.pins[i].handle))
	}
	r.pins = nil
	return r.closeErr
}

func createRegistrationFile(parent *nativePin, component string, sd *windows.SECURITY_DESCRIPTOR) (*nativeFile, error) {
	if !safeComponent(component) || sd == nil {
		return nil, errors.New("fixed protected registration component required")
	}
	if err := parent.check(); err != nil {
		return nil, err
	}
	name := parent.name + `\` + component
	pointer, err := windows.UTF16PtrFromString(physical(name))
	if err != nil {
		return nil, err
	}
	attributes := windows.SecurityAttributes{Length: uint32(unsafe.Sizeof(windows.SecurityAttributes{})), SecurityDescriptor: sd}
	handle, err := windows.CreateFile(pointer, windows.GENERIC_READ|windows.GENERIC_WRITE|windows.READ_CONTROL, 0, &attributes, windows.CREATE_NEW,
		windows.FILE_FLAG_OPEN_REPARSE_POINT|windows.FILE_FLAG_WRITE_THROUGH, 0)
	if err != nil {
		return nil, err
	}
	file := &nativeFile{File: os.NewFile(uintptr(handle), physical(name)), pin: &nativePin{handle: handle, name: name, security: sd}, parent: parent}
	if err = windows.GetFileInformationByHandle(handle, &file.pin.info); err == nil {
		err = file.check(0)
	}
	if err == nil {
		var kind uint32
		kind, err = windows.GetFileType(handle)
		if err == nil && kind != windows.FILE_TYPE_DISK {
			err = errors.New("ordinary disk registration file required")
		}
	}
	if err != nil {
		return nil, errors.Join(err, file.Close())
	}
	return file, nil
}

func readRegistrationEvidence(parent *nativePin, component string, sd *windows.SECURITY_DESCRIPTOR, bound int64) (result []byte, resultErr error) {
	if !safeComponent(component) {
		return nil, errors.New("fixed client reservation required")
	}
	if err := parent.check(); err != nil {
		return nil, err
	}
	name := parent.name + `\` + component
	pointer, err := windows.UTF16PtrFromString(physical(name))
	if err != nil {
		return nil, err
	}
	handle, err := windows.CreateFile(pointer, windows.GENERIC_READ|windows.READ_CONTROL, windows.FILE_SHARE_READ, nil, windows.OPEN_EXISTING, windows.FILE_FLAG_OPEN_REPARSE_POINT, 0)
	if err != nil {
		return nil, err
	}
	file := &nativeFile{File: os.NewFile(uintptr(handle), physical(name)), pin: &nativePin{handle: handle, name: name, security: sd}, parent: parent}
	defer func() {
		if err := file.Close(); err != nil {
			result = nil
			resultErr = errors.Join(resultErr, err)
		}
	}()
	if err = windows.GetFileInformationByHandle(handle, &file.pin.info); err != nil {
		return nil, err
	}
	length := int64(uint64(file.pin.info.FileSizeHigh)<<32 | uint64(file.pin.info.FileSizeLow))
	if length <= 0 || length > bound {
		return nil, errors.New("client reservation outside bound")
	}
	kind, err := windows.GetFileType(handle)
	if err != nil || kind != windows.FILE_TYPE_DISK {
		return nil, errors.New("ordinary disk reservation required")
	}
	if err = file.check(length); err != nil {
		return nil, err
	}
	data, err := io.ReadAll(io.LimitReader(file, bound+1))
	if err != nil || int64(len(data)) != length {
		return nil, errors.New("held client reservation read changed")
	}
	if err = file.check(length); err != nil {
		return nil, err
	}
	return data, nil
}

func readCompletedRegistration(parent *nativePin, client string, sd *windows.SECURITY_DESCRIPTOR) ([]byte, error) {
	marker, err := readRegistrationEvidence(parent, client+".once", sd, maxRegistrationMarkerBytes)
	if err != nil {
		return nil, err
	}
	digest, err := completedRegistrationDigest(marker, client)
	if err != nil {
		return nil, err
	}
	data, err := readRegistrationEvidence(parent, client+".json", sd, MaxRegistrationBytes)
	if err != nil {
		return nil, err
	}
	if registrationDigest(data) != digest {
		return nil, errors.New("registration differs from completed permanent reservation")
	}
	return data, nil
}
