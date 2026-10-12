package brokerstore

import (
	"bytes"
	"errors"
	"io"
	"os"
	"path/filepath"
	"runtime"
	"strings"
	"unsafe"

	"golang.org/x/sys/windows"
)

const (
	initializationFenceFilename = "UpdateBroker.initialized"
	initializationStarted       = "SKAGER-UpdateBroker-initialization-v1:0\n"
	initializationCompleted     = "SKAGER-UpdateBroker-initialization-v1:1\n"
)

// InitializeFresh creates only the fixed actual KnownFolder ProgramData/SKAGER/
// UpdateBroker root, candidates and registrations. It requires the actual,
// non-impersonated SkagerUpdateBroker process token; Admin/SYSTEM are not substitutes.
//
// Caller preconditions: SKAGER must ALREADY have been independently provisioned
// with its protective security descriptor at ORIGINAL creation, before exposure
// to untrusted principals. Present owner/DACL checks cannot establish this history
// or exclude old writable handles. This function does not authenticate that
// provisioning history, create/repair/adopt SKAGER, or provision a service.
//
// A permanent CREATE_NEW parent fence is created with its final broker/SYSTEM
// descriptor and synced in the started state before any broker directory creation.
// Completion is written/synced only after the exact full tree passes validation.
// Existing fence or root refuses initialization. Every failure retains the fence and any partial
// namespace; there is no automatic retry, deletion, reset or trust repair.
// Physical ancestry is held and fixed DOS paths are corroborated at boundaries.
// As with Open, callers must use the broker's own DOS namespace; this is not a
// continuously pinned DOS-device mapping. Successful creation grants no launch,
// registration, network or trust-reset authority. Normal callers still use Open.
func InitializeFresh() error {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	creation, err := prepareFreshCreation(true)
	if err != nil {
		return err
	}
	return initializeFresh(creation)
}

// ValidateInitialization reads local creation-completion evidence only. It
// requires the same actual broker token and independently provisioned parent
// preconditions as InitializeFresh; it does not authenticate provisioning history
// or establish startup health. The exact completed fence and exact existing
// root/candidates/registrations must all be held and validated. Nothing is created,
// adopted, repaired or reset; a partial initialization requires operator review.
func ValidateInitialization() (resultErr error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	creation, err := prepareFreshCreation(false)
	if err != nil {
		return err
	}
	defer func() { resultErr = errors.Join(resultErr, creation.close()) }()
	return creation.validateCompleted()
}

func (c *nativeFreshCreation) validateCompleted() error {
	if err := c.readCompletedFence(); err != nil {
		return err
	}
	root, err := c.existingDirectory(c.parent, "UpdateBroker")
	if err != nil {
		return err
	}
	c.root = root
	root.dos = filepath.Join(c.parent.dos, "UpdateBroker")
	for _, name := range []string{"candidates", "registrations"} {
		if _, err := c.existingDirectory(root, name); err != nil {
			return err
		}
	}
	return c.check()
}

func prepareFreshCreation(creationRights bool) (result *nativeFreshCreation, resultErr error) {
	broker, _, _, err := windows.LookupSID("", BrokerAccount)
	if err != nil || !strings.HasPrefix(broker.String(), "S-1-5-80-") {
		return nil, ErrPrincipal
	}
	if err = currentBroker(broker); err != nil {
		return nil, err
	}
	programData, err := windows.KnownFolderPath(windows.FOLDERID_ProgramData, windows.KF_FLAG_DEFAULT)
	if err != nil {
		return nil, err
	}
	parentDOS := filepath.Join(programData, "SKAGER")
	if filepath.Clean(parentDOS) != parentDOS || len(parentDOS) < 4 ||
		!((parentDOS[0] >= 'A' && parentDOS[0] <= 'Z') || (parentDOS[0] >= 'a' && parentDOS[0] <= 'z')) ||
		parentDOS[1] != ':' || parentDOS[2] != '\\' || strings.Contains(parentDOS[3:], ":") {
		return nil, errors.New("fixed actual ProgramData parent must be a local DOS path")
	}
	installer, _, _, err := windows.LookupSID("", `NT SERVICE\TrustedInstaller`)
	if err != nil {
		return nil, errors.New("trusted machine ancestor owner unavailable")
	}
	trusted := map[string]bool{"S-1-5-18": true, "S-1-5-32-544": true, installer.String(): true, broker.String(): true}
	creation := &nativeFreshCreation{broker: broker}
	// A preparation failure owns and drains the same handles as creation; neither
	// path deletes any object or suppresses an uncertain close result.
	ready := false
	defer func() {
		if !ready {
			resultErr = errors.Join(resultErr, creation.close())
		}
	}()
	creation.directorySD, err = descriptor(broker, nil, true)
	if err != nil {
		return nil, err
	}
	creation.fileSD, err = descriptor(broker, nil, false)
	if err != nil {
		return nil, err
	}
	if err = creation.holdParent(parentDOS, trusted, creationRights); err != nil {
		return nil, err
	}
	ready = true
	return creation, nil
}

type nativeFreshCreation struct {
	broker              *windows.SID
	directorySD, fileSD *windows.SECURITY_DESCRIPTOR
	pins                []*nativePin
	parent, root        *nativePin
	fence               *nativeFile
	fenceLength         int64
}

func (c *nativeFreshCreation) holdParent(parentDOS string, trusted map[string]bool, creationRights bool) error {
	ancestor := &ancestorPolicy{trusted: trusted}
	drive, err := openPin(parentDOS[:3], "", nil)
	if err != nil {
		return err
	}
	c.pins = append(c.pins, drive)
	drive.ancestor, drive.dos = ancestor, parentDOS[:3]
	if err = drive.check(); err != nil {
		return err
	}
	parts, err := physicalParts(drive.name)
	if err != nil {
		return err
	}
	walk := strings.Join(parts[:3], `\`)
	volume, err := openPin(physical(walk), walk, nil)
	if err != nil {
		return err
	}
	c.pins = append(c.pins, volume)
	volume.ancestor = ancestor
	if err = volume.check(); err != nil {
		return err
	}
	var serial, component, flags uint32
	fs := make([]uint16, 32)
	if err = windows.GetVolumeInformationByHandle(volume.handle, nil, 0, &serial, &component, &flags, &fs[0], uint32(len(fs))); err != nil || windows.UTF16ToString(fs) != "NTFS" || serial != volume.info.VolumeSerialNumber {
		return errors.New("broker parent must be native NTFS")
	}
	all := append(append([]string{}, parts[3:]...), strings.Split(parentDOS[3:], `\`)...)
	for i, part := range all {
		if part == "" || part == "." || part == ".." || strings.HasSuffix(part, ".") || strings.HasSuffix(part, " ") {
			return errors.New("ambiguous broker parent ancestry")
		}
		walk += `\` + part
		pin, err := openPin(physical(walk), walk, nil)
		if err != nil {
			return err
		}
		c.pins = append(c.pins, pin)
		pin.ancestor = ancestor
		if i == len(all)-1 {
			pin.ancestor = &ancestorPolicy{trusted: trusted, forbidChildAdds: true}
			pin.dos = parentDOS
			c.parent = pin
		}
		if pin.info.VolumeSerialNumber != serial {
			return errors.New("broker parent ancestry crossed volumes")
		}
		if err = pin.check(); err != nil {
			return err
		}
	}
	if !creationRights {
		return c.check()
	}
	// Check actual add-file and add-directory rights before consuming the fence.
	// FILE_WRITE_DATA/APPEND_DATA are directory FILE_ADD_FILE/SUBDIRECTORY.
	// This opens only the already held physical parent; it never creates a node.
	pointer, err := windows.UTF16PtrFromString(physical(c.parent.name))
	if err != nil {
		return err
	}
	handle, err := windows.CreateFile(pointer, windows.FILE_LIST_DIRECTORY|windows.FILE_WRITE_DATA|windows.FILE_APPEND_DATA|windows.FILE_READ_ATTRIBUTES|windows.READ_CONTROL, windows.FILE_SHARE_READ|windows.FILE_SHARE_WRITE, nil, windows.OPEN_EXISTING, windows.FILE_FLAG_BACKUP_SEMANTICS|windows.FILE_FLAG_OPEN_REPARSE_POINT, 0)
	if err != nil {
		return err
	}
	capability := &nativePin{handle: handle, name: c.parent.name, ancestor: c.parent.ancestor, dos: parentDOS}
	c.pins = append(c.pins, capability)
	if err = windows.GetFileInformationByHandle(handle, &capability.info); err != nil {
		return err
	}
	if !sameIdentity(capability.info, c.parent.info) {
		return errors.New("creation-capable parent identity differs")
	}
	return c.check()
}

func (c *nativeFreshCreation) check() error {
	if c.broker != nil {
		if err := currentBroker(c.broker); err != nil {
			return err
		}
	}
	if c.parent == nil {
		return errors.New("held independently provisioned parent required")
	}
	for _, pin := range c.pins {
		if err := pin.check(); err != nil {
			return err
		}
	}
	if c.fence != nil {
		return c.fence.check(c.fenceLength)
	}
	return nil
}

func (c *nativeFreshCreation) rootAbsent() error {
	pointer, err := windows.UTF16PtrFromString(physical(c.parent.name + `\UpdateBroker`))
	if err != nil {
		return err
	}
	_, err = windows.GetFileAttributes(pointer)
	if errors.Is(err, windows.ERROR_FILE_NOT_FOUND) {
		return nil
	}
	if err != nil {
		return err // Missing parent or any uncertainty is not a fresh namespace.
	}
	return errors.New("existing broker root refuses fresh initialization")
}

func (c *nativeFreshCreation) reserve() error {
	if err := c.check(); err != nil {
		return err
	}
	name := c.parent.name + `\` + initializationFenceFilename
	pointer, err := windows.UTF16PtrFromString(physical(name))
	if err != nil {
		return err
	}
	attributes := windows.SecurityAttributes{Length: uint32(unsafe.Sizeof(windows.SecurityAttributes{})), SecurityDescriptor: c.fileSD}
	handle, err := windows.CreateFile(pointer, windows.GENERIC_READ|windows.GENERIC_WRITE|windows.READ_CONTROL, 0, &attributes, windows.CREATE_NEW, windows.FILE_FLAG_OPEN_REPARSE_POINT|windows.FILE_FLAG_WRITE_THROUGH, 0)
	if err != nil {
		return err // Never open, truncate, repair or remove an existing fence.
	}
	c.fence = &nativeFile{File: os.NewFile(uintptr(handle), physical(name)), pin: &nativePin{handle: handle, name: name, security: c.fileSD}, parent: c.parent}
	if err = windows.GetFileInformationByHandle(handle, &c.fence.pin.info); err != nil {
		return err
	}
	if err = c.check(); err != nil {
		return err
	}
	written, err := c.fence.WriteString(initializationStarted)
	if err != nil {
		return err
	}
	if written != len(initializationStarted) {
		return io.ErrShortWrite
	}
	c.fenceLength = int64(len(initializationStarted))
	if err = c.fence.Sync(); err != nil {
		return err
	}
	return c.check()
}

func (c *nativeFreshCreation) newDirectory(parent *nativePin, component string) (*nativePin, error) {
	if c.fence == nil || c.fenceLength != int64(len(initializationStarted)) {
		return nil, errors.New("durable started initialization fence required")
	}
	if err := c.check(); err != nil {
		return nil, err
	}
	name := parent.name + `\` + component
	pointer, err := windows.UTF16PtrFromString(physical(name))
	if err != nil {
		return nil, err
	}
	attributes := windows.SecurityAttributes{Length: uint32(unsafe.Sizeof(windows.SecurityAttributes{})), SecurityDescriptor: c.directorySD}
	if err = windows.CreateDirectory(pointer, &attributes); err != nil {
		return nil, err // CREATE_NEW directory semantics; no adoption or repair.
	}
	pin, err := openPin(physical(name), name, c.directorySD)
	if err != nil {
		return nil, err // Created directory remains, even if immediate pinning fails.
	}
	c.pins = append(c.pins, pin)
	if pin.info.VolumeSerialNumber != parent.info.VolumeSerialNumber {
		return nil, errors.New("fresh protected directory crossed volumes")
	}
	return pin, c.check()
}
func (c *nativeFreshCreation) createRoot() error {
	root, err := c.newDirectory(c.parent, "UpdateBroker")
	if err != nil {
		return err
	}
	c.root = root
	c.root.dos = filepath.Join(c.parent.dos, "UpdateBroker")
	return c.check()
}
func (c *nativeFreshCreation) createCandidates() error {
	_, err := c.newDirectory(c.root, "candidates")
	return err
}
func (c *nativeFreshCreation) createRegistrations() error {
	_, err := c.newDirectory(c.root, "registrations")
	return err
}
func (c *nativeFreshCreation) complete() error {
	if err := c.check(); err != nil {
		return err
	}
	if c.fence == nil || c.root == nil {
		return errors.New("held initialization fence and fresh root required")
	}
	if _, err := c.fence.Seek(0, io.SeekStart); err != nil {
		return err
	}
	written, err := c.fence.WriteString(initializationCompleted)
	if err != nil {
		return err
	}
	if written != len(initializationCompleted) {
		return io.ErrShortWrite
	}
	if err = c.fence.Sync(); err != nil {
		return err
	}
	return c.checkFenceContents(initializationCompleted)
}

func (c *nativeFreshCreation) checkFenceContents(expected string) error {
	if err := c.check(); err != nil {
		return err
	}
	if _, err := c.fence.Seek(0, io.SeekStart); err != nil {
		return err
	}
	data, err := io.ReadAll(io.LimitReader(c.fence, int64(len(expected))+1))
	if err != nil {
		return err
	}
	if !bytes.Equal(data, []byte(expected)) {
		return errors.New("broker initialization is not completed")
	}
	return c.check()
}

func (c *nativeFreshCreation) readCompletedFence() error {
	if err := c.check(); err != nil {
		return err
	}
	name := c.parent.name + `\` + initializationFenceFilename
	pointer, err := windows.UTF16PtrFromString(physical(name))
	if err != nil {
		return err
	}
	handle, err := windows.CreateFile(pointer, windows.GENERIC_READ|windows.READ_CONTROL, windows.FILE_SHARE_READ, nil, windows.OPEN_EXISTING, windows.FILE_FLAG_OPEN_REPARSE_POINT, 0)
	if err != nil {
		return err
	}
	c.fence = &nativeFile{File: os.NewFile(uintptr(handle), physical(name)), pin: &nativePin{handle: handle, name: name, security: c.fileSD}, parent: c.parent}
	c.fenceLength = int64(len(initializationCompleted))
	if err = windows.GetFileInformationByHandle(handle, &c.fence.pin.info); err != nil {
		return err
	}
	return c.checkFenceContents(initializationCompleted)
}

func (c *nativeFreshCreation) existingDirectory(parent *nativePin, component string) (*nativePin, error) {
	if err := c.check(); err != nil {
		return nil, err
	}
	name := parent.name + `\` + component
	pin, err := openPin(physical(name), name, c.directorySD)
	if err != nil {
		return nil, err
	}
	c.pins = append(c.pins, pin)
	if pin.info.VolumeSerialNumber != parent.info.VolumeSerialNumber {
		return nil, errors.New("completed directory crossed volumes")
	}
	return pin, c.check()
}

func (c *nativeFreshCreation) close() error {
	var result error
	if c.fence != nil {
		result = errors.Join(result, c.fence.Close())
		c.fence = nil
	}
	for i := len(c.pins) - 1; i >= 0; i-- {
		result = errors.Join(result, windows.CloseHandle(c.pins[i].handle))
	}
	c.pins = nil
	return result
}
