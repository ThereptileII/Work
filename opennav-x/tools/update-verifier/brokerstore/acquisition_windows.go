package brokerstore

import (
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"errors"
	"io"
	"os"
	"unsafe"

	"golang.org/x/sys/windows"
)

type nativeAcquisition struct {
	candidate         *nativeCandidate
	directory         *nativePin
	identity          diskIdentity
	parentID          diskIdentity
	fileSD            *windows.SECURITY_DESCRIPTOR
	journal           *nativeFile
	index             *acquisitionIndex
	length            int64
	artifacts         []*nativeArtifact
	failure, closeErr error
	closed            bool
}
type nativeArtifact struct {
	acquisition     *nativeAcquisition
	slot            ArtifactSlot
	spec            ArtifactSpec
	identity        diskIdentity
	file            *os.File
	name            string
	indexed, closed bool
	closeErr        error
	custodyErr      error
}

func fileIdentity(h windows.Handle) (diskIdentity, error) {
	var id struct {
		Volume uint64
		ID     [16]byte
	}
	if err := windows.GetFileInformationByHandleEx(h, windows.FileIdInfo, (*byte)(unsafe.Pointer(&id)), uint32(unsafe.Sizeof(id))); err != nil {
		return diskIdentity{}, err
	}
	var info windows.ByHandleFileInformation
	if err := windows.GetFileInformationByHandle(h, &info); err != nil {
		return diskIdentity{}, err
	}
	result := diskIdentity{Volume: id.Volume, FileID: hex.EncodeToString(id.ID[:]), Created: uint64(info.CreationTime.HighDateTime)<<32 | uint64(info.CreationTime.LowDateTime)}
	if !result.valid() {
		return diskIdentity{}, errors.New("native full file identity unavailable")
	}
	return result, nil
}
func (c *nativeCandidate) beginAcquisition() (result acquisitionBackend, resultErr error) {
	if err := c.check(); err != nil {
		return nil, err
	}
	dirSD, err := descriptor(c.store.broker, nil, true)
	if err != nil {
		return nil, err
	}
	fileSD, err := descriptor(c.store.broker, nil, false)
	if err != nil {
		return nil, err
	}
	parent := c.directories[""]
	name := parent.name + `\` + acquisitionDirectory
	pointer, err := windows.UTF16PtrFromString(physical(name))
	if err != nil {
		return nil, err
	}
	attributes := windows.SecurityAttributes{Length: uint32(unsafe.Sizeof(windows.SecurityAttributes{})), SecurityDescriptor: dirSD}
	if err = windows.CreateDirectory(pointer, &attributes); err != nil {
		return nil, err
	}
	dir, err := openPin(physical(name), name, dirSD)
	if err != nil {
		return nil, err
	}
	a := &nativeAcquisition{candidate: c, directory: dir, fileSD: fileSD, index: newAcquisitionIndex()}
	if dir.info.VolumeSerialNumber != parent.info.VolumeSerialNumber {
		return nil, errors.Join(errors.New("acquisition directory crossed volume"), windows.CloseHandle(dir.handle))
	}
	ok := false
	defer func() {
		if !ok {
			resultErr = errors.Join(resultErr, a.closeHandles())
		}
	}()
	a.parentID, err = fileIdentity(parent.handle)
	if err != nil {
		return nil, err
	}
	a.identity, err = fileIdentity(dir.handle)
	if err != nil {
		return nil, err
	}
	a.journal, err = openAcquisitionFile(dir, acquisitionJournal, fileSD, windows.GENERIC_READ|windows.GENERIC_WRITE|windows.READ_CONTROL, windows.FILE_SHARE_READ, windows.CREATE_NEW)
	if err != nil {
		return nil, err
	}
	if err = a.append("lease", "", nil, nil); err != nil {
		return nil, err
	}
	ok = true
	return a, nil
}
func openAcquisitionFile(parent *nativePin, component string, sd *windows.SECURITY_DESCRIPTOR, access, share, creation uint32) (*nativeFile, error) {
	if !safeComponent(component) {
		return nil, errors.New("fixed acquisition component required")
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
	var attr *windows.SecurityAttributes
	if creation == windows.CREATE_NEW {
		attr = &attributes
	}
	h, err := windows.CreateFile(pointer, access, share, attr, creation, windows.FILE_FLAG_OPEN_REPARSE_POINT, 0)
	if err != nil {
		return nil, err
	}
	f := &nativeFile{File: os.NewFile(uintptr(h), physical(name)), parent: parent, pin: &nativePin{handle: h, name: name, security: sd}}
	if err = windows.GetFileInformationByHandle(h, &f.pin.info); err == nil {
		size := int64(uint64(f.pin.info.FileSizeHigh)<<32 | uint64(f.pin.info.FileSizeLow))
		err = f.check(size)
		var kind uint32
		if err == nil {
			kind, err = windows.GetFileType(h)
			if err == nil && kind != windows.FILE_TYPE_DISK {
				err = errors.New("acquisition file must be ordinary disk data")
			}
		}
	}
	if err != nil {
		return nil, errors.Join(err, f.Close())
	}
	return f, nil
}
func (a *nativeAcquisition) check() error {
	if a.closed {
		return ErrClosed
	}
	if a.failure != nil {
		return a.failure
	}
	if err := a.candidate.check(); err != nil {
		return err
	}
	if err := a.directory.check(); err != nil {
		return err
	}
	parentID, err := fileIdentity(a.candidate.directories[""].handle)
	if err != nil || parentID != a.parentID {
		return errors.New("acquisition parent identity changed")
	}
	id, err := fileIdentity(a.directory.handle)
	if err != nil || id != a.identity {
		return errors.New("acquisition directory identity changed")
	}
	return a.journal.check(a.length)
}
func (a *nativeAcquisition) append(state string, slot ArtifactSlot, id *diskIdentity, plan *AcquisitionPlan) error {
	if a.failure != nil {
		return a.failure
	}
	e := acquisitionEvent{Schema: 1, Sequence: len(a.index.events) + 1, State: state, StageID: a.candidate.stageID, ClientSID: a.candidate.client, Directory: a.identity, Parent: a.parentID, Slot: slot, Identity: id, Plan: plan}
	// Validate a separate state before any write, publishing it only after flush.
	preview := newAcquisitionIndex()
	for _, old := range a.index.events {
		if err := preview.apply(old); err != nil {
			return err
		}
	}
	if err := preview.apply(e); err != nil {
		return err
	}
	line, err := json.Marshal(e)
	if err != nil {
		return err
	}
	line = append(line, '\n')
	if a.length+int64(len(line)) > maxAcquisitionJournal {
		return errors.New("acquisition journal bound exhausted")
	}
	if err = a.journal.check(a.length); err != nil {
		a.failure = err
		return err
	}
	n, err := a.journal.Write(line)
	if err == nil && n != len(line) {
		err = io.ErrShortWrite
	}
	if err == nil {
		err = a.journal.Sync()
	}
	if err != nil {
		a.failure = err
		return err
	}
	a.length += int64(n)
	a.index = preview
	return nil
}
func (a *nativeAcquisition) bind(p AcquisitionPlan) error {
	if err := a.check(); err != nil {
		return err
	}
	return a.append("bound", "", nil, &p)
}
func (a *nativeAcquisition) create(slot ArtifactSlot, spec ArtifactSpec) (artifactBackend, error) {
	if err := a.check(); err != nil {
		return nil, err
	}
	if err := a.append("creating", slot, nil, nil); err != nil {
		return nil, err
	}
	f, err := openAcquisitionFile(a.directory, slotFilename(slot), a.fileSD, windows.GENERIC_READ|windows.GENERIC_WRITE|windows.DELETE|windows.READ_CONTROL, windows.FILE_SHARE_READ|windows.FILE_SHARE_DELETE, windows.CREATE_NEW)
	if err != nil {
		return nil, err
	}
	item := &nativeArtifact{acquisition: a, slot: slot, spec: spec, file: f.File, name: f.pin.name}
	a.artifacts = append(a.artifacts, item)
	item.identity, err = fileIdentity(f.pin.handle)
	if err != nil {
		return nil, err
	}
	if err = a.append("owned", slot, &item.identity, nil); err != nil {
		return nil, err
	}
	item.indexed = true
	return item, nil
}
func (f *nativeArtifact) Path() string { return physical(f.name) }
func (f *nativeArtifact) check() error {
	if f.file == nil {
		return ErrClosed
	}
	if err := f.acquisition.check(); err != nil {
		return err
	}
	h := windows.Handle(f.file.Fd())
	var info windows.ByHandleFileInformation
	if err := windows.GetFileInformationByHandle(h, &info); err != nil {
		return err
	}
	id, err := fileIdentity(h)
	if err != nil || id != f.identity {
		return errors.New("acquisition artifact full identity changed")
	}
	name, err := nativeName(h)
	if err != nil || name != f.name {
		return errors.New("acquisition artifact physical name changed")
	}
	size := int64(uint64(info.FileSizeHigh)<<32 | uint64(info.FileSizeLow))
	kind, err := windows.GetFileType(h)
	if err != nil || kind != windows.FILE_TYPE_DISK || info.FileAttributes&(windows.FILE_ATTRIBUTE_DIRECTORY|windows.FILE_ATTRIBUTE_REPARSE_POINT) != 0 || info.NumberOfLinks != 1 || size < 0 || size > f.spec.Size || id.Volume != f.acquisition.identity.Volume {
		return errors.New("unsafe artifact type/link/size/volume")
	}
	return checkSecurity(h, f.acquisition.fileSD)
}
func (f *nativeArtifact) Write(p []byte) (int, error) {
	if err := f.check(); err != nil {
		return 0, err
	}
	n, err := f.file.Write(p)
	if err == nil && n != len(p) {
		err = io.ErrShortWrite
	}
	if err != nil {
		return n, err
	}
	return n, f.check()
}
func (f *nativeArtifact) Sync() error {
	if err := f.check(); err != nil {
		return err
	}
	if err := f.file.Sync(); err != nil {
		return err
	}
	return f.check()
}
func (f *nativeArtifact) verify(spec ArtifactSpec) error {
	if err := f.check(); err != nil {
		return err
	}
	hash := sha256.New()
	n, err := io.Copy(hash, io.NewSectionReader(f.file, 0, spec.Size+1))
	if err != nil {
		return err
	}
	if n != spec.Size || hex.EncodeToString(hash.Sum(nil)) != spec.SHA256 {
		return errors.New("actual held acquisition bytes differ from bound plan")
	}
	return f.check()
}
func (f *nativeArtifact) seal(spec ArtifactSpec) (result *os.File, resultErr error) {
	if err := f.Sync(); err != nil {
		return nil, err
	}
	if err := f.verify(spec); err != nil {
		return nil, err
	}
	a := f.acquisition
	// A DELETE-only bridge remains held across writer close/reopen. Reduced-access
	// duplicates are not assumed to remove a FileObject's write-sharing requirement.
	// The final handle has READ+DELETE, no WRITE, and denies new data writers.
	bridge, err := openAcquisitionFile(a.directory, slotFilename(f.slot), a.fileSD, windows.DELETE|windows.READ_CONTROL|windows.FILE_READ_ATTRIBUTES, windows.FILE_SHARE_READ|windows.FILE_SHARE_WRITE|windows.FILE_SHARE_DELETE, windows.OPEN_EXISTING)
	if err != nil {
		return nil, err
	}
	defer func() {
		if bridge.File != nil {
			if err := bridge.Close(); err != nil {
				f.custodyErr = errors.Join(f.custodyErr, err)
				resultErr = errors.Join(resultErr, err)
			}
		}
		if resultErr != nil {
			result = nil
		}
	}()
	bridgeID, err := fileIdentity(bridge.pin.handle)
	if err != nil || bridgeID != f.identity {
		return nil, errors.New("delete bridge identity mismatch")
	}
	writer := f.file
	f.file = nil
	if err = writer.Close(); err != nil {
		f.custodyErr = errors.Join(f.custodyErr, err)
		f.file = bridge.File
		bridge.File = nil
		return nil, err
	}
	reader, err := openAcquisitionFile(a.directory, slotFilename(f.slot), a.fileSD, windows.GENERIC_READ|windows.DELETE|windows.READ_CONTROL, windows.FILE_SHARE_READ|windows.FILE_SHARE_DELETE, windows.OPEN_EXISTING)
	if err != nil {
		// Keep the actual bridge available for handle-based cleanup on refusal.
		f.file = bridge.File
		bridge.File = nil
		return nil, err
	}
	f.file = reader.File
	id, err := fileIdentity(reader.pin.handle)
	if err != nil || id != f.identity {
		return nil, errors.New("readonly artifact identity mismatch")
	}
	if err = f.verify(spec); err != nil {
		return nil, err
	}
	if _, err = f.file.Seek(0, io.SeekStart); err != nil {
		return nil, err
	}
	return f.file, nil
}
func (f *nativeArtifact) close() error {
	if f.closed {
		return f.closeErr
	}
	f.closed = true
	a := f.acquisition
	if f.file == nil {
		f.closeErr = errors.Join(f.custodyErr, errors.New("artifact cleanup handle unavailable"))
		return f.closeErr
	}
	if !f.indexed {
		f.closeErr = errors.New("unindexed artifact retained; recovery refuses uncertain creation")
	} else if err := f.check(); err != nil {
		f.closeErr = err
	} else if err = f.deleteIntent(); err != nil {
		f.closeErr = err
	} else {
		// FILE_DISPOSITION_INFO.DeleteFile is a native BOOLEAN. Deletion acts on
		// this retained verified file object, never on a pathname after closing it.
		disposition := byte(1)
		f.closeErr = windows.SetFileInformationByHandle(windows.Handle(f.file.Fd()), windows.FileDispositionInfo, &disposition, 1)
	}
	f.closeErr = errors.Join(f.closeErr, f.file.Close())
	f.file = nil
	if f.closeErr == nil {
		if err := artifactAbsent(a.directory, slotFilename(f.slot)); err != nil {
			f.closeErr = err
		} else {
			f.closeErr = a.append("deleted", f.slot, &f.identity, nil)
		}
	}
	f.closeErr = errors.Join(f.custodyErr, f.closeErr)
	return f.closeErr
}
func (f *nativeArtifact) deleteIntent() error {
	state := f.acquisition.index.states[f.slot]
	if state == "deleting" {
		return nil
	}
	if state != "owned" {
		return errors.New("artifact deletion lacks durable indexed ownership")
	}
	return f.acquisition.append("deleting", f.slot, &f.identity, nil)
}
func artifactAbsent(parent *nativePin, component string) error {
	if err := parent.check(); err != nil {
		return err
	}
	p, err := windows.UTF16PtrFromString(physical(parent.name + `\` + component))
	if err != nil {
		return err
	}
	h, err := windows.CreateFile(p, windows.FILE_READ_ATTRIBUTES, windows.FILE_SHARE_READ|windows.FILE_SHARE_WRITE|windows.FILE_SHARE_DELETE, nil, windows.OPEN_EXISTING, windows.FILE_FLAG_OPEN_REPARSE_POINT|windows.FILE_FLAG_BACKUP_SEMANTICS, 0)
	if errors.Is(err, windows.ERROR_FILE_NOT_FOUND) {
		return nil
	}
	if err == nil {
		return errors.Join(errors.New("artifact still exists after handle deletion"), windows.CloseHandle(h))
	}
	return errors.New("artifact absence uncertain")
}
func (a *nativeAcquisition) closeHandles() error {
	var err error
	if a.journal != nil {
		err = errors.Join(err, a.journal.Close())
		a.journal = nil
	}
	if a.directory != nil {
		err = errors.Join(err, windows.CloseHandle(a.directory.handle))
		a.directory = nil
	}
	return err
}
func (a *nativeAcquisition) close() error {
	if a.closed {
		return a.closeErr
	}
	for _, f := range a.artifacts {
		a.closeErr = errors.Join(a.closeErr, f.close())
	}
	a.closeErr = errors.Join(a.closeErr, a.failure)
	if a.closeErr == nil {
		if err := a.check(); err != nil {
			a.closeErr = err
		} else {
			entries, err := boundedEntries(a.directory, 1)
			if err != nil || len(entries) != 1 || entries[0].Name() != acquisitionJournal {
				a.closeErr = errors.New("unindexed acquisition residue prevents completion")
			} else if !a.index.complete {
				a.closeErr = a.append("complete", "", nil, nil)
			}
		}
	}
	a.closed = true
	a.closeErr = errors.Join(a.closeErr, a.closeHandles())
	return a.closeErr
}
