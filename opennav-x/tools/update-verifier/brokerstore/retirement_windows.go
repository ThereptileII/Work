//go:build windows

package brokerstore

import (
	"context"
	"encoding/json"
	"errors"
	"unsafe"

	"golang.org/x/sys/windows"
)

// Recovery names are fixed siblings on the original volumes. The generation
// tree is moved intact, including any unknown additions; no recursive deletion
// or profile/stock traversal occurs. A terminal marker prevents startup while
// any incomplete retirement requires explicit operator recovery.
func retirementGenerationName(binding DecommissionBinding) string {
	return "retired-" + binding.RequestID + "-" + binding.GenerationID
}
func retirementRegistrationName(binding DecommissionBinding, suffix string) string {
	return "retired-" + binding.RequestID + "-registration." + suffix
}
func retirementStartedName(binding DecommissionBinding) string {
	return "retire-" + binding.RequestID + ".once"
}
func retirementCompletedName(binding DecommissionBinding) string {
	return "retire-" + binding.RequestID + ".complete"
}

type retirementStartRecord struct {
	Schema              int          `json:"schema"`
	RequestID           string       `json:"requestId"`
	ClientSID           string       `json:"clientSid"`
	Generation          string       `json:"generation"`
	RegistrationSHA256  string       `json:"registrationSha256"`
	SelectedHeadSHA256  string       `json:"selectedHeadSha256"`
	RegistrationOnce    diskIdentity `json:"registrationOnce"`
	RegistrationJSON    diskIdentity `json:"registrationJson"`
	GenerationDirectory diskIdentity `json:"generationDirectory"`
}
type retirementCompleteRecord struct {
	Schema              int          `json:"schema"`
	RequestID           string       `json:"requestId"`
	StartedSHA256       string       `json:"startedSha256"`
	FenceSHA256         string       `json:"fenceSha256"`
	RegistrationOnce    diskIdentity `json:"registrationOnce"`
	RegistrationJSON    diskIdentity `json:"registrationJson"`
	GenerationDirectory diskIdentity `json:"generationDirectory"`
}

type nativeRetirementPreparation struct {
	store                                            *nativeStore
	binding                                          DecommissionBinding
	original                                         *nativeDecommission
	generationParent                                 *nativePin
	generationSecurity                               *windows.SECURITY_DESCRIPTOR
	registrationOnce, registrationJSON, generationID diskIdentity
	closed, transferred                              bool
}
type retirementObject struct {
	handle    windows.Handle
	parent    *nativePin
	name      string
	id        diskIdentity
	directory bool
	security  *windows.SECURITY_DESCRIPTOR
}
type nativeRetirement struct {
	store                               *nativeStore
	binding                             DecommissionBinding
	generationParent                    *nativePin
	objects                             [3]*retirementObject
	marker, started, completed          *nativeFile
	markerRaw, startedRaw, completedRaw []byte
	terminal, completedMove, closed     bool
	closeErr                            error
}

func (n *nativeStore) prepareRetirement(ctx context.Context, binding DecommissionBinding) (retirementPreparationBackend, error) {
	return n.prepareRetirementWithOpeners(ctx, binding, decommissionInventoryOpeners{n.openReadOnlyGeneration, n.openStageSeal})
}

// Inert native fixtures replace only the fixed Program Files opener; every
// actual original registration/inventory/identity rule remains shared.
func (n *nativeStore) prepareRetirementWithOpeners(ctx context.Context, binding DecommissionBinding, openers decommissionInventoryOpeners) (retirementPreparationBackend, error) {
	if ctx == nil || ctx.Err() != nil || !binding.valid() || !validHex(binding.GenerationID, 32) {
		return nil, errors.New("original selected retirement binding required")
	}
	if err := n.check(); err != nil {
		return nil, err
	}
	d, err := openDecommissionInventoryWithOpeners(ctx, n, binding, openers)
	if err != nil {
		return nil, err
	}
	p := &nativeRetirementPreparation{store: n, binding: binding, original: d}
	ok := false
	defer func() {
		if !ok {
			_ = p.close()
		}
	}()
	if !d.rootInventory.deployed || d.opened == nil || d.opened[binding.GenerationID] == nil {
		return nil, errors.New("actual completed selected generation required")
	}
	// This first retirement owns the original baseline deployment only. A
	// signed successor or pending stage needs an explicit lineage-wide recovery
	// policy before any selected directory can move out of its registry.
	if binding.GenerationID != d.baseline || len(d.opened) != 1 || len(d.rootInventory.stages) != 0 {
		return nil, errors.New("retirement of updated or pending generation ancestry is unsupported")
	}
	g := d.opened[binding.GenerationID]
	if len(g.siblings) != 0 {
		return nil, errors.New("retirement of generation history is unsupported")
	}
	root := g.tree.directories[""]
	if root == nil || g.generations == nil || g.tree.securityDir == nil || root.name != g.generations.name+`\`+binding.GenerationID {
		return nil, errors.New("original selected generation directory unavailable")
	}
	// The original generation reader has already constructed and checked this
	// descriptor against the actual selected directory. Retain that exact policy
	// through the read-to-DELETE handoff instead of resolving the client again.
	p.generationSecurity = g.tree.securityDir
	for i, file := range d.evidence {
		id, err := fileIdentity(file.pin.handle)
		if err != nil {
			return nil, err
		}
		if i == 0 {
			p.registrationOnce = id
		} else {
			p.registrationJSON = id
		}
	}
	p.generationID, err = fileIdentity(root.handle)
	if err != nil {
		return nil, err
	}
	// Acquire a second parent pin while the original read owner still proves the
	// complete inventory. This pin outlives that owner's controlled close and
	// keeps the original physical parent/ACL identity through DELETE acquisition.
	p.generationParent, err = openPin(physical(g.generations.name), g.generations.name, g.tree.securityDir)
	if err != nil {
		return nil, err
	}
	parentID, err := fileIdentity(p.generationParent.handle)
	if err != nil || parentID != g.pins[g.generations] {
		return nil, errors.Join(errors.New("retirement parent changed before handoff"), err)
	}
	if err := d.custody(); err != nil {
		return nil, err
	}
	ok = true
	return p, nil
}

func (p *nativeRetirementPreparation) check() error {
	if p == nil || p.closed || p.transferred || p.original == nil || p.generationParent == nil {
		return ErrClosed
	}
	if err := p.generationParent.check(); err != nil {
		return err
	}
	if err := p.original.custody(); err != nil {
		return err
	}
	return p.store.check()
}
func (p *nativeRetirementPreparation) close() error {
	if p == nil || p.closed {
		return nil
	}
	p.closed = true
	var err error
	if p.original != nil {
		err = errors.Join(err, p.original.close())
		p.original = nil
	}
	if p.generationParent != nil && !p.transferred {
		err = errors.Join(err, windows.CloseHandle(p.generationParent.handle))
		p.generationParent = nil
	}
	return err
}

func (p *nativeRetirementPreparation) commit(ctx context.Context) (retirementBackend, error) {
	if p == nil || p.closed || p.transferred || ctx == nil || ctx.Err() != nil {
		return nil, ErrClosed
	}
	// The production entry point always resolves a distinct real client user
	// before DELETE acquisition. Only the inert temporary native fixture below
	// enters the common handoff with its already checked directory descriptor.
	client, err := clientSID(p.binding.ClientSID, p.store.broker)
	if err != nil {
		return nil, err
	}
	selectedSD, err := descriptor(p.store.broker, client, true)
	if err != nil {
		return nil, err
	}
	return p.commitWithSelectedDescriptor(ctx, selectedSD)
}

func (p *nativeRetirementPreparation) commitWithSelectedDescriptor(ctx context.Context, selectedSD *windows.SECURITY_DESCRIPTOR) (retirementBackend, error) {
	if p == nil || p.closed || p.transferred || ctx == nil || ctx.Err() != nil || selectedSD == nil {
		return nil, ErrClosed
	}
	if err := p.check(); err != nil {
		return nil, err
	}
	r := &nativeRetirement{store: p.store, binding: p.binding, generationParent: p.generationParent}
	// Closing the read-only inventory is a controlled pre-marker handoff. The
	// protected parent remains held; fresh DELETE handles must match all three
	// full original file IDs before any terminal or retirement record is created.
	if err := p.original.close(); err != nil {
		return nil, err
	}
	p.original = nil
	// The preparation cannot close this parent after the read-owner handoff.
	// Return the retirement owner even on a pre-marker acquisition refusal so
	// Store.Close can drain any DELETE handles already acquired.
	p.transferred = true
	p.generationParent = nil
	var err error
	privateFile, err := descriptor(p.store.broker, nil, false)
	if err != nil {
		return r, err
	}
	items := []struct {
		parent *nativePin
		name   string
		id     diskIdentity
		dir    bool
		sd     *windows.SECURITY_DESCRIPTOR
	}{
		{p.store.registrations, p.binding.ClientSID + ".once", p.registrationOnce, false, privateFile},
		{p.store.registrations, p.binding.ClientSID + ".json", p.registrationJSON, false, privateFile},
		{r.generationParent, p.binding.GenerationID, p.generationID, true, selectedSD},
	}
	for i, item := range items {
		r.objects[i], err = openRetirementObject(item.parent, item.name, item.id, item.dir, item.sd)
		if err != nil {
			return r, err
		}
	}
	if err := r.originalCheck(); err != nil {
		return r, err
	}
	if err := ctx.Err(); err != nil {
		return r, err
	}
	return r, r.retire(ctx)
}

func openRetirementObject(parent *nativePin, name string, want diskIdentity, directory bool, sd *windows.SECURITY_DESCRIPTOR) (result *retirementObject, resultErr error) {
	if parent == nil || !safeComponent(name) || !want.valid() {
		return nil, errors.New("fixed original retirement object required")
	}
	if err := parent.check(); err != nil {
		return nil, err
	}
	path := parent.name + `\` + name
	pointer, err := windows.UTF16PtrFromString(physical(path))
	if err != nil {
		return nil, err
	}
	access, flags := uint32(windows.DELETE|windows.READ_CONTROL|windows.FILE_READ_ATTRIBUTES), uint32(windows.FILE_FLAG_OPEN_REPARSE_POINT)
	if directory {
		access |= windows.FILE_LIST_DIRECTORY
		flags |= windows.FILE_FLAG_BACKUP_SEMANTICS
	} else {
		access |= windows.GENERIC_READ
	}
	handle, err := windows.CreateFile(pointer, access, windows.FILE_SHARE_READ, nil, windows.OPEN_EXISTING, flags, 0)
	if err != nil {
		return nil, err
	}
	ok := false
	defer func() {
		if !ok {
			resultErr = errors.Join(resultErr, windows.CloseHandle(handle))
		}
	}()
	object := &retirementObject{handle: handle, parent: parent, name: path, id: want, directory: directory, security: sd}
	if err = object.check(); err != nil {
		return nil, err
	}
	ok = true
	return object, nil
}
func (o *retirementObject) check() error {
	if o == nil || o.parent == nil {
		return ErrClosed
	}
	if err := o.parent.check(); err != nil {
		return err
	}
	id, err := fileIdentity(o.handle)
	if err != nil || id != o.id {
		return errors.Join(errors.New("original retirement file identity changed"), err)
	}
	name, err := nativeName(o.handle)
	if err != nil || name != o.name {
		return errors.Join(errors.New("retirement file escaped held parent"), err)
	}
	var info windows.ByHandleFileInformation
	if err = windows.GetFileInformationByHandle(o.handle, &info); err != nil {
		return err
	}
	if info.FileAttributes&windows.FILE_ATTRIBUTE_REPARSE_POINT != 0 || info.VolumeSerialNumber != o.parent.info.VolumeSerialNumber || info.NumberOfLinks != 1 || (info.FileAttributes&windows.FILE_ATTRIBUTE_DIRECTORY != 0) != o.directory {
		return errors.New("retirement object type/link/volume differs")
	}
	return checkSecurity(o.handle, o.security)
}
func (o *retirementObject) close() error {
	if o == nil || o.handle == 0 {
		return nil
	}
	h := o.handle
	o.handle = 0
	return windows.CloseHandle(h)
}

func retirementAbsent(parent *nativePin, name string) error {
	if err := parent.check(); err != nil {
		return err
	}
	pointer, err := windows.UTF16PtrFromString(physical(parent.name + `\` + name))
	if err != nil {
		return err
	}
	_, err = windows.GetFileAttributes(pointer)
	if errors.Is(err, windows.ERROR_FILE_NOT_FOUND) {
		return nil
	}
	return errors.Join(errors.New("protected retirement destination already exists or uncertain"), err)
}
func (o *retirementObject) move(ctx context.Context, destination string) error {
	if ctx == nil || ctx.Err() != nil {
		return errors.New("live original retirement required")
	}
	if err := o.check(); err != nil {
		return err
	}
	if err := retirementAbsent(o.parent, destination); err != nil {
		return err
	}
	component, err := windows.UTF16FromString(destination)
	if err != nil {
		return err
	}
	component = component[:len(component)-1]
	type renameInfo struct {
		ReplaceIfExists uint32
		RootDirectory   windows.Handle
		FileNameLength  uint32
		FileName        [1]uint16
	}
	offset := unsafe.Offsetof(renameInfo{}.FileName)
	buffer := make([]byte, int(offset)+len(component)*2)
	header := (*renameInfo)(unsafe.Pointer(&buffer[0]))
	header.RootDirectory, header.FileNameLength = o.parent.handle, uint32(len(component)*2)
	copy(unsafe.Slice((*uint16)(unsafe.Pointer(&buffer[offset])), len(component)), component)
	if err = ctx.Err(); err != nil {
		return err
	}
	var status windows.IO_STATUS_BLOCK
	if err = windows.NtSetInformationFile(o.handle, &status, &buffer[0], uint32(len(buffer)), windows.FileRenameInformation); err != nil {
		return err
	}
	// NTFS may tunnel creation time on rename. Volume and 128-bit file ID must
	// remain exact; capture the new created timestamp only for subsequent checks.
	updated, err := fileIdentity(o.handle)
	if err != nil || updated.Volume != o.id.Volume || updated.FileID != o.id.FileID {
		return errors.Join(errors.New("retired original object identity differs"), err)
	}
	o.name = o.parent.name + `\` + destination
	o.id = updated
	if err = o.check(); err != nil {
		return err
	}
	return nil
}

func (r *nativeRetirement) originalCheck() error {
	if r == nil || r.closed || r.generationParent == nil {
		return ErrClosed
	}
	r.store.mu.Lock()
	err := r.store.checkCustodyLocked()
	r.store.mu.Unlock()
	if err != nil {
		return err
	}
	for _, o := range r.objects {
		if o == nil {
			return errors.New("complete original retirement handles required")
		}
		if err = o.check(); err != nil {
			return err
		}
	}
	for _, item := range []struct {
		parent *nativePin
		name   string
	}{
		{r.store.root, retirementStartedName(r.binding)},
		{r.store.root, retirementCompletedName(r.binding)},
		{r.store.registrations, retirementRegistrationName(r.binding, "once")},
		{r.store.registrations, retirementRegistrationName(r.binding, "json")},
		{r.generationParent, retirementGenerationName(r.binding)},
	} {
		if err = retirementAbsent(item.parent, item.name); err != nil {
			return err
		}
	}
	return nil
}

func (r *nativeRetirement) retire(ctx context.Context) (resultErr error) {
	if err := r.originalCheck(); err != nil {
		return err
	}
	fileSD, err := descriptor(r.store.broker, nil, false)
	if err != nil {
		return err
	}
	r.markerRaw = decommissionBytes(r.binding)
	marker, err := publishDecommission(ctx, func() (candidateFile, error) {
		file, e := createRegistrationFile(r.store.root, decommissionFilename, fileSD)
		r.marker = file
		if file != nil {
			r.store.mu.Lock()
			r.store.decommissioned = true
			r.store.mu.Unlock()
			r.terminal = true
		}
		return file, e
	}, r.markerRaw, r.originalCheck)
	_ = marker
	if err != nil {
		return errors.Join(ErrRetirementUncertain, err)
	}
	start := retirementStartRecord{1, r.binding.RequestID, r.binding.ClientSID, r.binding.GenerationID, r.binding.RegistrationSHA256, r.binding.SelectedHeadSHA256, r.objects[0].id, r.objects[1].id, r.objects[2].id}
	r.startedRaw, err = json.Marshal(start)
	if err != nil {
		return errors.Join(ErrRetirementUncertain, err)
	}
	r.started, err = createRegistrationFile(r.store.root, retirementStartedName(r.binding), fileSD)
	if err != nil {
		return errors.Join(ErrRetirementUncertain, err)
	}
	if err = writeRegistrationBytes(context.WithoutCancel(ctx), r.started, 0, r.startedRaw, r.terminalMarkerCheck); err != nil {
		return errors.Join(ErrRetirementUncertain, err)
	}
	for i, destination := range []string{retirementRegistrationName(r.binding, "once"), retirementRegistrationName(r.binding, "json"), retirementGenerationName(r.binding)} {
		if err = r.terminalCheck(); err != nil {
			return errors.Join(ErrRetirementUncertain, err)
		}
		if err = r.objects[i].move(context.WithoutCancel(ctx), destination); err != nil {
			return errors.Join(ErrRetirementUncertain, err)
		}
	}
	complete := retirementCompleteRecord{1, r.binding.RequestID, registrationDigest(r.startedRaw), registrationDigest(r.markerRaw), r.objects[0].id, r.objects[1].id, r.objects[2].id}
	r.completedRaw, err = json.Marshal(complete)
	if err != nil {
		return errors.Join(ErrRetirementUncertain, err)
	}
	r.completed, err = createRegistrationFile(r.store.root, retirementCompletedName(r.binding), fileSD)
	if err != nil {
		return errors.Join(ErrRetirementUncertain, err)
	}
	if err = writeRegistrationBytes(context.WithoutCancel(ctx), r.completed, 0, r.completedRaw, r.terminalCheck); err != nil {
		return errors.Join(ErrRetirementUncertain, err)
	}
	r.completedMove = true
	return r.check()
}
func (r *nativeRetirement) terminalCheck() error {
	if err := r.terminalMarkerCheck(); err != nil {
		return err
	}
	if r.started == nil {
		return ErrRetirementUncertain
	}
	_, err := decommissionReadExact(r.started, int64(len(r.startedRaw)), r.startedRaw)
	return err
}
func (r *nativeRetirement) terminalMarkerCheck() error {
	if !r.terminal || r.closed {
		return ErrRetirementUncertain
	}
	r.store.mu.Lock()
	err := r.store.checkCustodyLocked()
	r.store.mu.Unlock()
	if err != nil {
		return err
	}
	if r.marker == nil {
		return errors.New("original terminal marker unavailable")
	}
	_, err = decommissionReadExact(r.marker, int64(len(r.markerRaw)), r.markerRaw)
	if err != nil {
		return err
	}
	for _, o := range r.objects {
		if o == nil {
			return ErrRetirementUncertain
		}
		if err = o.check(); err != nil {
			return err
		}
	}
	return nil
}
func (r *nativeRetirement) check() error {
	if !r.completedMove || r.completed == nil {
		return ErrRetirementUncertain
	}
	if err := r.terminalCheck(); err != nil {
		return err
	}
	_, err := decommissionReadExact(r.completed, int64(len(r.completedRaw)), r.completedRaw)
	return err
}
func (r *nativeRetirement) fenceSHA256() (string, error) {
	if err := r.check(); err != nil {
		return "", err
	}
	return registrationDigest(r.markerRaw), nil
}
func (r *nativeRetirement) evidenceSHA256() (string, error) {
	if err := r.check(); err != nil {
		return "", err
	}
	return registrationDigest(r.completedRaw), nil
}
func (r *nativeRetirement) close() error {
	if r == nil || r.closed {
		return r.closeErr
	}
	r.closed = true
	for i := len(r.objects) - 1; i >= 0; i-- {
		r.closeErr = errors.Join(r.closeErr, r.objects[i].close())
	}
	for _, f := range []*nativeFile{r.completed, r.started, r.marker} {
		if f != nil {
			r.closeErr = errors.Join(r.closeErr, f.Close())
		}
	}
	if r.generationParent != nil {
		r.closeErr = errors.Join(r.closeErr, windows.CloseHandle(r.generationParent.handle))
		r.generationParent = nil
	}
	return r.closeErr
}

// Prevent accidental use as a generic filesystem plan or serialized approval.
func (*nativeRetirement) MarshalJSON() ([]byte, error) { return nil, ErrNativeRequired }
func (*nativeRetirement) UnmarshalJSON([]byte) error   { return ErrNativeRequired }
