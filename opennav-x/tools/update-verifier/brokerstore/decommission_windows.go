package brokerstore

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"io"
	"os"
	"reflect"

	"golang.org/x/sys/windows"
)

const decommissionFilename = "decommission.once"

// Absence means ONLY FILE_NOT_FOUND under the retained protected root. Neither
// marker parsing nor successful reading is needed to deny startup. A directory,
// reparse point, zero-length/partial file, access denial or uncertain parent all
// refuse. Presence is terminal, never recovery/adoption authority.
func decommissionAbsent(root *nativePin) error {
	if root == nil {
		return errors.Join(ErrDecommissioned, errors.New("original protected root unavailable"))
	}
	if err := root.check(); err != nil {
		return errors.Join(ErrDecommissioned, err)
	}
	pointer, err := windows.UTF16PtrFromString(physical(root.name + `\` + decommissionFilename))
	if err != nil {
		return errors.Join(ErrDecommissioned, err)
	}
	_, err = windows.GetFileAttributes(pointer)
	if errors.Is(err, windows.ERROR_FILE_NOT_FOUND) {
		return nil
	}
	return errors.Join(ErrDecommissioned, err)
}

type nativeDecommission struct {
	store                        *nativeStore
	binding                      DecommissionBinding
	evidence                     []*nativeFile
	expected                     [][]byte
	marker                       *nativeFile
	completed, attempted, closed bool
	closeErr                     error
	baseline                     string
	rootInventory                decommissionRootInventory
	extraFiles                   []decommissionCustodyFile
	extraPins                    map[*nativePin]diskIdentity
	extraInventories             map[*nativePin]map[string]bool
	owners                       []func() error
}

func (n *nativeStore) singletonRegistration(ctx context.Context, sid, digest string) (resultErr error) {
	if _, err := clientSID(sid, n.broker); err != nil {
		return err
	}
	d, err := openDecommissionInventory(ctx, n, DecommissionBinding{ClientSID: sid, RegistrationSHA256: digest})
	if err != nil {
		return err
	}
	defer func() { resultErr = errors.Join(resultErr, d.close()) }()
	return n.check()
}
func (n *nativeStore) prepareDecommission(ctx context.Context, binding DecommissionBinding) (decommissionBackend, error) {
	if !binding.valid() {
		return nil, errors.New("exact terminal binding required")
	}
	if _, err := clientSID(binding.ClientSID, n.broker); err != nil {
		return nil, err
	}
	return openDecommissionInventory(ctx, n, binding)
}
func (d *nativeDecommission) inventory() error {
	entries, err := boundedEntries(d.store.registrations, maxExecutionNamespaceFiles)
	if err != nil {
		return err
	}
	names := make([]string, 0, len(entries))
	for _, entry := range entries {
		names = append(names, entry.Name())
		if entry.IsDir() != (entry.Name() == d.binding.ClientSID+".generations") {
			return errors.New("protected evidence type differs from fixed schema")
		}
	}
	classified, err := classifyDecommissionInventory(names, d.binding.ClientSID, d.baseline)
	if err != nil {
		return err
	}
	if d.rootInventory.names != nil && !reflect.DeepEqual(classified.names, d.rootInventory.names) {
		return errors.New("original protected root inventory changed")
	}
	return nil
}

// This private helper is also used by inert native fixtures; the public native
// capability separately validates the canonical actual client account. Both
// original files remain held without write/delete sharing until owner cleanup.
func openDecommissionInventory(ctx context.Context, n *nativeStore, binding DecommissionBinding) (*nativeDecommission, error) {
	return openDecommissionInventoryWithOpeners(ctx, n, binding, decommissionInventoryOpeners{n.openReadOnlyGeneration, n.openStageSeal})
}
func openDecommissionInventoryWithOpeners(ctx context.Context, n *nativeStore, binding DecommissionBinding, openers decommissionInventoryOpeners) (result *nativeDecommission, resultErr error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if err := n.check(); err != nil {
		return nil, err
	}
	d := &nativeDecommission{store: n, binding: binding}
	ok := false
	defer func() {
		if !ok {
			resultErr = errors.Join(resultErr, d.close())
		}
	}()
	sd, err := descriptor(n.broker, nil, false)
	if err != nil {
		return nil, err
	}
	for i, name := range []string{binding.ClientSID + ".once", binding.ClientSID + ".json"} {
		bound := int64(maxRegistrationMarkerBytes)
		if i == 1 {
			bound = MaxRegistrationBytes
		}
		file, data, err := openDecommissionEvidence(n.registrations, name, sd, bound)
		if err != nil {
			return nil, err
		}
		d.evidence = append(d.evidence, file)
		d.expected = append(d.expected, data)
	}
	digest, err := completedRegistrationDigest(d.expected[0], binding.ClientSID)
	if err != nil || digest != binding.RegistrationSHA256 || registrationDigest(d.expected[1]) != digest {
		return nil, errors.New("original completed registration digest differs")
	}
	entries, err := boundedEntries(n.registrations, maxExecutionNamespaceFiles)
	if err != nil {
		return nil, err
	}
	names := make([]string, 0, len(entries))
	for _, entry := range entries {
		names = append(names, entry.Name())
	}
	if len(names) > 2 {
		var envelope baselineRegistrationEnvelope
		if json.Unmarshal(d.expected[1], &envelope) != nil {
			return nil, errors.New("original approved registration required")
		}
		request, err := validateBaselineRegistration(d.expected[1], d.expected[0], binding.ClientSID, envelope.ApprovedRequestSHA256)
		if err != nil {
			return nil, err
		}
		d.baseline = request.Installed.Generation
	}
	d.rootInventory, err = classifyDecommissionInventory(names, binding.ClientSID, d.baseline)
	if err != nil {
		return nil, err
	}
	if err = d.retainOwnedInventory(ctx, openers); err != nil {
		return nil, err
	}
	if err = d.inventory(); err != nil {
		return nil, err
	}
	if err = n.check(); err != nil {
		return nil, err
	}
	if err = ctx.Err(); err != nil {
		return nil, err
	}
	ok = true
	return d, nil
}
func openDecommissionEvidence(parent *nativePin, name string, sd *windows.SECURITY_DESCRIPTOR, bound int64) (result *nativeFile, data []byte, resultErr error) {
	if err := parent.check(); err != nil {
		return nil, nil, err
	}
	pointer, err := windows.UTF16PtrFromString(physical(parent.name + `\` + name))
	if err != nil {
		return nil, nil, err
	}
	handle, err := windows.CreateFile(pointer, windows.GENERIC_READ|windows.READ_CONTROL, windows.FILE_SHARE_READ, nil, windows.OPEN_EXISTING, windows.FILE_FLAG_OPEN_REPARSE_POINT, 0)
	if err != nil {
		return nil, nil, err
	}
	file := &nativeFile{File: os.NewFile(uintptr(handle), name), pin: &nativePin{handle: handle, name: parent.name + `\` + name, security: sd}, parent: parent}
	ok := false
	defer func() {
		if !ok {
			resultErr = errors.Join(resultErr, file.Close())
		}
	}()
	if err = windows.GetFileInformationByHandle(handle, &file.pin.info); err != nil {
		return nil, nil, err
	}
	length := int64(uint64(file.pin.info.FileSizeHigh)<<32 | uint64(file.pin.info.FileSizeLow))
	if length <= 0 || length > bound {
		return nil, nil, errors.New("terminal registration evidence outside bound")
	}
	kind, err := windows.GetFileType(handle)
	if err != nil || kind != windows.FILE_TYPE_DISK {
		return nil, nil, errors.New("ordinary registration evidence required")
	}
	data, err = decommissionReadExact(file, length, nil)
	if err != nil {
		return nil, nil, err
	}
	ok = true
	return file, data, nil
}
func decommissionReadExact(file candidateFile, length int64, want []byte) ([]byte, error) {
	if err := file.check(length); err != nil {
		return nil, err
	}
	if _, err := file.Seek(0, io.SeekStart); err != nil {
		return nil, err
	}
	raw, err := io.ReadAll(io.LimitReader(file, length+1))
	if err != nil {
		return nil, err
	}
	if int64(len(raw)) != length || (want != nil && !bytes.Equal(raw, want)) {
		return nil, errors.New("retained terminal evidence bytes changed")
	}
	if err = file.check(length); err != nil {
		return nil, err
	}
	return raw, nil
}
func (d *nativeDecommission) custody() error {
	if d.closed {
		return ErrClosed
	}
	n := d.store
	n.mu.Lock()
	defer n.mu.Unlock()
	if err := n.checkCustodyLocked(); err != nil {
		return err
	}
	if err := d.inventory(); err != nil {
		return err
	}
	for i, file := range d.evidence {
		if _, err := decommissionReadExact(file, int64(len(d.expected[i])), d.expected[i]); err != nil {
			return err
		}
	}
	for pin, id := range d.extraPins {
		if err := pin.check(); err != nil {
			return err
		}
		actual, err := fileIdentity(pin.handle)
		if err != nil || actual != id {
			return errors.Join(errors.New("original inventory full directory identity differs"), err)
		}
	}
	for pin, names := range d.extraInventories {
		if err := exactGenerationEntries(pin, names); err != nil {
			return err
		}
	}
	for _, held := range d.extraFiles {
		if err := held.check(); err != nil {
			return err
		}
	}
	return nil
}
func (d *nativeDecommission) commit(ctx context.Context) error {
	if d.attempted {
		return errors.New("terminal writer already attempted")
	}
	d.attempted = true
	if err := ctx.Err(); err != nil {
		return err
	}
	if err := d.custody(); err != nil {
		return err
	}
	if err := decommissionAbsent(d.store.root); err != nil {
		return err
	}
	sd, err := descriptor(d.store.broker, nil, false)
	if err != nil {
		return err
	}
	if err = ctx.Err(); err != nil {
		return err
	}
	_, err = publishDecommission(ctx, func() (candidateFile, error) {
		marker, createErr := createRegistrationFile(d.store.root, decommissionFilename, sd)
		d.marker = marker
		d.store.mu.Lock()
		d.store.decommissioned = true
		d.store.mu.Unlock()
		return marker, createErr
	}, decommissionBytes(d.binding), d.custody)
	if err != nil {
		return err
	}
	d.completed = true
	if err = d.check(); err != nil {
		return errors.Join(ErrDecommissionUncertain, err)
	}
	return nil
}
func (d *nativeDecommission) check() error {
	if !d.completed || d.marker == nil {
		return errors.New("original completed terminal writer required")
	}
	if err := d.custody(); err != nil {
		return err
	}
	raw := decommissionBytes(d.binding)
	_, err := decommissionReadExact(d.marker, int64(len(raw)), raw)
	return err
}
func (d *nativeDecommission) close() error {
	if d.closed {
		return d.closeErr
	}
	d.closed = true
	if d.marker != nil {
		d.closeErr = errors.Join(d.closeErr, d.marker.Close())
		d.marker = nil
	}
	for i := len(d.evidence) - 1; i >= 0; i-- {
		d.closeErr = errors.Join(d.closeErr, d.evidence[i].Close())
	}
	d.evidence = nil
	for i := len(d.owners) - 1; i >= 0; i-- {
		d.closeErr = errors.Join(d.closeErr, d.owners[i]())
	}
	d.owners = nil
	return d.closeErr
}
