package brokerstore

import (
	"bytes"
	"context"
	"errors"
	"path/filepath"
	"sort"
	"strings"

	"example.com/opennav-update-verifier/brokerregistration"
	"golang.org/x/sys/windows"
)

type nativeBaselineFile struct {
	*nativeFile
	identity diskIdentity
}

func (f *nativeBaselineFile) check(size int64) error {
	if err := f.nativeFile.check(size); err != nil {
		return err
	}
	id, err := fileIdentity(f.pin.handle)
	if err != nil {
		return err
	}
	if id != f.identity {
		return errors.New("baseline full file identity changed")
	}
	return nil
}
func baselineFile(parent *nativePin, component string, sd *windows.SECURITY_DESCRIPTOR, create bool) (*nativeBaselineFile, error) {
	var f *nativeFile
	var err error
	if create {
		f, err = createRegistrationFile(parent, component, sd)
	} else {
		f, err = openAcquisitionFile(parent, component, sd, windows.GENERIC_READ|windows.READ_CONTROL, windows.FILE_SHARE_READ, windows.OPEN_EXISTING)
	}
	if err != nil {
		return nil, err
	}
	id, err := fileIdentity(f.pin.handle)
	if err != nil {
		return nil, errors.Join(err, f.Close())
	}
	return &nativeBaselineFile{nativeFile: f, identity: id}, nil
}

type nativeInstalledBaseline struct {
	store                             *nativeStore
	client, generation, approved, dos string
	user, installed                   *nativePin
	tree                              *nativeCandidate
	privateFileSD                     *windows.SECURITY_DESCRIPTOR
	marker, record                    *nativeBaselineFile
	markerLength                      int64
	markerBytes                       []byte
	recordDigest                      string
	pins                              map[*nativePin]diskIdentity
	members                           map[string]*nativeBaselineFile
	closed                            bool
	closeErr                          error
}

func (n *nativeStore) prepareInstalledBaseline(client, generation, approved string) (installedBaselineBackend, error) {
	return n.baselineOwner(client, generation, approved)
}
func (n *nativeStore) baselineOwner(client, generation, approved string) (result *nativeInstalledBaseline, resultErr error) {
	if err := n.check(); err != nil {
		return nil, err
	}
	if !brokerregistration.IsCanonicalUserSID(client) || !validHex(generation, 32) || !validHex(approved, 64) || n.users == nil || n.registrations == nil {
		return nil, errors.New("fixed actual installed baseline identities required")
	}
	sid, err := clientSID(client, n.broker)
	if err != nil {
		return nil, err
	}
	return prepareNativeInstalledBaseline(n, sid, client, generation, approved)
}

// The production caller first validates the actual principal, fixed store and
// canonical real client account. Inert native tests use only this lower helper.
func prepareNativeInstalledBaseline(n *nativeStore, sid *windows.SID, client, generation, approved string) (result *nativeInstalledBaseline, resultErr error) {
	userSD, err := descriptor(n.broker, nil, true)
	if err != nil {
		return nil, err
	}
	dirSD, err := descriptor(n.broker, sid, true)
	if err != nil {
		return nil, err
	}
	fileSD, err := descriptor(n.broker, sid, false)
	if err != nil {
		return nil, err
	}
	privateSD, err := descriptor(n.broker, nil, false)
	if err != nil {
		return nil, err
	}
	b := &nativeInstalledBaseline{store: n, client: client, generation: generation, approved: approved, dos: filepath.Join(n.rootDOS, "users", client, "installed", generation), privateFileSD: privateSD, pins: map[*nativePin]diskIdentity{}, members: map[string]*nativeBaselineFile{}, tree: &nativeCandidate{store: n, client: client, directories: map[string]*nativePin{}, securityDir: dirSD, securityFile: fileSD}}
	ok := false
	defer func() {
		if !ok {
			resultErr = errors.Join(resultErr, b.close())
		}
	}()
	name := n.users.name + `\` + client
	b.user, err = openPin(physical(name), name, userSD)
	if err != nil {
		return nil, err
	}
	if err = b.pin(b.user); err != nil {
		return nil, err
	}
	if err = b.check(); err != nil {
		return nil, err
	}
	ok = true
	return b, nil
}
func (b *nativeInstalledBaseline) pin(pin *nativePin) error {
	id, err := fileIdentity(pin.handle)
	if err != nil {
		return err
	}
	if pin.info.VolumeSerialNumber != b.store.users.info.VolumeSerialNumber {
		return errors.New("baseline directory crossed volumes")
	}
	b.pins[pin] = id
	return nil
}
func (b *nativeInstalledBaseline) markerName() string {
	return b.client + "." + b.generation + ".baseline.once"
}
func (b *nativeInstalledBaseline) check() error {
	if b.closed {
		return ErrClosed
	}
	if err := b.store.check(); err != nil {
		return err
	}
	for pin, id := range b.pins {
		if err := pin.check(); err != nil {
			return err
		}
		actual, err := fileIdentity(pin.handle)
		if err != nil {
			return err
		}
		if actual != id {
			return errors.New("baseline full directory identity changed")
		}
	}
	if b.marker != nil {
		if err := b.marker.check(b.markerLength); err != nil {
			return err
		}
		if b.markerBytes != nil {
			raw, err := readBaselineFile(context.Background(), b.marker, b.markerLength)
			if err != nil {
				return err
			}
			if !bytes.Equal(raw, b.markerBytes) {
				return errors.New("held baseline reservation bytes changed")
			}
		}
	}
	if b.record != nil {
		length := int64(uint64(b.record.pin.info.FileSizeHigh)<<32 | uint64(b.record.pin.info.FileSizeLow))
		if err := b.record.check(length); err != nil {
			return err
		}
		if b.recordDigest != "" {
			raw, err := readBaselineFile(context.Background(), b.record, length)
			if err != nil {
				return err
			}
			if registrationDigest(raw) != b.recordDigest {
				return errors.New("held baseline completion evidence changed")
			}
		}
	}
	return nil
}
func (b *nativeInstalledBaseline) reserve(ctx context.Context) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	if err := b.check(); err != nil {
		return err
	}
	if b.marker != nil {
		return errors.New("baseline reservation consumed")
	}
	var err error
	b.marker, err = baselineFile(b.store.registrations, b.markerName(), b.privateFileSD, true)
	if err != nil {
		return err
	}
	raw := baselineMarker(b.client, b.generation, b.approved, "", false)
	if err = writeRegistrationBytes(ctx, b.marker, 0, raw, b.checkPins); err != nil {
		return err
	}
	b.markerLength = int64(len(raw))
	b.markerBytes = bytes.Clone(raw)
	return b.check()
}
func (b *nativeInstalledBaseline) checkPins() error {
	// Do not check a marker length while its initial durable bytes are written.
	marker := b.marker
	b.marker = nil
	err := b.check()
	b.marker = marker
	return err
}
func (b *nativeInstalledBaseline) newDirectory(parent *nativePin, component string) (*nativePin, error) {
	if len(b.tree.pins) >= MaxFiles {
		return nil, errors.New("baseline directory bound exhausted")
	}
	if b.marker == nil || b.markerLength == 0 {
		return nil, errors.New("durable baseline reservation required before directory creation")
	}
	if err := b.check(); err != nil {
		return nil, err
	}
	pin, err := b.tree.newDirectory(parent, component)
	if err != nil {
		return nil, err
	}
	if err = b.pin(pin); err != nil {
		return nil, err
	}
	return pin, nil
}
func (b *nativeInstalledBaseline) createTree() error {
	if b.installed != nil {
		return errors.New("baseline tree already created")
	}
	var err error
	b.installed, err = b.newDirectory(b.user, "installed")
	if err != nil {
		return err
	}
	root, err := b.newDirectory(b.installed, b.generation)
	if err != nil {
		return err
	}
	root.dos = b.dos
	b.tree.directories[""] = root
	return b.check()
}
func (b *nativeInstalledBaseline) create(relative string) (candidateFile, error) {
	if err := b.check(); err != nil {
		return nil, err
	}
	if b.record != nil || b.tree.directories[""] == nil {
		return nil, errors.New("one uncompleted fresh baseline tree required")
	}
	if !baselineEvidenceName(relative) {
		if err := memberPath(relative); err != nil {
			return nil, err
		}
	}
	parts := strings.Split(relative, "/")
	parent := b.tree.directories[""]
	for i, component := range parts[:len(parts)-1] {
		key := strings.Join(parts[:i+1], "/")
		next, ok := b.tree.directories[key]
		if !ok {
			var err error
			next, err = b.newDirectory(parent, component)
			if err != nil {
				return nil, err
			}
			b.tree.directories[key] = next
		}
		parent = next
	}
	f, err := baselineFile(parent, parts[len(parts)-1], b.tree.securityFile, true)
	if err != nil {
		return nil, err
	}
	b.members[relative] = f
	return f, nil
}
func (b *nativeInstalledBaseline) inventory(names []string, completed bool) error {
	if err := b.check(); err != nil {
		return err
	}
	if b.installed == nil || b.tree.directories[""] == nil {
		return errors.New("baseline namespace unavailable")
	}
	parentEntries, err := boundedEntries(b.installed, 2)
	if err != nil {
		return err
	}
	if len(parentEntries) != 1 || parentEntries[0].Name() != b.generation {
		return errors.New("installed generation namespace differs")
	}
	expected := map[string]map[string]bool{}
	for name := range b.tree.directories {
		expected[name] = map[string]bool{}
	}
	for name := range b.tree.directories {
		if name == "" {
			continue
		}
		parent, child := splitBaselineName(name)
		if expected[parent] == nil {
			return errors.New("baseline directory parent absent")
		}
		expected[parent][child] = true
	}
	for _, name := range names {
		parent, child := splitBaselineName(name)
		if expected[parent] == nil {
			return errors.New("baseline member parent absent")
		}
		if expected[parent][child] {
			return errors.New("baseline file/directory collision")
		}
		expected[parent][child] = true
	}
	if completed {
		expected[""][baselineRecordFilename] = true
	}
	for name, wanted := range expected {
		entries, err := boundedEntries(b.tree.directories[name], len(wanted)+1)
		if err != nil {
			return err
		}
		if len(entries) != len(wanted) {
			return errors.New("baseline has missing or additional namespace entries")
		}
		for _, entry := range entries {
			if !wanted[entry.Name()] {
				return errors.New("baseline namespace has alias or unknown entry")
			}
		}
	}
	return b.check()
}
func splitBaselineName(name string) (string, string) {
	i := strings.LastIndexByte(name, '/')
	if i < 0 {
		return "", name
	}
	return name[:i], name[i+1:]
}
func (b *nativeInstalledBaseline) identities(members map[string]baselineMember) (map[string]diskIdentity, map[string]diskIdentity, error) {
	if err := b.check(); err != nil {
		return nil, nil, err
	}
	dirs := map[string]diskIdentity{"@user": b.pins[b.user], "@installed": b.pins[b.installed]}
	files := map[string]diskIdentity{}
	for name, pin := range b.tree.directories {
		dirs[name] = b.pins[pin]
	}
	for name := range members {
		f := b.members[name]
		if f == nil {
			return nil, nil, errors.New("baseline file identity unavailable")
		}
		if err := f.check(members[name].Length); err != nil {
			return nil, nil, err
		}
		files[name] = f.identity
	}
	return dirs, files, nil
}
func (b *nativeInstalledBaseline) publish(ctx context.Context, raw []byte) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	if err := b.check(); err != nil {
		return err
	}
	if b.record != nil {
		return errors.New("baseline publication already attempted")
	}
	file, err := baselineFile(b.tree.directories[""], baselineRecordFilename, b.privateFileSD, true)
	if err != nil {
		return err
	}
	b.record = file
	if err = writeRegistrationBytes(ctx, b.record, 0, raw, func() error { record := b.record; b.record = nil; err := b.check(); b.record = record; return err }); err != nil {
		return err
	}
	b.recordDigest = registrationDigest(raw)
	// Retain the completed record's exact length for later custody checks.
	if err = windows.GetFileInformationByHandle(file.pin.handle, &file.pin.info); err != nil {
		return err
	}
	names := make([]string, 0, len(b.members))
	for name := range b.members {
		names = append(names, name)
	}
	if err = b.inventory(names, true); err != nil {
		return err
	}
	completed := baselineMarker(b.client, b.generation, b.approved, registrationDigest(raw), true)
	if int64(len(completed)) != b.markerLength {
		return errors.New("baseline reservation width differs")
	}
	check := func() error {
		if err := b.check(); err != nil {
			return err
		}
		return b.record.check(int64(len(raw)))
	}
	// The publisher verifies exact marker bytes itself while the status changes.
	// Retained content checks resume with the completed representation on success.
	started := b.markerBytes
	b.markerBytes = nil
	if err = publishRegistrationCompletion(ctx, b.marker, b.markerLength, completed, check); err != nil {
		b.markerBytes = started
		if errors.Is(err, ErrRegistrationUncertain) {
			return errors.Join(ErrInstalledBaselineUncertain, err)
		}
		return err
	}
	b.markerBytes = bytes.Clone(completed)
	return nil
}
func (n *nativeStore) openInstalledBaseline(ctx context.Context, client, generation, approved string) (result installedBaselineBackend, raw []byte, resultErr error) {
	b, err := n.baselineOwner(client, generation, approved)
	if err != nil {
		return nil, nil, err
	}
	return openNativeInstalledBaseline(ctx, b)
}
func openNativeInstalledBaseline(ctx context.Context, b *nativeInstalledBaseline) (result installedBaselineBackend, raw []byte, resultErr error) {
	client, generation, approved := b.client, b.generation, b.approved
	var err error
	ok := false
	defer func() {
		if !ok {
			resultErr = errors.Join(resultErr, b.close())
		}
	}()
	if err = ctx.Err(); err != nil {
		return nil, nil, err
	}
	b.marker, err = baselineFile(b.store.registrations, b.markerName(), b.privateFileSD, false)
	if err != nil {
		return nil, nil, err
	}
	b.markerLength = int64(uint64(b.marker.pin.info.FileSizeHigh)<<32 | uint64(b.marker.pin.info.FileSizeLow))
	if b.markerLength <= 0 || b.markerLength > maxRegistrationMarkerBytes {
		return nil, nil, errors.New("baseline reservation size bound")
	}
	marker, err := readBaselineFile(ctx, b.marker, b.markerLength)
	if err != nil {
		return nil, nil, err
	}
	b.markerBytes = bytes.Clone(marker)
	digest, err := completedBaselineDigest(marker, client, generation, approved)
	if err != nil {
		return nil, nil, err
	}
	installedName := b.user.name + `\installed`
	b.installed, err = openPin(physical(installedName), installedName, b.tree.securityDir)
	if err != nil {
		return nil, nil, err
	}
	b.tree.pins = append(b.tree.pins, b.installed)
	if err = b.pin(b.installed); err != nil {
		return nil, nil, err
	}
	name := b.installed.name + `\` + generation
	root, err := openPin(physical(name), name, b.tree.securityDir)
	if err != nil {
		return nil, nil, err
	}
	b.tree.pins = append(b.tree.pins, root)
	if err = b.pin(root); err != nil {
		return nil, nil, err
	}
	root.dos = b.dos
	b.tree.directories[""] = root
	b.record, err = baselineFile(root, baselineRecordFilename, b.privateFileSD, false)
	if err != nil {
		return nil, nil, err
	}
	length := int64(uint64(b.record.pin.info.FileSizeHigh)<<32 | uint64(b.record.pin.info.FileSizeLow))
	if length <= 0 || length > maxBaselineRecordBytes {
		return nil, nil, errors.New("baseline completion record bound")
	}
	raw, err = readBaselineFile(ctx, b.record, length)
	if err != nil {
		return nil, nil, err
	}
	b.recordDigest = digest
	if registrationDigest(raw) != digest {
		return nil, nil, errors.New("baseline record differs from completed permanent reservation")
	}
	if err = b.check(); err != nil {
		return nil, nil, err
	}
	ok = true
	return b, raw, nil
}
func readBaselineFile(ctx context.Context, f *nativeBaselineFile, length int64) ([]byte, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if err := f.check(length); err != nil {
		return nil, err
	}
	raw := make([]byte, length)
	if _, err := f.ReadAt(raw, 0); err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	return raw, f.check(length)
}
func (b *nativeInstalledBaseline) openMembers(members map[string]baselineMember, dirs map[string]diskIdentity) (result map[string]candidateFile, resultErr error) {
	result = map[string]candidateFile{}
	if dirs["@user"] != b.pins[b.user] || dirs["@installed"] != b.pins[b.installed] || dirs[""] != b.pins[b.tree.directories[""]] {
		return result, errors.New("baseline creation parent or generation identity differs")
	}
	var names []string
	for name, id := range dirs {
		if !id.valid() {
			return result, errors.New("invalid baseline directory identity")
		}
		if name == "" || name == "@user" || name == "@installed" {
			continue
		}
		if name != "app" && name != "docs" && name != "installer" {
			if err := memberPath(name); err != nil {
				return result, err
			}
		}
		names = append(names, name)
	}
	sort.Strings(names)
	for _, name := range names {
		parent, component := splitBaselineName(name)
		parentPin := b.tree.directories[parent]
		if parentPin == nil {
			return result, errors.New("baseline recorded directory parent missing")
		}
		physicalName := parentPin.name + `\` + component
		pin, err := openPin(physical(physicalName), physicalName, b.tree.securityDir)
		if err != nil {
			return result, err
		}
		b.tree.pins = append(b.tree.pins, pin)
		if err = b.pin(pin); err != nil {
			return result, err
		}
		if b.pins[pin] != dirs[name] {
			return result, errors.New("baseline created directory identity differs")
		}
		b.tree.directories[name] = pin
	}
	names = names[:0]
	for name := range members {
		names = append(names, name)
	}
	sort.Strings(names)
	for _, name := range names {
		parent, component := splitBaselineName(name)
		parentPin := b.tree.directories[parent]
		if parentPin == nil {
			return result, errors.New("baseline recorded member parent missing")
		}
		f, err := baselineFile(parentPin, component, b.tree.securityFile, false)
		if err != nil {
			return result, err
		}
		result[name] = f
		b.members[name] = f
		if f.identity != members[name].Identity {
			return result, errors.New("baseline created member identity differs")
		}
		if err = f.check(members[name].Length); err != nil {
			return result, err
		}
	}
	if err := b.inventory(names, true); err != nil {
		return result, err
	}
	return result, b.check()
}
func (b *nativeInstalledBaseline) directory() (string, error) {
	if err := b.check(); err != nil {
		return "", err
	}
	if b.record == nil {
		return "", errors.New("completed baseline required")
	}
	return b.dos, nil
}
func (b *nativeInstalledBaseline) close() error {
	if b.closed {
		return b.closeErr
	}
	b.closed = true
	if b.record != nil {
		b.closeErr = errors.Join(b.closeErr, b.record.Close())
	}
	if b.marker != nil {
		b.closeErr = errors.Join(b.closeErr, b.marker.Close())
	}
	b.closeErr = errors.Join(b.closeErr, b.tree.close())
	if b.user != nil {
		b.closeErr = errors.Join(b.closeErr, windows.CloseHandle(b.user.handle))
	}
	return b.closeErr
}
