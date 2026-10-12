package brokerstore

import (
	"errors"
	"io"
	"os"
	"unsafe"

	"golang.org/x/sys/windows"
)

// boundedEntries corroborates the actual opened directory against its retained
// pin before enumerating a bounded single level. This is not a recursive sweep.
func boundedEntries(pin *nativePin, limit int) (result []os.DirEntry, resultErr error) {
	if err := pin.check(); err != nil {
		return nil, err
	}
	f, err := os.Open(physical(pin.name))
	if err != nil {
		return nil, err
	}
	defer func() { resultErr = errors.Join(resultErr, f.Close()) }()
	h := windows.Handle(f.Fd())
	var info windows.ByHandleFileInformation
	if err = windows.GetFileInformationByHandle(h, &info); err != nil {
		return nil, err
	}
	name, err := nativeName(h)
	if err != nil || name != pin.name || !sameIdentity(info, pin.info) || info.FileAttributes&(windows.FILE_ATTRIBUTE_DIRECTORY|windows.FILE_ATTRIBUTE_REPARSE_POINT) != windows.FILE_ATTRIBUTE_DIRECTORY {
		return nil, errors.New("enumerated physical directory changed")
	}
	entries, err := f.ReadDir(limit + 1)
	if err != nil && !errors.Is(err, io.EOF) {
		return nil, err
	}
	if len(entries) > limit {
		return nil, errors.New("reconciliation namespace bound exceeded")
	}
	for _, entry := range entries {
		if !safeComponent(entry.Name()) {
			return nil, errors.New("uncertain namespace component")
		}
	}
	if err = pin.check(); err != nil {
		return nil, err
	}
	return entries, nil
}

// Existing candidate ACLs are reconstructed only from the exact original
// broker/SYSTEM/client form, with an actual canonical user SID. No journal path
// or claimed client string chooses what security or physical directory to trust.
func openRecoveryCandidate(n *nativeStore, stage string) (result *nativeCandidate, resultErr error) {
	name := n.candidates.name + `\` + stage
	pin, err := openPin(physical(name), name, nil)
	if err != nil {
		return nil, err
	}
	ok := false
	defer func() {
		if !ok {
			resultErr = errors.Join(resultErr, windows.CloseHandle(pin.handle))
		}
	}()
	sd, err := windows.GetSecurityInfo(pin.handle, windows.SE_FILE_OBJECT, windows.OWNER_SECURITY_INFORMATION|windows.DACL_SECURITY_INFORMATION)
	if err != nil {
		return nil, err
	}
	acl, _, err := sd.DACL()
	if err != nil || acl == nil || acl.AceCount != 3 {
		return nil, errors.New("unrecognized candidate ownership descriptor")
	}
	var ace *windows.ACCESS_ALLOWED_ACE
	if err = windows.GetAce(acl, 2, &ace); err != nil {
		return nil, err
	}
	// Validate the whole descriptor before using its client ACE as an identity.
	if ace.Header.AceType != windows.ACCESS_ALLOWED_ACE_TYPE {
		return nil, errors.New("candidate client ACE is not an allow")
	}
	client := aceSID(ace)
	if !client.IsValid() {
		return nil, errors.New("candidate client SID invalid")
	}
	user, err := clientSID(client.String(), n.broker)
	if err != nil {
		return nil, err
	}
	expected, err := descriptor(n.broker, user, true)
	if err != nil {
		return nil, err
	}
	if err = checkSecurity(pin.handle, expected); err != nil {
		return nil, err
	}
	if pin.info.VolumeSerialNumber != n.candidates.info.VolumeSerialNumber {
		return nil, errors.New("candidate crossed physical volume")
	}
	pin.security = expected
	c := &nativeCandidate{store: n, pins: []*nativePin{pin}, directories: map[string]*nativePin{"": pin}, securityDir: expected, stageID: stage, client: user.String()}
	ok = true
	return c, nil
}
func aceSID(ace *windows.ACCESS_ALLOWED_ACE) *windows.SID {
	return (*windows.SID)(unsafe.Pointer(&ace.SidStart))
}
func (n *nativeStore) reconcileAcquisitions() error {
	// Open owns the exclusive lifetime lock before entering here. No request has
	// been accepted; uncertain residue refuses readiness and is never adopted.
	if err := n.check(); err != nil {
		return err
	}
	entries, err := boundedEntries(n.candidates, MaxCandidates)
	if err != nil {
		return err
	}
	for _, entry := range entries {
		if !validHex(entry.Name(), 32) {
			return errors.New("unrecognized durable candidate reservation")
		}
		if err = n.reconcileCandidate(entry.Name()); err != nil {
			return err
		}
	}
	return n.check()
}
func (n *nativeStore) reconcileCandidate(stage string) (resultErr error) {
	c, err := openRecoveryCandidate(n, stage)
	if err != nil {
		return err
	}
	defer func() { resultErr = errors.Join(resultErr, c.close()) }()
	entries, err := boundedEntries(c.directories[""], 4)
	if err != nil {
		return err
	}
	hasLease := false
	for _, entry := range entries {
		switch entry.Name() {
		case "app", "docs", ReceiptFilename: // Payload/receipt are never deleted or adopted here.
		case acquisitionDirectory:
			hasLease = true
		default:
			return errors.New("unrecognized candidate residue")
		}
	}
	if !hasLease {
		return nil
	} // Older reservations remain durable and count toward the cap.
	a, err := openRecoveryAcquisition(c)
	if err != nil {
		return err
	}
	defer func() { resultErr = errors.Join(resultErr, a.closeHandles()) }()
	return reconcileAcquisition(a)
}

func reconcileAcquisition(a *nativeAcquisition) (resultErr error) {
	entries, err := boundedEntries(a.directory, 3)
	if err != nil {
		return err
	}
	present := map[ArtifactSlot]bool{}
	journalFound := false
	for _, entry := range entries {
		switch entry.Name() {
		case acquisitionJournal:
			journalFound = true
		case slotFilename(RecoverySlot):
			present[RecoverySlot] = true
		case slotFilename(InstallerSlot):
			present[InstallerSlot] = true
		default:
			return errors.New("unindexed acquisition residue")
		}
	}
	if !journalFound {
		return errors.New("lease journal missing")
	}
	// Validate every slot before deleting any. In particular a creation intent
	// without a persisted actual file ID cannot authorize deleting a matching name.
	for _, slot := range []ArtifactSlot{RecoverySlot, InstallerSlot} {
		state := a.index.states[slot]
		if state == "creating" || (present[slot] && state != "owned" && state != "deleting") || (!present[slot] && state == "owned") {
			return errors.New("uncertain acquisition artifact residue")
		}
	}
	opened := map[ArtifactSlot]*nativeArtifact{}
	defer func() {
		for _, f := range opened {
			if f.file != nil {
				resultErr = errors.Join(resultErr, f.file.Close())
				f.file = nil
			}
		}
	}()
	for _, slot := range []ArtifactSlot{RecoverySlot, InstallerSlot} {
		if !present[slot] {
			continue
		}
		spec, err := a.index.plan.spec(slot)
		if err != nil {
			return err
		}
		file, err := openAcquisitionFile(a.directory, slotFilename(slot), a.fileSD, windows.GENERIC_READ|windows.DELETE|windows.READ_CONTROL, windows.FILE_SHARE_READ, windows.OPEN_EXISTING)
		if err != nil {
			return err
		}
		f := &nativeArtifact{acquisition: a, slot: slot, spec: spec, identity: a.index.identities[slot], file: file.File, name: file.pin.name, indexed: true}
		opened[slot] = f
		if err = f.check(); err != nil {
			return err
		}
	}
	for _, slot := range []ArtifactSlot{RecoverySlot, InstallerSlot} {
		state := a.index.states[slot]
		if state != "owned" && state != "deleting" {
			continue
		}
		identity := a.index.identities[slot]
		if !present[slot] {
			// Only a durable delete intent permits acknowledging already absent bytes.
			if err = artifactAbsent(a.directory, slotFilename(slot)); err != nil {
				return err
			}
			if err = a.append("deleted", slot, &identity, nil); err != nil {
				return err
			}
			continue
		}
		f := opened[slot]
		// check() includes the full persisted file ID, exact private SD, size/type,
		// link count, physical directory and current non-impersonated broker token.
		if err = f.close(); err != nil {
			return err
		}
	}
	if err = a.check(); err != nil {
		return err
	}
	if !a.index.complete {
		if err = a.append("complete", "", nil, nil); err != nil {
			return err
		}
	}
	// Re-enumerate only this indexed lease; completion is not cleanup authority.
	entries, err = boundedEntries(a.directory, 1)
	if err != nil || len(entries) != 1 || entries[0].Name() != acquisitionJournal {
		return errors.New("acquisition completion residue uncertain")
	}
	return nil
}
func openRecoveryAcquisition(c *nativeCandidate) (result *nativeAcquisition, resultErr error) {
	dirSD, err := descriptor(c.store.broker, nil, true)
	if err != nil {
		return nil, err
	}
	fileSD, err := descriptor(c.store.broker, nil, false)
	if err != nil {
		return nil, err
	}
	name := c.directories[""].name + `\` + acquisitionDirectory
	dir, err := openPin(physical(name), name, dirSD)
	if err != nil {
		return nil, err
	}
	a := &nativeAcquisition{candidate: c, directory: dir, fileSD: fileSD}
	ok := false
	defer func() {
		if !ok {
			resultErr = errors.Join(resultErr, a.closeHandles())
		}
	}()
	a.parentID, err = fileIdentity(c.directories[""].handle)
	if err != nil {
		return nil, err
	}
	a.identity, err = fileIdentity(dir.handle)
	if err != nil {
		return nil, err
	}
	a.journal, err = openAcquisitionFile(dir, acquisitionJournal, fileSD, windows.GENERIC_READ|windows.GENERIC_WRITE|windows.READ_CONTROL, windows.FILE_SHARE_READ, windows.OPEN_EXISTING)
	if err != nil {
		return nil, err
	}
	info, err := a.journal.Stat()
	if err != nil {
		return nil, err
	}
	if info.Size() <= 0 || info.Size() > maxAcquisitionJournal {
		return nil, errors.New("invalid acquisition index length")
	}
	a.length = info.Size()
	data, err := io.ReadAll(io.LimitReader(a.journal, maxAcquisitionJournal+1))
	if err != nil || int64(len(data)) != a.length {
		return nil, errors.New("uncertain acquisition index read")
	}
	a.index, err = parseAcquisitionIndex(data)
	if err != nil {
		return nil, err
	}
	first := a.index.events[0]
	if first.StageID != c.stageID || first.ClientSID != c.client || first.Directory != a.identity || first.Parent != a.parentID {
		return nil, errors.New("acquisition index physical/client/stage binding mismatch")
	}
	if _, err = a.journal.Seek(0, io.SeekEnd); err != nil {
		return nil, err
	}
	if err = a.check(); err != nil {
		return nil, err
	}
	ok = true
	return a, nil
}
