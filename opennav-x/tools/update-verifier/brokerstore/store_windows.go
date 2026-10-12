package brokerstore

import (
	"errors"
	"io"
	"os"
	"path/filepath"
	"runtime"
	"strings"
	"sync"
	"unsafe"

	"golang.org/x/sys/windows"
)

type nativePin struct {
	handle   windows.Handle
	name     string
	info     windows.ByHandleFileInformation
	security *windows.SECURITY_DESCRIPTOR
	dos      string
	ancestor *ancestorPolicy
}
type nativeStore struct {
	mu                                     sync.Mutex
	beginMu                                sync.Mutex
	broker                                 *windows.SID
	rootDOS                                string
	pins                                   []*nativePin
	protected                              map[string]*nativePin
	root, candidates, registrations, users *nativePin
	lock                                   *nativeStoreLock
	failed, closed                         bool
	decommissioned                         bool
}
type nativeCandidate struct {
	store                     *nativeStore
	pins                      []*nativePin
	directories               map[string]*nativePin
	securityDir, securityFile *windows.SECURITY_DESCRIPTOR
	closed                    bool
	stageID, client           string
}
type nativeFile struct {
	*os.File
	pin    *nativePin
	parent *nativePin
}

func currentBroker(sid *windows.SID) error {
	user, err := windows.GetCurrentProcessToken().GetTokenUser()
	if err != nil || !user.User.Sid.Equals(sid) {
		return ErrPrincipal
	}
	var token windows.Token
	err = windows.OpenThreadToken(windows.CurrentThread(), windows.TOKEN_QUERY, true, &token)
	if err == nil {
		token.Close()
		return ErrPrincipal
	}
	if !errors.Is(err, windows.ERROR_NO_TOKEN) {
		return ErrPrincipal
	}
	effective, err := windows.GetCurrentThreadEffectiveToken().GetTokenUser()
	if err != nil || !effective.User.Sid.Equals(sid) {
		return ErrPrincipal
	}
	return nil
}
func clientSID(text string, broker *windows.SID) (*windows.SID, error) {
	sid, err := windows.StringToSid(text)
	if err != nil || !sid.IsValid() || sid.String() != text || sid.Equals(broker) || text == "S-1-5-18" {
		return nil, errors.New("canonical real client user SID required")
	}
	_, _, kind, err := sid.LookupAccount("")
	if err != nil || kind != windows.SidTypeUser {
		return nil, errors.New("client SID must resolve to an actual user account")
	}
	return sid, nil
}
func descriptor(broker, client *windows.SID, directory bool) (*windows.SECURITY_DESCRIPTOR, error) {
	flags := ""
	if directory {
		flags = "OICI"
	}
	value := "O:" + broker.String() + "D:P(A;" + flags + ";FA;;;" + broker.String() + ")(A;" + flags + ";FA;;;SY)"
	if client != nil {
		value += "(A;" + flags + ";FRFX;;;" + client.String() + ")"
	}
	return windows.SecurityDescriptorFromString(value)
}
func checkSecurity(handle windows.Handle, expected *windows.SECURITY_DESCRIPTOR) error {
	actual, err := windows.GetSecurityInfo(handle, windows.SE_FILE_OBJECT, windows.OWNER_SECURITY_INFORMATION|windows.DACL_SECURITY_INFORMATION)
	if err != nil {
		return err
	}
	control, _, err := actual.Control()
	if err != nil || control&(windows.SE_DACL_PRESENT|windows.SE_DACL_PROTECTED) != (windows.SE_DACL_PRESENT|windows.SE_DACL_PROTECTED) {
		return errors.New("protected non-null DACL required")
	}
	expectedControl, _, err := expected.Control()
	const daclFlags = windows.SE_DACL_DEFAULTED | windows.SE_DACL_AUTO_INHERIT_REQ | windows.SE_DACL_AUTO_INHERITED | windows.SE_DACL_PROTECTED
	if err != nil || control&daclFlags != expectedControl&daclFlags {
		return errors.New("unexpected DACL inheritance control")
	}
	owner, _, err := actual.Owner()
	if err != nil || owner == nil {
		return errors.New("owner unavailable")
	}
	wanted, _, err := expected.Owner()
	if err != nil || !owner.Equals(wanted) {
		return errors.New("broker ownership required")
	}
	acl, _, err := actual.DACL()
	if err != nil || acl == nil {
		return errors.New("non-null exact DACL required")
	}
	want, _, err := expected.DACL()
	if err != nil || want == nil || acl.AceCount != want.AceCount {
		return errors.New("unexpected DACL entries")
	}
	// Keep the exact broker/SYSTEM/client ACE order, as the existing verifier
	// private-state contract compares DACL SDDL rather than semantic equivalence.
	for i := uint32(0); i < uint32(acl.AceCount); i++ {
		var ace, allowed *windows.ACCESS_ALLOWED_ACE
		if err = windows.GetAce(acl, i, &ace); err != nil {
			return err
		}
		if err = windows.GetAce(want, i, &allowed); err != nil {
			return err
		}
		if ace.Header.AceType != windows.ACCESS_ALLOWED_ACE_TYPE {
			return errors.New("unexpected DACL ACE type")
		}
		sid := (*windows.SID)(unsafe.Pointer(&ace.SidStart))
		target := (*windows.SID)(unsafe.Pointer(&allowed.SidStart))
		if !sid.IsValid() || !sid.Equals(target) || ace.Header.AceFlags != allowed.Header.AceFlags || ace.Mask != allowed.Mask {
			return errors.New("DACL grants unexpected rights, order or inheritance")
		}
	}
	runtime.KeepAlive(actual)
	runtime.KeepAlive(expected)
	return nil
}

// Ancestors may allow unrelated child creation, but no untrusted principal
// may mutate their existing metadata/ACL or remove held entries. SKAGER is a
// provisioned boundary and must also forbid untrusted child additions.
type ancestorPolicy struct {
	trusted         map[string]bool
	forbidChildAdds bool
}

func checkAncestor(handle windows.Handle, policy *ancestorPolicy) error {
	sd, err := windows.GetSecurityInfo(handle, windows.SE_FILE_OBJECT, windows.OWNER_SECURITY_INFORMATION|windows.DACL_SECURITY_INFORMATION)
	if err != nil {
		return err
	}
	return checkAncestorDescriptor(sd, policy)
}
func checkAncestorDescriptor(sd *windows.SECURITY_DESCRIPTOR, policy *ancestorPolicy) error {
	owner, _, err := sd.Owner()
	if err != nil || owner == nil || !policy.trusted[owner.String()] {
		return errors.New("ancestor owner can grant untrusted mutation")
	}
	control, _, err := sd.Control()
	if err != nil || control&windows.SE_DACL_PRESENT == 0 {
		return errors.New("ancestor DACL unavailable")
	}
	acl, _, err := sd.DACL()
	if err != nil || acl == nil {
		return errors.New("ancestor non-null DACL required")
	}
	const fileDeleteChild uint32 = 0x40
	dangerous := uint32(windows.DELETE|windows.WRITE_DAC|windows.WRITE_OWNER|windows.GENERIC_ALL|windows.GENERIC_WRITE|windows.FILE_WRITE_ATTRIBUTES|windows.FILE_WRITE_EA) | fileDeleteChild
	if policy.forbidChildAdds {
		dangerous |= windows.FILE_WRITE_DATA | windows.FILE_APPEND_DATA
	} // FILE_ADD_FILE/SUBDIRECTORY
	for i := uint32(0); i < uint32(acl.AceCount); i++ {
		var ace *windows.ACCESS_ALLOWED_ACE
		if err = windows.GetAce(acl, i, &ace); err != nil {
			return err
		}
		if ace.Header.AceFlags&windows.INHERIT_ONLY_ACE != 0 {
			continue
		}
		switch ace.Header.AceType {
		case windows.ACCESS_DENIED_ACE_TYPE:
			continue // Denies never grant rights; no allow is excused by a deny.
		case windows.ACCESS_ALLOWED_ACE_TYPE:
		default:
			return errors.New("unsupported effective ancestor ACE")
		}
		sid := (*windows.SID)(unsafe.Pointer(&ace.SidStart))
		if !sid.IsValid() {
			return errors.New("invalid ancestor ACE SID")
		}
		if !policy.trusted[sid.String()] && uint32(ace.Mask)&dangerous != 0 {
			return errors.New("ancestor permits untrusted metadata, ACL or namespace mutation")
		}
	}
	runtime.KeepAlive(sd)
	return nil
}
func nativeName(handle windows.Handle) (string, error) {
	buffer := make([]uint16, 32768)
	n, err := windows.GetFinalPathNameByHandle(handle, &buffer[0], uint32(len(buffer)), 2)
	if err != nil || n == 0 || n >= uint32(len(buffer)) {
		return "", errors.New("held physical name unavailable")
	}
	name := strings.TrimRight(windows.UTF16ToString(buffer[:n]), `\`)
	if _, err = physicalParts(name); err != nil {
		return "", err
	}
	return name, nil
}
func physicalParts(name string) ([]string, error) {
	parts := strings.Split(name, `\`)
	if len(parts) < 3 || parts[0] != "" || parts[1] != "Device" || !strings.HasPrefix(parts[2], "HarddiskVolume") {
		return nil, errors.New("local physical NTFS volume required")
	}
	digits := strings.TrimPrefix(parts[2], "HarddiskVolume")
	if len(digits) < 1 || len(digits) > 10 {
		return nil, errors.New("invalid physical volume")
	}
	for _, c := range digits {
		if c < '0' || c > '9' {
			return nil, errors.New("invalid physical volume")
		}
	}
	for _, part := range parts[3:] {
		if part == "" || part == "." || part == ".." || strings.HasSuffix(part, ".") || strings.HasSuffix(part, " ") || strings.ContainsAny(part, ":/\x00") {
			return nil, errors.New("ambiguous physical ancestor")
		}
	}
	return parts, nil
}
func physical(name string) string {
	p := `\\?\GLOBALROOT` + name
	if parts, _ := physicalParts(name); len(parts) == 3 {
		p += `\`
	}
	return p
}
func sameIdentity(a, b windows.ByHandleFileInformation) bool {
	return a.VolumeSerialNumber == b.VolumeSerialNumber && a.FileIndexHigh == b.FileIndexHigh && a.FileIndexLow == b.FileIndexLow && a.CreationTime == b.CreationTime
}
func openPin(path, expected string, security *windows.SECURITY_DESCRIPTOR) (*nativePin, error) {
	pointer, err := windows.UTF16PtrFromString(path)
	if err != nil {
		return nil, err
	}
	// Attribute/control-only opens do not participate in read/write/delete
	// sharing enforcement. FILE_LIST_DIRECTORY is directory data-read access,
	// so omitting FILE_SHARE_DELETE actually excludes rename/delete opens.
	// This does not exclude child additions or replace the protected ACL policy.
	handle, err := windows.CreateFile(pointer, windows.FILE_LIST_DIRECTORY|windows.FILE_READ_ATTRIBUTES|windows.READ_CONTROL, windows.FILE_SHARE_READ|windows.FILE_SHARE_WRITE, nil, windows.OPEN_EXISTING, windows.FILE_FLAG_BACKUP_SEMANTICS|windows.FILE_FLAG_OPEN_REPARSE_POINT, 0)
	if err != nil {
		return nil, err
	}
	pin := &nativePin{handle: handle, security: security}
	ok := false
	defer func() {
		if !ok {
			windows.CloseHandle(handle)
		}
	}()
	if err = windows.GetFileInformationByHandle(handle, &pin.info); err != nil {
		return nil, err
	}
	pin.name, err = nativeName(handle)
	if err != nil {
		return nil, err
	}
	if expected != "" && pin.name != expected {
		return nil, errors.New("physical child differs from held parent")
	}
	if pin.info.FileAttributes&(windows.FILE_ATTRIBUTE_DIRECTORY|windows.FILE_ATTRIBUTE_REPARSE_POINT) != windows.FILE_ATTRIBUTE_DIRECTORY {
		return nil, errors.New("plain physical directory required")
	}
	if security != nil {
		if err = checkSecurity(handle, security); err != nil {
			return nil, err
		}
	}
	ok = true
	return pin, nil
}
func (pin *nativePin) check() error {
	var current windows.ByHandleFileInformation
	if err := windows.GetFileInformationByHandle(pin.handle, &current); err != nil {
		return err
	}
	name, err := nativeName(pin.handle)
	if err != nil || name != pin.name || !sameIdentity(pin.info, current) || current.FileAttributes&(windows.FILE_ATTRIBUTE_DIRECTORY|windows.FILE_ATTRIBUTE_REPARSE_POINT) != windows.FILE_ATTRIBUTE_DIRECTORY {
		return errors.New("held physical directory changed")
	}
	if pin.ancestor != nil {
		if err = checkAncestor(pin.handle, pin.ancestor); err != nil {
			return err
		}
	}
	if pin.security != nil {
		if err = checkSecurity(pin.handle, pin.security); err != nil {
			return err
		}
	}
	if pin.dos != "" {
		probe, err := openPin(pin.dos, pin.name, pin.security)
		if err != nil {
			return err
		}
		defer windows.CloseHandle(probe.handle)
		if !sameIdentity(pin.info, probe.info) {
			return errors.New("fixed DOS directory resolves to another identity")
		}
	}
	return nil
}

// Open accepts only the actual machine ProgramData/SKAGER/UpdateBroker layout.
// Provisioning must create every protected existing node with its final SD,
// before exposing it to clients. Open never repairs, adopts or resets ACLs.
func Open(root string) (result *Store, resultErr error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	broker, _, _, err := windows.LookupSID("", BrokerAccount)
	if err != nil {
		return nil, ErrPrincipal
	}
	if !strings.HasPrefix(broker.String(), "S-1-5-80-") {
		return nil, ErrPrincipal
	}
	if err = currentBroker(broker); err != nil {
		return nil, err
	}
	programData, err := windows.KnownFolderPath(windows.FOLDERID_ProgramData, windows.KF_FLAG_DEFAULT)
	if err != nil {
		return nil, err
	}
	wanted := filepath.Join(programData, "SKAGER", "UpdateBroker")
	if !strings.EqualFold(root, wanted) || filepath.Clean(root) != root || len(root) < 4 || !((root[0] >= 'A' && root[0] <= 'Z') || (root[0] >= 'a' && root[0] <= 'z')) || root[1] != ':' || root[2] != '\\' || strings.Contains(root[3:], ":") {
		return nil, errors.New("only fixed actual ProgramData broker root accepted")
	}
	installer, _, _, err := windows.LookupSID("", `NT SERVICE\TrustedInstaller`)
	if err != nil {
		return nil, errors.New("trusted machine ancestor owner unavailable")
	}
	trusted := map[string]bool{"S-1-5-18": true, "S-1-5-32-544": true, installer.String(): true, broker.String(): true}
	ancestor := &ancestorPolicy{trusted: trusted}
	n := &nativeStore{broker: broker, rootDOS: wanted, protected: map[string]*nativePin{}}
	ok := false
	defer func() {
		if !ok {
			resultErr = errors.Join(resultErr, n.close())
		}
	}()
	drive, err := openPin(wanted[:3], "", nil)
	if err != nil {
		return nil, err
	}
	n.pins = append(n.pins, drive)
	drive.ancestor = ancestor
	if err = drive.check(); err != nil {
		return nil, err
	}
	parts, err := physicalParts(drive.name)
	if err != nil {
		return nil, err
	}
	walk := strings.Join(parts[:3], `\`)
	volume, err := openPin(physical(walk), walk, nil)
	if err != nil {
		return nil, err
	}
	n.pins = append(n.pins, volume)
	volume.ancestor = ancestor
	if err = volume.check(); err != nil {
		return nil, err
	}
	var serial, component, flags uint32
	fs := make([]uint16, 32)
	if err = windows.GetVolumeInformationByHandle(volume.handle, nil, 0, &serial, &component, &flags, &fs[0], uint32(len(fs))); err != nil || windows.UTF16ToString(fs) != "NTFS" || serial != volume.info.VolumeSerialNumber {
		return nil, errors.New("broker root must be native NTFS")
	}
	requested := strings.Split(wanted[3:], `\`)
	all := append(append([]string{}, parts[3:]...), requested...)
	sd, err := descriptor(broker, nil, true)
	if err != nil {
		return nil, err
	}
	for i, part := range all {
		if part == "" || part == "." || part == ".." || strings.HasSuffix(part, ".") || strings.HasSuffix(part, " ") {
			return nil, errors.New("ambiguous broker root ancestry")
		}
		walk += `\` + part
		var expected *windows.SECURITY_DESCRIPTOR
		if i == len(all)-1 {
			expected = sd
		}
		pin, err := openPin(physical(walk), walk, expected)
		if err != nil {
			return nil, err
		}
		n.pins = append(n.pins, pin)
		if expected == nil {
			pin.ancestor = ancestor
			// The fixed SKAGER parent is provisioned, not a general ProgramData child.
			if i == len(all)-2 {
				pin.ancestor = &ancestorPolicy{trusted: trusted, forbidChildAdds: true}
			}
			if err = pin.check(); err != nil {
				return nil, err
			}
		}
		if pin.info.VolumeSerialNumber != serial {
			return nil, errors.New("broker ancestry crossed volumes")
		}
		n.root = pin
	}
	n.root.dos = wanted
	n.protected[""] = n.root
	if err = decommissionAbsent(n.root); err != nil {
		return nil, err
	}
	lockSD, err := descriptor(broker, nil, false)
	if err != nil {
		return nil, err
	}
	n.lock, err = acquireStoreLock(n.root, lockSD)
	if err != nil {
		return nil, err
	}
	n.candidates, err = n.existingChild(n.root, "candidates", sd)
	if err != nil {
		return nil, err
	}
	n.registrations, err = n.existingChild(n.root, "registrations", sd)
	if err != nil {
		return nil, err
	}
	n.users, err = n.existingChild(n.root, "users", sd)
	if err != nil {
		return nil, err
	}
	if err = n.check(); err != nil {
		return nil, err
	}
	if err = n.reconcileAcquisitions(); err != nil {
		return nil, err
	}
	ok = true
	return &Store{backend: n}, nil
}
func (n *nativeStore) existingChild(parent *nativePin, component string, sd *windows.SECURITY_DESCRIPTOR) (*nativePin, error) {
	name := parent.name + `\` + component
	if old, ok := n.protected[name]; ok {
		return old, nil
	}
	pin, err := openPin(physical(name), name, sd)
	if err != nil {
		return nil, err
	}
	if pin.info.VolumeSerialNumber != n.root.info.VolumeSerialNumber {
		windows.CloseHandle(pin.handle)
		return nil, errors.New("protected child crossed volumes")
	}
	n.pins = append(n.pins, pin)
	n.protected[name] = pin
	return pin, nil
}
func (n *nativeStore) check() (err error) {
	n.mu.Lock()
	defer n.mu.Unlock()
	if n.closed {
		return ErrClosed
	}
	if n.failed {
		return ErrPoisoned
	}
	defer func() {
		if err != nil && !errors.Is(err, ErrDecommissioned) {
			n.failed = true
		}
	}()
	if n.decommissioned {
		return ErrDecommissioned
	}
	if err = decommissionAbsent(n.root); err != nil {
		n.decommissioned = true
		return err
	}
	return n.checkCustodyLocked()
}

// Caller holds n.mu; only the original terminal writer bypasses absence.
func (n *nativeStore) checkCustodyLocked() (err error) {
	if n.closed {
		return ErrClosed
	}
	if n.failed {
		return ErrPoisoned
	}
	if err = currentBroker(n.broker); err != nil {
		return err
	}
	for _, pin := range n.pins {
		if err = pin.check(); err != nil {
			return err
		}
	}
	if n.lock == nil {
		return errors.New("native store lifetime lock unavailable")
	}
	return n.lock.check()
}

// candidateCapacity counts all durable entries, including abandoned/foreign
// reservations. The protected directory and serialized Begin exclude ordinary
// clients from adding children between this bound and CREATE_NEW. No eviction.
func (n *nativeStore) candidateCapacity() (resultErr error) {
	if err := n.candidates.check(); err != nil {
		return err
	}
	file, err := os.Open(physical(n.candidates.name))
	if err != nil {
		return err
	}
	defer func() { resultErr = errors.Join(resultErr, file.Close()) }()
	var info windows.ByHandleFileInformation
	handle := windows.Handle(file.Fd())
	if err = windows.GetFileInformationByHandle(handle, &info); err != nil {
		return err
	}
	name, err := nativeName(handle)
	if err != nil || name != n.candidates.name || !sameIdentity(info, n.candidates.info) || info.FileAttributes&(windows.FILE_ATTRIBUTE_DIRECTORY|windows.FILE_ATTRIBUTE_REPARSE_POINT) != windows.FILE_ATTRIBUTE_DIRECTORY {
		return errors.New("candidate enumeration identity changed")
	}
	entries, err := file.ReadDir(MaxCandidates + 1)
	if err != nil && !errors.Is(err, io.EOF) {
		return err
	}
	if len(entries) >= MaxCandidates {
		return errors.New("durable candidate capacity exhausted; explicit recovery required")
	}
	for _, entry := range entries {
		if !safeComponent(entry.Name()) {
			return errors.New("ambiguous candidate namespace entry")
		}
		pointer, err := windows.UTF16PtrFromString(physical(n.candidates.name + `\` + entry.Name()))
		if err != nil {
			return err
		}
		attributes, err := windows.GetFileAttributes(pointer)
		if err != nil || attributes&windows.FILE_ATTRIBUTE_REPARSE_POINT != 0 {
			return errors.New("candidate namespace entry uncertain or reparse")
		}
	}
	return n.candidates.check()
}

func (n *nativeStore) begin(stage, client string) (candidateBackend, error) {
	n.beginMu.Lock()
	defer n.beginMu.Unlock()
	if err := n.check(); err != nil {
		return nil, err
	}
	sid, err := clientSID(client, n.broker)
	if err != nil {
		return nil, err
	}
	directorySD, err := descriptor(n.broker, sid, true)
	if err != nil {
		return nil, err
	}
	fileSD, err := descriptor(n.broker, sid, false)
	if err != nil {
		return nil, err
	}
	if err := n.candidateCapacity(); err != nil {
		return nil, err
	}
	c := &nativeCandidate{store: n, stageID: stage, client: client, directories: map[string]*nativePin{}, securityDir: directorySD, securityFile: fileSD}
	ok := false
	defer func() {
		if !ok {
			c.close()
		}
	}()
	pin, err := c.newDirectory(n.candidates, stage)
	if err != nil {
		return nil, err
	}
	c.directories[""] = pin
	for _, name := range []string{"app", "docs"} {
		child, err := c.newDirectory(pin, name)
		if err != nil {
			return nil, err
		}
		c.directories[name] = child
	}
	if err = c.check(); err != nil {
		return nil, err
	}
	ok = true
	return c, nil
}
func (c *nativeCandidate) newDirectory(parent *nativePin, component string) (*nativePin, error) {
	if !safeComponent(component) {
		return nil, errors.New("unsafe directory component")
	}
	if err := parent.check(); err != nil {
		return nil, err
	}
	name := parent.name + `\` + component
	pointer, err := windows.UTF16PtrFromString(physical(name))
	if err != nil {
		return nil, err
	}
	attributes := windows.SecurityAttributes{Length: uint32(unsafe.Sizeof(windows.SecurityAttributes{})), SecurityDescriptor: c.securityDir}
	if err = windows.CreateDirectory(pointer, &attributes); err != nil {
		return nil, err
	} // Never adopt a pre-existing stage or child.
	pin, err := openPin(physical(name), name, c.securityDir)
	if err != nil {
		return nil, err
	}
	c.pins = append(c.pins, pin)
	if pin.info.VolumeSerialNumber != parent.info.VolumeSerialNumber {
		return nil, errors.New("new directory crossed volumes")
	}
	return pin, nil
}
func (c *nativeCandidate) check() error {
	if c.closed {
		return ErrClosed
	}
	if err := c.store.check(); err != nil {
		return err
	}
	for _, pin := range c.pins {
		if err := pin.check(); err != nil {
			return err
		}
	}
	return nil
}
func (c *nativeCandidate) create(relative string) (candidateFile, error) {
	if err := c.check(); err != nil {
		return nil, err
	}
	if relative != ReceiptFilename && !baselineEvidenceName(relative) {
		if err := memberPath(relative); err != nil {
			return nil, err
		}
	}
	parts := strings.Split(relative, "/")
	parent := c.directories[""]
	for i, part := range parts[:len(parts)-1] {
		key := strings.Join(parts[:i+1], "/")
		next, ok := c.directories[key]
		if !ok {
			var err error
			next, err = c.newDirectory(parent, part)
			if err != nil {
				return nil, err
			}
			c.directories[key] = next
		}
		parent = next
	}
	name := parent.name + `\` + parts[len(parts)-1]
	pointer, err := windows.UTF16PtrFromString(physical(name))
	if err != nil {
		return nil, err
	}
	attributes := windows.SecurityAttributes{Length: uint32(unsafe.Sizeof(windows.SecurityAttributes{})), SecurityDescriptor: c.securityFile}
	handle, err := windows.CreateFile(pointer, windows.GENERIC_READ|windows.GENERIC_WRITE|windows.READ_CONTROL, windows.FILE_SHARE_READ, &attributes, windows.CREATE_NEW, windows.FILE_FLAG_OPEN_REPARSE_POINT, 0)
	if err != nil {
		return nil, err
	}
	file := &nativeFile{File: os.NewFile(uintptr(handle), physical(name)), pin: &nativePin{handle: handle, name: name, security: c.securityFile}, parent: parent}
	if err = windows.GetFileInformationByHandle(handle, &file.pin.info); err != nil {
		file.Close()
		return nil, err
	}
	if err = file.check(0); err != nil {
		file.Close()
		return nil, err
	}
	return file, nil
}
func (f *nativeFile) check(length int64) error {
	if err := f.parent.check(); err != nil {
		return err
	}
	var info windows.ByHandleFileInformation
	if err := windows.GetFileInformationByHandle(f.pin.handle, &info); err != nil {
		return err
	}
	name, err := nativeName(f.pin.handle)
	if err != nil || name != f.pin.name || !sameIdentity(f.pin.info, info) || info.FileAttributes&(windows.FILE_ATTRIBUTE_DIRECTORY|windows.FILE_ATTRIBUTE_REPARSE_POINT) != 0 || info.NumberOfLinks != 1 || int64(uint64(info.FileSizeHigh)<<32|uint64(info.FileSizeLow)) != length {
		return errors.New("held staged file identity/type/link/length changed")
	}
	if info.VolumeSerialNumber != f.parent.info.VolumeSerialNumber || !strings.HasPrefix(name, f.parent.name+`\`) {
		return errors.New("held file escaped its physical parent")
	}
	return checkSecurity(f.pin.handle, f.pin.security)
}
func (n *nativeStore) registration(client string) ([]byte, error) {
	if err := n.check(); err != nil {
		return nil, err
	}
	sid, err := clientSID(client, n.broker)
	if err != nil {
		return nil, err
	}
	sd, err := descriptor(n.broker, nil, false)
	if err != nil {
		return nil, err
	}
	data, err := readCompletedRegistration(n.registrations, sid.String(), sd)
	if err != nil {
		return nil, err
	}
	if err = n.check(); err != nil {
		return nil, err
	}
	return data, nil
}

func (n *nativeStore) verifierDirectories(client, channel string) (string, string, error) {
	if err := n.check(); err != nil {
		return "", "", err
	}
	sid, err := clientSID(client, n.broker)
	if err != nil {
		return "", "", err
	}
	n.mu.Lock()
	defer n.mu.Unlock()
	if n.closed || n.failed {
		return "", "", ErrPoisoned
	}
	sd, err := descriptor(n.broker, nil, true)
	if err != nil {
		return "", "", err
	}
	users, err := n.existingChild(n.root, "users", sd)
	if err != nil {
		return "", "", err
	}
	user, err := n.existingChild(users, sid.String(), sd)
	if err != nil {
		return "", "", err
	}
	trust, err := n.existingChild(user, "trust", sd)
	if err != nil {
		return "", "", err
	}
	state, err := n.existingChild(trust, channel, sd)
	if err != nil {
		return "", "", err
	}
	artifacts, err := n.existingChild(user, "artifacts", sd)
	if err != nil {
		return "", "", err
	}
	state.dos = filepath.Join(n.rootDOS, "users", sid.String(), "trust", channel)
	artifacts.dos = filepath.Join(n.rootDOS, "users", sid.String(), "artifacts")
	if err = state.check(); err != nil {
		n.failed = true
		return "", "", err
	}
	if err = artifacts.check(); err != nil {
		n.failed = true
		return "", "", err
	}
	return state.dos, artifacts.dos, nil
}
func (c *nativeCandidate) close() error {
	if c.closed {
		return nil
	}
	c.closed = true
	var result error
	for i := len(c.pins) - 1; i >= 0; i-- {
		result = errors.Join(result, windows.CloseHandle(c.pins[i].handle))
	}
	c.pins = nil
	return result
}
func (n *nativeStore) close() error {
	n.mu.Lock()
	defer n.mu.Unlock()
	if n.closed {
		return nil
	}
	n.closed = true
	var result error
	for i := len(n.pins) - 1; i >= 0; i-- {
		result = errors.Join(result, windows.CloseHandle(n.pins[i].handle))
	}
	n.pins = nil
	// Release synchronization last, after all children (Store.Close) and every
	// root/ancestor handle have been attempted, including error paths.
	if n.lock != nil {
		result = errors.Join(result, windows.CloseHandle(n.lock.handle))
		n.lock = nil
	}
	return result
}

// candidateInventory enumerates each retained physical directory, refusing any
// payload/evidence entry not created by this candidate. The separately owned
// private acquisition lease is the sole fixed namespace exception.
func (c *nativeCandidate) candidateInventory(names []string, acquisition bool) error {
	if err := c.check(); err != nil {
		return err
	}
	expected := map[string]map[string]bool{}
	for name := range c.directories {
		expected[name] = map[string]bool{}
	}
	if expected[""] == nil {
		return errors.New("candidate root unavailable")
	}
	for name := range c.directories {
		if name == "" {
			continue
		}
		parent, child := splitBaselineName(name)
		if expected[parent] == nil {
			return errors.New("candidate directory parent absent")
		}
		expected[parent][child] = true
	}
	for _, name := range names {
		parent, child := splitBaselineName(name)
		if expected[parent] == nil || expected[parent][child] {
			return errors.New("candidate namespace collision")
		}
		expected[parent][child] = true
	}
	if acquisition {
		expected[""][acquisitionDirectory] = true
	}
	for name, wanted := range expected {
		entries, err := boundedEntries(c.directories[name], len(wanted)+1)
		if err != nil {
			return err
		}
		if len(entries) != len(wanted) {
			return errors.New("candidate namespace has missing or additional entries")
		}
		for _, entry := range entries {
			if !wanted[entry.Name()] {
				return errors.New("candidate namespace has alias or unknown entry")
			}
		}
	}
	return c.check()
}
