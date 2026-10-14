//go:build windows

package brokerenroll

import (
	"context"
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"errors"
	"path/filepath"
	"runtime"
	"sort"
	"strings"
	"unsafe"

	"golang.org/x/sys/windows"
)

type nativeInstalledMember struct {
	file                startupRestorationFile
	remove, pending     bool
	retirementName, sha string
}

type nativeInstalledUninstall struct {
	ctx                                               context.Context
	clientSID, generationDOS                          string
	baseline                                          ApprovedClientBaseline
	files                                             *nativeEnrollmentFiles
	root                                              *enrollmentPin
	linkRecovery                                      *enrollmentPin
	state                                             startupRestorationFile
	stateSHA                                          string
	members                                           []*nativeInstalledMember
	approvedTargets                                   map[string]bool
	registry                                          *installedRegistryOwner
	lock                                              windows.Handle
	cleanupAttempted, stateRetired, completed, closed bool
	closeErr                                          error
}

func openInstalledUninstallNative(ctx context.Context, authority maintenanceAuthority, artifactDirectory string) (result installedUninstallBackend, resultErr error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if err := maintenanceOperatorNative(authority.client.ClientSID); err != nil {
		return nil, err
	}
	n := &nativeInstalledUninstall{ctx: ctx, clientSID: authority.client.ClientSID, files: &nativeEnrollmentFiles{}, approvedTargets: map[string]bool{}}
	defer func() {
		if resultErr != nil {
			resultErr = errors.Join(resultErr, n.close())
		}
	}()
	var err error
	n.baseline, err = openClientBaselineInputNative(ctx, artifactDirectory)
	if err != nil {
		return nil, err
	}
	inventory, err := collectInstalledInventory(ctx, authority, n.baseline)
	if err != nil {
		return nil, err
	}
	local, err := windows.KnownFolderPath(windows.FOLDERID_LocalAppData, 0)
	if err != nil {
		return nil, err
	}
	parent, err := startupHoldAncestry(n.files, local, n.clientSID)
	if err != nil {
		return nil, err
	}
	n.root, err = maintenanceHoldChild(n.files, parent, installedIntegrationDirectory, nil, n.clientSID)
	if err != nil {
		return nil, err
	}
	// Reuse the original Lifecycle exclusion file. Never CREATE/adopt a missing
	// lock or rely on a JSON declaration that a different process owns it.
	lockName, _ := windows.UTF16PtrFromString(enrollmentPhysical(n.root.physical + `\transaction.lock`))
	n.lock, err = windows.CreateFile(lockName, windows.GENERIC_READ|windows.GENERIC_WRITE|windows.READ_CONTROL, 0, nil, windows.OPEN_EXISTING, windows.FILE_FLAG_OPEN_REPARSE_POINT, 0)
	if err != nil {
		return nil, err
	}
	lockInfo, _, err := enrollmentInfo(n.lock)
	if err != nil || lockInfo.FileAttributes&(windows.FILE_ATTRIBUTE_DIRECTORY|windows.FILE_ATTRIBUTE_REPARSE_POINT) != 0 || lockInfo.NumberOfLinks != 1 {
		return nil, errors.Join(errors.New("original exclusive native Lifecycle lock required"), err)
	}
	for _, name := range []string{"transaction.json", "update-pending.json", "signed-update-armed.jsonl", "retired-state.json"} {
		if err := startupRestorationAbsent(n.root, name); err != nil {
			return nil, err
		}
	}
	owner, err := startupOpenFile(n.files, n.root, "owner.json", false, 4096)
	if err != nil {
		return nil, err
	}
	raw, err := readClientRequestPin(ctx, owner)
	if err != nil || installedExactObject(raw, []string{"owner"}, nil) != nil {
		return nil, errors.Join(errors.New("original exact integration owner comparison required"), err)
	}
	var ownerRecord struct {
		Owner string `json:"owner"`
	}
	if json.Unmarshal(raw, &ownerRecord) != nil || ownerRecord.Owner != installedIntegrationOwner {
		return nil, errors.New("unknown integration owner")
	}
	n.state, err = openStartupRestorationFile(n.files, n.root, "state.json")
	if err != nil {
		return nil, err
	}
	raw, err = readClientRequestPin(ctx, n.state.pin)
	if err != nil {
		return nil, err
	}
	state, err := parseInstalledState(raw, authority.client)
	if err != nil {
		return nil, err
	}
	n.stateSHA = digest(raw)
	generationParent, err := maintenanceHoldChild(n.files, n.root, "generations", nil, n.clientSID)
	if err != nil {
		return nil, err
	}
	generation, err := maintenanceHoldChild(n.files, generationParent, state.Current, nil, n.clientSID)
	if err != nil {
		return nil, err
	}
	n.generationDOS = generation.dos
	// The stock executable is retained read-only throughout SCM and teardown.
	stockParent, err := startupHoldAncestry(n.files, filepath.Dir(authority.client.Stock.Path), n.clientSID)
	if err != nil {
		return nil, err
	}
	stock, err := startupOpenFile(n.files, stockParent, filepath.Base(authority.client.Stock.Path), false, maxBrokerBytes)
	if err != nil {
		return nil, err
	}
	if err := startupHashPin(ctx, stock, authority.client.Stock.ExecutableSHA256); err != nil {
		return nil, err
	}
	names := make([]string, 0, len(inventory))
	for name := range inventory {
		names = append(names, name)
	}
	sort.Strings(names)
	directories := map[string]*enrollmentPin{"": generation}
	for _, name := range names {
		member := inventory[name]
		parent, base, err := installedMemberParent(n.files, generation, directories, name, n.clientSID)
		if errors.Is(err, windows.ERROR_FILE_NOT_FOUND) || errors.Is(err, windows.ERROR_PATH_NOT_FOUND) {
			continue
		}
		if err != nil {
			return nil, err
		}
		file, err := openInstalledMember(parent, base, member.size, n.clientSID)
		if errors.Is(err, windows.ERROR_FILE_NOT_FOUND) {
			continue
		}
		if err != nil {
			return nil, err
		}
		owned := &nativeInstalledMember{file: file}
		n.members = append(n.members, owned)
		// A different bounded length or digest is a preserved modified member.
		if file.pin.length == member.size {
			actual, err := installedMemberDigest(ctx, file.pin)
			if err != nil {
				return nil, err
			}
			if actual == member.sha && file.pin.info.FileAttributes&windows.FILE_ATTRIBUTE_READONLY == 0 {
				owned.remove = true
				n.approvedTargets[strings.ToLower(file.dos)] = true
			}
		}
		if err := ctx.Err(); err != nil {
			return nil, err
		}
	}
	// Native system ShellLink parsing never resolves/executes a target. The
	// restored normal link belongs to the separate original startup owner.
	programs, err := windows.KnownFolderPath(windows.FOLDERID_Programs, 0)
	if err != nil {
		return nil, err
	}
	menu, err := startupHoldAncestry(n.files, filepath.Join(programs, "SKAGER"), n.clientSID)
	if err != nil {
		return nil, err
	}
	n.linkRecovery, err = maintenanceHoldChild(n.files, parent, startupRecoveryDirectory, nil, n.clientSID)
	if err != nil {
		return nil, err
	}
	for _, spec := range []struct{ name, target, args, work string }{
		{"OpenCPN Legacy.lnk", "app/opencpn.exe", "--legacy", "app"},
		{"Maintain Skager.lnk", "Maintain.exe", "", ""},
	} {
		file, err := openInstalledMember(menu, spec.name, maxStartupShortcutBytes, n.clientSID)
		if errors.Is(err, windows.ERROR_FILE_NOT_FOUND) {
			continue
		}
		if err != nil {
			return nil, err
		}
		member := &nativeInstalledMember{file: file}
		n.members = append(n.members, member)
		raw, err := readClientRequestPin(ctx, file.pin)
		if err != nil {
			return nil, err
		}
		target, args, working, err := startupLinkFields(raw)
		if err != nil {
			continue
		} // Malformed/custom link bytes are preserved.
		expectedTarget := filepath.Join(generation.dos, filepath.FromSlash(spec.target))
		expectedWorking := generation.dos
		if spec.work != "" {
			expectedWorking = filepath.Join(expectedWorking, spec.work)
		}
		member.remove = (spec.target == "Maintain.exe" || n.approvedTargets[strings.ToLower(expectedTarget)]) && strings.EqualFold(target, expectedTarget) && args == spec.args && strings.EqualFold(working, expectedWorking)
		if member.remove {
			member.retirementName = "retired-installed-" + spec.name
			member.sha = digest(raw)
			if err := startupRestorationAbsent(n.linkRecovery, member.retirementName); err != nil {
				return nil, err
			}
		}
	}
	n.registry, err = openInstalledRegistry(ctx, n.clientSID, n.root.dos, generation.dos, authority.client.Installed.Version)
	if err != nil {
		return nil, err
	}
	if err := n.check(ctx); err != nil {
		return nil, err
	}
	return n, nil
}

func installedMemberDigest(ctx context.Context, pin *enrollmentPin) (string, error) {
	if err := pin.check(); err != nil {
		return "", err
	}
	hash := sha256.New()
	buffer := make([]byte, 64<<10)
	for offset := int64(0); offset < pin.length; {
		if err := ctx.Err(); err != nil {
			return "", err
		}
		count := int64(len(buffer))
		if pin.length-offset < count {
			count = pin.length - offset
		}
		n, err := readClientBaselinePinAt(pin, buffer[:int(count)], offset)
		if err != nil || n != int(count) {
			return "", errors.Join(errors.New("original installed member read uncertain"), err)
		}
		_, _ = hash.Write(buffer[:n])
		offset += int64(n)
	}
	if err := errors.Join(ctx.Err(), pin.check()); err != nil {
		return "", err
	}
	return hex.EncodeToString(hash.Sum(nil)), nil
}

func installedMemberParent(files *nativeEnrollmentFiles, generation *enrollmentPin, cache map[string]*enrollmentPin, name, sid string) (*enrollmentPin, string, error) {
	parts := strings.Split(name, "/")
	parent := generation
	relative := ""
	for _, component := range parts[:len(parts)-1] {
		if relative != "" {
			relative += "/"
		}
		relative += component
		if cached := cache[relative]; cached != nil {
			parent = cached
			continue
		}
		child, err := maintenanceHoldChild(files, parent, component, nil, sid)
		if err != nil {
			return nil, "", err
		}
		cache[relative] = child
		parent = child
	}
	return parent, parts[len(parts)-1], nil
}

func openInstalledMember(parent *enrollmentPin, name string, bound int64, sid string) (startupRestorationFile, error) {
	dos := filepath.Join(parent.dos, name)
	physical := parent.physical + `\` + name
	pointer, err := windows.UTF16PtrFromString(enrollmentPhysical(physical))
	if err != nil {
		return startupRestorationFile{}, err
	}
	h, err := windows.CreateFile(pointer, windows.GENERIC_READ|windows.READ_CONTROL|windows.DELETE, windows.FILE_SHARE_READ, nil, windows.OPEN_EXISTING, windows.FILE_FLAG_OPEN_REPARSE_POINT, 0)
	if err != nil {
		return startupRestorationFile{}, err
	}
	pin := &enrollmentPin{handle: h, physical: physical, file: true, broker: sid}
	file := startupRestorationFile{pin, dos}
	failed := func(err error) (startupRestorationFile, error) {
		return startupRestorationFile{}, errors.Join(err, windows.CloseHandle(h))
	}
	pin.info, pin.id, err = enrollmentInfo(h)
	if err != nil {
		return failed(err)
	}
	pin.length = int64(uint64(pin.info.FileSizeHigh)<<32 | uint64(pin.info.FileSizeLow))
	// Modified files are retained even when their current size differs. Keep a
	// global read bound; authenticated member length is checked independently.
	if pin.length < 0 || pin.length > maxClientBaselineBytes || bound < 0 {
		return failed(errors.New("bounded original installed member required"))
	}
	if err := enrollmentSecurity(h, nil, false, sid); err != nil {
		return failed(err)
	}
	pin.descriptor, err = windows.GetSecurityInfo(h, windows.SE_FILE_OBJECT, windows.OWNER_SECURITY_INFORMATION|windows.DACL_SECURITY_INFORMATION)
	if err != nil {
		return failed(err)
	}
	if err := file.checkAlias(); err != nil {
		return failed(err)
	}
	return file, nil
}

func (n *nativeInstalledUninstall) ownsStartupLink(target, arguments, working string) bool {
	if n == nil || n.closed || !n.approvedTargets[strings.ToLower(target)] || !strings.EqualFold(working, filepath.Join(n.generationDOS, "app")) {
		return false
	}
	return (strings.EqualFold(target, filepath.Join(n.generationDOS, "app", "opencpn.exe")) || strings.EqualFold(target, filepath.Join(n.generationDOS, "app", "skager-start.exe"))) && (arguments == "--xnav" || arguments == "--legacy" || arguments == "--safe-mode")
}

func (m *nativeInstalledMember) check() error {
	if !m.pending {
		return m.file.checkAlias()
	}
	p := m.file.pin
	info, id, err := enrollmentInfo(p.handle)
	if err != nil || id != p.id || info.CreationTime != p.info.CreationTime || int64(uint64(info.FileSizeHigh)<<32|uint64(info.FileSizeLow)) != p.length {
		return errors.Join(errors.New("original delete-pending member changed"), err)
	}
	if err := enrollmentSecurity(p.handle, p.descriptor, false, p.broker); err != nil {
		return err
	}
	var standard struct {
		AllocationSize, EndOfFile int64
		NumberOfLinks             uint32
		DeletePending, Directory  byte
		Padding                   [2]byte
	}
	if err := windows.GetFileInformationByHandleEx(p.handle, windows.FileStandardInfo, (*byte)(unsafe.Pointer(&standard)), uint32(unsafe.Sizeof(standard))); err != nil {
		return err
	}
	if standard.DeletePending == 0 || standard.Directory != 0 {
		return errors.New("original member deletion disposition missing")
	}
	return nil
}

func (m *nativeInstalledMember) dispose(ctx context.Context) error {
	if !m.remove || m.pending || m.retirementName != "" || ctx == nil {
		return errors.New("fresh original owned member disposition required")
	}
	if err := errors.Join(ctx.Err(), m.check()); err != nil {
		return err
	}
	var status windows.IO_STATUS_BLOCK
	remove := byte(1)
	if err := windows.NtSetInformationFile(m.file.pin.handle, &status, &remove, 1, windows.FileDispositionInformation); err != nil {
		return err
	}
	m.pending = true
	return errors.Join(ctx.Err(), m.check())
}

func (n *nativeInstalledUninstall) check(ctx context.Context) error {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if n == nil || n.closed || ctx == nil || n.ctx == nil {
		return errors.New("live original installed custody required")
	}
	if err := errors.Join(ctx.Err(), n.ctx.Err(), maintenanceOperatorNative(n.clientSID)); err != nil {
		return err
	}
	if err := errors.Join(n.baseline.Validate(), n.files.Check(), n.state.checkAlias()); err != nil {
		return err
	}
	// During cleanup each current member is checked immediately around its own
	// operation. All original handles exclude write/delete throughout; avoid an
	// O(N squared) rescan while preserving full checks at phase boundaries.
	if !n.cleanupAttempted || n.completed {
		for _, member := range n.members {
			if err := member.check(); err != nil {
				return err
			}
		}
	}
	if n.registry != nil {
		if err := n.registry.check(ctx); err != nil {
			return err
		}
	}
	for _, name := range []string{"transaction.json", "update-pending.json", "signed-update-armed.jsonl"} {
		if err := startupRestorationAbsent(n.root, name); err != nil {
			return err
		}
	}
	stateAbsent := "retired-state.json"
	if n.stateRetired {
		stateAbsent = "state.json"
	}
	if err := startupRestorationAbsent(n.root, stateAbsent); err != nil {
		return err
	}
	return errors.Join(ctx.Err(), n.ctx.Err())
}

func (n *nativeInstalledUninstall) cleanup(ctx context.Context, guard func(context.Context) error) error {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if n.cleanupAttempted || guard == nil {
		return errors.New("single original installed cleanup attempt required")
	}
	n.cleanupAttempted = true
	if err := guard(ctx); err != nil {
		return err
	}
	for _, member := range n.members {
		if !member.remove {
			continue
		}
		if err := guard(ctx); err != nil {
			return err
		}
		if err := member.check(); err != nil {
			return err
		}
		if member.retirementName != "" {
			if err := startupRenameRestorationFile(ctx, &member.file, n.linkRecovery, member.retirementName, member.sha); err != nil {
				return err
			}
			if err := guard(ctx); err != nil {
				return err
			}
			continue
		}
		if err := member.dispose(ctx); err != nil {
			return err
		}
		if err := guard(ctx); err != nil {
			return err
		}
	}
	if err := guard(ctx); err != nil {
		return err
	}
	if err := startupRenameRestorationFile(ctx, &n.state, n.root, "retired-state.json", n.stateSHA); err != nil {
		return err
	}
	n.stateRetired = true
	if err := guard(ctx); err != nil {
		return err
	}
	if err := n.registry.remove(ctx, guard); err != nil {
		return err
	}
	if err := guard(ctx); err != nil {
		return err
	}
	n.completed = true
	return nil
}

func (n *nativeInstalledUninstall) close() error {
	if n == nil {
		return nil
	}
	if n.closed {
		return n.closeErr
	}
	n.closed = true
	if n.registry != nil {
		n.closeErr = errors.Join(n.closeErr, n.registry.close())
	}
	for i := len(n.members) - 1; i >= 0; i-- {
		n.closeErr = errors.Join(n.closeErr, windows.CloseHandle(n.members[i].file.pin.handle))
		n.members[i].file.pin.handle = 0
	}
	if n.lock != 0 && n.lock != windows.InvalidHandle {
		n.closeErr = errors.Join(n.closeErr, windows.CloseHandle(n.lock))
		n.lock = 0
	}
	if n.files != nil {
		n.closeErr = errors.Join(n.closeErr, n.files.Close())
	}
	if n.baseline != nil {
		n.closeErr = errors.Join(n.closeErr, n.baseline.Close())
	}
	return n.closeErr
}
