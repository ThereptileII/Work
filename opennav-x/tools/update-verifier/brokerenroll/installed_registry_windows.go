//go:build windows

package brokerenroll

import (
	"bytes"
	"context"
	"encoding/binary"
	"errors"
	"fmt"
	"io"
	"strings"
	"syscall"
	"time"
	"unsafe"

	"golang.org/x/sys/windows"
	"golang.org/x/sys/windows/registry"
)

var installedCreateTransaction = windows.NewLazySystemDLL("ktmw32.dll").NewProc("CreateTransaction")
var installedCommitTransaction = windows.NewLazySystemDLL("ktmw32.dll").NewProc("CommitTransaction")
var installedTransactionInformation = windows.NewLazySystemDLL("ktmw32.dll").NewProc("GetTransactionInformation")
var installedOpenKeyTransacted = windows.NewLazySystemDLL("advapi32.dll").NewProc("RegOpenKeyTransactedW")
var installedDeleteKey = windows.NewLazySystemDLL("ntdll.dll").NewProc("NtDeleteKey")

const installedProductKey = `Software\Microsoft\Windows\CurrentVersion\Uninstall\OpenNavXAlpha1`
const installedRegistryOpenLink = 0x00000008 // REG_OPTION_OPEN_LINK

type installedRegistryPin struct {
	key              windows.Handle
	name, descriptor string
}
type installedRegistryOwner struct {
	parents                                            []installedRegistryPin
	original                                           installedRegistryPin
	transacted, transaction                            windows.Handle
	want                                               map[string][]byte
	removeAttempted, staged, committed, failed, closed bool
	closeErr                                           error
}

func installedRegistryValues(root, generation, version string) map[string][]byte {
	stringsToWrite := map[string]string{
		"DisplayName": "SKAGER", "DisplayVersion": version, "Publisher": "SKAGER project", "InstallLocation": root,
		"DisplayIcon":     generation + `\app\opencpn.exe`,
		"UninstallString": `"` + generation + `\Maintain.exe" /ACTION=Uninstall`,
		"ModifyPath":      `"` + generation + `\Maintain.exe"`, "OpenNavOwner": installedIntegrationOwner,
	}
	result := map[string][]byte{}
	for name, value := range stringsToWrite {
		utf16, _ := windows.UTF16FromString(value)
		raw := make([]byte, len(utf16)*2)
		for i, c := range utf16 {
			binary.LittleEndian.PutUint16(raw[i*2:], c)
		}
		result[name] = raw
	}
	return result
}

func openInstalledRegistry(ctx context.Context, sid, root, generation, version string) (*installedRegistryOwner, error) {
	return openInstalledRegistryAt(ctx, sid, installedProductKey, installedRegistryValues(root, generation, version))
}

// path is fixed in production; the private seam permits disposable HKU fixtures.
// Open each ancestor in the explicit x86 Lifecycle view and retain that original
// key. NULL-subkey transaction binding reopens THIS object, not its pathname.
func openInstalledRegistryAt(ctx context.Context, sid, path string, want map[string][]byte) (n *installedRegistryOwner, resultErr error) {
	if ctx == nil || ctx.Err() != nil || !canonicalClientSID(sid) {
		return nil, errors.New("live original installed registry identity required")
	}
	for _, procedure := range []*windows.LazyProc{installedCreateTransaction, installedCommitTransaction, installedTransactionInformation, installedOpenKeyTransacted, installedDeleteKey} {
		if err := procedure.Find(); err != nil {
			return nil, errors.Join(errors.New("required native registry transaction API unavailable"), err)
		}
	}
	n = &installedRegistryOwner{want: want}
	defer func() {
		if resultErr != nil {
			resultErr = errors.Join(resultErr, n.close())
		}
	}()
	parent := windows.Handle(windows.HKEY_USERS)
	parts := append([]string{sid}, strings.Split(path, `\`)...)
	for index, part := range parts {
		name, err := windows.UTF16PtrFromString(part)
		if err != nil {
			return n, err
		}
		rights := uint32(windows.KEY_READ | windows.READ_CONTROL | windows.KEY_WOW64_32KEY)
		if index == len(parts)-1 {
			rights |= windows.DELETE
		}
		var key windows.Handle
		if err := windows.RegOpenKeyEx(parent, name, installedRegistryOpenLink, rights, &key); err != nil {
			return n, err
		}
		pin := installedRegistryPin{key: key}
		if index == len(parts)-1 {
			n.original = pin
		} else {
			n.parents = append(n.parents, pin)
		}
		resolved, err := nativeRegistryName(key)
		if err != nil {
			return n, err
		}
		expected := `\REGISTRY\USER\` + strings.Join(parts[:index+1], `\`)
		if !strings.EqualFold(resolved, expected) {
			return n, errors.New("installed registry fixed ancestry alias differs")
		}
		sd, err := windows.GetSecurityInfo(key, windows.SE_REGISTRY_KEY, windows.OWNER_SECURITY_INFORMATION|windows.DACL_SECURITY_INFORMATION)
		if err != nil {
			return n, err
		}
		if err := installedRegistrySecurity(sd, sid); err != nil {
			return n, err
		}
		pin.name, pin.descriptor = resolved, sd.String()
		if index == len(parts)-1 {
			n.original = pin
		} else {
			n.parents[len(n.parents)-1] = pin
		}
		parent = key
	}
	budget := 90 * time.Second
	if deadline, ok := ctx.Deadline(); ok && time.Until(deadline) < budget {
		budget = time.Until(deadline)
	}
	if budget < time.Millisecond {
		return n, context.DeadlineExceeded
	}
	h, _, err := installedCreateTransaction.Call(0, 0, 1, 0, 0, uintptr(budget.Milliseconds()), 0) // TRANSACTION_DO_NOT_PROMOTE
	if windows.Handle(h) == windows.InvalidHandle || h == 0 {
		return n, err
	}
	n.transaction = windows.Handle(h)
	status, _, _ := installedOpenKeyTransacted.Call(uintptr(n.original.key), 0, 0, uintptr(windows.KEY_READ|windows.READ_CONTROL|windows.DELETE|windows.KEY_WOW64_32KEY), uintptr(unsafe.Pointer(&n.transacted)), uintptr(n.transaction), 0)
	if status != 0 {
		return n, syscall.Errno(status)
	}
	if err := n.check(ctx); err != nil {
		return n, err
	}
	return n, nil
}

// Registry rights have different bit meanings from file rights. In particular,
// KEY_NOTIFY is read observation, not FILE_WRITE_EA. Never apply file masks to
// these user-hive ancestors or silently repair their native descriptors.
func installedRegistrySecurity(sd *windows.SECURITY_DESCRIPTOR, clientSID string) error {
	if sd == nil {
		return errors.New("original registry descriptor required")
	}
	trusted := func(sid string) bool {
		return sid == clientSID || sid == "S-1-5-18" || sid == "S-1-5-32-544" || sid == enrollmentTrustedInstaller
	}
	owner, _, err := sd.Owner()
	if err != nil || owner == nil || !trusted(owner.String()) {
		return errors.New("unexpected installed registry owner")
	}
	control, _, err := sd.Control()
	if err != nil || control&windows.SE_DACL_PRESENT == 0 {
		return errors.New("installed registry DACL unavailable")
	}
	acl, _, err := sd.DACL()
	if err != nil || acl == nil {
		return errors.New("installed registry non-null DACL required")
	}
	mutation := uint32(windows.KEY_SET_VALUE | windows.KEY_CREATE_SUB_KEY | windows.KEY_CREATE_LINK | windows.DELETE | windows.WRITE_DAC | windows.WRITE_OWNER | windows.GENERIC_WRITE | windows.GENERIC_ALL)
	for i := uint32(0); i < uint32(acl.AceCount); i++ {
		var ace *windows.ACCESS_ALLOWED_ACE
		if err := windows.GetAce(acl, i, &ace); err != nil {
			return err
		}
		if ace.Header.AceFlags&windows.INHERIT_ONLY_ACE != 0 || ace.Header.AceType == windows.ACCESS_DENIED_ACE_TYPE {
			continue
		}
		if ace.Header.AceType != windows.ACCESS_ALLOWED_ACE_TYPE {
			return errors.New("unsupported installed registry ACE")
		}
		sid := (*windows.SID)(unsafe.Pointer(&ace.SidStart))
		if !sid.IsValid() || !trusted(sid.String()) && uint32(ace.Mask)&mutation != 0 {
			return errors.New("foreign installed registry mutation permission")
		}
	}
	return nil
}

func installedRegistryPinCheck(pin installedRegistryPin) error {
	name, err := nativeRegistryName(pin.key)
	if err != nil || name != pin.name {
		return errors.Join(errors.New("original registry key resolution changed"), err)
	}
	sd, err := windows.GetSecurityInfo(pin.key, windows.SE_REGISTRY_KEY, windows.OWNER_SECURITY_INFORMATION|windows.DACL_SECURITY_INFORMATION)
	if err != nil || sd == nil || sd.String() != pin.descriptor {
		return errors.Join(errors.New("original registry descriptor changed"), err)
	}
	return nil
}

func (n *installedRegistryOwner) outcome() (uint32, error) {
	if err := installedTransactionInformation.Find(); err != nil {
		return 0, err
	}
	var outcome uint32
	ok, _, err := installedTransactionInformation.Call(uintptr(n.transaction), uintptr(unsafe.Pointer(&outcome)), 0, 0, 0, 0, 0)
	if ok == 0 {
		return 0, err
	}
	return outcome, nil
}

func (n *installedRegistryOwner) check(ctx context.Context) (resultErr error) {
	if n == nil || n.closed || n.failed || ctx == nil || ctx.Err() != nil {
		return errors.New("original installed registry transaction revoked")
	}
	defer func() {
		if resultErr != nil {
			n.failed = true
		}
	}()
	for _, pin := range n.parents {
		if err := installedRegistryPinCheck(pin); err != nil {
			return err
		}
	}
	outcome, err := n.outcome()
	if err != nil {
		return err
	}
	if n.committed {
		if outcome != 2 {
			return errors.New("original installed registry commit outcome differs")
		}
		// Corroborate absence only. A replacement key is never adopted/deleted.
		leaf := strings.TrimPrefix(n.original.name, n.parents[len(n.parents)-1].name+`\`)
		name, _ := windows.UTF16PtrFromString(leaf)
		var key windows.Handle
		err := windows.RegOpenKeyEx(n.parents[len(n.parents)-1].key, name, installedRegistryOpenLink, windows.KEY_READ|windows.KEY_WOW64_32KEY, &key)
		if key != 0 {
			err = errors.Join(err, windows.RegCloseKey(key))
		}
		if !errors.Is(err, windows.ERROR_FILE_NOT_FOUND) {
			return errors.Join(errors.New("installed registry fixed name not absent after commit"), err)
		}
		return ctx.Err()
	}
	if outcome != 1 {
		return errors.New("installed registry transaction aborted or uncertain")
	}
	if err := installedRegistryPinCheck(n.original); err != nil {
		return err
	}
	if n.staged {
		// The transacted view has deleted its key. The original outside view must
		// still own the original key until Commit; native fixtures qualify this.
		return ctx.Err()
	}
	name, err := nativeRegistryName(n.transacted)
	if err != nil || name != n.original.name {
		return errors.Join(errors.New("transaction not bound to original key object"), err)
	}
	key := registry.Key(n.transacted)
	info, err := key.Stat()
	if err != nil {
		return err
	}
	if info.SubKeyCount != 0 || info.ValueCount != uint32(len(n.want)) || info.MaxValueLen > 32768 || info.MaxValueNameLen > 128 {
		return errors.New("installed registry unknown values/subkeys or bounds")
	}
	names, err := key.ReadValueNames(len(n.want) + 1)
	if err != nil && !errors.Is(err, io.EOF) || len(names) != len(n.want) {
		return errors.Join(errors.New("installed registry value enumeration changed"), err)
	}
	for _, name := range names {
		want, ok := n.want[name]
		if !ok {
			return errors.New("installed registry unknown or case-aliased value")
		}
		buffer := make([]byte, 32768)
		used, kind, err := key.GetValue(name, buffer)
		if err != nil || kind != registry.SZ || used != len(want) || !bytes.Equal(buffer[:used], want) {
			return errors.Join(errors.New("installed registry exact value bytes changed"), err)
		}
	}
	return ctx.Err()
}

// Microsoft documents subsequent operations on a RegOpenKeyTransacted handle
// as transacted. NtDeleteKey acts on that SAME held key; a bounded native fixture
// must prove visibility, rollback and conflict semantics. No path delete fallback.
// https://learn.microsoft.com/en-us/windows/win32/api/winreg/nf-winreg-regopenkeytransactedw
// https://learn.microsoft.com/en-us/windows-hardware/drivers/ddi/wdm/nf-wdm-zwdeletekey
func (n *installedRegistryOwner) remove(ctx context.Context, guards ...func(context.Context) error) (resultErr error) {
	if n.removeAttempted {
		return errors.New("installed registry deletion already consumed")
	}
	n.removeAttempted = true
	defer func() {
		if resultErr != nil {
			n.failed = true
		}
	}()
	if err := n.check(ctx); err != nil {
		return err
	}
	if err := errors.Join(installedDeleteKey.Find(), installedCommitTransaction.Find()); err != nil {
		return err
	}
	if len(guards) > 1 || len(guards) == 1 && guards[0] == nil {
		return errors.New("single same-owner post-stop registry guard required")
	}
	if len(guards) == 1 {
		if err := guards[0](ctx); err != nil {
			return err
		}
	}
	status, _, _ := installedDeleteKey.Call(uintptr(n.transacted))
	if int32(status) < 0 {
		return fmt.Errorf("original transacted registry deletion refused NTSTATUS %#x", uint32(status))
	}
	n.staged = true
	if len(guards) == 1 {
		if err := guards[0](ctx); err != nil {
			return err
		}
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	ok, _, err := installedCommitTransaction.Call(uintptr(n.transaction))
	if ok == 0 {
		return err
	}
	n.committed = true
	return n.check(ctx)
}

func (n *installedRegistryOwner) close() error {
	if n == nil || n.closed {
		if n == nil {
			return nil
		}
		return n.closeErr
	}
	n.closed = true
	if n.transacted != 0 {
		n.closeErr = errors.Join(n.closeErr, windows.RegCloseKey(n.transacted))
		n.transacted = 0
	}
	if n.transaction != 0 && n.transaction != windows.InvalidHandle {
		n.closeErr = errors.Join(n.closeErr, windows.CloseHandle(n.transaction))
		n.transaction = 0
	} // uncommitted close rolls back
	if n.original.key != 0 {
		n.closeErr = errors.Join(n.closeErr, windows.RegCloseKey(n.original.key))
		n.original.key = 0
	}
	for i := len(n.parents) - 1; i >= 0; i-- {
		n.closeErr = errors.Join(n.closeErr, windows.RegCloseKey(n.parents[i].key))
	}
	return n.closeErr
}
