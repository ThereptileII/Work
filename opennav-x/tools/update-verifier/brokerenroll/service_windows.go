//go:build windows

package brokerenroll

import (
	"errors"
	"path/filepath"
	"runtime"
	"strings"
	"unsafe"

	"golang.org/x/sys/windows"
)

// CreateServiceW has no SECURITY_ATTRIBUTES parameter: SCM supplies the initial
// descriptor. Its documented default excludes untrusted config/start/stop/delete
// and security mutation. Default user-defined/interrogate rights cannot control
// this stopped service. We check that restriction before tightening, then never
// start until the final exact DACL has been checked. This does NOT claim the final
// descriptor existed at creation, or authenticate prior SCM/registry history.
// Independently authenticated elevated provisioning and originally protected
// machine SCM/registry are prerequisites; present ACLs cannot prove their history.
type nativeServiceBackend struct {
	scm, service                 windows.Handle
	registry                     []windows.Handle
	registryPaths, registryNames []string
	expected                     *windows.SECURITY_DESCRIPTOR
	expectedStart                uint32
	activationAttempted          bool
	image                        string
}

func serviceCaller() error {
	if !windows.GetCurrentProcessToken().IsElevated() {
		return errors.New("elevated native provisioner required")
	}
	var t windows.Token
	err := windows.OpenThreadToken(windows.CurrentThread(), windows.TOKEN_QUERY, true, &t)
	if err == nil {
		_ = t.Close()
		return errors.New("impersonated service provisioning refused")
	}
	if !errors.Is(err, windows.ERROR_NO_TOKEN) {
		return err
	}
	return nil
}

func newServiceBackend() (_ serviceBackend, resultErr error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if err := serviceCaller(); err != nil {
		return nil, err
	}
	n := &nativeServiceBackend{}
	ready := false
	defer func() {
		if !ready {
			resultErr = errors.Join(resultErr, n.close())
		}
	}()
	var err error
	n.scm, err = windows.OpenSCManager(nil, nil, windows.SC_MANAGER_CONNECT|windows.SC_MANAGER_CREATE_SERVICE|windows.READ_CONTROL)
	if err != nil {
		return nil, err
	}
	for _, name := range []string{`SYSTEM`, `SYSTEM\Select`, `SYSTEM\CurrentControlSet`, `SYSTEM\CurrentControlSet\Services`} {
		if err := n.holdRegistry(name); err != nil {
			return nil, err
		}
	}
	if err := n.checkMachine(); err != nil {
		return nil, err
	}
	ready = true
	return n, nil
}

func trustedServiceOwner(s string) bool {
	return s == "S-1-5-18" || s == "S-1-5-32-544" || s == "S-1-5-80-956008885-3418522649-1831038044-1853292631-2271478464"
}

type serviceSecurityRole uint8

const (
	serviceSecurityUnspecified serviceSecurityRole = iota
	serviceSecurityManager
	serviceSecurityRegistryAncestor
	serviceSecurityInitialService
)

func (r serviceSecurityRole) String() string {
	switch r {
	case serviceSecurityManager:
		return "sc-manager"
	case serviceSecurityRegistryAncestor:
		return "registry-ancestor"
	case serviceSecurityInitialService:
		return "initial-service"
	default:
		return "unspecified"
	}
}

type serviceSecurityLocation struct {
	role serviceSecurityRole
	// One-based index in the fixed held registry ancestry; zero otherwise.
	ancestor uint32
}

// Private facts from the descriptor already inspected on the original handle.
// Error deliberately exposes no additional machine or principal information.
type serviceMutationError struct {
	location         serviceSecurityLocation
	ownerSID, aceSID string
	mask, mutation   uint32
	flags            byte
}

func (*serviceMutationError) Error() string { return "untrusted machine mutation permission" }

// Unknown ACE layouts fail closed. Deny ACEs do not grant mutation and need no
// interpretation; allowed generic rights are included in the mutation mask.
func serviceDescriptorSafe(sd *windows.SECURITY_DESCRIPTOR, mutation uint32) error {
	return serviceDescriptorSafeAt(sd, mutation, serviceSecurityLocation{})
}

func serviceDescriptorSafeAt(sd *windows.SECURITY_DESCRIPTOR, mutation uint32, location serviceSecurityLocation) error {
	if sd == nil {
		return errors.New("machine security descriptor unavailable")
	}
	owner, _, err := sd.Owner()
	if err != nil || owner == nil || !trustedServiceOwner(owner.String()) {
		return errors.New("trusted machine owner required")
	}
	acl, _, err := sd.DACL()
	if err != nil || acl == nil {
		return errors.New("non-null machine DACL required")
	}
	for i := uint32(0); i < uint32(acl.AceCount); i++ {
		var ace *windows.ACCESS_ALLOWED_ACE
		if windows.GetAce(acl, i, &ace) != nil || ace == nil {
			return errors.New("machine ACE unavailable")
		}
		if ace.Header.AceType == windows.ACCESS_DENIED_ACE_TYPE {
			continue
		}
		if ace.Header.AceType != windows.ACCESS_ALLOWED_ACE_TYPE {
			return errors.New("unsupported machine ACE")
		}
		sid := (*windows.SID)(unsafe.Pointer(&ace.SidStart))
		if !sid.IsValid() {
			return errors.New("invalid machine ACE SID")
		}
		// CREATOR OWNER is an inheritance placeholder, replaced with the
		// actual creator SID in an effective child ACE, not a trusted principal:
		// https://learn.microsoft.com/windows/win32/secauthz/ace-inheritance-rules
		// Recognize only the observed registry template on the four fixed HKLM
		// ancestors or the new service key. Each child's own owner and effective
		// ACL must still pass this validator; no parent create right is implied.
		if location.role == serviceSecurityRegistryAncestor && location.ancestor >= 1 && location.ancestor <= 5 &&
			sid.String() == "S-1-3-0" && ace.Header.AceFlags == windows.CONTAINER_INHERIT_ACE && uint32(ace.Mask) == windows.KEY_ALL_ACCESS {
			continue
		}
		if ace.Header.AceFlags&windows.INHERIT_ONLY_ACE == 0 && uint32(ace.Mask)&mutation != 0 && !trustedServiceOwner(sid.String()) {
			return &serviceMutationError{location: location, ownerSID: owner.String(), aceSID: sid.String(),
				mask: uint32(ace.Mask), mutation: mutation, flags: ace.Header.AceFlags}
		}
	}
	return nil
}

func safeServiceHandle(h windows.Handle, kind windows.SE_OBJECT_TYPE, mutation uint32, location serviceSecurityLocation) error {
	sd, err := windows.GetSecurityInfo(h, kind, windows.OWNER_SECURITY_INFORMATION|windows.DACL_SECURITY_INFORMATION)
	if err != nil {
		return err
	}
	return serviceDescriptorSafeAt(sd, mutation, location)
}

func (n *nativeServiceBackend) checkMachine() error {
	if err := serviceCaller(); err != nil {
		return err
	}
	mutation := uint32(windows.DELETE | windows.WRITE_DAC | windows.WRITE_OWNER | windows.GENERIC_ALL | windows.GENERIC_WRITE)
	if err := safeServiceHandle(n.scm, windows.SE_SERVICE, mutation|windows.GENERIC_EXECUTE|windows.SC_MANAGER_CREATE_SERVICE|windows.SC_MANAGER_LOCK|windows.SC_MANAGER_MODIFY_BOOT_CONFIG, serviceSecurityLocation{role: serviceSecurityManager}); err != nil {
		return err
	}
	for i, h := range n.registry {
		if err := safeServiceHandle(h, windows.SE_REGISTRY_KEY, mutation|windows.KEY_SET_VALUE|windows.KEY_CREATE_SUB_KEY|windows.KEY_CREATE_LINK, serviceSecurityLocation{role: serviceSecurityRegistryAncestor, ancestor: uint32(i + 1)}); err != nil {
			return err
		}
		name, err := nativeRegistryName(h)
		if err != nil || name != n.registryNames[i] {
			return errors.New("held machine registry name changed")
		}
		p, err := windows.UTF16PtrFromString(n.registryPaths[i])
		if err != nil {
			return err
		}
		var reopened windows.Handle
		if err := windows.RegOpenKeyEx(windows.HKEY_LOCAL_MACHINE, p, 0, windows.KEY_READ|windows.READ_CONTROL, &reopened); err != nil {
			return err
		}
		actual, queryErr := nativeRegistryName(reopened)
		closeErr := windows.RegCloseKey(reopened)
		if queryErr != nil || closeErr != nil || actual != name {
			return errors.New("machine registry resolution changed or uncertain")
		}
	}
	return nil
}

func fixedServiceImage(image string) error {
	root, err := windows.KnownFolderPath(windows.FOLDERID_ProgramFilesX86, 0)
	if err != nil {
		return err
	}
	expected := filepath.Join(root, "SKAGER", "UpdateBroker", "skager-update-broker.exe")
	if image != expected || strings.ContainsAny(image, "\x00\r\n\"") {
		return errors.New("fixed native service image required")
	}
	return nil
}

func (n *nativeServiceBackend) create(image string) error {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if n.service != 0 {
		return errors.New("service creation already attempted")
	}
	if err := n.checkMachine(); err != nil {
		return err
	}
	if err := fixedServiceImage(image); err != nil {
		return err
	}
	name, _ := windows.UTF16PtrFromString(serviceName)
	account, _ := windows.UTF16PtrFromString(serviceAccount)
	quoted, _ := windows.UTF16PtrFromString(`"` + image + `"`)
	var err error
	// Exclusive SCM name creation refuses existing/deletion-pending services.
	// Every subsequent failure retains this service and any partial disk state.
	n.service, err = windows.CreateService(n.scm, name, name, windows.SERVICE_QUERY_CONFIG|windows.SERVICE_QUERY_STATUS|windows.SERVICE_START|windows.SERVICE_CHANGE_CONFIG|windows.READ_CONTROL|windows.WRITE_DAC,
		windows.SERVICE_WIN32_OWN_PROCESS, windows.SERVICE_DEMAND_START, windows.SERVICE_ERROR_NORMAL, quoted, nil, nil, nil, account, nil)
	if err != nil {
		return err
	}
	if err := n.holdRegistry(`SYSTEM\CurrentControlSet\Services\` + serviceName); err != nil {
		return err
	}
	mutation := uint32(windows.SERVICE_CHANGE_CONFIG | windows.SERVICE_START | windows.SERVICE_STOP | windows.SERVICE_PAUSE_CONTINUE | windows.DELETE | windows.WRITE_DAC | windows.WRITE_OWNER | windows.GENERIC_ALL | windows.GENERIC_WRITE | windows.GENERIC_EXECUTE)
	if err := safeServiceHandle(n.service, windows.SE_SERVICE, mutation, serviceSecurityLocation{role: serviceSecurityInitialService}); err != nil {
		return err
	}
	s, err := n.status()
	if err != nil || !serviceInitiallyStopped(s) {
		return &serviceInitialStateError{phase: serviceInitialCreated, state: s, cause: err}
	}
	broker, _, _, err := windows.LookupSID("", serviceAccount)
	if err != nil || broker == nil || !strings.HasPrefix(broker.String(), "S-1-5-80-") {
		return errors.New("broker service SID unavailable")
	}
	// AU's read-only query is needed by brokerpipe.Call's independent SCM check.
	n.expected, err = windows.SecurityDescriptorFromString("D:P(A;;0xf01ff;;;SY)(A;;0xf01ff;;;BA)(A;;0x20005;;;AU)(A;;0x20005;;;" + broker.String() + ")")
	if err != nil {
		return err
	}
	acl, _, err := n.expected.DACL()
	if err != nil {
		return err
	}
	if err := windows.SetSecurityInfo(n.service, windows.SE_SERVICE, windows.DACL_SECURITY_INFORMATION|windows.PROTECTED_DACL_SECURITY_INFORMATION, nil, nil, acl, nil); err != nil {
		return err
	}
	n.image = image
	return n.check(image)
}

func (n *nativeServiceBackend) check(image string) error {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if err := n.checkMachine(); err != nil {
		return err
	}
	if n.service == 0 || n.expected == nil {
		return errors.New("new tightened service required")
	}
	if err := fixedServiceImage(image); err != nil {
		return err
	}
	actual, err := windows.GetSecurityInfo(n.service, windows.SE_SERVICE, windows.OWNER_SECURITY_INFORMATION|windows.DACL_SECURITY_INFORMATION)
	if err != nil {
		return err
	}
	if err := finalServiceDescriptor(actual, n.expected); err != nil {
		return err
	}
	var needed uint32
	if err := windows.QueryServiceConfig(n.service, nil, 0, &needed); !errors.Is(err, windows.ERROR_INSUFFICIENT_BUFFER) || needed < uint32(unsafe.Sizeof(windows.QUERY_SERVICE_CONFIG{})) || needed > 8192 {
		return errors.New("service configuration required size unavailable or outside bound")
	}
	buffer := make([]byte, needed)
	config := (*windows.QUERY_SERVICE_CONFIG)(unsafe.Pointer(&buffer[0]))
	if err := windows.QueryServiceConfig(n.service, config, uint32(len(buffer)), &needed); err != nil {
		return err
	}
	imageName, imageErr := boundedServiceString(buffer, config.BinaryPathName)
	account, accountErr := boundedServiceString(buffer, config.ServiceStartName)
	group, groupErr := boundedServiceString(buffer, config.LoadOrderGroup)
	dependencies, dependenciesErr := boundedServiceString(buffer, config.Dependencies)
	if imageErr != nil || accountErr != nil || groupErr != nil || dependenciesErr != nil {
		return errors.New("service configuration strings outside bound")
	}
	if config.ServiceType != windows.SERVICE_WIN32_OWN_PROCESS || config.StartType != n.startType() || config.ErrorControl != windows.SERVICE_ERROR_NORMAL ||
		imageName != `"`+image+`"` || !strings.EqualFold(account, serviceAccount) || group != "" || dependencies != "" {
		return errors.New("service configuration differs")
	}
	return nil
}

func boundedServiceString(buffer []byte, p *uint16) (string, error) {
	if p == nil {
		return "", nil
	}
	if len(buffer) < 2 {
		return "", errors.New("service string buffer unavailable")
	}
	base := uintptr(unsafe.Pointer(&buffer[0]))
	address := uintptr(unsafe.Pointer(p))
	if address < base || address-base >= uintptr(len(buffer)-1) || (address-base)%2 != 0 {
		return "", errors.New("service string pointer outside buffer")
	}
	offset := int(address - base)
	units := unsafe.Slice(p, (len(buffer)-offset)/2)
	for i, v := range units {
		if v == 0 {
			return windows.UTF16ToString(units[:i]), nil
		}
	}
	return "", errors.New("service string not terminated within buffer")
}

func finalServiceDescriptor(actual, expected *windows.SECURITY_DESCRIPTOR) error {
	if actual == nil || expected == nil {
		return errors.New("service descriptor unavailable")
	}
	owner, _, err := actual.Owner()
	if err != nil || owner == nil || !trustedServiceOwner(owner.String()) {
		return errors.New("trusted service owner required")
	}
	control, _, err := actual.Control()
	if err != nil || control&(windows.SE_DACL_PRESENT|windows.SE_DACL_PROTECTED) != windows.SE_DACL_PRESENT|windows.SE_DACL_PROTECTED {
		return errors.New("protected service DACL required")
	}
	actualACL, _, err := actual.DACL()
	if err != nil || actualACL == nil {
		return errors.New("service DACL unavailable")
	}
	expectedACL, _, err := expected.DACL()
	if err != nil || expectedACL == nil || actualACL.AceCount != expectedACL.AceCount {
		return errors.New("service DACL differs")
	}
	for i := uint32(0); i < uint32(expectedACL.AceCount); i++ {
		var a, e *windows.ACCESS_ALLOWED_ACE
		if windows.GetAce(actualACL, i, &a) != nil || windows.GetAce(expectedACL, i, &e) != nil || a == nil || e == nil || a.Header != e.Header || a.Mask != e.Mask || !(*windows.SID)(unsafe.Pointer(&a.SidStart)).Equals((*windows.SID)(unsafe.Pointer(&e.SidStart))) {
			return errors.New("service DACL differs")
		}
	}
	return nil
}

func (n *nativeServiceBackend) start(mode string) error {
	if mode != "initialize-store" && mode != "verify-store" {
		return errors.New("stopped-only initialization command required")
	}
	p, err := windows.UTF16PtrFromString(mode)
	if err != nil {
		return err
	}
	return windows.StartService(n.service, 1, &p)
}

func (n *nativeServiceBackend) startClientRegistration(sid, approvedSHA string) error {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if !validPin(approvedSHA) || validateClientSIDNative(sid) != nil {
		return errors.New("exact independently approved client service arguments required")
	}
	if err := serviceCaller(); err != nil {
		return err
	}
	values := []string{"register-client", sid, approvedSHA}
	var arguments [3]*uint16
	for i, value := range values {
		p, err := windows.UTF16PtrFromString(value)
		if err != nil {
			return err
		}
		arguments[i] = p
	}
	// This uses the same exclusively created, checked, retained service handle.
	// Never append a path, normally start, announce Running or retry SCM refusal.
	return windows.StartService(n.service, uint32(len(arguments)), &arguments[0])
}

func (n *nativeServiceBackend) status() (serviceState, error) {
	var s windows.SERVICE_STATUS_PROCESS
	var used uint32
	if err := windows.QueryServiceStatusEx(n.service, windows.SC_STATUS_PROCESS_INFO, (*byte)(unsafe.Pointer(&s)), uint32(unsafe.Sizeof(s)), &used); err != nil {
		return serviceState{}, err
	}
	if s.ServiceType != windows.SERVICE_WIN32_OWN_PROCESS {
		return serviceState{}, errors.New("service status type differs")
	}
	return serviceState{s.CurrentState, s.ProcessId, s.Win32ExitCode, s.ServiceSpecificExitCode, s.ControlsAccepted}, nil
}

func (n *nativeServiceBackend) close() error {
	var result error
	if n.service != 0 {
		result = errors.Join(result, windows.CloseServiceHandle(n.service))
		n.service = 0
	}
	for i := len(n.registry) - 1; i >= 0; i-- {
		result = errors.Join(result, windows.RegCloseKey(n.registry[i]))
	}
	n.registry = nil
	n.registryPaths = nil
	n.registryNames = nil
	if n.scm != 0 {
		result = errors.Join(result, windows.CloseServiceHandle(n.scm))
		n.scm = 0
	}
	if result != nil {
		return errors.Join(ErrCleanup, result)
	}
	return nil
}

var queryEnrollmentRegistryKey = windows.NewLazySystemDLL("ntdll.dll").NewProc("NtQueryKey")

func nativeRegistryName(h windows.Handle) (string, error) {
	// KeyNameInformation gives the resolved kernel registry path. This handles
	// CurrentControlSet's protected registry alias, not a filesystem reparse point.
	// Repeated name lookup corroborates resolution; it is not a registry file ID
	// or proof of historical protection/absence of privileged registry writers.
	buffer := make([]byte, 32768)
	var used uint32
	status, _, _ := queryEnrollmentRegistryKey.Call(uintptr(h), 3, uintptr(unsafe.Pointer(&buffer[0])), uintptr(len(buffer)), uintptr(unsafe.Pointer(&used)))
	if status != 0 || used < 4 || used > uint32(len(buffer)) {
		return "", errors.New("native registry name unavailable")
	}
	length := *(*uint32)(unsafe.Pointer(&buffer[0]))
	if length == 0 || length%2 != 0 || length > used-4 {
		return "", errors.New("native registry name outside bound")
	}
	return windows.UTF16ToString(unsafe.Slice((*uint16)(unsafe.Pointer(&buffer[4])), length/2)), nil
}

func (n *nativeServiceBackend) holdRegistry(path string) error {
	p, err := windows.UTF16PtrFromString(path)
	if err != nil {
		return err
	}
	var h windows.Handle
	if err := windows.RegOpenKeyEx(windows.HKEY_LOCAL_MACHINE, p, 0, windows.KEY_READ|windows.READ_CONTROL, &h); err != nil {
		return err
	}
	n.registry = append(n.registry, h)
	n.registryPaths = append(n.registryPaths, path)
	name, err := nativeRegistryName(h)
	n.registryNames = append(n.registryNames, name)
	if err != nil {
		return err
	}
	if !fixedRegistryName(path, name, n.registryNames) {
		return errors.New("unexpected native machine registry ancestry")
	}
	return nil
}

func fixedRegistryName(path, name string, held []string) bool {
	const system = `\REGISTRY\MACHINE\SYSTEM`
	switch path {
	case `SYSTEM`:
		return strings.EqualFold(name, system)
	case `SYSTEM\Select`:
		return strings.EqualFold(name, system+`\Select`)
	case `SYSTEM\CurrentControlSet`:
		prefix := system + `\ControlSet`
		if len(name) != len(prefix)+3 || !strings.EqualFold(name[:len(prefix)], prefix) {
			return false
		}
		for _, c := range name[len(prefix):] {
			if c < '0' || c > '9' {
				return false
			}
		}
		return true
	case `SYSTEM\CurrentControlSet\Services`:
		return len(held) >= 3 && strings.EqualFold(name, held[2]+`\Services`)
	case `SYSTEM\CurrentControlSet\Services\` + serviceName:
		return len(held) >= 4 && strings.EqualFold(name, held[3]+`\`+serviceName)
	}
	return false
}

// Only the explicit original-enrollment completion path can call this private
// capability. Run never calls it; start(mode) still rejects normal activation.
func (n *nativeServiceBackend) startType() uint32 {
	if n.expectedStart == 0 {
		return windows.SERVICE_DEMAND_START
	}
	return n.expectedStart
}
func (n *nativeServiceBackend) activateNormal() error {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if n.activationAttempted {
		return errors.New("normal service activation already consumed")
	}
	n.activationAttempted = true
	if err := serviceCaller(); err != nil {
		return err
	}
	if n.service == 0 || n.expected == nil || n.startType() != windows.SERVICE_DEMAND_START || n.image == "" {
		return errors.New("original demand-start service required")
	}
	if err := n.check(n.image); err != nil {
		return err
	}
	s, err := n.status()
	if err != nil || !serviceCompleted(s) {
		return errors.Join(errors.New("original service must be stopped before activation"), err)
	}
	if err := windows.ChangeServiceConfig(n.service, windows.SERVICE_NO_CHANGE, windows.SERVICE_AUTO_START, windows.SERVICE_NO_CHANGE, nil, nil, nil, nil, nil, nil, nil); err != nil {
		return err
	}
	n.expectedStart = windows.SERVICE_AUTO_START
	if err := n.check(n.image); err != nil {
		return err
	}
	// The caller also checks retained file custody before and after.
	return windows.StartService(n.service, 0, nil)
}
