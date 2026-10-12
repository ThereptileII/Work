//go:build windows

package launcher

import (
	"bytes"
	"context"
	"errors"
	"fmt"
	"runtime"
	"strings"
	"sync"
	"unsafe"

	"golang.org/x/sys/windows"
)

const brokerQueryAccount = `NT SERVICE\SkagerUpdateBroker`

// A token object can be shared by other processes of this logon. Adding only
// TOKEN_QUERY exposes read-only token metadata to this fixed service for that
// object's lifetime. Closing this owner does NOT revoke the ACE or handles the
// broker already opened. No token duplication, impersonation, adjustment, source
// access, default-DACL change, privilege enablement or account change occurs.
// Own-process access is cooperative, not birth/image authority. The independently
// held original executable remains authoritative. Child process access is set
// at creation; threads receive no broker grant. This helper grants no launch,
// commissioning, health or selection authority.
type brokerQueryAccess struct {
	mu                                sync.Mutex
	process                           windows.Handle
	token                             windows.Token
	proof                             brokerQueryIdentity
	processSD, tokenSD                *windows.SECURITY_DESCRIPTOR
	broker                            *windows.SID
	trusted                           [][]byte
	childSD                           *windows.SECURITY_DESCRIPTOR
	attributes                        windows.SecurityAttributes
	attributesUsed, childUsed, closed bool
	poisoned                          error
	closeErr                          error
	children                          []*brokerQueryChild
}
type brokerQueryChild struct {
	process            windows.Handle
	token              windows.Token
	proof              brokerQueryIdentity
	processSD, tokenSD *windows.SECURITY_DESCRIPTOR
}
type brokerQueryIdentity struct {
	pid            uint32
	created        windows.Filetime
	sid            string
	session        uint32
	logon, tokenID windows.LUID
}
type brokerQueryStatistics struct {
	TokenID, AuthenticationID                                                                   windows.LUID
	ExpirationTime                                                                              int64
	TokenType, ImpersonationLevel, DynamicCharged, DynamicAvailable, GroupCount, PrivilegeCount uint32
	ModifiedID                                                                                  windows.LUID
}

func brokerQueryNoImpersonation() error {
	var token windows.Token
	err := windows.OpenThreadToken(windows.CurrentThread(), windows.TOKEN_QUERY, true, &token)
	if err == nil {
		return errors.Join(errBrokerQueryAccess, token.Close())
	}
	if err != windows.ERROR_NO_TOKEN {
		return errors.Join(errBrokerQueryAccess, err)
	}
	return nil
}
func brokerQueryIdentityOf(process windows.Handle, token windows.Token) (brokerQueryIdentity, error) {
	var out brokerQueryIdentity
	var err error
	out.pid, err = windows.GetProcessId(process)
	if err != nil || out.pid == 0 {
		return out, errBrokerQueryAccess
	}
	var exit, kernel, user windows.Filetime
	if windows.GetProcessTimes(process, &out.created, &exit, &kernel, &user) != nil || filetimeValue(out.created) == 0 {
		return out, errBrokerQueryAccess
	}
	status, err := windows.WaitForSingleObject(process, 0)
	if err != nil || status != uint32(windows.WAIT_TIMEOUT) {
		return out, errBrokerQueryAccess
	}
	u, err := token.GetTokenUser()
	if err != nil || u.User.Sid == nil || !u.User.Sid.IsValid() {
		return out, errBrokerQueryAccess
	}
	out.sid = u.User.Sid.String()
	var used uint32
	var stats brokerQueryStatistics
	if windows.GetTokenInformation(token, windows.TokenSessionId, (*byte)(unsafe.Pointer(&out.session)), 4, &used) != nil || used != 4 || out.session == 0 || !strings.HasPrefix(out.sid, "S-1-5-21-") {
		return out, errBrokerQueryAccess
	}
	if windows.GetTokenInformation(token, windows.TokenStatistics, (*byte)(unsafe.Pointer(&stats)), uint32(unsafe.Sizeof(stats)), &used) != nil || used != uint32(unsafe.Sizeof(stats)) || stats.TokenType != windows.TokenPrimary || stats.AuthenticationID == (windows.LUID{}) {
		return out, errBrokerQueryAccess
	}
	out.logon, out.tokenID = stats.AuthenticationID, stats.TokenID
	return out, nil
}
func brokerQuerySIDBytes(sid *windows.SID) []byte {
	return bytes.Clone(unsafe.Slice((*byte)(unsafe.Pointer(sid)), int(windows.GetLengthSid(sid))))
}
func brokerQueryACLBytes(sd *windows.SECURITY_DESCRIPTOR) ([]byte, error) {
	if sd == nil || !sd.IsValid() || sd.Length() > 65535 {
		return nil, errBrokerQueryAccess
	}
	control, _, err := sd.Control()
	if err != nil || control&windows.SE_DACL_PRESENT == 0 {
		return nil, errBrokerQueryAccess
	}
	acl, _, err := sd.DACL()
	if err != nil || acl == nil {
		return nil, errBrokerQueryAccess
	}
	header := unsafe.Slice((*byte)(unsafe.Pointer(acl)), 8)
	size := int(header[2]) | int(header[3])<<8
	if size < 8 || size > int(sd.Length()) {
		return nil, errBrokerQueryAccess
	}
	return bytes.Clone(unsafe.Slice((*byte)(unsafe.Pointer(acl)), size)), nil
}

const brokerQuerySDInfo = windows.OWNER_SECURITY_INFORMATION | windows.GROUP_SECURITY_INFORMATION | windows.DACL_SECURITY_INFORMATION

func brokerQuerySameSD(a, b *windows.SECURITY_DESCRIPTOR) bool {
	if a == nil || b == nil || !a.IsValid() || !b.IsValid() {
		return false
	}
	ao, ad, ae := a.Owner()
	bo, bd, be := b.Owner()
	ag, agd, age := a.Group()
	bg, bgd, bge := b.Group()
	ac, _, ace := a.Control()
	bc, _, bce := b.Control()
	aa, aerr := brokerQueryACLBytes(a)
	ba, berr := brokerQueryACLBytes(b)
	_, aDefault, aDe := a.DACL()
	_, bDefault, bDe := b.DACL()
	return ae == nil && be == nil && age == nil && bge == nil && ao != nil && bo != nil && ag != nil && bg != nil && ao.String() == bo.String() && ag.String() == bg.String() && ad == bd && agd == bgd && ace == nil && bce == nil && ac == bc && aerr == nil && berr == nil && aDe == nil && bDe == nil && aDefault == bDefault && bytes.Equal(aa, ba)
}
func brokerQueryReadSD(handle windows.Handle) (*windows.SECURITY_DESCRIPTOR, error) {
	return windows.GetSecurityInfo(handle, windows.SE_KERNEL_OBJECT, brokerQuerySDInfo)
}
func brokerQueryGrant(handle windows.Handle, sid string, broker *windows.SID, mask uint32, trusted [][]byte) (*windows.SECURITY_DESCRIPTOR, error) {
	original, err := brokerQueryReadSD(handle)
	if err != nil {
		return nil, err
	}
	if original == nil || !original.IsValid() {
		return nil, errBrokerQueryAccess
	}
	owner, _, err := original.Owner()
	group, _, ge := original.Group()
	if err != nil || ge != nil || owner == nil || group == nil || !owner.IsValid() || !group.IsValid() || (owner.String() != sid && owner.String() != "S-1-5-18") {
		return nil, errBrokerQueryAccess
	}
	raw, err := brokerQueryACLBytes(original)
	if err != nil {
		return nil, err
	}
	updated, err := appendBrokerQueryACE(raw, brokerQuerySIDBytes(broker), mask, trusted)
	if err != nil {
		return nil, err
	}
	// Build an expected descriptor from the original, changing only its DACL.
	expected, err := original.ToAbsolute()
	if err != nil {
		return nil, err
	}
	_, defaulted, err := original.DACL()
	if err != nil {
		return nil, err
	}
	acl := (*windows.ACL)(unsafe.Pointer(&updated[0]))
	if expected.SetDACL(acl, true, defaulted) != nil {
		return nil, errBrokerQueryAccess
	}
	expected, err = expected.ToSelfRelative()
	if err != nil {
		return nil, err
	}
	if !bytes.Equal(raw, updated) {
		// There is no kernel compare-and-set DACL API. Refuse an observed
		// intervening change; exact readback and later Validate remain required.
		before, e := brokerQueryReadSD(handle)
		if e != nil || !brokerQuerySameSD(original, before) {
			return nil, errors.Join(errBrokerQueryAccess, e)
		}
		if err = windows.SetSecurityInfo(handle, windows.SE_KERNEL_OBJECT, windows.DACL_SECURITY_INFORMATION, nil, nil, acl, nil); err != nil {
			return nil, errors.Join(errBrokerQueryAccess, err)
		}
	}
	actual, err := brokerQueryReadSD(handle)
	runtime.KeepAlive(updated)
	if err != nil || !brokerQuerySameSD(expected, actual) {
		return nil, errors.Join(errBrokerQueryAccess, err)
	}
	return actual, nil
}

// prepareBrokerQueryAccessNative has no SID, path, PID or privilege parameters.
// It deliberately modifies only the actual nonimpersonated current executor.
// An error after mutation is uncertain, never permission to restore/retry.
func prepareBrokerQueryAccessNative(ctx context.Context) (_ *brokerQueryAccess, retErr error) {
	if ctx == nil {
		return nil, errBrokerQueryAccess
	}
	if ctx.Err() != nil {
		return nil, errors.Join(errBrokerQueryAccess, ctx.Err())
	}
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if err := brokerQueryNoImpersonation(); err != nil {
		return nil, err
	}
	broker, _, _, err := windows.LookupSID("", brokerQueryAccount)
	if err != nil || broker == nil || !broker.IsValid() || !strings.HasPrefix(broker.String(), "S-1-5-80-") {
		return nil, errBrokerQueryAccess
	}
	process, err := windows.OpenProcess(windows.PROCESS_QUERY_INFORMATION|windows.PROCESS_QUERY_LIMITED_INFORMATION|windows.SYNCHRONIZE|windows.READ_CONTROL|windows.WRITE_DAC, false, windows.GetCurrentProcessId())
	if err != nil {
		return nil, err
	}
	out := &brokerQueryAccess{process: process, broker: broker}
	defer func() {
		if retErr != nil {
			retErr = errors.Join(retErr, out.Close())
		}
	}()
	if err = windows.OpenProcessToken(process, windows.TOKEN_QUERY|windows.READ_CONTROL|windows.WRITE_DAC, &out.token); err != nil {
		return nil, err
	}
	out.proof, err = brokerQueryIdentityOf(process, out.token)
	if err != nil {
		return nil, err
	}
	for _, s := range []string{out.proof.sid, "S-1-5-18", "S-1-5-32-544"} {
		sid, e := windows.StringToSid(s)
		if e != nil {
			return nil, e
		}
		out.trusted = append(out.trusted, brokerQuerySIDBytes(sid))
	}
	groups, err := out.token.GetTokenGroups()
	if err != nil || groups.GroupCount > 256 {
		return nil, errBrokerQueryAccess
	}
	logons := 0
	for _, g := range groups.AllGroups() {
		if g.Attributes&windows.SE_GROUP_LOGON_ID == windows.SE_GROUP_LOGON_ID {
			if g.Sid == nil || !g.Sid.IsValid() {
				return nil, errBrokerQueryAccess
			}
			out.trusted = append(out.trusted, brokerQuerySIDBytes(g.Sid))
			logons++
		}
	}
	if logons != 1 {
		return nil, errBrokerQueryAccess
	}
	out.processSD, err = brokerQueryGrant(process, out.proof.sid, broker, brokerQueryProcessMask, out.trusted)
	if err != nil {
		return nil, err
	}
	out.tokenSD, err = brokerQueryGrant(windows.Handle(out.token), out.proof.sid, broker, brokerQueryTokenMask, out.trusted)
	if err != nil {
		return nil, err
	}
	out.childSD, err = windows.SecurityDescriptorFromString(fmt.Sprintf("O:%sG:%sD:P(A;;0x001fffff;;;%s)(A;;0x001fffff;;;SY)(A;;0x00101400;;;%s)", out.proof.sid, out.proof.sid, out.proof.sid, broker.String()))
	if err != nil {
		return nil, err
	}
	out.attributes = windows.SecurityAttributes{Length: uint32(unsafe.Sizeof(windows.SecurityAttributes{})), SecurityDescriptor: out.childSD}
	if err = out.Validate(ctx); err != nil {
		return nil, err
	}
	return out, nil
}
func brokerQueryCheck(process windows.Handle, token windows.Token, proof brokerQueryIdentity, processSD, tokenSD *windows.SECURITY_DESCRIPTOR) error {
	actual, err := brokerQueryIdentityOf(process, token)
	if err != nil || actual != proof {
		return errBrokerQueryAccess
	}
	var current windows.Token
	if err = windows.OpenProcessToken(process, windows.TOKEN_QUERY, &current); err != nil {
		return err
	}
	currentProof, e := brokerQueryIdentityOf(process, current)
	closeErr := current.Close()
	if e != nil || currentProof != proof || closeErr != nil {
		return errors.Join(errBrokerQueryAccess, e, closeErr)
	}
	p, e := brokerQueryReadSD(process)
	t, te := brokerQueryReadSD(windows.Handle(token))
	if e != nil || te != nil || !brokerQuerySameSD(p, processSD) || !brokerQuerySameSD(t, tokenSD) {
		return errors.Join(errBrokerQueryAccess, e, te)
	}
	return nil
}
func (o *brokerQueryAccess) validateLocked(ctx context.Context) error {
	if ctx == nil {
		return errBrokerQueryAccess
	}
	if ctx.Err() != nil || o.closed || o.poisoned != nil {
		return errors.Join(errBrokerQueryAccess, o.poisoned, ctx.Err())
	}
	if err := brokerQueryNoImpersonation(); err != nil {
		return err
	}
	if windows.GetCurrentProcessId() != o.proof.pid {
		return errBrokerQueryAccess
	}
	if err := brokerQueryCheck(o.process, o.token, o.proof, o.processSD, o.tokenSD); err != nil {
		return err
	}
	for _, child := range o.children {
		if err := brokerQueryCheck(child.process, child.token, child.proof, child.processSD, child.tokenSD); err != nil {
			return err
		}
	}
	return ctx.Err()
}
func (o *brokerQueryAccess) Validate(ctx context.Context) error {
	if o == nil {
		return errBrokerQueryAccess
	}
	o.mu.Lock()
	defer o.mu.Unlock()
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	err := o.validateLocked(ctx)
	if err != nil {
		o.poisoned = errors.Join(o.poisoned, err)
	}
	return err
}
func (o *brokerQueryAccess) childProcessAttributes() (*windows.SecurityAttributes, error) {
	if o == nil {
		return nil, errBrokerQueryAccess
	}
	o.mu.Lock()
	defer o.mu.Unlock()
	if o.closed || o.poisoned != nil || o.attributesUsed {
		return nil, errBrokerQueryAccess
	}
	o.attributesUsed = true
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if err := o.validateLocked(context.Background()); err != nil {
		o.poisoned = errors.Join(o.poisoned, err)
		return nil, err
	}
	return &o.attributes, nil
}
func (o *brokerQueryAccess) bindSuspendedChild(ctx context.Context, child *nativeOrdinaryChild) (retErr error) {
	if o == nil {
		return errBrokerQueryAccess
	}
	o.mu.Lock()
	defer o.mu.Unlock()
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if o.childUsed || !o.attributesUsed || o.closed || o.poisoned != nil {
		return errBrokerQueryAccess
	}
	o.childUsed = true
	defer func() {
		if retErr != nil {
			o.poisoned = errors.Join(o.poisoned, retErr)
		}
	}()
	if err := o.validateLocked(ctx); err != nil {
		return err
	}
	if child == nil || !child.owned || child.resumeAttempted {
		return errBrokerQueryAccess
	}
	created, err := child.rawIdentity()
	if err != nil || created != child.creation || filetimeValue(created) < filetimeValue(o.proof.created) {
		return errBrokerQueryAccess
	}
	var info windows.PROCESS_BASIC_INFORMATION
	var used uint32
	if windows.NtQueryInformationProcess(child.process, windows.ProcessBasicInformation, unsafe.Pointer(&info), uint32(unsafe.Sizeof(info)), &used) != nil || used != uint32(unsafe.Sizeof(info)) || uint32(info.InheritedFromUniqueProcessId) != o.proof.pid || uint32(info.UniqueProcessId) != child.pid {
		return errBrokerQueryAccess
	}
	retained := &brokerQueryChild{}
	defer func() {
		if retErr != nil {
			retErr = errors.Join(retErr, retained.close())
		}
	}()
	if err = windows.DuplicateHandle(windows.CurrentProcess(), child.process, windows.CurrentProcess(), &retained.process, windows.PROCESS_QUERY_INFORMATION|windows.PROCESS_QUERY_LIMITED_INFORMATION|windows.SYNCHRONIZE|windows.READ_CONTROL, false, 0); err != nil {
		return err
	}
	if err = windows.OpenProcessToken(child.process, windows.TOKEN_QUERY|windows.READ_CONTROL|windows.WRITE_DAC, &retained.token); err != nil {
		return err
	}
	retained.proof, err = brokerQueryIdentityOf(retained.process, retained.token)
	if err != nil || retained.proof.sid != o.proof.sid || retained.proof.session != o.proof.session || retained.proof.logon != o.proof.logon || retained.proof.created != created || retained.proof.pid != child.pid {
		return errBrokerQueryAccess
	}
	retained.processSD, err = brokerQueryReadSD(retained.process)
	if err != nil || !brokerQuerySameSD(retained.processSD, o.childSD) {
		return errBrokerQueryAccess
	}
	retained.tokenSD, err = brokerQueryGrant(windows.Handle(retained.token), o.proof.sid, o.broker, brokerQueryTokenMask, o.trusted)
	if err != nil {
		return err
	}
	if err = brokerQueryCheck(retained.process, retained.token, retained.proof, retained.processSD, retained.tokenSD); err != nil {
		return err
	}
	if ctx.Err() != nil || child.resumeAttempted {
		return errors.Join(errBrokerQueryAccess, ctx.Err())
	}
	o.children = append(o.children, retained)
	return nil
}
func (c *brokerQueryChild) close() error {
	var err error
	if c.token != 0 {
		err = errors.Join(err, c.token.Close())
		c.token = 0
	}
	if c.process != 0 {
		err = errors.Join(err, windows.CloseHandle(c.process))
		c.process = 0
	}
	return err
}

// Close always drains once, caches any error, and leaves deliberate grants intact.
func (o *brokerQueryAccess) Close() error {
	if o == nil {
		return nil
	}
	o.mu.Lock()
	defer o.mu.Unlock()
	if o.closed {
		return o.closeErr
	}
	o.closed = true
	for i := len(o.children) - 1; i >= 0; i-- {
		o.closeErr = errors.Join(o.closeErr, o.children[i].close())
	}
	if o.token != 0 {
		o.closeErr = errors.Join(o.closeErr, o.token.Close())
		o.token = 0
	}
	if o.process != 0 {
		o.closeErr = errors.Join(o.closeErr, windows.CloseHandle(o.process))
		o.process = 0
	}
	o.closeErr = errors.Join(o.closeErr, o.poisoned)
	return o.closeErr
}
