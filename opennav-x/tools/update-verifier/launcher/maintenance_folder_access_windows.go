package launcher

import (
	"context"
	"errors"
	"runtime"
	"strings"
	"unsafe"

	"golang.org/x/sys/windows"
)

// Explicit maintenance only: lend the fixed broker the documented folder-query
// rights on this same primary token object, then restore the exact query-only
// descriptor. The token may be shared by this logon's processes; restoring the
// DACL does not revoke handles already opened during the interval. Abrupt
// executor death prevents in-process cleanup, so the grant can survive on a
// shared token; this owner does not promise crash-proof revocation. No thread
// impersonation, duplication, adjustment, default-DACL change or child grant is
// performed. This grants no operator, selection, launch or transaction authority.
// Windows has no compare-and-set DACL API: exact before/readback checks detect
// observed drift, but cannot exclude an unobserved intervening ACL change.
// Keep the portable literal tied to the actual Windows rights, in both
// directions: a mismatched bit is a compile error, including QUERY_SOURCE.
var _ [maintenanceFolderTokenMask - (windows.TOKEN_QUERY | windows.TOKEN_IMPERSONATE)]byte
var _ [(windows.TOKEN_QUERY | windows.TOKEN_IMPERSONATE) - maintenanceFolderTokenMask]byte

type maintenanceFolderAccess struct {
	access                             *brokerQueryAccess
	process                            windows.Handle
	token                              windows.Token
	proof                              brokerQueryIdentity
	processSD, originalSD, temporarySD *windows.SECURITY_DESCRIPTOR
	attempted, closed                  bool
	failure, closeErr                  error
}

func maintenanceFolderAdmin(token windows.Token) error {
	if !token.IsElevated() {
		return errBrokerQueryAccess
	}
	admin, err := windows.CreateWellKnownSid(windows.WinBuiltinAdministratorsSid)
	if err != nil {
		return err
	}
	var needed uint32
	err = windows.GetTokenInformation(token, windows.TokenGroups, nil, 0, &needed)
	if err != windows.ERROR_INSUFFICIENT_BUFFER || needed < uint32(unsafe.Sizeof(windows.Tokengroups{})) || needed > 64<<10 {
		return errBrokerQueryAccess
	}
	buffer := make([]byte, needed)
	var used uint32
	if err = windows.GetTokenInformation(token, windows.TokenGroups, &buffer[0], needed, &used); err != nil || used < uint32(unsafe.Sizeof(windows.Tokengroups{})) || used > needed {
		return errors.Join(errBrokerQueryAccess, err)
	}
	groups := (*windows.Tokengroups)(unsafe.Pointer(&buffer[0]))
	const offset = unsafe.Offsetof(windows.Tokengroups{}.Groups)
	const size = unsafe.Sizeof(windows.SIDAndAttributes{})
	if groups.GroupCount == 0 || uint64(groups.GroupCount) > uint64(used-uint32(offset))/uint64(size) {
		return errBrokerQueryAccess
	}
	found := false
	for _, group := range groups.AllGroups() {
		if group.Sid != nil && group.Sid.IsValid() && group.Sid.Equals(admin) {
			if found || group.Attributes&windows.SE_GROUP_ENABLED == 0 || group.Attributes&windows.SE_GROUP_USE_FOR_DENY_ONLY != 0 {
				return errBrokerQueryAccess
			}
			found = true
		}
	}
	if !found {
		return errBrokerQueryAccess
	}
	return nil
}

func maintenanceFolderDescriptor(original *windows.SECURITY_DESCRIPTOR, raw []byte) (*windows.SECURITY_DESCRIPTOR, error) {
	expected, err := original.ToAbsolute()
	if err != nil {
		return nil, err
	}
	_, defaulted, err := original.DACL()
	if err != nil {
		return nil, err
	}
	if err = expected.SetDACL((*windows.ACL)(unsafe.Pointer(&raw[0])), true, defaulted); err != nil {
		return nil, err
	}
	expected, err = expected.ToSelfRelative()
	runtime.KeepAlive(raw)
	return expected, err
}

func prepareMaintenanceFolderAccessNative(ctx context.Context, access *brokerQueryAccess) (_ *maintenanceFolderAccess, resultErr error) {
	if access == nil {
		return nil, errBrokerQueryAccess
	}
	access.mu.Lock()
	defer access.mu.Unlock()
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if err := access.validateLocked(ctx); err != nil {
		return nil, err
	}
	if access.attributesUsed || access.childUsed || len(access.children) != 0 || access.broker == nil || !access.broker.IsValid() || !strings.HasPrefix(access.broker.String(), "S-1-5-80-") {
		return nil, errBrokerQueryAccess
	}
	if err := maintenanceFolderAdmin(access.token); err != nil {
		return nil, err
	}
	raw, err := brokerQueryACLBytes(access.tokenSD)
	if err != nil {
		return nil, err
	}
	temporary, err := maintenanceFolderACL(raw, brokerQuerySIDBytes(access.broker), access.trusted)
	if err != nil {
		return nil, err
	}
	expected, err := maintenanceFolderDescriptor(access.tokenSD, temporary)
	if err != nil {
		return nil, err
	}
	owner := &maintenanceFolderAccess{access: access, process: access.process, token: access.token, proof: access.proof, processSD: access.processSD, originalSD: access.tokenSD, temporarySD: expected}
	defer func() {
		if resultErr != nil && owner.attempted {
			owner.failure = resultErr
			resultErr = errors.Join(resultErr, owner.closeLocked())
			access.poisoned = errors.Join(access.poisoned, resultErr)
		}
	}()
	// Exact native identity and expected original descriptor immediately before
	// mutation; cancellation here cannot enter the grant interval.
	if err = access.validateLocked(ctx); err != nil {
		return nil, err
	}
	before, err := brokerQueryReadSD(windows.Handle(access.token))
	if err != nil || !brokerQuerySameSD(before, owner.originalSD) {
		return nil, errors.Join(errBrokerQueryAccess, err)
	}
	if err = ctx.Err(); err != nil {
		return nil, err
	}
	owner.attempted = true // no retry after a native mutation attempt
	access.tokenSD = expected
	if err = windows.SetSecurityInfo(windows.Handle(access.token), windows.SE_KERNEL_OBJECT, windows.DACL_SECURITY_INFORMATION, nil, nil, (*windows.ACL)(unsafe.Pointer(&temporary[0])), nil); err != nil {
		return nil, errors.Join(errBrokerQueryAccess, err)
	}
	runtime.KeepAlive(temporary)
	actual, err := brokerQueryReadSD(windows.Handle(access.token))
	if err != nil || !brokerQuerySameSD(expected, actual) {
		return nil, errors.Join(errBrokerQueryAccess, err)
	}
	if err = access.validateLocked(ctx); err != nil {
		return nil, err
	}
	return owner, nil
}

func (o *maintenanceFolderAccess) Validate(ctx context.Context) error {
	if o == nil || o.access == nil {
		return errBrokerQueryAccess
	}
	access := o.access
	access.mu.Lock()
	defer access.mu.Unlock()
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if o.closed || !o.attempted || o.failure != nil {
		return errors.Join(errBrokerQueryAccess, o.failure)
	}
	err := o.custodyLocked()
	if err == nil {
		err = access.validateLocked(ctx)
	}
	if err == nil {
		err = maintenanceFolderAdmin(access.token)
	}
	if err != nil {
		o.failure = errors.Join(errBrokerQueryAccess, err)
		access.poisoned = errors.Join(access.poisoned, o.failure)
	}
	return err
}

// This deliberately ignores request cancellation/poison for restoration, while
// requiring original process/token identity and the exact temporary descriptor.
func (o *maintenanceFolderAccess) custodyLocked() error {
	access := o.access
	if access.closed || access.process != o.process || access.token != o.token || access.proof != o.proof || windows.GetCurrentProcessId() != o.proof.pid || !brokerQuerySameSD(access.tokenSD, o.temporarySD) {
		return errBrokerQueryAccess
	}
	if err := brokerQueryNoImpersonation(); err != nil {
		return err
	}
	return brokerQueryCheck(o.process, o.token, o.proof, o.processSD, o.temporarySD)
}

func (o *maintenanceFolderAccess) closeLocked() error {
	if o.closed {
		return o.closeErr
	}
	o.closed = true
	o.closeErr = errors.Join(o.failure, o.access.poisoned)
	if !o.attempted {
		return o.closeErr
	}
	access := o.access
	if err := o.custodyLocked(); err != nil {
		o.closeErr = errors.Join(errBrokerQueryAccess, o.closeErr, err)
		access.poisoned = errors.Join(access.poisoned, o.closeErr)
		return o.closeErr // Never blindly overwrite an unexpected object/descriptor.
	}
	acl, _, err := o.originalSD.DACL()
	if err == nil {
		err = windows.SetSecurityInfo(windows.Handle(o.token), windows.SE_KERNEL_OBJECT, windows.DACL_SECURITY_INFORMATION, nil, nil, acl, nil)
	}
	actual, readErr := brokerQueryReadSD(windows.Handle(o.token))
	if err != nil || readErr != nil || !brokerQuerySameSD(o.originalSD, actual) {
		o.closeErr = errors.Join(errBrokerQueryAccess, o.closeErr, err, readErr)
		access.poisoned = errors.Join(access.poisoned, o.closeErr)
		return o.closeErr
	}
	access.tokenSD = o.originalSD
	if err = brokerQueryCheck(o.process, o.token, o.proof, o.processSD, o.originalSD); err != nil {
		o.closeErr = errors.Join(errBrokerQueryAccess, o.closeErr, err)
		access.poisoned = errors.Join(access.poisoned, o.closeErr)
	}
	return o.closeErr
}

// Close restores query-only access before the outer query owner drains its
// original handles. A cancelled call still restores if native custody is intact;
// prior uncertainty is retained in the returned error. Cleanup is idempotent.
func (o *maintenanceFolderAccess) Close() error {
	if o == nil {
		return nil
	}
	if o.access == nil {
		return errBrokerQueryAccess
	}
	o.access.mu.Lock()
	defer o.access.mu.Unlock()
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	return o.closeLocked()
}
