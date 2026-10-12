//go:build windows

package brokerpipe

import (
	"context"
	"errors"
	"runtime"
	"sync/atomic"
	"unsafe"

	"golang.org/x/sys/windows"
)

// WithMaintenanceOperator permits one bounded use of the actual pipe peer only
// when its retained and current PRIMARY process tokens are the same elevated,
// enabled-administrator token of expectedClientSID. The caller must obtain that
// SID from the protected current registration, never from the request. This
// does not authorize a maintenance transaction or confer filesystem authority.
// The callback must obey ctx and must not reenter this capability.
func (p *PeerCapability) WithMaintenanceOperator(ctx context.Context, expectedClientSID string, use func(PeerIdentity) error) error {
	if p == nil || p.state == nil || use == nil || expectedClientSID == "" {
		return ErrRefused
	}
	p.state.mu.Lock()
	defer p.state.mu.Unlock()
	native := maintenanceNativePeer(p.state.backend)
	if native == nil {
		return ErrRefused // Portable/private backends cannot grant operator authority.
	}
	// A maintenance callback must never inherit the pipe-identification thread's
	// impersonation. Keep the checked service thread throughout this use.
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if err := p.state.checkLocked(ctx); err != nil {
		return err
	}
	if err := maintenanceOperatorToken(native, p.state.identity, expectedClientSID); err != nil {
		p.state.failure = errors.Join(ErrPeerRevoked, err)
		return p.state.failure
	}
	if err := p.state.checkLocked(ctx); err != nil {
		return err
	}
	useErr := use(p.state.identity)
	checkErr := p.state.checkLocked(ctx)
	if checkErr == nil {
		checkErr = maintenanceOperatorToken(native, p.state.identity, expectedClientSID)
		if checkErr != nil {
			p.state.failure = errors.Join(ErrPeerRevoked, checkErr)
			checkErr = p.state.failure
		} else {
			checkErr = p.state.checkLocked(ctx)
		}
	}
	return errors.Join(useErr, checkErr)
}

// WithMaintenanceOperatorChecked lends a checker only for this callback on its
// original locked OS thread. The checker is deliberately not a transferable
// capability: it refuses after return, on another thread, or during concurrent
// use. Call it immediately before each durable maintenance fact.
func (p *PeerCapability) WithMaintenanceOperatorChecked(ctx context.Context, expectedClientSID string, use func(PeerIdentity, func(context.Context) error) error) error {
	if use == nil {
		return ErrRefused
	}
	return p.WithMaintenanceOperator(ctx, expectedClientSID, func(identity PeerIdentity) error {
		native := maintenanceNativePeer(p.state.backend) // capability mutex is held
		thread := windows.GetCurrentThreadId()
		var active, checking atomic.Bool
		active.Store(true)
		defer active.Store(false)
		check := func(checkCtx context.Context) error {
			// Check lease and thread before touching the mutex-protected state. The
			// outer callback owns the mutex and pins this goroutine to the thread.
			if !active.Load() || windows.GetCurrentThreadId() != thread || !checking.CompareAndSwap(false, true) {
				return ErrRefused
			}
			defer checking.Store(false)
			if err := p.state.checkLocked(checkCtx); err != nil {
				return err
			}
			if err := maintenanceOperatorToken(native, identity, expectedClientSID); err != nil {
				p.state.failure = errors.Join(ErrPeerRevoked, err)
				return p.state.failure
			}
			return p.state.checkLocked(checkCtx)
		}
		if err := check(ctx); err != nil {
			return err
		}
		return errors.Join(use(identity, check), check(ctx))
	})
}

func maintenanceNativePeer(backend peerCapabilityBackend) *nativePeerCapability {
	switch b := backend.(type) {
	case *nativePeerCapability:
		return b
	case *executionNativePeer:
		if b != nil && (b.incoming == nil || !b.incoming.Load()) {
			return b.native
		}
	}
	return nil
}

type maintenanceTokenProof struct {
	tokenID windows.LUID
	logon   windows.LUID
}

func maintenanceOperatorToken(native *nativePeerCapability, identity PeerIdentity, expectedSID string) error {
	if native == nil || native.client == nil || identity.SID != expectedSID || identity.SessionID == 0 || identity.ProcessID != native.client.pid {
		return ErrRefused
	}
	sid, err := windows.StringToSid(expectedSID)
	if err != nil || sid.String() != expectedSID {
		return ErrRefused
	}
	if err := maintenanceNoThreadImpersonation(); err != nil {
		return err
	}
	client := native.client
	if err := client.live(); err != nil {
		return err
	}
	original, err := maintenancePrimaryToken(client.retainedToken, expectedSID, identity)
	if err != nil {
		return err
	}
	var current windows.Token
	if err := windows.OpenProcessToken(client.handle, windows.TOKEN_QUERY, &current); err != nil {
		return err
	}
	actual, checkErr := maintenancePrimaryToken(current, expectedSID, identity)
	closeErr := current.Close()
	if checkErr != nil || closeErr != nil || actual != original {
		return errors.Join(ErrRefused, checkErr, closeErr)
	}
	return client.live()
}

func maintenanceNoThreadImpersonation() error {
	var thread windows.Token
	err := windows.OpenThreadToken(windows.CurrentThread(), windows.TOKEN_QUERY, true, &thread)
	if err == nil {
		return errors.Join(ErrRefused, thread.Close())
	}
	if !errors.Is(err, windows.ERROR_NO_TOKEN) {
		return errors.Join(ErrRefused, err)
	}
	return nil
}

func maintenancePrimaryToken(token windows.Token, expectedSID string, identity PeerIdentity) (maintenanceTokenProof, error) {
	var proof maintenanceTokenProof
	actual, err := tokenInfo(token)
	if err != nil || actual.sid != expectedSID || actual.session != identity.SessionID || actual.logon.HighPart != identity.LogonIDHigh || actual.logon.LowPart != identity.LogonIDLow {
		return proof, errors.Join(ErrRefused, err)
	}
	var stats tokenStatistics
	var used uint32
	if err := windows.GetTokenInformation(token, windows.TokenStatistics, (*byte)(unsafe.Pointer(&stats)), uint32(unsafe.Sizeof(stats)), &used); err != nil || used != uint32(unsafe.Sizeof(stats)) || stats.TokenType != windows.TokenPrimary || stats.AuthenticationID != actual.logon {
		return proof, errors.Join(ErrRefused, err)
	}
	if !token.IsElevated() {
		return proof, ErrRefused
	}
	admin, err := windows.CreateWellKnownSid(windows.WinBuiltinAdministratorsSid)
	if err != nil {
		return proof, err
	}
	member, err := maintenanceEnabledAdminGroup(token, admin)
	if err != nil || !member {
		return proof, errors.Join(ErrRefused, err)
	}
	proof.tokenID, proof.logon = stats.TokenID, stats.AuthenticationID
	return proof, nil
}

// CheckTokenMembership requires an impersonation token. Inspect the actual
// primary token groups instead, with a fixed bound before any allocation.
func maintenanceEnabledAdminGroup(token windows.Token, admin *windows.SID) (bool, error) {
	if admin == nil || !admin.IsValid() {
		return false, ErrRefused
	}
	var needed uint32
	err := windows.GetTokenInformation(token, windows.TokenGroups, nil, 0, &needed)
	const maxTokenGroupsBytes = 64 << 10
	if !errors.Is(err, windows.ERROR_INSUFFICIENT_BUFFER) || needed < uint32(unsafe.Sizeof(windows.Tokengroups{})) || needed > maxTokenGroupsBytes {
		return false, ErrRefused
	}
	buffer := make([]byte, needed)
	var used uint32
	if err := windows.GetTokenInformation(token, windows.TokenGroups, &buffer[0], needed, &used); err != nil || used < uint32(unsafe.Sizeof(windows.Tokengroups{})) || used > needed {
		return false, errors.Join(ErrRefused, err)
	}
	groups := (*windows.Tokengroups)(unsafe.Pointer(&buffer[0]))
	const offset = unsafe.Offsetof(windows.Tokengroups{}.Groups)
	const entrySize = unsafe.Sizeof(windows.SIDAndAttributes{})
	if groups.GroupCount == 0 || uint64(groups.GroupCount) > uint64(used-uint32(offset))/uint64(entrySize) {
		return false, ErrRefused
	}
	return maintenanceAdminGroupEnabled(groups.AllGroups(), admin), nil
}

func maintenanceAdminGroupEnabled(groups []windows.SIDAndAttributes, admin *windows.SID) bool {
	found := false
	for _, group := range groups {
		if group.Sid == nil || !group.Sid.IsValid() || !group.Sid.Equals(admin) {
			continue
		}
		if found || group.Attributes&windows.SE_GROUP_ENABLED == 0 || group.Attributes&windows.SE_GROUP_USE_FOR_DENY_ONLY != 0 {
			return false
		}
		found = true
	}
	return found
}
