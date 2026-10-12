package brokerpipe

import (
	"errors"
	"runtime"

	"golang.org/x/sys/windows"
)

// The operator scope already retains the original native process and token.
// Open only documented folder-query rights from that same process handle, then
// require the exact original primary TokenID/logon proof. No PID lookup, grant,
// token duplication, privilege adjustment or thread impersonation occurs here.
// Caller owns the returned token and must close it inside the operator scope.
func openMaintenanceFolderToken(native *nativePeerCapability, identity PeerIdentity, expectedSID string) (result windows.Token, resultErr error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if err := maintenanceOperatorToken(native, identity, expectedSID); err != nil {
		return 0, err
	}
	client := native.client
	if identity.ProcessCreation != uint64(client.created.HighDateTime)<<32|uint64(client.created.LowDateTime) {
		return 0, ErrRefused
	}
	original, err := maintenancePrimaryToken(client.retainedToken, expectedSID, identity)
	if err != nil {
		return 0, err
	}
	var token windows.Token
	if err = windows.OpenProcessToken(client.handle, windows.TOKEN_QUERY|windows.TOKEN_IMPERSONATE, &token); err != nil {
		return 0, err
	}
	defer func() {
		if resultErr != nil {
			resultErr = errors.Join(resultErr, token.Close())
		}
	}()
	actual, err := maintenancePrimaryToken(token, expectedSID, identity)
	if err != nil || actual != original {
		return 0, errors.Join(ErrRefused, err)
	}
	if err = client.live(); err != nil {
		return 0, err
	}
	if err = maintenanceNoThreadImpersonation(); err != nil {
		return 0, err
	}
	return token, nil
}
