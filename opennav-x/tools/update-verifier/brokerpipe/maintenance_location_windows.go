//go:build windows

package brokerpipe

import (
	"context"
	"errors"
	"strings"

	"golang.org/x/sys/windows"
)

// WithMaintenanceOperatorLocation adds the original operator token's LocalAppData
// to the bounded elevated-operator callback. It never substitutes the service
// user's folder or request data. The callback must obey ctx and must not reenter
// this capability. The location grants no filesystem or maintenance authority.
// Original token folder-query failure (including insufficient retained-token
// rights or an unavailable user hive) refuses without acquiring extra rights.
func (p *PeerCapability) WithMaintenanceOperatorLocation(ctx context.Context, expectedSID string, use func(PeerIdentity, string) error) error {
	if use == nil {
		return ErrRefused
	}
	return p.WithMaintenanceOperatorLocationChecked(ctx, expectedSID, func(identity PeerIdentity, location string, _ func(context.Context) error) error {
		return use(identity, location)
	})
}

// WithMaintenanceOperatorLocationChecked lends the same non-transferable
// per-fact checker while resolving LocalAppData from the retained original
// operator token. No service-user or caller-supplied location is substituted.
func (p *PeerCapability) WithMaintenanceOperatorLocationChecked(ctx context.Context, expectedSID string, use func(PeerIdentity, string, func(context.Context) error) error) error {
	if use == nil {
		return ErrRefused
	}
	return p.WithMaintenanceOperatorChecked(ctx, expectedSID, func(identity PeerIdentity, check func(context.Context) error) error {
		// WithMaintenanceOperator already holds the capability lock and checked the
		// retained original native token; inspecting it here must not reacquire that
		// lock or obtain a token from a new process/PID lookup.
		native := maintenanceNativePeer(p.state.backend)
		if native == nil || native.client == nil || native.client.retainedToken == 0 {
			return ErrRefused
		}
		location, err := native.client.retainedToken.KnownFolderPath(windows.FOLDERID_LocalAppData, 0)
		if err != nil || location == "" {
			return errors.Join(ErrRefused, err)
		}
		// Folder resolution can outlast cancellation or a native peer change. Check
		// the already-locked owner and original/current primary tokens again before
		// granting the callback; no capability method or lock is reentered.
		if err := check(ctx); err != nil {
			return err
		}
		useErr := use(identity, location, check)
		current, locationErr := native.client.retainedToken.KnownFolderPath(windows.FOLDERID_LocalAppData, 0)
		if locationErr != nil || !strings.EqualFold(current, location) {
			locationErr = errors.Join(ErrRefused, locationErr)
		}
		return errors.Join(useErr, locationErr, check(ctx))
	})
}
