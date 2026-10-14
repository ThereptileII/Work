//go:build windows

package launcher

import (
	"context"

	"example.com/opennav-update-verifier/brokerpipe"
)

// CallDecommissionNative prepares only cooperative native query rights for the
// explicit elevated action. Authentication, the durable fence and SCM custody
// remain independently owned by brokerpipe and the original maintenance owner.
func CallDecommissionNative(ctx context.Context, q brokerpipe.MaintenanceRequest) (*brokerpipe.DecommissionWitness, error) {
	if q.Operation != brokerpipe.PrepareDecommission {
		return nil, brokerpipe.ErrRefused
	}
	return callDecommissionNative(ctx, q, brokerpipe.CallDecommission)
}

// CallProtectedUninstallNative retains the same cooperative query/folder token
// grant for the entire two-phase broker conversation. Shortcut restore/drain
// runs under the borrowed fence before retirement; any grant restoration or
// native owner cleanup uncertainty destroys the final SCM witness.
func CallProtectedUninstallNative(ctx context.Context, q brokerpipe.MaintenanceRequest, restoreAndDrain func(context.Context, *brokerpipe.UninstallFenceWitness) error) (*brokerpipe.DecommissionWitness, error) {
	if q.Operation != brokerpipe.PrepareUninstall || restoreAndDrain == nil {
		return nil, brokerpipe.ErrRefused
	}
	return callDecommissionNative(ctx, q, func(ctx context.Context, q brokerpipe.MaintenanceRequest) (*brokerpipe.DecommissionWitness, error) {
		return brokerpipe.CallProtectedUninstall(ctx, q, restoreAndDrain)
	})
}

func callDecommissionNative(ctx context.Context, q brokerpipe.MaintenanceRequest, call decommissionClientCall) (*brokerpipe.DecommissionWitness, error) {
	var access *brokerQueryAccess
	return callDecommissionWithCustody(ctx, q, func(ctx context.Context) (maintenanceClientCustody, error) {
		owner, err := prepareBrokerQueryAccessNative(ctx)
		access = owner
		if owner == nil {
			return nil, err
		}
		return owner, err
	}, func(ctx context.Context) (maintenanceClientCustody, error) {
		owner, err := prepareMaintenanceFolderAccessNative(ctx, access)
		if owner == nil {
			return nil, err
		}
		return owner, err
	}, call)
}
