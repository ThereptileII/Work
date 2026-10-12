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
	}, brokerpipe.CallDecommission)
}
