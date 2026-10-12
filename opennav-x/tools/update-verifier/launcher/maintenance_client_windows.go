//go:build windows

package launcher

import (
	"context"

	"example.com/opennav-update-verifier/brokerpipe"
)

// CallMaintenanceNative cooperatively exposes only the actual executor to the
// fixed broker for an explicit operator request. Temporary folder-query access
// requires the independently checked elevated primary token. These grants are
// neither peer authentication nor permission to select, restore or delete data;
// the original broker independently verifies the actual registered pipe peer.
func CallMaintenanceNative(ctx context.Context, q brokerpipe.MaintenanceRequest) (brokerpipe.MaintenanceResult, error) {
	var access *brokerQueryAccess
	return callMaintenanceWithCustody(ctx, q, func(ctx context.Context) (maintenanceClientCustody, error) {
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
	}, brokerpipe.CallMaintenance)
}
