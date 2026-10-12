//go:build !windows

package launcher

import (
	"context"

	"example.com/opennav-update-verifier/brokerpipe"
)

// CallMaintenanceNative requires actual retained Windows process/token custody.
func CallMaintenanceNative(context.Context, brokerpipe.MaintenanceRequest) (brokerpipe.MaintenanceResult, error) {
	return brokerpipe.MaintenanceResult{}, errMaintenanceClient
}
