//go:build windows

package main

import (
	"context"
	"errors"
	"example.com/opennav-update-verifier/brokerpipe"
)

func confirmCustomerRepairNative(ctx context.Context, plan brokerpipe.MaintenanceResult) (bool, error) {
	if ctx == nil || ctx.Err() != nil || plan.Version == "" {
		return false, errors.New("live authenticated repair confirmation required")
	}
	return confirmCustomerMaintenanceNative(ctx, "Confirm SKAGER repair", "Repair SKAGER "+plan.Version+"?\n\nBuild: "+plan.Commit+"\n\nThe selected application files will be restored from their original authenticated retained bytes. Current charts, routes, waypoints, plugins, connections and profile data are preserved. Damaged files and custom additions remain in the old generation. The application stays closed; Repair does not claim a new healthy start.")
}
