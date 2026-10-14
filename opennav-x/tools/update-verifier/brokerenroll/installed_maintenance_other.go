//go:build !windows

package brokerenroll

import (
	"context"
	"errors"
)

func readInstalledCompletionNative(context.Context) (installedMaintenanceInputs, error) {
	return installedMaintenanceInputs{}, errors.New("native Windows installed maintenance required")
}
func prepareInstalledFromCompletionNative(context.Context, *MaintenanceOwner, ApprovedClientBaseline) error {
	return errors.New("native Windows original installed owners required")
}
