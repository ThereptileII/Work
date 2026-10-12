//go:build !windows

package brokerenroll

import (
	"context"
	"errors"
)

func maintenanceOperatorNative(string) error {
	return errors.New("native Windows maintenance operator required")
}
func openMaintenanceNative(context.Context, maintenanceAuthority) (maintenanceBackend, error) {
	return nil, errors.New("retained native Windows service/process maintenance required")
}
