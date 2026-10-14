//go:build !windows

package brokerenroll

import (
	"context"
	"errors"
)

func openInstalledUninstallNative(context.Context, maintenanceAuthority, string) (installedUninstallBackend, error) {
	return nil, errors.New("retained installed teardown requires native Windows")
}
