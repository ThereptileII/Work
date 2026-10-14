//go:build !windows

package brokerpipe

import "context"

func ServeMaintenance(context.Context, MaintenanceHandler) error { return ErrUnavailable }
func ServeMaintenanceWithProtectedUninstall(context.Context, MaintenanceHandler, ProtectedUninstallHandler) error {
	return ErrUnavailable
}
func CallMaintenance(context.Context, MaintenanceRequest) (MaintenanceResult, error) {
	return MaintenanceResult{}, ErrUnavailable
}
