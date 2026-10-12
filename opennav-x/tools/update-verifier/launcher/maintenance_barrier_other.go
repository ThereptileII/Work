//go:build !windows

package launcher

import "context"

// MaintenanceBarrier requires retained native Windows directory custody.
type MaintenanceBarrier struct{}

func CaptureMaintenanceBarrierNative(context.Context, string) (*MaintenanceBarrier, error) {
	return nil, errArmedUpdateBarrier
}
func (*MaintenanceBarrier) Validate(context.Context) error { return errArmedUpdateBarrier }
func (*MaintenanceBarrier) Close() error                   { return nil }
