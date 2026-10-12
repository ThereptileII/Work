package brokerhost

import (
	"context"
	"errors"
)

// initializeServiceStart interprets SCM start arguments, never executable
// arguments. Initialization is a separate stopped service invocation: it cannot
// advertise Running or open the request pipe. Only the fixed native creator is
// passed here by the Windows host; the callback seam permits portable dispatch
// and cancellation checks without provisioning a machine.
func initializeServiceStart(ctx context.Context, args []string, initialize func() error) (bool, error) {
	if ctx == nil || ctx.Err() != nil {
		return false, errors.New("live service context required")
	}
	if len(args) == 1 && args[0] == ServiceName {
		return false, nil
	}
	if len(args) != 2 || args[0] != ServiceName || args[1] != "initialize-store" {
		return false, errors.New("unexpected broker service arguments")
	}
	if initialize == nil {
		return true, errors.New("native broker initialization required")
	}
	if err := initialize(); err != nil {
		return true, errors.New("protected broker initialization refused")
	}
	if ctx.Err() != nil {
		return true, errors.New("broker initialization interrupted")
	}
	return true, nil
}
