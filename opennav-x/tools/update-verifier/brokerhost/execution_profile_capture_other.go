//go:build !windows

package brokerhost

import "context"

func captureProfileTreeNative(context.Context) (profileCaptureSource, error) {
	return nil, ErrExecutionProfileCaptureRefused
}
