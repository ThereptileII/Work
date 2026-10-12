//go:build !windows

package brokerhost

import "context"

func reopenProfileSourceNative(context.Context, profileCaptureManifest) (profileCaptureSource, error) {
	return nil, ErrExecutionProfileCaptureRefused
}
