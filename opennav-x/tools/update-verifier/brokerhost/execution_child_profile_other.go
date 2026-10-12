//go:build !windows

package brokerhost

import (
	"context"
	"example.com/opennav-update-verifier/brokerstore"
)

func captureExecutionColdProfileNative(context.Context) (*ExecutionColdProfile, error) {
	return nil, brokerstore.ErrNativeRequired
}
