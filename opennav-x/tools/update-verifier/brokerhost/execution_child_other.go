//go:build !windows

package brokerhost

import (
	"context"
	"example.com/opennav-update-verifier/brokerstore"
)

func prepareExecutionChildNative(context.Context, executionChildAuthority, *brokerstore.GenerationApplicationImage, brokerstore.GenerationApplicationIdentity, *ExecutionColdProfile) (*ExecutionChildObserver, error) {
	return nil, brokerstore.ErrNativeRequired
}
