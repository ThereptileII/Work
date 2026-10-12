//go:build !windows

package launcher

import "context"

func runBrokerExecutionNative(context.Context, string) error { return errBrokerExecutionNative }
