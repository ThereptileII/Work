//go:build !windows

package launcher

import "context"

func runBrokerStartupNative(context.Context, string) error { return errBrokerExecutionNative }
