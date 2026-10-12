//go:build !windows

package launcher

import "context"

func runBrokerExecutionNative(context.Context, string) error { return errBrokerExecutionNative }

func runBrokerCurrentNative(context.Context, string) error { return errBrokerExecutionNative }
