//go:build !windows

package launcher

import "context"

func runBrokerOfferNative(context.Context, string) error { return errBrokerExecutionNative }
