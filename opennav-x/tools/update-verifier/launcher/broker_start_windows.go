//go:build windows

package launcher

import "context"

// Private admission entry until installed shortcut/provisioning is joined. The
// canonical baseline ID is routing data; only the broker's original registration,
// native executor and selected history choose qualification or terminal relay.
func runBrokerStartupNative(ctx context.Context, baseline string) error {
	return runBrokerIntentNative(ctx, baseline, "start-selected")
}
