//go:build !windows

package brokerenroll

import (
	"context"
	"errors"
)

func openStartupRestorationNative(context.Context, maintenanceAuthority, startupBindingRecord, []byte) (startupRestorationBackend, error) {
	return nil, errors.New("retained native Windows shortcut restoration required")
}
