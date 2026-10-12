package brokerhost

import (
	"context"
	"errors"

	"golang.org/x/sys/windows"
)

func reopenProfileSourceNative(ctx context.Context, m profileCaptureManifest) (profileCaptureSource, error) {
	// These are the exact owner/group/DACL strings emitted by the native capture;
	// SACL privileges and mutable caller paths are neither requested nor accepted.
	for _, entry := range m.Entries {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		sd, err := windows.SecurityDescriptorFromString(entry.Security)
		if err != nil || sd.String() != entry.Security {
			return nil, errors.Join(ErrExecutionProfileCaptureRefused, err)
		}
	}
	source, err := captureProfileTreeNative(ctx)
	if err != nil {
		return nil, err
	}
	return bindReopenedProfileSource(ctx, m, source)
}
