//go:build !windows

package brokerenroll

import (
	"context"
	"errors"

	"example.com/opennav-update-verifier/releasepolicy"
)

func openMaintenanceImageNative(context.Context, releasepolicy.Artifact) (maintenanceImageSource, error) {
	return nil, errors.New("original maintenance image custody requires native Windows")
}

func publishMaintenanceImageNative(context.Context, enrollmentFiles, maintenanceImageSource, releasepolicy.Artifact) error {
	return errors.New("original maintenance image publication requires native Windows")
}
