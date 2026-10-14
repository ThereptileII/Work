//go:build !windows

package brokerenroll

import (
	"context"
	"errors"
)

func installedCodeAdmissionNativeOperations() installedCodeAdmissionOperations {
	return installedCodeAdmissionOperations{open: func(context.Context) (installedCodeAdmissionAuthority, error) {
		return nil, errors.New("native Windows installed admission required")
	}, discover: nil}
}

func prepareInstalledCodeSuccessorNative(context.Context, InstalledCodeUpgradeComparisons) (installedCodeSuccessorPreparation, error) {
	return nil, errors.New("native Windows installed successor required")
}
