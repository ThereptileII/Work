//go:build !windows

package main

import (
	"context"
	"errors"
)

func installedElevationNativeOperations() installedElevationOperations {
	return installedElevationOperations{
		openSelf: func(context.Context) (installedElevationSelf, error) {
			return installedElevationSelf{}, errors.New("native Windows installed elevation required")
		},
		openParent: func(context.Context, installedElevationRequest, installedElevationIdentity) (installedElevationOwner, error) {
			return nil, errors.New("native Windows installed parent required")
		},
		launch: func(context.Context, installedElevationIdentity, []string) (installedElevationChild, error) {
			return nil, errors.New("native Windows installed elevation required")
		},
	}
}
