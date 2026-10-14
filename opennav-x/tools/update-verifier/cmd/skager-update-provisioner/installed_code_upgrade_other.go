//go:build !windows

package main

import (
	"context"
	"errors"
)

func installedElevationOperationsForAction(string) installedElevationOperations {
	return installedElevationNativeOperations()
}
func installedCodeSuccessorNativeOperations() installedCodeSuccessorOperations {
	return installedCodeSuccessorOperations{}
}
func launchInstalledCodeSuccessor(context.Context, installedCodeLaunchOwner, installedCodeSnapshot) (installedElevationChild, error) {
	return nil, errors.New("native Windows admitted successor launch required")
}
func retainInstalledCodeUntilExit(installedElevationChild, installedCodeLaunchOwner) error {
	// This platform never creates a native child or transfers launch custody.
	return errors.New("native Windows successor custody required")
}
