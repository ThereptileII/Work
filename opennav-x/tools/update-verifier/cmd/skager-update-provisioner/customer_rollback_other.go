//go:build !windows

package main

import (
	"context"
	"errors"
	"example.com/opennav-update-verifier/brokerpipe"
)

func confirmCustomerRollbackNative(context.Context, brokerpipe.MaintenanceResult) (bool, error) {
	return false, errors.New("native Windows customer confirmation required")
}
