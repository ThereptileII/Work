//go:build !windows

package main

import (
	"context"
	"errors"
	"example.com/opennav-update-verifier/brokerpipe"
)

func confirmCustomerRepairNative(context.Context, brokerpipe.MaintenanceResult) (bool, error) {
	return false, errors.New("native Windows repair confirmation required")
}
