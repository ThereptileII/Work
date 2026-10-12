//go:build !windows

package brokerpipe

import "context"

func CallDecommission(context.Context, MaintenanceRequest) (*DecommissionWitness, error) {
	return nil, ErrUnavailable
}
