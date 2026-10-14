//go:build !windows

package brokerpipe

import "context"

func CallDecommission(_ context.Context, q MaintenanceRequest) (*DecommissionWitness, error) {
	if q.Operation == PrepareUninstall {
		return nil, ErrRefused
	}
	return nil, ErrUnavailable
}

func CallProtectedUninstall(context.Context, MaintenanceRequest, func(context.Context, *UninstallFenceWitness) error) (*DecommissionWitness, error) {
	return nil, ErrUnavailable
}
