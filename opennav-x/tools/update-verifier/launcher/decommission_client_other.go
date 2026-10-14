//go:build !windows

package launcher

import (
	"context"
	"example.com/opennav-update-verifier/brokerpipe"
)

func CallDecommissionNative(_ context.Context, q brokerpipe.MaintenanceRequest) (*brokerpipe.DecommissionWitness, error) {
	if q.Operation == brokerpipe.PrepareUninstall {
		return nil, brokerpipe.ErrRefused
	}
	return nil, brokerpipe.ErrUnavailable
}

func CallProtectedUninstallNative(context.Context, brokerpipe.MaintenanceRequest, func(context.Context, *brokerpipe.UninstallFenceWitness) error) (*brokerpipe.DecommissionWitness, error) {
	return nil, brokerpipe.ErrUnavailable
}
