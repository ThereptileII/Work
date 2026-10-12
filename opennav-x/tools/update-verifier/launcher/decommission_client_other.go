//go:build !windows

package launcher

import (
	"context"
	"example.com/opennav-update-verifier/brokerpipe"
)

func CallDecommissionNative(context.Context, brokerpipe.MaintenanceRequest) (*brokerpipe.DecommissionWitness, error) {
	return nil, brokerpipe.ErrUnavailable
}
