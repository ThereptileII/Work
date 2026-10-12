//go:build !windows

package brokerenroll

import (
	"context"
	"errors"
	"example.com/opennav-update-verifier/brokerregistration"
)

func startupCurrentClientNative(context.Context, []byte, string) error {
	return errors.New("native Windows current-user startup enrollment required")
}
func initializeStartupNative(context.Context, enrollmentFiles, *brokerregistration.Request, ApprovedClientBaseline) error {
	return errors.New("native Windows original service startup required")
}
