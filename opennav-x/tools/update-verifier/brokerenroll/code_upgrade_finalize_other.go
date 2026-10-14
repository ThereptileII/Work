//go:build !windows

package brokerenroll

import (
	"context"
	"errors"
	"example.com/opennav-update-verifier/releasepolicy"
)

func openCodeResumeNative(context.Context, releasepolicy.BrokerCodeUpgradePolicy, []byte, []byte, string) (codeUpgradeResumeOwner, error) {
	return nil, errors.New("native authenticated code resume required")
}
