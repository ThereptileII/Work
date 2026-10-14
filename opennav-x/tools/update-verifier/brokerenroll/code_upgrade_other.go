//go:build !windows

package brokerenroll

import (
	"context"
	"errors"
	"example.com/opennav-update-verifier/releasepolicy"
	"example.com/opennav-update-verifier/repository"
)

func openCodeUpgradeNativeMode(context.Context, releasepolicy.BrokerCodeUpgradePolicy, []byte, []byte, string, bool) (codeUpgradeOwner, error) {
	return nil, errors.New("native Windows code upgrade required")
}

func codeUpgradeTrustNativeMode(context.Context, []byte, string, bool) (repository.Trust, error) {
	return repository.Trust{}, errors.New("native original installed trust custody required")
}
