package releasepolicy

import (
	"encoding/json"
	"errors"
)

// BrokerCodeUpgradePurpose cannot be supplied by an enrollment policy. Only a
// separately signed target of this purpose can select an existing code pair.
const BrokerCodeUpgradePurpose = "machine-update-broker-code-upgrade"

type BrokerCodeUpgradePolicy struct {
	Schema               int                   `json:"schema"`
	Purpose              string                `json:"purpose"`
	PreviousBrokerSHA256 string                `json:"previousBrokerSha256"`
	PreviousHelperSHA256 string                `json:"previousHelperSha256"`
	Bootstrap            BrokerBootstrapPolicy `json:"bootstrap"`
}

func BrokerCodeUpgradeTargetPath(channel string) (string, error) {
	if _, err := BrokerBootstrapTargetPath(channel); err != nil {
		return "", err
	}
	return "bootstrap/broker-code-upgrade-" + channel + ".json", nil
}

func ParseBrokerCodeUpgrade(data []byte) (BrokerCodeUpgradePolicy, error) {
	var p BrokerCodeUpgradePolicy
	object, err := decodeBrokerBootstrap(data, []string{"schema", "purpose", "previousBrokerSha256", "previousHelperSha256", "bootstrap"}, &p)
	if err != nil {
		return BrokerCodeUpgradePolicy{}, err
	}
	if p.Schema != 1 || p.Purpose != BrokerCodeUpgradePurpose {
		return BrokerCodeUpgradePolicy{}, errors.New("distinct broker code upgrade purpose required")
	}
	for _, pin := range []string{p.PreviousBrokerSHA256, p.PreviousHelperSHA256} {
		if !hex64.MatchString(pin) {
			return BrokerCodeUpgradePolicy{}, errors.New("exact original code upgrade identities required")
		}
	}
	p.Bootstrap, err = ParseBrokerBootstrap(object["bootstrap"])
	if err != nil {
		return BrokerCodeUpgradePolicy{}, err
	}
	if p.Bootstrap.Broker.SHA256 == p.PreviousBrokerSHA256 || p.Bootstrap.Provisioner.SHA256 == p.PreviousHelperSHA256 {
		return BrokerCodeUpgradePolicy{}, errors.New("distinct successor broker and helper required")
	}
	return p, nil
}

func ParseBrokerCodeUpgradeIdentity(data []byte) (BrokerBootstrapIdentity, error) {
	var i BrokerBootstrapIdentity
	if _, err := decodeBrokerBootstrap(data, []string{"purpose", "channel", "version", "commit", "trustConfigSha256"}, &i); err != nil {
		return i, err
	}
	if i.Purpose != BrokerCodeUpgradePurpose {
		return BrokerBootstrapIdentity{}, errors.New("signed code upgrade purpose required")
	}
	base := i
	base.Purpose = BrokerBootstrapPurpose
	if err := validateBrokerBootstrapIdentity(base); err != nil {
		return BrokerBootstrapIdentity{}, err
	}
	return i, nil
}

// Marshal is deliberately not authentication; producers must independently
// sign these exact bytes under the distinct code-upgrade target and identity.
func MarshalBrokerCodeUpgrade(p BrokerCodeUpgradePolicy) ([]byte, error) {
	raw, err := json.Marshal(p)
	if err != nil {
		return nil, err
	}
	_, err = ParseBrokerCodeUpgrade(raw)
	return raw, err
}
