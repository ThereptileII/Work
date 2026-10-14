package verifier

import (
	"bytes"
	"example.com/opennav-update-verifier/releasepolicy"
	"strings"
	"testing"
	"time"
)

func TestBrokerCodeUpgradeSignedTargetIsolation(t *testing.T) {
	for _, fault := range []string{"valid", "enrollment-custom-purpose", "enrollment-target-path", "policy-pin", "provisioner-pin", "trust-pin", "custom-commit", "old-pair-tamper", "expired"} {
		t.Run(fault, func(t *testing.T) {
			f, b, req := bootstrapFixture(t)
			p := releasepolicy.BrokerCodeUpgradePolicy{Schema: 1, Purpose: releasepolicy.BrokerCodeUpgradePurpose, PreviousBrokerSHA256: strings.Repeat("e", 64), PreviousHelperSHA256: strings.Repeat("f", 64), Bootstrap: b}
			raw := bootstrapBytes(t, p)
			identity := bootstrapIdentity(b)
			identity.Purpose = releasepolicy.BrokerCodeUpgradePurpose
			path, _ := releasepolicy.BrokerCodeUpgradeTargetPath("beta")
			req.ApprovedPolicySHA256 = bootstrapDigest(raw)
			switch fault {
			case "enrollment-custom-purpose":
				identity.Purpose = releasepolicy.BrokerBootstrapPurpose
			case "enrollment-target-path":
				path, _ = releasepolicy.BrokerBootstrapTargetPath("beta")
			case "policy-pin":
				req.ApprovedPolicySHA256 = strings.Repeat("1", 64)
			case "provisioner-pin":
				req.TrustedProvisionerSHA256 = strings.Repeat("1", 64)
			case "trust-pin":
				req.TrustConfigSHA256 = strings.Repeat("1", 64)
			case "custom-commit":
				identity.Commit = strings.Repeat("1", 40)
			case "old-pair-tamper":
				p.PreviousBrokerSHA256 = strings.Repeat("1", 64)
				raw = bootstrapBytes(t, p)
			}
			publishBootstrap(t, f, 2, raw, bootstrapBytes(t, identity), path, fault == "expired")
			got, verified, e := verifyBrokerCodeUpgradeWithClient(req, time.Now().Add(maxOperationTime), f.server.Client())
			if fault == "valid" {
				if e != nil || got != p || !bytes.Equal(raw, verified) {
					t.Fatal("verified tuple lost", e)
				}
			} else if e == nil {
				t.Fatal("upgrade target isolation failed", fault)
			}
		})
	}
}
