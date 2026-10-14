package verifier

import (
	"bytes"
	"errors"
	"example.com/opennav-update-verifier/releasepolicy"
	"strings"
	"testing"
	"time"

	"github.com/theupdateframework/go-tuf/v2/metadata"
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

func TestBrokerCodeUpgradeDiscoveryAuthenticatedSuccessor(t *testing.T) {
	for _, fault := range []string{"valid", "custom-purpose", "policy-purpose", "schema", "missing-predecessor", "wildcard-predecessor", "unchanged-broker", "unchanged-helper", "foreign-origin", "custom-commit", "trust", "expired"} {
		t.Run(fault, func(t *testing.T) {
			f, b, original := bootstrapFixture(t)
			p := releasepolicy.BrokerCodeUpgradePolicy{Schema: 1, Purpose: releasepolicy.BrokerCodeUpgradePurpose, PreviousBrokerSHA256: strings.Repeat("e", 64), PreviousHelperSHA256: strings.Repeat("f", 64), Bootstrap: b}
			identity := bootstrapIdentity(b)
			identity.Purpose = releasepolicy.BrokerCodeUpgradePurpose
			req := BrokerCodeUpgradeDiscoveryRequest{MetadataURL: original.MetadataURL, TrustedRoot: original.TrustedRoot, CacheDir: original.CacheDir, Channel: original.Channel, ArtifactOrigin: original.ArtifactOrigin, TrustConfigSHA256: original.TrustConfigSHA256}
			switch fault {
			case "custom-purpose":
				identity.Purpose = releasepolicy.BrokerBootstrapPurpose
			case "policy-purpose":
				p.Purpose = releasepolicy.BrokerBootstrapPurpose
			case "schema":
				p.Schema = 2
			case "missing-predecessor":
				p.PreviousHelperSHA256 = ""
			case "wildcard-predecessor":
				p.PreviousBrokerSHA256 = "*"
			case "unchanged-broker":
				p.PreviousBrokerSHA256 = b.Broker.SHA256
			case "unchanged-helper":
				p.PreviousHelperSHA256 = b.Provisioner.SHA256
			case "foreign-origin":
				p.Bootstrap.Provisioner.URL = "https://foreign.invalid/" + p.Bootstrap.Provisioner.Path
			case "custom-commit":
				identity.Commit = strings.Repeat("1", 40)
			case "trust":
				req.TrustConfigSHA256 = strings.Repeat("1", 64)
			}
			raw := bootstrapBytes(t, p)
			path, _ := releasepolicy.BrokerCodeUpgradeTargetPath("beta")
			publishBootstrap(t, f, 2, raw, bootstrapBytes(t, identity), path, fault == "expired")
			got, verified, err := discoverBrokerCodeUpgradeWithClient(req, time.Now().Add(maxOperationTime), f.server.Client())
			if fault != "valid" {
				if err == nil || verified != nil || got != (releasepolicy.BrokerCodeUpgradePolicy{}) {
					t.Fatal("unauthenticated discovery escaped", fault, err)
				}
				return
			}
			if err != nil || got != p || !bytes.Equal(verified, raw) {
				t.Fatal("signed discovered tuple changed", err)
			}
			verified[0] = 'x'
			if !bytes.Equal(f.files["/targets/"+path], raw) {
				t.Fatal("discovered bytes alias source")
			}
			// Discovery does not relax the existing explicitly approved entry.
			original.ApprovedPolicySHA256 = strings.Repeat("1", 64)
			if _, data, err := verifyBrokerCodeUpgradeWithClient(original, time.Now().Add(maxOperationTime), f.server.Client()); err == nil || data != nil {
				t.Fatal("discovery bypassed independent operator approval")
			}
		})
	}
}

func TestBrokerCodeUpgradeDiscoveryAbsenceRequiresFreshNondelegatedSignedMetadata(t *testing.T) {
	for _, fault := range []string{"absent", "expired", "offline", "missing-timestamp", "unsigned-absence", "delegated", "advertised-target-missing", "rollback"} {
		t.Run(fault, func(t *testing.T) {
			f, b, original := bootstrapFixture(t)
			req := BrokerCodeUpgradeDiscoveryRequest{MetadataURL: original.MetadataURL, TrustedRoot: original.TrustedRoot, CacheDir: original.CacheDir, Channel: original.Channel, ArtifactOrigin: original.ArtifactOrigin, TrustConfigSHA256: original.TrustConfigSHA256}
			p := releasepolicy.BrokerCodeUpgradePolicy{Schema: 1, Purpose: releasepolicy.BrokerCodeUpgradePurpose, PreviousBrokerSHA256: strings.Repeat("e", 64), PreviousHelperSHA256: strings.Repeat("f", 64), Bootstrap: b}
			raw := bootstrapBytes(t, p)
			identity := bootstrapIdentity(b)
			identity.Purpose = releasepolicy.BrokerCodeUpgradePurpose
			upgradePath, _ := releasepolicy.BrokerCodeUpgradeTargetPath("beta")
			otherPath, _ := releasepolicy.BrokerBootstrapTargetPath("beta")
			if fault == "rollback" {
				publishBootstrap(t, f, 3, raw, bootstrapBytes(t, identity), upgradePath, false)
				if _, _, err := discoverBrokerCodeUpgradeWithClient(req, time.Now().Add(maxOperationTime), f.server.Client()); err != nil {
					t.Fatal("cannot establish authenticated floor", err)
				}
			}
			path := otherPath
			if fault == "advertised-target-missing" || fault == "unsigned-absence" {
				path = upgradePath
			}
			publishBootstrap(t, f, 2, raw, bootstrapBytes(t, identity), path, fault == "expired")
			switch fault {
			case "offline":
				f.server.Close()
			case "missing-timestamp":
				delete(f.files, "/timestamp.json")
			case "advertised-target-missing":
				delete(f.files, "/targets/"+upgradePath)
			case "delegated", "unsigned-absence":
				targets, err := metadata.Targets().FromBytes(f.files["/targets.json"])
				if err != nil {
					t.Fatal(err)
				}
				if fault == "delegated" {
					targets.Signed.Delegations = &metadata.Delegations{Keys: map[string]*metadata.Key{}}
					targets.Signatures = nil
					sign(t, targets, f.keys["targets"])
				} else {
					delete(targets.Signed.Targets, upgradePath) // Deliberately unsigned mutation.
				}
				f.files["/targets.json"] = bytesOf(t, targets)
			}
			got, data, err := discoverBrokerCodeUpgradeWithClient(req, time.Now().Add(maxOperationTime), f.server.Client())
			if err == nil || data != nil || got != (releasepolicy.BrokerCodeUpgradePolicy{}) {
				t.Fatal("absence or failed discovery fabricated a successor", err)
			}
			if errors.Is(err, ErrAuthenticatedTargetAbsent) != (fault == "absent") {
				t.Fatal("only fresh signed nondelegated absence may become no update", fault, err)
			}
			if fault == "delegated" && !strings.HasPrefix(err.Error(), "target metadata: ") {
				t.Fatal("delegated fixture must pass refresh before its refused absence", err)
			}
		})
	}
}
