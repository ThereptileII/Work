package verifier

import (
	"bytes"
	"crypto/sha256"
	"encoding/hex"
	"errors"
	"net/http"
	"strings"
	"time"

	"example.com/opennav-update-verifier/releasepolicy"
)

// VerifyBrokerCodeUpgrade shares durable go-tuf refresh, rollback floors and
// redirect refusal with enrollment. The distinct target and custom purpose are
// mandatory; a signed enrollment policy never grants existing-code replacement.
func VerifyBrokerCodeUpgrade(req BrokerBootstrapRequest) (releasepolicy.BrokerCodeUpgradePolicy, []byte, error) {
	return verifyBrokerCodeUpgradeWithClient(req, time.Now().Add(maxOperationTime), nil)
}
func verifyBrokerCodeUpgradeWithClient(req BrokerBootstrapRequest, deadline time.Time, client *http.Client) (releasepolicy.BrokerCodeUpgradePolicy, []byte, error) {
	var empty releasepolicy.BrokerCodeUpgradePolicy
	for _, pin := range []string{req.ApprovedPolicySHA256, req.TrustedProvisionerSHA256, req.TrustConfigSHA256} {
		if !validBootstrapSHA256(pin) {
			return empty, nil, errors.New("independent upgrade policy, provisioner and trust pins required")
		}
	}
	p, raw, err := discoverBrokerCodeUpgradeWithClient(BrokerCodeUpgradeDiscoveryRequest{
		MetadataURL: req.MetadataURL, TrustedRoot: req.TrustedRoot, CacheDir: req.CacheDir,
		Channel: req.Channel, ArtifactOrigin: req.ArtifactOrigin, TrustConfigSHA256: req.TrustConfigSHA256,
	}, deadline, client)
	if err != nil {
		return empty, nil, err
	}
	if brokerUpgradeDigest(raw) != req.ApprovedPolicySHA256 {
		return empty, nil, errors.New("upgrade policy differs from independent approval")
	}
	if p.Bootstrap.Provisioner.SHA256 != req.TrustedProvisionerSHA256 {
		return empty, nil, errors.New("signed upgrade tuple differs from policy or provisioner approval")
	}
	return p, raw, nil
}

// BrokerCodeUpgradeDiscoveryRequest carries authority from an already authenticated
// installed helper's original trust and protected durable cache. Callers must not
// obtain these inputs from an untrusted adjacent declaration.
type BrokerCodeUpgradeDiscoveryRequest struct {
	MetadataURL       string
	TrustedRoot       []byte
	CacheDir          string
	Channel           string
	ArtifactOrigin    string
	TrustConfigSHA256 string
}

// DiscoverBrokerCodeUpgrade authenticates the distinct signed successor target
// without requiring its policy or executable hash in advance. It returns owned
// exact target bytes and a parsed value; it neither downloads nor executes code.
// The caller must bind both predecessor hashes to genuine installed completion,
// retain downloaded image/loader custody and authenticate the execution handoff.
func DiscoverBrokerCodeUpgrade(req BrokerCodeUpgradeDiscoveryRequest) (releasepolicy.BrokerCodeUpgradePolicy, []byte, error) {
	return discoverBrokerCodeUpgradeWithClient(req, time.Now().Add(maxOperationTime), nil)
}

func discoverBrokerCodeUpgradeWithClient(req BrokerCodeUpgradeDiscoveryRequest, deadline time.Time, client *http.Client) (releasepolicy.BrokerCodeUpgradePolicy, []byte, error) {
	var empty releasepolicy.BrokerCodeUpgradePolicy
	if !validBootstrapSHA256(req.TrustConfigSHA256) {
		return empty, nil, errors.New("original authenticated upgrade trust identity required")
	}
	if err := validateBootstrapOrigin(req.ArtifactOrigin); err != nil {
		return empty, nil, err
	}
	path, err := releasepolicy.BrokerCodeUpgradeTargetPath(req.Channel)
	if err != nil {
		return empty, nil, err
	}
	request := Request{MetadataURL: req.MetadataURL, TrustedRoot: req.TrustedRoot, CacheDir: req.CacheDir, Channel: req.Channel, TargetPath: path}
	metadataURL, err := validateTUFRequest(request)
	if err != nil {
		return empty, nil, err
	}
	if validateBootstrapOrigin(metadataURL.Scheme+"://"+metadataURL.Host) != nil || metadataURL.ForceQuery || metadataURL.RawPath != "" || metadataURL.Opaque != "" || strings.ContainsAny(req.MetadataURL, "\\#") {
		return empty, nil, errors.New("exact upgrade metadata HTTPS URL required")
	}
	var identity releasepolicy.BrokerBootstrapIdentity
	raw, err := verifyTargetWithClient(request, deadline, client, true, func(custom []byte) error {
		var err error
		identity, err = releasepolicy.ParseBrokerCodeUpgradeIdentity(custom)
		if err != nil {
			return err
		}
		if identity.Channel != req.Channel || identity.TrustConfigSHA256 != req.TrustConfigSHA256 {
			return errors.New("signed upgrade identity differs from approval")
		}
		return nil
	})
	if err != nil {
		return empty, nil, err
	}
	p, err := releasepolicy.ParseBrokerCodeUpgrade(raw)
	if err != nil {
		return empty, nil, err
	}
	b := p.Bootstrap
	if b.Channel != identity.Channel || b.Version != identity.Version || b.Commit != identity.Commit || b.TrustConfigSHA256 != identity.TrustConfigSHA256 {
		return empty, nil, errors.New("signed upgrade tuple differs from policy or provisioner approval")
	}
	for _, a := range []releasepolicy.Artifact{b.Broker, b.Provisioner, b.CorrespondingSource, b.Notices} {
		if validateDownload(a, req.ArtifactOrigin) != nil {
			return empty, nil, errors.New("upgrade artifact differs from approved origin")
		}
	}
	return p, bytes.Clone(raw), nil
}

func brokerUpgradeDigest(raw []byte) string { h := sha256.Sum256(raw); return hex.EncodeToString(h[:]) }
