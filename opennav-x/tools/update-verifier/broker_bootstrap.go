package verifier

import (
	"bytes"
	"crypto/sha256"
	"encoding/hex"
	"errors"
	"fmt"
	"net/http"
	"net/url"
	"strconv"
	"strings"
	"time"

	"example.com/opennav-update-verifier/releasepolicy"
)

// BrokerBootstrapRequest requires independent caller approval of the exact
// declaration, provisioner and trust configuration before any network request.
// TrustedRoot must already be pinned independently; CacheDir and its ancestors
// must remain protected by the caller for durable TUF rollback protection.
// These pins do not authenticate the initial verifier or obtain bootstrap consent.
type BrokerBootstrapRequest struct {
	MetadataURL              string
	TrustedRoot              []byte
	CacheDir                 string
	Channel                  string
	ArtifactOrigin           string
	ApprovedPolicySHA256     string
	TrustedProvisionerSHA256 string
	TrustConfigSHA256        string
}

// VerifyBrokerBootstrap reads the distinct signed broker-enrollment target using
// real go-tuf refresh, rollback checks and bounded target verification. It returns
// an owned copy of the exact approved bytes and their strictly parsed declaration.
// All metadata and target redirects are refused. Declared artifacts are checked
// against the caller's origin and provisioner pin but are not downloaded here.
// Nothing is installed, executed, elevated or granted service permission.
func VerifyBrokerBootstrap(req BrokerBootstrapRequest) (releasepolicy.BrokerBootstrapPolicy, []byte, error) {
	return verifyBrokerBootstrapWithClient(req, time.Now().Add(maxOperationTime), nil)
}

// Only local signed TLS fixtures inject their test certificate's client.
func verifyBrokerBootstrapWithClient(req BrokerBootstrapRequest, deadline time.Time, client *http.Client) (releasepolicy.BrokerBootstrapPolicy, []byte, error) {
	var empty releasepolicy.BrokerBootstrapPolicy
	for _, pin := range []string{req.ApprovedPolicySHA256, req.TrustedProvisionerSHA256, req.TrustConfigSHA256} {
		if !validBootstrapSHA256(pin) {
			return empty, nil, errors.New("independent policy, provisioner and trust-config SHA-256 pins required")
		}
	}
	if err := validateBootstrapOrigin(req.ArtifactOrigin); err != nil {
		return empty, nil, err
	}
	targetPath, err := releasepolicy.BrokerBootstrapTargetPath(req.Channel)
	if err != nil {
		return empty, nil, err
	}
	request := Request{MetadataURL: req.MetadataURL, TrustedRoot: req.TrustedRoot,
		CacheDir: req.CacheDir, Channel: req.Channel, TargetPath: targetPath}
	metadataURL, err := validateTUFRequest(request)
	if err != nil {
		return empty, nil, err
	}
	if validateBootstrapOrigin(metadataURL.Scheme+"://"+metadataURL.Host) != nil || metadataURL.ForceQuery || metadataURL.RawPath != "" || metadataURL.Opaque != "" || strings.ContainsAny(req.MetadataURL, "\\#") {
		return empty, nil, errors.New("invalid bootstrap metadata HTTPS URL")
	}
	var identity releasepolicy.BrokerBootstrapIdentity
	data, err := verifyTargetWithClient(request, deadline, client, true, func(custom []byte) error {
		var err error
		identity, err = releasepolicy.ParseBrokerBootstrapIdentity(custom)
		if err != nil {
			return fmt.Errorf("signed bootstrap identity: %w", err)
		}
		if identity.Purpose != releasepolicy.BrokerBootstrapPurpose || identity.Channel != req.Channel || identity.TrustConfigSHA256 != req.TrustConfigSHA256 {
			return errors.New("signed bootstrap identity differs from approved purpose, channel or trust configuration")
		}
		return nil
	})
	if err != nil {
		return empty, nil, err
	}
	sum := sha256.Sum256(data)
	if hex.EncodeToString(sum[:]) != req.ApprovedPolicySHA256 {
		return empty, nil, errors.New("bootstrap policy differs from independently approved SHA-256")
	}
	policy, err := releasepolicy.ParseBrokerBootstrap(data)
	if err != nil {
		return empty, nil, fmt.Errorf("bootstrap policy: %w", err)
	}
	if policy.Purpose != identity.Purpose || policy.Channel != identity.Channel || policy.Version != identity.Version || policy.Commit != identity.Commit || policy.TrustConfigSHA256 != identity.TrustConfigSHA256 {
		return empty, nil, errors.New("signed bootstrap identity does not match policy")
	}
	if policy.Provisioner.SHA256 != req.TrustedProvisionerSHA256 {
		return empty, nil, errors.New("bootstrap provisioner differs from independently trusted SHA-256")
	}
	for _, artifact := range []releasepolicy.Artifact{policy.Broker, policy.Provisioner, policy.CorrespondingSource, policy.Notices} {
		if err := validateDownload(artifact, req.ArtifactOrigin); err != nil {
			return empty, nil, errors.New("bootstrap artifact differs from approved HTTPS origin or path")
		}
	}
	return policy, bytes.Clone(data), nil
}

func validBootstrapSHA256(pin string) bool {
	if len(pin) != 64 {
		return false
	}
	value, err := hex.DecodeString(pin)
	return err == nil && hex.EncodeToString(value) == pin
}

func validateBootstrapOrigin(origin string) error {
	u, err := url.Parse(origin)
	if err != nil || u == nil || len(origin) > 2048 || strings.Contains(origin, "#") || u.Scheme != "https" || u.Hostname() == "" || u.User != nil || u.Path != "" || u.RawQuery != "" || u.ForceQuery || u.Fragment != "" || u.Opaque != "" || strings.ContainsAny(u.Host, "\\ \t\n\r") || strings.HasSuffix(u.Host, ":") {
		return errors.New("independent bootstrap artifact HTTPS origin required")
	}
	if u.Port() != "" {
		port, err := strconv.Atoi(u.Port())
		if err != nil || port < 1 || port > 65535 {
			return errors.New("invalid bootstrap artifact origin port")
		}
	}
	return nil
}
