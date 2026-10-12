package releasepolicy

import (
	"bytes"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"net/url"
	"strings"
	"unicode/utf8"
)

// BrokerBootstrapPurpose separates machine broker enrollment from application
// release policies. A declaration is never authentication, elevation or launch
// authority: callers still require operator approval of the exact policy hash,
// an independently pretrusted provisioner and independent TUF verification.
const BrokerBootstrapPurpose = "machine-update-broker-enrollment"

// BrokerBootstrapPolicy declares a fixed machine broker enrollment. CodeLocation
// and DataLocation are fixed KnownFolder policy IDs, never filesystem paths.
// Parsing does not resolve locations, fetch artifacts, follow redirects or run
// any provisioner, service or executable.
type BrokerBootstrapPolicy struct {
	Schema              int      `json:"schema"`
	Product             string   `json:"product"`
	Purpose             string   `json:"purpose"`
	Channel             string   `json:"channel"`
	Version             string   `json:"version"`
	Commit              string   `json:"commit"`
	Architecture        string   `json:"architecture"`
	ServiceName         string   `json:"serviceName"`
	ServiceAccount      string   `json:"serviceAccount"`
	CodeLocation        string   `json:"codeLocation"`
	DataLocation        string   `json:"dataLocation"`
	TrustConfigSHA256   string   `json:"trustConfigSha256"`
	Broker              Artifact `json:"broker"`
	Provisioner         Artifact `json:"provisioner"`
	CorrespondingSource Artifact `json:"correspondingSource"`
	Notices             Artifact `json:"notices"`
}

// BrokerBootstrapIdentity is the exact custom identity bound to a bootstrap
// target by an independently verified TUF envelope. It grants no authority alone.
type BrokerBootstrapIdentity struct {
	Purpose           string `json:"purpose"`
	Channel           string `json:"channel"`
	Version           string `json:"version"`
	Commit            string `json:"commit"`
	TrustConfigSHA256 string `json:"trustConfigSha256"`
}

// BrokerBootstrapTargetPath returns the only bootstrap policy target for a
// supported channel. It does not locate or authenticate a policy.
func BrokerBootstrapTargetPath(channel string) (string, error) {
	if channel != "beta" && channel != "stable" {
		return "", errors.New("unsupported bootstrap channel")
	}
	return "bootstrap/broker-" + channel + ".json", nil
}

// ParseBrokerBootstrap parses a bounded declaration with exact, required keys at
// every level. All artifacts share one exact HTTPS origin and commit directory.
func ParseBrokerBootstrap(data []byte) (BrokerBootstrapPolicy, error) {
	var p BrokerBootstrapPolicy
	object, err := decodeBrokerBootstrap(data, []string{
		"schema", "product", "purpose", "channel", "version", "commit", "architecture",
		"serviceName", "serviceAccount", "codeLocation", "dataLocation", "trustConfigSha256",
		"broker", "provisioner", "correspondingSource", "notices",
	}, &p)
	if err != nil {
		return BrokerBootstrapPolicy{}, err
	}
	for _, name := range []string{"broker", "provisioner", "correspondingSource", "notices"} {
		if _, err := exactObjectKeys(object[name], []string{"path", "url", "sha256", "bytes"}); err != nil {
			return BrokerBootstrapPolicy{}, fmt.Errorf("%s: %w", name, err)
		}
	}
	if err := validateBrokerBootstrap(p); err != nil {
		return BrokerBootstrapPolicy{}, err
	}
	return p, nil
}

// ParseBrokerBootstrapIdentity rejects malformed, null, duplicate, case-folded,
// missing or unknown fields and trailing JSON before validating its identity.
func ParseBrokerBootstrapIdentity(data []byte) (BrokerBootstrapIdentity, error) {
	var identity BrokerBootstrapIdentity
	if _, err := decodeBrokerBootstrap(data, []string{"purpose", "channel", "version", "commit", "trustConfigSha256"}, &identity); err != nil {
		return BrokerBootstrapIdentity{}, err
	}
	if err := validateBrokerBootstrapIdentity(identity); err != nil {
		return BrokerBootstrapIdentity{}, err
	}
	return identity, nil
}

func decodeBrokerBootstrap(data []byte, keys []string, destination any) (map[string]json.RawMessage, error) {
	if len(data) == 0 || len(data) > maxPolicyBytes || !utf8.Valid(data) {
		return nil, errors.New("invalid bootstrap byte length or UTF-8")
	}
	check := json.NewDecoder(bytes.NewReader(data))
	check.UseNumber()
	if err := uniqueValue(check, 0); err != nil {
		return nil, err
	}
	if _, err := check.Token(); err != io.EOF {
		return nil, errors.New("trailing JSON data")
	}
	object, err := exactObjectKeys(data, keys)
	if err != nil {
		return nil, err
	}
	for name, value := range object {
		if bytes.Equal(bytes.TrimSpace(value), []byte("null")) {
			return nil, fmt.Errorf("null bootstrap field %q", name)
		}
	}
	decoder := json.NewDecoder(bytes.NewReader(data))
	decoder.DisallowUnknownFields()
	if err := decoder.Decode(destination); err != nil {
		return nil, err
	}
	return object, nil
}

func validateBrokerBootstrapIdentity(identity BrokerBootstrapIdentity) error {
	if identity.Purpose != BrokerBootstrapPurpose {
		return errors.New("unsupported bootstrap purpose")
	}
	if _, err := BrokerBootstrapTargetPath(identity.Channel); err != nil {
		return err
	}
	version, err := ParseSemVer(identity.Version)
	if err != nil {
		return fmt.Errorf("bootstrap version: %w", err)
	}
	if identity.Channel == "stable" && version.IsPrerelease() {
		return errors.New("stable bootstrap rejects prerelease")
	}
	if !hex40.MatchString(identity.Commit) || !hex64.MatchString(identity.TrustConfigSHA256) {
		return errors.New("invalid bootstrap commit or trust configuration SHA-256")
	}
	return nil
}

func validateBrokerBootstrap(p BrokerBootstrapPolicy) error {
	if p.Schema != 1 || p.Product != "skager" || p.Architecture != "x86" {
		return errors.New("unsupported bootstrap schema, product or architecture")
	}
	if err := validateBrokerBootstrapIdentity(BrokerBootstrapIdentity{
		Purpose: p.Purpose, Channel: p.Channel, Version: p.Version, Commit: p.Commit, TrustConfigSHA256: p.TrustConfigSHA256,
	}); err != nil {
		return err
	}
	if p.ServiceName != "SkagerUpdateBroker" || p.ServiceAccount != `NT SERVICE\SkagerUpdateBroker` ||
		p.CodeLocation != "ProgramFilesX86/SKAGER/UpdateBroker" || p.DataLocation != "ProgramData/SKAGER/UpdateBroker" {
		return errors.New("unsupported bootstrap service identity or KnownFolder policy ID")
	}
	prefix := "artifacts/" + p.Commit + "/"
	origin := ""
	seen := map[string]bool{}
	for _, entry := range []struct {
		name     string
		artifact Artifact
	}{
		{"broker", p.Broker}, {"provisioner", p.Provisioner},
		{"correspondingSource", p.CorrespondingSource}, {"notices", p.Notices},
	} {
		a := entry.artifact
		if err := validateArtifact(a); err != nil {
			return fmt.Errorf("%s: %w", entry.name, err)
		}
		name, scoped := strings.CutPrefix(a.Path, prefix)
		if !scoped || len(name) == 0 || len(name) > 128 || strings.Contains(name, "/") {
			return fmt.Errorf("%s: artifact must have a bounded name in the bootstrap commit directory", entry.name)
		}
		if entry.name == "broker" && name != "skager-update-broker.exe" ||
			entry.name == "provisioner" && name != "skager-update-provisioner.exe" {
			return fmt.Errorf("%s: unexpected bootstrap executable name", entry.name)
		}
		folded := strings.ToLower(name)
		if seen[folded] {
			return errors.New("bootstrap artifact names collide")
		}
		seen[folded] = true
		// validateArtifact has already rejected credentials, queries, encoded
		// paths and fragments. Also reject an empty fragment and port alias.
		u, _ := url.Parse(a.URL)
		if strings.Contains(a.URL, "#") || strings.HasSuffix(u.Host, ":") {
			return errors.New("unsafe bootstrap artifact URL")
		}
		artifactOrigin := u.Scheme + "://" + u.Host
		if origin == "" {
			origin = artifactOrigin
		} else if origin != artifactOrigin {
			return errors.New("bootstrap artifacts must share one exact HTTPS origin")
		}
	}
	return nil
}
