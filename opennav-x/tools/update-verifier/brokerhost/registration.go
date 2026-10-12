// Package brokerhost joins authenticated update acquisition to the Windows
// broker's protected staging store. Staging never installs or executes code.
package brokerhost

import (
	"bytes"
	"encoding/json"
	"errors"
	"io"
	"regexp"
	"strings"
	"time"
	"unicode/utf8"

	verifier "example.com/opennav-update-verifier"
	"example.com/opennav-update-verifier/brokerpipe"
	"example.com/opennav-update-verifier/brokerregistration"
	"example.com/opennav-update-verifier/releasepolicy"
)

const ServiceName = brokerpipe.ServiceName
const stagedState = brokerpipe.Staged

var commitPattern = regexp.MustCompile(`^[0-9a-f]{40}$`)
var hashPattern = regexp.MustCompile(`^[0-9a-f]{64}$`)
var requestPattern = regexp.MustCompile(`^[0-9a-f]{32}$`)

// This is the independently validated stock baseline, not a server-supplied
// allowlist. A future baseline needs separate qualification and a source change.
var stockBaseline = brokerregistration.StockBaseline()

// Only the store's protected, SID-selected registration supplies these values.
// A UI/pipe request cannot choose a path, origin, installed version or trust root.
type registration struct {
	Schema    int    `json:"schema"`
	ClientSID string `json:"clientSid"`
	Installed struct {
		Version string `json:"version"`
		Commit  string `json:"commit"`
	} `json:"installed"`
	Stock struct {
		Path             string `json:"path"`
		Version          string `json:"version"`
		Arch             string `json:"arch"`
		ExecutableSHA256 string `json:"executableSha256"`
		UpstreamCommit   string `json:"upstreamCommit"`
	} `json:"stock"`
	Trust struct {
		BootstrapRoot  json.RawMessage `json:"bootstrapRoot"`
		MetadataURL    string          `json:"metadataUrl"`
		ArtifactOrigin string          `json:"artifactOrigin"`
		Channel        string          `json:"channel"`
	} `json:"updateTrust"`
}

func parseRegistration(data []byte, peerSID string) (registration, error) {
	return parseRegistrationAt(data, peerSID, time.Now().UTC())
}

// protectedRegistration contains the exact original operator-approved request.
// []byte uses bounded JSON base64 encoding to preserve every original request
// and trust byte; embedding RawMessage would compact those bytes on marshaling.
// There is no second mutable installed/trust declaration to reconcile.
type protectedRegistration struct {
	Schema                int    `json:"schema"`
	ClientSID             string `json:"clientSid"`
	ApprovedRequestSHA256 string `json:"approvedRequestSha256"`
	ApprovedRequest       []byte `json:"approvedRequest"`
}

func parseRegistrationAt(data []byte, peerSID string, now time.Time) (registration, error) {
	var r registration
	if len(data) == 0 || len(data) > 1<<20 || !utf8.Valid(data) || peerSID == "" {
		return r, errors.New("invalid protected registration")
	}
	// The protected-store reader separately requires completed native creation
	// evidence. Neither this envelope nor a self-selected hash authenticates the
	// original operator. That independent approval remains a writer precondition.
	d := json.NewDecoder(bytes.NewReader(data))
	d.UseNumber()
	if err := uniqueJSON(d, 0); err != nil {
		return r, errors.New("invalid protected registration JSON")
	}
	if _, err := d.Token(); err != io.EOF {
		return r, errors.New("trailing protected registration JSON")
	}
	var top map[string]json.RawMessage
	if json.Unmarshal(data, &top) != nil || !exactKeys(top, "schema", "clientSid", "approvedRequestSha256", "approvedRequest") {
		return r, errors.New("protected registration fields differ")
	}
	d = json.NewDecoder(bytes.NewReader(data))
	d.DisallowUnknownFields()
	var envelope protectedRegistration
	if d.Decode(&envelope) != nil || envelope.Schema != 2 || envelope.ClientSID != peerSID {
		return registration{}, errors.New("protected registration identity differs")
	}
	encoded, err := json.Marshal(envelope.ApprovedRequest)
	if err != nil || !bytes.Equal(encoded, top["approvedRequest"]) {
		return registration{}, errors.New("exact protected request base64 required")
	}
	request, err := brokerregistration.ParseApprovedRequest(envelope.ApprovedRequest, envelope.ApprovedRequestSHA256, now)
	if err != nil || request.ClientSID != peerSID {
		return registration{}, errors.New("protected approved request identity differs")
	}
	return registrationFromApproved(request), nil
}

// Values come only from the already parsed exact approved request. Evidence
// remains in its protected envelope, not a mutable per-user installation record.
func registrationFromApproved(request brokerregistration.Request) registration {
	r := registration{Schema: 2, ClientSID: request.ClientSID}
	r.Installed.Version = request.Installed.Version
	r.Installed.Commit = request.Installed.Commit
	r.Stock.Path = request.Stock.Path
	r.Stock.Version = request.Stock.Version
	r.Stock.Arch = request.Stock.Arch
	r.Stock.ExecutableSHA256 = request.Stock.ExecutableSHA256
	r.Stock.UpstreamCommit = request.Stock.UpstreamCommit
	trust := request.Trust()
	r.Trust.BootstrapRoot = trust.BootstrapRoot
	r.Trust.MetadataURL = trust.MetadataURL
	r.Trust.ArtifactOrigin = trust.ArtifactOrigin
	r.Trust.Channel = trust.Channel
	return r
}

func (r registration) observedDeclaration() releasepolicy.OpenCpn {
	return releasepolicy.OpenCpn{Version: r.Stock.Version, Arch: r.Stock.Arch,
		ExecutableSHA256: r.Stock.ExecutableSHA256, UpstreamCommit: r.Stock.UpstreamCommit}
}

func (r registration) configuration(state, artifacts string) verifier.PrepareConfig {
	return verifier.PrepareConfig{
		Trust:          verifier.TrustConfig{BootstrapRoot: append([]byte(nil), r.Trust.BootstrapRoot...), MetadataURL: r.Trust.MetadataURL, Channel: r.Trust.Channel},
		StateDirectory: state, ArtifactDirectory: artifacts, ArtifactOrigin: r.Trust.ArtifactOrigin,
		Installed:        releasepolicy.Installed{Version: r.Installed.Version, Commit: r.Installed.Commit},
		OpenCpnAllowlist: []releasepolicy.OpenCpn{stockBaseline},
	}
}

func localStockPath(value string) bool {
	if len(value) < 4 || len(value) > 240 || value[1:3] != `:\` ||
		!((value[0] >= 'A' && value[0] <= 'Z') || (value[0] >= 'a' && value[0] <= 'z')) ||
		strings.ContainsAny(value[2:], ":/\x00") {
		return false
	}
	parts := strings.Split(value[3:], `\`)
	for _, p := range parts {
		if p == "" || p == "." || p == ".." || strings.HasSuffix(p, ".") || strings.HasSuffix(p, " ") || strings.ContainsAny(p, `<>"|?*`) {
			return false
		}
		for _, c := range p {
			if c < 32 {
				return false
			}
		}
		base := strings.ToUpper(strings.SplitN(p, ".", 2)[0])
		if base == "CON" || base == "PRN" || base == "AUX" || base == "NUL" ||
			(len(base) == 4 && (strings.HasPrefix(base, "COM") || strings.HasPrefix(base, "LPT")) && base[3] >= '1' && base[3] <= '9') {
			return false
		}
	}
	return strings.EqualFold(parts[len(parts)-1], "opencpn.exe")
}

func exactKeys(m map[string]json.RawMessage, keys ...string) bool {
	if len(m) != len(keys) {
		return false
	}
	for _, k := range keys {
		if _, ok := m[k]; !ok {
			return false
		}
	}
	return true
}

func uniqueJSON(d *json.Decoder, depth int) error {
	if depth > 32 {
		return errors.New("JSON nesting limit")
	}
	t, err := d.Token()
	if err != nil {
		return err
	}
	delim, ok := t.(json.Delim)
	if !ok {
		return nil
	}
	switch delim {
	case '{':
		seen := map[string]bool{}
		for d.More() {
			key, err := d.Token()
			if err != nil {
				return err
			}
			name, ok := key.(string)
			folded := strings.ToLower(name)
			if !ok || seen[folded] {
				return errors.New("duplicate JSON field")
			}
			seen[folded] = true
			if err := uniqueJSON(d, depth+1); err != nil {
				return err
			}
		}
		t, err = d.Token()
		if err != nil || t != json.Delim('}') {
			return errors.New("invalid JSON object")
		}
	case '[':
		for d.More() {
			if err := uniqueJSON(d, depth+1); err != nil {
				return err
			}
		}
		t, err = d.Token()
		if err != nil || t != json.Delim(']') {
			return errors.New("invalid JSON array")
		}
	default:
		return errors.New("invalid JSON delimiter")
	}
	return nil
}
