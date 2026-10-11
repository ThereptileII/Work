// Package brokerhost joins authenticated update acquisition to the Windows
// broker's protected staging store. Staging never installs or executes code.
package brokerhost

import (
	"bytes"
	"encoding/json"
	"errors"
	"io"
	"net/url"
	"regexp"
	"strings"
	"unicode/utf8"

	verifier "example.com/opennav-update-verifier"
	"example.com/opennav-update-verifier/brokerpipe"
	"example.com/opennav-update-verifier/releasepolicy"
)

const ServiceName = brokerpipe.ServiceName
const stagedState = brokerpipe.Staged

var commitPattern = regexp.MustCompile(`^[0-9a-f]{40}$`)
var hashPattern = regexp.MustCompile(`^[0-9a-f]{64}$`)
var requestPattern = regexp.MustCompile(`^[0-9a-f]{32}$`)

// This is the independently validated stock baseline, not a server-supplied
// allowlist. A future baseline needs separate qualification and a source change.
var stockBaseline = releasepolicy.OpenCpn{
	Version: "5.12.4", Arch: "x86",
	ExecutableSHA256: "7c6547562cca7954671eaab72833ca9d788710fd9808b6a699b6dc823852ae0c",
	UpstreamCommit:   "37fd0cddb7334fe489e9f18aa163977a9c5c84f7",
}

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
	var r registration
	if len(data) == 0 || len(data) > 1<<20 || !utf8.Valid(data) || peerSID == "" {
		return r, errors.New("invalid protected registration")
	}
	// Reject duplicate keys even inside bootstrap metadata. The TUF verifier
	// remains responsible for signatures and trust semantics, independently.
	d := json.NewDecoder(bytes.NewReader(data))
	d.UseNumber()
	if err := uniqueJSON(d, 0); err != nil {
		return r, errors.New("invalid protected registration JSON")
	}
	if _, err := d.Token(); err != io.EOF {
		return r, errors.New("trailing protected registration JSON")
	}
	var top map[string]json.RawMessage
	if json.Unmarshal(data, &top) != nil || !exactKeys(top, "schema", "clientSid", "installed", "stock", "updateTrust") ||
		!exactObject(top["installed"], "version", "commit") ||
		!exactObject(top["stock"], "path", "version", "arch", "executableSha256", "upstreamCommit") ||
		!exactObject(top["updateTrust"], "bootstrapRoot", "metadataUrl", "artifactOrigin", "channel") {
		return r, errors.New("protected registration fields differ")
	}
	d = json.NewDecoder(bytes.NewReader(data))
	d.DisallowUnknownFields()
	if d.Decode(&r) != nil || r.Schema != 1 || r.ClientSID != peerSID ||
		!commitPattern.MatchString(r.Installed.Commit) {
		return registration{}, errors.New("protected registration identity differs")
	}
	if _, err := releasepolicy.ParseSemVer(r.Installed.Version); err != nil || len(r.Installed.Version) > 128 {
		return registration{}, errors.New("invalid protected installed version")
	}
	if r.observedDeclaration() != stockBaseline || !localStockPath(r.Stock.Path) {
		return registration{}, errors.New("unsupported protected OpenCPN registration")
	}
	if (r.Trust.Channel != "beta" && r.Trust.Channel != "stable") ||
		len(r.Trust.BootstrapRoot) == 0 || len(r.Trust.BootstrapRoot) > 512<<10 ||
		bytes.Equal(bytes.TrimSpace(r.Trust.BootstrapRoot), []byte("null")) ||
		!trustedURL(r.Trust.MetadataURL, false) || !trustedURL(r.Trust.ArtifactOrigin, true) {
		return registration{}, errors.New("invalid protected update trust configuration")
	}
	return r, nil
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

func trustedURL(value string, origin bool) bool {
	u, err := url.Parse(value)
	return err == nil && len(value) <= 2048 && u.Scheme == "https" && u.Hostname() != "" &&
		u.User == nil && u.RawQuery == "" && u.Fragment == "" && !u.ForceQuery &&
		!strings.ContainsAny(value, "\r\n\\#") && (!origin || (u.Path == "" && u.RawPath == ""))
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

func exactObject(data []byte, keys ...string) bool {
	var m map[string]json.RawMessage
	return json.Unmarshal(data, &m) == nil && exactKeys(m, keys...)
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
			if !ok || seen[name] {
				return errors.New("duplicate JSON field")
			}
			seen[name] = true
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
