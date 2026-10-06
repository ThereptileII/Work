// Package launcher is the installed Windows startup boundary. It never accepts
// executable paths, update URLs, trust roots or release identity from arguments.
package launcher

import (
	"bytes"
	"encoding/json"
	"errors"
	"io"
	"net/url"
	"path"
	"path/filepath"
	"regexp"
	"strings"
	"unicode/utf8"

	verifier "example.com/opennav-update-verifier"
	"example.com/opennav-update-verifier/releasepolicy"
	"github.com/theupdateframework/go-tuf/v2/metadata/trustedmetadata"
)

const owner = "OpenNavX.Alpha1.SideBySide.1"
const baselineVersion = "5.12.4"
const baselineCommit = "37fd0cddb7334fe489e9f18aa163977a9c5c84f7"
const maxTrustBytes = 576 << 10

var generationPattern = regexp.MustCompile(`^[a-f0-9]{32}$`)
var commitPattern = regexp.MustCompile(`^[a-f0-9]{40}$`)
var hashPattern = regexp.MustCompile(`^[a-f0-9]{64}$`)

type layout struct{ root, generation, directory, app string }

func localPath(p string) (string, error) {
	p = strings.ReplaceAll(p, "\\", "/")
	if len(p) < 3 || len(p) > 4096 || !utf8.ValidString(p) || !((p[0] >= 'A' && p[0] <= 'Z') || (p[0] >= 'a' && p[0] <= 'z')) || p[1:3] != ":/" {
		return "", errors.New("absolute local drive path required")
	}
	if len(p) > 3 {
		for _, segment := range strings.Split(p[3:], "/") {
			if !safeSegment(segment) {
				return "", errors.New("unsafe local path")
			}
		}
	}
	return p, nil
}
func safeSegment(s string) bool {
	if s == "" || s == "." || s == ".." || strings.HasSuffix(s, ".") || strings.HasSuffix(s, " ") || strings.ContainsAny(s, "<>:\"|?*\\/") {
		return false
	}
	for _, c := range s {
		if c < 32 || c == 127 {
			return false
		}
	}
	base := strings.ToUpper(strings.SplitN(s, ".", 2)[0])
	return base != "CON" && base != "PRN" && base != "AUX" && base != "NUL" && !(len(base) == 4 && (strings.HasPrefix(base, "COM") || strings.HasPrefix(base, "LPT")) && base[3] >= '0' && base[3] <= '9')
}
func ownedPath(directory, relative string) (string, error) {
	if len(relative) == 0 || len(relative) > 512 || !utf8.ValidString(relative) {
		return "", errors.New("invalid owned path")
	}
	for _, part := range strings.Split(relative, "/") {
		if !safeSegment(part) {
			return "", errors.New("unsafe owned path")
		}
	}
	return localPath(directory + "/" + relative)
}
func deriveLayout(executable string) (layout, error) {
	p, err := localPath(executable)
	if err != nil {
		return layout{}, err
	}
	app := path.Dir(p)
	directory := path.Dir(app)
	generations := path.Dir(directory)
	if !strings.EqualFold(path.Base(p), "skager-start.exe") || !strings.EqualFold(path.Base(app), "app") || !strings.EqualFold(path.Base(generations), "generations") || !generationPattern.MatchString(path.Base(directory)) || len(path.Dir(generations)) <= 3 {
		return layout{}, errors.New("launcher is outside an installed generation")
	}
	return layout{root: path.Dir(generations), generation: path.Base(directory), directory: directory, app: app}, nil
}

type stockRecord struct {
	Path    string `json:"path"`
	SHA256  string `json:"sha256"`
	Version string `json:"version"`
	Arch    string `json:"arch"`
}
type installState struct {
	Owner    string      `json:"owner"`
	Schema   int         `json:"schema"`
	Current  string      `json:"current"`
	Previous string      `json:"previous"`
	Stock    stockRecord `json:"stock"`
}
type fileRecord struct {
	Path   string `json:"path"`
	SHA256 string `json:"sha256"`
}
type ownership struct {
	Owner          string       `json:"owner"`
	Version        string       `json:"version"`
	Commit         string       `json:"commit"`
	PackageSHA256  string       `json:"packageSha256"`
	HardwarePolicy string       `json:"xnavHardwareOutputPolicy"`
	ManualControl  int          `json:"xnavManualControlContract"`
	Health         int          `json:"updateStartupHealth"`
	Files          []fileRecord `json:"files"`
	Managed        []fileRecord `json:"managedFiles"`
}
type trustRecord struct {
	Schema         int                     `json:"schema"`
	Root           json.RawMessage         `json:"bootstrapRoot"`
	MetadataURL    string                  `json:"metadataUrl"`
	ArtifactOrigin string                  `json:"artifactOrigin"`
	Channel        string                  `json:"channel"`
	Allowlist      []releasepolicy.OpenCpn `json:"localCompatibilityAllowlist"`
}

// Duplicate or case-folded keys and malformed UTF-8 are rejected before typed
// decoding. Existing installer records may carry unrelated additional fields;
// installer-provisioned trust configuration has an exact, closed schema.
func decodeRecord(data []byte, destination any, required []string, strict bool) error {
	if len(data) == 0 || !utf8.Valid(data) {
		return errors.New("invalid record encoding")
	}
	d := json.NewDecoder(bytes.NewReader(data))
	d.UseNumber()
	if err := uniqueJSON(d, 0); err != nil {
		return err
	}
	if _, err := d.Token(); err != io.EOF {
		return errors.New("trailing record data")
	}
	var fields map[string]json.RawMessage
	if err := json.Unmarshal(data, &fields); err != nil || fields == nil {
		return errors.New("record must be an object")
	}
	for _, key := range required {
		if _, ok := fields[key]; !ok {
			return errors.New("missing exact record field")
		}
	}
	if strict && len(fields) != len(required) {
		return errors.New("unknown record field")
	}
	d = json.NewDecoder(bytes.NewReader(data))
	if strict {
		d.DisallowUnknownFields()
	}
	if err := d.Decode(destination); err != nil {
		return errors.New("malformed record")
	}
	return nil
}
func uniqueJSON(d *json.Decoder, depth int) error {
	if depth > 24 {
		return errors.New("record nesting limit")
	}
	token, err := d.Token()
	if err != nil {
		return errors.New("malformed JSON")
	}
	delim, ok := token.(json.Delim)
	if !ok {
		return nil
	}
	if delim == '{' {
		seen := map[string]bool{}
		for d.More() {
			key, err := d.Token()
			if err != nil {
				return errors.New("malformed object")
			}
			text, ok := key.(string)
			if !ok || seen[strings.ToLower(text)] {
				return errors.New("duplicate record key")
			}
			seen[strings.ToLower(text)] = true
			if err := uniqueJSON(d, depth+1); err != nil {
				return err
			}
		}
	} else if delim == '[' {
		for d.More() {
			if err := uniqueJSON(d, depth+1); err != nil {
				return err
			}
		}
	} else {
		return errors.New("invalid JSON delimiter")
	}
	_, err = d.Token()
	return err
}
func parseState(data []byte) (installState, error) {
	var s installState
	if len(data) > 32768 {
		return s, errors.New("state exceeds bound")
	}
	if err := decodeRecord(data, &s, []string{"owner", "schema", "current", "previous", "stock"}, false); err != nil {
		return s, err
	}
	if s.Owner != owner || s.Schema != 1 || !generationPattern.MatchString(s.Current) || (s.Previous != "" && !generationPattern.MatchString(s.Previous)) || s.Current == s.Previous || !hashPattern.MatchString(s.Stock.SHA256) || s.Stock.Version != baselineVersion || s.Stock.Arch != "x86" {
		return s, errors.New("invalid installed state")
	}
	p, err := localPath(s.Stock.Path)
	if err != nil || !strings.EqualFold(path.Base(p), "opencpn.exe") {
		return s, errors.New("invalid installed stock path")
	}
	s.Stock.Path = p
	return s, nil
}
func parseOwnership(data []byte, directory string) (ownership, map[string]string, error) {
	var o ownership
	if len(data) > 4<<20 {
		return o, nil, errors.New("ownership exceeds bound")
	}
	if err := decodeRecord(data, &o, []string{"owner", "version", "commit", "packageSha256", "xnavHardwareOutputPolicy", "files", "managedFiles"}, false); err != nil {
		return o, nil, err
	}
	if _, err := releasepolicy.ParseSemVer(o.Version); err != nil || o.Owner != owner || !commitPattern.MatchString(o.Commit) || !hashPattern.MatchString(o.PackageSHA256) || !supportedHardwarePolicy(o.HardwarePolicy, o.ManualControl) || (o.Health != 0 && o.Health != 1) || len(o.Files) < 1 || len(o.Files) > 12000 || len(o.Managed) < 1 || len(o.Managed) > 12000 {
		return o, nil, errors.New("invalid generation ownership")
	}
	files := map[string]string{}
	managed := map[string]string{}
	for _, f := range o.Files {
		if _, err := ownedPath(directory, f.Path); err != nil || !hashPattern.MatchString(f.SHA256) || files[strings.ToLower(f.Path)] != "" {
			return o, nil, errors.New("invalid generation inventory")
		}
		files[strings.ToLower(f.Path)] = f.SHA256
	}
	for _, f := range o.Managed {
		key := strings.ToLower(f.Path)
		if _, err := ownedPath(directory, f.Path); err != nil || managed[key] != "" || files[key] != f.SHA256 || !hashPattern.MatchString(f.SHA256) {
			return o, nil, errors.New("invalid managed inventory")
		}
		managed[key] = f.SHA256
	}
	return o, managed, nil
}
func parseTrust(data []byte) (trustRecord, error) {
	var t trustRecord
	if len(data) > maxTrustBytes {
		return t, errors.New("trust configuration exceeds bound")
	}
	if err := decodeRecord(data, &t, []string{"schema", "bootstrapRoot", "metadataUrl", "artifactOrigin", "channel", "localCompatibilityAllowlist"}, true); err != nil {
		return t, err
	}
	var fields map[string]json.RawMessage
	_ = json.Unmarshal(data, &fields)
	var identities []json.RawMessage
	if err := json.Unmarshal(fields["localCompatibilityAllowlist"], &identities); err != nil {
		return t, errors.New("invalid compatibility allowlist")
	}
	for _, raw := range identities {
		var identity releasepolicy.OpenCpn
		if err := decodeRecord(raw, &identity, []string{"version", "arch", "executableSha256", "upstreamCommit"}, true); err != nil {
			return t, err
		}
	}
	if t.Schema != 1 || (t.Channel != "beta" && t.Channel != "stable") || len(t.Root) == 0 || len(t.Root) > 512<<10 || len(t.Allowlist) == 0 || len(t.Allowlist) > 16 {
		return t, errors.New("unsupported trust configuration")
	}
	if _, err := trustedmetadata.New(t.Root); err != nil {
		return t, errors.New("invalid installed bootstrap root")
	}
	for _, raw := range []string{t.MetadataURL, t.ArtifactOrigin} {
		u, err := url.Parse(raw)
		if len(raw) > 2048 || err != nil || u.Scheme != "https" || u.Hostname() == "" || u.User != nil || u.RawQuery != "" || u.ForceQuery || u.Fragment != "" || strings.ContainsAny(raw, "#\\\r\n") || (raw == t.ArtifactOrigin && (u.Path != "" || u.Opaque != "")) {
			return t, errors.New("invalid installed update origin")
		}
	}
	seen := map[string]bool{}
	for _, entry := range t.Allowlist {
		if entry.Version != baselineVersion || entry.Arch != "x86" || entry.UpstreamCommit != baselineCommit || !hashPattern.MatchString(entry.ExecutableSHA256) || seen[entry.ExecutableSHA256] {
			return t, errors.New("unsupported local OpenCPN identity")
		}
		seen[entry.ExecutableSHA256] = true
	}
	return t, nil
}
func preparationConfig(l layout, o ownership, t trustRecord) verifier.PrepareConfig {
	return verifier.PrepareConfig{Trust: verifier.TrustConfig{BootstrapRoot: t.Root, MetadataURL: t.MetadataURL, Channel: t.Channel}, StateDirectory: filepath.FromSlash(l.root + "/update-trust/" + t.Channel), ArtifactDirectory: filepath.FromSlash(l.root + "/update-artifacts"), ArtifactOrigin: t.ArtifactOrigin, Installed: releasepolicy.Installed{Version: o.Version, Commit: o.Commit}, OpenCpnAllowlist: t.Allowlist}
}

// Capability admission never enables an adapter or restores a control session.
func supportedHardwarePolicy(policy string, contract int) bool {
	return (policy == "status-only" && contract == 0) || (policy == "manual-commissioning" && contract == 1)
}
