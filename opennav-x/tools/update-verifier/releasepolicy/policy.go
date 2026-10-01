// Package releasepolicy parses bounded release declarations and evaluates
// eligibility. It has no network, cryptographic, installation or device API.
package releasepolicy

import (
	"bytes"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"net/url"
	"regexp"
	"strings"
	"unicode/utf8"
)

const maxPolicyBytes = 32 << 10
const maxArtifactBytes = int64(4) << 30

var hex40 = regexp.MustCompile(`^[0-9a-f]{40}$`)
var hex64 = regexp.MustCompile(`^[0-9a-f]{64}$`)
var safeSegment = regexp.MustCompile(`^[A-Za-z0-9][A-Za-z0-9._-]*$`)

type Artifact struct {
	Path   string `json:"path"`
	URL    string `json:"url"`
	SHA256 string `json:"sha256"`
	Bytes  int64  `json:"bytes"`
}
type OpenCpn struct {
	Version          string `json:"version"`
	Arch             string `json:"arch"`
	ExecutableSHA256 string `json:"executableSha256"`
	UpstreamCommit   string `json:"upstreamCommit"`
}
type Policy struct {
	Schema              int       `json:"schema"`
	Product             string    `json:"product"`
	Channel             string    `json:"channel"`
	Version             string    `json:"version"`
	Commit              string    `json:"commit"`
	ReleaseNotes        Artifact  `json:"releaseNotes"`
	Installer           Artifact  `json:"installer"`
	Recovery            Artifact  `json:"recovery"`
	CorrespondingSource Artifact  `json:"correspondingSource"`
	Notices             Artifact  `json:"notices"`
	SupportedOpenCpn    []OpenCpn `json:"supportedOpenCpn"`
}

// Parse rejects malformed UTF-8, duplicate/unknown keys at every level, trailing
// data and declarations larger than the fixed transport-independent policy bound.
func Parse(data []byte) (Policy, error) {
	var p Policy
	if len(data) == 0 || len(data) > maxPolicyBytes || !utf8.Valid(data) {
		return p, errors.New("invalid policy byte length or UTF-8")
	}
	duplicateCheck := json.NewDecoder(bytes.NewReader(data))
	duplicateCheck.UseNumber()
	if err := uniqueValue(duplicateCheck, 0); err != nil {
		return p, err
	}
	if _, err := duplicateCheck.Token(); err != io.EOF {
		return p, errors.New("trailing JSON data")
	}
	if err := exactPolicyKeys(data); err != nil {
		return p, err
	}
	dec := json.NewDecoder(bytes.NewReader(data))
	dec.DisallowUnknownFields()
	if err := dec.Decode(&p); err != nil {
		return Policy{}, err
	}
	if err := validate(p); err != nil {
		return Policy{}, err
	}
	return p, nil
}

func uniqueValue(d *json.Decoder, depth int) error {
	if depth > 16 {
		return errors.New("JSON nesting exceeds policy limit")
	}
	tok, err := d.Token()
	if err != nil {
		return err
	}
	start, ok := tok.(json.Delim)
	if !ok {
		return nil
	}
	switch start {
	case '{':
		seen := map[string]bool{}
		for d.More() {
			keyTok, err := d.Token()
			if err != nil {
				return err
			}
			key, ok := keyTok.(string)
			if !ok {
				return errors.New("invalid JSON object key")
			}
			if seen[key] {
				return fmt.Errorf("duplicate JSON key %q", key)
			}
			seen[key] = true
			if err := uniqueValue(d, depth+1); err != nil {
				return err
			}
		}
		_, err = d.Token()
		return err
	case '[':
		for d.More() {
			if err := uniqueValue(d, depth+1); err != nil {
				return err
			}
		}
		_, err = d.Token()
		return err
	default:
		return errors.New("invalid JSON delimiter")
	}
}

func validate(p Policy) error {
	if p.Schema != 1 || p.Product != "skager" {
		return errors.New("unsupported schema or product")
	}
	if p.Channel != "beta" && p.Channel != "stable" {
		return errors.New("unsupported channel")
	}
	v, err := ParseSemVer(p.Version)
	if err != nil {
		return fmt.Errorf("version: %w", err)
	}
	if p.Channel == "stable" && v.IsPrerelease() {
		return errors.New("stable channel rejects prerelease")
	}
	if !hex40.MatchString(p.Commit) {
		return errors.New("invalid release commit")
	}
	for name, a := range map[string]Artifact{"releaseNotes": p.ReleaseNotes, "installer": p.Installer, "recovery": p.Recovery, "correspondingSource": p.CorrespondingSource, "notices": p.Notices} {
		if err := validateArtifact(a); err != nil {
			return fmt.Errorf("%s: %w", name, err)
		}
	}
	if len(p.SupportedOpenCpn) == 0 || len(p.SupportedOpenCpn) > 16 {
		return errors.New("supportedOpenCpn count outside 1..16")
	}
	seen := map[string]bool{}
	for _, entry := range p.SupportedOpenCpn {
		if err := validateOpenCpn(entry); err != nil {
			return err
		}
		key := entry.Version + "/" + entry.Arch + "/" + entry.ExecutableSHA256 + "/" + entry.UpstreamCommit
		if seen[key] {
			return errors.New("duplicate OpenCPN identity")
		}
		seen[key] = true
	}
	return nil
}
func validateArtifact(a Artifact) error {
	if a.Bytes <= 0 || a.Bytes > maxArtifactBytes || !hex64.MatchString(a.SHA256) {
		return errors.New("invalid size or SHA-256")
	}
	if len(a.Path) == 0 || len(a.Path) > 240 || strings.Contains(a.Path, "\\") || strings.HasPrefix(a.Path, "/") {
		return errors.New("unsafe artifact path")
	}
	for _, seg := range strings.Split(a.Path, "/") {
		if !safeSegment.MatchString(seg) || seg == "." || seg == ".." || strings.HasSuffix(seg, ".") || reservedWindowsSegment(seg) {
			return errors.New("unsafe artifact path segment")
		}
	}
	u, err := url.Parse(a.URL)
	if err != nil || u == nil || u.Scheme != "https" || u.Hostname() == "" || u.User != nil || u.RawQuery != "" || u.ForceQuery || u.Fragment != "" || u.RawPath != "" || u.Path != "/"+a.Path {
		return errors.New("unsafe artifact HTTPS URL")
	}
	if strings.ContainsAny(u.Host, "\\ \t\n\r") {
		return errors.New("unsafe artifact URL host")
	}
	return nil
}
func validateOpenCpn(c OpenCpn) error {
	if c.Version == "" || len(c.Version) > 64 || c.Arch != "x86" || !hex64.MatchString(c.ExecutableSHA256) || !hex40.MatchString(c.UpstreamCommit) {
		return errors.New("invalid OpenCPN identity")
	}
	for _, part := range strings.Split(c.Version, ".") {
		if !numeric(part) || len(part) > 10 || (len(part) > 1 && part[0] == '0') {
			return errors.New("invalid OpenCPN version")
		}
	}
	if len(strings.Split(c.Version, ".")) != 3 {
		return errors.New("invalid OpenCPN version")
	}
	return nil
}

// encoding/json intentionally accepts case-folded aliases for struct fields.
// A signed release schema requires exact field names before typed decoding.
func exactObjectKeys(data []byte, allowed []string) (map[string]json.RawMessage, error) {
	var object map[string]json.RawMessage
	if err := json.Unmarshal(data, &object); err != nil {
		return nil, err
	}
	if len(object) != len(allowed) {
		return nil, errors.New("missing or unknown policy field")
	}
	for _, key := range allowed {
		if _, ok := object[key]; !ok {
			return nil, fmt.Errorf("missing exact policy field %q", key)
		}
	}
	return object, nil
}
func exactPolicyKeys(data []byte) error {
	object, err := exactObjectKeys(data, []string{"schema", "product", "channel", "version", "commit", "releaseNotes", "installer", "recovery", "correspondingSource", "notices", "supportedOpenCpn"})
	if err != nil {
		return err
	}
	for _, name := range []string{"releaseNotes", "installer", "recovery", "correspondingSource", "notices"} {
		if _, err := exactObjectKeys(object[name], []string{"path", "url", "sha256", "bytes"}); err != nil {
			return err
		}
	}
	var identities []json.RawMessage
	if err := json.Unmarshal(object["supportedOpenCpn"], &identities); err != nil {
		return err
	}
	for _, identity := range identities {
		if _, err := exactObjectKeys(identity, []string{"version", "arch", "executableSha256", "upstreamCommit"}); err != nil {
			return err
		}
	}
	return nil
}
func reservedWindowsSegment(segment string) bool {
	base := strings.ToUpper(strings.SplitN(segment, ".", 2)[0])
	switch base {
	case "CON", "PRN", "AUX", "NUL":
		return true
	}
	return len(base) == 4 && (strings.HasPrefix(base, "COM") || strings.HasPrefix(base, "LPT")) && base[3] >= '1' && base[3] <= '9'
}
