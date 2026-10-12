// Package brokerregistration parses a purpose-bound, independently approved
// operator handoff. It does not provision a machine, authenticate an operator,
// prove installed-byte or loader custody, or authorize executing any executable.
// A hash supplied by an untrusted client or an adjacent declaration is not
// independent approval. Native writers must separately resolve the actual user,
// retain approved installed evidence, and measure the selected stock executable.
package brokerregistration

import (
	"bytes"
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"errors"
	"io"
	"regexp"
	"strconv"
	"strings"
	"time"
	"unicode/utf8"

	"example.com/opennav-update-verifier/releasepolicy"
	"example.com/opennav-update-verifier/repository"
)

const Purpose = "client-registration"
const MaxRequestBytes = 1 << 20
const MaxTrustConfigurationBytes = 576 << 10

var hex40 = regexp.MustCompile(`^[0-9a-f]{40}$`)
var hex64 = regexp.MustCompile(`^[0-9a-f]{64}$`)
var hex32 = regexp.MustCompile(`^[0-9a-f]{32}$`)

// Installed contains operator-approved evidence identities, not proof that these
// bytes are currently installed. Mutable user-owned installation records and
// executable self-reports cannot establish that independent approval or custody.
type Installed struct {
	Version            string `json:"version"`
	Commit             string `json:"commit"`
	Generation         string `json:"generation"`
	PackageSHA256      string `json:"packageSha256"`
	ProductBuildSHA256 string `json:"productBuildSha256"`
	ExecutableSHA256   string `json:"executableSha256"`
	InventorySHA256    string `json:"inventorySha256"`
}

type Stock struct {
	Path             string `json:"path"`
	Version          string `json:"version"`
	Arch             string `json:"arch"`
	ExecutableSHA256 string `json:"executableSha256"`
	UpstreamCommit   string `json:"upstreamCommit"`
}

func (s Stock) Identity() releasepolicy.OpenCpn {
	return releasepolicy.OpenCpn{Version: s.Version, Arch: s.Arch, ExecutableSHA256: s.ExecutableSHA256, UpstreamCommit: s.UpstreamCommit}
}

// StockBaseline returns the independently qualified baseline, never a
// request-selected allowlist. A new baseline requires separate qualification.
func StockBaseline() releasepolicy.OpenCpn {
	return releasepolicy.OpenCpn{Version: "5.12.4", Arch: "x86", ExecutableSHA256: "7c6547562cca7954671eaab72833ca9d788710fd9808b6a699b6dc823852ae0c", UpstreamCommit: "37fd0cddb7334fe489e9f18aa163977a9c5c84f7"}
}

// Request owns its public trust and original request bytes. RequestSHA256 is
// bound to the separate approved input, never inferred as self-authorization.
// Reparse OriginalBytes at each later authority boundary; exported fields are
// caller-owned values and may have been changed after parsing.
type Request struct {
	Schema             int             `json:"schema"`
	Purpose            string          `json:"purpose"`
	ClientSID          string          `json:"clientSid"`
	Installed          Installed       `json:"installed"`
	Stock              Stock           `json:"stock"`
	TrustConfiguration json.RawMessage `json:"trustConfiguration"`
	TrustConfigSHA256  string          `json:"trustConfigSha256"`
	RequestSHA256      string          `json:"-"`
	original           []byte
	trust              repository.Trust
}

func (r Request) OriginalBytes() []byte { return bytes.Clone(r.original) }

// Trust returns an owned copy of the validated ORIGINAL public configuration.
// Root signatures establish internal root consistency, not publisher identity
// or first-execution trust; those remain independent operator prerequisites.
func (r Request) Trust() repository.Trust {
	t := r.trust
	t.BootstrapRoot = bytes.Clone(t.BootstrapRoot)
	t.LocalCompatibilityAllowlist = append([]releasepolicy.OpenCpn(nil), t.LocalCompatibilityAllowlist...)
	return t
}

// ParseApprovedRequest binds exact bounded bytes to separately authenticated
// operator approval and validates the strict declaration and public trust root.
// It performs no network or filesystem action, SID lookup, installed-byte
// observation, signature creation, service registration or executable launch.
func ParseApprovedRequest(raw []byte, independentlyApprovedSHA256 string, now time.Time) (Request, error) {
	if now.IsZero() {
		return Request{}, errors.New("live initial registration clock required")
	}
	return parseApprovedRequest(raw, independentlyApprovedSHA256, func(data []byte) (repository.Trust, error) { return repository.ValidateTrust(data, now) })
}

// ParseRetainedApprovedRequest validates an original approved request read ONLY
// from completed protected registration evidence. All byte/pin/schema/signature
// checks remain required. The original bootstrap anchor may have expired after
// durable root rotation; the actual verifier must check current retained root and
// metadata expiry and rollback floors. This identity-only path cannot authorize
// initial operator enrollment, native creation, state reset or trust migration.
func ParseRetainedApprovedRequest(raw []byte, independentlyApprovedSHA256 string) (Request, error) {
	return parseApprovedRequest(raw, independentlyApprovedSHA256, repository.ValidateTrustIdentity)
}

func parseApprovedRequest(raw []byte, independentlyApprovedSHA256 string, validateTrust func([]byte) (repository.Trust, error)) (Request, error) {
	refused := func() (Request, error) {
		return Request{}, errors.New("independently approved client registration refused")
	}
	if len(raw) == 0 || len(raw) > MaxRequestBytes || !utf8.Valid(raw) || !hex64.MatchString(independentlyApprovedSHA256) {
		return refused()
	}
	data := bytes.Clone(raw)
	if digest(data) != independentlyApprovedSHA256 {
		return refused()
	}
	d := json.NewDecoder(bytes.NewReader(data))
	d.UseNumber()
	if uniqueValue(d, 0) != nil {
		return refused()
	}
	if _, err := d.Token(); err != io.EOF {
		return refused()
	}
	fields, err := object(data, "schema", "purpose", "clientSid", "installed", "stock", "trustConfiguration", "trustConfigSha256")
	if err != nil {
		return refused()
	}
	if _, err = object(fields["installed"], "version", "commit", "generation", "packageSha256", "productBuildSha256", "executableSha256", "inventorySha256"); err != nil {
		return refused()
	}
	if _, err = object(fields["stock"], "path", "version", "arch", "executableSha256", "upstreamCommit"); err != nil {
		return refused()
	}
	var r Request
	d = json.NewDecoder(bytes.NewReader(data))
	d.DisallowUnknownFields()
	if d.Decode(&r) != nil || r.Schema != 1 || r.Purpose != Purpose || !canonicalUserSID(r.ClientSID) || len(r.Installed.Version) > 128 || !hex40.MatchString(r.Installed.Commit) || !hex32.MatchString(r.Installed.Generation) {
		return refused()
	}
	if _, err = releasepolicy.ParseSemVer(r.Installed.Version); err != nil {
		return refused()
	}
	for _, pin := range []string{r.Installed.PackageSHA256, r.Installed.ProductBuildSHA256, r.Installed.ExecutableSHA256, r.Installed.InventorySHA256, r.TrustConfigSHA256} {
		if !hex64.MatchString(pin) {
			return refused()
		}
	}
	if !localStockPath(r.Stock.Path) || r.Stock.Identity() != StockBaseline() {
		return refused()
	}
	r.TrustConfiguration, err = exactTrustConfiguration(data, r.TrustConfiguration, r.TrustConfigSHA256)
	if err != nil {
		return refused()
	}
	r.trust, err = validateTrust(r.TrustConfiguration)
	if err != nil {
		return refused()
	}
	r.RequestSHA256 = independentlyApprovedSHA256
	r.original = data
	return r, nil
}

// RawMessage excludes value-surrounding whitespace. Preserve canonical requests
// first; otherwise recover the single complete field span from the independently
// approved request. The producer writes trust last, immediately after the colon,
// so that span includes the original file's leading/trailing JSON whitespace.
// Never trim, rewrite, or search whitespace variants to manufacture a pin match.
// The caller has already enforced strict unique keys, syntax and request bounds.
func exactTrustConfiguration(data, decoded []byte, pin string) ([]byte, error) {
	if len(decoded) <= MaxTrustConfigurationBytes && digest(decoded) == pin {
		return bytes.Clone(decoded), nil
	}
	d := json.NewDecoder(bytes.NewReader(data))
	if token, err := d.Token(); err != nil || token != json.Delim('{') {
		return nil, errors.New("trust field object required")
	}
	for d.More() {
		key, err := d.Token()
		if err != nil {
			return nil, err
		}
		colon := skipJSONWhitespace(data, int(d.InputOffset()))
		if colon >= len(data) || data[colon] != ':' {
			return nil, errors.New("trust field separator required")
		}
		var value json.RawMessage
		if err := d.Decode(&value); err != nil {
			return nil, err
		}
		if key == "trustConfiguration" {
			end := skipJSONWhitespace(data, int(d.InputOffset()))
			if end >= len(data) || data[end] != ',' && data[end] != '}' {
				return nil, errors.New("trust field terminator required")
			}
			span := data[colon+1 : end]
			if len(span) > MaxTrustConfigurationBytes || digest(span) != pin {
				return nil, errors.New("exact approved trust field bytes required")
			}
			return bytes.Clone(span), nil
		}
	}
	return nil, errors.New("trust field missing")
}

func skipJSONWhitespace(data []byte, offset int) int {
	for offset < len(data) {
		switch data[offset] {
		case ' ', '\t', '\r', '\n':
			offset++
		default:
			return offset
		}
	}
	return offset
}

func digest(data []byte) string { sum := sha256.Sum256(data); return hex.EncodeToString(sum[:]) }

// Only canonical account-domain user SID syntax is accepted. This excludes
// SYSTEM, builtins, service/capability SIDs and well-known identities; it does not
// establish existence or SidTypeUser. The native writer must resolve that fact.
// IsCanonicalUserSID checks bounded account-domain SID syntax only. It does not
// establish existence or SidTypeUser; native creation must resolve the account.
func IsCanonicalUserSID(value string) bool { return canonicalUserSID(value) }

func canonicalUserSID(value string) bool {
	parts := strings.Split(value, "-")
	if len(parts) != 8 || strings.Join(parts[:4], "-") != "S-1-5-21" {
		return false
	}
	for _, part := range parts[4:] {
		n, err := strconv.ParseUint(part, 10, 32)
		if err != nil || strconv.FormatUint(n, 10) != part {
			return false
		}
	}
	return true
}

func localStockPath(value string) bool {
	if len(value) < 4 || len(value) > 240 || !utf8.ValidString(value) || value[1:3] != `:\` || !((value[0] >= 'A' && value[0] <= 'Z') || (value[0] >= 'a' && value[0] <= 'z')) || strings.ContainsAny(value[2:], ":/\x00") {
		return false
	}
	parts := strings.Split(value[3:], `\`)
	for _, p := range parts {
		if p == "" || p == "." || p == ".." || strings.HasSuffix(p, ".") || strings.HasSuffix(p, " ") || strings.ContainsAny(p, `<>"|?*`) {
			return false
		}
		for _, c := range p {
			if c < 32 || c == 127 {
				return false
			}
		}
		base := strings.ToUpper(strings.SplitN(p, ".", 2)[0])
		if base == "CON" || base == "PRN" || base == "AUX" || base == "NUL" ||
			(strings.HasPrefix(base, "COM") || strings.HasPrefix(base, "LPT")) && reservedDeviceDigit(base[3:]) {
			return false
		}
	}
	return strings.EqualFold(parts[len(parts)-1], "opencpn.exe")
}

func reservedDeviceDigit(value string) bool {
	return len(value) == 1 && value[0] >= '0' && value[0] <= '9' || value == "¹" || value == "²" || value == "³"
}

func object(data []byte, keys ...string) (map[string]json.RawMessage, error) {
	var m map[string]json.RawMessage
	if json.Unmarshal(data, &m) != nil || len(m) != len(keys) {
		return nil, errors.New("exact registration object required")
	}
	for _, key := range keys {
		v, ok := m[key]
		if !ok || bytes.Equal(bytes.TrimSpace(v), []byte("null")) {
			return nil, errors.New("exact non-null registration field required")
		}
	}
	return m, nil
}

func uniqueValue(d *json.Decoder, depth int) error {
	if depth > 32 {
		return errors.New("registration JSON nesting limit")
	}
	t, err := d.Token()
	if err != nil {
		return err
	}
	if t == nil {
		return errors.New("null registration value")
	}
	delim, ok := t.(json.Delim)
	if !ok {
		return nil
	}
	switch delim {
	case '{':
		seen := map[string]bool{}
		for d.More() {
			k, err := d.Token()
			if err != nil {
				return err
			}
			name, ok := k.(string)
			folded := strings.ToLower(name)
			if !ok || seen[folded] {
				return errors.New("duplicate registration field")
			}
			seen[folded] = true
			if err = uniqueValue(d, depth+1); err != nil {
				return err
			}
		}
		t, err = d.Token()
		if err != nil || t != json.Delim('}') {
			return errors.New("invalid registration object")
		}
	case '[':
		for d.More() {
			if err = uniqueValue(d, depth+1); err != nil {
				return err
			}
		}
		t, err = d.Token()
		if err != nil || t != json.Delim(']') {
			return errors.New("invalid registration array")
		}
	default:
		return errors.New("invalid registration delimiter")
	}
	return nil
}
