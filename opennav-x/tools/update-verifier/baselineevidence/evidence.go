// Package baselineevidence checks exact recovery sidecar schemas and approved
// byte identities. It authenticates no publisher, operator, filesystem or launch.
package baselineevidence

import (
	"bytes"
	"context"
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"errors"
	"io"
	"regexp"
	"strings"
	"unicode/utf8"

	"example.com/opennav-update-verifier/brokerregistration"
	"example.com/opennav-update-verifier/releasepolicy"
)

const maxRecoveryFiles = 12000
const maxEvidenceBytes = 4 << 20

var recoveryPathChars = regexp.MustCompile(`^[A-Za-z0-9_ .()&@/+-]+$`)
var recoveryHash = regexp.MustCompile(`^[a-f0-9]{64}$`)

// Identity describes observed original recovery bytes. It grants no execution,
// commissioning or installed-running authority. DeclaredPayloadSHA256 is the
// manifest declaration, not an observed payload.zip or outer ZIP hash.
type Identity struct {
	Version                    string
	Commit                     string
	ExecutableSHA256           string
	ProductBuildSHA256         string
	InventorySHA256            string
	LifecycleManifestAvailable bool
	LifecycleManifestSHA256    string
	DeclaredPayloadSHA256      string
}

func digest(data []byte) string { sum := sha256.Sum256(data); return hex.EncodeToString(sum[:]) }

func safeRecoveryPath(name string) bool {
	if len(name) == 0 || len(name) > 240 || !recoveryPathChars.MatchString(name) {
		return false
	}
	for _, part := range strings.Split(name, "/") {
		base := strings.ToUpper(strings.SplitN(part, ".", 2)[0])
		if part == "" || part == "." || part == ".." || strings.HasSuffix(part, ".") || strings.HasSuffix(part, " ") ||
			base == "CON" || base == "PRN" || base == "AUX" || base == "NUL" || len(base) == 4 && (strings.HasPrefix(base, "COM") || strings.HasPrefix(base, "LPT")) && base[3] >= '0' && base[3] <= '9' {
			return false
		}
	}
	return true
}

type recoveryProduct struct {
	Version                      string          `json:"version"`
	Commit                       string          `json:"commit"`
	TestFixtures                 *bool           `json:"test_fixtures"`
	BuildPurpose                 string          `json:"build_purpose"`
	OutputPolicy                 string          `json:"xnav_hardware_output_policy"`
	ManualControlContract        int             `json:"xnav_manual_control_contract"`
	ExecutableSHA256             string          `json:"executable_sha256"`
	RestartHelperSHA256          string          `json:"restart_helper_sha256"`
	CommissioningRestartProtocol int             `json:"commissioning_restart_protocol"`
	UpdateStartupHealth          int             `json:"update_startup_health"`
	StartupLauncherSHA256        string          `json:"startup_launcher_sha256"`
	UpdateTrust                  json.RawMessage `json:"update_trust,omitempty"` // provenance only, never a trust input
}

// Reject duplicate/case-folded keys and trailing data before typed decoding.
// Struct decoding is also checked for exact key spelling, not Go's case aliases.
func strictRecoveryJSON(data []byte, target interface{}) error {
	if !utf8.Valid(data) {
		return errors.New("invalid recovery UTF-8")
	}
	d := json.NewDecoder(bytes.NewReader(data))
	d.UseNumber()
	if err := recoveryJSONValue(d, 0); err != nil {
		return err
	}
	if _, err := d.Token(); err != io.EOF {
		return errors.New("trailing recovery JSON")
	}
	if _, ok := target.(*recoveryProduct); ok {
		var fields map[string]json.RawMessage
		if err := json.Unmarshal(data, &fields); err != nil {
			return err
		}
		keys := []string{"version", "commit", "test_fixtures", "build_purpose", "xnav_hardware_output_policy", "xnav_manual_control_contract", "executable_sha256", "restart_helper_sha256", "commissioning_restart_protocol", "update_startup_health", "startup_launcher_sha256"}
		for _, key := range keys {
			if value, ok := fields[key]; !ok || bytes.Equal(bytes.TrimSpace(value), []byte("null")) {
				return errors.New("missing exact recovery product field")
			}
		}
		if len(fields) != len(keys) {
			if _, ok := fields["update_trust"]; !ok || len(fields) != len(keys)+1 {
				return errors.New("unknown recovery product field")
			}
		}
	}
	d = json.NewDecoder(bytes.NewReader(data))
	d.DisallowUnknownFields()
	return d.Decode(target)
}

type recoveryPackage struct {
	Schema           int                          `json:"schema"`
	Version          string                       `json:"version"`
	Commit           string                       `json:"commit"`
	PayloadSHA256    string                       `json:"payloadSha256"`
	SupportedOpenCpn []map[string]json.RawMessage `json:"supportedOpenCpn"`
	Files            []map[string]json.RawMessage `json:"files"`
	Qualification    string                       `json:"qualification"`
}

func validateRecoveryPackage(data []byte, policy releasepolicy.Policy, inventory map[string]string) (string, error) {
	var p recoveryPackage
	if err := strictRecoveryJSON(data, &p); err != nil {
		return "", errors.New("invalid recovery installer manifest")
	}
	var fields map[string]json.RawMessage
	if err := json.Unmarshal(data, &fields); err != nil || !recoveryExactKeys(fields, []string{"schema", "version", "commit", "payloadSha256", "supportedOpenCpn", "files", "qualification"}, nil) ||
		p.Schema != 1 || p.Version != policy.Version || p.Commit != policy.Commit || !recoveryHash.MatchString(p.PayloadSHA256) ||
		(p.Qualification != "candidate" && p.Qualification != "accepted") || len(p.Files) == 0 || len(p.Files) > maxRecoveryFiles || len(p.SupportedOpenCpn) != len(policy.SupportedOpenCpn) {
		return "", errors.New("recovery installer manifest identity differs")
	}
	seen := map[string]bool{}
	for _, entry := range p.Files {
		if !recoveryExactKeys(entry, []string{"path", "sha256"}, nil) {
			return "", errors.New("invalid installer file record")
		}
		var path, hash string
		if json.Unmarshal(entry["path"], &path) != nil || json.Unmarshal(entry["sha256"], &hash) != nil || !safeRecoveryPath(path) ||
			!(strings.HasPrefix(path, "app/") || strings.HasPrefix(path, "docs/")) || path == "app/OPENNAV_PORTABLE_PREVIEW" ||
			seen[strings.ToLower(path)] || !recoveryHash.MatchString(hash) || inventory[path] != hash {
			return "", errors.New("installer files differ from authenticated recovery")
		}
		seen[strings.ToLower(path)] = true
	}
	for name := range inventory {
		if (strings.HasPrefix(name, "app/") || strings.HasPrefix(name, "docs/")) && name != "app/OPENNAV_PORTABLE_PREVIEW" && !seen[strings.ToLower(name)] {
			return "", errors.New("installer file inventory incomplete")
		}
	}
	identities := map[releasepolicy.OpenCpn]bool{}
	for _, entry := range p.SupportedOpenCpn {
		if !recoveryExactKeys(entry, []string{"version", "arch", "executableSha256", "upstreamCommit"}, []string{"qualification", "integrationPackage", "qualificationCommit", "qualificationRun", "qualificationEvidence", "stockSetupSha256"}) {
			return "", errors.New("invalid installer OpenCPN identity")
		}
		var c releasepolicy.OpenCpn
		for key, value := range entry {
			var s string
			if json.Unmarshal(value, &s) != nil || len(s) > 2048 {
				return "", errors.New("invalid installer compatibility field")
			}
			switch key {
			case "version":
				c.Version = s
			case "arch":
				c.Arch = s
			case "executableSha256":
				c.ExecutableSHA256 = s
			case "upstreamCommit":
				c.UpstreamCommit = s
			}
		}
		if identities[c] {
			return "", errors.New("duplicate installer OpenCPN identity")
		}
		identities[c] = true
	}
	for _, c := range policy.SupportedOpenCpn {
		if !identities[c] {
			return "", errors.New("installer compatibility differs from signed policy")
		}
	}
	return p.PayloadSHA256, nil
}

func recoveryExactKeys(fields map[string]json.RawMessage, required, optional []string) bool {
	allowed := map[string]bool{}
	for _, key := range required {
		value, ok := fields[key]
		if !ok || bytes.Equal(bytes.TrimSpace(value), []byte("null")) {
			return false
		}
		allowed[key] = true
	}
	for _, key := range optional {
		allowed[key] = true
	}
	for key, value := range fields {
		if !allowed[key] || bytes.Equal(bytes.TrimSpace(value), []byte("null")) {
			return false
		}
	}
	return true
}
func recoveryJSONValue(d *json.Decoder, depth int) error {
	if depth > 12 {
		return errors.New("recovery JSON nesting bound")
	}
	tok, err := d.Token()
	if err != nil {
		return err
	}
	delim, ok := tok.(json.Delim)
	if !ok {
		return nil
	}
	switch delim {
	case '{':
		seen := map[string]bool{}
		for d.More() {
			token, err := d.Token()
			if err != nil {
				return err
			}
			key, ok := token.(string)
			if !ok || len(key) == 0 || len(key) > 240 || seen[strings.ToLower(key)] {
				return errors.New("ambiguous recovery JSON key")
			}
			seen[strings.ToLower(key)] = true
			if err := recoveryJSONValue(d, depth+1); err != nil {
				return err
			}
		}
	case '[':
		for d.More() {
			if err := recoveryJSONValue(d, depth+1); err != nil {
				return err
			}
		}
	default:
		return errors.New("invalid recovery JSON delimiter")
	}
	_, err = d.Token()
	return err
}

// DecodeInventory checks the bounded strict original inventory. It does not
// authenticate its contents; callers must bind original bytes independently.
func DecodeInventory(raw []byte, inventory *map[string]string) error {
	if inventory == nil || len(raw) == 0 || len(raw) > maxEvidenceBytes || strictRecoveryJSON(raw, inventory) != nil || len(*inventory) == 0 || len(*inventory) >= maxRecoveryFiles {
		return errors.New("invalid original recovery inventory")
	}
	aliases := map[string]bool{}
	for name, hash := range *inventory {
		key := strings.ToLower(name)
		if !safeRecoveryPath(name) || (strings.EqualFold(name, "app/OPENNAV_PORTABLE_PREVIEW") && name != "app/OPENNAV_PORTABLE_PREVIEW") || strings.EqualFold(name, "FILE_SHA256.json") || !recoveryHash.MatchString(hash) || aliases[key] {
			return errors.New("invalid recovery inventory member")
		}
		aliases[key] = true
	}
	for name := range aliases {
		parts := strings.Split(name, "/")
		for i := 1; i < len(parts); i++ {
			if aliases[strings.Join(parts[:i], "/")] {
				return errors.New("recovery inventory file/directory alias")
			}
		}
	}
	return nil
}

// CheckRecoveryEvidence checks schemas and full observed inventory against exact
// originals under a caller's policy. It supplies NO provenance or authorization:
// the verifier's signed acquisition boundary must authenticate the outer bytes.
// A missing Lifecycle sidecar remains supported only for older recovery layouts.
func CheckRecoveryEvidence(ctx context.Context, inventoryRaw, packageRaw, productRaw []byte, observed map[string]string, policy releasepolicy.Policy) (Identity, error) {
	var empty Identity
	if ctx == nil {
		return empty, errors.New("evidence context required")
	}
	if err := ctx.Err(); err != nil {
		return empty, err
	}
	var inventory map[string]string
	if err := DecodeInventory(inventoryRaw, &inventory); err != nil {
		return empty, err
	}
	if len(inventory) != len(observed) {
		return empty, errors.New("observed inventory differs")
	}
	for name, hash := range inventory {
		if observed[name] != hash {
			return empty, errors.New("observed inventory bytes differ")
		}
	}
	if len(productRaw) == 0 || len(productRaw) > 32<<10 || inventory["docs/PRODUCT_BUILD.json"] != digest(productRaw) {
		return empty, errors.New("product original bytes differ")
	}
	var product recoveryProduct
	if strictRecoveryJSON(productRaw, &product) != nil {
		return empty, errors.New("invalid recovery product record")
	}
	if product.Version != policy.Version || product.Commit != policy.Commit || product.TestFixtures == nil || *product.TestFixtures || product.BuildPurpose != "INSTALLED PRODUCT" || product.UpdateStartupHealth != 1 || product.CommissioningRestartProtocol != 1 ||
		!recoveryHash.MatchString(product.ExecutableSHA256) || !recoveryHash.MatchString(product.RestartHelperSHA256) || !recoveryHash.MatchString(product.StartupLauncherSHA256) ||
		inventory["app/opencpn.exe"] != product.ExecutableSHA256 || inventory["app/opennav-restart.exe"] != product.RestartHelperSHA256 || inventory["app/skager-start.exe"] != product.StartupLauncherSHA256 ||
		!((product.OutputPolicy == "status-only" && product.ManualControlContract == 0) || (product.OutputPolicy == "manual-commissioning" && product.ManualControlContract == 1)) {
		return empty, errors.New("recovery product identity differs or is unsupported")
	}
	identity := Identity{Version: product.Version, Commit: product.Commit, ExecutableSHA256: product.ExecutableSHA256, ProductBuildSHA256: digest(productRaw), InventorySHA256: digest(inventoryRaw)}
	if len(packageRaw) > 0 {
		if len(packageRaw) > maxEvidenceBytes || inventory["installer/package.json"] != digest(packageRaw) {
			return empty, errors.New("manifest original bytes differ")
		}
		payload, err := validateRecoveryPackage(packageRaw, policy, inventory)
		if err != nil {
			return empty, err
		}
		identity.LifecycleManifestAvailable = true
		identity.LifecycleManifestSHA256 = digest(packageRaw)
		identity.DeclaredPayloadSHA256 = payload
	} else if _, ok := inventory["installer/package.json"]; ok {
		return empty, errors.New("manifest original bytes absent")
	}
	if _, ok := inventory["package.json"]; ok {
		return empty, errors.New("unsupported recovery installer manifest layout")
	}
	if err := ctx.Err(); err != nil {
		return empty, err
	}
	return identity, nil
}

// Validate binds exact original sidecars and measured installed payload members
// to independently approved evidence pins. The original inventory may also name
// recovery profile/launcher members, which are not installed payload. Approval,
// custody and current running identity remain separate caller obligations.
func Validate(ctx context.Context, inventoryRaw, packageRaw, productRaw []byte, observed map[string]string, expected brokerregistration.Installed) (Identity, map[string]string, error) {
	var empty Identity
	if ctx == nil {
		return empty, nil, errors.New("evidence context required")
	}
	if err := ctx.Err(); err != nil {
		return empty, nil, err
	}
	if len(inventoryRaw) == 0 || len(inventoryRaw) > maxEvidenceBytes || len(packageRaw) == 0 || len(packageRaw) > maxEvidenceBytes || len(productRaw) == 0 || len(productRaw) > 32<<10 || len(observed) >= maxRecoveryFiles {
		return empty, nil, errors.New("approved evidence size bound")
	}
	if !recoveryHash.MatchString(expected.InventorySHA256) || !recoveryHash.MatchString(expected.PackageSHA256) || !recoveryHash.MatchString(expected.ProductBuildSHA256) || !recoveryHash.MatchString(expected.ExecutableSHA256) ||
		digest(inventoryRaw) != expected.InventorySHA256 || digest(packageRaw) != expected.PackageSHA256 || digest(productRaw) != expected.ProductBuildSHA256 || len(packageRaw) == 0 {
		return empty, nil, errors.New("approved original evidence differs")
	}
	var inventory map[string]string
	if err := DecodeInventory(inventoryRaw, &inventory); err != nil {
		return empty, nil, err
	}
	identity, err := CheckRecoveryEvidence(ctx, inventoryRaw, packageRaw, productRaw, inventory, releasepolicy.Policy{Version: expected.Version, Commit: expected.Commit, SupportedOpenCpn: []releasepolicy.OpenCpn{brokerregistration.StockBaseline()}})
	if err != nil {
		return empty, nil, err
	}
	if identity.ExecutableSHA256 != expected.ExecutableSHA256 {
		return empty, nil, errors.New("approved executable identity differs")
	}
	payload := map[string]string{}
	for name, hash := range inventory {
		if (strings.HasPrefix(name, "app/") || strings.HasPrefix(name, "docs/")) && name != "app/OPENNAV_PORTABLE_PREVIEW" {
			payload[name] = hash
		}
	}
	if len(observed) != len(payload) {
		return empty, nil, errors.New("retained payload namespace differs")
	}
	for name, hash := range payload {
		if observed[name] != hash {
			return empty, nil, errors.New("retained payload bytes differ")
		}
	}
	return identity, payload, nil
}
