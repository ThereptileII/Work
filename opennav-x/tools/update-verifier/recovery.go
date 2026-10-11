package verifier

import (
	"archive/zip"
	"bytes"
	"context"
	"crypto/sha256"
	"encoding/binary"
	"encoding/hex"
	"encoding/json"
	"errors"
	"io"
	"net/http"
	"regexp"
	"sort"
	"strings"
	"unicode/utf8"

	"example.com/opennav-update-verifier/releasepolicy"
)

const recoveryPrefix = "SKAGER-Beta2-Portable-Recovery/"
const maxRecoveryFiles = 12000
const maxRecoveryExpanded = uint64(2) << 30
const maxRecoveryMember = uint64(512) << 20
const maxRecoveryInventory = int64(4) << 20

// RecoveryIdentity describes actual bytes inside the authenticated recovery ZIP.
// It is informational, not installer, commissioning or launch authority. The
// Older recovery layouts lack Lifecycle's exact package.json. Its hash is
// available only with the inventoried installer/package.json sidecar, and is
// never inferred from the executable, a serialization or the outer ZIP hash.
type RecoveryIdentity struct {
	Version                    string
	Commit                     string
	ExecutableSHA256           string
	ProductBuildSHA256         string
	InventorySHA256            string
	LifecycleManifestAvailable bool
	LifecycleManifestSHA256    string
	// Declared by the authenticated manifest; payload.zip is not in Recovery.
	DeclaredPayloadSHA256 string
}

// PreparedRecovery owns the retained authenticated ZIP handle. Close every
// result, and retain it through any separately authorized use. The inherited
// VerifiedArtifact directory/ancestor protection requirements still apply.
// This does not supply a guardian, broker handoff, extraction or execution API.
type PreparedRecovery struct {
	Policy   releasepolicy.Policy
	Consent  ConsentIdentity
	Identity RecoveryIdentity
	Artifact *VerifiedArtifact
}

func (p *PreparedRecovery) Close() error {
	if p == nil || p.Artifact == nil {
		return nil
	}
	return p.Artifact.Close()
}

// PrepareRecovery repeats the actual protected-state authenticated check and
// exact upgrade consent comparison before acquiring the signed recovery bytes.
// Neither a caller-supplied Policy nor a path/JSON record is accepted as proof.
func PrepareRecovery(ctx context.Context, config PrepareConfig, observed releasepolicy.OpenCpn, consent ConsentIdentity) (*PreparedRecovery, error) {
	return prepareRecovery(ctx, config, observed, consent, nil)
}

// The injected TLS client is private, as in the existing signed fixture tests.
func prepareRecovery(ctx context.Context, config PrepareConfig, observed releasepolicy.OpenCpn, consent ConsentIdentity, client *http.Client) (_ *PreparedRecovery, resultErr error) {
	if !validConsentIdentity(consent) {
		return nil, errors.New("exact confirmed recovery identity required")
	}
	candidate, err := checkUpdate(ctx, config, observed, client)
	if err != nil {
		return nil, err
	}
	if candidate.Decision != releasepolicy.Upgrade || candidate.Consent != consent {
		return nil, errors.New("recovery requires the exact confirmed eligible upgrade")
	}
	if err := validateDownload(candidate.Policy.Recovery, config.ArtifactOrigin); err != nil {
		return nil, errors.New("signed recovery differs from installed acquisition policy")
	}
	return acquireRecovery(ctx, config, candidate, client)
}

// candidate is supplied only by an in-process authenticated check. Sharing this
// acquisition step lets the installed set retain one policy snapshot for both
// artifacts rather than authenticate independently between their downloads.
func acquireRecovery(ctx context.Context, config PrepareConfig, candidate UpdateCandidate, client *http.Client) (_ *PreparedRecovery, resultErr error) {
	var artifact *VerifiedArtifact
	var err error
	if client == nil {
		artifact, err = DownloadArtifact(ctx, candidate.Policy.Recovery, config.ArtifactOrigin, config.ArtifactDirectory)
	} else {
		artifact, err = downloadArtifact(ctx, candidate.Policy.Recovery, config.ArtifactOrigin, config.ArtifactDirectory, client)
	}
	if err != nil {
		return nil, err
	}
	defer func() {
		if resultErr != nil {
			resultErr = errors.Join(resultErr, artifact.Close())
		}
	}()
	identity, err := readRecoveryIdentity(ctx, artifact, candidate.Policy)
	if err != nil {
		return nil, err
	}
	return &PreparedRecovery{Policy: candidate.Policy, Consent: candidate.Consent, Identity: identity, Artifact: artifact}, nil
}

// No exported arbitrary-reader/artifact parser: provenance comes from the
// authenticated check/download above. All archive reads use the retained handle.
func readRecoveryIdentity(ctx context.Context, artifact *VerifiedArtifact, policy releasepolicy.Policy) (RecoveryIdentity, error) {
	contents, err := readRecoveryContents(ctx, artifact, policy)
	return contents.identity, err
}

type recoveryContents struct {
	identity  RecoveryIdentity
	files     map[string]*zip.File
	inventory map[string]string
}

func readRecoveryContents(ctx context.Context, artifact *VerifiedArtifact, policy releasepolicy.Policy) (recoveryContents, error) {
	var empty recoveryContents
	if ctx == nil || artifact == nil || artifact.file == nil {
		return empty, errors.New("retained recovery handle required")
	}
	info, err := artifact.file.Stat()
	if err != nil || !info.Mode().IsRegular() || info.Size() != policy.Recovery.Bytes {
		return empty, errors.New("retained recovery size differs")
	}
	r := recoveryReaderAt{ctx, artifact}
	// Recheck signed outer bytes from this handle, without disturbing its offset.
	if err := copyVerifiedDownload(ctx, io.NewSectionReader(r, 0, info.Size()), io.Discard, policy.Recovery); err != nil {
		return empty, err
	}
	if err := recoveryZIPDirectory(r, info.Size()); err != nil {
		return empty, err
	}
	z, err := zip.NewReader(r, info.Size())
	if err != nil {
		return empty, errors.New("invalid recovery ZIP")
	}
	if len(z.File) == 0 || len(z.File) > maxRecoveryFiles {
		return empty, errors.New("recovery member count bound")
	}
	files := make(map[string]*zip.File)
	aliases := make(map[string]bool)
	var total uint64
	for _, f := range z.File {
		if !strings.HasPrefix(f.Name, recoveryPrefix) || !safeRecoveryPath(strings.TrimPrefix(f.Name, recoveryPrefix)) ||
			!f.Mode().IsRegular() || f.Flags&0x41 != 0 || (f.Method != zip.Store && f.Method != zip.Deflate) ||
			f.UncompressedSize64 > maxRecoveryMember || f.UncompressedSize64 > maxRecoveryExpanded-total {
			return empty, errors.New("unsupported recovery member")
		}
		name := strings.TrimPrefix(f.Name, recoveryPrefix)
		key := strings.ToLower(name)
		if aliases[key] {
			return empty, errors.New("ambiguous recovery member")
		}
		aliases[key] = true
		files[name] = f
		total += f.UncompressedSize64
	}
	for name := range aliases {
		for i := strings.IndexByte(name, '/'); i >= 0; {
			if aliases[name[:i]] {
				return empty, errors.New("recovery file/directory alias")
			}
			next := strings.IndexByte(name[i+1:], '/')
			if next < 0 {
				break
			}
			i += next + 1
		}
	}
	inventoryBytes, err := recoveryMemberBytes(files["FILE_SHA256.json"], maxRecoveryInventory)
	if err != nil {
		return empty, err
	}
	var inventory map[string]string
	if err := strictRecoveryJSON(inventoryBytes, &inventory); err != nil || len(inventory) != len(files)-1 {
		return empty, errors.New("recovery inventory is incomplete or invalid")
	}
	for name, f := range files {
		if name == "FILE_SHA256.json" {
			continue
		}
		digest, ok := inventory[name]
		if !ok || !recoveryHash.MatchString(digest) {
			return empty, errors.New("recovery inventory identity missing")
		}
		actual, err := recoveryMemberHash(f)
		if err != nil || actual != digest {
			return empty, errors.New("recovery inventory bytes differ")
		}
	}
	productBytes, err := recoveryMemberBytes(files["docs/PRODUCT_BUILD.json"], 32<<10)
	if err != nil {
		return empty, err
	}
	var product recoveryProduct
	if err := strictRecoveryJSON(productBytes, &product); err != nil {
		return empty, errors.New("invalid recovery product record")
	}
	if product.Version != policy.Version || product.Commit != policy.Commit || product.TestFixtures == nil || *product.TestFixtures ||
		product.BuildPurpose != "INSTALLED PRODUCT" || product.UpdateStartupHealth != 1 || product.CommissioningRestartProtocol != 1 ||
		!recoveryHash.MatchString(product.ExecutableSHA256) || !recoveryHash.MatchString(product.RestartHelperSHA256) || !recoveryHash.MatchString(product.StartupLauncherSHA256) ||
		inventory["app/opencpn.exe"] != product.ExecutableSHA256 || inventory["app/opennav-restart.exe"] != product.RestartHelperSHA256 || inventory["app/skager-start.exe"] != product.StartupLauncherSHA256 ||
		!((product.OutputPolicy == "status-only" && product.ManualControlContract == 0) || (product.OutputPolicy == "manual-commissioning" && product.ManualControlContract == 1)) {
		return empty, errors.New("recovery product identity differs or is unsupported")
	}
	identity := RecoveryIdentity{Version: product.Version, Commit: product.Commit, ExecutableSHA256: product.ExecutableSHA256,
		ProductBuildSHA256: recoveryDigest(productBytes), InventorySHA256: recoveryDigest(inventoryBytes)}
	if f := files["installer/package.json"]; f != nil {
		manifest, err := recoveryMemberBytes(f, maxRecoveryInventory)
		if err != nil {
			return empty, err
		}
		payload, err := validateRecoveryPackage(manifest, policy, inventory)
		if err != nil {
			return empty, err
		}
		identity.LifecycleManifestAvailable = true
		identity.LifecycleManifestSHA256 = recoveryDigest(manifest)
		identity.DeclaredPayloadSHA256 = payload
	}
	if _, ok := files["package.json"]; ok {
		return empty, errors.New("unsupported recovery installer manifest layout")
	}
	if err := ctx.Err(); err != nil {
		return empty, err
	}
	return recoveryContents{identity: identity, files: files, inventory: inventory}, nil
}

type recoveryReaderAt struct {
	ctx      context.Context
	artifact *VerifiedArtifact
}

func (r recoveryReaderAt) ReadAt(p []byte, off int64) (int, error) {
	if err := r.ctx.Err(); err != nil {
		return 0, err
	}
	return r.artifact.ReadAt(p, off)
}

// Bound allocation before archive/zip reads the central directory. Current
// Python packaging writes single-disk, comment-free ZIP32 with no SFX prefix.
// ZIP64 and other layouts require an explicitly reviewed producer/reader change.
func recoveryZIPDirectory(r io.ReaderAt, size int64) error {
	if size < 22 || size > maxDownloadBytes {
		return errors.New("recovery ZIP size bound")
	}
	var end [22]byte
	var start [4]byte
	if _, err := r.ReadAt(end[:], size-22); err != nil {
		return errors.New("recovery ZIP end unavailable")
	}
	if _, err := r.ReadAt(start[:], 0); err != nil {
		return errors.New("recovery ZIP start unavailable")
	}
	count := binary.LittleEndian.Uint16(end[10:12])
	length := binary.LittleEndian.Uint32(end[12:16])
	offset := binary.LittleEndian.Uint32(end[16:20])
	if binary.LittleEndian.Uint32(start[:]) != 0x04034b50 || binary.LittleEndian.Uint32(end[:4]) != 0x06054b50 ||
		binary.LittleEndian.Uint32(end[4:8]) != 0 || binary.LittleEndian.Uint16(end[8:10]) != count || count == 0 || count > maxRecoveryFiles ||
		length > 16<<20 || uint64(offset)+uint64(length) != uint64(size-22) || binary.LittleEndian.Uint16(end[20:22]) != 0 {
		return errors.New("unsupported recovery ZIP directory")
	}
	// Check central/local name agreement and nonoverlapping local regions too;
	// archive/zip otherwise treats the central name as authoritative. No member
	// may alias another member's compressed bytes or point into the directory.
	central := make([]byte, int(length))
	if _, err := r.ReadAt(central, int64(offset)); err != nil {
		return errors.New("recovery ZIP directory unavailable")
	}
	type region struct{ begin, end uint64 }
	regions := make([]region, 0, int(count))
	position := 0
	for n := 0; n < int(count); n++ {
		if len(central)-position < 46 || binary.LittleEndian.Uint32(central[position:]) != 0x02014b50 {
			return errors.New("invalid recovery central entry")
		}
		h := central[position : position+46]
		nameLength := int(binary.LittleEndian.Uint16(h[28:30]))
		extraLength := int(binary.LittleEndian.Uint16(h[30:32]))
		commentLength := int(binary.LittleEndian.Uint16(h[32:34]))
		entryLength := 46 + nameLength + extraLength + commentLength
		if nameLength == 0 || nameLength > len(recoveryPrefix)+240 || entryLength > len(central)-position || binary.LittleEndian.Uint16(h[34:36]) != 0 {
			return errors.New("unsupported recovery central entry")
		}
		localOffset := uint64(binary.LittleEndian.Uint32(h[42:46]))
		compressed := uint64(binary.LittleEndian.Uint32(h[20:24]))
		var local [30]byte
		if localOffset+30 > uint64(offset) {
			return errors.New("invalid recovery local offset")
		}
		if _, err := r.ReadAt(local[:], int64(localOffset)); err != nil {
			return errors.New("recovery local entry unavailable")
		}
		localNameLength := int(binary.LittleEndian.Uint16(local[26:28]))
		localExtraLength := int(binary.LittleEndian.Uint16(local[28:30]))
		dataOffset := localOffset + 30 + uint64(localNameLength) + uint64(localExtraLength)
		if binary.LittleEndian.Uint32(local[:4]) != 0x04034b50 || localNameLength != nameLength || !bytes.Equal(local[6:10], h[8:12]) || dataOffset+compressed > uint64(offset) {
			return errors.New("recovery local/central layout differs")
		}
		localName := make([]byte, localNameLength)
		if _, err := r.ReadAt(localName, int64(localOffset+30)); err != nil || !bytes.Equal(localName, central[position+46:position+46+nameLength]) {
			return errors.New("ambiguous recovery local name")
		}
		regions = append(regions, region{localOffset, dataOffset + compressed})
		position += entryLength
	}
	if position != len(central) {
		return errors.New("unexpected recovery directory bytes")
	}
	sort.Slice(regions, func(i, j int) bool { return regions[i].begin < regions[j].begin })
	if len(regions) == 0 || regions[0].begin != 0 {
		return errors.New("recovery prefix layout differs")
	}
	for i := 1; i < len(regions); i++ {
		if regions[i].begin < regions[i-1].end {
			return errors.New("overlapping recovery members")
		}
	}
	return nil
}

var recoveryPathChars = regexp.MustCompile(`^[A-Za-z0-9_ .()&@/+-]+$`)
var recoveryHash = regexp.MustCompile(`^[a-f0-9]{64}$`)

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
func recoveryMemberBytes(f *zip.File, limit int64) ([]byte, error) {
	if f == nil || f.UncompressedSize64 == 0 || f.UncompressedSize64 > uint64(limit) {
		return nil, errors.New("recovery record size bound")
	}
	r, err := f.Open()
	if err != nil {
		return nil, errors.New("recovery record unavailable")
	}
	defer r.Close()
	data, err := io.ReadAll(io.LimitReader(r, limit+1))
	if err != nil || uint64(len(data)) != f.UncompressedSize64 {
		return nil, errors.New("recovery record bytes differ")
	}
	return data, nil
}
func recoveryMemberHash(f *zip.File) (string, error) {
	r, err := f.Open()
	if err != nil {
		return "", err
	}
	defer r.Close()
	h := sha256.New()
	n, err := io.CopyBuffer(h, io.LimitReader(r, int64(f.UncompressedSize64)+1), make([]byte, 32<<10))
	if err != nil || uint64(n) != f.UncompressedSize64 {
		return "", errors.New("recovery member length or CRC differs")
	}
	return hex.EncodeToString(h.Sum(nil)), nil
}
func recoveryDigest(data []byte) string {
	sum := sha256.Sum256(data)
	return hex.EncodeToString(sum[:])
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
