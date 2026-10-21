package verifier

import (
	"archive/zip"
	"bytes"
	"context"
	"crypto/sha256"
	"encoding/binary"
	"encoding/hex"
	"errors"
	"io"
	"net/http"
	"regexp"
	"sort"
	"strings"

	"example.com/opennav-update-verifier/baselineevidence"
	"example.com/opennav-update-verifier/releasepolicy"
)

const recoveryPrefix = "SKAGER-Beta2-Portable-Recovery/"
const maxRecoveryFiles = 12000
const maxRecoveryExpanded = uint64(2) << 30
const maxRecoveryMember = uint64(512) << 20
const maxRecoveryInventory = int64(4) << 20

// RecoveryIdentity describes actual bytes inside the authenticated recovery ZIP.
// It is informational, not installer, commissioning or launch authority.
// Older recovery layouts lack Lifecycle's exact package.json. Its hash is
// available only with the inventoried installer/package.json sidecar, and is
// never inferred from the executable, a serialization or the outer ZIP hash.
type RecoveryIdentity = baselineevidence.Identity

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
	artifact, err = acquireArtifact(ctx, candidate.Policy.Recovery, config, client)
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

// This signed recovery path gets provenance from authenticated check/download
// above and reads only its retained handle. Approved baseline import has its
// separate purpose-bound operator approval boundary.
func readRecoveryIdentity(ctx context.Context, artifact *VerifiedArtifact, policy releasepolicy.Policy) (RecoveryIdentity, error) {
	contents, err := readRecoveryContents(ctx, artifact, policy)
	return contents.identity, err
}

type recoveryContents struct {
	identity                                   RecoveryIdentity
	files                                      map[string]*zip.File
	inventory                                  map[string]string
	inventoryBytes, packageBytes, productBytes []byte
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
	return parseRecoveryContents(ctx, r, info.Size(), policy)
}

// Private parser shared by authenticated acquisition and purpose-bound approved
// import. Callers must establish their distinct authority before reaching it.
func parseRecoveryContents(ctx context.Context, r io.ReaderAt, size int64, policy releasepolicy.Policy) (recoveryContents, error) {
	var empty recoveryContents
	if err := recoveryZIPDirectory(r, size); err != nil {
		return empty, err
	}
	z, err := zip.NewReader(r, size)
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
		if (baselineevidence.IsEngineHelper(name) || baselineevidence.IsCommissioningHelper(name)) && (f.UncompressedSize64 == 0 || f.UncompressedSize64 > uint64(maxRecoveryInventory)) {
			return empty, errors.New("original execution helper size bound")
		}
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
	if err := baselineevidence.DecodeInventory(inventoryBytes, &inventory); err != nil || len(inventory) != len(files)-1 {
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
	var manifest []byte
	if f := files["installer/package.json"]; f != nil {
		manifest, err = recoveryMemberBytes(f, maxRecoveryInventory)
		if err != nil {
			return empty, err
		}
	}
	identity, err := baselineevidence.CheckRecoveryEvidence(ctx, inventoryBytes, manifest, productBytes, inventory, policy)
	if err != nil {
		return empty, err
	}
	if _, ok := files["package.json"]; ok {
		return empty, errors.New("unsupported recovery installer manifest layout")
	}
	if err := ctx.Err(); err != nil {
		return empty, err
	}
	return recoveryContents{identity: identity, files: files, inventory: inventory, inventoryBytes: inventoryBytes, packageBytes: manifest, productBytes: productBytes}, nil
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
