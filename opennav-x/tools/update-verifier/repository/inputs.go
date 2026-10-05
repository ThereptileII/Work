// Package repository implements a private beta operator repository, not release
// qualification or public activation. No function generates or persists keys.
package repository

import (
	"archive/zip"
	"bytes"
	"crypto/ed25519"
	"crypto/sha256"
	"encoding/base64"
	"encoding/hex"
	"encoding/json"
	"errors"
	"io"
	"net/url"
	"os"
	"path"
	"path/filepath"
	"reflect"
	"regexp"
	"strings"
	"time"
	"unicode/utf8"

	"example.com/opennav-update-verifier/releasepolicy"
	"github.com/theupdateframework/go-tuf/v2/metadata"
	"github.com/theupdateframework/go-tuf/v2/metadata/trustedmetadata"
)

var roles = []string{"root", "targets", "snapshot", "timestamp"}
var hashRE = regexp.MustCompile(`^[a-f0-9]{64}$`)
var stock = releasepolicy.OpenCpn{Version: "5.12.4", Arch: "x86", ExecutableSHA256: "7c6547562cca7954671eaab72833ca9d788710fd9808b6a699b6dc823852ae0c", UpstreamCommit: "37fd0cddb7334fe489e9f18aa163977a9c5c84f7"}

const maxJSON = 1 << 20

// Decode rejects duplicate, unknown and case-folded field aliases and trailing
// data. Comparing with the typed encoding also rejects missing required fields.
func decode(data []byte, into any) error {
	if len(data) == 0 || len(data) > maxJSON || !utf8.Valid(data) {
		return errors.New("JSON size refused")
	}
	dec := json.NewDecoder(bytes.NewReader(data))
	dec.UseNumber()
	var walk func(int) (any, error)
	walk = func(depth int) (any, error) {
		if depth > 32 {
			return nil, errors.New("JSON depth refused")
		}
		t, e := dec.Token()
		if e != nil {
			return nil, e
		}
		switch t {
		case json.Delim('{'):
			m := map[string]any{}
			for dec.More() {
				k, e := dec.Token()
				if e != nil {
					return nil, e
				}
				s, ok := k.(string)
				if !ok {
					return nil, errors.New("JSON key refused")
				}
				if _, ok = m[s]; ok {
					return nil, errors.New("duplicate JSON key")
				}
				v, e := walk(depth + 1)
				if e != nil {
					return nil, e
				}
				m[s] = v
			}
			_, e = dec.Token()
			return m, e
		case json.Delim('['):
			a := []any{}
			for dec.More() {
				v, e := walk(depth + 1)
				if e != nil {
					return nil, e
				}
				a = append(a, v)
			}
			_, e = dec.Token()
			return a, e
		default:
			return t, nil
		}
	}
	original, e := walk(0)
	if e != nil {
		return errors.New("invalid JSON")
	}
	if _, e = dec.Token(); e != io.EOF {
		return errors.New("trailing JSON")
	}
	d := json.NewDecoder(bytes.NewReader(data))
	d.UseNumber()
	d.DisallowUnknownFields()
	if e = d.Decode(into); e != nil {
		return errors.New("JSON fields or types refused")
	}
	// Enforce exact casing/required fields using the encoding's canonical field names.
	encoded, e := json.Marshal(into)
	if e != nil {
		return e
	}
	var canonical any
	d = json.NewDecoder(bytes.NewReader(encoded))
	d.UseNumber()
	if e = d.Decode(&canonical); e != nil {
		return e
	}
	if !reflect.DeepEqual(original, canonical) {
		return errors.New("noncanonical JSON fields or values")
	}
	return nil
}
func checksum(b []byte) string { h := sha256.Sum256(b); return hex.EncodeToString(h[:]) }
func jsonBytes(v any) []byte   { b, _ := json.MarshalIndent(v, "", "  "); return append(b, '\n') }

// Keys are ephemeral process memory. Feed a private stdin/FD envelope; never
// place it in the repository or pass secret material through argument strings.
type Keys map[string]ed25519.PrivateKey

func ReadKeys(reader io.Reader, initialize bool) (Keys, error) {
	data, e := io.ReadAll(io.LimitReader(reader, 16385))
	if e != nil || len(data) > 16384 {
		return nil, errors.New("key envelope unavailable or oversized")
	}
	defer clear(data)
	var env struct {
		Schema int               `json:"schema"`
		Keys   map[string]string `json:"keys"`
	}
	if e = decode(data, &env); e != nil || env.Schema != 1 {
		return nil, errors.New("key envelope schema refused")
	}
	want := roles[1:]
	if initialize {
		want = roles
	}
	if len(env.Keys) != len(want) {
		return nil, errors.New("explicit distinct role keys required")
	}
	out := Keys{}
	seen := map[string]bool{}
	for _, role := range want {
		seed, e := base64.StdEncoding.Strict().DecodeString(env.Keys[role])
		if e != nil || len(seed) != ed25519.SeedSize {
			return nil, errors.New("Ed25519 role seed refused")
		}
		key := ed25519.NewKeyFromSeed(seed)
		clear(seed)
		id := checksum(key.Public().(ed25519.PublicKey))
		if seen[id] {
			return nil, errors.New("roles must use distinct keys")
		}
		seen[id] = true
		out[role] = key
	}
	return out, nil
}
func (k Keys) Clear() {
	for _, key := range k {
		clear(key)
	}
}

type Trust struct {
	Schema                      int                     `json:"schema"`
	BootstrapRoot               json.RawMessage         `json:"bootstrapRoot"`
	MetadataURL                 string                  `json:"metadataUrl"`
	ArtifactOrigin              string                  `json:"artifactOrigin"`
	Channel                     string                  `json:"channel"`
	LocalCompatibilityAllowlist []releasepolicy.OpenCpn `json:"localCompatibilityAllowlist"`
}

func endpoint(raw string, origin bool) bool {
	u, e := url.Parse(raw)
	return e == nil && len(raw) <= 2048 && u.Scheme == "https" && u.Hostname() != "" && u.User == nil && u.RawQuery == "" && !u.ForceQuery && u.Fragment == "" && u.RawPath == "" && u.Opaque == "" && !strings.ContainsAny(raw, "#\\\r\n\t ") && (!origin || u.Path == "")
}
func rootMetadata(data []byte, now time.Time) (*metadata.Metadata[metadata.RootType], error) {
	if len(data) == 0 || len(data) > 512<<10 {
		return nil, errors.New("root size refused")
	}
	// Reject duplicate/case aliases even though go-tuf supports extensions.
	var raw map[string]any
	if e := decode(data, &raw); e != nil {
		return nil, e
	}
	root, e := metadata.Root().FromBytes(data)
	if e != nil {
		return nil, errors.New("root decoding failed")
	}
	if _, e = trustedmetadata.New(data); e != nil {
		return nil, errors.New("root signature refused")
	}
	if root.Signed.Version != 1 || !root.Signed.ConsistentSnapshot || root.Signed.IsExpired(now) || len(root.Signed.Keys) != 4 || len(root.Signed.Roles) != 4 {
		return nil, errors.New("unsupported or expired operator root")
	}
	if len(root.UnrecognizedFields) != 0 || len(root.Signed.UnrecognizedFields) != 0 {
		return nil, errors.New("root extension refused")
	}
	for _, sig := range root.Signatures {
		if len(sig.UnrecognizedFields) != 0 {
			return nil, errors.New("signature extension refused")
		}
	}
	seen := map[string]bool{}
	for _, role := range roles {
		r, ok := root.Signed.Roles[role]
		if !ok || r.Threshold != 1 || len(r.KeyIDs) != 1 || len(r.UnrecognizedFields) != 0 {
			return nil, errors.New("role threshold refused")
		}
		key, ok := root.Signed.Keys[r.KeyIDs[0]]
		if !ok || len(key.UnrecognizedFields) != 0 || len(key.Value.UnrecognizedFields) != 0 || key.Type != "ed25519" || key.Scheme != "ed25519" || keyID(key) != r.KeyIDs[0] || seen[key.Value.PublicKey] {
			return nil, errors.New("role key identity refused")
		}
		public, e := key.ToPublicKey()
		if e != nil {
			return nil, errors.New("invalid role public key")
		}
		ed, ok := public.(ed25519.PublicKey)
		if !ok || len(ed) != ed25519.PublicKeySize {
			return nil, errors.New("invalid Ed25519 public key")
		}
		seen[key.Value.PublicKey] = true
	}
	return root, nil
}
func ValidateTrust(data []byte, now time.Time) (Trust, error) {
	var t Trust
	if e := decode(data, &t); e != nil {
		return t, e
	}
	if t.Schema != 1 || t.Channel != "beta" || !endpoint(t.MetadataURL, false) || !endpoint(t.ArtifactOrigin, true) || len(t.LocalCompatibilityAllowlist) != 1 || t.LocalCompatibilityAllowlist[0] != stock {
		return t, errors.New("private beta trust configuration refused")
	}
	_, e := rootMetadata(t.BootstrapRoot, now)
	return t, e
}

// Selection is a separately reviewed root-operator handoff. Its SHA256 is an
// explicit CLI argument, not inferred from this self-described document.
type Selection struct {
	Schema                int            `json:"schema"`
	Version               string         `json:"version"`
	Commit                string         `json:"commit"`
	ReleaseManifestSHA256 string         `json:"releaseManifestSha256"`
	QualificationSHA256   string         `json:"qualificationSha256"`
	PolicySHA256          string         `json:"policySha256"`
	Files                 []SelectedFile `json:"files"`
}
type SelectedFile struct {
	Path   string `json:"path"`
	SHA256 string `json:"sha256"`
	Bytes  int64  `json:"bytes"`
}

func artifacts(p releasepolicy.Policy) []releasepolicy.Artifact {
	return []releasepolicy.Artifact{p.ReleaseNotes, p.Installer, p.Recovery, p.CorrespondingSource, p.Notices}
}

func validateSelection(directory string, selectionRaw, policyRaw []byte, expected string, trust Trust) (Selection, releasepolicy.Policy, error) {
	var s Selection
	var p releasepolicy.Policy
	fail := func() (Selection, releasepolicy.Policy, error) {
		return s, p, errors.New("reviewed asset selection refused")
	}
	if !hashRE.MatchString(expected) || checksum(selectionRaw) != expected || decode(selectionRaw, &s) != nil {
		return fail()
	}
	var e error
	p, e = releasepolicy.Parse(policyRaw)
	if e != nil {
		return fail()
	}
	if s.Schema != 1 || p.Channel != "beta" || p.Version != s.Version || p.Commit != s.Commit || s.PolicySHA256 != checksum(policyRaw) || len(s.Files) != 5 || len(p.SupportedOpenCpn) != 1 || p.SupportedOpenCpn[0] != stock {
		return fail()
	}
	manifestRaw, e := readPlain(filepath.Join(directory, "RELEASE.json"), maxJSON)
	if e != nil || !hashRE.MatchString(s.ReleaseManifestSHA256) || checksum(manifestRaw) != s.ReleaseManifestSHA256 {
		return fail()
	}
	qRaw, e := readPlain(filepath.Join(directory, "QUALIFICATION.json"), maxJSON)
	if e != nil || !hashRE.MatchString(s.QualificationSHA256) || checksum(qRaw) != s.QualificationSHA256 {
		return fail()
	}
	var m, q map[string]any
	if decode(manifestRaw, &m) != nil || decode(qRaw, &q) != nil {
		return fail()
	}
	if m["schemaVersion"] != json.Number("1") || (q["schemaVersion"] != json.Number("1") && q["schemaVersion"] != json.Number("2")) || m["channel"] != "staging" || m["version"] != p.Version || m["commit"] != p.Commit || q["channel"] != "staging" || q["commit"] != p.Commit || q["publicAccess"] != false {
		return fail()
	}
	gates, ok := q["gates"].(map[string]any)
	if !ok {
		return fail()
	}
	for _, gate := range []string{"linux", "windows", "installer", "package", "restart"} {
		if gates[gate] != "passed" {
			return fail()
		}
	}
	records, ok := m["files"].([]any)
	if !ok {
		return fail()
	}
	pins := map[string]SelectedFile{}
	for _, record := range records {
		r, ok := record.(map[string]any)
		if !ok {
			return fail()
		}
		name, ok := r["name"].(string)
		if !ok || path.Base(name) != name || pins[name].Path != "" {
			return fail()
		}
		hash, ok := r["sha256"].(string)
		if !ok {
			return fail()
		}
		size, ok := r["size"].(json.Number)
		if !ok {
			return fail()
		}
		n, e := size.Int64()
		if e != nil {
			return fail()
		}
		pins[name] = SelectedFile{name, hash, n}
	}
	seen := map[string]bool{}
	names := []string{"SKAGER-Beta2-Release-Notes.md", "SKAGER-Beta2-Setup.exe", "SKAGER-Beta2-Portable-Recovery.zip", "SKAGER-Beta2-source.zip", "SOURCE_AND_LICENSES.md"}
	for i, a := range artifacts(p) {
		f := s.Files[i]
		if path.Base(a.Path) != names[i] || f != (SelectedFile{a.Path, a.SHA256, a.Bytes}) || seen[a.Path] || !strings.HasPrefix(a.Path, "artifacts/"+p.Commit+"/") || strings.Count(a.Path, "/") != 2 || a.URL != trust.ArtifactOrigin+"/"+a.Path {
			return fail()
		}
		seen[a.Path] = true
		local := filepath.Join(directory, path.Base(a.Path))
		size, hash, e := hashFile(local)
		if e != nil || size != a.Bytes || hash != a.SHA256 {
			return fail()
		}
		if i < 4 {
			r, ok := pins[path.Base(a.Path)]
			if !ok || r.SHA256 != a.SHA256 || r.Bytes != a.Bytes {
				return fail()
			}
		}
	}
	// Notices must be the exact corresponding recovery-package notice, never
	// an unqualified extra file supplied beside an otherwise qualified release.
	recovery := filepath.Join(directory, path.Base(p.Recovery.Path))
	z, e := zip.OpenReader(recovery)
	if e != nil {
		return fail()
	}
	defer z.Close()
	count := 0
	for _, f := range z.File {
		if f.Name == "SKAGER-Beta2-Portable-Recovery/docs/SOURCE_AND_LICENSES.md" {
			count++
			if !f.Mode().IsRegular() || f.UncompressedSize64 > maxJSON {
				return fail()
			}
			r, e := f.Open()
			if e != nil {
				return fail()
			}
			b, e := io.ReadAll(io.LimitReader(r, maxJSON+1))
			r.Close()
			if e != nil || int64(len(b)) != p.Notices.Bytes || checksum(b) != p.Notices.SHA256 {
				return fail()
			}
		}
	}
	if count != 1 {
		return fail()
	}
	return s, p, nil
}

// Plain reads reject symlink components, devices, nonregular and oversized data.
func plain(name string) error {
	full, e := filepath.Abs(name)
	if e != nil {
		return e
	}
	for p := full; ; p = filepath.Dir(p) {
		st, e := os.Lstat(p)
		if e != nil {
			return e
		}
		if st.Mode()&os.ModeSymlink != 0 {
			return errors.New("symlink path refused")
		}
		if p == filepath.Dir(p) {
			break
		}
	}
	return nil
}
func readBounded(reader io.Reader, max int64) ([]byte, error) {
	if max < 1 || max > 4<<30 {
		return nil, errors.New("invalid read bound")
	}
	data, e := io.ReadAll(io.LimitReader(reader, max+1))
	if e != nil || int64(len(data)) > max {
		return nil, errors.New("file grew beyond read bound")
	}
	return data, nil
}
func readPlain(name string, max int64) ([]byte, error) {
	if e := plain(name); e != nil {
		return nil, e
	}
	f, e := os.Open(name)
	if e != nil {
		return nil, e
	}
	defer f.Close()
	st, e := f.Stat()
	if e != nil || !st.Mode().IsRegular() || st.Size() < 1 || st.Size() > max {
		return nil, errors.New("file type or size refused")
	}
	data, e := readBounded(f, max)
	if e != nil {
		return nil, e
	}
	after, e := f.Stat()
	if e != nil || int64(len(data)) != st.Size() || after.Size() != st.Size() {
		return nil, errors.New("file changed during bounded read")
	}
	return data, nil
}

func hashFile(name string) (int64, string, error) {
	if e := plain(name); e != nil {
		return 0, "", e
	}
	f, e := os.Open(name)
	if e != nil {
		return 0, "", e
	}
	defer f.Close()
	st, e := f.Stat()
	if e != nil || !st.Mode().IsRegular() || st.Size() <= 0 || st.Size() > 4<<30 {
		return 0, "", errors.New("artifact type or size refused")
	}
	h := sha256.New()
	n, e := io.Copy(h, io.LimitReader(f, st.Size()+1))
	if e != nil || n != st.Size() {
		return 0, "", errors.New("artifact changed during read")
	}
	return n, hex.EncodeToString(h.Sum(nil)), nil
}

func keyID(k *metadata.Key) string { id, _ := k.ID(); return id }
