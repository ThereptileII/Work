package repository

import (
	"bytes"
	"crypto"
	"crypto/ed25519"
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"errors"
	"example.com/opennav-update-verifier/releasepolicy"
	"fmt"
	"github.com/sigstore/sigstore/pkg/signature"
	"github.com/theupdateframework/go-tuf/v2/metadata"
	"io"
	"math"
	"os"
	"path"
	"path/filepath"
	"runtime"
	"strconv"
	"strings"
	"time"
)

type state struct {
	Schema          int    `json:"schema"`
	ConfigSHA256    string `json:"configSha256"`
	Version         int64  `json:"version"`
	ProductVersion  string `json:"productVersion"`
	ProductCommit   string `json:"productCommit"`
	PolicySHA256    string `json:"policySha256"`
	SelectionSHA256 string `json:"selectionSha256"`
}

func sign[T metadata.Roles](m *metadata.Metadata[T], key ed25519.PrivateKey) ([]byte, error) {
	if len(key) != ed25519.PrivateKeySize || !bytes.Equal(key, ed25519.NewKeyFromSeed(key[:ed25519.SeedSize])) {
		return nil, errors.New("required signing key missing")
	}
	signer, e := signature.LoadSigner(key, crypto.Hash(0))
	if e != nil {
		return nil, errors.New("signer unavailable")
	}
	if _, e = m.Sign(signer); e != nil {
		return nil, errors.New("metadata signing failed")
	}
	return m.ToBytes(false)
}
func syncDir(name string) error {
	f, e := os.Open(name)
	if e != nil {
		return e
	}
	defer f.Close()
	return f.Sync()
}
func mkdir(name string) error {
	if _, e := os.Lstat(name); e == nil {
		return plain(name)
	} else if !os.IsNotExist(e) {
		return e
	}
	parent := filepath.Dir(name)
	if parent == name {
		return errors.New("directory root unavailable")
	}
	if e := mkdir(parent); e != nil {
		return e
	}
	if e := os.Mkdir(name, 0700); e != nil {
		return e
	}
	return syncDir(parent)
}

func atomic(name string, data []byte, replace bool) error {
	parent := filepath.Dir(name)
	if e := plain(parent); e != nil {
		return e
	}
	if st, e := os.Lstat(name); e == nil {
		if !st.Mode().IsRegular() {
			return errors.New("publication destination is not regular")
		}
		if !replace {
			old, e := readPlain(name, int64(len(data)))
			if e != nil || string(old) != string(data) {
				return errors.New("immutable publication collision")
			}
			return nil
		}
	} else if !os.IsNotExist(e) {
		return e
	}
	f, e := os.CreateTemp(parent, ".pending-")
	if e != nil {
		return e
	}
	tmp := f.Name()
	defer os.Remove(tmp)
	if e = f.Chmod(0600); e == nil {
		_, e = f.Write(data)
	}
	if e == nil {
		e = f.Sync()
	}
	ce := f.Close()
	if e != nil {
		return e
	}
	if ce != nil {
		return ce
	}
	if replace {
		e = os.Rename(tmp, name)
	} else {
		e = os.Link(tmp, name)
	}
	if e != nil {
		return e
	}
	return syncDir(parent)
}
func requireOperator() error {
	if runtime.GOOS != "linux" {
		return errors.New("operator mutations require Linux; validation is portable")
	}
	return nil
}

// Initialize uses four externally supplied distinct keys. An interrupted fresh
// initializer cannot be silently reset by invoking initialize again.
func Initialize(directory, metadataURL, artifactOrigin string, keys Keys, now time.Time) error {
	if e := requireOperator(); e != nil {
		return e
	}
	if len(keys) != 4 || !endpoint(metadataURL, false) || !endpoint(artifactOrigin, true) {
		return errors.New("explicit keys and HTTPS endpoints required")
	}
	if e := plain(filepath.Dir(directory)); e != nil {
		return e
	}
	root := metadata.Root(now.Add(365 * 24 * time.Hour))
	root.Signed.ConsistentSnapshot = true
	seen := map[string]bool{}
	for _, role := range roles {
		key := keys[role]
		if len(key) != ed25519.PrivateKeySize || !bytes.Equal(key, ed25519.NewKeyFromSeed(key[:ed25519.SeedSize])) {
			return errors.New("missing role key")
		}
		pub, e := metadata.KeyFromPublicKey(key.Public())
		if e != nil {
			return e
		}
		id := keyID(pub)
		if seen[id] {
			return errors.New("distinct role keys required")
		}
		seen[id] = true
		if e = root.Signed.AddKey(pub, role); e != nil {
			return e
		}
	}
	raw, e := sign(root, keys["root"])
	if e != nil {
		return e
	}
	config := jsonBytes(Trust{1, raw, metadataURL, artifactOrigin, "beta", []releasepolicy.OpenCpn{stock}})
	if _, e = ValidateTrust(config, now); e != nil {
		return e
	}
	if e = os.Mkdir(directory, 0700); e != nil {
		return errors.New("fresh repository directory required")
	}
	if e = mkdir(filepath.Join(directory, "public", "targets", "releases")); e != nil {
		return e
	}
	if e = atomic(filepath.Join(directory, "public", "1.root.json"), raw, false); e != nil {
		return e
	}
	if e = atomic(filepath.Join(directory, "update-trust.json"), config, false); e != nil {
		return e
	}
	if e = atomic(filepath.Join(directory, "state.json"), jsonBytes(state{Schema: 1, ConfigSHA256: checksum(config)}), false); e != nil {
		return e
	}
	return syncDir(filepath.Dir(directory))
}

// Export returns only original public bytes. Root rotation/migration is separate.
func Export(directory, kind string, now time.Time) ([]byte, error) {
	config, e := readPlain(filepath.Join(directory, "update-trust.json"), maxJSON)
	if e != nil {
		return nil, e
	}
	t, e := ValidateTrust(config, now)
	if e != nil {
		return nil, e
	}
	raw, e := readPlain(filepath.Join(directory, "state.json"), maxJSON)
	var retained state
	if e != nil || decode(raw, &retained) != nil || retained.Schema != 1 || retained.ConfigSHA256 != checksum(config) {
		return nil, errors.New("public export does not match retained operator configuration")
	}
	switch kind {
	case "config":
		return config, nil
	case "root":
		return append(append([]byte{}, t.BootstrapRoot...), '\n'), nil
	default:
		return nil, errors.New("unknown public export")
	}
}
func matchKeys(root *metadata.Metadata[metadata.RootType], keys Keys) error {
	if len(keys) != 3 {
		return errors.New("only three online role keys accepted for publishing")
	}
	for _, role := range roles[1:] {
		key := keys[role]
		if len(key) != ed25519.PrivateKeySize || !bytes.Equal(key, ed25519.NewKeyFromSeed(key[:ed25519.SeedSize])) {
			return errors.New("online role key missing")
		}
		pub, e := metadata.KeyFromPublicKey(key.Public())
		if e != nil || keyID(pub) != root.Signed.Roles[role].KeyIDs[0] {
			return errors.New("signing key does not match retained root")
		}
	}
	return nil
}
func load(directory string, now time.Time) (Trust, state, *metadata.Metadata[metadata.RootType], error) {
	var s state
	var t Trust
	fail := func() (Trust, state, *metadata.Metadata[metadata.RootType], error) {
		return t, s, nil, errors.New("retained operator state inconsistent; explicit recovery required")
	}
	if e := plain(directory); e != nil {
		return fail()
	}
	st, e := os.Stat(directory)
	if e != nil || st.Mode().Perm()&0077 != 0 {
		return fail()
	}
	config, e := readPlain(filepath.Join(directory, "update-trust.json"), maxJSON)
	if e != nil {
		return fail()
	}
	t, e = ValidateTrust(config, now)
	if e != nil {
		return fail()
	}
	raw, e := readPlain(filepath.Join(directory, "state.json"), maxJSON)
	if e != nil || decode(raw, &s) != nil || s.Schema != 1 || s.ConfigSHA256 != checksum(config) || s.Version < 0 || s.Version == math.MaxInt64 {
		return fail()
	}
	root, e := rootMetadata(t.BootstrapRoot, now)
	if e != nil {
		return fail()
	}
	canonicalRoot, e := root.ToBytes(false)
	if e != nil {
		return fail()
	}
	diskRoot, e := readPlain(filepath.Join(directory, "public", "1.root.json"), 512<<10)
	if e != nil || checksum(diskRoot) != checksum(canonicalRoot) {
		return fail()
	}
	if s.Version == 0 {
		if s.ProductVersion != "" || s.ProductCommit != "" || s.PolicySHA256 != "" || s.SelectionSHA256 != "" {
			return fail()
		}
	} else {
		if _, e = releasepolicy.ParseSemVer(s.ProductVersion); e != nil || len(s.ProductCommit) != 40 || !hashRE.MatchString(s.PolicySHA256) || !hashRE.MatchString(s.SelectionSHA256) {
			return fail()
		}
	}
	entries, e := os.ReadDir(filepath.Join(directory, "public"))
	if e != nil {
		return fail()
	}
	for _, entry := range entries {
		if entry.Type()&os.ModeSymlink != 0 {
			return fail()
		}
		name := entry.Name()
		for _, suffix := range []string{".targets.json", ".snapshot.json"} {
			if strings.HasSuffix(name, suffix) {
				n, e := strconv.ParseInt(strings.TrimSuffix(name, suffix), 10, 64)
				if e != nil || n < 1 || n > s.Version {
					return fail()
				}
			}
		}
	}
	stampPath := filepath.Join(directory, "public", "timestamp.json")
	if _, e = os.Lstat(stampPath); e == nil {
		raw, e := readPlain(stampPath, 16384)
		if e != nil {
			return fail()
		}
		stamp, e := metadata.Timestamp().FromBytes(raw)
		if e != nil || root.VerifyDelegate("timestamp", stamp) != nil || stamp.Signed.Version < 1 || stamp.Signed.Version > s.Version {
			return fail()
		}
	} else if !os.IsNotExist(e) {
		return fail()
	}
	return t, s, root, nil
}
func copyArtifact(source, dest string, a releasepolicy.Artifact) error {
	if e := plain(source); e != nil {
		return e
	}
	if e := mkdir(filepath.Dir(dest)); e != nil {
		return e
	}
	if _, e := os.Lstat(dest); e == nil {
		n, h, e := hashFile(dest)
		if e != nil || n != a.Bytes || h != a.SHA256 {
			return errors.New("immutable artifact collision")
		}
		return nil
	} else if !os.IsNotExist(e) {
		return e
	}
	in, e := os.Open(source)
	if e != nil {
		return e
	}
	defer in.Close()
	out, e := os.CreateTemp(filepath.Dir(dest), ".pending-")
	if e != nil {
		return e
	}
	tmp := out.Name()
	defer os.Remove(tmp)
	h := sha256.New()
	n, e := io.Copy(io.MultiWriter(out, h), io.LimitReader(in, a.Bytes+1))
	if e == nil {
		e = out.Sync()
	}
	ce := out.Close()
	if e != nil || ce != nil || n != a.Bytes || hex.EncodeToString(h.Sum(nil)) != a.SHA256 {
		return errors.New("artifact changed before publication")
	}
	if e = os.Link(tmp, dest); e != nil {
		return e
	}
	return syncDir(filepath.Dir(dest))
}
func metaFile(version int64, data []byte) *metadata.MetaFiles {
	m := metadata.MetaFile(version)
	m.Length = int64(len(data))
	h := sha256.Sum256(data)
	m.Hashes = map[string]metadata.HexBytes{"sha256": h[:]}
	return m
}

// Publish never fetches or qualifies a release. The operator authenticates the
// reviewed selection hash and its remote provenance before invoking this API.
func Publish(directory, assets string, selection, policy []byte, selectionSHA string, keys Keys, now time.Time) (int64, error) {
	return publish(directory, assets, selection, policy, selectionSHA, keys, now, nil)
}
func publish(directory, assets string, selection, policy []byte, selectionSHA string, keys Keys, now time.Time, checkpoint func(string) error) (int64, error) {
	if e := requireOperator(); e != nil {
		return 0, e
	}
	unlock, e := lock(directory)
	if e != nil {
		return 0, e
	}
	defer unlock()
	trust, s, root, e := load(directory, now)
	if e != nil {
		return 0, e
	}
	if e = matchKeys(root, keys); e != nil {
		return 0, e
	}
	_, p, e := validateSelection(assets, selection, policy, selectionSHA, trust)
	if e != nil {
		return 0, e
	}
	if s.Version > 0 {
		old, _ := releasepolicy.ParseSemVer(s.ProductVersion)
		next, _ := releasepolicy.ParseSemVer(p.Version)
		order := next.Compare(old)
		if order < 0 || (order == 0 && (p.Commit != s.ProductCommit || checksum(policy) != s.PolicySHA256 || selectionSHA != s.SelectionSHA256)) {
			return 0, errors.New("product downgrade or same-version replacement refused")
		}
	}
	// Reserve durable counter/product floor before exposing signed metadata. A
	// failed attempt consumes a version; a retry must advance rather than reset it.
	s.Version++
	s.ProductVersion = p.Version
	s.ProductCommit = p.Commit
	s.PolicySHA256 = checksum(policy)
	s.SelectionSHA256 = selectionSHA
	if e = atomic(filepath.Join(directory, "state.json"), jsonBytes(s), true); e != nil {
		return 0, e
	}
	step := func(name string) error {
		if checkpoint != nil {
			return checkpoint(name)
		}
		return nil
	}
	if e = step("reserved"); e != nil {
		return 0, e
	}
	for _, a := range artifacts(p) {
		if e = copyArtifact(filepath.Join(assets, path.Base(a.Path)), filepath.Join(directory, "public", filepath.FromSlash(a.Path)), a); e != nil {
			return 0, e
		}
	}
	targets := metadata.Targets(now.Add(7 * 24 * time.Hour))
	targets.Signed.Version = s.Version
	target, e := metadata.TargetFile().FromBytes("releases/beta.json", policy, "sha256")
	if e != nil {
		return 0, e
	}
	custom := json.RawMessage(jsonBytes(struct {
		Channel string `json:"channel"`
		Version string `json:"version"`
		Commit  string `json:"commit"`
	}{"beta", p.Version, p.Commit}))
	target.Custom = &custom
	targets.Signed.Targets["releases/beta.json"] = target
	targetBytes, e := sign(targets, keys["targets"])
	if e != nil {
		return 0, e
	}
	snapshot := metadata.Snapshot(now.Add(7 * 24 * time.Hour))
	snapshot.Signed.Version = s.Version
	snapshot.Signed.Meta["targets.json"] = metaFile(s.Version, targetBytes)
	snapshotBytes, e := sign(snapshot, keys["snapshot"])
	if e != nil {
		return 0, e
	}
	timestamp := metadata.Timestamp(now.Add(24 * time.Hour))
	timestamp.Signed.Version = s.Version
	timestamp.Signed.Meta["snapshot.json"] = metaFile(s.Version, snapshotBytes)
	timestampBytes, e := sign(timestamp, keys["timestamp"])
	if e != nil {
		return 0, e
	}
	public := filepath.Join(directory, "public")
	for _, file := range []struct {
		name string
		data []byte
	}{{"targets/releases/" + checksum(policy) + ".beta.json", policy}, {fmt.Sprintf("%d.targets.json", s.Version), targetBytes}, {fmt.Sprintf("%d.snapshot.json", s.Version), snapshotBytes}} {
		if e = atomic(filepath.Join(public, filepath.FromSlash(file.name)), file.data, false); e != nil {
			return 0, e
		}
	}
	if e = step("metadata-staged"); e != nil {
		return 0, e
	}
	// Timestamp is the only publication switch. Consistent snapshots retain all
	// previous metadata and hash-prefixed policy paths for concurrent readers.
	if e = atomic(filepath.Join(public, "timestamp.json"), timestampBytes, true); e != nil {
		return 0, e
	}
	return s.Version, nil
}
