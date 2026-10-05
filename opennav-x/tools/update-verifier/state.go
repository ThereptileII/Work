package verifier

import (
	"bytes"
	"crypto/rand"
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"errors"
	"io"
	"net/http"
	"net/url"
	"os"
	"path/filepath"
	"regexp"
	"strings"
	"time"

	"example.com/opennav-update-verifier/releasepolicy"
	"github.com/theupdateframework/go-tuf/v2/metadata/trustedmetadata"
)

// TrustConfig is provisioned with the installed helper, never by a network
// response. Each channel has a separate store. No secret belongs in this record.
type TrustConfig struct {
	BootstrapRoot []byte
	MetadataURL   string
	Channel       string
}

type stateIdentity struct {
	Schema          int               `json:"schema"`
	BootstrapSHA256 string            `json:"bootstrapSha256"`
	MetadataURL     string            `json:"metadataUrl"`
	Channel         string            `json:"channel"`
	Generation      string            `json:"generation"`
	Metadata        map[string]string `json:"metadata"`
}

var stateGeneration = regexp.MustCompile(`^[a-f0-9]{32}$`)
var stateMetadataName = regexp.MustCompile(`^[A-Za-z0-9_%.-]+\.json$`)

const maxStateBytes = 32 << 10
const maxCacheFileBytes = 5 << 20
const maxCacheTotalBytes = 32 << 20

func trustIdentity(c TrustConfig) (stateIdentity, error) {
	u, err := url.Parse(c.MetadataURL)
	if err != nil || u.Scheme != "https" || u.Host == "" || u.User != nil || u.RawQuery != "" || u.Fragment != "" || strings.ContainsAny(c.MetadataURL, "\r\n\\") {
		return stateIdentity{}, errors.New("invalid installed update origin")
	}
	if c.Channel != "beta" && c.Channel != "stable" {
		return stateIdentity{}, errors.New("invalid installed update channel")
	}
	if len(c.BootstrapRoot) == 0 || len(c.BootstrapRoot) > 512<<10 {
		return stateIdentity{}, errors.New("invalid installed bootstrap root size")
	}
	if _, err := trustedmetadata.New(c.BootstrapRoot); err != nil {
		return stateIdentity{}, errors.New("invalid installed bootstrap root")
	}
	return stateIdentity{Schema: 1, BootstrapSHA256: stateDigest(c.BootstrapRoot), MetadataURL: c.MetadataURL, Channel: c.Channel}, nil
}

// InitializeState is an explicit fresh-install operation. An existing path is
// always refused, even if empty. Update/check never calls this function to repair
// lost state. The installer must independently remember initialization outside
// this directory; deleting all state is not authorization to reset trust.
func InitializeState(directory string, c TrustConfig) error {
	s, err := trustIdentity(c)
	if err != nil {
		return err
	}
	if err := plainStatePath(filepath.Dir(directory)); err != nil {
		return err
	}
	if !filepath.IsAbs(directory) || filepath.Clean(directory) != directory {
		return errors.New("absolute clean state path required")
	}
	if err := makePrivateStateDirectory(directory); err != nil {
		return errors.New("new private update directory required")
	}
	// On interruption the incomplete directory remains and cannot silently be
	// initialized again. Explicit installer recovery is required.
	unlock, err := lockState(directory)
	if err != nil {
		return err
	}
	defer unlock()
	id, err := newStateGeneration()
	if err != nil {
		return err
	}
	s.Generation = id
	cache := filepath.Join(directory, id)
	if err := os.Mkdir(cache, 0700); err != nil {
		return err
	}
	if err := os.Mkdir(filepath.Join(cache, "metadata"), 0700); err != nil {
		return err
	}
	if err := os.WriteFile(filepath.Join(cache, "metadata", "root.json"), c.BootstrapRoot, 0600); err != nil {
		return err
	}
	s.Metadata = map[string]string{"root.json": stateDigest(c.BootstrapRoot)}
	if err := syncStateFiles(cache); err != nil {
		return err
	}
	return writeState(directory, s)
}

// VerifyReleaseInState serializes refreshes, validates every retained cache
// byte before use, and atomically retains the verified metadata observation.
// Missing/corrupt/replaced state is an error, never a fresh installation.
// No package is executed. The caller must still evaluate compatibility and
// installed-version policy and retain verified artifact custody.
func VerifyReleaseInState(directory string, c TrustConfig) (releasepolicy.Policy, error) {
	return verifyReleaseInState(directory, c, nil)
}

// ValidateState is a read-only installer check. It never contacts a server,
// recreates missing state, adopts a cache generation, or resets a trust floor.
func ValidateState(directory string, c TrustConfig) error {
	want, err := trustIdentity(c)
	if err != nil {
		return err
	}
	if err := plainStatePath(directory); err != nil {
		return err
	}
	if err := assertPrivateStateDirectory(directory); err != nil {
		return err
	}
	unlock, err := lockState(directory)
	if err != nil {
		return err
	}
	defer unlock()
	if _, err := os.Lstat(filepath.Join(directory, "refresh.pending")); !os.IsNotExist(err) {
		return errors.New("interrupted update trust refresh; explicit recovery required")
	}
	s, err := readState(directory, want)
	if err != nil {
		return err
	}
	files, err := readStateMetadata(filepath.Join(directory, s.Generation))
	if err != nil {
		return err
	}
	if len(files) != len(s.Metadata) {
		return errors.New("retained update metadata inventory changed")
	}
	for name, data := range files {
		if s.Metadata[name] != stateDigest(data) {
			return errors.New("retained update metadata changed")
		}
	}
	if _, ok := files["root.json"]; !ok {
		return errors.New("retained update root missing")
	}
	return nil
}

func verifyReleaseInState(directory string, c TrustConfig, client *http.Client) (releasepolicy.Policy, error) {
	var empty releasepolicy.Policy
	want, err := trustIdentity(c)
	if err != nil {
		return empty, err
	}
	if err := plainStatePath(directory); err != nil {
		return empty, err
	}
	if err := assertPrivateStateDirectory(directory); err != nil {
		return empty, err
	}
	unlock, err := lockState(directory)
	if err != nil {
		return empty, err
	}
	defer unlock()
	if _, err := os.Lstat(filepath.Join(directory, "refresh.pending")); !os.IsNotExist(err) {
		return empty, errors.New("interrupted update trust refresh; explicit recovery required")
	}
	current, err := readState(directory, want)
	if err != nil {
		return empty, err
	}
	old := filepath.Join(directory, current.Generation)
	files, err := readStateMetadata(old)
	if err != nil {
		return empty, err
	}
	if len(files) != len(current.Metadata) {
		return empty, errors.New("retained update metadata inventory changed")
	}
	for name, data := range files {
		if current.Metadata[name] != stateDigest(data) {
			return empty, errors.New("retained update metadata changed")
		}
	}
	root, ok := files["root.json"]
	if !ok {
		return empty, errors.New("retained update root missing")
	}
	id, err := newStateGeneration()
	if err != nil {
		return empty, err
	}
	stage := filepath.Join(directory, id)
	if err := os.Mkdir(stage, 0700); err != nil {
		return empty, err
	}
	committed := false
	refreshStarted := false
	defer func() {
		if !committed && !refreshStarted {
			_ = os.RemoveAll(stage)
		}
	}()
	if err := os.Mkdir(filepath.Join(stage, "metadata"), 0700); err != nil {
		return empty, err
	}
	for name, data := range files {
		if err := os.WriteFile(filepath.Join(stage, "metadata", name), data, 0600); err != nil {
			return empty, err
		}
	}
	// Once newer signed versions might be observed, a crash must not silently
	// fall back to an older rollback floor. Leave a durable interrupted marker
	// until the new complete cache pointer has been committed.
	marker := filepath.Join(directory, "refresh.pending")
	m, err := os.OpenFile(marker, os.O_WRONLY|os.O_CREATE|os.O_EXCL, 0600)
	if err != nil {
		return empty, err
	}
	refreshStarted = true
	if _, err := m.WriteString(id); err != nil {
		m.Close()
		return empty, err
	}
	if err := m.Sync(); err != nil {
		m.Close()
		return empty, err
	}
	if err := m.Close(); err != nil {
		return empty, err
	}
	if err := syncStateDirectory(directory); err != nil {
		return empty, err
	}
	req := Request{MetadataURL: c.MetadataURL, TrustedRoot: root, CacheDir: stage, Channel: c.Channel, TargetPath: "releases/" + c.Channel + ".json"}
	policy, verifyErr := verifyReleaseWithClient(req, time.Now().Add(maxOperationTime), client)
	// go-tuf persists each role only after signature/chain verification. Keep
	// newer root/timestamp versions even when later snapshot/target/download fails;
	// otherwise a failed request could undo an observed rollback floor.
	nextFiles, err := readStateMetadata(stage)
	if err != nil {
		return empty, err
	}
	next := want
	next.Generation = id
	next.Metadata = make(map[string]string, len(nextFiles))
	for name, data := range nextFiles {
		next.Metadata[name] = stateDigest(data)
	}
	if _, ok := next.Metadata["root.json"]; !ok {
		return empty, errors.New("refreshed update root missing")
	}
	// Target bytes are returned from the verifier, not later read from cache.
	if err := os.RemoveAll(filepath.Join(stage, "targets")); err != nil {
		return empty, err
	}
	if err := syncStateFiles(stage); err != nil {
		return empty, err
	}
	if err := writeState(directory, next); err != nil {
		return empty, err
	}
	committed = true
	if err := os.Remove(marker); err != nil {
		return empty, err
	}
	if err := syncStateDirectory(directory); err != nil {
		return empty, err
	}
	// Only the exact preceding generation is retired; never traverse unrelated
	// user paths or adopt an orphan directory as a fallback trust source.
	_ = os.RemoveAll(old)
	if verifyErr != nil {
		return empty, errors.New("signed update refresh unavailable or rejected")
	}
	return policy, nil
}

func stateDigest(b []byte) string { h := sha256.Sum256(b); return hex.EncodeToString(h[:]) }
func newStateGeneration() (string, error) {
	var b [16]byte
	_, err := rand.Read(b[:])
	return hex.EncodeToString(b[:]), err
}

func plainStatePath(path string) error {
	if !filepath.IsAbs(path) || filepath.Clean(path) != path {
		return errors.New("absolute clean update path required")
	}
	for p := path; ; p = filepath.Dir(p) {
		st, err := os.Lstat(p)
		if err != nil || !st.IsDir() || st.Mode()&os.ModeSymlink != 0 {
			return errors.New("plain existing update directory required")
		}
		if err := rejectStateReparse(p); err != nil {
			return err
		}
		if filepath.Dir(p) == p {
			break
		}
	}
	return nil
}
func boundedStateRead(path string, limit int64) ([]byte, error) {
	st, err := os.Lstat(path)
	if err != nil || !st.Mode().IsRegular() || st.Size() <= 0 || st.Size() > limit {
		return nil, errors.New("invalid update-state file")
	}
	if err := rejectStateReparse(path); err != nil {
		return nil, err
	}
	f, err := os.Open(path)
	if err != nil {
		return nil, err
	}
	defer f.Close()
	b, err := io.ReadAll(io.LimitReader(f, limit+1))
	if err != nil || int64(len(b)) > limit {
		return nil, errors.New("update-state file exceeds limit")
	}
	return b, nil
}
func readState(directory string, want stateIdentity) (stateIdentity, error) {
	b, err := boundedStateRead(filepath.Join(directory, "state.json"), maxStateBytes)
	if err != nil {
		return stateIdentity{}, errors.New("update state unavailable; explicit recovery required")
	}
	var s stateIdentity
	if err := json.Unmarshal(b, &s); err != nil {
		return s, errors.New("invalid update state")
	}
	canonical, _ := json.Marshal(s)
	if !bytes.Equal(b, canonical) || s.Schema != 1 || s.BootstrapSHA256 != want.BootstrapSHA256 || s.MetadataURL != want.MetadataURL || s.Channel != want.Channel || !stateGeneration.MatchString(s.Generation) || len(s.Metadata) < 1 || len(s.Metadata) > 128 {
		return s, errors.New("update state identity mismatch")
	}
	return s, nil
}
func readStateMetadata(cache string) (map[string][]byte, error) {
	dir := filepath.Join(cache, "metadata")
	if err := plainStatePath(dir); err != nil {
		return nil, err
	}
	entries, err := os.ReadDir(dir)
	if err != nil || len(entries) < 1 || len(entries) > 128 {
		return nil, errors.New("invalid metadata inventory")
	}
	files := map[string][]byte{}
	total := 0
	for _, entry := range entries {
		name := entry.Name()
		if entry.IsDir() || !stateMetadataName.MatchString(name) || strings.Contains(name, "..") {
			return nil, errors.New("unexpected metadata file")
		}
		data, err := boundedStateRead(filepath.Join(dir, name), maxCacheFileBytes)
		if err != nil {
			return nil, err
		}
		total += len(data)
		if total > maxCacheTotalBytes {
			return nil, errors.New("metadata cache exceeds limit")
		}
		files[name] = data
	}
	return files, nil
}
func syncStateFiles(cache string) error {
	entries, err := os.ReadDir(filepath.Join(cache, "metadata"))
	if err != nil {
		return err
	}
	for _, e := range entries {
		f, err := os.OpenFile(filepath.Join(cache, "metadata", e.Name()), os.O_RDWR, 0)
		if err != nil {
			return err
		}
		err = f.Sync()
		closeErr := f.Close()
		if err != nil {
			return err
		}
		if closeErr != nil {
			return closeErr
		}
	}
	if err := syncStateDirectory(filepath.Join(cache, "metadata")); err != nil {
		return err
	}
	return syncStateDirectory(cache)
}
func writeState(directory string, s stateIdentity) error {
	b, err := json.Marshal(s)
	if err != nil || len(b) > maxStateBytes {
		return errors.New("invalid state serialization")
	}
	f, err := os.CreateTemp(directory, "state-pending-")
	if err != nil {
		return err
	}
	name := f.Name()
	defer os.Remove(name)
	if _, err = f.Write(b); err != nil {
		f.Close()
		return err
	}
	if err = f.Sync(); err != nil {
		f.Close()
		return err
	}
	if err = f.Close(); err != nil {
		return err
	}
	if err := replaceStateFile(name, filepath.Join(directory, "state.json")); err != nil {
		return err
	}
	return syncStateDirectory(directory)
}
