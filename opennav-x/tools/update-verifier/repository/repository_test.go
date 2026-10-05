//go:build linux

package repository

import (
	"archive/zip"
	"bytes"
	"crypto/ed25519"
	"encoding/base64"
	"encoding/json"
	"errors"
	"net/http"
	"net/http/httptest"
	"os"
	"path/filepath"
	"strings"
	"testing"
	"time"

	"example.com/opennav-update-verifier/releasepolicy"
	"github.com/theupdateframework/go-tuf/v2/metadata/config"
	"github.com/theupdateframework/go-tuf/v2/metadata/updater"
)

type fixture struct {
	dir, assets       string
	keys              Keys
	now               time.Time
	trust             Trust
	root              []byte
	selection, policy []byte
	server            *httptest.Server
}

func put(t *testing.T, name string, data []byte) {
	t.Helper()
	if e := os.WriteFile(name, data, 0600); e != nil {
		t.Fatal(e)
	}
}
func newFixture(t *testing.T) *fixture {
	t.Helper()
	base := t.TempDir()
	f := &fixture{dir: filepath.Join(base, "repository"), assets: filepath.Join(base, "assets"), keys: Keys{}, now: time.Now().UTC()}
	if e := os.Mkdir(f.assets, 0700); e != nil {
		t.Fatal(e)
	}
	// Ephemeral keys occur only in tests, never in the operator implementation.
	for _, role := range roles {
		_, k, e := ed25519.GenerateKey(nil)
		if e != nil {
			t.Fatal(e)
		}
		f.keys[role] = k
	}
	f.server = httptest.NewTLSServer(http.FileServer(http.Dir(filepath.Join(f.dir, "public"))))
	t.Cleanup(f.server.Close)
	if e := Initialize(f.dir, f.server.URL, f.server.URL, f.keys, f.now); e != nil {
		t.Fatal(e)
	}
	b, e := Export(f.dir, "config", f.now)
	if e != nil {
		t.Fatal(e)
	}
	f.trust, e = ValidateTrust(b, f.now)
	if e != nil {
		t.Fatal(e)
	}
	f.root = f.trust.BootstrapRoot
	f.release(t, "0.4.1-beta.1", strings.Repeat("a", 40))
	return f
}
func (f *fixture) online() Keys {
	k := Keys{}
	for _, role := range roles[1:] {
		k[role] = f.keys[role]
	}
	return k
}
func (f *fixture) release(t *testing.T, version, commit string) {
	t.Helper()
	names := []string{"SKAGER-Beta2-Release-Notes.md", "SKAGER-Beta2-Setup.exe", "SKAGER-Beta2-Portable-Recovery.zip", "SKAGER-Beta2-source.zip", "SOURCE_AND_LICENSES.md"}
	for _, n := range names {
		put(t, filepath.Join(f.assets, n), []byte(n+commit))
	}
	var b bytes.Buffer
	z := zip.NewWriter(&b)
	w, e := z.Create("SKAGER-Beta2-Portable-Recovery/docs/SOURCE_AND_LICENSES.md")
	if e != nil {
		t.Fatal(e)
	}
	notice, _ := os.ReadFile(filepath.Join(f.assets, names[4]))
	w.Write(notice)
	z.Close()
	put(t, filepath.Join(f.assets, names[2]), b.Bytes())
	as := []releasepolicy.Artifact{}
	records := []any{}
	selected := []SelectedFile{}
	for i, n := range names {
		raw, _ := os.ReadFile(filepath.Join(f.assets, n))
		p := "artifacts/" + commit + "/" + n
		a := releasepolicy.Artifact{Path: p, URL: f.trust.ArtifactOrigin + "/" + p, SHA256: checksum(raw), Bytes: int64(len(raw))}
		as = append(as, a)
		selected = append(selected, SelectedFile{p, a.SHA256, a.Bytes})
		if i < 4 {
			records = append(records, map[string]any{"name": n, "sha256": a.SHA256, "size": a.Bytes})
		}
	}
	policy := releasepolicy.Policy{Schema: 1, Product: "skager", Channel: "beta", Version: version, Commit: commit, ReleaseNotes: as[0], Installer: as[1], Recovery: as[2], CorrespondingSource: as[3], Notices: as[4], SupportedOpenCpn: []releasepolicy.OpenCpn{stock}}
	f.policy = jsonBytes(policy)
	manifest := jsonBytes(map[string]any{"schemaVersion": 1, "channel": "staging", "version": version, "commit": commit, "files": records})
	gates := map[string]string{}
	for _, g := range []string{"linux", "windows", "installer", "package", "restart"} {
		gates[g] = "passed"
	}
	q := jsonBytes(map[string]any{"schemaVersion": 2, "channel": "staging", "commit": commit, "publicAccess": false, "gates": gates})
	put(t, filepath.Join(f.assets, "RELEASE.json"), manifest)
	put(t, filepath.Join(f.assets, "QUALIFICATION.json"), q)
	f.selection = jsonBytes(Selection{1, version, commit, checksum(manifest), checksum(q), checksum(f.policy), selected})
}
func (f *fixture) publish() (int64, error) {
	return Publish(f.dir, f.assets, f.selection, f.policy, checksum(f.selection), f.online(), f.now)
}
func (f *fixture) verify(t *testing.T) ([]byte, error) {
	t.Helper()
	cfg, e := config.New(f.server.URL, f.root)
	if e != nil {
		return nil, e
	}
	cfg.LocalMetadataDir = t.TempDir()
	cfg.LocalTargetsDir = t.TempDir()
	if e = cfg.SetDefaultFetcherHTTPClient(f.server.Client()); e != nil {
		return nil, e
	}
	u, e := updater.New(cfg)
	if e != nil {
		return nil, e
	}
	if e = u.Refresh(); e != nil {
		return nil, e
	}
	target, e := u.GetTargetInfo("releases/beta.json")
	if e != nil {
		return nil, e
	}
	_, b, e := u.DownloadTarget(target, "", "")
	return b, e
}
func TestSignedMetadataAndTamper(t *testing.T) {
	f := newFixture(t)
	if v, e := f.publish(); e != nil || v != 1 {
		t.Fatalf("publish: %d %v", v, e)
	}
	got, e := f.verify(t)
	if e != nil || !bytes.Equal(got, f.policy) {
		t.Fatalf("real go-tuf verification failed: %v", e)
	}
	name := filepath.Join(f.dir, "public", "targets", "releases", checksum(f.policy)+".beta.json")
	b := append([]byte{}, f.policy...)
	b[1] = 'x'
	put(t, name, b)
	if _, e = f.verify(t); e == nil {
		t.Fatal("modified target accepted")
	}
}
func TestPartialPublicationAndMonotonicFloors(t *testing.T) {
	f := newFixture(t)
	if _, e := f.publish(); e != nil {
		t.Fatal(e)
	}
	stamp, _ := os.ReadFile(filepath.Join(f.dir, "public", "timestamp.json"))
	oldPolicy := append([]byte{}, f.policy...)
	f.release(t, "0.4.1-beta.2", strings.Repeat("b", 40))
	_, e := publish(f.dir, f.assets, f.selection, f.policy, checksum(f.selection), f.online(), f.now, func(p string) error {
		if p == "metadata-staged" {
			return errors.New("injected interruption")
		}
		return nil
	})
	if e == nil {
		t.Fatal("interruption missed")
	}
	after, _ := os.ReadFile(filepath.Join(f.dir, "public", "timestamp.json"))
	if !bytes.Equal(stamp, after) {
		t.Fatal("partial publication exposed new timestamp")
	}
	got, e := f.verify(t)
	if e != nil || !bytes.Equal(got, oldPolicy) {
		t.Fatalf("prior publication lost: %v", e)
	}
	if v, e := f.publish(); e != nil || v != 3 {
		t.Fatalf("retry reset reserved version: %d %v", v, e)
	}
	got, e = f.verify(t)
	if e != nil || !bytes.Equal(got, f.policy) {
		t.Fatalf("new policy unavailable: %v", e)
	}
	f.release(t, "0.4.1-beta.1", strings.Repeat("a", 40))
	if _, e = f.publish(); e == nil {
		t.Fatal("downgrade accepted")
	}
	f.release(t, "0.4.1-beta.2", strings.Repeat("c", 40))
	if _, e = f.publish(); e == nil {
		t.Fatal("same-version replacement accepted")
	}
}
func TestInputTamperAndWrongKeysNeverReserve(t *testing.T) {
	for _, scenario := range []string{"artifact", "policy", "selection pin", "manifest", "qualification", "notices", "key", "expired root"} {
		t.Run(scenario, func(t *testing.T) {
			f := newFixture(t)
			before, _ := os.ReadFile(filepath.Join(f.dir, "state.json"))
			pin := checksum(f.selection)
			keys := f.online()
			switch scenario {
			case "artifact":
				put(t, filepath.Join(f.assets, "SKAGER-Beta2-Setup.exe"), []byte("tampered"))
			case "policy":
				f.policy = append(f.policy, ' ')
			case "selection pin":
				pin = strings.Repeat("0", 64)
			case "manifest":
				put(t, filepath.Join(f.assets, "RELEASE.json"), []byte("{}"))
			case "qualification":
				put(t, filepath.Join(f.assets, "QUALIFICATION.json"), []byte("{}"))
			case "notices":
				put(t, filepath.Join(f.assets, "SOURCE_AND_LICENSES.md"), []byte("unreviewed"))
			case "key":
				_, keys["targets"], _ = ed25519.GenerateKey(nil)
			case "expired root":
				f.now = f.now.Add(366 * 24 * time.Hour)
			}
			if _, e := Publish(f.dir, f.assets, f.selection, f.policy, pin, keys, f.now); e == nil {
				t.Fatal("unsafe publication accepted")
			}
			after, _ := os.ReadFile(filepath.Join(f.dir, "state.json"))
			if !bytes.Equal(before, after) {
				t.Fatal("invalid selection reserved state")
			}
		})
	}
}
func TestSymlinkAndConcurrentWriterRefusal(t *testing.T) {
	f := newFixture(t)
	unlock, e := lock(f.dir)
	if e != nil {
		t.Fatal(e)
	}
	if _, e = f.publish(); e == nil {
		t.Fatal("concurrent writer accepted")
	}
	unlock()
	external := filepath.Join(t.TempDir(), "private")
	put(t, external, []byte("private"))
	name := filepath.Join(f.assets, "SKAGER-Beta2-Setup.exe")
	os.Remove(name)
	if e = os.Symlink(external, name); e != nil {
		t.Fatal(e)
	}
	if _, e = f.publish(); e == nil {
		t.Fatal("source symlink accepted")
	}
	os.Remove(name)
	f.release(t, "0.4.1-beta.1", strings.Repeat("a", 40))
	os.Remove(name)
	put(t, name, []byte("SKAGER-Beta2-Setup.exe"+strings.Repeat("a", 40)))
	// No nested output directory may be created through a symlink.
	outside := t.TempDir()
	if e = os.Mkdir(filepath.Join(f.dir, "public", "artifacts"), 0700); e != nil {
		t.Fatal(e)
	}
	if e = os.Symlink(outside, filepath.Join(f.dir, "public", "artifacts", strings.Repeat("a", 40))); e != nil {
		t.Fatal(e)
	}
	if _, e = f.publish(); e == nil {
		t.Fatal("destination symlink accepted")
	}
	entries, _ := os.ReadDir(outside)
	if len(entries) != 0 {
		t.Fatal("created files outside repository")
	}
}
func TestReadKeysAndPublicConfiguration(t *testing.T) {
	f := newFixture(t)
	env := map[string]any{"schema": 1}
	keys := map[string]string{}
	for role, k := range f.keys {
		keys[role] = base64.StdEncoding.EncodeToString(k.Seed())
	}
	env["keys"] = keys
	raw := jsonBytes(env)
	parsed, e := ReadKeys(bytes.NewReader(raw), true)
	if e != nil {
		t.Fatal(e)
	}
	defer parsed.Clear()
	keys["targets"] = keys["root"]
	if _, e = ReadKeys(bytes.NewReader(jsonBytes(env)), true); e == nil {
		t.Fatal("shared signing roles accepted")
	}
	if _, e = ReadKeys(strings.NewReader(`{"schema":1,"schema":1,"keys":{}}`), true); e == nil {
		t.Fatal("duplicate key accepted")
	}
	config, e := Export(f.dir, "config", f.now)
	if e != nil {
		t.Fatal(e)
	}
	for _, k := range f.keys {
		if bytes.Contains(config, []byte(base64.StdEncoding.EncodeToString(k.Seed()))) {
			t.Fatal("private key in public export")
		}
	}
	if _, e = ValidateTrust(bytes.Replace(config, []byte(`"channel": "beta"`), []byte(`"channel": "stable"`), 1), f.now); e == nil {
		t.Fatal("stable configuration accepted")
	}
	var c Trust
	json.Unmarshal(config, &c)
	c.MetadataURL = "https://example.invalid/?token=secret"
	if _, e = ValidateTrust(jsonBytes(c), f.now); e == nil {
		t.Fatal("credential-like query accepted")
	}
	if e = Initialize(f.dir, f.server.URL, f.server.URL, f.keys, f.now); e == nil {
		t.Fatal("existing state reset by initialize")
	}
}

func TestRootSignatureAndFailedQualificationCannotBeRepinned(t *testing.T) {
	f := newFixture(t)
	config, _ := Export(f.dir, "config", f.now)
	var c Trust
	json.Unmarshal(config, &c)
	var root map[string]any
	json.Unmarshal(c.BootstrapRoot, &root)
	signatures := root["signatures"].([]any)
	signatures[0].(map[string]any)["sig"] = strings.Repeat("0", 128)
	c.BootstrapRoot = jsonBytes(root)
	if _, e := ValidateTrust(jsonBytes(c), f.now); e == nil {
		t.Fatal("unsigned/tampered bootstrap root accepted")
	}
	var selection Selection
	json.Unmarshal(f.selection, &selection)
	raw, _ := os.ReadFile(filepath.Join(f.assets, "QUALIFICATION.json"))
	var q map[string]any
	json.Unmarshal(raw, &q)
	q["gates"].(map[string]any)["installer"] = "failed"
	raw = jsonBytes(q)
	put(t, filepath.Join(f.assets, "QUALIFICATION.json"), raw)
	selection.QualificationSHA256 = checksum(raw)
	f.selection = jsonBytes(selection)
	if _, e := f.publish(); e == nil {
		t.Fatal("failed qualification accepted after repinning selection")
	}
}

// Models a source which keeps growing after its initial stat. The bounded read
// must stop at max+1 and must not return partial data as a valid record.
type growingReader struct{ consumed int }

func (r *growingReader) Read(p []byte) (int, error) {
	clear(p)
	r.consumed += len(p)
	return len(p), nil
}
func TestGrowingInputCannotEscapeReadBound(t *testing.T) {
	reader := &growingReader{}
	if data, e := readBounded(reader, 1024); e == nil || data != nil {
		t.Fatal("unbounded/growing record accepted")
	}
	if reader.consumed != 1025 {
		t.Fatalf("reader consumed %d bytes; expected hard max+1", reader.consumed)
	}
	if data, e := readBounded(strings.NewReader("exact"), 5); e != nil || string(data) != "exact" {
		t.Fatal("exact bounded record refused")
	}
	name := filepath.Join(t.TempDir(), "record")
	put(t, name, []byte("too large"))
	if _, e := readPlain(name, 4); e == nil {
		t.Fatal("oversized regular file accepted")
	}
}
