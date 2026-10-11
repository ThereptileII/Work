package verifier

import (
	"archive/zip"
	"bytes"
	"context"
	"encoding/binary"
	"encoding/json"
	"io"
	"net/http"
	"os"
	"path/filepath"
	"runtime"
	"sort"
	"strings"
	"sync/atomic"
	"testing"
)

type recoveryFixture struct {
	*prepareFixture
	requests atomic.Int32
	entries  map[string][]byte
}
type recoveryTransport struct {
	f    *recoveryFixture
	base http.RoundTripper
}

func (r recoveryTransport) RoundTrip(q *http.Request) (*http.Response, error) {
	if q.URL.Path == "/"+r.f.policy.Recovery.Path {
		r.f.requests.Add(1)
	}
	return r.base.RoundTrip(q)
}
func recoveryJSON(t *testing.T, value interface{}) []byte {
	t.Helper()
	b, e := json.Marshal(value)
	if e != nil {
		t.Fatal(e)
	}
	return b
}
func newRecoveryFixture(t *testing.T, sidecar bool) *recoveryFixture {
	t.Helper()
	f := &recoveryFixture{prepareFixture: newPrepareFixture(t), entries: map[string][]byte{
		"app/opencpn.exe":              []byte("inert exact application bytes"),
		"app/opennav-restart.exe":      []byte("inert helper bytes"),
		"app/skager-start.exe":         []byte("inert startup bytes"),
		"app/OPENNAV_PORTABLE_PREVIEW": []byte("portable-only marker"),
		"profile/opencpn.conf":         []byte("inert recovery profile"),
	}}
	f.entries["docs/PRODUCT_BUILD.json"] = recoveryJSON(t, map[string]interface{}{
		"version": f.policy.Version, "commit": f.policy.Commit, "test_fixtures": false, "build_purpose": "INSTALLED PRODUCT",
		"xnav_hardware_output_policy": "status-only", "xnav_manual_control_contract": 0,
		"executable_sha256":              recoveryDigest(f.entries["app/opencpn.exe"]),
		"restart_helper_sha256":          recoveryDigest(f.entries["app/opennav-restart.exe"]),
		"startup_launcher_sha256":        recoveryDigest(f.entries["app/skager-start.exe"]),
		"commissioning_restart_protocol": 1, "update_startup_health": 1,
	})
	if sidecar {
		var records []map[string]string
		for _, name := range recoveryNames(f.entries) {
			if (strings.HasPrefix(name, "app/") || strings.HasPrefix(name, "docs/")) && name != "app/OPENNAV_PORTABLE_PREVIEW" {
				records = append(records, map[string]string{"path": name, "sha256": recoveryDigest(f.entries[name])})
			}
		}
		f.entries["installer/package.json"] = recoveryJSON(t, map[string]interface{}{"schema": 1, "version": f.policy.Version, "commit": f.policy.Commit,
			"payloadSha256": strings.Repeat("b", 64), "supportedOpenCpn": f.policy.SupportedOpenCpn, "files": records, "qualification": "candidate"})
	}
	f.client.Transport = recoveryTransport{f, f.client.Transport}
	f.publish(t, recoveryArchive(t, f.entries, nil))
	return f
}
func recoveryNames(entries map[string][]byte) []string {
	names := make([]string, 0, len(entries))
	for n := range entries {
		names = append(names, n)
	}
	sort.Strings(names)
	return names
}
func recoveryArchive(t *testing.T, entries map[string][]byte, alter func(*zip.Writer)) []byte {
	t.Helper()
	entriesCopy := map[string][]byte{}
	inventory := map[string]string{}
	for name, body := range entries {
		entriesCopy[name] = body
		if name != "FILE_SHA256.json" {
			inventory[name] = recoveryDigest(body)
		}
	}
	if _, ok := entriesCopy["FILE_SHA256.json"]; !ok {
		entriesCopy["FILE_SHA256.json"] = recoveryJSON(t, inventory)
	}
	var b bytes.Buffer
	z := zip.NewWriter(&b)
	for _, name := range recoveryNames(entriesCopy) {
		w, e := z.Create(recoveryPrefix + name)
		if e != nil {
			t.Fatal(e)
		}
		if _, e = w.Write(entriesCopy[name]); e != nil {
			t.Fatal(e)
		}
	}
	if alter != nil {
		alter(z)
	}
	if e := z.Close(); e != nil {
		t.Fatal(e)
	}
	return b.Bytes()
}
func (f *recoveryFixture) publish(t *testing.T, archive []byte) {
	t.Helper()
	a := downloadTestArtifact(f.repository.server.URL, string(archive))
	a.Path = "releases/recovery.zip"
	a.URL = f.repository.server.URL + "/" + a.Path
	f.policy.Recovery = a
	f.repository.files["/"+a.Path] = archive
	f.repository.publish(t, 2, "beta", policyBytes(t, f.policy), false)
}
func (f *recoveryFixture) prepare(t *testing.T) (*PreparedRecovery, error) {
	t.Helper()
	offer := f.check(t)
	return prepareRecovery(context.Background(), f.config, f.observed, offer.Consent, f.client)
}

func TestPrepareRecoveryAuthenticatesActualInnerIdentities(t *testing.T) {
	f := newRecoveryFixture(t, true)
	p, err := f.prepare(t)
	if err != nil {
		t.Fatal(err)
	}
	defer p.Close()
	if f.requests.Load() != 1 || f.artifactRequests.Load() != 0 || p.Identity.Commit != f.policy.Commit || p.Identity.Version != f.policy.Version ||
		p.Identity.ExecutableSHA256 != recoveryDigest(f.entries["app/opencpn.exe"]) || !p.Identity.LifecycleManifestAvailable ||
		p.Identity.LifecycleManifestSHA256 != recoveryDigest(f.entries["installer/package.json"]) || p.Identity.DeclaredPayloadSHA256 != strings.Repeat("b", 64) {
		t.Fatal("wrong authenticated inner identity")
	}
	if p.Identity.LifecycleManifestSHA256 == p.Policy.Recovery.SHA256 || p.Identity.ExecutableSHA256 == p.Policy.Recovery.SHA256 {
		t.Fatal("outer hash substituted for inner identity")
	}
	if _, err := p.Artifact.file.Write([]byte("change")); err == nil {
		t.Fatal("artifact is writable")
	}
	if err := p.Close(); err != nil {
		t.Fatal(err)
	}
	assertDownloadDirectoryEmpty(t, f.config.ArtifactDirectory)
}
func TestPrepareRecoveryReportsMissingLifecycleManifest(t *testing.T) {
	f := newRecoveryFixture(t, false)
	p, err := f.prepare(t)
	if err != nil {
		t.Fatal(err)
	}
	defer p.Close()
	if p.Identity.LifecycleManifestAvailable || p.Identity.LifecycleManifestSHA256 != "" || p.Identity.DeclaredPayloadSHA256 != "" {
		t.Fatal("fabricated Lifecycle identity")
	}
}

func TestPrepareRecoveryAcceptsRealProducerContractVariants(t *testing.T) {
	for _, scenario := range []string{"accepted-compatibility", "manual-commissioning", "public-trust-provenance"} {
		t.Run(scenario, func(t *testing.T) {
			f := newRecoveryFixture(t, true)
			var product, manifest map[string]interface{}
			json.Unmarshal(f.entries["docs/PRODUCT_BUILD.json"], &product)
			json.Unmarshal(f.entries["installer/package.json"], &manifest)
			switch scenario {
			case "accepted-compatibility":
				manifest["qualification"] = "accepted"
				c := manifest["supportedOpenCpn"].([]interface{})[0].(map[string]interface{})
				c["integrationPackage"] = "SKAGER-Beta2-Setup.exe"
				c["qualificationCommit"] = strings.Repeat("b", 40)
				c["qualificationRun"] = "https://github.com/example/repo/actions/runs/123"
				c["qualificationEvidence"] = "docs/evidence/inert.json"
				c["stockSetupSha256"] = strings.Repeat("c", 64)
			case "manual-commissioning":
				product["xnav_hardware_output_policy"] = "manual-commissioning"
				product["xnav_manual_control_contract"] = 1
			case "public-trust-provenance":
				product["update_trust"] = map[string]interface{}{"schema": 1, "productCommit": f.policy.Commit,
					"sourcePath": "opennav-x/installer/windows/staging-update-trust.json", "gitPath": "opennav-x/installer/windows/staging-update-trust.json",
					"gitBlob": strings.Repeat("a", 40), "gitBlobSha256": strings.Repeat("b", 64), "sourceSha256": strings.Repeat("b", 64),
					"configSha256": strings.Repeat("b", 64), "channel": "beta"}
			}
			f.entries["docs/PRODUCT_BUILD.json"] = recoveryJSON(t, product)
			for _, record := range manifest["files"].([]interface{}) {
				r := record.(map[string]interface{})
				if r["path"] == "docs/PRODUCT_BUILD.json" {
					r["sha256"] = recoveryDigest(f.entries["docs/PRODUCT_BUILD.json"])
				}
			}
			f.entries["installer/package.json"] = recoveryJSON(t, manifest)
			f.publish(t, recoveryArchive(t, f.entries, nil))
			p, err := f.prepare(t)
			if err != nil {
				t.Fatal(err)
			}
			defer p.Close()
			if !p.Identity.LifecycleManifestAvailable {
				t.Fatal("lost exact producer sidecar")
			}
		})
	}
}

func TestPrepareRecoveryRejectsChangedSignedOuterBytes(t *testing.T) {
	for _, scenario := range []string{"hash", "length"} {
		t.Run(scenario, func(t *testing.T) {
			f := newRecoveryFixture(t, true)
			body := append([]byte(nil), f.repository.files["/"+f.policy.Recovery.Path]...)
			if scenario == "hash" {
				body[len(body)/2] ^= 1
			} else {
				body = append(body, 'x')
			}
			f.repository.files["/"+f.policy.Recovery.Path] = body
			p, err := f.prepare(t)
			if err == nil || p != nil {
				if p != nil {
					p.Close()
				}
				t.Fatal("accepted changed signed outer bytes")
			}
			assertDownloadDirectoryEmpty(t, f.config.ArtifactDirectory)
		})
	}
}
func TestPrepareRecoveryRefusesUnauthenticatedOrUnconfirmedAcquisition(t *testing.T) {
	for _, scenario := range []string{"wrong-consent", "malformed-consent", "policy-tamper", "metadata-rollback", "no-upgrade", "incompatible", "canceled", "unapproved-origin"} {
		t.Run(scenario, func(t *testing.T) {
			f := newRecoveryFixture(t, true)
			offer := f.check(t)
			consent := offer.Consent
			ctx := context.Background()
			switch scenario {
			case "wrong-consent":
				consent.PolicySHA256 = strings.Repeat("f", 64)
			case "malformed-consent":
				consent.Commit = "unverified"
			case "policy-tamper":
				f.repository.publish(t, 3, "beta", policyBytes(t, f.policy), false)
				f.repository.files["/targets.json"] = []byte("not signed metadata")
			case "metadata-rollback":
				f.repository.publish(t, 1, "beta", policyBytes(t, f.policy), false)
			case "no-upgrade":
				f.config.Installed.Version = f.policy.Version
				f.config.Installed.Commit = f.policy.Commit
			case "incompatible":
				f.observed.ExecutableSHA256 = strings.Repeat("f", 64)
			case "canceled":
				var cancel context.CancelFunc
				ctx, cancel = context.WithCancel(ctx)
				cancel()
			case "unapproved-origin":
				f.policy.Recovery.URL = "https://other.invalid/" + f.policy.Recovery.Path
				f.repository.publish(t, 3, "beta", policyBytes(t, f.policy), false)
				consent, _ = policyConsent(f.policy)
			}
			p, err := prepareRecovery(ctx, f.config, f.observed, consent, f.client)
			if err == nil || p != nil || f.requests.Load() != 0 {
				t.Fatalf("acquired without authenticated exact consent: %v", err)
			}
			assertDownloadDirectoryEmpty(t, f.config.ArtifactDirectory)
		})
	}
}
func TestPrepareRecoveryRejectsMalformedAuthenticatedArchive(t *testing.T) {
	for _, scenario := range []string{"not-zip", "duplicate", "case-alias", "traversal", "reserved", "wrong-root", "symlink", "file-directory-alias", "inventory-missing", "inventory-extra", "inventory-duplicate", "inventory-hash", "product-commit", "product-version", "product-case-key", "product-duplicate", "product-null-integer", "product-fraction", "product-unknown", "product-fixture", "product-executable", "package-commit", "package-file-hash", "package-incomplete", "package-duplicate-path", "package-schema", "package-null-schema", "package-compatibility", "package-unknown", "entry-oversize", "zip-count-bound", "zip-comment", "crc-corruption", "local-name-alias", "overlapping-offset", "record-oversize"} {
		t.Run(scenario, func(t *testing.T) {
			f := newRecoveryFixture(t, true)
			var alter func(*zip.Writer)
			product := map[string]interface{}{}
			if err := json.Unmarshal(f.entries["docs/PRODUCT_BUILD.json"], &product); err != nil {
				t.Fatal(err)
			}
			manifest := map[string]interface{}{}
			if err := json.Unmarshal(f.entries["installer/package.json"], &manifest); err != nil {
				t.Fatal(err)
			}
			switch scenario {
			case "duplicate", "case-alias", "wrong-root", "symlink":
				alter = func(z *zip.Writer) {
					name := recoveryPrefix + "app/opencpn.exe"
					if scenario == "case-alias" {
						name = recoveryPrefix + "app/OpenCPN.exe"
					}
					if scenario == "wrong-root" {
						name = "other/app.exe"
					}
					h := &zip.FileHeader{Name: name, Method: zip.Store}
					if scenario == "symlink" {
						h.Name = recoveryPrefix + "app/link"
						h.SetMode(os.ModeSymlink | 0777)
					}
					w, e := z.CreateHeader(h)
					if e != nil {
						t.Fatal(e)
					}
					w.Write([]byte("bad"))
				}
			case "traversal":
				f.entries["app/../escape"] = []byte("bad")
			case "reserved":
				f.entries["app/CON.dll"] = []byte("bad")
			case "file-directory-alias":
				f.entries["app"] = []byte("bad")
			case "inventory-missing":
				f.entries["FILE_SHA256.json"] = []byte(`{}`)
			case "inventory-extra":
				f.entries["FILE_SHA256.json"] = []byte(`{"missing":"` + strings.Repeat("a", 64) + `"}`)
			case "inventory-duplicate":
				f.entries["FILE_SHA256.json"] = []byte(`{"x":"a","X":"b"}`)
			case "inventory-hash":
				f.entries["FILE_SHA256.json"] = []byte(`{"app/opencpn.exe":"` + strings.Repeat("f", 64) + `"}`)
			case "product-commit":
				product["commit"] = strings.Repeat("f", 40)
			case "product-version":
				product["version"] = "3.0.0"
			case "product-case-key":
				product["Version"] = product["version"]
				delete(product, "version")
			case "product-null-integer":
				product["xnav_manual_control_contract"] = nil
			case "product-fraction":
				product["update_startup_health"] = 1.5
			case "product-unknown":
				product["approval"] = true
			case "product-fixture":
				product["test_fixtures"] = true
			case "product-executable":
				product["executable_sha256"] = strings.Repeat("f", 64)
			case "record-oversize":
				product["excess"] = strings.Repeat("x", 32<<10)
			case "package-commit":
				manifest["commit"] = strings.Repeat("f", 40)
			case "package-file-hash":
				manifest["files"].([]interface{})[0].(map[string]interface{})["sha256"] = strings.Repeat("f", 64)
			case "package-incomplete":
				manifest["files"] = manifest["files"].([]interface{})[1:]
			case "package-duplicate-path":
				records := manifest["files"].([]interface{})
				manifest["files"] = append(records, records[0])
			case "package-schema":
				manifest["schema"] = 2
			case "package-null-schema":
				manifest["schema"] = nil
			case "package-compatibility":
				manifest["supportedOpenCpn"].([]interface{})[0].(map[string]interface{})["executableSha256"] = strings.Repeat("f", 64)
			case "package-unknown":
				manifest["approved"] = true
			}
			if strings.HasPrefix(scenario, "product-") || scenario == "record-oversize" {
				f.entries["docs/PRODUCT_BUILD.json"] = recoveryJSON(t, product)
			}
			if scenario == "product-duplicate" {
				f.entries["docs/PRODUCT_BUILD.json"] = append([]byte(`{"version":"bad",`), f.entries["docs/PRODUCT_BUILD.json"][1:]...)
			}
			if strings.HasPrefix(scenario, "package-") {
				f.entries["installer/package.json"] = recoveryJSON(t, manifest)
			}
			archive := recoveryArchive(t, f.entries, alter)
			switch scenario {
			case "not-zip":
				archive = []byte("plaintext authority")
			case "zip-count-bound":
				binary.LittleEndian.PutUint16(archive[len(archive)-14:], maxRecoveryFiles+1)
				binary.LittleEndian.PutUint16(archive[len(archive)-12:], maxRecoveryFiles+1)
			case "zip-comment":
				archive = append(archive, 'x')
				binary.LittleEndian.PutUint16(archive[len(archive)-3:], 1)
			case "entry-oversize":
				i := bytes.Index(archive, []byte{'P', 'K', 1, 2})
				binary.LittleEndian.PutUint32(archive[i+24:], uint32(maxRecoveryMember+1))
			case "crc-corruption":
				i := bytes.Index(archive, []byte{'P', 'K', 1, 2})
				archive[i+16] ^= 1
			case "local-name-alias":
				archive[30] ^= 1
			case "overlapping-offset":
				i := bytes.Index(archive, []byte{'P', 'K', 1, 2})
				j := i + 46 + int(binary.LittleEndian.Uint16(archive[i+28:])) + int(binary.LittleEndian.Uint16(archive[i+30:])) + int(binary.LittleEndian.Uint16(archive[i+32:]))
				binary.LittleEndian.PutUint32(archive[j+42:], 0)
			}
			f.publish(t, archive)
			p, err := f.prepare(t)
			if err == nil || p != nil {
				if p != nil {
					p.Close()
				}
				t.Fatal("accepted malformed authenticated archive")
			}
			if f.requests.Load() != 1 {
				t.Fatal("malformed ZIP was not reached through authenticated download")
			}
			assertDownloadDirectoryEmpty(t, f.config.ArtifactDirectory)
		})
	}
}
func TestRecoveryUsesRetainedHandleNotArtifactPath(t *testing.T) {
	if runtime.GOOS == "windows" {
		t.Skip("native custody denies this rename; portable read-handle seam only")
	}
	f := newRecoveryFixture(t, true)
	p, err := f.prepare(t)
	if err != nil {
		t.Fatal(err)
	}
	defer p.Close()
	path := p.Artifact.Path()
	moved := filepath.Join(f.config.ArtifactDirectory, "held-original")
	if err := os.Rename(path, moved); err != nil {
		t.Fatal(err)
	}
	defer func() { os.Remove(path); os.Rename(moved, path) }()
	if err := os.WriteFile(path, []byte("untrusted pathname replacement"), 0600); err != nil {
		t.Fatal(err)
	}
	identity, err := readRecoveryIdentity(context.Background(), p.Artifact, p.Policy)
	if err != nil || identity != p.Identity {
		t.Fatalf("reopened mutable pathname: %v", err)
	}
	if _, err := p.Artifact.Seek(0, io.SeekStart); err != nil {
		t.Fatal(err)
	}
}
