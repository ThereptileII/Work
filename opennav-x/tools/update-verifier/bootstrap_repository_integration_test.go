//go:build linux

package verifier

import (
	"archive/zip"
	"bytes"
	"crypto/ed25519"
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"os"
	"path/filepath"
	"strings"
	"testing"
	"time"

	"example.com/opennav-update-verifier/releasepolicy"
	"example.com/opennav-update-verifier/repository"
)

// The real operator publishes inert local assets with ephemeral keys. Fixture
// qualification strings satisfy the input format only; they qualify no product.
func TestBrokerBootstrapRepositoryPublisherToReader(t *testing.T) {
	base := t.TempDir()
	directory, assets := filepath.Join(base, "repository"), filepath.Join(base, "assets")
	if err := os.Mkdir(assets, 0700); err != nil {
		t.Fatal(err)
	}
	write := func(name string, data []byte) {
		t.Helper()
		if err := os.WriteFile(filepath.Join(assets, name), data, 0600); err != nil {
			t.Fatal(err)
		}
	}
	marshal := func(value any) []byte {
		t.Helper()
		data, err := json.Marshal(value)
		if err != nil {
			t.Fatal(err)
		}
		return data
	}
	digest := func(data []byte) string {
		sum := sha256.Sum256(data)
		return hex.EncodeToString(sum[:])
	}

	keys := repository.Keys{}
	defer keys.Clear()
	for _, role := range []string{"root", "targets", "snapshot", "timestamp"} {
		_, key, err := ed25519.GenerateKey(nil)
		if err != nil {
			t.Fatal(err)
		}
		keys[role] = key
	}
	server := httptest.NewTLSServer(http.FileServer(http.Dir(filepath.Join(directory, "public"))))
	defer server.Close()
	now := time.Now().UTC()
	if err := repository.Initialize(directory, server.URL, server.URL, keys, now); err != nil {
		t.Fatal(err)
	}
	trustBytes, err := repository.Export(directory, "config", now)
	if err != nil {
		t.Fatal(err)
	}
	trust, err := repository.ValidateTrust(trustBytes, now)
	if err != nil {
		t.Fatal(err)
	}
	// Initialization fixes the stock allowlist; do not supply a fixture-specific
	// compatibility expansion to make this publication pass.
	stock := releasepolicy.OpenCpn{Version: "5.12.4", Arch: "x86",
		ExecutableSHA256: "7c6547562cca7954671eaab72833ca9d788710fd9808b6a699b6dc823852ae0c",
		UpstreamCommit:   "37fd0cddb7334fe489e9f18aa163977a9c5c84f7"}
	if len(trust.LocalCompatibilityAllowlist) != 1 || trust.LocalCompatibilityAllowlist[0] != stock {
		t.Fatal("operator initialized an unexpected stock allowlist")
	}

	commit, version := strings.Repeat("a", 40), "0.4.1-beta.1"
	names := []string{"SKAGER-Beta2-Release-Notes.md", "SKAGER-Beta2-Setup.exe", "SKAGER-Beta2-Portable-Recovery.zip", "SKAGER-Beta2-source.zip", "SOURCE_AND_LICENSES.md"}
	notices := []byte("SANITIZED INERT LOCAL TEST NOTICE\n")
	var recovery bytes.Buffer
	archive := zip.NewWriter(&recovery)
	entry, err := archive.Create("SKAGER-Beta2-Portable-Recovery/docs/SOURCE_AND_LICENSES.md")
	if err != nil {
		t.Fatal(err)
	}
	if _, err := entry.Write(notices); err != nil {
		t.Fatal(err)
	}
	if err := archive.Close(); err != nil {
		t.Fatal(err)
	}
	contents := [][]byte{[]byte("INERT TEST RELEASE NOTES"), []byte("INERT TEST SETUP - NEVER EXECUTE"), recovery.Bytes(), []byte("INERT TEST SOURCE ARCHIVE"), notices}
	artifacts := make([]releasepolicy.Artifact, len(names))
	selected := make([]repository.SelectedFile, len(names))
	records := make([]map[string]any, 0, 4)
	for i, name := range names {
		write(name, contents[i])
		path := "artifacts/" + commit + "/" + name
		artifacts[i] = releasepolicy.Artifact{Path: path, URL: server.URL + "/" + path, SHA256: digest(contents[i]), Bytes: int64(len(contents[i]))}
		selected[i] = repository.SelectedFile{Path: path, SHA256: artifacts[i].SHA256, Bytes: artifacts[i].Bytes}
		if i < 4 {
			records = append(records, map[string]any{"name": name, "sha256": artifacts[i].SHA256, "size": artifacts[i].Bytes})
		}
	}
	app := releasepolicy.Policy{Schema: 1, Product: "skager", Channel: "beta", Version: version, Commit: commit,
		ReleaseNotes: artifacts[0], Installer: artifacts[1], Recovery: artifacts[2],
		CorrespondingSource: artifacts[3], Notices: artifacts[4], SupportedOpenCpn: []releasepolicy.OpenCpn{stock}}
	appBytes := marshal(app)
	manifest := marshal(map[string]any{"schemaVersion": 1, "channel": "staging", "version": version, "commit": commit, "files": records})
	qualification := marshal(map[string]any{"schemaVersion": 2, "channel": "staging", "commit": commit, "publicAccess": false,
		"gates": map[string]string{"linux": "passed", "windows": "passed", "installer": "passed", "package": "passed", "restart": "passed"}})
	write("RELEASE.json", manifest)
	write("QUALIFICATION.json", qualification)
	selection := marshal(repository.Selection{Schema: 1, Version: version, Commit: commit,
		ReleaseManifestSHA256: digest(manifest), QualificationSHA256: digest(qualification), PolicySHA256: digest(appBytes), Files: selected})

	bootstrapArtifact := func(name string) releasepolicy.Artifact {
		data := []byte("INERT LOCAL BOOTSTRAP TEST - NEVER EXECUTE: " + name)
		write(name, data)
		path := "artifacts/" + commit + "/" + name
		return releasepolicy.Artifact{Path: path, URL: server.URL + "/" + path, SHA256: digest(data), Bytes: int64(len(data))}
	}
	bootstrap := releasepolicy.BrokerBootstrapPolicy{Schema: 1, Product: "skager", Purpose: releasepolicy.BrokerBootstrapPurpose,
		Channel: "beta", Version: version, Commit: commit, Architecture: "x86", ServiceName: "SkagerUpdateBroker",
		ServiceAccount: `NT SERVICE\SkagerUpdateBroker`, CodeLocation: "ProgramFilesX86/SKAGER/UpdateBroker", DataLocation: "ProgramData/SKAGER/UpdateBroker",
		TrustConfigSHA256: digest(trustBytes), Broker: bootstrapArtifact("skager-update-broker.exe"),
		Provisioner: bootstrapArtifact("skager-update-provisioner.exe"), CorrespondingSource: app.CorrespondingSource, Notices: app.Notices}
	// Preserve deliberate whitespace through publication and verification: the
	// independent approval concerns these exact bytes, not a JSON re-encoding.
	bootstrapBytes := append(append([]byte(" \n"), marshal(bootstrap)...), '\n')
	approvedPolicySHA := digest(bootstrapBytes)
	online := repository.Keys{"targets": keys["targets"], "snapshot": keys["snapshot"], "timestamp": keys["timestamp"]}
	if publication, err := repository.PublishWithBrokerBootstrap(directory, assets, selection, appBytes, digest(selection), bootstrapBytes, approvedPolicySHA, online, now); err != nil || publication != 1 {
		t.Fatalf("actual operator publication failed: version=%d, %v", publication, err)
	}

	cache := filepath.Join(base, "reader-cache")
	got, owned, err := verifyBrokerBootstrapWithClient(BrokerBootstrapRequest{MetadataURL: trust.MetadataURL, TrustedRoot: trust.BootstrapRoot,
		CacheDir: cache, Channel: trust.Channel, ArtifactOrigin: trust.ArtifactOrigin, ApprovedPolicySHA256: approvedPolicySHA,
		TrustedProvisionerSHA256: bootstrap.Provisioner.SHA256, TrustConfigSHA256: digest(trustBytes)}, time.Now().Add(maxOperationTime), server.Client())
	if err != nil || got != bootstrap || !bytes.Equal(owned, bootstrapBytes) {
		t.Fatalf("actual bootstrap reader did not return the exact published policy: %v", err)
	}
	owned[0] = 'x'
	published, err := os.ReadFile(filepath.Join(directory, "public", "targets", "bootstrap", approvedPolicySHA+".broker-beta.json"))
	if err != nil || !bytes.Equal(published, bootstrapBytes) {
		t.Fatalf("returned policy bytes do not have independent ownership: %v", err)
	}
	identity, ordinary, err := verifyWithClient(Request{MetadataURL: trust.MetadataURL, TrustedRoot: trust.BootstrapRoot,
		CacheDir: cache, Channel: trust.Channel, TargetPath: "releases/beta.json"}, time.Now().Add(maxOperationTime), server.Client())
	if err != nil || identity != (Release{Channel: app.Channel, Version: app.Version, Commit: app.Commit}) || !bytes.Equal(ordinary, appBytes) {
		t.Fatalf("ordinary release verification changed in the same publication: %v", err)
	}
}
