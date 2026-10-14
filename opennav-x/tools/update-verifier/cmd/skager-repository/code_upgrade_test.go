//go:build linux

package main

import (
	"archive/zip"
	"bytes"
	"crypto/ed25519"
	"crypto/sha256"
	"encoding/base64"
	"encoding/hex"
	"encoding/json"
	"os"
	"path/filepath"
	"strconv"
	"strings"
	"testing"
	"time"

	"example.com/opennav-update-verifier/releasepolicy"
	"example.com/opennav-update-verifier/repository"
	"github.com/theupdateframework/go-tuf/v2/metadata"
)

func TestCodeUpgradeCLIFlagsRefuseBeforePrivateKeyRead(t *testing.T) {
	for _, flags := range [][]string{
		{"--broker-code-upgrade-policy", "missing.json"},
		{"--broker-code-upgrade-policy-sha256", strings.Repeat("a", 64)},
		{"--broker-code-upgrade-policy", "missing.json", "--broker-code-upgrade-policy", "missing.json", "--broker-code-upgrade-policy-sha256", strings.Repeat("a", 64)},
		{"--broker-code-upgrade-policy-sha", strings.Repeat("a", 64)},
		{"--broker-code-upgrade-policy=", "--broker-code-upgrade-policy-sha256="},
		{"--broker-code-upgrade-policy", "missing.json", "--broker-code-upgrade-policy-sha256", strings.Repeat("A", 64)},
		{"--broker-code-upgrade-policy", "missing.json", "--broker-code-upgrade-policy-sha256", "malformed"},
	} {
		var out bytes.Buffer
		args := append([]string{"publish", "--repository", "missing", "--keys-file", "missing-private-input"}, flags...)
		e := run(args, &out)
		if e == nil || out.Len() != 0 || strings.Contains(e.Error(), "key file") {
			t.Fatalf("bad approval reached private key input: %v", e)
		}
	}
}

// Exercises the real CLI -> approval read -> repository -> signed target path.
// All signing material and package contents are ephemeral, inert test fixtures.
func TestCodeUpgradeCLIActualSignedPublicationAndOrdinaryRetention(t *testing.T) {
	base := t.TempDir()
	repo := filepath.Join(base, "repository")
	assets := filepath.Join(base, "assets")
	if e := os.Mkdir(assets, 0700); e != nil {
		t.Fatal(e)
	}
	marshal := func(v any) []byte {
		b, e := json.Marshal(v)
		if e != nil {
			t.Fatal(e)
		}
		return b
	}
	pin := func(b []byte) string { h := sha256.Sum256(b); return hex.EncodeToString(h[:]) }
	write := func(name string, b []byte) {
		if e := os.WriteFile(name, b, 0600); e != nil {
			t.Fatal(e)
		}
	}
	keys := repository.Keys{}
	encoded := map[string]string{}
	for _, role := range []string{"root", "targets", "snapshot", "timestamp"} {
		_, k, e := ed25519.GenerateKey(nil)
		if e != nil {
			t.Fatal(e)
		}
		keys[role] = k
		if role != "root" {
			encoded[role] = base64.StdEncoding.EncodeToString(k.Seed())
		}
	}
	defer keys.Clear()
	if e := repository.Initialize(repo, "https://fixture.invalid", "https://fixture.invalid", keys, time.Now()); e != nil {
		t.Fatal(e)
	}
	keyPath := filepath.Join(base, "ephemeral-test-keys.json")
	write(keyPath, marshal(map[string]any{"schema": 1, "keys": encoded}))
	config, e := repository.Export(repo, "config", time.Now())
	if e != nil {
		t.Fatal(e)
	}
	artifact := func(commit, name string, b []byte) releasepolicy.Artifact {
		write(filepath.Join(assets, name), b)
		path := "artifacts/" + commit + "/" + name
		return releasepolicy.Artifact{Path: path, URL: "https://fixture.invalid/" + path, SHA256: pin(b), Bytes: int64(len(b))}
	}
	var bootstrap releasepolicy.BrokerBootstrapPolicy
	publish := func(version, commit string, old *releasepolicy.BrokerBootstrapPolicy, includeBootstrap bool) {
		notices := []byte("INERT NOTICES " + commit)
		var recovery bytes.Buffer
		z := zip.NewWriter(&recovery)
		w, e := z.Create("SKAGER-Beta2-Portable-Recovery/docs/SOURCE_AND_LICENSES.md")
		if e != nil {
			t.Fatal(e)
		}
		w.Write(notices)
		z.Close()
		app := releasepolicy.Policy{Schema: 1, Product: "skager", Channel: "beta", Version: version, Commit: commit,
			ReleaseNotes:        artifact(commit, "SKAGER-Beta2-Release-Notes.md", []byte("INERT NOTES "+commit)),
			Installer:           artifact(commit, "SKAGER-Beta2-Setup.exe", []byte("INERT APP "+commit)),
			Recovery:            artifact(commit, "SKAGER-Beta2-Portable-Recovery.zip", recovery.Bytes()),
			CorrespondingSource: artifact(commit, "SKAGER-Beta2-source.zip", []byte("INERT SOURCE "+commit)),
			Notices:             artifact(commit, "SOURCE_AND_LICENSES.md", notices), SupportedOpenCpn: []releasepolicy.OpenCpn{{Version: "5.12.4", Arch: "x86", ExecutableSHA256: "7c6547562cca7954671eaab72833ca9d788710fd9808b6a699b6dc823852ae0c", UpstreamCommit: "37fd0cddb7334fe489e9f18aa163977a9c5c84f7"}}}
		list := []releasepolicy.Artifact{app.ReleaseNotes, app.Installer, app.Recovery, app.CorrespondingSource, app.Notices}
		records := []any{}
		selected := []repository.SelectedFile{}
		for i, a := range list {
			selected = append(selected, repository.SelectedFile{Path: a.Path, SHA256: a.SHA256, Bytes: a.Bytes})
			if i < 4 {
				records = append(records, map[string]any{"name": filepath.Base(a.Path), "sha256": a.SHA256, "size": a.Bytes})
			}
		}
		manifest := marshal(map[string]any{"schemaVersion": 1, "channel": "staging", "version": version, "commit": commit, "files": records})
		write(filepath.Join(assets, "RELEASE.json"), manifest)
		qualification := marshal(map[string]any{"schemaVersion": 2, "channel": "staging", "commit": commit, "publicAccess": false, "gates": map[string]string{"linux": "passed", "windows": "passed", "installer": "passed", "package": "passed", "restart": "passed"}})
		write(filepath.Join(assets, "QUALIFICATION.json"), qualification)
		policy := marshal(app)
		policyPath := filepath.Join(base, "app.json")
		write(policyPath, policy)
		selection := marshal(repository.Selection{Schema: 1, Version: version, Commit: commit, ReleaseManifestSHA256: pin(manifest), QualificationSHA256: pin(qualification), PolicySHA256: pin(policy), Files: selected})
		selectionPath := filepath.Join(base, "selection.json")
		write(selectionPath, selection)
		args := []string{"publish", "--repository", repo, "--assets", assets, "--policy", policyPath, "--selection", selectionPath, "--selection-sha256", pin(selection), "--keys-file", keyPath}
		if includeBootstrap || old != nil {
			bootstrap = releasepolicy.BrokerBootstrapPolicy{Schema: 1, Product: "skager", Purpose: releasepolicy.BrokerBootstrapPurpose, Channel: "beta", Version: version, Commit: commit, Architecture: "x86", ServiceName: "SkagerUpdateBroker", ServiceAccount: `NT SERVICE\SkagerUpdateBroker`, CodeLocation: "ProgramFilesX86/SKAGER/UpdateBroker", DataLocation: "ProgramData/SKAGER/UpdateBroker", TrustConfigSHA256: pin(config), Broker: artifact(commit, "skager-update-broker.exe", []byte("INERT BROKER "+commit)), Provisioner: artifact(commit, "skager-update-provisioner.exe", []byte("INERT HELPER "+commit)), CorrespondingSource: app.CorrespondingSource, Notices: app.Notices}
		}
		if includeBootstrap {
			raw := marshal(bootstrap)
			name := filepath.Join(base, "bootstrap.json")
			write(name, raw)
			args = append(args, "--broker-bootstrap-policy", name, "--broker-bootstrap-policy-sha256", pin(raw))
		}
		if old != nil {
			raw := marshal(releasepolicy.BrokerCodeUpgradePolicy{Schema: 1, Purpose: releasepolicy.BrokerCodeUpgradePurpose, PreviousBrokerSHA256: old.Broker.SHA256, PreviousHelperSHA256: old.Provisioner.SHA256, Bootstrap: bootstrap})
			name := filepath.Join(base, "upgrade.json")
			write(name, raw)
			args = append(args, "--broker-code-upgrade-policy", name, "--broker-code-upgrade-policy-sha256", pin(raw))
		}
		var out bytes.Buffer
		if e := run(args, &out); e != nil {
			t.Fatal(e)
		}
		var result map[string]any
		if e := json.Unmarshal(out.Bytes(), &result); e != nil || result["status"] != "published" {
			t.Fatalf("publication output: %v", e)
		}
	}
	publish("0.4.1-beta.1", strings.Repeat("a", 40), nil, true)
	original := bootstrap
	publish("0.4.1-beta.2", strings.Repeat("b", 40), &original, true)
	targetPath := "bootstrap/broker-code-upgrade-beta.json"
	check := func(version int) {
		raw, e := os.ReadFile(filepath.Join(repo, "public", strconv.Itoa(version)+".targets.json"))
		if e != nil {
			t.Fatal(e)
		}
		signed, e := metadata.Targets().FromBytes(raw)
		if e != nil {
			t.Fatal(e)
		}
		target := signed.Signed.Targets[targetPath]
		if target == nil || target.Custom == nil {
			t.Fatal("CLI did not sign the upgrade target")
		}
		identity, e := releasepolicy.ParseBrokerCodeUpgradeIdentity(*target.Custom)
		if e != nil || identity.Version != "0.4.1-beta.2" {
			t.Fatalf("signed upgrade identity: %v", e)
		}
		hash := hex.EncodeToString(target.Hashes["sha256"])
		got, e := os.ReadFile(filepath.Join(repo, "public", "targets", "bootstrap", hash+".broker-code-upgrade-beta.json"))
		if e != nil {
			t.Fatal(e)
		}
		approved, e := os.ReadFile(filepath.Join(base, "upgrade.json"))
		if e != nil || !bytes.Equal(got, approved) {
			t.Fatal("CLI approved bytes differ")
		}
	}
	check(2)
	publish("0.4.1-beta.3", strings.Repeat("c", 40), nil, false)
	check(3)
}
