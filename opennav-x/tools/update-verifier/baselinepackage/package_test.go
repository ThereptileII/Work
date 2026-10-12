package baselinepackage

import (
	"archive/zip"
	"bytes"
	"context"
	"crypto"
	"crypto/ed25519"
	"crypto/sha256"
	"encoding/json"
	"errors"
	"os"
	"path/filepath"
	"sort"
	"strings"
	"testing"
	"time"

	"example.com/opennav-update-verifier/brokerregistration"
	"example.com/opennav-update-verifier/releasepolicy"
	"example.com/opennav-update-verifier/repository"
	"github.com/sigstore/sigstore/pkg/signature"
	"github.com/theupdateframework/go-tuf/v2/metadata"
)

func jsonBytes(t *testing.T, value any) []byte {
	t.Helper()
	b, err := json.Marshal(value)
	if err != nil {
		t.Fatal(err)
	}
	return append(b, '\n')
}

func testTrust(t *testing.T) []byte {
	t.Helper()
	root := metadata.Root(time.Now().Add(24 * time.Hour))
	root.Signed.ConsistentSnapshot = true
	var rootKey ed25519.PrivateKey
	for _, role := range []string{"root", "timestamp", "snapshot", "targets"} {
		seed := sha256.Sum256([]byte("INERT BASELINE PACKAGE TEST ONLY " + role))
		key := ed25519.NewKeyFromSeed(seed[:])
		public, err := metadata.KeyFromPublicKey(key.Public())
		if err != nil {
			t.Fatal(err)
		}
		if err := root.Signed.AddKey(public, role); err != nil {
			t.Fatal(err)
		}
		if role == "root" {
			rootKey = key
		}
	}
	signer, err := signature.LoadSigner(rootKey, crypto.Hash(0))
	if err != nil {
		t.Fatal(err)
	}
	if _, err := root.Sign(signer); err != nil {
		t.Fatal(err)
	}
	rootBytes, err := root.ToBytes(false)
	if err != nil {
		t.Fatal(err)
	}
	trust, err := json.Marshal(repository.Trust{Schema: 1, BootstrapRoot: rootBytes,
		MetadataURL: "https://updates.invalid/metadata/", ArtifactOrigin: "https://updates.invalid",
		Channel: "beta", LocalCompatibilityAllowlist: []releasepolicy.OpenCpn{brokerregistration.StockBaseline()}})
	if err != nil {
		t.Fatal(err)
	}
	return trust
}

type fixture struct {
	in      Input
	entries map[string][]byte
	archive []byte
}

func newFixture(t *testing.T) fixture {
	t.Helper()
	root := t.TempDir()
	tree, output := filepath.Join(root, "source-tree"), filepath.Join(root, "output")
	if err := os.Mkdir(tree, 0700); err != nil {
		t.Fatal(err)
	}
	if err := os.Mkdir(output, 0700); err != nil {
		t.Fatal(err)
	}
	entries := map[string][]byte{
		"app/opencpn.exe":              []byte("inert baseline executable"),
		"app/opennav-restart.exe":      []byte("inert restart helper"),
		"app/skager-start.exe":         []byte("inert startup launcher"),
		"app/OPENNAV_PORTABLE_PREVIEW": []byte("portable marker excluded from installed view"),
		"profile/opencpn.conf":         []byte("inert recovery profile excluded from installed view"),
	}
	commit := strings.Repeat("a", 40)
	version := "2.0.0"
	entries["docs/PRODUCT_BUILD.json"] = jsonBytes(t, map[string]any{
		"version": version, "commit": commit, "test_fixtures": false, "build_purpose": "INSTALLED PRODUCT",
		"xnav_hardware_output_policy": "status-only", "xnav_manual_control_contract": 0,
		"executable_sha256":              digest(entries["app/opencpn.exe"]),
		"restart_helper_sha256":          digest(entries["app/opennav-restart.exe"]),
		"startup_launcher_sha256":        digest(entries["app/skager-start.exe"]),
		"commissioning_restart_protocol": 1, "update_startup_health": 1,
	})
	var records []map[string]string
	for name, data := range entries {
		if (strings.HasPrefix(name, "app/") || strings.HasPrefix(name, "docs/")) && name != "app/OPENNAV_PORTABLE_PREVIEW" {
			records = append(records, map[string]string{"path": name, "sha256": digest(data)})
		}
	}
	entries["installer/package.json"] = jsonBytes(t, map[string]any{
		"schema": 1, "version": version, "commit": commit, "payloadSha256": strings.Repeat("b", 64),
		"supportedOpenCpn": []releasepolicy.OpenCpn{brokerregistration.StockBaseline()}, "files": records, "qualification": "candidate",
	})
	inventory := map[string]string{}
	for name, data := range entries {
		inventory[name] = digest(data)
	}
	entries["FILE_SHA256.json"] = jsonBytes(t, inventory)
	names := make([]string, 0, len(entries))
	for name := range entries {
		names = append(names, name)
	}
	sort.Strings(names)
	var b bytes.Buffer
	z := zip.NewWriter(&b)
	for _, name := range names {
		p := filepath.Join(tree, filepath.FromSlash(name))
		if err := os.MkdirAll(filepath.Dir(p), 0700); err != nil {
			t.Fatal(err)
		}
		if err := os.WriteFile(p, entries[name], 0600); err != nil {
			t.Fatal(err)
		}
		w, err := z.Create(recoveryPrefix + name)
		if err != nil {
			t.Fatal(err)
		}
		if _, err := w.Write(entries[name]); err != nil {
			t.Fatal(err)
		}
	}
	if err := z.Close(); err != nil {
		t.Fatal(err)
	}
	archive := filepath.Join(root, "original-recovery.zip")
	if err := os.WriteFile(archive, b.Bytes(), 0600); err != nil {
		t.Fatal(err)
	}
	var prettyTrust bytes.Buffer
	if err := json.Indent(&prettyTrust, testTrust(t), "", "  "); err != nil {
		t.Fatal(err)
	}
	trust := prettyTrust.Bytes()
	if _, err := repository.ValidateTrust(trust, time.Now().UTC()); err != nil {
		t.Fatal("fixture trust:", err)
	}
	trustPath := filepath.Join(root, "approved-trust.json")
	if err := os.WriteFile(trustPath, trust, 0600); err != nil {
		t.Fatal(err)
	}
	in := Input{SourceArchive: archive, BaselineTree: tree, TrustConfig: trustPath,
		StockPath: `C:\Program Files (x86)\OpenCPN\opencpn.exe`, OutputDirectory: output,
		ClientSID: "S-1-5-21-123-456-789-500", Generation: strings.Repeat("c", 32), Version: version, Commit: commit,
		ApprovedSourceSHA256: digest(b.Bytes()), ApprovedPackageSHA256: digest(entries["installer/package.json"]),
		ApprovedBuildSHA256: digest(entries["docs/PRODUCT_BUILD.json"]), ApprovedExecutableSHA256: digest(entries["app/opencpn.exe"]),
		ApprovedInventorySHA256: digest(entries["FILE_SHA256.json"]), ApprovedTrustSHA256: digest(trust),
		ApprovedStockSHA256: brokerregistration.StockBaseline().ExecutableSHA256}
	return fixture{in, entries, b.Bytes()}
}

func TestProducesExactApprovedArchiveAndStrictRequest(t *testing.T) {
	f := newFixture(t)
	result, err := produce(context.Background(), f.in, func(string) error { return nil })
	if err != nil {
		t.Fatal(err)
	}
	archive, err := os.ReadFile(result.ArchivePath)
	if err != nil || !bytes.Equal(archive, f.archive) {
		t.Fatal("final archive changed original bytes", err)
	}
	raw, err := os.ReadFile(result.RequestPath)
	if err != nil || digest(raw) != result.RequestSHA256 {
		t.Fatal("request output hash differs", err)
	}
	request, err := brokerregistration.ParseApprovedRequest(raw, result.RequestSHA256, time.Now().UTC())
	if err != nil || request.Installed.PackageSHA256 != f.in.ApprovedPackageSHA256 || request.Stock.ExecutableSHA256 != f.in.ApprovedStockSHA256 {
		t.Fatal("strict request identity differs", err)
	}
	if _, err := produce(context.Background(), f.in, func(string) error { return nil }); err == nil {
		t.Fatal("existing outputs accepted")
	}
}

func TestRefusesChangedMissingAndLinkedTree(t *testing.T) {
	for _, mode := range []string{"changed", "missing", "linked"} {
		t.Run(mode, func(t *testing.T) {
			f := newFixture(t)
			path := filepath.Join(f.in.BaselineTree, "app", "opencpn.exe")
			if err := os.Remove(path); err != nil {
				t.Fatal(err)
			}
			switch mode {
			case "changed":
				if err := os.WriteFile(path, []byte("different"), 0600); err != nil {
					t.Fatal(err)
				}
			case "linked":
				if err := os.Symlink(filepath.Join(f.in.BaselineTree, "app", "skager-start.exe"), path); err != nil {
					t.Fatal(err)
				}
			}
			if _, err := produce(context.Background(), f.in, func(string) error { return nil }); err == nil {
				t.Fatal("invalid tree accepted")
			}
		})
	}
}

func TestRefusesIndependentPinsAndObservedStockMismatch(t *testing.T) {
	for _, mode := range []string{"source", "package", "build", "executable", "inventory", "trust", "stock"} {
		t.Run(mode, func(t *testing.T) {
			f := newFixture(t)
			stockCheck := func(string) error { return nil }
			switch mode {
			case "source":
				f.in.ApprovedSourceSHA256 = strings.Repeat("0", 64)
			case "package":
				f.in.ApprovedPackageSHA256 = strings.Repeat("0", 64)
			case "build":
				f.in.ApprovedBuildSHA256 = strings.Repeat("0", 64)
			case "executable":
				f.in.ApprovedExecutableSHA256 = strings.Repeat("0", 64)
			case "inventory":
				f.in.ApprovedInventorySHA256 = strings.Repeat("0", 64)
			case "trust":
				f.in.ApprovedTrustSHA256 = strings.Repeat("0", 64)
			case "stock":
				stockCheck = func(string) error { return errors.New("measured stock mismatch") }
			}
			if _, err := produce(context.Background(), f.in, stockCheck); err == nil {
				t.Fatal("unapproved identity accepted")
			}
		})
	}
}

func TestRefusesOversizedSourceBeforeRead(t *testing.T) {
	f := newFixture(t)
	file, err := os.OpenFile(f.in.SourceArchive, os.O_WRONLY, 0600)
	if err != nil {
		t.Fatal(err)
	}
	if err := file.Truncate(maxArchiveBytes + 1); err != nil {
		t.Fatal(err)
	}
	if err := file.Close(); err != nil {
		t.Fatal(err)
	}
	if _, err := produce(context.Background(), f.in, func(string) error { return nil }); err == nil {
		t.Fatal("oversized archive accepted")
	}
}

func TestObservedStockRejectsWrongBytesAndLinkedPath(t *testing.T) {
	root := t.TempDir()
	wrong := filepath.Join(root, "opencpn.exe")
	if err := os.WriteFile(wrong, []byte("not the fixed qualified stock executable"), 0600); err != nil {
		t.Fatal(err)
	}
	if err := observeStock(wrong); err == nil {
		t.Fatal("wrong measured stock executable accepted")
	}
	linked := filepath.Join(root, "linked-opencpn.exe")
	if err := os.Symlink(wrong, linked); err != nil {
		t.Fatal(err)
	}
	if err := observeStock(linked); err == nil {
		t.Fatal("linked stock path accepted")
	}
}

type endlessReader struct{ total int64 }

func (r *endlessReader) Read(p []byte) (int, error) {
	for i := range p {
		p[i] = 'x'
	}
	r.total += int64(len(p))
	return len(p), nil
}

func TestBoundedDigestStopsGrowingInputAndCancellation(t *testing.T) {
	r := &endlessReader{}
	if _, n, err := boundedDigest(context.Background(), r, 5); err == nil || n != 6 || r.total != 6 {
		t.Fatalf("growing reader consumed beyond declared size: count=%d consumed=%d error=%v", n, r.total, err)
	}
	ctx, cancel := context.WithCancel(context.Background())
	cancel()
	r = &endlessReader{}
	if _, _, err := boundedDigest(ctx, r, 5); !errors.Is(err, context.Canceled) || r.total != 0 {
		t.Fatalf("cancelled read continued: consumed=%d error=%v", r.total, err)
	}
}

func TestRefusesOversizedTrustFile(t *testing.T) {
	f := newFixture(t)
	file, err := os.OpenFile(f.in.TrustConfig, os.O_WRONLY, 0600)
	if err != nil {
		t.Fatal(err)
	}
	if err := file.Truncate(int64(brokerregistration.MaxTrustConfigurationBytes) + 1); err != nil {
		t.Fatal(err)
	}
	if err := file.Close(); err != nil {
		t.Fatal(err)
	}
	if _, err := produce(context.Background(), f.in, func(string) error { return nil }); err == nil {
		t.Fatal("oversized trust file accepted")
	}
}

func TestRefusesSourceChangedAfterOutput(t *testing.T) {
	f := newFixture(t)
	checks := 0
	_, err := produce(context.Background(), f.in, func(string) error {
		checks++
		if checks == 2 {
			file, openErr := os.OpenFile(f.in.SourceArchive, os.O_WRONLY, 0600)
			if openErr != nil {
				return openErr
			}
			_, writeErr := file.WriteAt([]byte("bad!"), 30)
			return errors.Join(writeErr, file.Close())
		}
		return nil
	})
	if err == nil || checks != 2 {
		t.Fatalf("changed retained source accepted: checks=%d error=%v", checks, err)
	}
	if _, err := os.Stat(filepath.Join(f.in.OutputDirectory, RequestName)); err != nil {
		t.Fatal("failed handoff should retain request for review", err)
	}
}
