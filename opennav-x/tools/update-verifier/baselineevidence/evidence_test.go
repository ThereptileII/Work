package baselineevidence

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"strings"
	"testing"

	"example.com/opennav-update-verifier/brokerregistration"
	"example.com/opennav-update-verifier/releasepolicy"
)

type evidenceFixture struct {
	inventory, manifest, product []byte
	observed                     map[string]string
	expected                     brokerregistration.Installed
}

func fixtureJSON(t *testing.T, v interface{}) []byte {
	t.Helper()
	raw, err := json.Marshal(v)
	if err != nil {
		t.Fatal(err)
	}
	return raw
}
func newEvidenceFixture(t *testing.T) evidenceFixture {
	t.Helper()
	observed := map[string]string{"app/opencpn.exe": digest([]byte("inert app")), "app/opennav-restart.exe": digest([]byte("inert restart")), "app/skager-start.exe": digest([]byte("inert launcher"))}
	no := false
	product := fixtureJSON(t, recoveryProduct{Version: "2.0.0", Commit: strings.Repeat("a", 40), TestFixtures: &no, BuildPurpose: "INSTALLED PRODUCT", OutputPolicy: "manual-commissioning", ManualControlContract: 1, ExecutableSHA256: observed["app/opencpn.exe"], RestartHelperSHA256: observed["app/opennav-restart.exe"], StartupLauncherSHA256: observed["app/skager-start.exe"], CommissioningRestartProtocol: 1, UpdateStartupHealth: 1})
	observed["docs/PRODUCT_BUILD.json"] = digest(product)
	var files []map[string]string
	for name, hash := range observed {
		files = append(files, map[string]string{"path": name, "sha256": hash})
	}
	manifest := fixtureJSON(t, map[string]interface{}{"schema": 1, "version": "2.0.0", "commit": strings.Repeat("a", 40), "payloadSha256": strings.Repeat("b", 64), "supportedOpenCpn": []releasepolicy.OpenCpn{brokerregistration.StockBaseline()}, "files": files, "qualification": "candidate"})
	inventory := map[string]string{}
	for name, hash := range observed {
		inventory[name] = hash
	}
	inventory["installer/package.json"] = digest(manifest)
	inventory["profile/opencpn.conf"] = digest([]byte("excluded recovery profile"))
	inventory["app/OPENNAV_PORTABLE_PREVIEW"] = digest([]byte("excluded portable marker"))
	raw := fixtureJSON(t, inventory)
	return evidenceFixture{raw, manifest, product, observed, brokerregistration.Installed{Version: "2.0.0", Commit: strings.Repeat("a", 40), Generation: strings.Repeat("c", 32), PackageSHA256: digest(manifest), InventorySHA256: digest(raw), ProductBuildSHA256: digest(product), ExecutableSHA256: observed["app/opencpn.exe"]}}
}
func TestValidateExactApprovedEvidence(t *testing.T) {
	f := newEvidenceFixture(t)
	identity, payload, err := Validate(context.Background(), f.inventory, f.manifest, f.product, f.observed, f.expected)
	if err != nil || !identity.LifecycleManifestAvailable || identity.LifecycleManifestSHA256 != f.expected.PackageSHA256 || len(payload) != 4 {
		t.Fatalf("valid exact evidence refused: %+v %v", identity, err)
	}
	payload["app/opencpn.exe"] = "changed"
	if f.observed["app/opencpn.exe"] != f.expected.ExecutableSHA256 {
		t.Fatal("returned map aliases input")
	}
}
func TestValidateRefusesSchemaEvenWhenOriginalPinsMatch(t *testing.T) {
	for _, mode := range []string{"product duplicate", "product case alias", "product null", "product unknown", "invalid health protocol", "manifest duplicate", "manifest null", "manifest unknown", "manifest wrong stock", "inventory self alias", "inventory parent alias", "inventory null", "inventory trailing", "inventory NaN", "inventory oversized"} {
		t.Run(mode, func(t *testing.T) {
			f := newEvidenceFixture(t)
			switch mode {
			case "product duplicate":
				f.product = bytes.Replace(f.product, []byte(`"version":"2.0.0"`), []byte(`"version":"2.0.0","version":"2.0.0"`), 1)
			case "product case alias":
				f.product = bytes.Replace(f.product, []byte(`"version"`), []byte(`"Version"`), 1)
			case "product null":
				f.product = bytes.Replace(f.product, []byte(`"test_fixtures":false`), []byte(`"test_fixtures":null`), 1)
			case "product unknown":
				f.product = append([]byte(`{"unknown":true,`), f.product[1:]...)
			case "invalid health protocol":
				f.product = bytes.Replace(f.product, []byte(`"update_startup_health":1`), []byte(`"update_startup_health":2`), 1)
			case "manifest duplicate":
				f.manifest = bytes.Replace(f.manifest, []byte(`"schema":1`), []byte(`"schema":1,"schema":1`), 1)
			case "manifest null":
				f.manifest = bytes.Replace(f.manifest, []byte(`"schema":1`), []byte(`"schema":null`), 1)
			case "manifest unknown":
				f.manifest = append([]byte(`{"unknown":true,`), f.manifest[1:]...)
			case "manifest wrong stock":
				f.manifest = bytes.Replace(f.manifest, []byte(`"5.12.4"`), []byte(`"5.12.5"`), 1)
			case "inventory self alias":
				var m map[string]string
				json.Unmarshal(f.inventory, &m)
				m["file_sha256.JSON"] = strings.Repeat("0", 64)
				f.inventory = fixtureJSON(t, m)
			case "inventory parent alias":
				var m map[string]string
				json.Unmarshal(f.inventory, &m)
				m["app"] = strings.Repeat("0", 64)
				f.inventory = fixtureJSON(t, m)
			case "inventory null":
				f.inventory = []byte(`null`)
			case "inventory trailing":
				f.inventory = append(f.inventory, []byte(`{}`)...)
			case "inventory NaN":
				f.inventory = []byte(`{"app/opencpn.exe":NaN}`)
			case "inventory oversized":
				f.inventory = bytes.Repeat([]byte(" "), maxEvidenceBytes+1)
			}
			// Repin exact changed originals and inventory entries so semantic refusal is
			// tested independently of hash mismatch refusal.
			if strings.HasPrefix(mode, "product") || mode == "invalid health protocol" {
				var m map[string]string
				json.Unmarshal(f.inventory, &m)
				m["docs/PRODUCT_BUILD.json"] = digest(f.product)
				f.observed["docs/PRODUCT_BUILD.json"] = digest(f.product)
				f.inventory = fixtureJSON(t, m)
			}
			if strings.HasPrefix(mode, "manifest") {
				var m map[string]string
				json.Unmarshal(f.inventory, &m)
				m["installer/package.json"] = digest(f.manifest)
				f.inventory = fixtureJSON(t, m)
			}
			f.expected.ProductBuildSHA256 = digest(f.product)
			f.expected.PackageSHA256 = digest(f.manifest)
			f.expected.InventorySHA256 = digest(f.inventory)
			identity, payload, err := Validate(context.Background(), f.inventory, f.manifest, f.product, f.observed, f.expected)
			if err == nil || identity != (Identity{}) || payload != nil {
				t.Fatal("invalid original schema accepted after repinning")
			}
		})
	}
}
func TestValidateContextAndBounds(t *testing.T) {
	f := newEvidenceFixture(t)
	if err := DecodeInventory(f.inventory, nil); err == nil {
		t.Fatal("nil inventory destination accepted")
	}
	ctx, cancel := context.WithCancel(context.Background())
	cancel()
	if _, _, err := Validate(ctx, f.inventory, f.manifest, f.product, f.observed, f.expected); !errors.Is(err, context.Canceled) {
		t.Fatalf("cancellation lost: %v", err)
	}
	if _, _, err := Validate(nil, f.inventory, f.manifest, f.product, f.observed, f.expected); err == nil {
		t.Fatal("nil context accepted")
	}
}

func TestValidateRejectsPortableMarkerCaseAliasInManifest(t *testing.T) {
	f := newEvidenceFixture(t)
	var inventory map[string]string
	json.Unmarshal(f.inventory, &inventory)
	alias := "app/opennav_portable_preview"
	hash := inventory["app/OPENNAV_PORTABLE_PREVIEW"]
	delete(inventory, "app/OPENNAV_PORTABLE_PREVIEW")
	inventory[alias] = hash
	var manifest map[string]json.RawMessage
	json.Unmarshal(f.manifest, &manifest)
	var files []map[string]string
	json.Unmarshal(manifest["files"], &files)
	files = append(files, map[string]string{"path": alias, "sha256": hash})
	manifest["files"] = fixtureJSON(t, files)
	f.manifest = fixtureJSON(t, manifest)
	inventory["installer/package.json"] = digest(f.manifest)
	f.inventory = fixtureJSON(t, inventory)
	f.observed[alias] = hash
	f.expected.InventorySHA256 = digest(f.inventory)
	f.expected.PackageSHA256 = digest(f.manifest)
	if _, _, err := Validate(context.Background(), f.inventory, f.manifest, f.product, f.observed, f.expected); err == nil {
		t.Fatal("portable marker case alias became installed payload")
	}
}
