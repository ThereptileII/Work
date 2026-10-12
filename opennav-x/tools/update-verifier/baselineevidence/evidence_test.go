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

func helperEvidenceFixture(t *testing.T) (evidenceFixture, map[string][]byte) {
	t.Helper()
	f := newEvidenceFixture(t)
	originals := map[string][]byte{
		"installer/engine/Lifecycle.ps1":         []byte("\xef\xbb\xbf# exact Lifecycle snapshot\r\n"),
		"installer/engine/UpdateSupervisor.ps1":  []byte("# exact Supervisor snapshot\r\n"),
		"installer/engine/UpdateTransaction.ps1": []byte("# exact Transaction snapshot\r\n"),
	}
	var inventory map[string]string
	if err := json.Unmarshal(f.inventory, &inventory); err != nil {
		t.Fatal(err)
	}
	for name, raw := range originals {
		inventory[name] = digest(raw)
		f.observed[name] = digest(raw)
	}
	f.inventory = fixtureJSON(t, inventory)
	f.expected.InventorySHA256 = digest(f.inventory)
	return f, originals
}

func TestEngineHelpersExactHistoricalAndCompleteContracts(t *testing.T) {
	historical := newEvidenceFixture(t)
	var inventory map[string]string
	if err := DecodeInventory(historical.inventory, &inventory); err != nil {
		t.Fatal(err)
	}
	helpers, err := EngineHelpers(inventory)
	if err != nil || len(helpers) != 0 {
		t.Fatal("historical inventory fabricated execution helpers", err)
	}
	f, _ := helperEvidenceFixture(t)
	if err := DecodeInventory(f.inventory, &inventory); err != nil {
		t.Fatal(err)
	}
	helpers, err = EngineHelpers(inventory)
	expected := []string{"installer/engine/Lifecycle.ps1", "installer/engine/UpdateSupervisor.ps1", "installer/engine/UpdateTransaction.ps1"}
	if err != nil || len(helpers) != len(expected) {
		t.Fatal("complete helper set refused", err)
	}
	for i, name := range expected {
		if helpers[i] != name || !IsEngineHelper(name) {
			t.Fatal("helper order or exact recognition differs")
		}
	}
	helpers[0] = "changed"
	again, err := EngineHelpers(inventory)
	if err != nil || again[0] != expected[0] {
		t.Fatal("returned helper names alias shared state")
	}
	for _, name := range []string{"installer/engine/lifecycle.ps1", "Installer/engine/Lifecycle.ps1", "installer/Engine/Lifecycle.ps1", "installer/engine/Other.ps1", "installer/engine/Lifecycle.ps1/child", "installer/engine", "app/Lifecycle.ps1"} {
		if IsEngineHelper(name) {
			t.Fatalf("nonexact helper recognized: %s", name)
		}
	}
}

func TestValidateRetainsExactOriginalHelperDigests(t *testing.T) {
	f, originals := helperEvidenceFixture(t)
	identity, payload, err := Validate(context.Background(), f.inventory, f.manifest, f.product, f.observed, f.expected)
	if err != nil || len(payload) != 7 || identity.InventorySHA256 != f.expected.InventorySHA256 {
		t.Fatal("full helper evidence refused", err)
	}
	for name, raw := range originals {
		if payload[name] != digest(raw) {
			t.Fatalf("exact original helper digest changed: %s", name)
		}
		normalized := bytes.ReplaceAll(bytes.TrimPrefix(raw, []byte{0xef, 0xbb, 0xbf}), []byte("\r\n"), []byte("\n"))
		if payload[name] == digest(normalized) {
			t.Fatal("normalized helper bytes substituted")
		}
	}
	// Helper snapshots are outside Lifecycle's app/docs installation manifest;
	// original manifest bytes and all existing identity fields retain their meaning.
	if identity.LifecycleManifestSHA256 != digest(f.manifest) || identity.ProductBuildSHA256 != digest(f.product) || identity.ExecutableSHA256 != f.expected.ExecutableSHA256 {
		t.Fatal("helper retention reinterpreted existing identities")
	}
	payload["installer/engine/Lifecycle.ps1"] = "changed"
	if f.observed["installer/engine/Lifecycle.ps1"] != digest(originals["installer/engine/Lifecycle.ps1"]) {
		t.Fatal("returned helper payload aliases observations")
	}
}

func TestEngineHelperInventoryRefusesPartialAliasAndUnknownEvenWhenRepinned(t *testing.T) {
	for _, scenario := range []string{"one-missing", "two-missing", "case-alias", "parent-case-alias", "engine-case-alias", "unknown-member", "nested-member", "engine-file", "invalid-digest"} {
		t.Run(scenario, func(t *testing.T) {
			f, _ := helperEvidenceFixture(t)
			var inventory map[string]string
			if err := json.Unmarshal(f.inventory, &inventory); err != nil {
				t.Fatal(err)
			}
			switch scenario {
			case "one-missing":
				delete(inventory, "installer/engine/Lifecycle.ps1")
			case "two-missing":
				delete(inventory, "installer/engine/Lifecycle.ps1")
				delete(inventory, "installer/engine/UpdateTransaction.ps1")
			case "case-alias":
				hash := inventory["installer/engine/Lifecycle.ps1"]
				delete(inventory, "installer/engine/Lifecycle.ps1")
				inventory["installer/engine/lifecycle.ps1"] = hash
			case "parent-case-alias":
				hash := inventory["installer/engine/Lifecycle.ps1"]
				delete(inventory, "installer/engine/Lifecycle.ps1")
				inventory["Installer/engine/Lifecycle.ps1"] = hash
			case "engine-case-alias":
				hash := inventory["installer/engine/Lifecycle.ps1"]
				delete(inventory, "installer/engine/Lifecycle.ps1")
				inventory["installer/Engine/Lifecycle.ps1"] = hash
			case "unknown-member":
				inventory["installer/engine/Unknown.ps1"] = digest([]byte("unknown"))
			case "nested-member":
				inventory["installer/engine/extra/file.ps1"] = digest([]byte("unknown"))
			case "engine-file":
				inventory["installer/engine"] = digest([]byte("file instead of directory"))
			case "invalid-digest":
				inventory["installer/engine/Lifecycle.ps1"] = strings.Repeat("A", 64)
			}
			f.inventory = fixtureJSON(t, inventory)
			f.expected.InventorySHA256 = digest(f.inventory)
			if helpers, err := EngineHelpers(inventory); err == nil || helpers != nil {
				t.Fatal("unsafe helper namespace accepted directly")
			}
			var decoded map[string]string
			if err := DecodeInventory(f.inventory, &decoded); err == nil {
				t.Fatal("unsafe helper namespace accepted by full inventory")
			}
			// Use a full observed inventory and updated pin to isolate schema consistency
			// from missing observations or independent original hash mismatches.
			policy := releasepolicy.Policy{Version: f.expected.Version, Commit: f.expected.Commit, SupportedOpenCpn: []releasepolicy.OpenCpn{brokerregistration.StockBaseline()}}
			if identity, err := CheckRecoveryEvidence(context.Background(), f.inventory, f.manifest, f.product, inventory, policy); err == nil || identity != (Identity{}) {
				t.Fatal("signed recovery schema admitted unsafe helper namespace")
			}
			if identity, payload, err := Validate(context.Background(), f.inventory, f.manifest, f.product, f.observed, f.expected); err == nil || identity != (Identity{}) || payload != nil {
				t.Fatal("approved helper schema accepted after repinning")
			}
		})
	}
}

func TestValidateRefusesMissingExtraOrTamperedObservedHelpers(t *testing.T) {
	for _, scenario := range []string{"missing-one", "missing-all", "tampered-original", "extra-helper", "observed-case-alias", "changed-inventory-original", "canceled"} {
		t.Run(scenario, func(t *testing.T) {
			f, originals := helperEvidenceFixture(t)
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			switch scenario {
			case "missing-one":
				delete(f.observed, "installer/engine/Lifecycle.ps1")
			case "missing-all":
				for name := range originals {
					delete(f.observed, name)
				}
			case "tampered-original":
				raw := bytes.Clone(originals["installer/engine/Lifecycle.ps1"])
				raw[len(raw)-1] ^= 1
				f.observed["installer/engine/Lifecycle.ps1"] = digest(raw)
			case "extra-helper":
				f.observed["installer/engine/Unknown.ps1"] = digest([]byte("extra"))
			case "observed-case-alias":
				hash := f.observed["installer/engine/Lifecycle.ps1"]
				delete(f.observed, "installer/engine/Lifecycle.ps1")
				f.observed["installer/engine/lifecycle.ps1"] = hash
			case "changed-inventory-original":
				f.inventory = append(f.inventory, '\n')
			case "canceled":
				cancel()
			}
			identity, payload, err := Validate(ctx, f.inventory, f.manifest, f.product, f.observed, f.expected)
			if err == nil || identity != (Identity{}) || payload != nil {
				t.Fatal("incomplete, changed or extra observed helper accepted")
			}
			if scenario == "canceled" && !errors.Is(err, context.Canceled) {
				t.Fatal("helper validation lost cancellation")
			}
		})
	}
}

func commissioningEvidenceFixture(t *testing.T) evidenceFixture {
	t.Helper()
	f, _ := helperEvidenceFixture(t) // Both reserved contracts coexist independently.
	var inventory map[string]string
	if err := json.Unmarshal(f.inventory, &inventory); err != nil {
		t.Fatal(err)
	}
	for _, name := range commissioningFixtureNames() {
		raw := []byte("\xef\xbb\xbf# inert original " + name + "\r\n ")
		inventory[name] = digest(raw)
		f.observed[name] = digest(raw)
	}
	f.inventory = fixtureJSON(t, inventory)
	f.expected.InventorySHA256 = digest(f.inventory)
	return f
}
func commissioningFixtureNames() []string {
	return []string{
		"installer/commissioning-v1/commission-read-only.ps1",
		"installer/commissioning-v1/verify-commissioning-launch.ps1",
		"installer/commissioning-v1/Commissioning.ps1",
		"installer/commissioning-v1/Preparation.ps1",
		"installer/commissioning-v1/Common.ps1",
		"installer/commissioning-v1/CommissioningProfileLock.cs",
		"installer/commissioning-v1/InstalledResourceReview.ps1",
		"installer/commissioning-v1/CommissioningBaseline.ps1",
		"installer/commissioning-v1/ColdBaseline.ps1",
		"installer/commissioning-v1/RestartCommissioningPolicy.ps1",
		"installer/commissioning-v1/RestartAuiPersistence.ps1",
		"installer/commissioning-v1/RestartDashboardPersistence.ps1",
	}
}

func TestCommissioningHelpersExactIndependentContracts(t *testing.T) {
	for _, present := range []bool{false, true} {
		f, _ := helperEvidenceFixture(t)
		if present {
			f = commissioningEvidenceFixture(t)
		}
		var inventory map[string]string
		if err := DecodeInventory(f.inventory, &inventory); err != nil {
			t.Fatal(err)
		}
		helpers, err := CommissioningHelpers(inventory)
		if err != nil || (len(helpers) == 12) != present || (!present && helpers != nil) {
			t.Fatal("commissioning presence differs", err)
		}
		engine, err := EngineHelpers(inventory)
		if err != nil || len(engine) != 3 {
			t.Fatal("engine contract changed", err)
		}
		if present {
			for i, expected := range commissioningFixtureNames() {
				if helpers[i] != expected || !IsCommissioningHelper(expected) {
					t.Fatal("exact stable commissioning closure differs")
				}
			}
			helpers[0] = "mutated"
			again, err := CommissioningHelpers(inventory)
			if err != nil || again[0] != commissioningFixtureNames()[0] {
				t.Fatal("returned names alias shared state", err)
			}
		}
	}
}

func TestCommissioningInventoryRefusesRepinnedPartialAliasUnknownAndDigest(t *testing.T) {
	for _, scenario := range []string{"one-missing", "only-one", "alias", "parent-alias", "directory-alias", "unknown", "nested", "directory-file", "digest"} {
		t.Run(scenario, func(t *testing.T) {
			f := commissioningEvidenceFixture(t)
			var inventory map[string]string
			if err := json.Unmarshal(f.inventory, &inventory); err != nil {
				t.Fatal(err)
			}
			first := commissioningFixtureNames()[0]
			switch scenario {
			case "one-missing":
				delete(inventory, first)
			case "only-one":
				for _, name := range commissioningFixtureNames()[1:] {
					delete(inventory, name)
				}
			case "alias", "parent-alias", "directory-alias":
				alias := "installer/commissioning-v1/Commission-read-only.ps1"
				if scenario == "parent-alias" {
					alias = "Installer/commissioning-v1/commission-read-only.ps1"
				}
				if scenario == "directory-alias" {
					alias = "installer/Commissioning-v1/commission-read-only.ps1"
				}
				inventory[alias] = inventory[first]
				delete(inventory, first)
			case "unknown":
				inventory["installer/commissioning-v1/Other.ps1"] = digest([]byte("extra"))
			case "nested":
				inventory[first+"/child"] = digest([]byte("nested"))
			case "directory-file":
				inventory["installer/commissioning-v1"] = digest([]byte("file"))
			case "digest":
				inventory[first] = strings.Repeat("A", 64)
			}
			if helpers, err := CommissioningHelpers(inventory); err == nil || helpers != nil {
				t.Fatal("invalid commissioning namespace recognized")
			}
			f.inventory = fixtureJSON(t, inventory)
			f.expected.InventorySHA256 = digest(f.inventory)
			var decoded map[string]string
			if DecodeInventory(f.inventory, &decoded) == nil {
				t.Fatal("invalid repinned inventory accepted")
			}
			if _, _, err := Validate(context.Background(), f.inventory, f.manifest, f.product, f.observed, f.expected); err == nil {
				t.Fatal("repinning bypassed commissioning contract")
			}
		})
	}
}

func TestValidateCommissioningRequiresExactObservedOriginals(t *testing.T) {
	for _, scenario := range []string{"complete", "missing", "changed", "extra", "alias"} {
		t.Run(scenario, func(t *testing.T) {
			f := commissioningEvidenceFixture(t)
			first := commissioningFixtureNames()[0]
			switch scenario {
			case "missing":
				delete(f.observed, first)
			case "changed":
				f.observed[first] = digest([]byte("normalized or changed"))
			case "extra":
				f.observed["installer/commissioning-v1/Other.ps1"] = digest([]byte("extra"))
			case "alias":
				f.observed[strings.ToUpper(first)] = f.observed[first]
				delete(f.observed, first)
			}
			identity, payload, err := Validate(context.Background(), f.inventory, f.manifest, f.product, f.observed, f.expected)
			if scenario != "complete" {
				if err == nil {
					t.Fatal("nonexact observed originals accepted")
				}
				return
			}
			if err != nil || len(payload) != len(f.observed) || identity.LifecycleManifestSHA256 != digest(f.manifest) {
				t.Fatal("exact closure or original identity lost", err)
			}
			for name, hash := range f.observed {
				if payload[name] != hash {
					t.Fatal("original digest lost")
				}
			}
		})
	}
}
