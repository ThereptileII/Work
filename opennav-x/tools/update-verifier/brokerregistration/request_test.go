package brokerregistration

import (
	"bytes"
	"crypto"
	"crypto/ed25519"
	"crypto/sha256"
	"encoding/json"
	"strings"
	"testing"
	"time"

	"example.com/opennav-update-verifier/releasepolicy"
	"example.com/opennav-update-verifier/repository"
	"github.com/sigstore/sigstore/pkg/signature"
	"github.com/theupdateframework/go-tuf/v2/metadata"
)

var fixtureNow = time.Date(2026, 10, 12, 12, 0, 0, 0, time.UTC)

// Deterministic, test-only keys make the fixture self-contained. These keys
// never authenticate production/operator input and no network or files are used.
func approvedFixture(t *testing.T) []byte {
	t.Helper()
	root := metadata.Root(fixtureNow.Add(24 * time.Hour))
	root.Signed.ConsistentSnapshot = true
	var rootPrivate ed25519.PrivateKey
	for _, role := range []string{"root", "timestamp", "snapshot", "targets"} {
		seed := sha256.Sum256([]byte("INERT BROKER REGISTRATION TEST ONLY " + role))
		private := ed25519.NewKeyFromSeed(seed[:])
		key, err := metadata.KeyFromPublicKey(private.Public())
		if err != nil {
			t.Fatal(err)
		}
		if err = root.Signed.AddKey(key, role); err != nil {
			t.Fatal(err)
		}
		if role == "root" {
			rootPrivate = private
		}
	}
	signer, err := signature.LoadSigner(rootPrivate, crypto.Hash(0))
	if err != nil {
		t.Fatal(err)
	}
	if _, err = root.Sign(signer); err != nil {
		t.Fatal(err)
	}
	rootBytes, err := root.ToBytes(false)
	if err != nil {
		t.Fatal(err)
	}
	trust, err := json.Marshal(repository.Trust{Schema: 1, BootstrapRoot: rootBytes, MetadataURL: "https://updates.invalid/metadata/", ArtifactOrigin: "https://updates.invalid", Channel: "beta", LocalCompatibilityAllowlist: []releasepolicy.OpenCpn{StockBaseline()}})
	if err != nil {
		t.Fatal(err)
	}
	baseline := StockBaseline()
	r := Request{Schema: 1, Purpose: Purpose, ClientSID: "S-1-5-21-123-456-789-1001", Installed: Installed{Version: "0.4.1-beta.1", Commit: strings.Repeat("a", 40), Generation: strings.Repeat("b", 32), PackageSHA256: strings.Repeat("c", 64), ProductBuildSHA256: strings.Repeat("d", 64), ExecutableSHA256: strings.Repeat("e", 64), InventorySHA256: strings.Repeat("f", 64)}, Stock: Stock{Path: `C:\Program Files (x86)\OpenCPN\opencpn.exe`, Version: baseline.Version, Arch: baseline.Arch, ExecutableSHA256: baseline.ExecutableSHA256, UpstreamCommit: baseline.UpstreamCommit}, TrustConfiguration: trust, TrustConfigSHA256: digest(trust)}
	raw, err := json.Marshal(r)
	if err != nil {
		t.Fatal(err)
	}
	return raw
}

func TestApprovedRequestOwnsExactOriginalAndTrustBytes(t *testing.T) {
	raw := approvedFixture(t)
	original := bytes.Clone(raw)
	pin := digest(raw)
	r, err := ParseApprovedRequest(raw, pin, fixtureNow)
	if err != nil {
		t.Fatal(err)
	}
	if r.RequestSHA256 != pin || digest(r.TrustConfiguration) != r.TrustConfigSHA256 || r.Stock.Identity() != StockBaseline() {
		t.Fatal("approved evidence identity lost")
	}
	raw[0] = '!'
	if !bytes.Equal(r.OriginalBytes(), original) {
		t.Fatal("caller changed retained request")
	}
	copy := r.OriginalBytes()
	copy[0] = '!'
	if !bytes.Equal(r.OriginalBytes(), original) {
		t.Fatal("returned bytes changed retained request")
	}
	trust := r.Trust()
	root := bytes.Clone(trust.BootstrapRoot)
	trust.BootstrapRoot[0] = '!'
	trust.LocalCompatibilityAllowlist[0].Arch = "x64"
	r.TrustConfiguration[0] = '!'
	if !bytes.Equal(r.Trust().BootstrapRoot, root) || r.Trust().LocalCompatibilityAllowlist[0] != StockBaseline() {
		t.Fatal("returned configuration changed retained trust")
	}
}

func TestApprovedRequestRequiresSeparateExactApproval(t *testing.T) {
	raw := approvedFixture(t)
	for _, pin := range []string{"", strings.Repeat("z", 64), strings.ToUpper(digest(raw)), strings.Repeat("0", 64)} {
		if _, err := ParseApprovedRequest(raw, pin, fixtureNow); err == nil {
			t.Fatal("missing or mismatched approval accepted")
		}
	}
	if _, err := ParseApprovedRequest(append(bytes.Clone(raw), ' '), digest(raw), fixtureNow); err == nil {
		t.Fatal("changed original bytes accepted")
	}
	if _, err := ParseApprovedRequest(raw, digest(raw), time.Time{}); err == nil {
		t.Fatal("missing clock accepted")
	}
	if _, err := ParseApprovedRequest(raw, digest(raw), fixtureNow.Add(25*time.Hour)); err == nil {
		t.Fatal("expired public root accepted")
	}
}

func TestApprovedRequestRejectsAmbiguousJSONAndUnknownFields(t *testing.T) {
	valid := approvedFixture(t)
	cases := map[string][]byte{
		"empty": nil, "oversize": bytes.Repeat([]byte(" "), MaxRequestBytes+1), "utf8": {0xff},
		"duplicate":        bytes.Replace(valid, []byte(`"schema":1`), []byte(`"schema":1,"schema":1`), 1),
		"case alias":       bytes.Replace(valid, []byte(`"clientSid"`), []byte(`"ClientSid"`), 1),
		"nested duplicate": bytes.Replace(valid, []byte(`"generation":`), []byte(`"Generation":"ignored","generation":`), 1),
		"unknown":          bytes.Replace(valid, []byte(`"schema":1`), []byte(`"extra":1,"schema":1`), 1),
		"nested unknown":   bytes.Replace(valid, []byte(`"inventorySha256":`), []byte(`"extra":false,"inventorySha256":`), 1),
		"null":             bytes.Replace(valid, []byte(`"schema":1`), []byte(`"schema":null`), 1),
		"nested null":      bytes.Replace(valid, []byte(`"version":"0.4.1-beta.1"`), []byte(`"version":null`), 1),
		"NaN":              bytes.Replace(valid, []byte(`"schema":1`), []byte(`"schema":NaN`), 1),
		"trailing":         append(bytes.Clone(valid), []byte(`{}`)...),
		"missing":          bytes.Replace(valid, []byte(`"purpose":"client-registration",`), nil, 1),
	}
	for name, raw := range cases {
		t.Run(name, func(t *testing.T) {
			if _, err := ParseApprovedRequest(raw, digest(raw), fixtureNow); err == nil {
				t.Fatal("ambiguous or invalid request accepted")
			}
		})
	}
}

func TestApprovedRequestRejectsUnapprovedIdentityAndPaths(t *testing.T) {
	valid := approvedFixture(t)
	cases := map[string]func(*Request){
		"purpose":       func(r *Request) { r.Purpose = "machine-update-broker-enrollment" },
		"schema":        func(r *Request) { r.Schema = 2 },
		"version":       func(r *Request) { r.Installed.Version = "invalid" },
		"commit":        func(r *Request) { r.Installed.Commit = strings.Repeat("A", 40) },
		"generation":    func(r *Request) { r.Installed.Generation = "../other" },
		"package":       func(r *Request) { r.Installed.PackageSHA256 = "" },
		"product":       func(r *Request) { r.Installed.ProductBuildSHA256 = "bad" },
		"executable":    func(r *Request) { r.Installed.ExecutableSHA256 = strings.Repeat("A", 64) },
		"inventory":     func(r *Request) { r.Installed.InventorySHA256 = "bad" },
		"stock version": func(r *Request) { r.Stock.Version = "5.12.5" },
		"stock arch":    func(r *Request) { r.Stock.Arch = "x64" },
		"stock hash":    func(r *Request) { r.Stock.ExecutableSHA256 = strings.Repeat("0", 64) },
		"stock commit":  func(r *Request) { r.Stock.UpstreamCommit = strings.Repeat("0", 40) },
		"trust pin":     func(r *Request) { r.TrustConfigSHA256 = strings.Repeat("0", 64) },
	}
	for name, change := range cases {
		t.Run(name, func(t *testing.T) {
			var r Request
			if err := json.Unmarshal(valid, &r); err != nil {
				t.Fatal(err)
			}
			change(&r)
			raw, err := json.Marshal(r)
			if err != nil {
				t.Fatal(err)
			}
			if _, err = ParseApprovedRequest(raw, digest(raw), fixtureNow); err == nil {
				t.Fatal("invalid identity accepted")
			}
		})
	}
	for _, sid := range []string{"S-1-5-18", "S-1-5-32-544", "S-1-5-80-1-2-3-4-5", "s-1-5-21-123-456-789-1001", "S-1-5-21-0123-456-789-1001", "S-1-5-21-123-456-789-0500", "S-1-5-21-123-456-789-4294967296", "S-1-5-21-4294967296-456-789-1001", "S-1-5-21-123-456-789--1001"} {
		if canonicalUserSID(sid) {
			t.Errorf("invalid/non-client SID accepted: %s", sid)
		}
	}
	for _, path := range []string{`\\server\share\opencpn.exe`, `C:\OpenCPN\..\opencpn.exe`, `C:\OpenCPN\opencpn.exe:stream`, `C:/OpenCPN/opencpn.exe`, `C:\OpenCPN.\opencpn.exe`, `C:\OpenCPN\other.exe`, `C:\\OpenCPN\opencpn.exe`, `C:\CON\opencpn.exe`, `C:\COM0\opencpn.exe`, `C:\COM¹\opencpn.exe`, `C:\LPT³.txt\opencpn.exe`, "C:\\Open\nCPN\\opencpn.exe"} {
		if localStockPath(path) {
			t.Errorf("unsafe stock path accepted: %q", path)
		}
	}
}

func TestApprovedRequestPreservesNoncanonicalJSONBytesAndAccountRID(t *testing.T) {
	valid := approvedFixture(t)
	for _, sid := range []string{"S-1-5-21-123-456-789-500", "S-1-5-21-4294967295-456-789-4294967295"} {
		var r Request
		if err := json.Unmarshal(valid, &r); err != nil {
			t.Fatal(err)
		}
		r.ClientSID = sid
		trust := append([]byte("{\n  "), r.TrustConfiguration[1:]...)
		r.TrustConfigSHA256 = digest(trust)
		r.TrustConfiguration = json.RawMessage(`"TRUST_PLACEHOLDER"`)
		raw, err := json.Marshal(r)
		if err != nil {
			t.Fatal(err)
		}
		raw = bytes.Replace(raw, []byte(`"TRUST_PLACEHOLDER"`), trust, 1)
		raw = append(append([]byte("\n  "), raw...), '\n')
		parsed, err := ParseApprovedRequest(raw, digest(raw), fixtureNow)
		if err != nil {
			t.Fatal(err)
		}
		if parsed.ClientSID != sid || !bytes.Equal(parsed.TrustConfiguration, trust) || !bytes.Equal(parsed.OriginalBytes(), raw) {
			t.Fatal("exact public trust/request bytes or account RID lost")
		}
	}
}

func TestApprovedRequestValidatesOriginalFullTrustConfiguration(t *testing.T) {
	valid := approvedFixture(t)
	for _, scenario := range []string{"unknown", "case alias", "duplicate", "null", "bad root", "root extra", "http", "different stock", "wrong channel", "bad signature", "oversize"} {
		t.Run(scenario, func(t *testing.T) {
			var r Request
			if err := json.Unmarshal(valid, &r); err != nil {
				t.Fatal(err)
			}
			trust := bytes.Clone(r.TrustConfiguration)
			switch scenario {
			case "unknown":
				trust = bytes.Replace(trust, []byte(`"schema":1`), []byte(`"schema":1,"extra":true`), 1)
			case "case alias":
				trust = bytes.Replace(trust, []byte(`"channel"`), []byte(`"Channel"`), 1)
			case "duplicate":
				trust = bytes.Replace(trust, []byte(`"channel":"beta"`), []byte(`"channel":"beta","Channel":"beta"`), 1)
			case "null":
				trust = bytes.Replace(trust, []byte(`"schema":1`), []byte(`"schema":null`), 1)
			case "bad root":
				trust = bytes.Replace(trust, []byte(`"bootstrapRoot":`), []byte(`"bootstrapRoot":false,"ignored":`), 1)
			case "root extra":
				trust = bytes.Replace(trust, []byte(`"consistent_snapshot":true`), []byte(`"consistent_snapshot":true,"extra":1`), 1)
			case "http":
				trust = bytes.ReplaceAll(trust, []byte("https://"), []byte("http://"))
			case "different stock":
				trust = bytes.Replace(trust, []byte(`"arch":"x86"`), []byte(`"arch":"x64"`), 1)
			case "wrong channel":
				trust = bytes.Replace(trust, []byte(`"channel":"beta"`), []byte(`"channel":"stable"`), 1)
			case "bad signature":
				var fields map[string]json.RawMessage
				_ = json.Unmarshal(trust, &fields)
				fields["bootstrapRoot"] = json.RawMessage(`{"signed":{},"signatures":[]}`)
				trust, _ = json.Marshal(fields)
			case "oversize":
				trust = append(bytes.Repeat([]byte(" "), MaxTrustConfigurationBytes), trust...)
			}
			// Preserve exact trust bytes even when JSON marshaling would compact them.
			r.TrustConfiguration = json.RawMessage(`"TRUST_PLACEHOLDER"`)
			r.TrustConfigSHA256 = digest(trust)
			raw, err := json.Marshal(r)
			if err != nil {
				t.Fatal(err)
			}
			raw = bytes.Replace(raw, []byte(`"TRUST_PLACEHOLDER"`), trust, 1)
			if _, err = ParseApprovedRequest(raw, digest(raw), fixtureNow); err == nil {
				t.Fatal("invalid public trust accepted despite separate request approval")
			}
		})
	}
}
