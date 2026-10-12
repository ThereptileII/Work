package verifier

import (
	"archive/zip"
	"bytes"
	"context"
	"crypto/ed25519"
	"crypto/sha256"
	"encoding/json"
	"errors"
	"io"
	"strings"
	"testing"
	"time"

	"example.com/opennav-update-verifier/baselineevidence"
	"example.com/opennav-update-verifier/brokerregistration"
	"example.com/opennav-update-verifier/releasepolicy"
	"example.com/opennav-update-verifier/repository"
	"github.com/theupdateframework/go-tuf/v2/metadata"
)

var approvedBaselineNow = time.Date(2030, 1, 2, 12, 0, 0, 0, time.UTC)

type approvedBaselineFixture struct {
	entries      map[string][]byte
	raw, archive []byte
	installed    brokerregistration.Installed
}

func newApprovedBaselineFixture(t *testing.T, alter func(map[string][]byte)) approvedBaselineFixture {
	t.Helper()
	entries := map[string][]byte{"app/opencpn.exe": []byte("inert baseline executable"), "app/opennav-restart.exe": []byte("inert restart helper"), "app/skager-start.exe": []byte("inert startup launcher"), "app/OPENNAV_PORTABLE_PREVIEW": []byte("portable marker excluded"), "profile/opencpn.conf": []byte("recovery profile excluded")}
	entries["docs/PRODUCT_BUILD.json"] = recoveryJSON(t, map[string]interface{}{"version": "2.0.0", "commit": strings.Repeat("a", 40), "test_fixtures": false, "build_purpose": "INSTALLED PRODUCT", "xnav_hardware_output_policy": "status-only", "xnav_manual_control_contract": 0, "executable_sha256": recoveryDigest(entries["app/opencpn.exe"]), "restart_helper_sha256": recoveryDigest(entries["app/opennav-restart.exe"]), "startup_launcher_sha256": recoveryDigest(entries["app/skager-start.exe"]), "commissioning_restart_protocol": 1, "update_startup_health": 1})
	var files []map[string]string
	for _, name := range recoveryNames(entries) {
		if (strings.HasPrefix(name, "app/") || strings.HasPrefix(name, "docs/")) && name != "app/OPENNAV_PORTABLE_PREVIEW" {
			files = append(files, map[string]string{"path": name, "sha256": recoveryDigest(entries[name])})
		}
	}
	entries["installer/package.json"] = append([]byte(" \n"), recoveryJSON(t, map[string]interface{}{"schema": 1, "version": "2.0.0", "commit": strings.Repeat("a", 40), "payloadSha256": strings.Repeat("b", 64), "supportedOpenCpn": []releasepolicy.OpenCpn{brokerregistration.StockBaseline()}, "files": files, "qualification": "candidate"})...)
	if alter != nil {
		alter(entries)
	}
	inventory := map[string]string{}
	for name, raw := range entries {
		if name != "FILE_SHA256.json" {
			inventory[name] = recoveryDigest(raw)
		}
	}
	if _, ok := entries["FILE_SHA256.json"]; !ok {
		entries["FILE_SHA256.json"] = append(recoveryJSON(t, inventory), '\n')
	}
	root := metadata.Root(approvedBaselineNow.Add(24 * time.Hour))
	root.Signed.ConsistentSnapshot = true
	var rootPrivate ed25519.PrivateKey
	for _, role := range []string{"root", "timestamp", "snapshot", "targets"} {
		seed := sha256.Sum256([]byte("INERT APPROVED BASELINE TEST ONLY " + role))
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
	sign(t, root, rootPrivate)
	trust := recoveryJSON(t, repository.Trust{Schema: 1, BootstrapRoot: bytesOf(t, root), MetadataURL: "https://updates.invalid/metadata/", ArtifactOrigin: "https://updates.invalid", Channel: "beta", LocalCompatibilityAllowlist: []releasepolicy.OpenCpn{brokerregistration.StockBaseline()}})
	installed := brokerregistration.Installed{Version: "2.0.0", Commit: strings.Repeat("a", 40), Generation: strings.Repeat("c", 32), PackageSHA256: recoveryDigest(entries["installer/package.json"]), ProductBuildSHA256: recoveryDigest(entries["docs/PRODUCT_BUILD.json"]), ExecutableSHA256: recoveryDigest(entries["app/opencpn.exe"]), InventorySHA256: recoveryDigest(entries["FILE_SHA256.json"])}
	stock := brokerregistration.StockBaseline()
	raw := recoveryJSON(t, brokerregistration.Request{Schema: 1, Purpose: brokerregistration.Purpose, ClientSID: "S-1-5-21-123-456-789-500", Installed: installed, Stock: brokerregistration.Stock{Path: `C:\Program Files (x86)\OpenCPN\opencpn.exe`, Version: stock.Version, Arch: stock.Arch, ExecutableSHA256: stock.ExecutableSHA256, UpstreamCommit: stock.UpstreamCommit}, TrustConfiguration: trust, TrustConfigSHA256: recoveryDigest(trust)})
	return approvedBaselineFixture{entries, raw, recoveryArchive(t, entries, nil), installed}
}

type approvedBaselineSink struct {
	files, evidence map[string][]byte
	onFile          func(string, io.Reader) error
	onEvidence      func(string) error
}

func newApprovedBaselineSink() *approvedBaselineSink {
	return &approvedBaselineSink{files: map[string][]byte{}, evidence: map[string][]byte{}}
}
func (s *approvedBaselineSink) WriteFile(ctx context.Context, name string, r io.Reader, size int64, hash string) error {
	if s.onFile != nil {
		return s.onFile(name, r)
	}
	raw, err := io.ReadAll(r)
	if err != nil {
		return err
	}
	if int64(len(raw)) != size || recoveryDigest(raw) != hash {
		return errors.New("sink observed identity differs")
	}
	s.files[name] = raw
	return nil
}
func (s *approvedBaselineSink) WriteEvidence(ctx context.Context, name string, raw []byte, hash string) error {
	if s.onEvidence != nil {
		if err := s.onEvidence(name); err != nil {
			return err
		}
	}
	if recoveryDigest(raw) != hash {
		return errors.New("sink evidence identity differs")
	}
	s.evidence[name] = bytes.Clone(raw)
	raw[0] = '!'
	return nil
}
func importFixture(ctx context.Context, f approvedBaselineFixture, s BaselineSink) (RecoveryIdentity, error) {
	return importApprovedBaselineAt(ctx, f.raw, recoveryDigest(f.raw), bytes.NewReader(f.archive), int64(len(f.archive)), s, approvedBaselineNow)
}

func TestImportApprovedBaselineRetainsExactEvidenceAndPayload(t *testing.T) {
	f := newApprovedBaselineFixture(t, nil)
	sink := newApprovedBaselineSink()
	identity, err := importFixture(context.Background(), f, sink)
	if err != nil {
		t.Fatal(err)
	}
	if identity.Version != f.installed.Version || identity.Commit != f.installed.Commit || identity.ExecutableSHA256 != f.installed.ExecutableSHA256 || identity.InventorySHA256 != f.installed.InventorySHA256 || identity.ProductBuildSHA256 != f.installed.ProductBuildSHA256 || identity.LifecycleManifestSHA256 != f.installed.PackageSHA256 || !identity.LifecycleManifestAvailable {
		t.Fatal("approved identities lost")
	}
	if identity.DeclaredPayloadSHA256 == recoveryDigest(f.archive) || identity.LifecycleManifestSHA256 == recoveryDigest(f.archive) {
		t.Fatal("outer ZIP substituted for manifest identity")
	}
	if len(sink.files) != 4 || len(sink.evidence) != 2 {
		t.Fatalf("unexpected namespaces: %v %v", sink.files, sink.evidence)
	}
	for name, raw := range sink.files {
		if !bytes.Equal(raw, f.entries[name]) {
			t.Fatal("payload bytes changed")
		}
	}
	for _, name := range []string{"FILE_SHA256.json", "installer/package.json"} {
		if !bytes.Equal(sink.evidence[name], f.entries[name]) {
			t.Fatal("original evidence serialized or changed")
		}
	}
	observed := map[string]string{}
	for name, raw := range sink.files {
		observed[name] = recoveryDigest(raw)
	}
	retained, payload, err := baselineevidence.Validate(context.Background(), sink.evidence["FILE_SHA256.json"], sink.evidence["installer/package.json"], sink.files["docs/PRODUCT_BUILD.json"], observed, f.installed)
	if err != nil || retained != identity || len(payload) != len(sink.files) {
		t.Fatalf("retained native evidence check differs: %v", err)
	}
}

func TestImportApprovedBaselinePreservesInventoriedEngine(t *testing.T) {
	f := newApprovedBaselineFixture(t, func(entries map[string][]byte) {
		for name, raw := range recoveryEngineFixture() {
			entries[name] = raw
		}
	})
	sink := newApprovedBaselineSink()
	identity, err := importFixture(context.Background(), f, sink)
	if err != nil || len(sink.files) != 4 || len(sink.evidence) != 5 {
		t.Fatalf("exact original helpers missing: %v", err)
	}
	observed := map[string]string{}
	for name, raw := range sink.files {
		observed[name] = recoveryDigest(raw)
	}
	for name, raw := range recoveryEngineFixture() {
		if !bytes.Equal(sink.evidence[name], raw) {
			t.Fatalf("original helper reserialized: %s", name)
		}
		observed[name] = recoveryDigest(sink.evidence[name])
	}
	retained, members, err := baselineevidence.Validate(context.Background(), sink.evidence["FILE_SHA256.json"], sink.evidence["installer/package.json"], sink.files["docs/PRODUCT_BUILD.json"], observed, f.installed)
	if err != nil || retained != identity || len(members) != 7 {
		t.Fatalf("helper bytes not bound to original approval: %v", err)
	}
	for _, scenario := range []string{"refused", "canceled"} {
		t.Run(scenario, func(t *testing.T) {
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			failedSink := newApprovedBaselineSink()
			failedSink.onEvidence = func(name string) error {
				if baselineevidence.IsEngineHelper(name) {
					if scenario == "canceled" {
						cancel()
					}
					return errors.New("helper refused")
				}
				return nil
			}
			identity, err := importFixture(ctx, f, failedSink)
			if err == nil || identity != (RecoveryIdentity{}) {
				t.Fatal("failed helper returned baseline identity")
			}
			if scenario == "canceled" && !errors.Is(err, context.Canceled) {
				t.Fatal("helper cancellation lost")
			}
		})
	}
}

func TestImportApprovedBaselineRejectsUnapprovedOrMalformedSource(t *testing.T) {
	cases := map[string]func(map[string][]byte){
		"missing package":    func(e map[string][]byte) { delete(e, "installer/package.json") },
		"missing product":    func(e map[string][]byte) { delete(e, "docs/PRODUCT_BUILD.json") },
		"extra payload":      func(e map[string][]byte) { e["app/extra.dll"] = []byte("unmanifested loader") },
		"case alias":         func(e map[string][]byte) { e["app/OpenCPN.exe"] = []byte("alias") },
		"parent alias":       func(e map[string][]byte) { e["app"] = []byte("file") },
		"unsafe name":        func(e map[string][]byte) { e["app/../bad.dll"] = []byte("unsafe") },
		"missing executable": func(e map[string][]byte) { delete(e, "app/opencpn.exe") },
		"package unknown field": func(e map[string][]byte) {
			e["installer/package.json"] = bytes.Replace(e["installer/package.json"], []byte(`"schema":1`), []byte(`"schema":1,"unknown":true`), 1)
		},
		"package alias field": func(e map[string][]byte) {
			e["installer/package.json"] = bytes.Replace(e["installer/package.json"], []byte(`"schema":1`), []byte(`"Schema":1`), 1)
		},
		"package duplicate": func(e map[string][]byte) {
			e["installer/package.json"] = bytes.Replace(e["installer/package.json"], []byte(`"schema":1`), []byte(`"schema":1,"schema":1`), 1)
		},
		"package null": func(e map[string][]byte) {
			e["installer/package.json"] = bytes.Replace(e["installer/package.json"], []byte(`"schema":1`), []byte(`"schema":null`), 1)
		},
		"product fixtures": func(e map[string][]byte) {
			e["docs/PRODUCT_BUILD.json"] = bytes.Replace(e["docs/PRODUCT_BUILD.json"], []byte(`"test_fixtures":false`), []byte(`"test_fixtures":true`), 1)
		},
		"product alias field": func(e map[string][]byte) {
			e["docs/PRODUCT_BUILD.json"] = bytes.Replace(e["docs/PRODUCT_BUILD.json"], []byte(`"version"`), []byte(`"Version"`), 1)
		},
		"product output policy": func(e map[string][]byte) {
			e["docs/PRODUCT_BUILD.json"] = bytes.Replace(e["docs/PRODUCT_BUILD.json"], []byte(`"status-only"`), []byte(`"unguarded"`), 1)
		},
		"inventory duplicate": func(e map[string][]byte) {
			e["FILE_SHA256.json"] = []byte(`{"app/opencpn.exe":"` + recoveryDigest(e["app/opencpn.exe"]) + `","APP/OPENCPN.EXE":"` + recoveryDigest(e["app/opencpn.exe"]) + `"}`)
		},
		"wrong inner bytes": func(e map[string][]byte) {
			e["FILE_SHA256.json"] = []byte(`{"app/opencpn.exe":"` + strings.Repeat("0", 64) + `"}`)
		},
	}
	for name, alter := range cases {
		t.Run(name, func(t *testing.T) {
			f := newApprovedBaselineFixture(t, alter)
			sink := newApprovedBaselineSink()
			identity, err := importFixture(context.Background(), f, sink)
			if err == nil || identity != (RecoveryIdentity{}) || len(sink.files) != 0 || len(sink.evidence) != 0 {
				t.Fatalf("unsafe source reached sink: %v", err)
			}
		})
	}
}

func TestImportApprovedBaselineRefusesWrongIndependentPinsAndExpiredInitialTrust(t *testing.T) {
	f := newApprovedBaselineFixture(t, nil)
	for _, field := range []string{"packageSha256", "productBuildSha256", "executableSha256", "inventorySha256"} {
		t.Run(field, func(t *testing.T) {
			var request map[string]json.RawMessage
			json.Unmarshal(f.raw, &request)
			var installed map[string]string
			json.Unmarshal(request["installed"], &installed)
			installed[field] = strings.Repeat("0", 64)
			request["installed"] = recoveryJSON(t, installed)
			changed := f
			changed.raw = recoveryJSON(t, request)
			sink := newApprovedBaselineSink()
			identity, err := importFixture(context.Background(), changed, sink)
			if err == nil || identity != (RecoveryIdentity{}) || len(sink.files) != 0 {
				t.Fatal("wrong approved pin accepted")
			}
		})
	}
	for name, now := range map[string]time.Time{"expired": approvedBaselineNow.Add(48 * time.Hour), "zero": {}} {
		t.Run(name, func(t *testing.T) {
			sink := newApprovedBaselineSink()
			_, err := importApprovedBaselineAt(context.Background(), f.raw, recoveryDigest(f.raw), bytes.NewReader(f.archive), int64(len(f.archive)), sink, now)
			if err == nil || len(sink.files) != 0 {
				t.Fatal("invalid initial trust clock accepted")
			}
		})
	}
	_, err := importApprovedBaselineAt(context.Background(), f.raw, strings.Repeat("0", 64), bytes.NewReader(f.archive), int64(len(f.archive)), newApprovedBaselineSink(), approvedBaselineNow)
	if err == nil {
		t.Fatal("adjacent request hash accepted")
	}
}

func TestImportApprovedBaselineRequiresConsumptionAndCancellation(t *testing.T) {
	f := newApprovedBaselineFixture(t, nil)
	for _, mode := range []string{"unread", "partial", "sink failure", "evidence failure", "cancel payload", "cancel evidence", "pre-cancel"} {
		t.Run(mode, func(t *testing.T) {
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			sink := newApprovedBaselineSink()
			switch mode {
			case "unread":
				sink.onFile = func(string, io.Reader) error { return nil }
			case "partial":
				sink.onFile = func(_ string, r io.Reader) error { var b [1]byte; _, err := r.Read(b[:]); return err }
			case "sink failure":
				sink.onFile = func(string, io.Reader) error { return errors.New("injected sink failure") }
			case "evidence failure":
				sink.onEvidence = func(string) error { return errors.New("injected evidence failure") }
			case "cancel payload":
				sink.onFile = func(_ string, r io.Reader) error { _, err := io.Copy(io.Discard, r); cancel(); return err }
			case "cancel evidence":
				sink.onEvidence = func(string) error { cancel(); return nil }
			case "pre-cancel":
				cancel()
			}
			identity, err := importFixture(ctx, f, sink)
			if err == nil || identity != (RecoveryIdentity{}) {
				t.Fatal("incomplete import succeeded")
			}
			if strings.Contains(mode, "cancel") && !errors.Is(err, context.Canceled) {
				t.Fatalf("cancellation identity lost: %v", err)
			}
		})
	}
}

func TestImportApprovedBaselineRejectsZIPAliasAndCRC(t *testing.T) {
	for _, mode := range []string{"duplicate member", "CRC", "missing inventory", "declared truncated", "trailing bytes"} {
		t.Run(mode, func(t *testing.T) {
			f := newApprovedBaselineFixture(t, nil)
			switch mode {
			case "missing inventory":
				var raw bytes.Buffer
				archive := zip.NewWriter(&raw)
				for _, name := range recoveryNames(f.entries) {
					if name == "FILE_SHA256.json" {
						continue
					}
					member, err := archive.Create(recoveryPrefix + name)
					if err != nil {
						t.Fatal(err)
					}
					if _, err = member.Write(f.entries[name]); err != nil {
						t.Fatal(err)
					}
				}
				if err := archive.Close(); err != nil {
					t.Fatal(err)
				}
				f.archive = raw.Bytes()
			case "duplicate member":
				f.archive = recoveryArchive(t, f.entries, func(z *zip.Writer) {
					w, err := z.Create(recoveryPrefix + "app/opencpn.exe")
					if err != nil {
						t.Fatal(err)
					}
					w.Write(f.entries["app/opencpn.exe"])
				})
			case "CRC":
				z, err := zip.NewReader(bytes.NewReader(f.archive), int64(len(f.archive)))
				if err != nil {
					t.Fatal(err)
				}
				offset, err := z.File[0].DataOffset()
				if err != nil {
					t.Fatal(err)
				}
				f.archive[offset] ^= 0x40
			case "declared truncated":
				f.archive = f.archive[:len(f.archive)-1]
			case "trailing bytes":
				f.archive = append(f.archive, 0)
			}
			sink := newApprovedBaselineSink()
			identity, err := importFixture(context.Background(), f, sink)
			if err == nil || identity != (RecoveryIdentity{}) || len(sink.files) != 0 {
				t.Fatalf("invalid ZIP accepted: %v", err)
			}
		})
	}
}

func TestRetainedBaselineEvidenceRequiresExactObservedPayload(t *testing.T) {
	f := newApprovedBaselineFixture(t, nil)
	observed := map[string]string{}
	for name, raw := range f.entries {
		if (strings.HasPrefix(name, "app/") || strings.HasPrefix(name, "docs/")) && name != "app/OPENNAV_PORTABLE_PREVIEW" {
			observed[name] = recoveryDigest(raw)
		}
	}
	for _, mode := range []string{"missing", "extra", "alias", "wrong digest", "portable marker", "wrong manifest pin", "changed original inventory"} {
		t.Run(mode, func(t *testing.T) {
			copy := map[string]string{}
			for name, hash := range observed {
				copy[name] = hash
			}
			pins := f.installed
			inventory := f.entries["FILE_SHA256.json"]
			switch mode {
			case "missing":
				delete(copy, "app/opencpn.exe")
			case "extra":
				copy["app/extra.dll"] = strings.Repeat("0", 64)
			case "alias":
				copy["app/OpenCPN.exe"] = copy["app/opencpn.exe"]
			case "wrong digest":
				copy["app/opencpn.exe"] = strings.Repeat("0", 64)
			case "portable marker":
				copy["app/OPENNAV_PORTABLE_PREVIEW"] = recoveryDigest(f.entries["app/OPENNAV_PORTABLE_PREVIEW"])
			case "wrong manifest pin":
				pins.PackageSHA256 = strings.Repeat("0", 64)
			case "changed original inventory":
				inventory = append(bytes.Clone(inventory), ' ')
			}
			identity, payload, err := baselineevidence.Validate(context.Background(), inventory, f.entries["installer/package.json"], f.entries["docs/PRODUCT_BUILD.json"], copy, pins)
			if err == nil || identity != (baselineevidence.Identity{}) || payload != nil {
				t.Fatal("unqualified retained baseline accepted")
			}
		})
	}
}

func TestImportApprovedBaselineProductionClockAndSourceBounds(t *testing.T) {
	f := newApprovedBaselineFixture(t, nil)
	var request brokerregistration.Request
	if err := json.Unmarshal(f.raw, &request); err != nil {
		t.Fatal(err)
	}
	var trust repository.Trust
	if err := json.Unmarshal(request.TrustConfiguration, &trust); err != nil {
		t.Fatal(err)
	}
	root, err := metadata.Root().FromBytes(trust.BootstrapRoot)
	if err != nil {
		t.Fatal(err)
	}
	root.Signed.Expires = time.Date(2000, 1, 1, 0, 0, 0, 0, time.UTC)
	root.Signatures = nil
	seed := sha256.Sum256([]byte("INERT APPROVED BASELINE TEST ONLY root"))
	sign(t, root, ed25519.NewKeyFromSeed(seed[:]))
	trust.BootstrapRoot = bytesOf(t, root)
	request.TrustConfiguration = recoveryJSON(t, trust)
	request.TrustConfigSHA256 = recoveryDigest(request.TrustConfiguration)
	expired := recoveryJSON(t, request)
	if _, err := ImportApprovedBaseline(context.Background(), expired, recoveryDigest(expired), bytes.NewReader(f.archive), int64(len(f.archive)), newApprovedBaselineSink()); err == nil {
		t.Fatal("production initial enrollment ignored actual expiry")
	}
	for _, size := range []int64{-1, 0, 21, maxDownloadBytes + 1} {
		if _, err := importApprovedBaselineAt(context.Background(), f.raw, recoveryDigest(f.raw), bytes.NewReader(f.archive), size, newApprovedBaselineSink(), approvedBaselineNow); err == nil {
			t.Fatalf("unsafe source size %d accepted", size)
		}
	}
	if _, err := importApprovedBaselineAt(nil, f.raw, recoveryDigest(f.raw), bytes.NewReader(f.archive), int64(len(f.archive)), newApprovedBaselineSink(), approvedBaselineNow); err == nil {
		t.Fatal("nil context accepted")
	}
	if _, err := importApprovedBaselineAt(context.Background(), f.raw, recoveryDigest(f.raw), nil, int64(len(f.archive)), newApprovedBaselineSink(), approvedBaselineNow); err == nil {
		t.Fatal("nil source accepted")
	}
	if _, err := importApprovedBaselineAt(context.Background(), f.raw, recoveryDigest(f.raw), bytes.NewReader(f.archive), int64(len(f.archive)), nil, approvedBaselineNow); err == nil {
		t.Fatal("nil sink accepted")
	}
}

func TestImportApprovedBaselineDetectsMutationDuringStaging(t *testing.T) {
	f := newApprovedBaselineFixture(t, nil)
	z, err := zip.NewReader(bytes.NewReader(f.archive), int64(len(f.archive)))
	if err != nil {
		t.Fatal(err)
	}
	var mutateOffset int64
	for _, member := range z.File {
		if member.Name == recoveryPrefix+"docs/PRODUCT_BUILD.json" {
			mutateOffset, err = member.DataOffset()
			if err != nil {
				t.Fatal(err)
			}
		}
	}
	if mutateOffset == 0 {
		t.Fatal("fixture product member absent")
	}
	sink := newApprovedBaselineSink()
	first := true
	sink.onFile = func(_ string, r io.Reader) error {
		_, err := io.Copy(io.Discard, r)
		if first {
			first = false
			f.archive[mutateOffset] ^= 0x40
		}
		return err
	}
	identity, err := importFixture(context.Background(), f, sink)
	if err == nil || identity != (RecoveryIdentity{}) || len(sink.evidence) != 0 {
		t.Fatalf("source mutation survived exact streaming check: %v", err)
	}
}

type cancelBaselineReaderAt struct {
	source io.ReaderAt
	cancel context.CancelFunc
}

func (r cancelBaselineReaderAt) ReadAt(p []byte, off int64) (int, error) {
	n, err := r.source.ReadAt(p, off)
	r.cancel()
	return n, err
}
func TestImportApprovedBaselineCancellationDuringArchiveRead(t *testing.T) {
	f := newApprovedBaselineFixture(t, nil)
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	sink := newApprovedBaselineSink()
	identity, err := importApprovedBaselineAt(ctx, f.raw, recoveryDigest(f.raw), cancelBaselineReaderAt{bytes.NewReader(f.archive), cancel}, int64(len(f.archive)), sink, approvedBaselineNow)
	if err == nil || identity != (RecoveryIdentity{}) || len(sink.files) != 0 {
		t.Fatal("cancelled source reached sink")
	}
	// The ZIP framing helper deliberately hides physical read errors; callers must
	// still receive cancellation identity at the public authority boundary.
	if !errors.Is(err, context.Canceled) {
		t.Fatalf("archive cancellation identity lost: %v", err)
	}
}
