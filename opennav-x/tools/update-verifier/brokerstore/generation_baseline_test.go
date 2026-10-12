package brokerstore

import (
	"bytes"
	"context"
	"crypto"
	"crypto/ed25519"
	"crypto/sha256"
	"encoding/json"
	"errors"
	"strings"
	"testing"
	"time"

	"example.com/opennav-update-verifier/brokerregistration"
	"example.com/opennav-update-verifier/releasepolicy"
	"example.com/opennav-update-verifier/repository"
	"github.com/sigstore/sigstore/pkg/signature"
	"github.com/theupdateframework/go-tuf/v2/metadata"
)

// Deterministic inert signing material never authenticates any operator/product.
func baselineViewRequest(t *testing.T, sid string, f baselineFixtureData) []byte {
	t.Helper()
	root := metadata.Root(time.Date(2036, 1, 1, 0, 0, 0, 0, time.UTC))
	root.Signed.ConsistentSnapshot = true
	var signing ed25519.PrivateKey
	for _, role := range []string{"root", "timestamp", "snapshot", "targets"} {
		seed := sha256.Sum256([]byte("INERT BASELINE VIEW ONLY " + role))
		key := ed25519.NewKeyFromSeed(seed[:])
		public, err := metadata.KeyFromPublicKey(key.Public())
		if err != nil {
			t.Fatal(err)
		}
		if err = root.Signed.AddKey(public, role); err != nil {
			t.Fatal(err)
		}
		if role == "root" {
			signing = key
		}
	}
	signer, err := signature.LoadSigner(signing, crypto.Hash(0))
	if err != nil {
		t.Fatal(err)
	}
	if _, err = root.Sign(signer); err != nil {
		t.Fatal(err)
	}
	rootRaw, err := root.ToBytes(false)
	if err != nil {
		t.Fatal(err)
	}
	trust, err := json.Marshal(repository.Trust{Schema: 1, BootstrapRoot: rootRaw, MetadataURL: "https://updates.invalid/metadata/", ArtifactOrigin: "https://updates.invalid", Channel: "beta", LocalCompatibilityAllowlist: []releasepolicy.OpenCpn{brokerregistration.StockBaseline()}})
	if err != nil {
		t.Fatal(err)
	}
	stock := brokerregistration.StockBaseline()
	raw, err := json.Marshal(brokerregistration.Request{Schema: 1, Purpose: brokerregistration.Purpose, ClientSID: sid, Installed: f.expected, Stock: brokerregistration.Stock{Path: `C:\Program Files (x86)\OpenCPN\opencpn.exe`, Version: stock.Version, Arch: stock.Arch, ExecutableSHA256: stock.ExecutableSHA256, UpstreamCommit: stock.UpstreamCommit}, TrustConfiguration: trust, TrustConfigSHA256: registrationDigest(trust)})
	if err != nil {
		t.Fatal(err)
	}
	return raw
}
func baselineViewEnvelope(t *testing.T, raw []byte, sid string) []byte {
	t.Helper()
	envelope, err := json.Marshal(baselineRegistrationEnvelope{2, sid, registrationDigest(raw), raw})
	if err != nil {
		t.Fatal(err)
	}
	return envelope
}

type memoryBaselineViewStore struct {
	*baselineMemoryStore
	view     *memoryBaselineView
	prepares int
}

func (s *memoryBaselineViewStore) prepareBaselineGeneration(_ context.Context, previous installedBaselineBackend, request brokerregistration.Request) (baselineGenerationBackend, error) {
	s.prepares++
	if s.view.registration == nil {
		return nil, testFault
	}
	return s.view, nil
}

type memoryBaselineView struct {
	*readOnlyMemoryBackend
	previous      *InstalledBaseline
	registration  []byte
	request       brokerregistration.Request
	sourceChanged bool
}

func (b *memoryBaselineView) baselineSource(m map[string]baselineMember) (map[string]diskIdentity, map[string]diskIdentity, generationBaselineOrigin, error) {
	dirs := map[string]diskIdentity{}
	for name, id := range b.previous.record.Directories {
		dirs[name] = id
	}
	files := map[string]diskIdentity{}
	for name, m := range m {
		files[name] = m.Identity
	}
	if b.sourceChanged && b.tree {
		files["app/opencpn.exe"] = baselineTestID(999)
	}
	raw, _ := json.Marshal(b.previous.record)
	origin := generationBaselineOrigin{ApprovedRequestSHA256: b.request.RequestSHA256, Registration: bytes.Clone(b.registration), RegistrationMarker: registrationMarker(b.request.ClientSID, b.request.Trust().Channel, b.request.RequestSHA256, registrationDigest(b.registration), true), RegistrationIdentity: baselineTestID(700), RegistrationMarkerIdentity: baselineTestID(701), BaselineMarker: baselineMarker(b.request.ClientSID, b.request.Installed.Generation, b.request.RequestSHA256, registrationDigest(raw), true), BaselineMarkerIdentity: baselineTestID(702), BaselineCompletionIdentity: baselineTestID(703)}
	return dirs, files, origin, nil
}
func baselineViewFixture(t *testing.T) (*Store, *InstalledBaseline, *memoryBaselineView, baselineFixtureData, []byte) {
	t.Helper()
	f := baselineEngineData(t)
	raw := baselineViewRequest(t, registrationTestSID, f)
	pin := registrationDigest(raw)
	m := &memoryBaseline{memoryCandidate: &memoryCandidate{files: map[string]*memoryFile{}}}
	s := &Store{backend: &baselineMemoryStore{memoryStore: &memoryStore{}, b: m}}
	p, err := s.BeginInstalledBaselineFresh(context.Background(), registrationTestSID, stageA, pin)
	if err != nil {
		t.Fatal(err)
	}
	writeBaseline(t, p, f)
	if err = p.Complete(context.Background(), f.identity); err != nil {
		t.Fatal(err)
	}
	request, err := brokerregistration.ParseRetainedApprovedRequest(raw, pin)
	if err != nil {
		t.Fatal(err)
	}
	view := &memoryBaselineView{readOnlyMemoryBackend: &readOnlyMemoryBackend{memoryGeneration: &memoryGeneration{memoryCandidate: &memoryCandidate{files: map[string]*memoryFile{}}}, old: map[string]*memoryFile{}, readers: map[string]*memoryFile{}}, previous: p, registration: baselineViewEnvelope(t, raw, registrationTestSID), request: request}
	s.backend = &memoryBaselineViewStore{baselineMemoryStore: s.backend.(*baselineMemoryStore), view: view}
	t.Cleanup(func() { s.Close() })
	return s, p, view, f, raw
}
func TestGenerationBaselineCopiesOriginalsWithDistinctProvenanceAndReadonlyLifetime(t *testing.T) {
	s, p, b, f, raw := baselineViewFixture(t)
	owner, err := s.MaterializeApprovedBaselineFresh(context.Background(), p, raw, registrationDigest(raw))
	if err != nil {
		t.Fatal(err)
	}
	completion, err := owner.Completion()
	if err != nil || completion.Generation != p.record.Generation || completion.Purpose != approvedBaselineGenerationPurpose || completion.ApprovedRequestSHA256 != registrationDigest(raw) || completion.Identity != f.identity {
		t.Fatal("approved baseline completion differs", err)
	}
	if _, err = owner.generation.Completion(); err == nil {
		t.Fatal("baseline masquerades as signed candidate")
	}
	var fields map[string]json.RawMessage
	if err = json.Unmarshal(owner.generation.raw, &fields); err != nil {
		t.Fatal(err)
	}
	if !bytes.Contains(fields["approvedBaseline"], []byte("registrationIdentity")) {
		t.Fatal("protected registration original identity missing")
	}
	if owner.generation.record.Acquisition != (AcquisitionPlan{}) {
		t.Fatal("fabricated candidate acquisition")
	}
	originals := map[string][]byte{"FILE_SHA256.json": f.inventory, "installer/package.json": f.manifest}
	for name, raw := range f.payload {
		originals[name] = raw
	}
	for name, want := range originals {
		if !bytes.Equal(b.files[name].data, want) {
			t.Fatal("original changed", name)
		}
		b.old[name] = b.files[name]
	}
	r, err := owner.TransitionReadOnly(context.Background())
	if err != nil {
		t.Fatal(err)
	}
	after, err := r.Completion()
	if err != nil || after != completion {
		t.Fatal("readonly approval changed", err)
	}
	for name, want := range originals {
		h, err := r.OpenMember(name)
		if err != nil {
			t.Fatal(err)
		}
		got := make([]byte, len(want))
		if _, err = h.ReadAt(got, 0); err != nil || !bytes.Equal(got, want) {
			t.Fatal(name, err)
		}
		h.Close()
	}
	if err = p.Close(); err != nil {
		t.Fatal(err)
	}
	if err = r.Validate(); err != nil {
		t.Fatal("public owner depends on closed private source", err)
	}
	h, err := r.OpenMember("app/opencpn.exe")
	if err != nil {
		t.Fatal(err)
	}
	if err = s.Close(); err != nil {
		t.Fatal(err)
	}
	if _, err = h.ReadAt(make([]byte, 1), 0); !errors.Is(err, ErrClosed) {
		t.Fatal("Store did not revoke baseline view", err)
	}
	for name, file := range b.old {
		if file.closes != 1 {
			t.Fatal("writer close repeated", name)
		}
	}
	for name, file := range b.readers {
		if file.closes != 1 {
			t.Fatal("reader close repeated", name)
		}
	}
}
func TestGenerationBaselineRefusesUnboundOriginalsBeforeReservation(t *testing.T) {
	for _, scenario := range []string{"cancelled", "other-store", "not-complete", "wrong-independent-pin", "changed-original-request", "changed-installed-pin", "missing-registration", "changed-registration", "mixed-acquisition", "tampered-source", "missing-helper"} {
		t.Run(scenario, func(t *testing.T) {
			s, p, b, _, raw := baselineViewFixture(t)
			pin := registrationDigest(raw)
			ctx := context.Background()
			switch scenario {
			case "cancelled":
				c, cancel := context.WithCancel(ctx)
				cancel()
				ctx = c
			case "other-store":
				p.store = &Store{}
			case "not-complete":
				p.completed = false
			case "wrong-independent-pin":
				pin = strings.Repeat("e", 64)
			case "changed-original-request":
				raw = append(bytes.Clone(raw), ' ')
			case "changed-installed-pin":
				p.expected.ExecutableSHA256 = strings.Repeat("e", 64)
			case "missing-registration":
				b.registration = nil
			case "changed-registration":
				b.registration = append(bytes.Clone(b.registration), ' ')
			case "mixed-acquisition":
				b.request.Installed.Generation = strings.Repeat("9", 32)
			case "tampered-source":
				p.files["app/opencpn.exe"].(*memoryFile).data[0] ^= 1
			case "missing-helper":
				delete(p.files, "installer/engine/Lifecycle.ps1")
			}
			result, err := s.MaterializeApprovedBaselineFresh(ctx, p, raw, pin)
			if result != nil || err == nil || b.reserved {
				t.Fatal("unbound authority reached reservation", err)
			}
		})
	}
}
func TestGenerationBaselineFailuresPreservePermanentEvidenceAndCloseOnce(t *testing.T) {
	for _, scenario := range []string{"reserve", "tree", "copy", "inventory", "source-changed", "cancel-after-reserve", "uncertain-publish", "cancel-after-commit"} {
		t.Run(scenario, func(t *testing.T) {
			s, p, b, _, raw := baselineViewFixture(t)
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			switch scenario {
			case "source-changed":
				b.sourceChanged = true
			case "cancel-after-reserve":
				b.onReserve = cancel
			case "uncertain-publish":
				b.publishErr = errors.Join(ErrInstalledGenerationUncertain, testFault)
			case "cancel-after-commit":
				b.onPublish = cancel
			default:
				b.faultAt = scenario
			}
			result, err := s.MaterializeApprovedBaselineFresh(ctx, p, raw, registrationDigest(raw))
			if scenario == "cancel-after-commit" {
				if err != nil || result == nil || !b.completed {
					t.Fatal("durable commit treated as cancelled", err)
				}
				return
			}
			if result != nil || err == nil {
				t.Fatal("failure accepted")
			}
			if scenario == "uncertain-publish" && !errors.Is(err, ErrInstalledGenerationUncertain) {
				t.Fatal("uncertainty lost", err)
			}
			if b.closes != 1 {
				t.Fatal("failed owner cleanup not exact", b.closes)
			}
			for name, file := range b.files {
				if file.closes != 1 {
					t.Fatal("failed copied member not closed once", name, file.closes)
				}
			}
		})
	}
}
func TestGenerationBaselineStrictPurposeDispatchAndCandidateBytesUnchanged(t *testing.T) {
	s, p, _, _, raw := baselineViewFixture(t)
	owner, err := s.MaterializeApprovedBaselineFresh(context.Background(), p, raw, registrationDigest(raw))
	if err != nil {
		t.Fatal(err)
	}
	original := owner.generation.record
	if err = validateGenerationPurpose(original); err != nil {
		t.Fatal(err)
	}
	for _, scenario := range []string{"unknown", "mixed-candidate", "no-origin", "registration-marker", "baseline-marker", "source-id", "member-digest", "empty-completed"} {
		t.Run(scenario, func(t *testing.T) {
			data, _ := json.Marshal(original)
			var record generationRecord
			json.Unmarshal(data, &record)
			switch scenario {
			case "unknown":
				record.Purpose = "generic-generation"
			case "mixed-candidate":
				record.Acquisition = acquisitionPlan()
			case "no-origin":
				record.ApprovedBaseline = nil
			case "registration-marker":
				record.ApprovedBaseline.RegistrationMarker[0] ^= 1
			case "baseline-marker":
				record.ApprovedBaseline.BaselineMarker[0] ^= 1
			case "source-id":
				record.SourceMembers["app/opencpn.exe"] = baselineTestID(999)
			case "member-digest":
				m := record.Members["app/opencpn.exe"]
				m.SHA256 = strings.Repeat("e", 64)
				record.Members["app/opencpn.exe"] = m
			case "empty-completed":
				record.Members = map[string]baselineMember{}
			}
			if validateGenerationPurpose(record) == nil {
				t.Fatal("invalid purpose lineage accepted")
			}
		})
	}
	cs, cp, c, _, _ := generationFixture(t)
	candidate, err := cs.MaterializeCandidateFresh(context.Background(), cp, c)
	if err != nil {
		t.Fatal(err)
	}
	var fields map[string]json.RawMessage
	json.Unmarshal(candidate.raw, &fields)
	if _, exists := fields["approvedBaseline"]; exists {
		t.Fatal("candidate serialization changed")
	}
	if err = validateGenerationPurpose(candidate.record); err != nil {
		t.Fatal(err)
	}
	candidate.record.ApprovedBaseline = original.ApprovedBaseline
	if validateGenerationPurpose(candidate.record) == nil {
		t.Fatal("candidate accepts operator provenance")
	}
}
