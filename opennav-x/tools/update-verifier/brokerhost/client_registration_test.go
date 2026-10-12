package brokerhost

import (
	"bytes"
	"context"
	"crypto"
	"crypto/ed25519"
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"errors"
	"reflect"
	"regexp"
	"strings"
	"testing"
	"time"

	verifier "example.com/opennav-update-verifier"
	"example.com/opennav-update-verifier/brokerregistration"
	"example.com/opennav-update-verifier/brokerstore"
	"example.com/opennav-update-verifier/releasepolicy"
	"example.com/opennav-update-verifier/repository"
	"github.com/sigstore/sigstore/pkg/signature"
	"github.com/theupdateframework/go-tuf/v2/metadata"
)

var clientRegistrationTestNow = time.Date(2026, 10, 12, 12, 0, 0, 0, time.UTC)
var fixtureTrustPin = regexp.MustCompile(`"trustConfigSha256"\s*:\s*"[a-f0-9]{64}"`)

func clientRequestDigest(raw []byte) string {
	sum := sha256.Sum256(raw)
	return hex.EncodeToString(sum[:])
}
func clientFixtureJSON(value any) []byte {
	raw, err := json.Marshal(value)
	if err != nil {
		panic(err)
	}
	return raw
}

// Inert deterministic test keys and a fixed expiry keep tests independent of
// network/files. The production clock still verifies expiry at every read.
// These fixture approvals and evidence hashes qualify no real installation.
func testApprovedClientRequest() []byte {
	root := metadata.Root(time.Date(2036, 1, 1, 0, 0, 0, 0, time.UTC))
	root.Signed.ConsistentSnapshot = true
	var privateRoot ed25519.PrivateKey
	for _, role := range []string{"root", "timestamp", "snapshot", "targets"} {
		seed := sha256.Sum256([]byte("INERT HOST CLIENT REGISTRATION TEST ONLY " + role))
		private := ed25519.NewKeyFromSeed(seed[:])
		key, err := metadata.KeyFromPublicKey(private.Public())
		if err != nil {
			panic(err)
		}
		if err = root.Signed.AddKey(key, role); err != nil {
			panic(err)
		}
		if role == "root" {
			privateRoot = private
		}
	}
	signer, err := signature.LoadSigner(privateRoot, crypto.Hash(0))
	if err != nil {
		panic(err)
	}
	if _, err = root.Sign(signer); err != nil {
		panic(err)
	}
	rootBytes, err := root.ToBytes(false)
	if err != nil {
		panic(err)
	}
	trust := clientFixtureJSON(repository.Trust{Schema: 1, BootstrapRoot: rootBytes, MetadataURL: "https://updates.invalid/metadata/", ArtifactOrigin: "https://updates.invalid", Channel: "beta", LocalCompatibilityAllowlist: []releasepolicy.OpenCpn{stockBaseline}})
	return clientFixtureJSON(brokerregistration.Request{Schema: 1, Purpose: brokerregistration.Purpose, ClientSID: testSID, Installed: brokerregistration.Installed{Version: "0.2.0-beta.1", Commit: strings.Repeat("a", 40), Generation: strings.Repeat("b", 32), PackageSHA256: strings.Repeat("c", 64), ProductBuildSHA256: strings.Repeat("d", 64), ExecutableSHA256: strings.Repeat("e", 64), InventorySHA256: strings.Repeat("f", 64)}, Stock: brokerregistration.Stock{Path: `C:\Program Files (x86)\OpenCPN\opencpn.exe`, Version: stockBaseline.Version, Arch: stockBaseline.Arch, ExecutableSHA256: stockBaseline.ExecutableSHA256, UpstreamCommit: stockBaseline.UpstreamCommit}, TrustConfiguration: trust, TrustConfigSHA256: clientRequestDigest(trust)})
}
func testProtectedClientRegistration() []byte {
	raw := testApprovedClientRequest()
	return clientFixtureJSON(protectedRegistration{Schema: 2, ClientSID: testSID, ApprovedRequestSHA256: clientRequestDigest(raw), ApprovedRequest: raw})
}
func mutateProtectedClientRequest(raw []byte, mutate func([]byte) []byte) []byte {
	var envelope protectedRegistration
	if err := json.Unmarshal(raw, &envelope); err != nil {
		panic(err)
	}
	envelope.ApprovedRequest = mutate(envelope.ApprovedRequest)
	// Keep independent fixture pins consistent so schema/trust tests reach the
	// actual rejection boundary, without normalizing duplicate original keys.
	var fields map[string]json.RawMessage
	if json.Unmarshal(envelope.ApprovedRequest, &fields) == nil {
		trust := fields["trustConfiguration"]
		envelope.ApprovedRequest = fixtureTrustPin.ReplaceAll(envelope.ApprovedRequest, []byte(`"trustConfigSha256":"`+clientRequestDigest(trust)+`"`))
	}
	envelope.ApprovedRequestSHA256 = clientRequestDigest(envelope.ApprovedRequest)
	return clientFixtureJSON(envelope)
}

type clientRegistrationFixture struct {
	events                       []string
	fail                         string
	checks                       int
	nilStore, nilOwner, nilStock bool
	cancelAt                     string
	cancel                       context.CancelFunc
	completed                    []byte
	completeErr                  error
	original                     []byte
	onOpen                       func()
}

func (f *clientRegistrationFixture) record(event string) error {
	f.events = append(f.events, event)
	if event == f.cancelAt && f.cancel != nil {
		f.cancel()
	}
	if f.fail == event {
		return errors.New("inert registration fault: " + event)
	}
	return nil
}
func (f *clientRegistrationFixture) Validate() error {
	f.checks++
	if err := f.record("check"); err != nil {
		return err
	}
	if f.fail == "check-"+string(rune('0'+f.checks)) {
		return errors.New("inert namespace fault")
	}
	return nil
}
func (f *clientRegistrationFixture) beginRegistration(_ context.Context, sid, channel, pin string) (clientRegistrationOwner, error) {
	if sid != testSID || channel != "beta" || pin != clientRequestDigest(f.original) {
		return nil, errors.New("reservation did not use exact approved identity")
	}
	err := f.record("begin")
	if f.nilOwner {
		return nil, err
	}
	return clientOwnerFixture{f}, err
}
func (f *clientRegistrationFixture) Close() error { return f.record("store-close") }

type clientOwnerFixture struct{ f *clientRegistrationFixture }

func (o clientOwnerFixture) VerifierDirectories() (string, string, error) {
	return "absent-broker-state", "fresh-broker-artifacts", o.f.record("directories")
}
func (o clientOwnerFixture) Complete(ctx context.Context, raw []byte) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	if _, err := parseRegistrationAt(raw, testSID, clientRegistrationTestNow); err != nil {
		return err
	}
	o.f.completed = bytes.Clone(raw)
	return errors.Join(o.f.record("complete"), o.f.completeErr)
}
func (o clientOwnerFixture) Close() error { return o.f.record("owner-close") }

type clientStockFixture struct{ f *clientRegistrationFixture }

func (o clientStockFixture) validate() error { return o.f.record("stock-validate") }
func (o clientStockFixture) Close() error    { return o.f.record("stock-close") }
func (f *clientRegistrationFixture) operations() clientRegistrationOperations {
	return clientRegistrationOperations{
		open: func() (clientRegistrationStore, error) {
			if f.onOpen != nil {
				f.onOpen()
			}
			err := f.record("open")
			if f.nilStore {
				return nil, err
			}
			return f, err
		},
		observe: func(_ context.Context, r registration) (releasepolicy.OpenCpn, stockOwner, error) {
			if r.Stock.Path != `C:\Program Files (x86)\OpenCPN\opencpn.exe` || r.Installed.Commit != strings.Repeat("a", 40) {
				return releasepolicy.OpenCpn{}, nil, errors.New("unapproved stock/installed identity")
			}
			err := f.record("observe")
			if f.nilStock {
				return stockBaseline, nil, err
			}
			return stockBaseline, clientStockFixture{f}, err
		},
		initialize: func(state string, trust verifier.TrustConfig) error {
			if state != "absent-broker-state" || trust.Channel != "beta" || trust.MetadataURL != "https://updates.invalid/metadata/" {
				return errors.New("trust not from approved original request")
			}
			return f.record("initialize")
		},
		validate: func(state string, trust verifier.TrustConfig) error {
			if state != "absent-broker-state" || trust.Channel != "beta" {
				return errors.New("wrong validated trust")
			}
			return f.record("validate-trust")
		},
	}
}
func newClientRegistrationFixture() *clientRegistrationFixture {
	return &clientRegistrationFixture{original: testApprovedClientRequest()}
}
func (f *clientRegistrationFixture) run(ctx context.Context) error {
	return registerApprovedClient(ctx, f.original, clientRequestDigest(f.original), clientRegistrationTestNow, f.operations())
}

func TestClientRegistrationInitializesAndValidatesBeforeCompletion(t *testing.T) {
	f := newClientRegistrationFixture()
	if err := f.run(context.Background()); err != nil {
		t.Fatal(err)
	}
	expected := []string{"open", "check", "observe", "check", "begin", "check", "directories", "check", "initialize", "check", "validate-trust", "check", "stock-validate", "check", "complete", "owner-close", "stock-close", "check", "store-close"}
	if !reflect.DeepEqual(f.events, expected) {
		t.Fatalf("incorrect custody/initialization order: %v", f.events)
	}
	var envelope protectedRegistration
	if err := json.Unmarshal(f.completed, &envelope); err != nil {
		t.Fatal(err)
	}
	if !bytes.Equal(envelope.ApprovedRequest, f.original) || envelope.ApprovedRequestSHA256 != clientRequestDigest(f.original) {
		t.Fatal("registration did not retain exact evidence")
	}
}

func TestClientRegistrationEveryFaultRefusesAndDrainsOwnedHandles(t *testing.T) {
	for _, fault := range []string{"open", "check", "observe", "begin", "directories", "initialize", "validate-trust", "stock-validate", "complete", "owner-close", "stock-close", "store-close", "check-2", "check-3", "check-4", "check-5", "check-6", "check-7", "check-8"} {
		t.Run(fault, func(t *testing.T) {
			f := newClientRegistrationFixture()
			f.fail = fault
			if err := f.run(context.Background()); err == nil {
				t.Fatal("fault returned successful registration")
			}
			if contains(f.events, "open") && !contains(f.events, "store-close") {
				t.Fatal("store owner not drained")
			}
			if contains(f.events, "observe") && !contains(f.events, "stock-close") {
				t.Fatal("stock owner not drained")
			}
			if contains(f.events, "begin") && !contains(f.events, "owner-close") {
				t.Fatal("registration owner not drained")
			}
			if fault != "complete" && fault != "owner-close" && fault != "stock-close" && fault != "store-close" && fault != "check-8" && len(f.completed) != 0 {
				t.Fatal("failed preparation published completed evidence")
			}
			if fault == "owner-close" || fault == "stock-close" || fault == "store-close" || fault == "check-8" {
				// A valid completed record may still be read later. This outcome
				// never authorizes a second creation attempt or automatic adoption.
				if _, err := parseRegistrationAt(f.completed, testSID, clientRegistrationTestNow); err != nil {
					t.Fatal(err)
				}
			}
			for _, closeEvent := range []string{"owner-close", "stock-close", "store-close"} {
				count := 0
				for _, event := range f.events {
					if event == closeEvent {
						count++
					}
				}
				if count > 1 {
					t.Fatalf("owner closed twice: %s", closeEvent)
				}
			}
		})
	}
}

func TestClientRegistrationWithholdsCompletionOnCancellation(t *testing.T) {
	for _, at := range []string{"open", "observe", "begin", "directories", "initialize", "validate-trust", "stock-validate"} {
		t.Run(at, func(t *testing.T) {
			f := newClientRegistrationFixture()
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			f.cancel, f.cancelAt = cancel, at
			if err := f.run(ctx); !errors.Is(err, context.Canceled) {
				t.Fatalf("cancellation accepted/lost: %v", err)
			}
			if len(f.completed) != 0 {
				t.Fatal("cancelled preparation published evidence")
			}
		})
	}
}

func TestClientRegistrationCompletedHandoffSurvivesLateCancellation(t *testing.T) {
	for _, at := range []string{"complete", "owner-close", "stock-close", "store-close"} {
		t.Run(at, func(t *testing.T) {
			f := newClientRegistrationFixture()
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			f.cancel, f.cancelAt = cancel, at
			if err := f.run(ctx); err != nil {
				t.Fatalf("late cancellation reclassified committed registration: %v", err)
			}
			if _, err := parseRegistrationAt(f.completed, testSID, clientRegistrationTestNow); err != nil {
				t.Fatal(err)
			}
		})
	}
}

func TestClientRegistrationClassifiesPublicationAndCleanupUncertainty(t *testing.T) {
	for _, at := range []string{"publication", "owner-close", "stock-close", "store-close", "check-8"} {
		t.Run(at, func(t *testing.T) {
			f := newClientRegistrationFixture()
			if at == "publication" {
				f.completeErr = errors.Join(brokerstore.ErrRegistrationUncertain, errors.New("inert publication IO failure"))
			} else {
				f.fail = at
			}
			err := f.run(context.Background())
			if !errors.Is(err, ErrClientRegistrationUncertain) {
				t.Fatalf("durable result uncertainty lost: %v", err)
			}
			if at == "publication" && !errors.Is(err, brokerstore.ErrRegistrationUncertain) {
				t.Fatal("native uncertainty identity discarded")
			}
			if _, err := parseRegistrationAt(f.completed, testSID, clientRegistrationTestNow); err != nil {
				t.Fatal(err)
			}
		})
	}
}

func TestClientRegistrationRejectsMissingOwnerAndApprovalBeforeCreation(t *testing.T) {
	for _, kind := range []string{"store", "stock", "registration"} {
		t.Run(kind, func(t *testing.T) {
			f := newClientRegistrationFixture()
			f.nilStore = kind == "store"
			f.nilStock = kind == "stock"
			f.nilOwner = kind == "registration"
			if err := f.run(context.Background()); err == nil {
				t.Fatal("missing native owner accepted")
			}
			if len(f.completed) != 0 {
				t.Fatal("missing owner published completion")
			}
		})
	}
	f := newClientRegistrationFixture()
	if err := registerApprovedClient(context.Background(), f.original, strings.Repeat("0", 64), clientRegistrationTestNow, f.operations()); err == nil || len(f.events) != 0 {
		t.Fatal("unapproved bytes reached native operations")
	}
	oversized := append(bytes.Repeat([]byte(" "), 800<<10), f.original...)
	if err := registerApprovedClient(context.Background(), oversized, clientRequestDigest(oversized), clientRegistrationTestNow, f.operations()); err == nil || len(f.events) != 0 {
		t.Fatal("oversized protected envelope reached mutation")
	}
}

func TestClientRegistrationCopiesApprovalBeforeNativeOperations(t *testing.T) {
	f := newClientRegistrationFixture()
	input := bytes.Clone(f.original)
	f.onOpen = func() { input[0] = '!' }
	if err := registerApprovedClient(context.Background(), input, clientRequestDigest(input), clientRegistrationTestNow, f.operations()); err != nil {
		t.Fatal(err)
	}
	var envelope protectedRegistration
	_ = json.Unmarshal(f.completed, &envelope)
	if !bytes.Equal(envelope.ApprovedRequest, f.original) {
		t.Fatal("caller mutation changed approved evidence")
	}
}

func TestClientRegistrationJoinsOperationAndCleanupFailure(t *testing.T) {
	f := newClientRegistrationFixture()
	op := f.operations()
	primary, cleanup := errors.New("inert initialization refusal"), errors.New("inert store close uncertainty")
	op.initialize = func(string, verifier.TrustConfig) error { return primary }
	op.open = func() (clientRegistrationStore, error) {
		return &closeErrorClientStore{clientRegistrationFixture: f, err: cleanup}, nil
	}
	err := registerApprovedClient(context.Background(), f.original, clientRequestDigest(f.original), clientRegistrationTestNow, op)
	if !errors.Is(err, primary) || !errors.Is(err, cleanup) {
		t.Fatal("operation or cleanup failure discarded")
	}
	if len(f.completed) != 0 {
		t.Fatal("failed trust initialization completed registration")
	}
}

type closeErrorClientStore struct {
	*clientRegistrationFixture
	err error
}

func (s *closeErrorClientStore) Close() error { return s.err }
