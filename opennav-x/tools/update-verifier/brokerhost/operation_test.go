package brokerhost

import (
	"context"
	"encoding/json"
	"errors"
	"io"
	"reflect"
	"strings"
	"testing"

	verifier "example.com/opennav-update-verifier"
	"example.com/opennav-update-verifier/brokerpipe"
	"example.com/opennav-update-verifier/releasepolicy"
)

const testSID = "S-1-5-21-100-200-300-1001"

func testRegistration() []byte {
	r := registration{Schema: 1, ClientSID: testSID}
	r.Installed.Version = "0.2.0-beta.1"
	r.Installed.Commit = strings.Repeat("a", 40)
	r.Stock.Path = `C:\Program Files (x86)\OpenCPN\opencpn.exe`
	r.Stock.Version = stockBaseline.Version
	r.Stock.Arch = stockBaseline.Arch
	r.Stock.ExecutableSHA256 = stockBaseline.ExecutableSHA256
	r.Stock.UpstreamCommit = stockBaseline.UpstreamCommit
	r.Trust.BootstrapRoot = json.RawMessage(`{"signed":{},"signatures":[]}`)
	r.Trust.MetadataURL = "https://updates.invalid/metadata/"
	r.Trust.ArtifactOrigin = "https://updates.invalid"
	r.Trust.Channel = "beta"
	b, _ := json.Marshal(r)
	return b
}
func testRequest() brokerpipe.Request {
	return brokerpipe.Request{Schema: 1, Operation: "prepare-and-stage", RequestID: strings.Repeat("b", 32),
		Consent: verifier.ConsentIdentity{Version: "0.2.0-beta.2", Commit: strings.Repeat("c", 40), PolicySHA256: strings.Repeat("d", 64)}}
}
func testRecovery(c verifier.ConsentIdentity) verifier.RecoveryIdentity {
	h := strings.Repeat("e", 64)
	return verifier.RecoveryIdentity{Version: c.Version, Commit: c.Commit, ExecutableSHA256: h, ProductBuildSHA256: h,
		InventorySHA256: h, LifecycleManifestAvailable: true, LifecycleManifestSHA256: h, DeclaredPayloadSHA256: h}
}

// Inert seams test operation sequencing only. They confer no signed-source,
// Windows principal, ACL, IPC, filesystem or destination-custody acceptance.
type stageFixture struct {
	events         []string
	registration   []byte
	fail           string
	reserved       map[string]bool
	receipt        []byte
	entered        chan struct{}
	release        chan struct{}
	validations    int
	failValidation int
}

func (f *stageFixture) record(event string) error {
	f.events = append(f.events, event)
	if f.fail == event {
		return errors.New("private sensitive path must not cross pipe")
	}
	return nil
}
func (f *stageFixture) Validate() error {
	f.validations++
	if f.failValidation == f.validations {
		_ = f.record("validate")
		return errors.New("protected namespace changed")
	}
	return f.record("validate")
}
func (f *stageFixture) ReadRegistration(sid string) ([]byte, error) {
	if sid != testSID {
		return nil, errors.New("unexpected SID")
	}
	return f.registration, f.record("registration")
}
func (f *stageFixture) VerifierDirectories(sid, channel string) (string, string, error) {
	if sid != testSID || channel != "beta" {
		return "", "", errors.New("unexpected directories")
	}
	return "protected-state", "protected-artifacts", f.record("directories")
}
func (f *stageFixture) begin(id, sid string) (candidate, error) {
	if err := f.record("reserve"); err != nil {
		return nil, err
	}
	if sid != testSID || f.reserved[id] {
		return nil, errors.New("reservation refused")
	}
	f.reserved[id] = true
	return fakeStage{f}, nil
}

type fakeStage struct{ f *stageFixture }

func (s fakeStage) WriteFile(context.Context, string, io.Reader, int64, string) error {
	return errors.New("not used by inert operation fixture")
}
func (s fakeStage) Seal(_ context.Context, b []byte) error {
	s.f.receipt = append([]byte(nil), b...)
	return s.f.record("seal")
}
func (s fakeStage) Close() error { return s.f.record("stage-close") }

type fakeStock struct{ f *stageFixture }

func (s fakeStock) validate() error { return s.f.record("stock-validate") }
func (s fakeStock) Close() error    { return s.f.record("stock-close") }

type fakeAcquisition struct {
	f        *stageFixture
	identity verifier.RecoveryIdentity
}

func (a fakeAcquisition) StageRecovery(ctx context.Context, _ verifier.RecoverySink) (verifier.RecoveryIdentity, error) {
	if a.f.entered != nil {
		close(a.f.entered)
		select {
		case <-a.f.release:
		case <-ctx.Done():
			return verifier.RecoveryIdentity{}, ctx.Err()
		}
	}
	return a.identity, a.f.record("extract")
}
func (a fakeAcquisition) Close() error { return a.f.record("artifact-close") }
func fixtureEngine(f *stageFixture) *engine {
	return &engine{store: f,
		observe: func(_ context.Context, r registration) (releasepolicy.OpenCpn, stockOwner, error) {
			if r.Stock.Path != `C:\Program Files (x86)\OpenCPN\opencpn.exe` {
				return releasepolicy.OpenCpn{}, nil, errors.New("unexpected stock")
			}
			if err := f.record("observe"); err != nil {
				return releasepolicy.OpenCpn{}, nil, err
			}
			return stockBaseline, fakeStock{f}, nil
		},
		prepare: func(_ context.Context, c verifier.PrepareConfig, o releasepolicy.OpenCpn, consent verifier.ConsentIdentity) (acquisition, error) {
			if c.StateDirectory != "protected-state" || c.ArtifactDirectory != "protected-artifacts" || c.ArtifactOrigin != "https://updates.invalid" ||
				c.Installed.Version != "0.2.0-beta.1" || !reflect.DeepEqual(c.OpenCpnAllowlist, []releasepolicy.OpenCpn{stockBaseline}) || o != stockBaseline {
				return nil, errors.New("configuration not from protected registration")
			}
			if err := f.record("prepare"); err != nil {
				return nil, err
			}
			identity := testRecovery(consent)
			if f.fail == "identity" {
				identity.Commit = strings.Repeat("f", 40)
			}
			return fakeAcquisition{f, identity}, nil
		}}
}
func newFixture() *stageFixture {
	return &stageFixture{registration: testRegistration(), reserved: map[string]bool{}}
}
func testPeer() brokerpipe.Peer { return brokerpipe.Peer{SID: testSID, SessionID: 1, ProcessID: 77} }

func TestStagingOperationOrdersCleanupBeforeReceipt(t *testing.T) {
	f := newFixture()
	result, err := fixtureEngine(f).handle(context.Background(), testPeer(), testRequest())
	if err != nil || result.State != stagedState || result.StageID != testRequest().RequestID {
		t.Fatalf("result=%+v err=%v", result, err)
	}
	want := []string{"validate", "registration", "directories", "observe", "reserve", "validate", "prepare", "validate", "extract", "stock-validate", "validate", "artifact-close", "validate", "seal", "stage-close", "stock-close", "validate"}
	if !reflect.DeepEqual(f.events, want) {
		t.Fatalf("events %v", f.events)
	}
	var receipt stagingReceipt
	if json.Unmarshal(f.receipt, &receipt) != nil || receipt.State != stagedState || receipt.ClientSID != testSID || receipt.Consent != testRequest().Consent || receipt.Recovery != testRecovery(testRequest().Consent) {
		t.Fatal("receipt does not bind exact peer, consent and inner identity")
	}
	// The receipt/result explicitly remain staging evidence, never install or
	// launch success; no callback in this operation can execute candidate code.
}

func TestStagingOperationFaultsWithholdSuccess(t *testing.T) {
	for _, fault := range []string{"validate", "registration", "directories", "observe", "reserve", "prepare", "extract", "identity", "stock-validate", "artifact-close", "seal", "stage-close", "stock-close"} {
		t.Run(fault, func(t *testing.T) {
			f := newFixture()
			f.fail = fault
			result, err := fixtureEngine(f).handle(context.Background(), testPeer(), testRequest())
			if result != (brokerpipe.Result{}) || err != errStageRefused {
				t.Fatalf("unexpected partial success or leak: %+v %v", result, err)
			}
			if fault != "seal" && fault != "stage-close" && fault != "stock-close" && len(f.receipt) != 0 {
				t.Fatal("failed acquisition/extraction/cleanup published receipt")
			}
			if contains(f.events, "prepare") && !contains(f.events, "stage-close") {
				t.Fatal("reserved stage owner not closed")
			}
			if contains(f.events, "extract") && !contains(f.events, "artifact-close") {
				t.Fatal("acquired artifact owner not closed")
			}
		})
	}
}

func TestStagingOperationRejectsReplayBeforeAcquisition(t *testing.T) {
	f := newFixture()
	e := fixtureEngine(f)
	if _, err := e.handle(context.Background(), testPeer(), testRequest()); err != nil {
		t.Fatal(err)
	}
	f.events = nil
	if result, err := e.handle(context.Background(), testPeer(), testRequest()); err == nil || result != (brokerpipe.Result{}) {
		t.Fatal("replayed ID accepted")
	}
	if contains(f.events, "prepare") || contains(f.events, "extract") || contains(f.events, "seal") {
		t.Fatal("replay reached acquisition")
	}
}

func TestStagingOperationNamespaceFailureNeverReturnsSuccess(t *testing.T) {
	for _, at := range []int{3, 5, 6} {
		f := newFixture()
		f.failValidation = at
		result, err := fixtureEngine(f).handle(context.Background(), testPeer(), testRequest())
		if result != (brokerpipe.Result{}) || err != errStageRefused {
			t.Fatalf("validation %d accepted: %+v %v", at, result, err)
		}
		if at < 6 && len(f.receipt) != 0 {
			t.Fatalf("validation %d published receipt before namespace check", at)
		}
	}
}

func TestStagingOperationRejectsConcurrentAndCancelledWork(t *testing.T) {
	f := newFixture()
	f.entered = make(chan struct{})
	f.release = make(chan struct{})
	e := fixtureEngine(f)
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	done := make(chan error, 1)
	go func() {
		result, err := e.handle(ctx, testPeer(), testRequest())
		if result != (brokerpipe.Result{}) {
			done <- errors.New("cancelled stage accepted")
			return
		}
		done <- err
	}()
	<-f.entered
	if result, err := e.handle(context.Background(), testPeer(), testRequest()); result != (brokerpipe.Result{}) || err == nil {
		t.Fatal("concurrent operation accepted")
	}
	cancel()
	if err := <-done; err != errStageRefused {
		t.Fatalf("cancel did not refuse: %v", err)
	}
	if len(f.receipt) != 0 || !contains(f.events, "artifact-close") || !contains(f.events, "stage-close") || !contains(f.events, "stock-close") {
		t.Fatal("cancellation cleanup or receipt incorrect")
	}
}

func TestStagingOperationRejectsUntrustedRequestAndRegistration(t *testing.T) {
	for _, change := range []func(*brokerpipe.Request, *brokerpipe.Peer, *stageFixture){
		func(r *brokerpipe.Request, _ *brokerpipe.Peer, _ *stageFixture) { r.Operation = "execute" },
		func(r *brokerpipe.Request, _ *brokerpipe.Peer, _ *stageFixture) { r.RequestID = "../candidate" },
		func(r *brokerpipe.Request, _ *brokerpipe.Peer, _ *stageFixture) { r.Consent.PolicySHA256 = "" },
		func(_ *brokerpipe.Request, p *brokerpipe.Peer, _ *stageFixture) { p.ProcessID = 0 },
		func(_ *brokerpipe.Request, _ *brokerpipe.Peer, f *stageFixture) {
			f.registration = []byte(`{"schema":1}`)
		},
	} {
		f := newFixture()
		r, p := testRequest(), testPeer()
		change(&r, &p, f)
		if result, err := fixtureEngine(f).handle(context.Background(), p, r); result != (brokerpipe.Result{}) || err == nil {
			t.Fatal("untrusted operation accepted")
		}
		if contains(f.events, "reserve") || contains(f.events, "prepare") {
			t.Fatal("invalid request reserved or acquired")
		}
	}
}
func contains(events []string, want string) bool {
	for _, e := range events {
		if e == want {
			return true
		}
	}
	return false
}
