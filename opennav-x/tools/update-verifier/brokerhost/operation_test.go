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
	"example.com/opennav-update-verifier/brokerregistration"
	"example.com/opennav-update-verifier/brokerstore"
	"example.com/opennav-update-verifier/releasepolicy"
)

const testSID = "S-1-5-21-100-200-300-1001"

func testRegistration() []byte {
	return testProtectedClientRegistration()
}
func testRequest() brokerpipe.Request {
	return brokerpipe.Request{Schema: 1, Operation: "prepare-and-stage", RequestID: strings.Repeat("9", 32),
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
	events           []string
	registration     []byte
	fail             string
	reserved         map[string]bool
	receipt          []byte
	entered          chan struct{}
	release          chan struct{}
	validations      int
	failValidation   int
	nilStage         bool
	cancelAt         string
	cancel           context.CancelFunc
	changeCompletion func(*brokerstore.GenerationCompletion)
}

func (f *stageFixture) record(event string) error {
	f.events = append(f.events, event)
	if f.cancelAt == event && f.cancel != nil {
		f.cancel()
	}
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
func (f *stageFixture) openBaseline(_ context.Context, r registration) (baselineOwner, error) {
	if r.ClientSID != testSID || r.Installed.Generation != strings.Repeat("b", 32) || r.Installed.InventorySHA256 != strings.Repeat("f", 64) || !hashPattern.MatchString(r.ApprovedRequestSHA256) {
		return nil, errors.New("unapproved baseline identity")
	}
	err := f.record("baseline-open")
	if f.fail == "nil-baseline" {
		return nil, nil
	}
	return fakeBaseline{f}, err
}

type fakeBaseline struct{ f *stageFixture }

func (b fakeBaseline) Validate() error { return b.f.record("baseline-validate") }
func (b fakeBaseline) Close() error    { return b.f.record("baseline-close") }
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
	if f.fail == "reserve-owner-error" {
		return fakeStage{f}, f.record("reserve-owner-error")
	}
	if f.nilStage {
		return nil, nil
	}
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

func testGenerationCompletion() brokerstore.GenerationCompletion {
	h := strings.Repeat("a", 64)
	return brokerstore.GenerationCompletion{Schema: 1, ClientSID: testSID,
		Generation: testRequest().RequestID, PreviousGeneration: strings.Repeat("b", 32),
		PreviousCompletionSHA256: h, CompletionSHA256: h, PolicySHA256: testRequest().Consent.PolicySHA256,
		RecoverySHA256: h, InstallerSHA256: h, Identity: testRecovery(testRequest().Consent)}
}
func (f *stageFixture) materialize(_ context.Context, baseline baselineOwner, stage candidate) (generationOwner, error) {
	b, ok := baseline.(fakeBaseline)
	s, stageOK := stage.(fakeStage)
	if !ok || !stageOK || b.f != f || s.f != f || !contains(f.events, "artifact-close") ||
		contains(f.events, "baseline-close") || contains(f.events, "stage-close") || contains(f.events, "seal") {
		return nil, errors.New("materialization lost original source custody or ordering")
	}
	err := f.record("materialize")
	if f.fail == "nil-generation" {
		return nil, nil
	}
	return fakeGeneration{f}, err
}

type fakeGeneration struct{ f *stageFixture }

func (g fakeGeneration) Validate() error { return g.f.record("generation-validate") }
func (g fakeGeneration) Completion() (brokerstore.GenerationCompletion, error) {
	c := testGenerationCompletion()
	if g.f.changeCompletion != nil {
		g.f.changeCompletion(&c)
	}
	return c, g.f.record("generation-completion")
}
func (g fakeGeneration) Close() error { return g.f.record("generation-close") }

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
		prepare: func(_ context.Context, c verifier.PrepareConfig, o releasepolicy.OpenCpn, consent verifier.ConsentIdentity, stage candidate) (acquisition, error) {
			reserved, ok := stage.(fakeStage)
			if !ok || reserved.f != f {
				return nil, errors.New("preparation did not receive reserved candidate")
			}
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
	want := []string{"validate", "registration", "baseline-open", "baseline-validate", "directories", "observe", "reserve", "validate", "prepare", "validate", "extract", "baseline-validate", "stock-validate", "validate", "artifact-close", "validate", "baseline-validate", "materialize", "generation-validate", "generation-completion", "baseline-validate", "stock-validate", "validate", "generation-close", "baseline-close", "seal", "stage-close", "stock-close", "validate"}
	if !reflect.DeepEqual(f.events, want) {
		t.Fatalf("events %v", f.events)
	}
	var receipt stagingReceipt
	if json.Unmarshal(f.receipt, &receipt) != nil || receipt.State != stagedState || receipt.ClientSID != testSID || receipt.Consent != testRequest().Consent || receipt.Recovery != testRecovery(testRequest().Consent) {
		t.Fatal("receipt does not bind exact peer, consent and inner identity")
	}
	var original protectedRegistration
	if json.Unmarshal(f.registration, &original) != nil {
		t.Fatal("bad fixture original")
	}
	request, err := brokerregistration.ParseRetainedApprovedRequest(original.ApprovedRequest, original.ApprovedRequestSHA256)
	if err != nil || receipt.Schema != 3 || receipt.Installed != request.Installed || receipt.ApprovedRequestSHA256 != request.RequestSHA256 || receipt.Materialized != testGenerationCompletion() {
		t.Fatal("receipt lost measured baseline approval identity")
	}
	// The receipt/result explicitly remain staging evidence, never install or
	// launch success; no callback in this operation can execute candidate code.
}

func TestStagingOperationFaultsWithholdSuccess(t *testing.T) {
	for _, fault := range []string{"validate", "registration", "baseline-open", "baseline-validate", "baseline-close", "nil-baseline", "directories", "observe", "reserve", "reserve-owner-error", "prepare", "extract", "identity", "stock-validate", "artifact-close", "materialize", "nil-generation", "generation-validate", "generation-completion", "generation-close", "seal", "stage-close", "stock-close"} {
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
			if fault == "reserve-owner-error" && countEvent(f.events, "stage-close") != 1 {
				t.Fatal("candidate returned with an error was not drained exactly once")
			}
			if contains(f.events, "extract") && !contains(f.events, "artifact-close") {
				t.Fatal("acquired artifact owner not closed")
			}
			if contains(f.events, "materialize") && fault != "nil-generation" && !contains(f.events, "generation-close") {
				t.Fatal("returned materialization owner not closed")
			}
			if contains(f.events, "generation-close") && countEvent(f.events, "generation-close") != 1 {
				t.Fatal("generation cleanup retried")
			}
			if contains([]string{"validate", "registration", "baseline-open", "baseline-validate", "nil-baseline", "directories", "observe", "reserve", "reserve-owner-error", "prepare", "extract", "identity", "stock-validate", "artifact-close"}, fault) && contains(f.events, "materialize") {
				t.Fatal("prior failure reached materialization")
			}
		})
	}
}

func TestMaterializationCompletionMustBindOriginalOperation(t *testing.T) {
	changes := map[string]func(*brokerstore.GenerationCompletion){
		"schema":              func(c *brokerstore.GenerationCompletion) { c.Schema++ },
		"client":              func(c *brokerstore.GenerationCompletion) { c.ClientSID += "0" },
		"generation":          func(c *brokerstore.GenerationCompletion) { c.Generation = strings.Repeat("a", 32) },
		"previous":            func(c *brokerstore.GenerationCompletion) { c.PreviousGeneration = strings.Repeat("a", 32) },
		"policy":              func(c *brokerstore.GenerationCompletion) { c.PolicySHA256 = strings.Repeat("a", 64) },
		"identity":            func(c *brokerstore.GenerationCompletion) { c.Identity.Commit = strings.Repeat("a", 40) },
		"previous completion": func(c *brokerstore.GenerationCompletion) { c.PreviousCompletionSHA256 = "" },
		"completion":          func(c *brokerstore.GenerationCompletion) { c.CompletionSHA256 = "" },
		"recovery":            func(c *brokerstore.GenerationCompletion) { c.RecoverySHA256 = "" },
		"installer":           func(c *brokerstore.GenerationCompletion) { c.InstallerSHA256 = "" },
	}
	for name, change := range changes {
		t.Run(name, func(t *testing.T) {
			f := newFixture()
			f.changeCompletion = change
			result, err := fixtureEngine(f).handle(context.Background(), testPeer(), testRequest())
			if result != (brokerpipe.Result{}) || err != errStageRefused || len(f.receipt) != 0 {
				t.Fatalf("mismatched completion published success: %+v %v", result, err)
			}
			for _, event := range []string{"generation-close", "baseline-close", "stage-close", "stock-close"} {
				if countEvent(f.events, event) != 1 {
					t.Fatalf("owner not drained exactly once: %s %v", event, f.events)
				}
			}
		})
	}
}

func TestMaterializationCancellationWithholdsReceiptAndDrainsOwners(t *testing.T) {
	for _, at := range []string{"materialize", "generation-validate", "generation-completion", "generation-close", "baseline-close"} {
		t.Run(at, func(t *testing.T) {
			f := newFixture()
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			f.cancelAt, f.cancel = at, cancel
			result, err := fixtureEngine(f).handle(ctx, testPeer(), testRequest())
			if result != (brokerpipe.Result{}) || err != errStageRefused || len(f.receipt) != 0 {
				t.Fatalf("cancelled materialization published success: %+v %v", result, err)
			}
			for _, event := range []string{"artifact-close", "generation-close", "baseline-close", "stage-close", "stock-close"} {
				if countEvent(f.events, event) != 1 {
					t.Fatalf("owner not drained exactly once: %s %v", event, f.events)
				}
			}
		})
	}
}

func countEvent(events []string, want string) int {
	n := 0
	for _, event := range events {
		if event == want {
			n++
		}
	}
	return n
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

func TestStagingOperationRequiresReservedCandidate(t *testing.T) {
	f := newFixture()
	f.nilStage = true
	result, err := fixtureEngine(f).handle(context.Background(), testPeer(), testRequest())
	if result != (brokerpipe.Result{}) || err != errStageRefused {
		t.Fatalf("missing candidate accepted: %+v %v", result, err)
	}
	if contains(f.events, "prepare") || contains(f.events, "extract") || contains(f.events, "seal") {
		t.Fatal("missing reserved owner reached acquisition or published a receipt")
	}
}

func TestStagingOperationNamespaceFailureNeverReturnsSuccess(t *testing.T) {
	for _, at := range []int{3, 5, 6, 7} {
		f := newFixture()
		f.failValidation = at
		result, err := fixtureEngine(f).handle(context.Background(), testPeer(), testRequest())
		if result != (brokerpipe.Result{}) || err != errStageRefused {
			t.Fatalf("validation %d accepted: %+v %v", at, result, err)
		}
		if at < 7 && len(f.receipt) != 0 {
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
