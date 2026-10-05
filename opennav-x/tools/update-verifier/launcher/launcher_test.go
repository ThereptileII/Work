package launcher

import (
	"bytes"
	"context"
	"crypto"
	"crypto/ed25519"
	"crypto/sha256"
	"encoding/binary"
	"encoding/hex"
	"encoding/json"
	"errors"
	"io"
	"os"
	"path/filepath"
	"reflect"
	"strings"
	"testing"
	"time"

	verifier "example.com/opennav-update-verifier"
	"example.com/opennav-update-verifier/releasepolicy"
	"github.com/sigstore/sigstore/pkg/signature"
	"github.com/theupdateframework/go-tuf/v2/metadata"
)

const fixtureRoot = "C:/Users/Owner/AppData/Local/OpenNavXAlpha1"
const fixtureGeneration = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
const nextGeneration = "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb"

func digest(data []byte) string { hash := sha256.Sum256(data); return hex.EncodeToString(hash[:]) }
func jsonBytes(t *testing.T, value any) []byte {
	t.Helper()
	data, err := json.Marshal(value)
	if err != nil {
		t.Fatal(err)
	}
	return data
}
func testPE() []byte {
	b := make([]byte, 128)
	copy(b, "MZ")
	binary.LittleEndian.PutUint32(b[60:], 64)
	copy(b[64:], "PE\x00\x00")
	binary.LittleEndian.PutUint16(b[68:], 0x14c)
	binary.LittleEndian.PutUint16(b[84:], 2)
	binary.LittleEndian.PutUint16(b[88:], 0x10b)
	return b
}
func testRoot(t *testing.T) []byte {
	t.Helper()
	root := metadata.Root(time.Now().Add(24 * time.Hour))
	_, private, err := ed25519.GenerateKey(nil)
	if err != nil {
		t.Fatal(err)
	}
	key, err := metadata.KeyFromPublicKey(private.Public())
	if err != nil {
		t.Fatal(err)
	}
	for _, role := range []string{"root", "timestamp", "snapshot", "targets"} {
		if err := root.Signed.AddKey(key, role); err != nil {
			t.Fatal(err)
		}
	}
	signer, err := signature.LoadSigner(private, crypto.Hash(0))
	if err != nil {
		t.Fatal(err)
	}
	if _, err := root.Sign(signer); err != nil {
		t.Fatal(err)
	}
	b, err := root.ToBytes(false)
	if err != nil {
		t.Fatal(err)
	}
	return b
}

type fakeCloser struct{ closed bool }

func (c *fakeCloser) Close() error { c.closed = true; return nil }

type fakePlatform struct {
	self             string
	files            map[string][]byte
	waits, starts    []processSpec
	held             []*fakeCloser
	onWait           func(context.Context, processSpec) (processResult, error)
	private          map[string]bool
	lockHeld         bool
	lockBusy         bool
	onLock           func()
	readsWhileLocked int
}

func (f *fakePlatform) executable() (string, error) { return f.self, nil }
func (f *fakePlatform) read(name string, limit int64) ([]byte, error) {
	if f.lockHeld && name == fixtureRoot+"/state.json" {
		f.readsWhileLocked++
	}
	b, ok := f.files[name]
	if !ok {
		return nil, os.ErrNotExist
	}
	if len(b) == 0 || int64(len(b)) > limit {
		return nil, errors.New("bounded read rejected")
	}
	return append([]byte(nil), b...), nil
}
func (f *fakePlatform) hold(name, hash string) (io.Closer, error) {
	b, ok := f.files[name]
	if !ok || digest(b) != hash {
		return nil, errors.New("hash mismatch")
	}
	c := &fakeCloser{}
	f.held = append(f.held, c)
	return c, nil
}
func (f *fakePlatform) measure(name string) (string, error) {
	b, ok := f.files[name]
	if !ok {
		return "", os.ErrNotExist
	}
	if err := validatePE32(bytes.NewReader(b), int64(len(b))); err != nil {
		return "", err
	}
	return digest(b), nil
}
func (f *fakePlatform) powershell() (string, error) {
	return "C:/Windows/System32/WindowsPowerShell/v1.0/powershell.exe", nil
}
func (f *fakePlatform) wait(ctx context.Context, spec processSpec) (processResult, error) {
	f.waits = append(f.waits, spec)
	if f.onWait != nil {
		return f.onWait(ctx, spec)
	}
	return processResult{}, nil
}
func (f *fakePlatform) start(spec processSpec) error {
	if !f.lockHeld {
		return errors.New("fake process creation without maintenance lock")
	}
	f.starts = append(f.starts, spec)
	return nil
}

type fakeLock struct{ platform *fakePlatform }

func (l fakeLock) Close() error { l.platform.lockHeld = false; return nil }
func (f *fakePlatform) lockStartup(name string) (io.Closer, error) {
	if name != fixtureRoot+"/transaction.lock" || f.lockBusy || f.lockHeld {
		return nil, errors.New("lock refused")
	}
	f.lockHeld = true
	if f.onLock != nil {
		f.onLock()
	}
	return fakeLock{f}, nil
}
func (f *fakePlatform) createMarker(name string, data []byte) error {
	if _, ok := f.files[name]; ok {
		return os.ErrExist
	}
	f.files[name] = append([]byte(nil), data...)
	return nil
}
func (f *fakePlatform) privateDirectory(name string, create bool) error {
	if f.private[name] {
		return nil
	}
	if !create {
		return os.ErrNotExist
	}
	f.private[name] = true
	return nil
}

type fakePreparation struct {
	artifactPath string
	p            releasepolicy.Policy
	closed       bool
}

func (p *fakePreparation) path() string                 { return p.artifactPath }
func (p *fakePreparation) policy() releasepolicy.Policy { return p.p }
func (p *fakePreparation) close() error                 { p.closed = true; return nil }

type launchFixture struct {
	p                *fakePlatform
	s                services
	state            installState
	owned            ownership
	trust            trustRecord
	candidate        verifier.UpdateCandidate
	prepared         *fakePreparation
	checks, prepares int
	t                *testing.T
}

func newLaunchFixture(t *testing.T) *launchFixture {
	t.Helper()
	f := &launchFixture{t: t, p: &fakePlatform{self: fixtureRoot + "/generations/" + fixtureGeneration + "/app/skager-start.exe", files: map[string][]byte{}, private: map[string]bool{}}}
	stockPath := "C:/Program Files (x86)/OpenCPN/opencpn.exe"
	stock := testPE()
	f.p.files[stockPath] = stock
	f.state = installState{Owner: owner, Schema: 1, Current: fixtureGeneration, Stock: stockRecord{Path: stockPath, SHA256: digest(stock), Version: baselineVersion, Arch: "x86"}}
	f.owned = ownership{Owner: owner, Version: "1.0.0", Commit: strings.Repeat("a", 40), PackageSHA256: strings.Repeat("c", 64), HardwarePolicy: "status-only", Health: 1}
	f.trust = trustRecord{Schema: 1, Root: testRoot(t), MetadataURL: "https://metadata.example.test", ArtifactOrigin: "https://artifacts.example.test", Channel: "beta", Allowlist: []releasepolicy.OpenCpn{{Version: baselineVersion, Arch: "x86", ExecutableSHA256: digest(stock), UpstreamCommit: baselineCommit}}}
	f.p.files[fixtureRoot+"/owner.json"] = []byte(`{"owner":"` + owner + `"}`)
	f.storeState()
	f.writeGeneration(fixtureGeneration, f.owned, true)
	f.p.files[fixtureRoot+"/known-good/"+fixtureGeneration+".receipt"] = []byte("fixture supervisor receipt")
	f.candidate = verifier.UpdateCandidate{Decision: releasepolicy.Upgrade, Consent: verifier.ConsentIdentity{Version: "2.0.0", Commit: strings.Repeat("b", 40), PolicySHA256: strings.Repeat("d", 64)}, Policy: releasepolicy.Policy{Version: "2.0.0", Commit: strings.Repeat("b", 40)}}
	f.prepared = &fakePreparation{artifactPath: fixtureRoot + "/update-artifacts/verified.exe", p: f.candidate.Policy}
	f.s = services{platform: f.p, check: func(ctx context.Context, c verifier.PrepareConfig, o releasepolicy.OpenCpn) (verifier.UpdateCandidate, error) {
		f.checks++
		deadline, ok := ctx.Deadline()
		if !ok || time.Until(deadline) > 5*time.Second {
			t.Error("check is not bounded to five seconds")
		}
		if c.Trust.MetadataURL != f.trust.MetadataURL || c.ArtifactOrigin != f.trust.ArtifactOrigin || c.StateDirectory != filepath.FromSlash(fixtureRoot+"/update-trust/beta") || o != f.trust.Allowlist[0] {
			t.Error("check did not use installed configuration and measured stock")
		}
		return f.candidate, nil
	}, prepare: func(ctx context.Context, c verifier.PrepareConfig, o releasepolicy.OpenCpn, consent verifier.ConsentIdentity) (preparation, error) {
		f.prepares++
		if consent != f.candidate.Consent {
			t.Error("prepare received changed user consent")
		}
		return f.prepared, nil
	}}
	return f
}
func (f *launchFixture) storeState() { f.p.files[fixtureRoot+"/state.json"] = jsonBytes(f.t, f.state) }
func (f *launchFixture) writeGeneration(id string, owned ownership, trust bool) {
	directory := fixtureRoot + "/generations/" + id
	owned.Files = nil
	owned.Managed = nil
	for _, relative := range []string{"app/opencpn.exe", "app/skager-start.exe", "app/skager-update-prompt.exe", "Lifecycle.ps1", "UpdateSupervisor.ps1", "UpdateTransaction.ps1"} {
		b := []byte("owned fixture " + relative + " " + id)
		f.p.files[directory+"/"+relative] = b
		record := fileRecord{Path: relative, SHA256: digest(b)}
		owned.Files = append(owned.Files, record)
		owned.Managed = append(owned.Managed, record)
	}
	if trust {
		b := jsonBytes(f.t, f.trust)
		f.p.files[directory+"/app/update-trust.json"] = b
		record := fileRecord{Path: "app/update-trust.json", SHA256: digest(b)}
		owned.Files = append(owned.Files, record)
		owned.Managed = append(owned.Managed, record)
	}
	f.p.files[directory+"/ownership.json"] = jsonBytes(f.t, owned)
}
func hasAction(spec processSpec, action string) bool {
	for i := 0; i+1 < len(spec.arguments); i++ {
		if spec.arguments[i] == "-Action" && spec.arguments[i+1] == action {
			return true
		}
	}
	return false
}
func (f *launchFixture) publishSuccessor() {
	next := f.owned
	next.Version = f.candidate.Policy.Version
	next.Commit = f.candidate.Policy.Commit
	f.writeGeneration(nextGeneration, next, true)
	f.state.Previous = fixtureGeneration
	f.state.Current = nextGeneration
	f.storeState()
	oldLayout := selectedLayout(layout{root: fixtureRoot}, fixtureGeneration)
	newLayout := selectedLayout(oldLayout, nextGeneration)
	old, err := loadGeneration(f.p, oldLayout)
	if err != nil {
		f.t.Fatal(err)
	}
	defer old.close()
	new, err := loadGeneration(f.p, newLayout)
	if err != nil {
		f.t.Fatal(err)
	}
	defer new.close()
	pending := pendingRecord{Schema: 1, Owner: "SKAGER.UpdateStartup.1", Transaction: strings.Repeat("e", 32), Candidate: identity(new), Previous: identity(old)}
	f.p.files[fixtureRoot+"/update-pending.json"] = jsonBytes(f.t, pending)
}

func TestLauncherOfflineModesAndFirstQualification(t *testing.T) {
	for _, scenario := range []string{"missing config", "bad config", "first startup", "legacy", "safe", "historical", "check offline", "current"} {
		t.Run(scenario, func(t *testing.T) {
			f := newLaunchFixture(t)
			args := []string{}
			switch scenario {
			case "missing config":
				delete(f.p.files, fixtureRoot+"/generations/"+fixtureGeneration+"/app/update-trust.json")
			case "bad config":
				f.p.files[fixtureRoot+"/generations/"+fixtureGeneration+"/app/update-trust.json"] = []byte("bad")
			case "first startup":
				delete(f.p.files, fixtureRoot+"/known-good/"+fixtureGeneration+".receipt")
			case "legacy":
				args = []string{"--legacy"}
			case "safe":
				args = []string{"--safe-mode"}
			case "historical":
				f.owned.Health = 0
				f.writeGeneration(fixtureGeneration, f.owned, true)
			case "check offline":
				f.s.check = func(context.Context, verifier.PrepareConfig, releasepolicy.OpenCpn) (verifier.UpdateCandidate, error) {
					f.checks++
					return verifier.UpdateCandidate{}, errors.New("offline")
				}
			case "current":
				f.candidate.Decision = releasepolicy.Current
			}
			if err := run(context.Background(), args, f.s); err != nil {
				t.Fatal(err)
			}
			if f.prepares != 0 {
				t.Fatal("offline/bypass mode prepared update")
			}
			if scenario != "check offline" && scenario != "current" && f.checks != 0 {
				t.Fatal("offline/bypass mode reached network")
			}
			if scenario != "first startup" {
				if len(f.p.starts) != 1 || len(f.p.waits) != 0 {
					t.Fatal("explicit mode did not start existing app directly")
				}
			} else if len(f.p.waits) != 1 || !hasAction(f.p.waits[0], "QualifyCurrent") {
				t.Fatal("ordinary startup did not qualify current")
			}
		})
	}
}

func TestLauncherLaterNeverPreparesAndUsesBoundedPipe(t *testing.T) {
	for _, exit := range []int{0, 1, 11} {
		t.Run(string(rune('A'+exit)), func(t *testing.T) {
			f := newLaunchFixture(t)
			f.p.onWait = func(ctx context.Context, spec processSpec) (processResult, error) {
				if strings.HasSuffix(spec.executable, "skager-update-prompt.exe") {
					expected, _ := promptInput(f.candidate.Consent)
					if !bytes.Equal(spec.input, expected) || len(spec.arguments) != 0 || len(spec.input) > 256 {
						t.Fatal("prompt identity was not the fixed pipe protocol")
					}
					return processResult{code: exit}, nil
				}
				return processResult{}, nil
			}
			if err := run(context.Background(), nil, f.s); err != nil {
				t.Fatal(err)
			}
			if f.prepares != 0 || len(f.p.waits) != 1 || len(f.p.starts) != 1 {
				t.Fatal("Later/error changed startup behavior")
			}
		})
	}
}

func TestLauncherConfirmedInstallerAndExactSupervisorHandoff(t *testing.T) {
	f := newLaunchFixture(t)
	f.p.onWait = func(ctx context.Context, spec processSpec) (processResult, error) {
		if strings.HasSuffix(spec.executable, "skager-update-prompt.exe") {
			return processResult{code: 10}, nil
		}
		if spec.executable == f.prepared.path() {
			if f.prepared.closed || !reflect.DeepEqual(spec.arguments, []string{"/S", "/ACTION=Update", "/SUPERVISED=1"}) || spec.limit != 15*time.Minute {
				t.Fatal("installer lost custody or fixed arguments")
			}
			f.publishSuccessor()
			return processResult{}, nil
		}
		if hasAction(spec, "LaunchPending") {
			if spec.arguments[5] != fixtureRoot+"/generations/"+nextGeneration+"/UpdateSupervisor.ps1" || spec.arguments[len(spec.arguments)-1] != strings.Repeat("e", 32) {
				t.Fatal("supervisor used wrong generation or transaction")
			}
			delete(f.p.files, fixtureRoot+"/update-pending.json")
			return processResult{}, nil
		}
		t.Fatal("unexpected process")
		return processResult{}, nil
	}
	if err := run(context.Background(), nil, f.s); err != nil {
		t.Fatal(err)
	}
	if f.checks != 1 || f.prepares != 1 || !f.prepared.closed || len(f.p.waits) != 3 || len(f.p.starts) != 0 {
		t.Fatal("unexpected update process lifecycle")
	}
}

func TestLauncherRejectsPointerTamperAndInstallerFailure(t *testing.T) {
	for _, scenario := range []string{"stale shortcut", "helper tamper", "stock tamper", "changed after consent", "prepare failure", "installer failure", "no pending", "wrong pending", "timed out"} {
		t.Run(scenario, func(t *testing.T) {
			f := newLaunchFixture(t)
			wantError := true
			if scenario == "stale shortcut" {
				f.state.Current = nextGeneration
				f.storeState()
			}
			if scenario == "helper tamper" {
				f.p.files[fixtureRoot+"/generations/"+fixtureGeneration+"/Lifecycle.ps1"] = []byte("tampered")
			}
			if scenario == "stock tamper" {
				f.p.files[f.state.Stock.Path] = testPE()
				f.p.files[f.state.Stock.Path][127] = 1
				wantError = false
			}
			if scenario == "prepare failure" {
				f.s.prepare = func(context.Context, verifier.PrepareConfig, releasepolicy.OpenCpn, verifier.ConsentIdentity) (preparation, error) {
					f.prepares++
					return nil, errors.New("changed policy")
				}
				wantError = false
			}
			f.p.onWait = func(ctx context.Context, spec processSpec) (processResult, error) {
				if strings.HasSuffix(spec.executable, "skager-update-prompt.exe") {
					if scenario == "changed after consent" {
						f.state.Current = nextGeneration
						f.storeState()
					}
					return processResult{code: 10}, nil
				}
				if spec.executable == f.prepared.path() {
					switch scenario {
					case "installer failure":
						return processResult{code: 1}, nil
					case "timed out":
						done := make(chan struct{})
						close(done)
						return processResult{timedOut: true, done: done}, errors.New("timeout")
					case "no pending":
						f.publishSuccessor()
						delete(f.p.files, fixtureRoot+"/update-pending.json")
					case "wrong pending":
						f.publishSuccessor()
						f.p.files[fixtureRoot+"/update-pending.json"] = []byte(`{}`)
					}
					return processResult{}, nil
				}
				return processResult{}, nil
			}
			err := run(context.Background(), nil, f.s)
			if (err != nil) != wantError {
				t.Fatalf("result=%v expected error=%v", err, wantError)
			}
			if scenario == "stale shortcut" || scenario == "helper tamper" {
				if len(f.p.waits) != 0 || f.checks != 0 {
					t.Fatal("untrusted generation caused external effects")
				}
			}
			if scenario == "stock tamper" && f.checks != 0 {
				t.Fatal("unmeasured stock reached network")
			}
			for _, spec := range f.p.waits {
				if hasAction(spec, "LaunchPending") {
					t.Fatal("failed installation attempted candidate startup")
				}
			}
			if wantError && len(f.p.starts) != 0 {
				t.Fatal("failed installation launched unqualified application")
			}
		})
	}
}

func TestLauncherPendingRecoveryPrecedesNetwork(t *testing.T) {
	for _, corrupt := range []string{"app/opencpn.exe", "UpdateSupervisor.ps1", "Lifecycle.ps1", "state committed before shortcut"} {
		t.Run(corrupt, func(t *testing.T) {
			f := newLaunchFixture(t)
			f.publishSuccessor()
			if corrupt != "state committed before shortcut" {
				f.p.self = fixtureRoot + "/generations/" + nextGeneration + "/app/skager-start.exe"
				f.p.files[fixtureRoot+"/generations/"+nextGeneration+"/"+corrupt] = []byte("corrupt candidate cannot block recovery")
			}
			f.p.onWait = func(ctx context.Context, spec processSpec) (processResult, error) {
				if !hasAction(spec, "RecoverPending") || spec.arguments[5] != fixtureRoot+"/generations/"+fixtureGeneration+"/UpdateSupervisor.ps1" {
					t.Fatal("recovery did not use fully verified previous supervisor")
				}
				delete(f.p.files, fixtureRoot+"/update-pending.json")
				f.state.Current = fixtureGeneration
				f.state.Previous = ""
				f.storeState()
				return processResult{}, nil
			}
			if err := run(context.Background(), nil, f.s); err != nil {
				t.Fatal(err)
			}
			if f.checks != 0 || f.prepares != 0 || len(f.p.waits) != 1 || len(f.p.starts) != 1 || f.p.starts[0].executable != fixtureRoot+"/generations/"+fixtureGeneration+"/app/opencpn.exe" {
				t.Fatal("pending recovery failed to restore ordinary startup before networking")
			}
		})
	}
}

func TestProvisioningOnceMarkerSurvivesFailureAndNeverResets(t *testing.T) {
	f := newLaunchFixture(t)
	l, _ := deriveLayout(f.p.self)
	config := preparationConfig(l, f.owned, f.trust)
	data := jsonBytes(t, f.trust)
	initializations, validations := 0, 0
	initialize := func(string, verifier.TrustConfig) error {
		initializations++
		if _, ok := f.p.files[fixtureRoot+"/update-trust-initialized-beta"]; !ok {
			t.Fatal("state initialized before once marker")
		}
		return errors.New("simulated interruption")
	}
	validate := func(string, verifier.TrustConfig) error { validations++; return errors.New("state missing") }
	if err := provisionTrust(f.p, fixtureRoot, config, data, initialize, validate); err == nil {
		t.Fatal("interrupted initialization accepted")
	}
	if err := provisionTrust(f.p, fixtureRoot, config, data, initialize, validate); err == nil {
		t.Fatal("missing retained state accepted")
	}
	if initializations != 1 || validations != 1 {
		t.Fatal("retry reset retained trust")
	}
	if err := provisionTrust(f.p, fixtureRoot, config, append(data, ' '), initialize, validate); err == nil {
		t.Fatal("changed provisioned configuration accepted")
	}
}

func TestPreviousLauncherRecoveryExceptionCannotAuthorizeUnrelatedEntry(t *testing.T) {
	for _, scenario := range []string{"unrelated generation", "running image changed", "previous identity changed"} {
		t.Run(scenario, func(t *testing.T) {
			f := newLaunchFixture(t)
			f.publishSuccessor()
			switch scenario {
			case "unrelated generation":
				id := strings.Repeat("c", 32)
				f.writeGeneration(id, f.owned, true)
				f.p.self = fixtureRoot + "/generations/" + id + "/app/skager-start.exe"
			case "running image changed":
				f.p.files[f.p.self] = []byte("tampered previous launcher")
			case "previous identity changed":
				changed := f.owned
				changed.Commit = strings.Repeat("c", 40)
				f.writeGeneration(fixtureGeneration, changed, true)
			}
			if err := run(context.Background(), nil, f.s); err == nil {
				t.Fatal("unrelated or modified previous entry gained recovery authority")
			}
			if len(f.p.waits) != 0 || len(f.p.starts) != 0 || f.checks != 0 {
				t.Fatal("rejected recovery entry caused an external effect")
			}
		})
	}
}

func TestExplicitProvisioningWithoutInstalledConfigIsInert(t *testing.T) {
	f := newLaunchFixture(t)
	delete(f.p.files, fixtureRoot+"/generations/"+fixtureGeneration+"/app/update-trust.json")
	if err := run(context.Background(), []string{"--initialize-trust"}, f.s); err != nil {
		t.Fatal(err)
	}
	if f.checks != 0 || f.prepares != 0 || len(f.p.waits) != 0 || len(f.p.starts) != 0 || len(f.p.private) != 0 {
		t.Fatal("absent configuration provisioning caused external effects")
	}
}

func TestOrdinaryStartupSerializesFinalPointerReadAndProcessCreation(t *testing.T) {
	for _, scenario := range []string{"available", "installer owns lock", "pointer changed before lock"} {
		t.Run(scenario, func(t *testing.T) {
			f := newLaunchFixture(t)
			f.candidate.Decision = releasepolicy.Current
			if scenario == "installer owns lock" {
				f.p.lockBusy = true
			}
			if scenario == "pointer changed before lock" {
				f.p.onLock = func() { f.state.Current = nextGeneration; f.storeState() }
			}
			err := run(context.Background(), nil, f.s)
			if scenario == "available" {
				if err != nil || len(f.p.starts) != 1 || f.p.readsWhileLocked != 1 {
					t.Fatalf("startup not serialized: starts=%d reads=%d err=%v", len(f.p.starts), f.p.readsWhileLocked, err)
				}
			} else if err == nil || len(f.p.starts) != 0 {
				t.Fatal("concurrent maintenance allowed app launch")
			}
			if f.p.lockHeld {
				t.Fatal("startup leaked maintenance lock")
			}
		})
	}
}
