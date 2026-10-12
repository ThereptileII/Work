package brokerenroll

import (
	"bytes"
	"context"
	"crypto"
	"crypto/ed25519"
	"encoding/json"
	"errors"
	"io"
	"reflect"
	"strings"
	"testing"
	"time"

	verifier "example.com/opennav-update-verifier"
	"example.com/opennav-update-verifier/releasepolicy"
	"example.com/opennav-update-verifier/repository"
	"github.com/sigstore/sigstore/pkg/signature"
	"github.com/theupdateframework/go-tuf/v2/metadata"
)

type testOwner struct {
	closed   bool
	onClose  func()
	closeErr error
}

func (o *testOwner) Close() error {
	if o.closed {
		panic("owner closed twice")
	}
	o.closed = true
	if o.onClose != nil {
		o.onClose()
	}
	return o.closeErr
}

type testBroker struct {
	*bytes.Reader
	*testOwner
}
type testFiles struct {
	*testOwner
	checks   int
	checkErr error
}

func (f *testFiles) ImagePath() string { return "inert-not-a-native-path" }
func (f *testFiles) Check() error {
	if f.closed {
		panic("closed machine owner used")
	}
	f.checks++
	return f.checkErr
}

type enrollmentFixture struct {
	req         Request
	raw         []byte
	policy      releasepolicy.BrokerBootstrapPolicy
	brokerBytes []byte
	op          operations
	events      []string
	inputs      *testOwner
	broker      *testBroker
	files       *testFiles
}

func newEnrollmentFixture(t *testing.T) *enrollmentFixture {
	t.Helper()
	root := metadata.Root(time.Now().Add(24 * time.Hour))
	root.Signed.ConsistentSnapshot = true
	var rootPrivate ed25519.PrivateKey
	for _, role := range []string{"root", "timestamp", "snapshot", "targets"} {
		_, private, err := ed25519.GenerateKey(nil)
		if err != nil {
			t.Fatal(err)
		}
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
	trustBytes, err := json.Marshal(repository.Trust{Schema: 1, BootstrapRoot: rootBytes, MetadataURL: "https://private.invalid/metadata", ArtifactOrigin: "https://private.invalid", Channel: "beta", LocalCompatibilityAllowlist: []releasepolicy.OpenCpn{{Version: "5.12.4", Arch: "x86", ExecutableSHA256: "7c6547562cca7954671eaab72833ca9d788710fd9808b6a699b6dc823852ae0c", UpstreamCommit: "37fd0cddb7334fe489e9f18aa163977a9c5c84f7"}}})
	if err != nil {
		t.Fatal(err)
	}
	f := &enrollmentFixture{brokerBytes: []byte("INERT EXACT BROKER TEST BYTES")}
	commit := strings.Repeat("a", 40)
	artifact := func(name string, body []byte) releasepolicy.Artifact {
		p := "artifacts/" + commit + "/" + name
		return releasepolicy.Artifact{Path: p, URL: "https://private.invalid/" + p, SHA256: digest(body), Bytes: int64(len(body))}
	}
	f.policy = releasepolicy.BrokerBootstrapPolicy{Schema: 1, Product: "skager", Purpose: releasepolicy.BrokerBootstrapPurpose, Channel: "beta", Version: "0.4.2-beta.1", Commit: commit, Architecture: "x86", ServiceName: serviceName, ServiceAccount: serviceAccount, CodeLocation: "ProgramFilesX86/SKAGER/UpdateBroker", DataLocation: "ProgramData/SKAGER/UpdateBroker", TrustConfigSHA256: digest(trustBytes), Broker: artifact("skager-update-broker.exe", f.brokerBytes), Provisioner: artifact("skager-update-provisioner.exe", []byte("INERT PROVISIONER")), CorrespondingSource: artifact("source.zip", []byte("INERT SOURCE")), Notices: artifact("notices.md", []byte("INERT NOTICES"))}
	f.raw, err = json.Marshal(f.policy)
	if err != nil {
		t.Fatal(err)
	}
	f.req = Request{TrustConfiguration: trustBytes, BootstrapCacheDir: "inert-protected-cache", ArtifactDirectory: "inert-protected-artifacts", PolicySHA256: digest(f.raw), ProvisionerSHA256: f.policy.Provisioner.SHA256, TrustConfigSHA256: digest(trustBytes)}
	f.inputs = &testOwner{onClose: func() { f.events = append(f.events, "inputs.close") }}
	f.broker = &testBroker{bytes.NewReader(f.brokerBytes), &testOwner{onClose: func() { f.events = append(f.events, "broker.close") }}}
	f.files = &testFiles{testOwner: &testOwner{onClose: func() { f.events = append(f.events, "files.close") }}}
	f.op = operations{
		holdDirectories: func(cache, artifacts string) (io.Closer, error) {
			f.events = append(f.events, "hold")
			if cache != f.req.BootstrapCacheDir || artifacts != f.req.ArtifactDirectory {
				t.Fatal("directories changed")
			}
			return f.inputs, nil
		},
		verify: func(req verifier.BrokerBootstrapRequest) (releasepolicy.BrokerBootstrapPolicy, []byte, error) {
			f.events = append(f.events, "verify")
			if f.inputs.closed || req.ApprovedPolicySHA256 != f.req.PolicySHA256 || req.TrustedProvisionerSHA256 != f.req.ProvisionerSHA256 || req.TrustConfigSHA256 != f.req.TrustConfigSHA256 || req.Channel != "beta" || req.CacheDir != f.req.BootstrapCacheDir || req.ArtifactOrigin != "https://private.invalid" {
				t.Fatal("independent trust or custody lost")
			}
			return f.policy, bytes.Clone(f.raw), nil
		},
		acquire: func(ctx context.Context, a releasepolicy.Artifact, origin, dir string) (acquiredBroker, error) {
			f.events = append(f.events, "acquire")
			if a != f.policy.Broker || origin != "https://private.invalid" || dir != f.req.ArtifactDirectory || f.inputs.closed {
				t.Fatal("acquisition selection changed")
			}
			return f.broker, nil
		},
		create: func(ctx context.Context, r io.Reader, a releasepolicy.Artifact) (enrollmentFiles, error) {
			f.events = append(f.events, "create")
			b, e := io.ReadAll(r)
			if e != nil || !bytes.Equal(b, f.brokerBytes) || a != f.policy.Broker || f.broker.closed || f.inputs.closed {
				t.Fatal("retained exact broker source lost")
			}
			return f.files, nil
		},
		initialize: func(ctx context.Context, files enrollmentFiles) error {
			f.events = append(f.events, "initialize")
			if files != f.files || f.files.closed || f.broker.closed || f.inputs.closed {
				t.Fatal("SCM initialization lost owners")
			}
			return nil
		},
	}
	return f
}

func TestEnrollmentRetainsOriginalOwnersThroughCompletion(t *testing.T) {
	f := newEnrollmentFixture(t)
	if err := run(context.Background(), f.req, f.op); err != nil {
		t.Fatal(err)
	}
	want := []string{"hold", "verify", "acquire", "create", "initialize", "files.close", "broker.close", "inputs.close"}
	if !reflect.DeepEqual(f.events, want) || f.files.checks != 2 {
		t.Fatalf("events=%v checks=%d", f.events, f.files.checks)
	}
}

func TestEnrollmentBadPublicInputsHaveNoEffects(t *testing.T) {
	for _, change := range []struct {
		name   string
		mutate func(*Request)
	}{
		{"missing policy pin", func(r *Request) { r.PolicySHA256 = "" }},
		{"uppercase pin", func(r *Request) { r.ProvisionerSHA256 = strings.Repeat("A", 64) }},
		{"wrong trust hash", func(r *Request) { r.TrustConfigSHA256 = strings.Repeat("0", 64) }},
		{"missing durable cache", func(r *Request) { r.BootstrapCacheDir = "" }},
		{"missing acquisition directory", func(r *Request) { r.ArtifactDirectory = "" }},
		{"malformed approved trust", func(r *Request) {
			r.TrustConfiguration = []byte(`{"schema":1,"schema":1}`)
			r.TrustConfigSHA256 = digest(r.TrustConfiguration)
		}},
		{"oversized trust", func(r *Request) { r.TrustConfiguration = bytes.Repeat([]byte(" "), maxTrustConfigurationBytes+1) }},
	} {
		t.Run(change.name, func(t *testing.T) {
			f := newEnrollmentFixture(t)
			change.mutate(&f.req)
			if err := run(context.Background(), f.req, f.op); err == nil || len(f.events) != 0 {
				t.Fatalf("bad input effects=%v err=%v", f.events, err)
			}
		})
	}
}

func TestEnrollmentRefusesBeforeMachineMutation(t *testing.T) {
	for _, stage := range []string{"directory custody", "signed target", "target byte substitution", "wrong broker size", "acquisition", "cancel during verification", "cancel after acquisition"} {
		t.Run(stage, func(t *testing.T) {
			f := newEnrollmentFixture(t)
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			switch stage {
			case "directory custody":
				f.op.holdDirectories = func(string, string) (io.Closer, error) { return nil, errors.New("private path") }
			case "signed target":
				f.op.verify = func(verifier.BrokerBootstrapRequest) (releasepolicy.BrokerBootstrapPolicy, []byte, error) {
					return f.policy, nil, errors.New("private endpoint")
				}
			case "target byte substitution":
				f.raw = append(f.raw, ' ')
			case "wrong broker size":
				f.policy.Broker.Bytes = maxBrokerBytes + 1
				f.raw, _ = json.Marshal(f.policy)
				f.req.PolicySHA256 = digest(f.raw)
			case "acquisition":
				f.op.acquire = func(context.Context, releasepolicy.Artifact, string, string) (acquiredBroker, error) {
					return nil, verifier.ErrArtifactCleanup
				}
			case "cancel during verification":
				old := f.op.verify
				f.op.verify = func(r verifier.BrokerBootstrapRequest) (releasepolicy.BrokerBootstrapPolicy, []byte, error) {
					p, b, e := old(r)
					cancel()
					return p, b, e
				}
			case "cancel after acquisition":
				old := f.op.acquire
				f.op.acquire = func(c context.Context, a releasepolicy.Artifact, o, d string) (acquiredBroker, error) {
					b, e := old(c, a, o, d)
					cancel()
					return b, e
				}
			}
			err := run(ctx, f.req, f.op)
			if err == nil || f.files.checks != 0 || strings.Contains(err.Error(), "private") {
				t.Fatalf("effects=%v err=%v", f.events, err)
			}
			for _, event := range f.events {
				if event == "create" || event == "initialize" {
					t.Fatalf("invalid input mutated machine: %v", f.events)
				}
			}
			if stage == "acquisition" && !errors.Is(err, ErrCleanup) {
				t.Fatal("acquisition cleanup uncertainty lost")
			}
		})
	}
}

func TestEnrollmentPartialAndCleanupFailuresNeverBecomeSuccess(t *testing.T) {
	for _, stage := range []string{"create partial", "file check", "service", "cancel during service", "file close", "artifact close", "input close"} {
		t.Run(stage, func(t *testing.T) {
			f := newEnrollmentFixture(t)
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			switch stage {
			case "create partial":
				f.op.create = func(context.Context, io.Reader, releasepolicy.Artifact) (enrollmentFiles, error) {
					return f.files, errors.New("private partial path")
				}
			case "file check":
				f.files.checkErr = errors.New("private identity")
			case "service":
				f.op.initialize = func(context.Context, enrollmentFiles) error { return errors.New("private service information") }
			case "cancel during service":
				f.op.initialize = func(context.Context, enrollmentFiles) error { cancel(); return nil }
			case "file close":
				f.files.closeErr = errors.New("private close")
			case "artifact close":
				f.broker.closeErr = errors.New("private close")
			case "input close":
				f.inputs.closeErr = errors.New("private close")
			}
			err := run(ctx, f.req, f.op)
			if err == nil || !f.inputs.closed || !f.broker.closed || !f.files.closed || strings.Contains(err.Error(), "private") {
				t.Fatalf("err=%v owner closes=%v/%v/%v", err, f.inputs.closed, f.broker.closed, f.files.closed)
			}
			if strings.Contains(stage, "close") && !errors.Is(err, ErrCleanup) {
				t.Fatal("owner cleanup uncertainty lost")
			}
		})
	}
}

func TestEnrollmentPreservesNativeUncertainCleanupClassification(t *testing.T) {
	for _, stage := range []string{"input preparation", "partial native creation", "SCM owner", "initial custody check", "final custody check"} {
		t.Run(stage, func(t *testing.T) {
			f := newEnrollmentFixture(t)
			nativeErr := errors.Join(ErrCleanup, errors.New("private native detail"))
			switch stage {
			case "input preparation":
				f.op.holdDirectories = func(string, string) (io.Closer, error) { return f.inputs, nativeErr }
			case "partial native creation":
				f.op.create = func(context.Context, io.Reader, releasepolicy.Artifact) (enrollmentFiles, error) {
					return f.files, nativeErr
				}
			case "SCM owner":
				f.op.initialize = func(context.Context, enrollmentFiles) error { return nativeErr }
			case "initial custody check":
				f.files.checkErr = nativeErr
			case "final custody check":
				f.op.initialize = func(context.Context, enrollmentFiles) error { f.files.checkErr = nativeErr; return nil }
			}
			err := run(context.Background(), f.req, f.op)
			if !errors.Is(err, ErrCleanup) || strings.Contains(err.Error(), "private") {
				t.Fatalf("uncertainty lost/leaked: %v", err)
			}
			if !f.inputs.closed {
				t.Fatal("original input owner leaked")
			}
		})
	}
}
