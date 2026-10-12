package brokerenroll

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"io"
	"reflect"
	"runtime"
	"strings"
	"testing"

	verifier "example.com/opennav-update-verifier"
	"example.com/opennav-update-verifier/brokerregistration"
	"example.com/opennav-update-verifier/releasepolicy"
)

const inertClientSID = "S-1-5-21-11-22-33-1001"

func approvedClientFixture(t *testing.T, trust []byte) []byte {
	t.Helper()
	baseline := brokerregistration.StockBaseline()
	installed := brokerregistration.Installed{Version: "0.4.2-beta.1", Commit: strings.Repeat("a", 40), Generation: strings.Repeat("b", 32), PackageSHA256: strings.Repeat("c", 64), ProductBuildSHA256: strings.Repeat("d", 64), ExecutableSHA256: strings.Repeat("e", 64), InventorySHA256: strings.Repeat("f", 64)}
	raw, err := json.Marshal(brokerregistration.Request{Schema: 1, Purpose: brokerregistration.Purpose, ClientSID: inertClientSID, Installed: installed, Stock: brokerregistration.Stock{Path: `C:\inert\opencpn.exe`, Version: baseline.Version, Arch: baseline.Arch, ExecutableSHA256: baseline.ExecutableSHA256, UpstreamCommit: baseline.UpstreamCommit}, TrustConfiguration: trust, TrustConfigSHA256: digest(trust)})
	if err != nil {
		t.Fatal(err)
	}
	return raw
}

type clientFilesFixture struct {
	*testFiles
	raw               []byte
	sid, sha          string
	publishes         int
	fault             error
	hook              func()
	baseline          []byte
	baselinePublishes int
	baselineFault     error
	baselineHook      func()
}

func (f *clientFilesFixture) publishClientBaseline(ctx context.Context, source ApprovedClientBaseline, sid string) error {
	f.baselinePublishes++
	if f.closed || ctx.Err() != nil || f.publishes != 1 || f.baselinePublishes != 1 || f.sid != sid {
		panic("lost fresh baseline publication order or custody")
	}
	var copy bytes.Buffer
	if _, err := copyClientBaseline(ctx, &copy, source); err != nil {
		return err
	}
	f.baseline = copy.Bytes()
	if f.baselineHook != nil {
		f.baselineHook()
	}
	return f.baselineFault
}

func (f *clientFilesFixture) publishClientRequest(ctx context.Context, raw []byte, sid, sha string) error {
	f.publishes++
	if f.closed || ctx.Err() != nil || f.publishes != 1 {
		panic("lost fresh publication custody")
	}
	f.raw, f.sid, f.sha = bytes.Clone(raw), sid, sha
	if f.hook != nil {
		f.hook()
	}
	return f.fault
}
func (f *clientFilesFixture) clientRegistration() (string, string) { return f.sid, f.sha }

func TestEnrollmentApprovedClientRequestRetainsExactOriginal(t *testing.T) {
	f := newEnrollmentFixture(t)
	f.req.ClientRegistrationRequest = approvedClientFixture(t, f.req.TrustConfiguration)
	f.req.ApprovedClientRequestSHA256 = digest(f.req.ClientRegistrationRequest)
	baseline := clientBaselineOperations(t, f)
	original := bytes.Clone(f.req.ClientRegistrationRequest)
	client := &clientFilesFixture{testFiles: f.files}
	f.op.validateClient = func(sid string) error {
		if sid != inertClientSID || len(f.events) != 0 {
			t.Fatal("lookup before original approval or after effects")
		}
		return nil
	}
	create := f.op.create
	f.op.create = func(ctx context.Context, reader io.Reader, a releasepolicy.Artifact) (enrollmentFiles, error) {
		if _, err := create(ctx, reader, a); err != nil {
			return nil, err
		}
		return client, nil
	}
	verify := f.op.verify
	f.op.verify = func(r verifier.BrokerBootstrapRequest) (releasepolicy.BrokerBootstrapPolicy, []byte, error) {
		p, raw, err := verify(r)
		// Caller bytes may change after the authority boundary: later publication
		// must retain the separately approved original copy, not the alias.
		f.req.ClientRegistrationRequest[0] = '!'
		return p, raw, err
	}
	f.op.initialize = func(ctx context.Context, files enrollmentFiles) error {
		if files != client || client.closed || !bytes.Equal(client.raw, original) || client.sid != inertClientSID || client.sha != digest(original) || client.publishes != 1 || client.baselinePublishes != 1 || baseline.closed || !bytes.Equal(client.baseline, baseline.Bytes()) {
			t.Fatal("SCM lost independently approved original bytes or owner")
		}
		return nil
	}
	if err := run(context.Background(), f.req, f.op); err != nil {
		t.Fatal(err)
	}
	if !client.closed || !f.broker.closed || !f.inputs.closed || !baseline.closed {
		t.Fatal("owners not drained")
	}
}

func TestEnrollmentClientApprovalRefusesBeforeEffects(t *testing.T) {
	for _, fault := range []string{"missing-hash", "missing-bytes", "bad-hash", "uppercase-hash", "malformed", "duplicate", "different-trust", "different-trust-spelling", "actual-account", "missing-lookup"} {
		t.Run(fault, func(t *testing.T) {
			f := newEnrollmentFixture(t)
			f.req.ClientRegistrationRequest = approvedClientFixture(t, f.req.TrustConfiguration)
			f.req.ApprovedClientRequestSHA256 = digest(f.req.ClientRegistrationRequest)
			clientBaselineOperations(t, f)
			f.op.validateClient = func(string) error { return nil }
			switch fault {
			case "missing-hash":
				f.req.ApprovedClientRequestSHA256 = ""
			case "missing-bytes":
				f.req.ClientRegistrationRequest = nil
			case "bad-hash":
				f.req.ApprovedClientRequestSHA256 = strings.Repeat("0", 64)
			case "uppercase-hash":
				f.req.ApprovedClientRequestSHA256 = strings.Repeat("A", 64)
			case "malformed":
				f.req.ClientRegistrationRequest = []byte(`{`)
				f.req.ApprovedClientRequestSHA256 = digest(f.req.ClientRegistrationRequest)
			case "duplicate":
				f.req.ClientRegistrationRequest = bytes.Replace(f.req.ClientRegistrationRequest, []byte(`"schema":1`), []byte(`"schema":1,"schema":1`), 1)
				f.req.ApprovedClientRequestSHA256 = digest(f.req.ClientRegistrationRequest)
			case "different-trust":
				other := newEnrollmentFixture(t)
				f.req.ClientRegistrationRequest = approvedClientFixture(t, other.req.TrustConfiguration)
				f.req.ApprovedClientRequestSHA256 = digest(f.req.ClientRegistrationRequest)
			case "different-trust-spelling":
				f.req.TrustConfiguration = append([]byte(" "), f.req.TrustConfiguration...)
				f.req.TrustConfigSHA256 = digest(f.req.TrustConfiguration)
			case "actual-account":
				f.op.validateClient = func(string) error { return errors.New("unresolved") }
			case "missing-lookup":
				f.op.validateClient = nil
			}
			if err := run(context.Background(), f.req, f.op); err == nil || len(f.events) != 0 {
				t.Fatalf("fault=%s err=%v effects=%v", fault, err, f.events)
			}
		})
	}
}

func TestEnrollmentClientPublicationFailureWithholdsSCM(t *testing.T) {
	for _, fault := range []string{"missing-capability", "publication", "cleanup", "binding", "custody", "cancellation"} {
		t.Run(fault, func(t *testing.T) {
			f := newEnrollmentFixture(t)
			f.req.ClientRegistrationRequest = approvedClientFixture(t, f.req.TrustConfiguration)
			f.req.ApprovedClientRequestSHA256 = digest(f.req.ClientRegistrationRequest)
			clientBaselineOperations(t, f)
			f.op.validateClient = func(string) error { return nil }
			client := &clientFilesFixture{testFiles: f.files}
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			switch fault {
			case "publication":
				client.fault = errors.New("inert fault")
			case "cleanup":
				client.fault = ErrCleanup
			case "binding":
				client.hook = func() { client.sha = strings.Repeat("0", 64) }
			case "custody":
				client.hook = func() { f.files.checkErr = errors.New("inert fault") }
			case "cancellation":
				client.hook = cancel
			}
			if fault != "missing-capability" {
				f.op.create = func(context.Context, io.Reader, releasepolicy.Artifact) (enrollmentFiles, error) { return client, nil }
			}
			starts := 0
			f.op.initialize = func(context.Context, enrollmentFiles) error { starts++; return nil }
			err := run(ctx, f.req, f.op)
			if err == nil || starts != 0 || !client.closed || !f.inputs.closed || !f.broker.closed {
				t.Fatalf("fault=%s err=%v starts=%d", fault, err, starts)
			}
			if fault == "cleanup" && !errors.Is(err, ErrCleanup) {
				t.Fatal("cleanup uncertainty lost")
			}
		})
	}
}

type clientServiceFilesFixture struct {
	*serviceFilesFixture
	sid, sha string
}

func (*clientServiceFilesFixture) publishClientRequest(context.Context, []byte, string, string) error {
	panic("SCM must not publish")
}
func (*clientServiceFilesFixture) publishClientBaseline(context.Context, ApprovedClientBaseline, string) error {
	panic("SCM must not publish baseline")
}
func (f *clientServiceFilesFixture) clientRegistration() (string, string) { return f.sid, f.sha }

type clientServiceFixture struct {
	*serviceFixture
	sid, sha string
}

func (b *clientServiceFixture) startClientRegistration(sid, sha string) error {
	b.sid, b.sha = sid, sha
	return b.start("register-client")
}

func TestClientServiceStartsExactStoppedHandoffAfterVerification(t *testing.T) {
	f := &clientServiceFilesFixture{serviceFilesFixture: &serviceFilesFixture{image: "held"}, sid: inertClientSID, sha: strings.Repeat("a", 64)}
	b := &clientServiceFixture{serviceFixture: &serviceFixture{}}
	if err := initializeServiceWith(context.Background(), f, func() (serviceBackend, error) { return b, nil }, inertServicePoll); err != nil {
		t.Fatal(err)
	}
	if b.sid != f.sid || b.sha != f.sha || b.closes != 1 || !reflect.DeepEqual(b.events, []string{"create", "initialize-store", "verify-store", "register-client", "close"}) {
		t.Fatalf("lost exact stopped handoff: %+v events=%v", b, b.events)
	}
}

func TestClientServiceFaultsNeverRetryOrServe(t *testing.T) {
	for _, fault := range []string{"bad-sid", "bad-hash", "missing-backend", "initialize-store", "verify-store", "binding-change", "file-custody", "cancel-after-verify", "unexpected-running", "register-client", "registration-running", "registration-exit", "close"} {
		t.Run(fault, func(t *testing.T) {
			f := &clientServiceFilesFixture{serviceFilesFixture: &serviceFilesFixture{image: "held"}, sid: inertClientSID, sha: strings.Repeat("a", 64)}
			b := &clientServiceFixture{serviceFixture: &serviceFixture{failure: fault}}
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			if fault == "bad-sid" {
				f.sid = "S-1-5-18"
			}
			if fault == "bad-hash" {
				f.sha = strings.Repeat("A", 64)
			}
			b.startHook = func(mode string) {
				if mode == "verify-store" {
					switch fault {
					case "binding-change":
						f.sha = strings.Repeat("b", 64)
					case "file-custody":
						f.failCheck = f.checks + 1
					case "cancel-after-verify":
						cancel()
					}
				}
			}
			if fault == "unexpected-running" {
				b.states = []serviceState{{state: serviceStopped}, {state: serviceStopped}, {state: serviceStopped}, {state: 4}}
			}
			if fault == "registration-running" || fault == "registration-exit" {
				terminal := serviceState{state: 4}
				if fault == "registration-exit" {
					terminal = serviceState{state: serviceStopped, serviceExit: 1}
				}
				b.states = []serviceState{{state: serviceStopped}, {state: serviceStopped}, {state: serviceStopped}, {state: serviceStopped}, terminal}
			}
			err := initializeServiceWith(ctx, f, func() (serviceBackend, error) {
				if fault == "missing-backend" {
					return b.serviceFixture, nil
				}
				return b, nil
			}, inertServicePoll)
			if err == nil {
				t.Fatal("fault accepted")
			}
			counts := map[string]int{}
			for _, event := range b.events {
				counts[event]++
				if counts[event] > 1 {
					t.Fatal("action retried")
				}
			}
			if fault != "register-client" && fault != "registration-running" && fault != "registration-exit" && fault != "close" && counts["register-client"] != 0 {
				t.Fatalf("failure reached registration: %v", b.events)
			}
			if fault != "bad-sid" && fault != "bad-hash" && b.closes != 1 {
				t.Fatal("SCM custody not drained")
			}
		})
	}
}

func TestCanonicalClientIdentityRejectsAlternateNamespaces(t *testing.T) {
	for _, value := range []string{"", "S-1-5-18", "S-1-5-32-544", "S-1-5-80-1-2-3-4-5", "s-1-5-21-11-22-33-1001", "S-1-5-21-011-22-33-1001", "S-1-5-21-11-22-33-4294967296", inertClientSID + "\\extra"} {
		if canonicalClientSID(value) {
			t.Fatalf("unsafe identity: %q", value)
		}
	}
	if !canonicalClientSID(inertClientSID) {
		t.Fatal("canonical domain identity refused")
	}
}

func TestClientRequestOtherPlatformExplicitlyRefuses(t *testing.T) {
	if runtime.GOOS == "windows" {
		t.Skip("non-Windows refusal")
	}
	if raw, err := ReadApprovedClientRequestNative(context.Background(), inertClientSID, strings.Repeat("a", 64)); err == nil || raw != nil {
		t.Fatal("non-native request authority accepted")
	}
}
