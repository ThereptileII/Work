package brokerenroll

import (
	"bytes"
	"context"
	"errors"
	"reflect"
	"runtime"
	"testing"
	"time"

	"example.com/opennav-update-verifier/brokerregistration"
)

type startupServiceFixture struct {
	*clientServiceFixture
	activations  int
	activateErr  error
	activateHook func()
}

func (b *startupServiceFixture) activateNormal() error {
	b.activations++
	b.events = append(b.events, "activate-normal")
	if b.activateHook != nil {
		b.activateHook()
	}
	if b.activateErr == nil && len(b.states) == 0 {
		b.states = []serviceState{{state: 4, processID: 42}}
	}
	return b.activateErr
}

type startupPublicationFixture struct {
	checks, closes     int
	checkErr, closeErr error
}

func (p *startupPublicationFixture) Check(ctx context.Context) error {
	p.checks++
	if ctx.Err() != nil {
		return ctx.Err()
	}
	return p.checkErr
}
func (p *startupPublicationFixture) Close() error { p.closes++; return p.closeErr }

func startupFlowFixture(t *testing.T) (*clientServiceFilesFixture, *startupServiceFixture, *brokerregistration.Request, *baselineFixture) {
	t.Helper()
	enrollment := newEnrollmentFixture(t)
	raw := approvedClientFixture(t, enrollment.req.TrustConfiguration)
	parsed, err := brokerregistration.ParseApprovedRequest(raw, digest(raw), time.Now().UTC())
	if err != nil {
		t.Fatal(err)
	}
	files := &clientServiceFilesFixture{serviceFilesFixture: &serviceFilesFixture{image: "held-original-image"}, sid: parsed.ClientSID, sha: parsed.RequestSHA256}
	backend := &startupServiceFixture{clientServiceFixture: &clientServiceFixture{serviceFixture: &serviceFixture{}}}
	return files, backend, &parsed, newBaselineFixture()
}

func TestStartupExplicitCompletionUsesOriginalStoppedRegistration(t *testing.T) {
	files, b, client, baseline := startupFlowFixture(t)
	publication := &startupPublicationFixture{}
	err := initializeServiceWithCompletion(context.Background(), files, func() (serviceBackend, error) { return b, nil }, inertServicePoll, func(ctx context.Context, owned enrollmentFiles, service serviceBackend) error {
		if owned != files || service != b || b.closes != 0 || !reflect.DeepEqual(b.events, []string{"create", "initialize-store", "verify-store", "register-client"}) {
			t.Fatal("completion reached without original stopped SCM registration")
		}
		return completeStartup(ctx, owned, service, client, baseline, func(context.Context, enrollmentFiles, *brokerregistration.Request, ApprovedClientBaseline) (startupPublication, error) {
			b.events = append(b.events, "publish-shortcut")
			return publication, nil
		}, inertServicePoll)
	})
	if err != nil {
		t.Fatal(err)
	}
	if !reflect.DeepEqual(b.events, []string{"create", "initialize-store", "verify-store", "register-client", "publish-shortcut", "activate-normal", "close"}) || b.activations != 1 || b.closes != 1 || publication.closes != 1 || publication.checks < 2 {
		t.Fatalf("invalid explicit lifecycle %+v %+v", b.events, publication)
	}
}

func TestStartupStoppedFailureNeverPublishesOrActivates(t *testing.T) {
	for _, failure := range []string{"create", "initialize-store", "verify-store", "register-client", "configuration", "registration-running", "registration-exit", "no-client"} {
		t.Run(failure, func(t *testing.T) {
			files, b, _, _ := startupFlowFixture(t)
			b.failure = failure
			called := 0
			if failure == "configuration" {
				b.failure = "check"
			}
			if failure == "registration-running" || failure == "registration-exit" {
				state := serviceState{state: 4, processID: 42}
				if failure == "registration-exit" {
					state = serviceState{state: serviceStopped, serviceExit: 1}
				}
				b.states = []serviceState{{state: 1}, {state: 1}, {state: 1}, {state: 1}, state}
			}
			if failure == "no-client" {
				files.sid = ""
				files.sha = ""
			}
			err := initializeServiceWithCompletion(context.Background(), files, func() (serviceBackend, error) { return b, nil }, inertServicePoll, func(context.Context, enrollmentFiles, serviceBackend) error { called++; return nil })
			if err == nil || called != 0 || b.activations != 0 || b.closes != 1 {
				t.Fatalf("failure reached effects err=%v called=%d events=%v", err, called, b.events)
			}
		})
	}
}

func TestStartupPublicationAndActivationFaultsRemainUncertain(t *testing.T) {
	for _, failure := range []string{"publication", "missing-publication", "shortcut-custody", "baseline-custody", "binding-change", "cancel-after-publication", "service-state-change", "activation", "missing-running-pid", "running-exit", "poll", "close"} {
		t.Run(failure, func(t *testing.T) {
			files, b, client, baseline := startupFlowFixture(t)
			publication := &startupPublicationFixture{}
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			publish := func(context.Context, enrollmentFiles, *brokerregistration.Request, ApprovedClientBaseline) (startupPublication, error) {
				b.events = append(b.events, "publish-shortcut")
				switch failure {
				case "publication":
					return publication, errServiceFixture
				case "missing-publication":
					return nil, nil
				case "shortcut-custody":
					publication.checkErr = errServiceFixture
				case "baseline-custody":
					baseline.checkErr = errServiceFixture
				case "binding-change":
					files.sha = "changed"
				case "cancel-after-publication":
					cancel()
				case "service-state-change":
					b.states = []serviceState{{state: 4, processID: 42}}
				case "close":
					publication.closeErr = ErrCleanup
				}
				return publication, nil
			}
			if failure == "activation" {
				b.activateErr = errServiceFixture
			}
			if failure == "missing-running-pid" {
				b.activateHook = func() { b.states = []serviceState{{state: 4}} }
			}
			if failure == "running-exit" {
				b.activateHook = func() { b.states = []serviceState{{state: 4, processID: 42, serviceExit: 1}} }
			}
			if failure == "poll" {
				b.activateHook = func() { b.states = []serviceState{{state: 2}} }
			}
			poll := func(context.Context) error { return errServiceFixture }
			err := completeStartup(ctx, files, b, client, baseline, publish, poll)
			if err == nil || !errors.Is(err, ErrStartupUncertain) {
				t.Fatalf("uncertainty lost: %v", err)
			}
			activated := failure == "activation" || failure == "missing-running-pid" || failure == "running-exit" || failure == "poll" || failure == "close"
			if activated != (b.activations == 1) || b.activations > 1 {
				t.Fatalf("unexpected/repeated activation events=%v", b.events)
			}
			if failure != "missing-publication" && publication.closes != 1 {
				t.Fatal("publication owner leaked")
			}
		})
	}
}

func TestStartupRunRetainsOriginalApprovalAndBaseline(t *testing.T) {
	f, files, baseline := approvedBaselineEnrollment(t)
	called := 0
	original := bytes.Clone(f.req.ClientRegistrationRequest)
	f.op.initialize = func(context.Context, enrollmentFiles) error {
		t.Fatal("explicit startup routed stopped-only initializer")
		return nil
	}
	f.op.initializeStartup = func(ctx context.Context, owned enrollmentFiles, client *brokerregistration.Request, source ApprovedClientBaseline) error {
		called++
		if owned != files || source != baseline || baseline.closed || files.closed || !bytes.Equal(client.OriginalBytes(), original) || client.RequestSHA256 != digest(original) || files.publishes != 1 || files.baselinePublishes != 1 {
			t.Fatal("startup lost original private approval/source ownership")
		}
		return ctx.Err()
	}
	if err := run(context.Background(), f.req, f.op); err != nil {
		t.Fatal(err)
	}
	if called != 1 || !files.closed || !baseline.closed || !f.inputs.closed || !f.broker.closed {
		t.Fatal("startup completion ownership leaked")
	}
}

func TestStartupExplicitAPIRefusesNonWindows(t *testing.T) {
	if runtime.GOOS == "windows" {
		t.Skip("non-Windows boundary only")
	}
	if err := RunAndActivateStartup(context.Background(), Request{}); err == nil {
		t.Fatal("native startup accepted off Windows")
	}
}
