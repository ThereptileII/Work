package brokerenroll

import (
	"context"
	"errors"
	"reflect"
	"strings"
	"testing"
	"time"
)

var errServiceFixture = errors.New("inert service fault")

type serviceFilesFixture struct {
	image     string
	checks    int
	failCheck int
}

func (f *serviceFilesFixture) ImagePath() string { return f.image }
func (f *serviceFilesFixture) Check() error {
	f.checks++
	if f.checks == f.failCheck {
		return errServiceFixture
	}
	return nil
}
func (*serviceFilesFixture) Close() error { return nil }

type serviceFixture struct {
	events    []string
	states    []serviceState
	failure   string
	checks    int
	failCheck int
	closes    int
	startHook func(string)
}

func (b *serviceFixture) create(string) error {
	b.events = append(b.events, "create")
	if b.failure == "create" {
		return errServiceFixture
	}
	return nil
}
func (b *serviceFixture) check(string) error {
	b.checks++
	if b.failure == "check" || b.checks == b.failCheck {
		return errServiceFixture
	}
	return nil
}
func (b *serviceFixture) start(mode string) error {
	b.events = append(b.events, mode)
	if b.startHook != nil {
		b.startHook(mode)
	}
	if b.failure == mode {
		return errServiceFixture
	}
	return nil
}
func (b *serviceFixture) status() (serviceState, error) {
	if b.failure == "status" {
		return serviceState{}, errServiceFixture
	}
	if len(b.states) == 0 {
		return serviceState{state: serviceStopped}, nil
	}
	s := b.states[0]
	b.states = b.states[1:]
	return s, nil
}
func (b *serviceFixture) close() error {
	b.closes++
	b.events = append(b.events, "close")
	if b.failure == "close" {
		return errServiceFixture
	}
	return nil
}

func runServiceFixture(ctx context.Context, f *serviceFilesFixture, b *serviceFixture, poll func(context.Context) error) error {
	return initializeServiceWith(ctx, f, func() (serviceBackend, error) { return b, nil }, poll)
}
func inertServicePoll(context.Context) error { return nil }

func TestServiceNeverStartedIsFreshOnlyAndNotCompletion(t *testing.T) {
	initial := serviceState{state: serviceStopped, win32Exit: serviceNeverStarted, processID: 123}
	if !serviceInitiallyStopped(initial) || serviceCompleted(initial) {
		t.Fatal("never-started initial status confused with successful initialization")
	}
	for _, bad := range []serviceState{
		{state: 4}, {state: serviceStartPending}, {state: serviceStopPending}, {state: 7}, {},
		{state: serviceStopped, win32Exit: 1}, {state: serviceStopped, win32Exit: serviceNeverStarted, serviceExit: 1},
		{state: serviceStopped, win32Exit: serviceNeverStarted, controls: 1},
	} {
		if serviceInitiallyStopped(bad) {
			t.Fatal("unsafe initial service status accepted", bad)
		}
	}
	f := &serviceFilesFixture{image: "held"}
	b := &serviceFixture{states: []serviceState{initial}}
	if err := runServiceFixture(context.Background(), f, b, inertServicePoll); err != nil ||
		!reflect.DeepEqual(b.events, []string{"create", "initialize-store", "verify-store", "close"}) {
		t.Fatal("original fresh never-started status prevented bounded initialization", err, b.events)
	}
}

func TestServiceNeverStartedAfterEitherStartWithholdsFollowupAndActivation(t *testing.T) {
	for _, failedMode := range []string{"initialize-store", "verify-store"} {
		t.Run(failedMode, func(t *testing.T) {
			f := &clientServiceFilesFixture{serviceFilesFixture: &serviceFilesFixture{image: "held"}, sid: inertClientSID, sha: strings.Repeat("a", 64)}
			states := []serviceState{{state: serviceStopped, win32Exit: serviceNeverStarted}}
			if failedMode == "verify-store" {
				states = append(states, serviceState{state: serviceStopped})
			}
			states = append(states, serviceState{state: serviceStopped, win32Exit: serviceNeverStarted})
			b := &clientServiceFixture{serviceFixture: &serviceFixture{states: states}}
			activated := 0
			err := initializeServiceWithCompletion(context.Background(), f, func() (serviceBackend, error) { return b, nil }, inertServicePoll,
				func(context.Context, enrollmentFiles, serviceBackend) error { activated++; return nil })
			want := []string{"create", "initialize-store", "close"}
			if failedMode == "verify-store" {
				want = []string{"create", "initialize-store", "verify-store", "close"}
			}
			if err == nil || activated != 0 || b.closes != 1 || !reflect.DeepEqual(b.events, want) {
				t.Fatal("post-start never-started status permitted retry, registration or activation", err, b.events)
			}
		})
	}
}

func TestServiceInitialStateDiagnosticPreservesReadAndQueryFailure(t *testing.T) {
	state := serviceState{state: serviceStopped, processID: 123, win32Exit: 5, serviceExit: 6, controls: 7}
	for _, phase := range []serviceInitialPhase{serviceInitialCreated, serviceInitialBeforeStart} {
		for _, cause := range []error{nil, errServiceFixture} {
			refusal := &serviceInitialStateError{phase: phase, state: state, cause: cause}
			var observed *serviceInitialStateError
			want := "new service was not stopped"
			if phase == serviceInitialBeforeStart {
				want = "new service must remain stopped"
			}
			if !errors.As(errors.Join(refusal, errors.New("fixture cleanup")), &observed) || observed.state != state || observed.phase != phase ||
				observed.cause != cause || refusal.Error() != want || (cause != nil && !errors.Is(refusal, cause)) {
				t.Fatal("original initial service status/query error or sanitized text lost")
			}
		}
	}
	files := &serviceFilesFixture{image: "held"}
	backend := &serviceFixture{failure: "status"}
	refusal := runServiceFixture(context.Background(), files, backend, inertServicePoll)
	var observed *serviceInitialStateError
	if !errors.As(refusal, &observed) || observed.phase != serviceInitialBeforeStart || !errors.Is(refusal, errServiceFixture) ||
		!reflect.DeepEqual(backend.events, []string{"create", "close"}) {
		t.Fatal("failed original pre-first-start query reached a start or lost cause", refusal, backend.events)
	}
}

func TestServiceEnrollmentOnlyInitializesAndIndependentlyVerifies(t *testing.T) {
	f := &serviceFilesFixture{image: "fixed-held-image"}
	b := &serviceFixture{states: []serviceState{{state: serviceStopped}, {state: serviceStartPending, processID: 23}, {state: serviceStopPending}, {state: serviceStopped}, {state: serviceStartPending}, {state: serviceStopped}}}
	polls := 0
	err := runServiceFixture(context.Background(), f, b, func(context.Context) error { polls++; return nil })
	if err != nil {
		t.Fatal(err)
	}
	if !reflect.DeepEqual(b.events, []string{"create", "initialize-store", "verify-store", "close"}) || b.closes != 1 || polls != 3 || f.checks < 8 {
		t.Fatalf("unexpected lifecycle: events=%v closes=%d polls=%d fileChecks=%d", b.events, b.closes, polls, f.checks)
	}
}

func TestServiceEnrollmentFailuresNeverRetryDeleteOrNormallyStart(t *testing.T) {
	for _, fault := range []string{"create", "check", "status", "initialize-store", "verify-store", "close"} {
		t.Run(fault, func(t *testing.T) {
			f := &serviceFilesFixture{image: "fixed-held-image"}
			b := &serviceFixture{failure: fault}
			if err := runServiceFixture(context.Background(), f, b, inertServicePoll); err == nil {
				t.Fatal("failure accepted")
			}
			if b.closes != 1 {
				t.Fatal("service owners not drained exactly once")
			}
			counts := map[string]int{}
			for _, e := range b.events {
				counts[e]++
				if e != "create" && e != "initialize-store" && e != "verify-store" && e != "close" {
					t.Fatalf("unexpected action %q", e)
				}
			}
			for action, count := range counts {
				if count > 1 {
					t.Fatalf("retried %s", action)
				}
			}
			if fault != "verify-store" && fault != "close" && counts["verify-store"] != 0 {
				t.Fatal("failure reached verification start")
			}
		})
	}
}

func TestServiceEnrollmentRefusesUnsuccessfulOrServingTerminalState(t *testing.T) {
	for name, s := range map[string]serviceState{
		"running": {state: 4, processID: 23}, "paused": {state: 7}, "win32-failure": {state: serviceStopped, win32Exit: 1},
		"service-failure": {state: serviceStopped, serviceExit: 2}, "stopped-accepts-controls": {state: serviceStopped, controls: 1},
		"pending-accepts-controls": {state: serviceStartPending, controls: 1}, "pending-failure": {state: serviceStopPending, win32Exit: 1},
	} {
		t.Run(name, func(t *testing.T) {
			f := &serviceFilesFixture{image: "fixed-held-image"}
			b := &serviceFixture{states: []serviceState{{state: serviceStopped}, s}}
			if err := runServiceFixture(context.Background(), f, b, inertServicePoll); err == nil {
				t.Fatalf("invalid state accepted: %+v", s)
			}
			if !reflect.DeepEqual(b.events, []string{"create", "initialize-store", "close"}) {
				t.Fatalf("unexpected followup: %v", b.events)
			}
		})
	}
}

func TestServiceEnrollmentPollAndCustodyFailuresWithholdVerification(t *testing.T) {
	for _, fault := range []string{"poll", "files", "spelling", "configuration", "cancellation"} {
		t.Run(fault, func(t *testing.T) {
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			f := &serviceFilesFixture{image: "fixed-held-image"}
			b := &serviceFixture{states: []serviceState{{state: serviceStopped}, {state: serviceStartPending}}}
			b.startHook = func(string) {
				switch fault {
				case "files":
					f.failCheck = f.checks + 1
				case "spelling":
					f.image = "substituted"
				case "configuration":
					b.failCheck = b.checks + 1
				case "cancellation":
					cancel()
				}
			}
			poll := func(context.Context) error { return errServiceFixture }
			if err := runServiceFixture(ctx, f, b, poll); err == nil {
				t.Fatal("uncertain initialization accepted")
			}
			if !reflect.DeepEqual(b.events, []string{"create", "initialize-store", "close"}) {
				t.Fatalf("unexpected followup: %v", b.events)
			}
		})
	}
}

func TestServiceEnrollmentRequiresCustodyBeforeOpeningSCM(t *testing.T) {
	for _, kind := range []string{"nil-context", "cancelled", "missing-files", "failed-files", "empty-image", "open-error", "nil-backend"} {
		t.Run(kind, func(t *testing.T) {
			ctx := context.Background()
			f := &serviceFilesFixture{image: "fixed-held-image"}
			var files enrollmentFiles = f
			if kind == "nil-context" {
				ctx = nil
			}
			if kind == "cancelled" {
				var cancel context.CancelFunc
				ctx, cancel = context.WithCancel(ctx)
				cancel()
			}
			if kind == "missing-files" {
				files = nil
			}
			if kind == "failed-files" {
				f.failCheck = 1
			}
			if kind == "empty-image" {
				f.image = ""
			}
			opens := 0
			err := initializeServiceWith(ctx, files, func() (serviceBackend, error) {
				opens++
				if kind == "nil-backend" {
					return nil, nil
				}
				return nil, errServiceFixture
			}, inertServicePoll)
			if err == nil {
				t.Fatal("invalid enrollment accepted")
			}
			if kind != "open-error" && kind != "nil-backend" && opens != 0 {
				t.Fatal("SCM opened before valid custody")
			}
		})
	}
}

func TestServicePollingHonorsBoundedContext(t *testing.T) {
	ctx, cancel := context.WithCancel(context.Background())
	cancel()
	b := &serviceFixture{}
	if err := waitServiceStopped(ctx, b, func() error { return nil }, inertServicePoll); !errors.Is(err, context.Canceled) {
		t.Fatalf("cancelled wait: %v", err)
	}
	if err := servicePoll(ctx); !errors.Is(err, context.Canceled) {
		t.Fatalf("cancelled native poll: %v", err)
	}
	expired, cancelExpired := context.WithDeadline(context.Background(), time.Now().Add(-time.Second))
	defer cancelExpired()
	if err := waitServiceStopped(expired, b, func() error { return nil }, inertServicePoll); !errors.Is(err, context.DeadlineExceeded) {
		t.Fatalf("expired bounded wait: %v", err)
	}
}

func TestServiceEnrollmentRefusesInitiallyRunningService(t *testing.T) {
	f := &serviceFilesFixture{image: "fixed-held-image"}
	b := &serviceFixture{states: []serviceState{{state: 4, processID: 23, controls: 1}}}
	if err := runServiceFixture(context.Background(), f, b, inertServicePoll); err == nil {
		t.Fatal("initially running service accepted")
	}
	if !reflect.DeepEqual(b.events, []string{"create", "close"}) {
		t.Fatalf("unexpected startup: %v", b.events)
	}
}
