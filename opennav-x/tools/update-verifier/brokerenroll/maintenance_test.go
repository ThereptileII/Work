package brokerenroll

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"reflect"
	"runtime"
	"strings"
	"testing"
	"time"
)

type maintenanceFixture struct {
	events                    []string
	state                     serviceState
	process                   maintenanceProcessFixture
	req                       MaintenanceRequest
	op                        maintenanceOperations
	stops, closes, authChecks int
	fault                     string
	afterStop                 func()
	pollHook                  func(context.Context) error
	onOpen                    func()
}
type maintenanceProcessFixture struct {
	fixture *maintenanceFixture
	exited  bool
	code    uint32
	closes  int
}

func newMaintenanceFixture(t *testing.T) *maintenanceFixture {
	t.Helper()
	enrollment := newEnrollmentFixture(t)
	raw := approvedClientFixture(t, enrollment.req.TrustConfiguration)
	f := &maintenanceFixture{
		req:   MaintenanceRequest{ApprovedBrokerSHA256: strings.Repeat("a", 64), ClientRegistrationRequest: raw, ApprovedClientRequestSHA256: digest(raw)},
		state: serviceState{state: 4, processID: 23, controls: 5},
	}
	f.process.fixture = f
	f.op = maintenanceOperations{
		authenticate: func(sid string) error {
			f.authChecks++
			if sid != inertClientSID || f.fault == "operator" {
				return errors.New("actual elevated nonimpersonated approved client differs")
			}
			return nil
		},
		open: func(ctx context.Context, authority maintenanceAuthority) (maintenanceBackend, error) {
			f.events = append(f.events, "open")
			if authority.brokerSHA != strings.Repeat("a", 64) || authority.client.ClientSID != inertClientSID || !bytes.Equal(authority.client.OriginalBytes(), raw) {
				t.Fatal("lost original approval binding")
			}
			if f.onOpen != nil {
				f.onOpen()
			}
			if f.fault == "open" {
				return f, errors.New("partial native open")
			}
			return f, nil
		},
		poll: func(ctx context.Context) error {
			f.events = append(f.events, "poll")
			if f.pollHook != nil {
				return f.pollHook(ctx)
			}
			f.state = serviceState{state: serviceStopped}
			f.process.exited = true
			return nil
		},
		limit: maintenanceStopLimit,
	}
	return f
}
func (f *maintenanceFixture) check(context.Context) error {
	f.events = append(f.events, "check")
	if f.fault == "custody" {
		return errors.New("exact image/request/config/ACL custody differs")
	}
	return nil
}
func (f *maintenanceFixture) status() (serviceState, error) {
	f.events = append(f.events, "status")
	if f.fault == "status" {
		return serviceState{}, errors.New("status unavailable")
	}
	return f.state, nil
}
func (f *maintenanceFixture) bindProcess(pid uint32) (maintenanceProcess, error) {
	f.events = append(f.events, "bind")
	if pid != f.state.processID {
		panic("binding different process")
	}
	if f.fault == "bind" {
		return &f.process, errors.New("partial process binding")
	}
	return &f.process, nil
}
func (f *maintenanceFixture) stop(pid uint32) error {
	f.events = append(f.events, "stop")
	f.stops++
	if pid != 23 || f.process.closes != 0 || f.closes != 0 {
		panic("STOP without retained original process and service")
	}
	if f.afterStop != nil {
		f.afterStop()
	}
	if f.fault == "stop" {
		return errors.New("stop dispatch uncertain")
	}
	f.state = serviceState{state: serviceStopPending}
	return nil
}
func (f *maintenanceFixture) close() error {
	f.events = append(f.events, "close-service")
	f.closes++
	if f.fault == "service-close" || f.fault == "both-close" {
		return errors.New("service close uncertain")
	}
	return nil
}
func (p *maintenanceProcessFixture) live() error {
	p.fixture.events = append(p.fixture.events, "live")
	if p.exited || p.closes != 0 || p.fixture.fault == "process" {
		return errors.New("original process/token/image identity changed")
	}
	return nil
}
func (p *maintenanceProcessFixture) exit() (bool, uint32, error) {
	p.fixture.events = append(p.fixture.events, "exit")
	if p.closes != 0 || p.fixture.fault == "process" {
		return false, 0, errors.New("original process exit identity uncertain")
	}
	return p.exited, p.code, nil
}
func (p *maintenanceProcessFixture) close() error {
	p.fixture.events = append(p.fixture.events, "close-process")
	p.closes++
	if p.fixture.fault == "process-close" || p.fixture.fault == "both-close" {
		return errors.New("process close uncertain")
	}
	return nil
}

func TestMaintenanceBindsOriginalBeforeOneStopAndRetainsExitProof(t *testing.T) {
	f := newMaintenanceFixture(t)
	original := bytes.Clone(f.req.ClientRegistrationRequest)
	f.onOpen = func() { f.req.ClientRegistrationRequest[0] = '!' }
	owner, err := openMaintenanceWith(context.Background(), f.req, f.op)
	if err != nil || owner == nil {
		t.Fatal(err)
	}
	if !bytes.Equal(owner.authority.client.OriginalBytes(), original) || f.stops != 0 {
		t.Fatal("open mutated service or lost approved original")
	}
	var copied MaintenanceOwner
	reflect.ValueOf(&copied).Elem().Set(reflect.ValueOf(owner).Elem())
	if copied.Check(context.Background()) == nil || copied.Quiesce(context.Background()) == nil || copied.Close() == nil || f.stops != 0 || f.closes != 0 {
		t.Fatal("copied owner gained authority or released original custody")
	}
	if err := owner.Check(context.Background()); err != nil {
		t.Fatal(err)
	}
	if err := owner.Quiesce(context.Background()); err != nil {
		t.Fatal(err)
	}
	if err := owner.Check(context.Background()); err != nil {
		t.Fatal(err)
	}
	if !owner.quiescent || f.stops != 1 || f.closes != 0 || f.process.closes != 0 {
		t.Fatal("quiescence lost original owners or repeated STOP")
	}
	bind, stop, live := -1, -1, -1
	for i, event := range f.events {
		if event == "bind" {
			bind = i
		}
		if event == "live" && stop == -1 {
			live = i
		}
		if event == "stop" {
			stop = i
		}
	}
	if bind < 0 || live < bind || stop < live {
		t.Fatal("STOP preceded actual retained process binding", f.events)
	}
	if err := owner.Quiesce(context.Background()); err == nil || f.stops != 1 {
		t.Fatal("stop attempt replayed")
	}
	if err := owner.Close(); err != nil {
		t.Fatal(err)
	}
	if err := owner.Close(); err != nil {
		t.Fatal(err)
	}
	if f.closes != 1 || f.process.closes != 1 || !reflect.DeepEqual(f.events[len(f.events)-2:], []string{"close-process", "close-service"}) {
		t.Fatal("original owners not drained once in order", f.events)
	}
}

func TestMaintenanceIndependentApprovalAndCurrentOperatorBeforeNativeOpen(t *testing.T) {
	for _, fault := range []string{"broker-hash", "client-hash", "client-bytes", "operator", "nil-context", "cancelled", "nil-auth", "nil-open", "nil-poll", "unbounded-wait"} {
		t.Run(fault, func(t *testing.T) {
			f := newMaintenanceFixture(t)
			ctx := context.Background()
			switch fault {
			case "broker-hash":
				f.req.ApprovedBrokerSHA256 = ""
			case "client-hash":
				f.req.ApprovedClientRequestSHA256 = strings.Repeat("0", 64)
			case "client-bytes":
				f.req.ClientRegistrationRequest = []byte(`{"clientSid":"` + inertClientSID + `"}`)
				f.req.ApprovedClientRequestSHA256 = digest(f.req.ClientRegistrationRequest)
			case "operator":
				f.fault = fault
			case "nil-context":
				ctx = nil
			case "cancelled":
				var cancel context.CancelFunc
				ctx, cancel = context.WithCancel(ctx)
				cancel()
			case "nil-auth":
				f.op.authenticate = nil
			case "nil-open":
				f.op.open = nil
			case "nil-poll":
				f.op.poll = nil
			case "unbounded-wait":
				f.op.limit = maintenanceStopLimit + time.Second
			}
			owner, err := openMaintenanceWith(ctx, f.req, f.op)
			if err == nil || owner != nil || len(f.events) != 0 || f.stops != 0 {
				t.Fatal("invalid approval reached native effects", err, f.events)
			}
		})
	}
}

func TestMaintenanceOpenRefusesUnknownStoppedPendingAndPartialOwners(t *testing.T) {
	for _, fault := range []string{"stopped", "start-pending", "stop-pending", "wrong-controls", "service-error", "no-pid", "open", "bind", "custody", "status", "process"} {
		t.Run(fault, func(t *testing.T) {
			f := newMaintenanceFixture(t)
			switch fault {
			case "stopped":
				f.state = serviceState{state: serviceStopped}
			case "start-pending":
				f.state.state = serviceStartPending
			case "stop-pending":
				f.state.state = serviceStopPending
			case "wrong-controls":
				f.state.controls = 1
			case "service-error":
				f.state.serviceExit = 1
			case "no-pid":
				f.state.processID = 0
			default:
				f.fault = fault
			}
			owner, err := openMaintenanceWith(context.Background(), f.req, f.op)
			if err == nil || owner != nil || f.stops != 0 || f.closes != 1 {
				t.Fatal("unknown state adopted or partial service not closed", err, f.events)
			}
			if (fault == "bind" || fault == "process") && f.process.closes != 1 {
				t.Fatal("partial process owner not drained")
			}
		})
	}
}

func TestMaintenanceStopFailureNeverRetriesRestartsOrTreatsPIDAbsenceAsExit(t *testing.T) {
	for _, fault := range []string{"operator", "custody", "process", "stop", "status", "wrong-pid", "failed-service", "failed-process", "stopped-but-live", "pending-forever", "exit-identity"} {
		t.Run(fault, func(t *testing.T) {
			f := newMaintenanceFixture(t)
			owner, err := openMaintenanceWith(context.Background(), f.req, f.op)
			if err != nil {
				t.Fatal(err)
			}
			defer owner.Close()
			dispatched := fault != "operator" && fault != "custody" && fault != "process" && fault != "wrong-pid"
			switch fault {
			case "operator", "custody", "process", "stop":
				f.fault = fault
			case "wrong-pid":
				f.state.processID++
			default:
				f.pollHook = func(ctx context.Context) error {
					switch fault {
					case "status":
						f.fault = "status"
					case "failed-service":
						f.state = serviceState{state: serviceStopped, serviceExit: 1}
						f.process.exited = true
					case "failed-process":
						f.state = serviceState{state: serviceStopped}
						f.process.exited, f.process.code = true, 1
					case "exit-identity":
						f.fault = "process"
					case "stopped-but-live":
						if f.state.state != serviceStopped {
							f.state = serviceState{state: serviceStopped}
							return nil
						}
					}
					if fault == "pending-forever" || fault == "stopped-but-live" {
						return context.DeadlineExceeded
					}
					return nil
				}
			}
			err = owner.Quiesce(context.Background())
			if err == nil || owner.quiescent || dispatched != errors.Is(err, ErrMaintenanceUncertain) {
				t.Fatal("failure authorized quiescence or lost stop uncertainty", err)
			}
			stops := f.stops
			if err := owner.Quiesce(context.Background()); err == nil || f.stops != stops {
				t.Fatal("failed maintenance retried")
			}
			if err := owner.Check(context.Background()); err == nil {
				t.Fatal("failed owner revived")
			}
			if dispatched && stops != 1 || !dispatched && stops != 0 {
				t.Fatal("incorrect stop effect", f.events)
			}
		})
	}
}

func TestMaintenanceBoundedWaitAndRestartRevokeProof(t *testing.T) {
	f := newMaintenanceFixture(t)
	f.op.limit = 5 * time.Millisecond
	f.pollHook = func(ctx context.Context) error { <-ctx.Done(); return ctx.Err() }
	owner, err := openMaintenanceWith(context.Background(), f.req, f.op)
	if err != nil {
		t.Fatal(err)
	}
	if err := owner.Quiesce(context.Background()); !errors.Is(err, context.DeadlineExceeded) || !errors.Is(err, ErrMaintenanceUncertain) {
		t.Fatal("wait not bounded", err)
	}
	owner.Close()
	f = newMaintenanceFixture(t)
	owner, err = openMaintenanceWith(context.Background(), f.req, f.op)
	if err != nil {
		t.Fatal(err)
	}
	defer owner.Close()
	if err := owner.Quiesce(context.Background()); err != nil {
		t.Fatal(err)
	}
	f.state = serviceState{state: 4, processID: 24, controls: 5}
	if err := owner.Check(context.Background()); !errors.Is(err, ErrMaintenanceUncertain) {
		t.Fatal("unexpected restart accepted", err)
	}
	f.state = serviceState{state: serviceStopped}
	if err := owner.Check(context.Background()); err == nil {
		t.Fatal("revoked owner revived")
	}
}

func TestMaintenanceCleanupUncertaintyPersistsAndDrainsEveryOwner(t *testing.T) {
	for _, fault := range []string{"service-close", "process-close", "both-close"} {
		t.Run(fault, func(t *testing.T) {
			f := newMaintenanceFixture(t)
			owner, err := openMaintenanceWith(context.Background(), f.req, f.op)
			if err != nil {
				t.Fatal(err)
			}
			if err := owner.Quiesce(context.Background()); err != nil {
				t.Fatal(err)
			}
			f.fault = fault
			for i := 0; i < 2; i++ {
				if err := owner.Close(); !errors.Is(err, ErrCleanup) || !errors.Is(err, ErrMaintenanceUncertain) {
					t.Fatal("cleanup uncertainty lost", err)
				}
			}
			if f.closes != 1 || f.process.closes != 1 {
				t.Fatal("close retried or partial owner lost")
			}
		})
	}
}

func TestMaintenanceExplicitAPIRequiresWindows(t *testing.T) {
	if runtime.GOOS == "windows" {
		t.Skip("non-Windows boundary only")
	}
	f := newMaintenanceFixture(t)
	if owner, err := OpenMaintenance(context.Background(), f.req); err == nil || owner != nil {
		t.Fatal("non-Windows native maintenance accepted")
	}
}

func TestMaintenanceOwnerCannotBeReconstructedFromJSONOrZeroValue(t *testing.T) {
	var owner MaintenanceOwner
	if err := json.Unmarshal([]byte(`{}`), &owner); err == nil {
		t.Fatal("maintenance owner reconstructed from declaration")
	}
	if _, err := json.Marshal(&owner); err == nil {
		t.Fatal("maintenance owner exported as authority")
	}
	if err := owner.Check(context.Background()); err == nil {
		t.Fatal("zero-value maintenance owner authorized")
	}
	if err := owner.Quiesce(context.Background()); err == nil {
		t.Fatal("zero-value maintenance owner stopped service")
	}
}
