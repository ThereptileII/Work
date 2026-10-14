package brokerenroll

import (
	"context"
	"errors"
	"reflect"
	"strings"
	"testing"

	"example.com/opennav-update-verifier/brokerpipe"
	"example.com/opennav-update-verifier/releasepolicy"
)

type finalizeBackendFixture struct {
	state                serviceState
	checkErr, processErr error
	closes               int
	events               *[]string
}

func (f *finalizeBackendFixture) check(ctx context.Context) error {
	return errors.Join(ctx.Err(), f.checkErr)
}
func (f *finalizeBackendFixture) status() (serviceState, error) { return f.state, nil }
func (f *finalizeBackendFixture) bindProcess(pid uint32) (maintenanceProcess, error) {
	if pid != f.state.processID {
		return nil, errors.New("PID changed")
	}
	return &finalizeProcessFixture{f}, nil
}
func (f *finalizeBackendFixture) close() error {
	f.closes++
	*f.events = append(*f.events, "backend.close")
	return nil
}

type finalizeProcessFixture struct{ backend *finalizeBackendFixture }

func (p *finalizeProcessFixture) live() error { return p.backend.processErr }
func (p *finalizeProcessFixture) exit() (bool, uint32, error) {
	return false, 0, errors.New("finalization never observes stop/exit")
}
func (p *finalizeProcessFixture) close() error {
	*p.backend.events = append(*p.backend.events, "process.close")
	return nil
}

type finalizePublicationFixture struct {
	checkErr, closeErr error
	guardHook          func()
	effects, closes    int
	events             *[]string
}

func (p *finalizePublicationFixture) check(ctx context.Context) error {
	return errors.Join(ctx.Err(), p.checkErr)
}
func (p *finalizePublicationFixture) publish(ctx context.Context, guard func(context.Context) error) error {
	if p.guardHook != nil {
		p.guardHook()
	}
	if err := guard(ctx); err != nil {
		return err
	}
	p.effects++
	*p.events = append(*p.events, "ready.create")
	return guard(ctx)
}
func (p *finalizePublicationFixture) close() error {
	p.closes++
	*p.events = append(*p.events, "publication.close")
	return p.closeErr
}
func finalizeResponse(q brokerpipe.MaintenanceRequest) brokerpipe.MaintenanceResult {
	ancestors := []brokerpipe.MaintenanceAncestor{{Generation: strings.Repeat("2", 32), CompletionSHA256: strings.Repeat("3", 64)}}
	return brokerpipe.MaintenanceResult{Schema: 1, RequestID: q.RequestID, State: brokerpipe.SelectedHistory, Generation: strings.Repeat("4", 32), CompletionSHA256: strings.Repeat("5", 64), SelectedHeadSHA256: strings.Repeat("6", 64), Version: "1.0.0-beta.2", Commit: strings.Repeat("7", 40), QualificationSHA256: strings.Repeat("8", 64), QualificationClosedSHA256: strings.Repeat("9", 64), InventorySHA256: strings.Repeat("a", 64), Page: q.Page, Ancestors: &ancestors}
}
func TestCodeUpgradeFinalizePublishesOnceAndClosesBorrowedProcessFirst(t *testing.T) {
	events := []string{}
	backend := &finalizeBackendFixture{state: serviceState{state: 4, processID: 42, controls: 5}, events: &events}
	pub := &finalizePublicationFixture{events: &events}
	calls := 0
	owner, err := openCodeFinalizerWith(context.Background(), backend, pub, func(_ context.Context, q brokerpipe.MaintenanceRequest) (brokerpipe.MaintenanceResult, error) {
		calls++
		events = append(events, "ready.query")
		return finalizeResponse(q), nil
	})
	if err != nil {
		t.Fatal(err)
	}
	if err = owner.resume(context.Background()); err != nil {
		t.Fatal(err)
	}
	if err = owner.resume(context.Background()); err == nil {
		t.Fatal("finalization replayed")
	}
	if calls != 1 || pub.effects != 1 {
		t.Fatal("finalization repeated readiness or publication", calls, pub.effects)
	}
	if err = owner.close(); err != nil {
		t.Fatal(err)
	}
	if err = owner.close(); err != nil {
		t.Fatal(err)
	}
	want := []string{"ready.query", "ready.create", "process.close", "backend.close", "publication.close"}
	if !reflect.DeepEqual(events, want) || pub.closes != 1 || backend.closes != 1 {
		t.Fatal("borrowed process/image lifetime changed", events)
	}
}
func TestCodeUpgradeFinalizeRevocationWithholdsReadyPublication(t *testing.T) {
	for _, fault := range []string{"transport-cleanup", "invalid-history", "process-after-readiness", "pid-after-readiness", "custody-after-readiness", "process-before-create"} {
		t.Run(fault, func(t *testing.T) {
			events := []string{}
			backend := &finalizeBackendFixture{state: serviceState{state: 4, processID: 42, controls: 5}, events: &events}
			pub := &finalizePublicationFixture{events: &events}
			calls := 0
			call := func(_ context.Context, q brokerpipe.MaintenanceRequest) (brokerpipe.MaintenanceResult, error) {
				calls++
				r := finalizeResponse(q)
				switch fault {
				case "transport-cleanup":
					return r, ErrCleanup
				case "invalid-history":
					r.RequestID = strings.Repeat("b", 32)
				case "process-after-readiness":
					backend.processErr = errors.New("exited")
				case "pid-after-readiness":
					backend.state.processID++
				case "custody-after-readiness":
					backend.checkErr = errors.New("changed exact config")
				case "process-before-create":
					pub.guardHook = func() { backend.processErr = errors.New("token/image changed") }
				}
				return r, nil
			}
			owner, err := openCodeFinalizerWith(context.Background(), backend, pub, call)
			if err != nil {
				t.Fatal(err)
			}
			if err = owner.resume(context.Background()); err == nil || pub.effects != 0 {
				t.Fatal("unverified finalization published", err, pub.effects)
			}
			if err = owner.resume(context.Background()); err == nil || calls != 1 {
				t.Fatal("failed readiness retried", calls)
			}
			if err = owner.close(); err != nil {
				t.Fatal(err)
			}
		})
	}
}
func TestCodeUpgradeFinalizeRejectsPendingStoppedOrExistingReadyBeforeQuery(t *testing.T) {
	for _, fault := range []string{"stopped", "pending", "nonzero-exit", "missing-controls", "existing-ready", "changed-identity"} {
		t.Run(fault, func(t *testing.T) {
			events := []string{}
			backend := &finalizeBackendFixture{state: serviceState{state: 4, processID: 42, controls: 5}, events: &events}
			pub := &finalizePublicationFixture{events: &events}
			switch fault {
			case "stopped":
				backend.state = serviceState{state: serviceStopped}
			case "pending":
				backend.state.state = serviceStartPending
			case "nonzero-exit":
				backend.state.serviceExit = 1
			case "missing-controls":
				backend.state.controls = 0
			case "existing-ready":
				pub.checkErr = errors.New("any ready presence")
			case "changed-identity":
				backend.processErr = errors.New("token/image changed")
			}
			if owner, err := openCodeFinalizerWith(context.Background(), backend, pub, func(context.Context, brokerpipe.MaintenanceRequest) (brokerpipe.MaintenanceResult, error) {
				t.Fatal("unverified finalization queried readiness")
				return brokerpipe.MaintenanceResult{}, nil
			}); err == nil || owner != nil {
				t.Fatal("invalid finalizer acquired")
			}
			if pub.effects != 0 || pub.closes != 1 || backend.closes != 1 {
				t.Fatal("invalid opener mutated or leaked custody")
			}
		})
	}
}
func TestCodeUpgradeFinalizeCleanupFailureRetainsUncertainty(t *testing.T) {
	events := []string{}
	backend := &finalizeBackendFixture{state: serviceState{state: 4, processID: 42, controls: 5}, events: &events}
	pub := &finalizePublicationFixture{events: &events, closeErr: errors.New("close uncertain")}
	owner, err := openCodeFinalizerWith(context.Background(), backend, pub, func(_ context.Context, q brokerpipe.MaintenanceRequest) (brokerpipe.MaintenanceResult, error) {
		return finalizeResponse(q), nil
	})
	if err != nil {
		t.Fatal(err)
	}
	if err = owner.resume(context.Background()); err != nil {
		t.Fatal(err)
	}
	first := owner.close()
	second := owner.close()
	if !errors.Is(first, ErrCodeUpgradeUncertain) || !errors.Is(first, ErrCleanup) || second != first || pub.effects != 1 {
		t.Fatal("late cleanup uncertainty lost or publication retried", first, second)
	}
}

type resumeOwnerFixture struct {
	events *[]string
	closed bool
}

func (n *resumeOwnerFixture) check(context.Context) error {
	if n.closed {
		return errors.New("closed")
	}
	return nil
}
func (n *resumeOwnerFixture) resume(context.Context) error {
	*n.events = append(*n.events, "resume-selected-phase")
	return nil
}
func (n *resumeOwnerFixture) close() error {
	n.closed = true
	*n.events = append(*n.events, "resume.close")
	return nil
}
func TestCodeUpgradeResumeSelectsNativePhaseWithoutReplacementAcquisition(t *testing.T) {
	r, op, _, events := upgradeFixture(t)
	r.Resume = true
	op.acquire = nil
	op.open = nil
	op.openResume = func(_ context.Context, _ releasepolicy.BrokerCodeUpgradePolicy, raw, trust []byte, pin string) (codeUpgradeResumeOwner, error) {
		if digest(raw) != r.PolicySHA256 || digest(trust) != r.TrustConfigSHA256 || pin != r.OriginalCompletionSHA256 {
			t.Fatal("resume lost independent exact authority")
		}
		*events = append(*events, "resume.open")
		return &resumeOwnerFixture{events: events}, nil
	}
	if err := upgradeEnrolledCodeWith(context.Background(), r, op); err != nil {
		t.Fatal(err)
	}
	want := []string{"native-original-trust", "hold", "verify-upgrade-purpose", "source", "resume.open", "resume-selected-phase", "resume.close", "source.close", "inputs.close"}
	if !reflect.DeepEqual(*events, want) {
		t.Fatal("resume crossed into replacement acquisition/enrollment", *events)
	}
}
