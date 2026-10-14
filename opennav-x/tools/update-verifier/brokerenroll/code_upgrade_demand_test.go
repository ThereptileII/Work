package brokerenroll

import (
	"context"
	"errors"
	"strings"
	"testing"
)

// Inject failures at the irreversible boundary. The fake models SCM changing
// despite an API error and a second invocation encountering the durable fence;
// neither may trigger another configuration call or normal start.
type demandBackendFixture struct {
	state                   serviceState
	attempt                 bool
	auto                    bool
	fences, configs, closes int
	fenceErr, configErr     error
	checkErr, closeErr      error
	afterFence              func()
	events                  *[]string
}

func (b *demandBackendFixture) check(context.Context) error   { return b.checkErr }
func (b *demandBackendFixture) status() (serviceState, error) { return b.state, nil }
func (b *demandBackendFixture) fence(context.Context) error {
	b.fences++
	*b.events = append(*b.events, "attempt.flush.readback")
	if b.attempt {
		return errors.New("existing attempt")
	}
	b.attempt = true
	if b.afterFence != nil {
		b.afterFence()
	}
	return b.fenceErr
}
func (b *demandBackendFixture) automatic(context.Context) error {
	b.configs++
	*b.events = append(*b.events, "configure.auto")
	b.auto = true // A failing configuration call can still take effect.
	return b.configErr
}
func (b *demandBackendFixture) close() error {
	b.closes++
	*b.events = append(*b.events, "config-capability.closed")
	return b.closeErr
}

type demandPublicationFixture struct {
	err, closeErr error
	closes        int
	events        *[]string
}

func (p *demandPublicationFixture) check(context.Context) error { return p.err }
func (p *demandPublicationFixture) publish(context.Context, func(context.Context) error) error {
	panic("demand transition must never publish ready")
}
func (p *demandPublicationFixture) close() error {
	p.closes++
	*p.events = append(*p.events, "pair-custody.closed")
	return p.closeErr
}

type demandAutomaticFixture struct {
	checks, resumes, closes int
	err                     error
	events                  *[]string
}

func (a *demandAutomaticFixture) check(context.Context) error { a.checks++; return a.err }
func (a *demandAutomaticFixture) resume(context.Context) error {
	a.resumes++
	*a.events = append(*a.events, "existing-auto-resume")
	return a.err
}
func (a *demandAutomaticFixture) close() error { a.closes++; return nil }

func demandTransitionFixture() (*codeUpgradeDemandTransition, *demandBackendFixture, *demandPublicationFixture, *demandAutomaticFixture, *[]string) {
	events := []string{}
	b := &demandBackendFixture{state: serviceState{state: serviceStopped}, events: &events}
	p := &demandPublicationFixture{events: &events}
	a := &demandAutomaticFixture{events: &events}
	n := &codeUpgradeDemandTransition{backend: b, publication: p, openAutomatic: func(context.Context) (codeUpgradeResumeOwner, error) {
		events = append(events, "reauthenticate-auto")
		if b.closes != 1 || p.closes != 1 || !b.auto || !b.attempt {
			return nil, errors.New("unreleased or unqualified configuration handoff")
		}
		return a, nil
	}}
	n.self = n
	return n, b, p, a, &events
}

func TestCodeUpgradeDemandReleasesConfigBeforeExistingFencedStartHandoff(t *testing.T) {
	n, b, p, a, events := demandTransitionFixture()
	if err := n.resume(context.Background()); err != nil {
		t.Fatal(err)
	}
	if b.fences != 1 || b.configs != 1 || a.resumes != 1 || len(*events) != 6 ||
		(*events)[0] != "attempt.flush.readback" || (*events)[1] != "configure.auto" ||
		(*events)[2] != "config-capability.closed" || (*events)[3] != "pair-custody.closed" {
		t.Fatal("unexpected mutation/capability order", *events)
	}
	if n.resume(context.Background()) == nil {
		t.Fatal("configuration/start handoff repeated")
	}
	if err := n.close(); err != nil || b.closes != 1 || p.closes != 1 || a.closes != 1 {
		t.Fatal("original capability ownership not preserved", err)
	}
}

func TestCodeUpgradeDemandUncertainEffectNeverRetriesOrHandsOffInSameInvocation(t *testing.T) {
	for _, fault := range []string{"partial-fence", "config-error-after-effect", "readback-error-after-effect", "close-error"} {
		t.Run(fault, func(t *testing.T) {
			n, b, p, a, _ := demandTransitionFixture()
			switch fault {
			case "partial-fence":
				b.fenceErr = errors.New("partial durable attempt")
			case "config-error-after-effect":
				b.configErr = errors.New("API error after AUTO effect")
			case "readback-error-after-effect":
				b.afterFence = func() {
					n.backend = &demandReadbackFault{demandBackendFixture: b}
				}
			case "close-error":
				p.closeErr = errors.New("original custody close uncertain")
			}
			err := n.resume(context.Background())
			if !errors.Is(err, ErrCodeUpgradeUncertain) || n.resume(context.Background()) == nil || a.resumes != 0 {
				t.Fatal("uncertain operation retried or adopted", err)
			}
			if b.fences != 1 || b.configs > 1 || !b.attempt {
				t.Fatal("durable attempt lost")
			}
			n.close()
		})
	}
}

type demandReadbackFault struct{ *demandBackendFixture }

func (b *demandReadbackFault) check(context.Context) error {
	if b.auto {
		return errors.New("exact config readback refused")
	}
	return nil
}

func TestCodeUpgradeDemandExistingAttemptAndPhaseChangesForbidConfigMutation(t *testing.T) {
	for _, fault := range []string{"existing-attempt", "running", "start-pending", "nonzero-exit", "pair-revoked-after-fence", "phase-changed-after-fence"} {
		t.Run(fault, func(t *testing.T) {
			n, b, p, a, _ := demandTransitionFixture()
			switch fault {
			case "existing-attempt":
				b.attempt = true
			case "running":
				b.state = serviceState{state: 4, processID: 42, controls: 5}
			case "start-pending":
				b.state.state = serviceStartPending
			case "nonzero-exit":
				b.state.win32Exit = 1
			case "pair-revoked-after-fence":
				b.afterFence = func() { p.err = errors.New("complete pair changed") }
			case "phase-changed-after-fence":
				b.afterFence = func() { b.state.state = serviceStartPending }
			}
			if n.resume(context.Background()) == nil || b.configs != 0 || a.resumes != 0 {
				t.Fatal("ineligible state authorized configuration/start")
			}
			n.close()
		})
	}
}

func TestCodeUpgradeDemandCancellationAndReacquisitionFailureCannotReachStart(t *testing.T) {
	for _, fault := range []string{"canceled", "reacquire", "fresh-owner-refused"} {
		t.Run(fault, func(t *testing.T) {
			n, b, _, a, _ := demandTransitionFixture()
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			if fault == "canceled" {
				cancel()
			} else if fault == "reacquire" {
				n.openAutomatic = func(context.Context) (codeUpgradeResumeOwner, error) {
					return nil, errors.New("original service changed")
				}
			} else {
				a.err = errors.New("fresh AUTO custody refused")
			}
			if n.resume(ctx) == nil || a.resumes != 0 || b.configs > 1 {
				t.Fatal("failed fresh authority reached start")
			}
			if fault == "canceled" && (b.fences != 0 || b.configs != 0) {
				t.Fatal("canceled caller made effects")
			}
			n.close()
		})
	}
}

func TestCodeUpgradeDemandAttemptRejectsForeignTransactionTornAndNoncanonicalBytes(t *testing.T) {
	pins := []string{strings.Repeat("a", 64), strings.Repeat("b", 64), strings.Repeat("c", 64), strings.Repeat("d", 64)}
	exact := codeDemandAttemptBytes(pins[0], pins[1], pins[2], pins[3])
	if err := matchCodeDemandAttempt(exact, exact); err != nil {
		t.Fatal(err)
	}
	foreign := [][]byte{nil, []byte("{}"), exact[:len(exact)-1], append(append([]byte{}, exact...), '\n'),
		codeStartAttemptBytes(pins[0], pins[1], pins[2], pins[3])}
	for i := range pins {
		changed := append([]string{}, pins...)
		changed[i] = strings.Repeat("e", 64)
		foreign = append(foreign, codeDemandAttemptBytes(changed[0], changed[1], changed[2], changed[3]))
	}
	for _, raw := range foreign {
		if matchCodeDemandAttempt(raw, exact) == nil {
			t.Fatal("foreign or partial config attempt accepted")
		}
	}
	if matchCodeDemandAttempt(nil, codeDemandAttemptBytes("", pins[1], pins[2], pins[3])) == nil {
		t.Fatal("invalid authority accepted")
	}
}
