package brokerenroll

import (
	"context"
	"errors"
	"example.com/opennav-update-verifier/brokerpipe"
	"strings"
	"testing"
)

type startBackendFixture struct {
	finalizeBackendFixture
	fenceErr, startErr    error
	fences, starts, binds int
}

func (b *startBackendFixture) fence(context.Context) error {
	b.fences++
	*b.events = append(*b.events, "fence.flush.readback")
	return b.fenceErr
}
func (b *startBackendFixture) start() error {
	b.starts++
	*b.events = append(*b.events, "start")
	if b.startErr == nil {
		b.state = serviceState{state: 4, processID: 42, controls: 5}
	}
	return b.startErr
}
func (b *startBackendFixture) bindProcess(pid uint32) (maintenanceProcess, error) {
	b.binds++
	return b.finalizeBackendFixture.bindProcess(pid)
}
func TestCodeUpgradeStoppedStartFenceBeforeSingleStartAndSameProcessFinalizer(t *testing.T) {
	events := []string{}
	b := &startBackendFixture{finalizeBackendFixture: finalizeBackendFixture{state: serviceState{state: serviceStopped}, events: &events}}
	p := &finalizePublicationFixture{events: &events}
	n := &codeUpgradeStarter{backend: b, publication: p, call: func(_ context.Context, q brokerpipe.MaintenanceRequest) (brokerpipe.MaintenanceResult, error) {
		return finalizeResponse(q), nil
	}}
	n.self = n
	if err := n.resume(context.Background()); err != nil {
		t.Fatal(err)
	}
	if b.fences != 1 || b.starts != 1 || b.binds != 1 || p.effects != 1 || events[0] != "fence.flush.readback" || events[1] != "start" {
		t.Fatal(events)
	}
	if n.resume(context.Background()) == nil {
		t.Fatal("reused start")
	}
	b.processErr = errors.New("original process lost")
	if n.check(context.Background()) == nil {
		t.Fatal("adopted replacement")
	}
	n.close()
}
func TestCodeUpgradeStoppedStartFenceAndStartErrorsConsumeAttempt(t *testing.T) {
	for _, phase := range []string{"fence", "already-running", "timeout"} {
		t.Run(phase, func(t *testing.T) {
			events := []string{}
			b := &startBackendFixture{finalizeBackendFixture: finalizeBackendFixture{state: serviceState{state: serviceStopped}, events: &events}}
			if phase == "fence" {
				b.fenceErr = errors.New("partial fence")
			} else {
				b.startErr = errors.New(phase)
			}
			p := &finalizePublicationFixture{events: &events}
			n := &codeUpgradeStarter{backend: b, publication: p}
			n.self = n
			if n.resume(context.Background()) == nil || n.resume(context.Background()) == nil {
				t.Fatal("failure reused")
			}
			if b.fences != 1 || b.starts > 1 || b.binds != 0 || p.effects != 0 {
				t.Fatal("unexpected effects")
			}
			n.close()
		})
	}
}
func TestCodeUpgradeStartAttemptBindsEveryDigest(t *testing.T) {
	a, b, c, d := strings.Repeat("a", 64), strings.Repeat("b", 64), strings.Repeat("c", 64), strings.Repeat("d", 64)
	raw := string(codeStartAttemptBytes(a, b, c, d))
	for i := 0; i < 4; i++ {
		v := []string{a, b, c, d}
		v[i] = strings.Repeat("e", 64)
		if raw == string(codeStartAttemptBytes(v[0], v[1], v[2], v[3])) {
			t.Fatal("unbound digest")
		}
	}
	if codeStartAttemptBytes("", b, c, d) != nil {
		t.Fatal("invalid digest")
	}
}

func (b *startBackendFixture) finalization(context.Context) (codeFinalizeBackend, error) {
	return b, nil
}
