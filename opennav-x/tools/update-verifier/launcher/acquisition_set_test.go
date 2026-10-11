package launcher

import (
	"context"
	"errors"
	"strings"
	"testing"
	"time"
)

func TestLauncherRetainsBothAcquisitionOwnersUntilTimedOutInstallerExits(t *testing.T) {
	f := newLaunchFixture(t)
	installerDone := make(chan struct{})
	waitReturned := make(chan struct{})
	f.p.onWait = func(_ context.Context, spec processSpec) (processResult, error) {
		if strings.HasSuffix(spec.executable, "skager-update-prompt.exe") {
			return processResult{code: 10}, nil
		}
		if spec.executable != f.prepared.path() {
			t.Error("unexpected child after confirmed preparation")
		}
		close(waitReturned)
		return processResult{timedOut: true, done: installerDone}, errors.New("inert installer timeout")
	}
	result := make(chan error, 1)
	go func() { result <- run(context.Background(), nil, f.s) }()
	select {
	case <-waitReturned:
	case <-time.After(2 * time.Second):
		t.Fatal("installer fixture did not reach wait")
	}
	if f.prepared.installerClosed || f.prepared.recoveryClosed {
		t.Fatal("timed-out live installer lost an artifact owner")
	}
	select {
	case <-result:
		t.Fatal("launcher returned before exact installer exit")
	default:
	}
	close(installerDone)
	select {
	case err := <-result:
		if err == nil {
			t.Fatal("timeout accepted")
		}
	case <-time.After(2 * time.Second):
		t.Fatal("launcher did not finish after exact installer exit")
	}
	if !f.prepared.installerClosed || !f.prepared.recoveryClosed || len(f.p.starts) != 0 {
		t.Fatal("timeout failed set cleanup or started application")
	}
}

func TestAcquisitionCleanupFailuresRemainVisibleAtCallerBoundaries(t *testing.T) {
	cleanup := errors.New("inert acquisition-set cleanup failure")
	t.Run("progress-refusal", func(t *testing.T) {
		prepared := &fakePreparation{closeError: cleanup}
		progress := &fakeProgress{exited: make(chan struct{}), finishError: errors.New("inert prompt failure")}
		result, err := prepareWithProgress(context.Background(), &fakePlatform{progressWindow: progress}, "C:/fixture/app", func(context.Context) (preparation, error) { return prepared, nil })
		if result != nil || !errors.Is(err, cleanup) || !prepared.installerClosed || !prepared.recoveryClosed {
			t.Fatal("progress refusal hid cleanup or skipped an owner")
		}
	})
	t.Run("installer-refusal", func(t *testing.T) {
		f := newLaunchFixture(t)
		f.prepared.closeError = cleanup
		f.p.onWait = func(_ context.Context, spec processSpec) (processResult, error) {
			if strings.HasSuffix(spec.executable, "skager-update-prompt.exe") {
				return processResult{code: 10}, nil
			}
			return processResult{code: 1}, nil
		}
		err := run(context.Background(), nil, f.s)
		if !errors.Is(err, cleanup) || !strings.Contains(err.Error(), "supervised installation did not complete") || !f.prepared.installerClosed || !f.prepared.recoveryClosed || len(f.p.starts) != 0 {
			t.Fatal("installer refusal lost original error or set cleanup")
		}
	})
}
