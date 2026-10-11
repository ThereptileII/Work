package launcher

import (
	"context"
	"errors"
	"strings"
	"testing"
	"time"

	verifier "example.com/opennav-update-verifier"
	"example.com/opennav-update-verifier/releasepolicy"
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
	cleanup := verifier.ErrArtifactCleanup
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

func TestLauncherPreparationCleanupFailureWithholdsStartupAndRedactsDetails(t *testing.T) {
	for _, scenario := range []string{"acquisition-cleanup", "progress-cleanup", "artifact-and-error"} {
		t.Run(scenario, func(t *testing.T) {
			f := newLaunchFixture(t)
			private := errors.New("private-artifact-path-and-transport-token")
			f.p.progressWindow = &fakeProgress{exited: make(chan struct{})}
			f.p.onWait = func(_ context.Context, spec processSpec) (processResult, error) {
				if !strings.HasSuffix(spec.executable, "skager-update-prompt.exe") {
					t.Fatal("preparation cleanup uncertainty started a transaction")
				}
				return processResult{code: 10}, nil
			}
			f.s.prepare = func(context.Context, verifier.PrepareConfig, releasepolicy.OpenCpn, verifier.ConsentIdentity) (preparation, error) {
				f.prepares++
				switch scenario {
				case "acquisition-cleanup":
					return nil, errors.Join(private, verifier.ErrArtifactCleanup)
				case "progress-cleanup":
					f.p.progressWindow.finishError = private
					f.prepared.closeError = private
					return f.prepared, nil
				default:
					return f.prepared, errors.Join(private, verifier.ErrArtifactCleanup)
				}
			}
			err := run(context.Background(), nil, f.s)
			if !errors.Is(err, verifier.ErrArtifactCleanup) || strings.Contains(err.Error(), private.Error()) ||
				len(f.p.starts) != 0 || len(f.p.waits) != 1 || f.prepares != 1 || !f.p.progressWindow.closed {
				t.Fatalf("actual caller swallowed cleanup, leaked details or started a child: %v", err)
			}
			if scenario != "acquisition-cleanup" && (!f.prepared.installerClosed || !f.prepared.recoveryClosed) {
				t.Fatal("preparation refusal skipped an acquisition owner")
			}
		})
	}
}

func TestLauncherCleanPreparationRefusalStillStartsCurrent(t *testing.T) {
	for _, refusal := range []error{context.Canceled, errors.New("offline transport"), errors.New("recovery sidecar unavailable"), errors.New(verifier.ErrArtifactCleanup.Error())} {
		t.Run(refusal.Error(), func(t *testing.T) {
			f := newLaunchFixture(t)
			f.p.onWait = func(_ context.Context, spec processSpec) (processResult, error) {
				if !strings.HasSuffix(spec.executable, "skager-update-prompt.exe") {
					t.Fatal("clean preparation refusal started an installer")
				}
				return processResult{code: 10}, nil
			}
			f.s.prepare = func(context.Context, verifier.PrepareConfig, releasepolicy.OpenCpn, verifier.ConsentIdentity) (preparation, error) {
				f.prepares++
				return nil, refusal
			}
			if err := run(context.Background(), nil, f.s); err != nil || len(f.p.starts) != 1 || len(f.p.waits) != 1 || f.prepares != 1 {
				t.Fatalf("clean refusal changed ordinary startup: %v", err)
			}
		})
	}
}
