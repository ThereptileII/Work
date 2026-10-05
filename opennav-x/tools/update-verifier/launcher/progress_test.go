package launcher

import (
	"context"
	"errors"
	"reflect"
	"strings"
	"testing"
	"time"

	verifier "example.com/opennav-update-verifier"
	"example.com/opennav-update-verifier/releasepolicy"
)

func TestDownloadProgressCancellationNeverStartsInstaller(t *testing.T) {
	for _, scenario := range []string{"cannot start", "already exited", "cancel during preparation", "exit at completion", "exit during finish", "finish error", "artifact with error"} {
		t.Run(scenario, func(t *testing.T) {
			f := newLaunchFixture(t)
			progress := &fakeProgress{exited: make(chan struct{})}
			f.p.progressWindow = progress
			if scenario == "cannot start" {
				f.p.progressError = errors.New("fixture start failure")
			}
			if scenario == "already exited" {
				close(progress.exited)
			}
			if scenario == "exit during finish" {
				progress.onFinish = func() { close(progress.exited) }
			}
			if scenario == "finish error" {
				progress.finishError = errors.New("fixture nonzero exit")
			}
			f.p.onWait = func(_ context.Context, spec processSpec) (processResult, error) {
				if strings.HasSuffix(spec.executable, "skager-update-prompt.exe") {
					return processResult{code: 10}, nil
				}
				t.Fatal("cancellation started transaction process")
				return processResult{}, nil
			}
			f.s.prepare = func(ctx context.Context, _ verifier.PrepareConfig, _ releasepolicy.OpenCpn, _ verifier.ConsentIdentity) (preparation, error) {
				f.prepares++
				deadline, ok := ctx.Deadline()
				if !ok || time.Until(deadline) > 30*time.Minute || progress.closed {
					t.Fatal("preparation missing bounded live progress")
				}
				switch scenario {
				case "cancel during preparation":
					close(progress.exited)
					select {
					case <-ctx.Done():
					case <-time.After(time.Second):
						t.Fatal("prompt exit did not cancel network context")
					}
					return nil, ctx.Err()
				case "exit at completion":
					close(progress.exited)
				case "artifact with error":
					return f.prepared, errors.New("fixture preparation error")
				}
				return f.prepared, nil
			}
			if err := run(context.Background(), nil, f.s); err != nil {
				t.Fatal(err)
			}
			if len(f.p.starts) != 1 || len(f.p.waits) != 1 {
				t.Fatal("cancel/error did not start current exactly once")
			}
			if len(f.p.progressSpecs) != 1 {
				t.Fatal("unexpected progress start count")
			}
			spec := f.p.progressSpecs[0]
			app := fixtureRoot + "/generations/" + fixtureGeneration + "/app"
			if spec.executable != app+"/skager-update-prompt.exe" || spec.directory != app || !reflect.DeepEqual(spec.arguments, []string{"--download-progress"}) || len(spec.input) != 0 {
				t.Fatal("progress protocol included unexpected path, arguments or data")
			}
			if scenario == "cannot start" || scenario == "already exited" {
				if f.prepares != 0 {
					t.Fatal("unavailable progress started preparation")
				}
			} else if f.prepares != 1 {
				t.Fatal("unexpected preparation count")
			}
			if scenario != "cannot start" && !progress.closed {
				t.Fatal("progress handle leaked")
			}
			if f.prepares != 0 && scenario != "cancel during preparation" && !f.prepared.closed {
				t.Fatal("discarded artifact custody leaked")
			}
		})
	}
}

func TestDownloadProgressParentCancellationAtCompletionDiscardsArtifact(t *testing.T) {
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	progress := &fakeProgress{exited: make(chan struct{}), onFinish: cancel}
	platform := &fakePlatform{progressWindow: progress}
	prepared := &fakePreparation{}
	result, err := prepareWithProgress(ctx, platform, "C:/fixture/app", func(context.Context) (preparation, error) { return prepared, nil })
	if err == nil || result != nil || !prepared.closed || !progress.closed {
		t.Fatal("parent cancellation crossed installation gate")
	}
}

func TestDownloadProgressSuccessfulCustodyIsTransferredAfterCompletion(t *testing.T) {
	progress := &fakeProgress{exited: make(chan struct{})}
	platform := &fakePlatform{progressWindow: progress}
	prepared := &fakePreparation{}
	result, err := prepareWithProgress(context.Background(), platform, "C:/fixture/app", func(context.Context) (preparation, error) { return prepared, nil })
	if err != nil || result != prepared || prepared.closed || !progress.finished || !progress.closed {
		t.Fatalf("successful preparation custody/completion: %v", err)
	}
	_ = result.close()
}
