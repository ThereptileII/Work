package launcher

import (
	"bytes"
	"context"
	"errors"
	"os"
	"reflect"
	"strings"
	"testing"

	verifier "example.com/opennav-update-verifier"
	"example.com/opennav-update-verifier/releasepolicy"
)

func armedRouteFixture(t *testing.T, route string) (*launchFixture, []string) {
	t.Helper()
	f := newLaunchFixture(t)
	args := []string{"--xnav"}
	switch route {
	case "legacy":
		args[0] = "--legacy"
	case "safe":
		args[0] = "--safe-mode"
	case "offline":
		f.s.check = func(context.Context, verifier.PrepareConfig, releasepolicy.OpenCpn) (verifier.UpdateCandidate, error) {
			f.checks++
			return verifier.UpdateCandidate{}, errors.New("offline")
		}
	case "qualification":
		delete(f.p.files, fixtureRoot+"/known-good/"+fixtureGeneration+".receipt")
	case "initialize":
		args[0] = "--initialize-trust"
	case "bootstrap", "bootstrap qualification":
		delete(f.p.files, fixtureRoot+"/generations/"+fixtureGeneration+"/app/update-trust.json")
		expectation := digest(f.p.files[fixtureRoot+"/known-good/"+fixtureGeneration+".receipt"])
		if route == "bootstrap qualification" {
			delete(f.p.files, fixtureRoot+"/known-good/"+fixtureGeneration+".receipt")
			expectation = "absent"
		}
		args[0] = "--boat-bootstrap=" + fixtureGeneration + ":" + expectation
	case "recovery", "fallback":
		f.publishSuccessor()
		if route == "fallback" {
			f.p.onWait = func(context.Context, processSpec) (processResult, error) {
				delete(f.p.files, fixtureRoot+"/update-pending.json")
				f.state.Current, f.state.Previous = fixtureGeneration, ""
				f.storeState()
				return processResult{}, nil
			}
		}
	case "installer":
		f.p.onWait = func(context.Context, processSpec) (processResult, error) { return processResult{code: 10}, nil }
	}
	return f, args
}

func TestArmedUpdateBlocksEveryEntryWithoutReadingOrChangingMarker(t *testing.T) {
	for _, route := range []string{"xnav", "legacy", "safe", "later", "offline", "qualification", "initialize", "bootstrap", "bootstrap qualification", "recovery", "fallback", "installer"} {
		for _, content := range []string{"empty", "garbage", "oversized", "claimed dead owner", "access error", "path error"} {
			t.Run(route+"/"+content, func(t *testing.T) {
				f, args := armedRouteFixture(t, route)
				marker := []byte("not a journal")
				switch content {
				case "empty":
					marker = nil
				case "oversized":
					marker = bytes.Repeat([]byte("x"), 1<<20)
				case "claimed dead owner":
					marker = []byte(`{"ownerPid":0,"ownerDead":true,"armed":false}`)
				case "access error":
					f.p.armedError = os.ErrPermission
				case "path error":
					f.p.armedError = &os.PathError{Op: "attributes", Path: fixtureRoot, Err: os.ErrNotExist}
				}
				name := fixtureRoot + "/signed-update-armed.jsonl"
				if f.p.armedError == nil {
					f.p.files[name] = marker
				}
				before := make(map[string][]byte, len(f.p.files))
				for key, data := range f.p.files {
					before[key] = data
				}
				if err := run(context.Background(), args, f.s); !errors.Is(err, errArmedUpdateBarrier) {
					t.Fatalf("armed or uncertain entry was not refused: %v", err)
				}
				if len(f.p.starts)+len(f.p.waits)+len(f.p.progressSpecs)+f.checks+f.prepares != 0 || f.p.lockHeld {
					t.Fatal("armed entry performed startup/update actions")
				}
				if !reflect.DeepEqual(before, f.p.files) {
					t.Fatal("armed refusal changed installation or marker bytes")
				}
			})
		}
	}
}

func TestArmedUpdatePublicationUnderLockBlocksOrdinaryBootstrapAndInitialization(t *testing.T) {
	for _, route := range []string{"xnav", "legacy", "safe", "later", "offline", "fallback", "bootstrap", "bootstrap qualification", "initialize"} {
		t.Run(route, func(t *testing.T) {
			f, args := armedRouteFixture(t, route)
			if route == "xnav" {
				delete(f.p.files, fixtureRoot+"/generations/"+fixtureGeneration+"/app/update-trust.json")
			}
			f.p.onLock = func() { f.p.files[fixtureRoot+"/"+armedUpdateMarker] = nil }
			if err := run(context.Background(), args, f.s); !errors.Is(err, errArmedUpdateBarrier) {
				t.Fatalf("late armed marker bypassed lock recheck: %v", err)
			}
			wantWaits := 0
			if route == "later" || route == "fallback" {
				wantWaits = 1
			}
			if len(f.p.starts) != 0 || len(f.p.waits) != wantWaits || f.p.lockHeld {
				t.Fatal("late armed marker crossed a locked startup boundary")
			}
			if _, exists := f.p.files[fixtureRoot+"/"+armedUpdateMarker]; !exists {
				t.Fatal("refusal removed armed marker")
			}
			if _, exists := f.p.files[fixtureRoot+"/update-trust-initialized-beta"]; exists {
				t.Fatal("trust initialization proceeded while armed")
			}
		})
	}
}

func TestArmedUpdatePublicationBlocksExternalCreationStages(t *testing.T) {
	for _, stage := range []string{"offer prompt", "progress", "installer", "launch pending", "qualification", "recovery"} {
		t.Run(stage, func(t *testing.T) {
			route := "installer"
			if stage == "qualification" || stage == "recovery" {
				route = stage
			}
			f, args := armedRouteFixture(t, route)
			arm := func() { f.p.files[fixtureRoot+"/"+armedUpdateMarker] = []byte("armed") }
			if stage == "offer prompt" {
				check := f.s.check
				f.s.check = func(ctx context.Context, c verifier.PrepareConfig, o releasepolicy.OpenCpn) (verifier.UpdateCandidate, error) {
					candidate, err := check(ctx, c, o)
					arm()
					return candidate, err
				}
			} else if stage == "qualification" || stage == "recovery" {
				checks := 0
				f.p.onArmedCheck = func() {
					checks++
					if checks == 2 {
						arm()
					}
				}
			} else {
				f.p.onWait = func(_ context.Context, spec processSpec) (processResult, error) {
					if strings.HasSuffix(spec.executable, "skager-update-prompt.exe") {
						if stage == "progress" {
							arm()
						}
						return processResult{code: 10}, nil
					}
					f.publishSuccessor()
					arm()
					return processResult{}, nil
				}
				if stage == "installer" {
					prepare := f.s.prepare
					f.s.prepare = func(ctx context.Context, c verifier.PrepareConfig, o releasepolicy.OpenCpn, i verifier.ConsentIdentity) (preparation, error) {
						prepared, err := prepare(ctx, c, o, i)
						arm()
						return prepared, err
					}
				}
			}
			if err := run(context.Background(), args, f.s); !errors.Is(err, errArmedUpdateBarrier) {
				t.Fatalf("armed publication did not stop %s: %v", stage, err)
			}
			wantWaits, wantProgress := 0, 0
			if stage == "progress" || stage == "installer" {
				wantWaits = 1
			}
			if stage == "installer" || stage == "launch pending" {
				wantProgress = 1
			}
			if stage == "launch pending" {
				wantWaits = 2
			}
			if len(f.p.starts) != 0 || len(f.p.waits) != wantWaits || len(f.p.progressSpecs) != wantProgress {
				t.Fatal("armed publication reached a later process creation")
			}
			if wantProgress != 0 && !f.prepared.closed {
				t.Fatal("refused installer/supervisor released prepared-artifact custody incorrectly")
			}
		})
	}
}

func TestArmedUpdateUncertaintyNeverFallsBackAfterPromptOrProgress(t *testing.T) {
	for _, stage := range []string{"prompt", "progress"} {
		t.Run(stage, func(t *testing.T) {
			f, args := armedRouteFixture(t, "installer")
			if stage == "prompt" {
				checks := 0
				f.p.onArmedCheck = func() {
					checks++
					f.p.armedError = nil
					if checks == 2 {
						f.p.armedError = os.ErrPermission
					}
				}
			} else {
				f.p.progressError = errArmedUpdateBarrier
			}
			if err := run(context.Background(), args, f.s); !errors.Is(err, errArmedUpdateBarrier) {
				t.Fatalf("uncertain armed observation was not fatal: %v", err)
			}
			if len(f.p.starts) != 0 || f.prepares != 0 {
				t.Fatal("uncertain armed observation fell back or prepared an update")
			}
		})
	}
}

type armingOrdinaryImage struct {
	creatorImage
	platform *fakePlatform
	at       int
}

func (image armingOrdinaryImage) recheck() error {
	err := image.creatorImage.recheck()
	if image.f.rechecks == image.at {
		image.platform.files[fixtureRoot+"/"+armedUpdateMarker] = nil
	}
	return err
}

func TestArmedUpdateOrdinaryCreatorRefusesBeforeCreateAndResume(t *testing.T) {
	for _, at := range []int{1, 2} {
		f := newLaunchFixture(t)
		creator := &creatorFixture{resumeCount: 1}
		spec := processSpec{expectedImageHash: strings.Repeat("a", 64)}
		err := startOrdinary(spec, func(processSpec) (ordinaryImage, error) {
			return unarmedOrdinaryImage{ordinaryImage: armingOrdinaryImage{creatorImage{creator}, f.p, at}, platform: f.p, root: fixtureRoot}, nil
		})
		want := []string{"before create", "close image"}
		if at == 2 {
			want = []string{"before create", "create", "corroborate", "before resume", "abort suspended", "close child", "close image"}
		}
		if !errors.Is(err, errArmedUpdateBarrier) || !reflect.DeepEqual(creator.events, want) {
			t.Fatalf("armed ordinary custody ordering at %d: %v %v", at, creator.events, err)
		}
	}
}
