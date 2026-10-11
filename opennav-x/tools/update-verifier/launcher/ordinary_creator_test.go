package launcher

import (
	"context"
	"errors"
	"reflect"
	"strings"
	"testing"
)

type creatorFixture struct {
	events      []string
	failure     string
	rechecks    int
	resumeCount uint32
	spec        processSpec
}

func (f *creatorFixture) event(name string) error {
	f.events = append(f.events, name)
	if f.failure == name {
		return errors.New("injected " + name)
	}
	return nil
}

type creatorImage struct{ f *creatorFixture }
type creatorChild struct{ f *creatorFixture }

func (i creatorImage) Close() error { return i.f.event("close image") }
func (i creatorImage) recheck() error {
	i.f.rechecks++
	if i.f.rechecks == 1 {
		return i.f.event("before create")
	}
	return i.f.event("before resume")
}
func (i creatorImage) create(spec processSpec) (ordinaryChild, error) {
	i.f.spec = spec
	if err := i.f.event("create"); err != nil {
		return nil, err
	}
	return creatorChild{i.f}, nil
}
func (c creatorChild) Close() error          { return c.f.event("close child") }
func (c creatorChild) corroborate() error    { return c.f.event("corroborate") }
func (c creatorChild) abortSuspended() error { return c.f.event("abort suspended") }
func (c creatorChild) resume() (uint32, error) {
	return c.f.resumeCount, c.f.event("resume")
}

func TestOrdinaryCreatorCustodyAndNoRetry(t *testing.T) {
	for _, scenario := range []struct {
		name, failure string
		count         uint32
		want          []string
	}{
		{"success", "", 1, []string{"acquire", "before create", "create", "corroborate", "before resume", "resume", "close child", "close image"}},
		{"acquisition refusal", "acquire", 1, []string{"acquire"}},
		{"held metadata changed before create", "before create", 1, []string{"acquire", "before create", "close image"}},
		{"uncertain create", "create", 1, []string{"acquire", "before create", "create", "close image"}},
		{"raw identity or image differs", "corroborate", 1, []string{"acquire", "before create", "create", "corroborate", "abort suspended", "close child", "close image"}},
		{"held metadata changed before resume", "before resume", 1, []string{"acquire", "before create", "create", "corroborate", "before resume", "abort suspended", "close child", "close image"}},
		{"uncertain resume", "resume", 1, []string{"acquire", "before create", "create", "corroborate", "before resume", "resume", "close child", "close image"}},
		{"already resumed", "", 0, []string{"acquire", "before create", "create", "corroborate", "before resume", "resume", "close child", "close image"}},
		{"extra suspension", "", 2, []string{"acquire", "before create", "create", "corroborate", "before resume", "resume", "close child", "close image"}},
	} {
		t.Run(scenario.name, func(t *testing.T) {
			f := &creatorFixture{failure: scenario.failure, resumeCount: scenario.count}
			spec := processSpec{executable: "C:/owned/app/opencpn.exe", directory: "C:/owned/app", expectedImageHash: strings.Repeat("a", 64), arguments: []string{"--legacy"}}
			err := startOrdinary(spec, func(got processSpec) (ordinaryImage, error) {
				if !reflect.DeepEqual(got, spec) {
					t.Fatal("acquisition lost the original managed expectation")
				}
				if err := f.event("acquire"); err != nil {
					return nil, err
				}
				return creatorImage{f}, nil
			})
			if (err == nil) != (scenario.name == "success") || !reflect.DeepEqual(f.events, scenario.want) {
				t.Fatalf("custody ordering/retry: events=%v err=%v", f.events, err)
			}
		})
	}
}

func TestOrdinaryCreatorRejectsMissingManagedExpectationBeforeAcquire(t *testing.T) {
	for _, spec := range []processSpec{{}, {expectedImageHash: strings.Repeat("A", 64)},
		{expectedImageHash: strings.Repeat("a", 64), input: []byte("challenge")},
		{expectedImageHash: strings.Repeat("a", 64), limit: 1}} {
		if startOrdinary(spec, func(processSpec) (ordinaryImage, error) {
			t.Fatal("invalid ordinary request acquired native ownership")
			return nil, nil
		}) == nil {
			t.Fatal("invalid ordinary request accepted")
		}
	}
}

func TestOrdinaryCreatorFailurePropagatesAcrossEntryRoutes(t *testing.T) {
	for _, route := range []string{"xnav", "legacy", "safe", "later", "bootstrap", "recovery", "fallback"} {
		t.Run(route, func(t *testing.T) {
			f := newLaunchFixture(t)
			f.p.startError = errors.New("uncertain native creation/resume")
			args := []string{"--xnav"}
			wantWaits := 0
			switch route {
			case "legacy":
				args = []string{"--legacy"}
			case "safe":
				args = []string{"--safe-mode"}
			case "later":
				wantWaits = 1 // default inert prompt result selects Later
			case "bootstrap":
				delete(f.p.files, fixtureRoot+"/generations/"+fixtureGeneration+"/app/update-trust.json")
				receipt := f.p.files[fixtureRoot+"/known-good/"+fixtureGeneration+".receipt"]
				args = []string{"--boat-bootstrap=" + fixtureGeneration + ":" + digest(receipt)}
			case "recovery", "fallback":
				f.publishSuccessor()
				wantWaits = 1
				f.p.onWait = func(_ context.Context, _ processSpec) (processResult, error) {
					delete(f.p.files, fixtureRoot+"/update-pending.json")
					f.state.Current, f.state.Previous = fixtureGeneration, ""
					f.storeState()
					return processResult{}, nil
				}
			default:
				delete(f.p.files, fixtureRoot+"/generations/"+fixtureGeneration+"/app/update-trust.json")
			}
			var err error
			if route == "fallback" {
				previous, loadErr := loadGeneration(f.p, selectedLayout(layout{root: fixtureRoot}, fixtureGeneration))
				if loadErr != nil {
					t.Fatal(loadErr)
				}
				defer previous.close()
				err = launchInstalled(context.Background(), f.p, previous, f.state.Stock, f.candidate.Policy)
			} else {
				err = run(context.Background(), args, f.s)
			}
			if !errors.Is(err, f.p.startError) || len(f.p.starts) != 1 || len(f.p.waits) != wantWaits || f.p.lockHeld {
				t.Fatalf("creator failure retried/switched route: err=%v starts=%d waits=%d", err, len(f.p.starts), len(f.p.waits))
			}
			got := f.p.starts[0]
			if got.expectedImageHash != digest(f.p.files[got.executable]) || len(got.arguments) != 1 {
				t.Fatal("ordinary entry lost the managed image hash or fixed mode")
			}
		})
	}
}
