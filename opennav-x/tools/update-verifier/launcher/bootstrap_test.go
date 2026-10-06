package launcher

import (
	"context"
	"strings"
	"testing"
)

func TestPrivateBootstrapReceiptSelection(t *testing.T) {
	receipt := fixtureRoot + "/known-good/" + fixtureGeneration + ".receipt"
	for _, scenario := range []string{"direct", "absent", "created", "removed", "replaced", "generation", "pending", "trust", "malformed", "missing"} {
		t.Run(scenario, func(t *testing.T) {
			f := newLaunchFixture(t)
			delete(f.p.files, fixtureRoot+"/generations/"+fixtureGeneration+"/app/update-trust.json")
			expected := digest(f.p.files[receipt])
			if scenario == "absent" || scenario == "created" {
				delete(f.p.files, receipt)
				expected = "absent"
			}
			if scenario == "malformed" {
				expected = "ABSENT"
			}
			if scenario == "missing" {
				expected = ""
			}
			f.p.onLock = func() {
				switch scenario {
				case "created":
					f.p.files[receipt] = []byte("late receipt")
				case "removed":
					delete(f.p.files, receipt)
				case "replaced":
					f.p.files[receipt] = []byte("changed receipt")
				case "generation":
					f.state.Current = nextGeneration
					f.storeState()
				case "pending":
					f.p.files[fixtureRoot+"/update-pending.json"] = []byte("malformed pending still refuses")
				case "trust":
					f.p.files[fixtureRoot+"/generations/"+fixtureGeneration+"/app/update-trust.json"] = []byte("malformed trust still refuses")
				}
			}
			f.p.onWait = func(_ context.Context, spec processSpec) (processResult, error) {
				if f.p.lockHeld {
					t.Fatal("supervisor must acquire its own lock")
				}
				n := len(spec.arguments)
				if n != 12 || spec.arguments[n-2] != "-BootstrapExpectation" || spec.arguments[n-1] != fixtureGeneration+":absent" || spec.arguments[9] != "QualifyCurrent" {
					t.Fatalf("unbound supervisor route: %#v", spec)
				}
				return processResult{}, nil
			}
			err := run(context.Background(), []string{"--boat-bootstrap=" + fixtureGeneration + ":" + expected}, f.s)
			if scenario == "direct" {
				if err != nil || len(f.p.starts) != 1 || len(f.p.waits) != 0 {
					t.Fatalf("direct route: %v", err)
				}
				if len(f.p.starts[0].arguments) != 1 || f.p.starts[0].arguments[0] != "--xnav" {
					t.Fatal("private request leaked into app arguments")
				}
			} else if scenario == "absent" {
				if err != nil || len(f.p.starts) != 0 || len(f.p.waits) != 1 {
					t.Fatalf("qualification route: %v", err)
				}
			} else if err == nil || len(f.p.starts) != 0 || len(f.p.waits) != 0 {
				t.Fatalf("changed expectation gained process creation: %v", err)
			}
			if f.checks != 0 || f.prepares != 0 {
				t.Fatal("bootstrap performed update networking")
			}
		})
	}
}
func TestPrivateBootstrapRejectsUnboundRequests(t *testing.T) {
	for _, request := range []string{"", fixtureGeneration, nextGeneration + ":absent", fixtureGeneration + ":" + strings.Repeat("a", 64) + ":extra"} {
		f := newLaunchFixture(t)
		if run(context.Background(), []string{"--boat-bootstrap=" + request}, f.s) == nil || len(f.p.starts)+len(f.p.waits) > 0 {
			t.Fatal("invalid binding accepted")
		}
	}
}

func TestPrivateBootstrapSupervisorRefusalNeverFallsBack(t *testing.T) {
	f := newLaunchFixture(t)
	delete(f.p.files, fixtureRoot+"/generations/"+fixtureGeneration+"/app/update-trust.json")
	delete(f.p.files, fixtureRoot+"/known-good/"+fixtureGeneration+".receipt")
	f.p.onWait = func(_ context.Context, spec processSpec) (processResult, error) {
		// The supervisor observes a late receipt after acquiring its own lock and
		// refuses. The launcher must not reinterpret it as permission for direct start.
		f.p.files[fixtureRoot+"/known-good/"+fixtureGeneration+".receipt"] = []byte("late receipt")
		return processResult{code: 1}, nil
	}
	if run(context.Background(), []string{"--boat-bootstrap=" + fixtureGeneration + ":absent"}, f.s) == nil || len(f.p.starts) != 0 || len(f.p.waits) != 1 || f.checks != 0 {
		t.Fatal("supervisor refusal switched ancestry or attempted recovery")
	}
}
