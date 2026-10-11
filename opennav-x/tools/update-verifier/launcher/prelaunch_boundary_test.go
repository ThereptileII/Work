package launcher

import (
	"context"
	"encoding/json"
	"errors"
	"strings"
	"testing"
)

// These exercise the existing installed-launch boundary with inert process
// doubles. They do not supply commissioning authority or emulate Windows file
// sharing, loader closure, profile custody, or authenticated broker IPC.
func installedBoundaryFixture(t *testing.T) (*launchFixture, *generation) {
	t.Helper()
	f := newLaunchFixture(t)
	previous, err := loadGeneration(f.p, selectedLayout(layout{root: fixtureRoot}, fixtureGeneration))
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(previous.close)
	f.publishSuccessor()
	return f, previous
}

func TestInstalledSuccessorRejectsChangedIdentityAndConsumedAttempt(t *testing.T) {
	for _, scenario := range []string{"current generation", "stock identity", "candidate generation", "candidate commit", "candidate package", "candidate executable", "previous generation", "consumed attempt", "session without attempt"} {
		t.Run(scenario, func(t *testing.T) {
			f, previous := installedBoundaryFixture(t)
			stock := f.state.Stock
			pendingPath := fixtureRoot + "/update-pending.json"
			var pending pendingRecord
			if err := json.Unmarshal(f.p.files[pendingPath], &pending); err != nil {
				t.Fatal(err)
			}
			switch scenario {
			case "current generation":
				f.state.Current = fixtureGeneration
			case "stock identity":
				f.state.Stock.SHA256 = strings.Repeat("f", 64)
			case "candidate generation":
				pending.Candidate.Generation = strings.Repeat("f", 32)
			case "candidate commit":
				pending.Candidate.Commit = strings.Repeat("f", 40)
			case "candidate package":
				pending.Candidate.PackageSHA256 = strings.Repeat("f", 64)
			case "candidate executable":
				pending.Candidate.ExecutableSHA256 = strings.Repeat("f", 64)
			case "previous generation":
				pending.Previous.Generation = strings.Repeat("f", 32)
			case "consumed attempt":
				pending.Attempts = 1
				pending.Session = strings.Repeat("f", 32)
			case "session without attempt":
				pending.Session = strings.Repeat("f", 32)
			}
			f.storeState()
			f.p.files[pendingPath] = jsonBytes(t, pending)
			if err := launchInstalled(context.Background(), f.p, previous, stock, f.candidate.Policy); err == nil {
				t.Fatal("changed identity or consumed launch was accepted")
			}
			if len(f.p.waits) != 0 || len(f.p.starts) != 0 {
				t.Fatal("rejected handoff reached a process-creation boundary")
			}
			if _, exists := f.p.files[pendingPath]; !exists {
				t.Fatal("rejected handoff discarded pending recovery evidence")
			}
		})
	}
}

func TestInstalledUncertainSupervisorNeverRetriesOrFallsBack(t *testing.T) {
	for _, scenario := range []string{"lost response", "timeout", "nonzero exit"} {
		t.Run(scenario, func(t *testing.T) {
			f, previous := installedBoundaryFixture(t)
			f.p.onWait = func(_ context.Context, spec processSpec) (processResult, error) {
				if !hasAction(spec, "LaunchPending") {
					t.Fatal("unexpected supervisor action")
				}
				// Even a plausible completed rollback cannot turn an uncertain
				// process outcome into permission to start the old generation.
				delete(f.p.files, fixtureRoot+"/update-pending.json")
				f.state.Current = fixtureGeneration
				f.state.Previous = ""
				f.storeState()
				switch scenario {
				case "lost response":
					return processResult{}, errors.New("supervisor outcome unavailable")
				case "timeout":
					done := make(chan struct{})
					close(done)
					return processResult{timedOut: true, done: done}, nil
				default:
					return processResult{code: 1}, nil
				}
			}
			if err := launchInstalled(context.Background(), f.p, previous, f.state.Stock, f.candidate.Policy); err == nil {
				t.Fatal("uncertain supervisor outcome accepted")
			}
			if len(f.p.waits) != 1 || len(f.p.starts) != 0 {
				t.Fatal("uncertain supervisor outcome caused retry or fallback")
			}
		})
	}
}

func TestInstalledFallbackRechecksPointerUnderCreationLock(t *testing.T) {
	f, previous := installedBoundaryFixture(t)
	f.p.onWait = func(_ context.Context, spec processSpec) (processResult, error) {
		if !hasAction(spec, "LaunchPending") {
			t.Fatal("unexpected supervisor action")
		}
		delete(f.p.files, fixtureRoot+"/update-pending.json")
		f.state.Current = fixtureGeneration
		f.state.Previous = ""
		f.storeState()
		return processResult{}, nil
	}
	f.p.onLock = func() {
		f.state.Current = nextGeneration
		f.storeState()
	}
	if err := launchInstalled(context.Background(), f.p, previous, f.state.Stock, f.candidate.Policy); err == nil {
		t.Fatal("fallback accepted a pointer changed before the creation lock")
	}
	if len(f.p.waits) != 1 || len(f.p.starts) != 0 || f.p.readsWhileLocked != 1 || f.p.lockHeld {
		t.Fatal("fallback did not refuse under the creation lock and release it")
	}
}
