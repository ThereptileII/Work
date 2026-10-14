package brokerhost

import (
	"context"
	"errors"
	"strings"
	"testing"
)

type repairFrozenProfileFixture struct {
	failure error
	checks  int
	onCheck func()
}

func (p *repairFrozenProfileFixture) validateFrozen(ctx context.Context) error {
	p.checks++
	if p.onCheck != nil {
		p.onCheck()
	}
	return errors.Join(p.failure, ctx.Err())
}

func TestRepairColdCustodyRequiresFrozenProfileWithoutInventingProcessExit(t *testing.T) {
	e, profile := &engine{}, &repairFrozenProfileFixture{}
	ctx := context.Background()
	c, err := captureRepairApplicationCustodyWith(ctx, e, applicationOwnerTestSID, strings.Repeat("a", 32), profile)
	if err != nil || c == nil || c.original != nil || c.entry != nil || len(e.applications.entries) != 0 || profile.checks != 1 {
		t.Fatal("cold policy did not require independent frozen profile", err)
	}
	// Ordinary shared observation still refuses missing custody. This private
	// Repair owner cannot be reused as a fake process-exit or startup receipt.
	if _, _, err := e.observeApplicationOwnerLocked(ctx, applicationOwnerTestSID); err == nil {
		t.Fatal("Repair changed ordinary keeper semantics")
	}
	if err = c.validate(ctx); err != nil {
		t.Fatal(err)
	}
	if err = c.finish(ctx); err != nil || len(e.applications.entries) != 0 {
		t.Fatal("cold finish invented/released keeper", err)
	}
	if err = c.finish(ctx); err == nil {
		t.Fatal("finished custody reused")
	}
	if err = c.close(); err != nil {
		t.Fatal(err)
	}
	if err = c.close(); err != nil {
		t.Fatal(err)
	}
	if err = c.validate(ctx); err == nil {
		t.Fatal("closed state retained authority")
	}
}

func TestRepairCustodyEveryPresentUncertainKeeperRefusesColdFallback(t *testing.T) {
	for _, fault := range []string{"nil-entry", "nil-owner", "cleanup", "live", "unknown", "generation", "owner-generation", "sid", "observe"} {
		t.Run(fault, func(t *testing.T) {
			e, profile := &engine{}, &repairFrozenProfileFixture{}
			o := retainApplicationOwnerTest(t, e)
			o.state = executionApplicationExited
			entry := e.applications.entries[o.sid]
			switch fault {
			case "nil-entry":
				e.applications.entries[o.sid] = nil
			case "nil-owner":
				entry.owner = nil
			case "cleanup":
				entry.cleanupErr = errors.New("prior drain uncertain")
			case "live":
				o.state = executionApplicationLive
			case "unknown":
				o.state = executionApplicationUnknown
			case "generation":
				entry.generation = strings.Repeat("b", 32)
			case "owner-generation":
				o.gen = strings.Repeat("b", 32)
			case "sid":
				o.sid = "S-1-5-21-9-9-9-1001"
			case "observe":
				o.observeErr = errors.New("original token/handle lost")
			}
			if _, err := captureRepairApplicationCustodyWith(context.Background(), e, applicationOwnerTestSID, strings.Repeat("a", 32), profile); err == nil || profile.checks != 0 || o.closes != 0 {
				t.Fatal("uncertain keeper converted to cold absence or consumed", err)
			}
		})
	}
}

func TestRepairWarmCustodyKeepsExactOriginalUntilFinishAndNeverOwnsProfile(t *testing.T) {
	e, profile := &engine{}, &repairFrozenProfileFixture{}
	o := retainApplicationOwnerTest(t, e)
	o.state = executionApplicationExited
	c, err := captureRepairApplicationCustodyWith(context.Background(), e, o.sid, o.gen, profile)
	if err != nil || c.original != o || c.entry != e.applications.entries[o.sid] || o.closes != 0 {
		t.Fatal(err)
	}
	if err = c.validate(context.Background()); err != nil || o.closes != 0 {
		t.Fatal("validation consumed warm owner", err)
	}
	if err = c.close(); err != nil || o.closes != 0 {
		t.Fatal("custody close stole borrowed keeper/profile", err)
	}
	c, err = captureRepairApplicationCustodyWith(context.Background(), e, o.sid, o.gen, profile)
	if err != nil {
		t.Fatal(err)
	}
	if err = c.finish(context.Background()); err != nil || o.closes != 1 || len(e.applications.entries) != 0 {
		t.Fatal("original exited keeper not consumed exactly once", err)
	}
}

func TestRepairCustodyLateAppearanceReplacementProfileOrContextFailurePoisonsOwner(t *testing.T) {
	for _, fault := range []string{"cold-appearance", "warm-replacement", "profile", "context", "closed-engine"} {
		t.Run(fault, func(t *testing.T) {
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			e, profile := &engine{}, &repairFrozenProfileFixture{}
			if fault == "warm-replacement" {
				o := retainApplicationOwnerTest(t, e)
				o.state = executionApplicationExited
			}
			c, err := captureRepairApplicationCustodyWith(ctx, e, applicationOwnerTestSID, strings.Repeat("a", 32), profile)
			if err != nil {
				t.Fatal(err)
			}
			original := e.applications.entries[applicationOwnerTestSID]
			switch fault {
			case "cold-appearance":
				e.applications.entries = map[string]*executionApplicationEntry{applicationOwnerTestSID: nil}
			case "warm-replacement":
				copy := *original
				e.applications.entries[applicationOwnerTestSID] = &copy
			case "profile":
				profile.failure = errors.New("frozen native profile changed")
			case "context":
				cancel()
			case "closed-engine":
				e.applications.closed = true
			}
			if err = c.validate(context.Background()); err == nil {
				t.Fatal("late uncertainty accepted")
			}
			// Even restoring the observation cannot turn a failed original lease
			// into another mutation attempt or consume an application keeper.
			if original == nil {
				delete(e.applications.entries, applicationOwnerTestSID)
			} else {
				e.applications.entries[applicationOwnerTestSID] = original
			}
			profile.failure, e.applications.closed = nil, false
			if err = c.finish(context.Background()); err == nil || c.close() == nil {
				t.Fatal("poisoned custody regained authority")
			}
			if original != nil && original.owner.(*applicationOwnerTestCustody).closes != 0 {
				t.Fatal("failure consumed original warm keeper")
			}
		})
	}
}

func TestRepairCustodyCaptureAndPostProfileObservationRefusalsDoNotConsumeOwners(t *testing.T) {
	e, profile := &engine{}, &repairFrozenProfileFixture{failure: errors.New("kernel freeze refused")}
	if _, err := captureRepairApplicationCustodyWith(context.Background(), e, applicationOwnerTestSID, strings.Repeat("a", 32), profile); err == nil || len(e.applications.entries) != 0 {
		t.Fatal("failed freeze became cold authority")
	}
	profile.failure = nil
	profile.onCheck = func() { e.applications.entries = map[string]*executionApplicationEntry{applicationOwnerTestSID: nil} }
	if _, err := captureRepairApplicationCustodyWith(context.Background(), e, applicationOwnerTestSID, strings.Repeat("a", 32), profile); err == nil {
		t.Fatal("appearance during frozen validation accepted")
	}
	for _, fault := range []string{"nil-profile", "bad-sid", "bad-generation", "closed"} {
		e, sid, generation := &engine{}, applicationOwnerTestSID, strings.Repeat("a", 32)
		var source repairFrozenProfile = &repairFrozenProfileFixture{}
		if fault == "nil-profile" {
			source = nil
		}
		if fault == "bad-sid" {
			sid = "arbitrary"
		}
		if fault == "bad-generation" {
			generation = "arbitrary"
		}
		if fault == "closed" {
			e.applications.closed = true
		}
		if _, err := captureRepairApplicationCustodyWith(context.Background(), e, sid, generation, source); err == nil {
			t.Fatal("invalid custody accepted", fault)
		}
	}
}

func TestRepairCustodyWarmDrainFailureRetainsTombstoneAndCannotRetry(t *testing.T) {
	e, profile := &engine{}, &repairFrozenProfileFixture{}
	o := retainApplicationOwnerTest(t, e)
	o.state, o.closeErr = executionApplicationExited, errors.New("original keeper drain uncertain")
	c, err := captureRepairApplicationCustodyWith(context.Background(), e, o.sid, o.gen, profile)
	if err != nil {
		t.Fatal(err)
	}
	if err = c.finish(context.Background()); err == nil || o.closes != 1 || e.applications.entries[o.sid].cleanupErr == nil {
		t.Fatal("failed drain discarded original tombstone", err)
	}
	if err = c.finish(context.Background()); err == nil || o.closes != 1 || c.close() == nil {
		t.Fatal("uncertain drain retried")
	}
	if _, err = captureRepairApplicationCustodyWith(context.Background(), e, o.sid, o.gen, profile); err == nil || o.closes != 1 {
		t.Fatal("failed warm drain became a cold Repair")
	}
}
