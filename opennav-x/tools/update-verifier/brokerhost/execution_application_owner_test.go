package brokerhost

import (
	"context"
	"errors"
	"fmt"
	"strings"
	"testing"
)

const applicationOwnerTestSID = "S-1-5-21-1-2-3-500"

type applicationOwnerTestCustody struct {
	sid, gen             string
	state                executionApplicationState
	observeErr, closeErr error
	observations, closes int
}

func (o *applicationOwnerTestCustody) clientSID() string  { return o.sid }
func (o *applicationOwnerTestCustody) generation() string { return o.gen }
func (o *applicationOwnerTestCustody) observe(ctx context.Context) (executionApplicationState, error) {
	o.observations++
	return o.state, errors.Join(o.observeErr, ctx.Err())
}
func (o *applicationOwnerTestCustody) Close() error { o.closes++; return o.closeErr }

func retainApplicationOwnerTest(t *testing.T, e *engine) *applicationOwnerTestCustody {
	t.Helper()
	o := &applicationOwnerTestCustody{sid: applicationOwnerTestSID, gen: strings.Repeat("a", 32), state: executionApplicationLive}
	start, err := e.applications.reserve(context.Background(), o.sid, o.gen)
	if err != nil {
		t.Fatal(err)
	}
	if err = start.retain(context.Background(), o); err != nil {
		t.Fatal(err)
	}
	start.finish(true)
	return o
}

func TestExecutionApplicationOwnerRetainsExitProofUntilConsumed(t *testing.T) {
	ctx := context.Background()
	e := &engine{}
	o := retainApplicationOwnerTest(t, e)
	e.busy.Lock()
	defer e.busy.Unlock()
	actual, state, err := e.observeApplicationOwnerLocked(ctx, o.sid)
	if actual != o || state != executionApplicationLive || !errors.Is(err, errExecutionApplicationLive) || o.closes != 0 {
		t.Fatal("original live custody lost", state, err)
	}
	if _, err = e.applications.reserve(ctx, o.sid, strings.Repeat("b", 32)); !errors.Is(err, errExecutionApplicationOwner) {
		t.Fatal("concurrent application permitted", err)
	}
	if err = e.releaseApplicationOwnerLocked(ctx, o.sid); err == nil || o.closes != 0 {
		t.Fatal("live ordinary app owner released", err)
	}
	o.state = executionApplicationExited // normal application close, no tracker command
	for i := 0; i < 2; i++ {
		actual, state, err = e.observeApplicationOwnerLocked(ctx, o.sid)
		if actual != o || state != executionApplicationExited || err != nil || o.closes != 0 {
			t.Fatal("maintenance exit proof discarded", state, err)
		}
	}
	if err = e.releaseApplicationOwnerLocked(ctx, o.sid); err != nil || o.closes != 1 {
		t.Fatal("original query custody did not drain", err)
	}
	actual, state, err = e.observeApplicationOwnerLocked(ctx, o.sid)
	if actual != nil || state != executionApplicationUnknown || !errors.Is(err, errExecutionApplicationOwner) {
		t.Fatal("released custody became absence authority", state, err)
	}
}

func TestExecutionApplicationOwnerMissingRestartAndUncertaintyRefuse(t *testing.T) {
	ctx := context.Background()
	for _, consumed := range []bool{false, true} {
		t.Run(fmt.Sprint("consumed=", consumed), func(t *testing.T) {
			e := &engine{}
			if _, state, err := e.observeApplicationOwnerLocked(ctx, applicationOwnerTestSID); state != executionApplicationUnknown || err == nil {
				t.Fatal("service restart inferred application absence")
			}
			start, err := e.applications.reserve(ctx, applicationOwnerTestSID, strings.Repeat("a", 32))
			if err != nil {
				t.Fatal(err)
			}
			start.finish(consumed)
			next, err := e.applications.reserve(ctx, applicationOwnerTestSID, strings.Repeat("b", 32))
			if consumed {
				if next != nil || !errors.Is(err, errExecutionApplicationOwner) {
					t.Fatal("uncertain creator retried", err)
				}
			} else if err != nil {
				t.Fatal("pre-creator refusal consumed future startup", err)
			}
		})
	}
}

func TestExecutionApplicationOwnerFaultsKeepOriginalCustody(t *testing.T) {
	for _, failure := range []string{"cancelled", "native-error", "unknown", "cleanup"} {
		t.Run(failure, func(t *testing.T) {
			e := &engine{}
			o := retainApplicationOwnerTest(t, e)
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			switch failure {
			case "cancelled":
				cancel()
			case "native-error":
				o.observeErr = hostTestFailure
			case "unknown":
				o.state = executionApplicationUnknown
			case "cleanup":
				o.state = executionApplicationExited
				o.closeErr = hostTestFailure
			}
			if err := e.releaseApplicationOwnerLocked(ctx, o.sid); !errors.Is(err, errExecutionApplicationOwner) {
				t.Fatal("uncertainty permitted release", err)
			}
			if e.applications.entries[o.sid].owner != o {
				t.Fatal("original owner replaced")
			}
			if failure != "cleanup" && o.closes != 0 {
				t.Fatal("uncertain original owner closed")
			}
			if _, err := e.applications.reserve(ctx, o.sid, strings.Repeat("b", 32)); err == nil {
				t.Fatal("uncertain successor permitted")
			}
			if failure == "cleanup" {
				if o.closes != 1 {
					t.Fatal("cleanup failure retried by start")
				}
				return
			}
			// Transient observation failure may recover only through the same owner.
			o.observeErr = nil
			o.state = executionApplicationExited
			actual, state, err := e.observeApplicationOwnerLocked(context.Background(), o.sid)
			if actual != o || state != executionApplicationExited || err != nil {
				t.Fatal("same original exit did not recover", err)
			}
			if err = e.releaseApplicationOwnerLocked(context.Background(), o.sid); err != nil || o.closes != 1 {
				t.Fatal("recovered owner not drained", err)
			}
		})
	}
}

func TestExecutionApplicationOwnerExitedPredecessorAndBoundedClients(t *testing.T) {
	ctx := context.Background()
	e := &engine{}
	old := retainApplicationOwnerTest(t, e)
	old.state = executionApplicationExited
	replacement, err := e.applications.reserve(ctx, old.sid, strings.Repeat("b", 32))
	if err != nil || old.closes != 1 || replacement.entry.owner != nil {
		t.Fatal("exited predecessor not drained before successor", err)
	}
	if err = replacement.retain(ctx, old); err == nil {
		t.Fatal("prior generation owner substituted")
	}
	// Unknown reservations are intentionally bounded and cannot be evicted as absence.
	for i := 1; i < maxExecutionApplicationOwners; i++ {
		sid := fmt.Sprintf("S-1-5-21-1-2-3-%d", 1000+i)
		start, err := e.applications.reserve(ctx, sid, strings.Repeat("c", 32))
		if err != nil {
			t.Fatal(i, err)
		}
		start.finish(true)
	}
	if len(e.applications.entries) != maxExecutionApplicationOwners {
		t.Fatal("client custody accounting")
	}
	if _, err = e.applications.reserve(ctx, "S-1-5-21-1-2-3-9999", strings.Repeat("d", 32)); err == nil {
		t.Fatal("service custody unbounded")
	}
	if len(e.applications.entries) != maxExecutionApplicationOwners {
		t.Fatal("unknown client evicted")
	}
}

func TestExecutionApplicationOwnerTransferAndShutdown(t *testing.T) {
	ctx := context.Background()
	e := &engine{}
	o := retainApplicationOwnerTest(t, e)
	entry := e.applications.entries[o.sid]
	duplicate := &executionApplicationStart{owners: &e.applications, sid: o.sid, entry: entry}
	if err := duplicate.retain(ctx, o); err == nil {
		t.Fatal("custody transferred twice")
	}
	e.busy.Lock()
	if err := e.closeApplicationOwners(); err == nil || o.closes != 0 {
		t.Fatal("active handler custody drained", err)
	}
	e.busy.Unlock()
	if err := e.closeApplicationOwners(); err != nil || o.closes != 1 {
		t.Fatal("shutdown failed to release original handles", err)
	}
	if err := e.closeApplicationOwners(); err != nil || o.closes != 1 {
		t.Fatal("shutdown not idempotent", err)
	}
	if _, err := e.applications.reserve(ctx, o.sid, o.gen); err == nil {
		t.Fatal("closed tracker reused")
	}
	if _, state, err := e.observeApplicationOwnerLocked(ctx, o.sid); state != executionApplicationUnknown || err == nil {
		t.Fatal("shutdown became cold-profile proof")
	}
}
