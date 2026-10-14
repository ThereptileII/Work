package main

import (
	"context"
	"errors"
	"reflect"
	"sync"
	"testing"
	"time"
)

type elevationFixtureOwner struct {
	mu                      sync.Mutex
	closed, fail, failClose bool
	checks, closes          int
}

func (o *elevationFixtureOwner) check(ctx context.Context) error {
	o.mu.Lock()
	defer o.mu.Unlock()
	o.checks++
	if o.closed || o.fail {
		return errors.New("original fixture owner revoked")
	}
	return ctx.Err()
}
func (o *elevationFixtureOwner) close() error {
	o.mu.Lock()
	defer o.mu.Unlock()
	o.closes++
	o.closed = true
	if o.failClose {
		return errors.New("original close uncertain")
	}
	return nil
}

type elevationFixtureChild struct {
	elevationFixtureOwner
	exit    uint32
	waitErr error
	waited  bool
}

func (c *elevationFixtureChild) wait(ctx context.Context) (uint32, error) {
	c.waited = true
	return c.exit, errors.Join(c.waitErr, ctx.Err())
}
func elevationFixture(t *testing.T, elevated bool) (installedElevationOperations, *elevationFixtureOwner, *elevationFixtureOwner, *elevationFixtureChild) {
	t.Helper()
	self, parent, child := &elevationFixtureOwner{}, &elevationFixtureOwner{}, &elevationFixtureChild{}
	return installedElevationOperations{
		openSelf: func(ctx context.Context) (installedElevationSelf, error) {
			if d, ok := ctx.Deadline(); !ok || time.Until(d) > installedElevationLimit {
				t.Error("missing whole105s budget")
			}
			return installedElevationSelf{identity: installedElevationIdentity{pid: 17, created: 123456, sid: "S-1-5-21-1-2-3-1001", session: 2, elevated: elevated}, owner: self}, nil
		},
		openParent: func(_ context.Context, q installedElevationRequest, id installedElevationIdentity) (installedElevationOwner, error) {
			if q.pid != 16 || q.created != 123450 || id.pid != 17 || !id.elevated {
				t.Error("native comparison inputs changed")
			}
			return parent, nil
		},
		launch: func(_ context.Context, id installedElevationIdentity, args []string) (installedElevationChild, error) {
			want := []string{"--maintenance=uninstall-current-elevated", "--elevation-parent-pid=17", "--elevation-parent-created=123456"}
			if id.pid != 17 || !reflect.DeepEqual(args, want) {
				t.Error("fixed elevation launch arguments changed", args)
			}
			return child, nil
		},
	}, self, parent, child
}
func TestInstalledElevationStartupStrictPrivateArguments(t *testing.T) {
	valid := [][]string{{"--maintenance=uninstall-current"}, {"--maintenance", "uninstall-current"}, {"--maintenance=uninstall-current-elevated", "--elevation-parent-pid=16", "--elevation-parent-created=123450"}}
	for _, args := range valid {
		if _, handled, err := parseInstalledElevation(args); !handled || err != nil {
			t.Fatal(args, handled, err)
		}
	}
	invalid := [][]string{
		{"--maintenance=uninstall-current", "--sid=S-1-5-21-1-2-3-1001"},
		{"--maintenance=uninstall-current", "--helper=C:\\mutable.exe"},
		{"--maintenance=uninstall-current-elevated"},
		{"--maintenance=uninstall-current-elevated", "--elevation-parent-pid=016", "--elevation-parent-created=123450"},
		{"--maintenance=uninstall-current-elevated", "--elevation-parent-pid=16", "--elevation-parent-created=0123450"},
		{"--maintenance=uninstall-current-elevated", "--elevation-parent-pid=0", "--elevation-parent-created=123450"},
		{"--maintenance=uninstall-current-elevated", "--elevation-parent-pid=4294967296", "--elevation-parent-created=123450"},
		{"--maintenance=uninstall-current-elevated", "--elevation-parent-pid=16", "--elevation-parent-created=0"},
		{"--maintenance=uninstall-current-elevated", "--elevation-parent-created=123450", "--elevation-parent-pid=16"},
		{"--maintenance", "uninstall-current-elevated", "--elevation-parent-pid=16", "--elevation-parent-created=123450"},
	}
	for _, args := range invalid {
		if _, handled, err := parseInstalledElevation(args); !handled || err == nil {
			t.Fatal("private authority input accepted", args, handled, err)
		}
	}
	if _, handled, err := parseInstalledElevation([]string{"--maintenance=decommission"}); handled || err != nil {
		t.Fatal("operator route intercepted")
	}
}
func TestInstalledElevationDecommissionParentWaitsAndClosesWithoutMutation(t *testing.T) {
	op, self, parent, child := elevationFixture(t, false)
	calls := 0
	handled, err := installedElevationDispatchWith(context.Background(), []string{"--maintenance=uninstall-current"}, func(context.Context) error { calls++; return nil }, op)
	if !handled || err != nil || calls != 0 || !child.waited || !self.closed || !child.closed || parent.closed {
		t.Fatal("unelevated process performed cleanup or lost custody", handled, err, calls)
	}
}
func TestInstalledElevationDecommissionParentFailuresNeverRunCleanup(t *testing.T) {
	for _, fault := range []string{"UAC cancelled", "launch handle with failure", "child failed", "wait failed", "self changed", "child changed", "self close failed", "child close failed", "already elevated"} {
		t.Run(fault, func(t *testing.T) {
			op, self, _, child := elevationFixture(t, fault == "already elevated")
			switch fault {
			case "UAC cancelled":
				op.launch = func(context.Context, installedElevationIdentity, []string) (installedElevationChild, error) {
					return nil, errors.New("UAC cancelled")
				}
			case "launch handle with failure":
				op.launch = func(context.Context, installedElevationIdentity, []string) (installedElevationChild, error) {
					return child, errors.New("child comparison failed")
				}
			case "child failed":
				child.exit = 1
			case "wait failed":
				child.waitErr = context.DeadlineExceeded
			case "self changed":
				self.fail = true
			case "child changed":
				child.fail = true
			case "self close failed":
				self.failClose = true
			case "child close failed":
				child.failClose = true
			}
			calls := 0
			handled, err := installedElevationDispatchWith(context.Background(), []string{"--maintenance=uninstall-current"}, func(context.Context) error { calls++; return nil }, op)
			if !handled || err == nil || calls != 0 || !self.closed {
				t.Fatal("failed parent manufactured cleanup authority", err, calls)
			}
		})
	}
}
func TestInstalledElevationBindingChildRetainsParentUntilCleanupAndClose(t *testing.T) {
	op, self, parent, child := elevationFixture(t, true)
	calls := 0
	handled, err := installedElevationDispatchWith(context.Background(), []string{"--maintenance=uninstall-current-elevated", "--elevation-parent-pid=16", "--elevation-parent-created=123450"}, func(ctx context.Context) error {
		calls++
		if ctx.Err() != nil || self.closed || parent.closed {
			t.Fatal("original child/parent closed before maintenance")
		}
		return nil
	}, op)
	if !handled || err != nil || calls != 1 || !self.closed || !parent.closed || child.closed {
		t.Fatal("original child lifetime lost", err, calls)
	}
}
func TestInstalledElevationBindingParentExitCancelsMaintenance(t *testing.T) {
	op, self, parent, _ := elevationFixture(t, true)
	handled, err := installedElevationDispatchWith(context.Background(), []string{"--maintenance=uninstall-current-elevated", "--elevation-parent-pid=16", "--elevation-parent-created=123450"}, func(ctx context.Context) error {
		parent.mu.Lock()
		parent.fail = true
		parent.mu.Unlock()
		select {
		case <-ctx.Done():
			return nil // Even a swallowed cancellation must refuse success.
		case <-time.After(time.Second):
			t.Fatal("parent exit did not cancel maintenance")
			return nil
		}
	}, op)
	if !handled || err == nil || !self.closed || !parent.closed {
		t.Fatal("exited original parent yielded success", err)
	}
}
func TestInstalledElevationRestorationChildRefusalWithholdsMaintenance(t *testing.T) {
	for _, fault := range []string{"unelevated child", "parent refused", "parent changed", "parent close failed", "run failed", "cancelled"} {
		t.Run(fault, func(t *testing.T) {
			op, self, parent, _ := elevationFixture(t, fault != "unelevated child")
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			switch fault {
			case "parent refused":
				op.openParent = func(context.Context, installedElevationRequest, installedElevationIdentity) (installedElevationOwner, error) {
					return parent, errors.New("wrong original SID/session/creation/image")
				}
			case "parent changed":
				parent.fail = true
			case "parent close failed":
				parent.failClose = true
			case "cancelled":
				cancel()
			}
			calls := 0
			handled, err := installedElevationDispatchWith(ctx, []string{"--maintenance=uninstall-current-elevated", "--elevation-parent-pid=16", "--elevation-parent-created=123450"}, func(context.Context) error {
				calls++
				if fault == "run failed" {
					return errors.New("restoration uncertain")
				}
				return nil
			}, op)
			if !handled || err == nil {
				t.Fatal("failed child completed", err)
			}
			if fault != "run failed" && fault != "parent close failed" && calls != 0 {
				t.Fatal("unbound child reached maintenance")
			}
			if fault != "cancelled" && !self.closed {
				t.Fatal("child original owner leaked")
			}
		})
	}
}

type elevationFixtureWaitingChild struct{ elevationFixtureOwner }

func (c *elevationFixtureWaitingChild) wait(ctx context.Context) (uint32, error) {
	<-ctx.Done()
	return 0, ctx.Err()
}
func TestInstalledElevationDecommissionTimeoutClosesWithoutRetry(t *testing.T) {
	op, self, _, _ := elevationFixture(t, false)
	child := &elevationFixtureWaitingChild{}
	launches, runs := 0, 0
	op.launch = func(context.Context, installedElevationIdentity, []string) (installedElevationChild, error) {
		launches++
		return child, nil
	}
	ctx, cancel := context.WithTimeout(context.Background(), 20*time.Millisecond)
	defer cancel()
	handled, err := installedElevationDispatchWith(ctx, []string{"--maintenance=uninstall-current"}, func(context.Context) error { runs++; return nil }, op)
	if !handled || !errors.Is(err, context.DeadlineExceeded) || launches != 1 || runs != 0 || !self.closed || !child.closed {
		t.Fatal("timeout lost original custody or retried cleanup", err, launches, runs)
	}
}
