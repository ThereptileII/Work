package main

import (
	"bytes"
	"context"
	"errors"
	"reflect"
	"strings"
	"testing"
	"time"
)

func successorFixtureArgs() []string {
	return installedCodeSuccessorArgs(installedElevationIdentity{pid: 16, created: 123450}, installedCodeComparisons{strings.Repeat("a", 64), strings.Repeat("b", 64), strings.Repeat("c", 64), strings.Repeat("d", 64)})
}

func TestInstalledCodeSuccessorStrictComparisonArguments(t *testing.T) {
	args := successorFixtureArgs()
	q, handled, err := parseInstalledCodeSuccessor(args)
	if !handled || err != nil || q.parent.pid != 16 || q.parent.created != 123450 || q.pins.policy != strings.Repeat("a", 64) {
		t.Fatal(q, handled, err)
	}
	invalid := [][]string{
		append(append([]string{}, args...), "--helper=C:\\mutable.exe"),
		append(append([]string{}, args...), "--sid=S-1-5-21-1-2-3-1001"),
		append(append([]string{}, args...), "--resume=true"),
		{"--maintenance", codeUpgradeSuccessor},
		{"--maintenance=" + codeUpgradeSuccessor},
	}
	for i, replacement := range []string{"--elevation-parent-pid=016", "--elevation-parent-created=0123450", "--policy-sha256=" + strings.Repeat("A", 64), "--provisioner-sha256=x", "--trust-config-sha256=", "--original-completion-sha256=x"} {
		copyArgs := append([]string{}, args...)
		copyArgs[1+i] = replacement
		invalid = append(invalid, copyArgs)
	}
	duplicate := append([]string{}, args...)
	duplicate[4] = duplicate[3]
	invalid = append(invalid, duplicate, append([]string{"--untrusted=x"}, args...))
	for _, bad := range invalid {
		if _, handled, err := parseInstalledCodeSuccessor(bad); !handled || err == nil {
			t.Fatal("private successor authority input accepted", bad, handled, err)
		}
		calls := 0
		op := installedCodeSuccessorOperations{openSelf: func(context.Context) (installedElevationSelf, error) { calls++; return installedElevationSelf{}, nil }}
		if handled, err := installedCodeSuccessorDispatchWith(context.Background(), bad, op); !handled || err == nil || calls != 0 {
			t.Fatal("invalid command reached native opener", handled, err, calls)
		}
	}
}

func TestInstalledCodeSuccessorRetainsOriginalOwnersAndComparisons(t *testing.T) {
	self, parent := &elevationFixtureOwner{}, &elevationFixtureOwner{}
	calls := 0
	op := installedCodeSuccessorOperations{
		openSelf: func(context.Context) (installedElevationSelf, error) {
			return installedElevationSelf{installedElevationIdentity{17, 123456, "S-1-5-21-1-2-3-1001", 2, true}, self}, nil
		},
		openParent: func(_ context.Context, q installedElevationRequest, id installedElevationIdentity) (installedElevationOwner, error) {
			if q.pid != 16 || q.created != 123450 || !id.elevated {
				t.Fatal("parent comparison changed")
			}
			return parent, nil
		},
		run: func(ctx context.Context, pins installedCodeComparisons) error {
			calls++
			q, _, _ := parseInstalledCodeSuccessor(successorFixtureArgs())
			if !reflect.DeepEqual(q.pins, pins) || self.closed || parent.closed || ctx.Err() != nil {
				t.Fatal("handoff custody/pins lost")
			}
			return nil
		},
	}
	if handled, err := installedCodeSuccessorDispatchWith(context.Background(), successorFixtureArgs(), op); !handled || err != nil || calls != 1 || !self.closed || !parent.closed {
		t.Fatal(handled, err, calls)
	}
}

func TestInstalledCodeSuccessorParentLossAndCleanupCannotReportSuccess(t *testing.T) {
	for _, failure := range []string{"parent lost", "self close", "parent close", "run refused"} {
		t.Run(failure, func(t *testing.T) {
			self, parent := &elevationFixtureOwner{failClose: failure == "self close"}, &elevationFixtureOwner{failClose: failure == "parent close"}
			op := installedCodeSuccessorOperations{
				openSelf: func(context.Context) (installedElevationSelf, error) {
					return installedElevationSelf{installedElevationIdentity{17, 123456, "S-1-5-21-1-2-3-1001", 2, true}, self}, nil
				},
				openParent: func(context.Context, installedElevationRequest, installedElevationIdentity) (installedElevationOwner, error) {
					return parent, nil
				},
				run: func(ctx context.Context, _ installedCodeComparisons) error {
					if failure == "run refused" {
						return errors.New("independent successor admission refused")
					}
					if failure == "parent lost" {
						parent.mu.Lock()
						parent.fail = true
						parent.mu.Unlock()
						select {
						case <-ctx.Done():
						case <-time.After(time.Second):
							t.Fatal("parent loss not observed")
						}
					}
					return nil
				},
			}
			if handled, err := installedCodeSuccessorDispatchWith(context.Background(), successorFixtureArgs(), op); !handled || err == nil || !self.closed || !parent.closed {
				t.Fatal(handled, err)
			}
		})
	}
}

func TestInstalledCodeCurrentActionAndPrivateReporting(t *testing.T) {
	for _, args := range [][]string{{"--maintenance=" + codeUpgradeCurrent}, {"--maintenance", codeUpgradeCurrent}, {"--maintenance=" + codeUpgradeCurrent + "-elevated", "--elevation-parent-pid=16", "--elevation-parent-created=123450"}} {
		q, handled, err := parseInstalledElevation(args)
		if !handled || err != nil || q.action != codeUpgradeCurrent {
			t.Fatal(q, handled, err)
		}
	}
	for _, args := range [][]string{{"--maintenance=" + codeUpgradeCurrent, "--policy-sha256=" + strings.Repeat("a", 64)}, {"--maintenance=" + codeUpgradeCurrent + "-elevated"}} {
		if _, handled, err := parseInstalledElevation(args); !handled || err == nil {
			t.Fatal("new action fell through", handled, err)
		}
	}
	var output bytes.Buffer
	if err := reportMaintenanceCompletion(successorFixtureArgs(), &output); err != nil || output.Len() != 0 {
		t.Fatal("private successor received false generic completion", err, output.String())
	}
	called := false
	if err := runMaintenanceDispatch(context.Background(), []string{"--maintenance=unknown-customer-code-action"}, func(string) ([]byte, error) { called = true; return nil, nil }, nil); err == nil || called {
		t.Fatal("unknown action admitted", err, called)
	}
}

type codeFixtureLaunch struct {
	elevationFixtureOwner
	value installedCodeSnapshot
}

func (o *codeFixtureLaunch) snapshot(ctx context.Context) (installedCodeSnapshot, error) {
	return o.value, o.check(ctx)
}

type codeFixtureAdmission struct {
	codeFixtureLaunch
	consumed   bool
	launch     *codeFixtureLaunch
	consumeErr error
}

func (o *codeFixtureAdmission) consume(ctx context.Context) (installedCodeLaunchOwner, error) {
	if err := o.check(ctx); err != nil {
		return nil, err
	}
	o.consumed = true
	return o.launch, o.consumeErr
}

func TestInstalledCodeCustomerConsumeRetainsAdmissionUntilOriginalExit(t *testing.T) {
	for _, fault := range []string{"none", "wait uncertain", "nonzero exit", "launch uncertainty", "changed snapshot", "cleanup failure"} {
		t.Run(fault, func(t *testing.T) {
			q, _, _ := parseInstalledCodeSuccessor(successorFixtureArgs())
			launch := &codeFixtureLaunch{value: installedCodeSnapshot{"fixed-admitted-image", "original-cache", q.pins}}
			initial := &codeFixtureAdmission{codeFixtureLaunch: codeFixtureLaunch{value: launch.value}, launch: launch}
			child := &elevationFixtureChild{}
			if fault == "wait uncertain" {
				child.waitErr = context.DeadlineExceeded
			}
			if fault == "nonzero exit" {
				child.exit = 1
			}
			if fault == "changed snapshot" {
				launch.value.pins.policy = strings.Repeat("e", 64)
			}
			if fault == "cleanup failure" {
				launch.failClose = true
			}
			launches, retained := 0, false
			op := installedCodeCustomerOperations{
				acquire: func(context.Context) (installedCodeAdmissionOwner, error) { return initial, nil },
				launch: func(ctx context.Context, owner installedCodeLaunchOwner, s installedCodeSnapshot) (installedElevationChild, error) {
					launches++
					if !initial.consumed || initial.closed || launch.closed || owner != launch || s != launch.value {
						t.Fatal("completion pins not consumed or acquisition custody lost")
					}
					if fault == "launch uncertainty" {
						return child, errors.New("resume unverified")
					}
					return child, nil
				},
				retainUntilExit: func(c installedElevationChild, o installedCodeLaunchOwner) error {
					retained = true
					if c != child || o != launch || child.closed || launch.closed {
						t.Fatal("uncertain child acquisition released")
					}
					return nil
				},
			}
			var output bytes.Buffer
			err := runCustomerCodeUpgradeWith(context.Background(), &output, op)
			if (err == nil) != (fault == "none") || !initial.closed {
				t.Fatal("incorrect customer outcome", err, fault)
			}
			wantRetain := fault == "wait uncertain" || fault == "launch uncertainty"
			if retained != wantRetain || launch.closed == wantRetain {
				t.Fatal("original child lifetime mismatch", retained, launch.closed)
			}
			if fault == "changed snapshot" && launches != 0 {
				t.Fatal("changed admission launched")
			}
			if fault == "none" && (!child.waited || !child.closed || !strings.Contains(output.String(), "broker-code-upgraded")) {
				t.Fatal("verified upgrade missing", output.String())
			}
		})
	}
}

func TestInstalledCodeCustomerOnlyAuthenticatedNoChangeIsSuccess(t *testing.T) {
	current, unavailable := errors.New("verified current"), errors.New("authenticated absent")
	for _, candidate := range []error{current, unavailable, errors.New("network failure"), errors.New("partial transaction"), nil} {
		var output bytes.Buffer
		calls := 0
		err := runCustomerCodeUpgradeWith(context.Background(), &output, installedCodeCustomerOperations{
			acquire: func(context.Context) (installedCodeAdmissionOwner, error) { return nil, candidate },
			launch: func(context.Context, installedCodeLaunchOwner, installedCodeSnapshot) (installedElevationChild, error) {
				calls++
				return nil, nil
			},
			retainUntilExit: func(installedElevationChild, installedCodeLaunchOwner) error {
				t.Fatal("no-change retained child")
				return nil
			}, current: current, unavailable: unavailable,
		})
		if (err == nil) != (candidate == current || candidate == unavailable) || calls != 0 {
			t.Fatal("unverified state became no-change", candidate, err)
		}
		if err != nil && output.Len() != 0 {
			t.Fatal("failed discovery reported success")
		}
	}
}

func TestInstalledCodeFailureCannotReturnBeforeOriginalChildExit(t *testing.T) {
	q, _, _ := parseInstalledCodeSuccessor(successorFixtureArgs())
	launch := &codeFixtureLaunch{value: installedCodeSnapshot{"admitted", "cache", q.pins}}
	initial := &codeFixtureAdmission{codeFixtureLaunch: codeFixtureLaunch{value: launch.value}, launch: launch}
	child := &elevationFixtureChild{waitErr: context.DeadlineExceeded}
	draining, exited, finished := make(chan struct{}), make(chan struct{}), make(chan error, 1)
	go func() {
		var output bytes.Buffer
		finished <- runCustomerCodeUpgradeWith(context.Background(), &output, installedCodeCustomerOperations{
			acquire: func(context.Context) (installedCodeAdmissionOwner, error) { return initial, nil },
			launch: func(context.Context, installedCodeLaunchOwner, installedCodeSnapshot) (installedElevationChild, error) {
				return child, nil
			},
			retainUntilExit: func(c installedElevationChild, o installedCodeLaunchOwner) error {
				if c != child || o != launch || initial.closed || launch.closed || child.closed {
					t.Error("original live custody lost")
				}
				close(draining)
				<-exited
				return errors.Join(c.close(), o.close())
			},
		})
	}()
	select {
	case <-draining:
	case <-time.After(time.Second):
		t.Fatal("original child drain not reached")
	}
	select {
	case err := <-finished:
		t.Fatal("command returned while original child pending", err)
	case <-time.After(25 * time.Millisecond):
	}
	close(exited)
	select {
	case err := <-finished:
		if err == nil || !initial.closed || !launch.closed || !child.closed {
			t.Fatal("failure or custody lost after original exit", err)
		}
	case <-time.After(time.Second):
		t.Fatal("original exit did not release command")
	}
}
