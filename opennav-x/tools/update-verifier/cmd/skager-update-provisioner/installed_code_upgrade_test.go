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
