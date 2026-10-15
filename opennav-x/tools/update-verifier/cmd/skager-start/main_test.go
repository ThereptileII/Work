package main

import (
	"bytes"
	"context"
	"errors"
	"reflect"
	"strings"
	"testing"
)

func TestStartupFailureReporting(t *testing.T) {
	for _, args := range [][]string{nil, {"--xnav"}, {"--legacy"}, {"--safe-mode"},
		{"--initialize-trust"}, {"--initialize-trust", "extra"}, {"--initialize-trust="}, {"--INITIALIZE-TRUST"}} {
		for _, failed := range []bool{false, true} {
			t.Run(strings.Join(args, " ")+"/failed="+map[bool]string{false: "false", true: "true"}[failed], func(t *testing.T) {
				var stderr bytes.Buffer
				calls, dialogs := 0, 0
				run := func(ctx context.Context, actual []string) error {
					calls++
					if ctx == nil || !reflect.DeepEqual(actual, args) {
						t.Fatal("startup changed the command")
					}
					if failed {
						return errors.New("inert initialization failure")
					}
					return nil
				}
				status := startup(context.Background(), args, run, &stderr, func() { dialogs++ })
				wantStatus, wantDialogs := 0, 0
				if failed {
					wantStatus = 1
					if len(args) != 1 || args[0] != "--initialize-trust" {
						wantDialogs = 1
					}
				}
				if status != wantStatus || dialogs != wantDialogs || calls != 1 ||
					(failed && !strings.Contains(stderr.String(), "inert initialization failure")) ||
					(!failed && stderr.Len() != 0) {
					t.Fatalf("failure contract: status=%d dialogs=%d calls=%d stderr=%q", status, dialogs, calls, stderr.String())
				}
			})
		}
	}
}
