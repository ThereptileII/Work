package brokerhost

import (
	"context"
	"errors"
	"strings"
	"testing"
)

func TestSCMCompletionVerificationIsStoppedReadOnly(t *testing.T) {
	for _, tc := range []struct {
		name            string
		result          error
		missing, cancel bool
	}{
		{name: "completed"}, {name: "missing durable completion", result: errors.New("private path")},
		{name: "no native validator", missing: true}, {name: "cancelled validation", cancel: true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			creates, validates := 0, 0
			validate := func() error {
				validates++
				if tc.cancel {
					cancel()
				}
				return tc.result
			}
			if tc.missing {
				validate = nil
			}
			handled, err := initializeServiceStart(ctx, []string{ServiceName, "verify-store"}, func() error { creates++; return nil }, validate)
			wantErr := tc.result != nil || tc.missing || tc.cancel
			if !handled || creates != 0 || (err != nil) != wantErr || validates != map[bool]int{true: 0, false: 1}[tc.missing] {
				t.Fatalf("handled=%v creates=%d validates=%d err=%v", handled, creates, validates, err)
			}
			if err != nil && strings.Contains(err.Error(), "private path") {
				t.Fatal("validator error leaked")
			}
		})
	}
	for _, args := range [][]string{{ServiceName, "VERIFY-STORE"}, {ServiceName, "verify-store", "extra"}, {"other", "verify-store"}} {
		calls := 0
		handled, err := initializeServiceStart(context.Background(), args, func() error { calls++; return nil }, func() error { calls++; return nil })
		if handled || err == nil || calls != 0 {
			t.Fatalf("ambiguous verification dispatch: %v %v %d", handled, err, calls)
		}
	}
}
