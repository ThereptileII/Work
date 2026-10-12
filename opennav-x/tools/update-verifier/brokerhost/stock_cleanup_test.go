package brokerhost

import (
	"errors"
	"testing"
)

func TestStockProbeCloseFailureCannotBecomeSuccess(t *testing.T) {
	observed := errors.New("inert observation failure")
	closed := errors.New("inert close failure")
	for _, tc := range []struct {
		name           string
		observe, close error
	}{{"success", nil, nil}, {"observation", observed, nil}, {"close", nil, closed}, {"both", observed, closed}} {
		t.Run(tc.name, func(t *testing.T) {
			checks, closes := 0, 0
			err := checkAndCloseStockProbe(func() error { checks++; return tc.observe }, func() error { closes++; return tc.close })
			if checks != 1 || closes != 1 {
				t.Fatal("probe or cleanup repeated/skipped")
			}
			if (err == nil) != (tc.observe == nil && tc.close == nil) {
				t.Fatal("incorrect successful outcome")
			}
			if tc.observe != nil && !errors.Is(err, observed) {
				t.Fatal("observation error lost")
			}
			if tc.close != nil && (!errors.Is(err, closed) || !errors.Is(err, ErrStockCleanup)) {
				t.Fatal("cleanup uncertainty lost")
			}
			if tc.close == nil && errors.Is(err, ErrStockCleanup) {
				t.Fatal("ordinary refusal relabelled uncertain")
			}
		})
	}
}
