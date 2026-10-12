//go:build !windows

package brokerstore

import (
	"errors"
	"testing"
)

func TestFreshInitializationRequiresNativeWindows(t *testing.T) {
	for _, operation := range []func() error{InitializeFresh, ValidateInitialization} {
		if err := operation(); !errors.Is(err, ErrNativeRequired) {
			t.Fatalf("portable operation: %v", err)
		}
	}
}
