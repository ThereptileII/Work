//go:build !windows

package brokerpipe

import (
	"context"
	"errors"
	"testing"
)

func TestExecutionNativeUnavailableHasNoPortableAuthority(t *testing.T) {
	if !errors.Is(ServeExecution(context.Background(), nil), ErrUnavailable) {
		t.Fatal("portable server authority")
	}
	if !errors.Is(CallExecution(context.Background(), executionTestRequest(), nil), ErrUnavailable) {
		t.Fatal("portable client authority")
	}
}
