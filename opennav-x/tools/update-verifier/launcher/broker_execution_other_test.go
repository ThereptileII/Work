//go:build !windows

package launcher

import (
	"context"
	"errors"
	"strings"
	"testing"
)

func TestBrokerExecutionEarlyDispatchExplicitNativeRefusal(t *testing.T) {
	if err := Run(context.Background(), []string{"--broker-execute=" + strings.Repeat("a", 32)}); !errors.Is(err, errBrokerExecutionNative) {
		t.Fatal(err)
	}
}

func TestBrokerExecutionCurrentEarlyDispatchExplicitNativeRefusal(t *testing.T) {
	if err := Run(context.Background(), []string{"--broker-current=" + strings.Repeat("a", 32)}); !errors.Is(err, errBrokerExecutionNative) {
		t.Fatal(err)
	}
}
