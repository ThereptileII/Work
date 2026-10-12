//go:build !windows

package launcher

import (
	"context"
	"errors"
	"testing"
)

func TestMaintenanceBarrierRefusesWithoutNativeCustody(t *testing.T) {
	for _, location := range []string{"", `C:\Users\client\AppData\Local`, "/tmp/inert"} {
		owner, err := CaptureMaintenanceBarrierNative(context.Background(), location)
		if owner != nil || !errors.Is(err, errArmedUpdateBarrier) {
			t.Fatal("non-native path produced maintenance custody", err)
		}
	}
	if err := (&MaintenanceBarrier{}).Validate(context.Background()); !errors.Is(err, errArmedUpdateBarrier) {
		t.Fatal("forged non-native barrier validated", err)
	}
}
