//go:build !windows

package launcher

import (
	"context"
	"testing"

	"example.com/opennav-update-verifier/brokerpipe"
)

func TestMaintenanceClientNativeUnavailableOffWindows(t *testing.T) {
	result, err := CallMaintenanceNative(context.Background(), maintenanceClientTestRequest())
	if err == nil || result != (brokerpipe.MaintenanceResult{}) {
		t.Fatalf("result=%+v err=%v", result, err)
	}
}
