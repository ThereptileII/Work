package brokerpipe

import (
	"context"
	"testing"
)

func TestMaintenanceOperatorRefusesPortableAndForgedCapabilities(t *testing.T) {
	ctx, cap, _, identity := privatePeerFixture(t)
	defer cap.Close()
	used := false
	callback := func(PeerIdentity) error { used = true; return nil }
	for _, candidate := range []*PeerCapability{nil, {}, cap} {
		if err := candidate.WithMaintenanceOperator(ctx, identity.SID, callback); err == nil || used {
			t.Fatal("portable or forged capability granted maintenance operator")
		}
	}
	if err := cap.WithMaintenanceOperator(context.Background(), identity.SID, callback); err == nil || used {
		t.Fatal("unowned context granted maintenance operator")
	}
}
