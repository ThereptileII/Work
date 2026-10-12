package brokerpipe

import (
	"context"
	"testing"
)

func TestMaintenanceOperatorLocationRefusesForgedPortableAndUnownedScope(t *testing.T) {
	ctx, cap, _, identity := privatePeerFixture(t)
	defer cap.Close()
	used := false
	callback := func(PeerIdentity, string) error { used = true; return nil }
	for _, candidate := range []*PeerCapability{nil, {}, cap} {
		if err := candidate.WithMaintenanceOperatorLocation(ctx, identity.SID, callback); err == nil || used {
			t.Fatal("forged/portable capability supplied operator location")
		}
	}
	if err := cap.WithMaintenanceOperatorLocation(context.Background(), identity.SID, callback); err == nil || used {
		t.Fatal("unowned context supplied operator location")
	}
	if err := cap.WithMaintenanceOperatorLocation(ctx, identity.SID, nil); err == nil {
		t.Fatal("missing location callback accepted")
	}
}

func TestMaintenanceOperatorLocationCheckedRefusesPortableAndUnownedScope(t *testing.T) {
	ctx, cap, _, identity := privatePeerFixture(t)
	defer cap.Close()
	used := false
	callback := func(PeerIdentity, string, func(context.Context) error) error { used = true; return nil }
	for _, candidate := range []*PeerCapability{nil, {}, cap} {
		if err := candidate.WithMaintenanceOperatorLocationChecked(ctx, identity.SID, callback); err == nil || used {
			t.Fatal("forged/portable capability supplied operator location checker")
		}
	}
	if err := cap.WithMaintenanceOperatorLocationChecked(context.Background(), identity.SID, callback); err == nil || used {
		t.Fatal("unowned context supplied operator location checker")
	}
}
