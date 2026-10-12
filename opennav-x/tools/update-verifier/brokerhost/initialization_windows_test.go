package brokerhost

import (
	"context"
	"testing"

	"example.com/opennav-update-verifier/brokerstore"
	"golang.org/x/sys/windows"
	"golang.org/x/sys/windows/svc"
)

// Execute is deliberately invoked only with an ordinary token. The actual native
// creator must reject that token before resolving or changing the fixed namespace.
// These tests cannot qualify service enrollment and never create a service.
func TestNativeSCMInitializationRefusesOrdinaryPrincipal(t *testing.T) {
	user, err := windows.GetCurrentProcessToken().GetTokenUser()
	if err != nil {
		t.Fatal(err)
	}
	broker, _, _, err := windows.LookupSID("", brokerstore.BrokerAccount)
	if err == nil && user.User.Sid.Equals(broker) {
		t.Skip("ordinary-principal refusal requires an ordinary token")
	}
	for _, args := range [][]string{{ServiceName}, {ServiceName, "initialize-store"}} {
		t.Run(args[len(args)-1], func(t *testing.T) {
			h := &serviceHost{ctx: context.Background()}
			status := make(chan svc.Status, 4)
			specific, exit := h.Execute(args, nil, status)
			if specific || exit != 1 || h.result == nil {
				t.Fatalf("ordinary service entry accepted: specific=%v exit=%d result=%v", specific, exit, h.result)
			}
			if len(status) != 1 || (<-status).State != svc.StartPending {
				t.Fatal("refused initialization announced readiness")
			}
		})
	}
}
