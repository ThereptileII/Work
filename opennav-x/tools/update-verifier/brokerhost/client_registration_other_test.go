//go:build !windows

package brokerhost

import (
	"context"
	"errors"
	"testing"

	"example.com/opennav-update-verifier/brokerstore"
)

func TestClientRegistrationNativeEntryRefusesPortableHost(t *testing.T) {
	raw := testApprovedClientRequest()
	if err := RegisterApprovedClientNative(context.Background(), raw, clientRequestDigest(raw)); !errors.Is(err, brokerstore.ErrNativeRequired) {
		t.Fatalf("portable host emulated native registration: %v", err)
	}
}
