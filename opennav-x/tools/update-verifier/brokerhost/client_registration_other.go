//go:build !windows

package brokerhost

import (
	"context"
	"example.com/opennav-update-verifier/brokerstore"
)

// RegisterApprovedClientNative never emulates the distinct Windows principal,
// protected native creation or independently authenticated operator handoff.
func RegisterApprovedClientNative(context.Context, []byte, string) error {
	return brokerstore.ErrNativeRequired
}
