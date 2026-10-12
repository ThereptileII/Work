//go:build !windows

package brokerpipe

import "context"

// WithMaintenanceOperator is available only for a live native Windows peer.
func (p *PeerCapability) WithMaintenanceOperator(context.Context, string, func(PeerIdentity) error) error {
	return ErrRefused
}
