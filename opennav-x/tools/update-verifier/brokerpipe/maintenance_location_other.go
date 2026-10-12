//go:build !windows

package brokerpipe

import "context"

// WithMaintenanceOperatorLocation requires a live native Windows operator.
func (p *PeerCapability) WithMaintenanceOperatorLocation(context.Context, string, func(PeerIdentity, string) error) error {
	return ErrRefused
}
