//go:build !windows

package brokerpipe

import "context"

// WithMaintenanceOperatorLocation requires a live native Windows operator.
func (p *PeerCapability) WithMaintenanceOperatorLocation(context.Context, string, func(PeerIdentity, string) error) error {
	return ErrRefused
}

// WithMaintenanceOperatorLocationChecked requires a live native Windows peer.
func (p *PeerCapability) WithMaintenanceOperatorLocationChecked(context.Context, string, func(PeerIdentity, string, func(context.Context) error) error) error {
	return ErrRefused
}
