//go:build !windows

package brokerpipe

import "context"

// WithMaintenanceOperator is available only for a live native Windows peer.
func (p *PeerCapability) WithMaintenanceOperator(context.Context, string, func(PeerIdentity) error) error {
	return ErrRefused
}

// WithMaintenanceOperatorChecked has no authority outside an actual native peer.
func (p *PeerCapability) WithMaintenanceOperatorChecked(context.Context, string, func(PeerIdentity, func(context.Context) error) error) error {
	return ErrRefused
}
