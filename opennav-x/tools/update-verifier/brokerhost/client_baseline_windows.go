//go:build windows

package brokerhost

import (
	"context"
	"errors"
	"sync"

	verifier "example.com/opennav-update-verifier"
	"example.com/opennav-update-verifier/brokerenroll"
	"example.com/opennav-update-verifier/brokerregistration"
	"example.com/opennav-update-verifier/brokerstore"
)

// The fixed protected operator ZIP remains held through final client completion.
// This imports an approved baseline; it does not attest an older LocalAppData
// tree, adopt added plugins, choose a running image or grant launch authority.
type registeredBaseline struct {
	mu       sync.Mutex
	owner    *brokerstore.InstalledBaseline
	source   brokerenroll.ApprovedClientBaseline
	closed   bool
	closeErr error
}

func (b *registeredBaseline) Validate() error {
	b.mu.Lock()
	defer b.mu.Unlock()
	if b.closed {
		return errors.New("registered baseline custody closed")
	}
	return errors.Join(b.source.Validate(), b.owner.Validate())
}

func (b *registeredBaseline) Close() error {
	b.mu.Lock()
	defer b.mu.Unlock()
	if !b.closed {
		b.closed = true
		b.closeErr = errors.Join(b.owner.Close(), b.source.Close())
	}
	return b.closeErr
}

func importClientBaselineNative(ctx context.Context, store clientRegistrationStore, request brokerregistration.Request) (result baselineOwner, resultErr error) {
	native, ok := store.(nativeClientRegistrationStore)
	if !ok || native.Store == nil {
		return nil, errors.New("actual retained native broker store required")
	}
	source, err := brokerenroll.OpenApprovedClientBaselineNative(ctx, request.ClientSID)
	if source != nil {
		defer func() {
			if result == nil {
				resultErr = errors.Join(resultErr, source.Close())
			}
		}()
	}
	if err != nil {
		return nil, err
	}
	if source == nil {
		return nil, errors.New("retained fixed approved baseline source required")
	}
	if err := source.Validate(); err != nil {
		return nil, err
	}
	owner, err := native.BeginInstalledBaselineFresh(ctx, request.ClientSID, request.Installed.Generation, request.RequestSHA256)
	if owner != nil {
		defer func() {
			if result == nil {
				resultErr = errors.Join(resultErr, owner.Close())
			}
		}()
	}
	if err != nil {
		return nil, err
	}
	if owner == nil {
		return nil, errors.New("fresh protected installed baseline required")
	}
	identity, err := verifier.ImportApprovedBaseline(ctx, request.OriginalBytes(), request.RequestSHA256, source, source.Size(), owner)
	if err != nil {
		return nil, err
	}
	if err := source.Validate(); err != nil {
		return nil, err
	}
	if err := owner.Complete(ctx, identity); err != nil {
		return nil, err
	}
	b := &registeredBaseline{owner: owner, source: source}
	if err := b.Validate(); err != nil {
		return nil, err
	}
	return b, nil
}
