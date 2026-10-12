//go:build windows

package brokerhost

import (
	"context"
	"errors"
	"runtime"

	"example.com/opennav-update-verifier/brokerpipe"
	"example.com/opennav-update-verifier/brokerstore"
)

// Private startup admission for the original approved baseline executor. This is
// not yet an installed shortcut route. Protected history is inspected BEFORE any
// effects: empty permits explicit initial qualification; nonempty must resolve a
// complete selection and can only relay to a fresh selected executor. No failed
// qualification/current attempt is reinterpreted as permission for another flow.
func (e *engine) handleNativeStartupExecution(ctx context.Context, peer brokerpipe.Peer, request brokerpipe.ExecutionRequest, session *brokerpipe.ExecutionSession) (resultErr error) {
	if e == nil || ctx == nil || ctx.Err() != nil || session == nil || e.observe == nil ||
		request.Schema != 1 || request.Operation != "start-selected" || !requestPattern.MatchString(request.RequestID) || !requestPattern.MatchString(request.StageID) {
		return errExecutionHostRefused
	}
	adapter, ok := e.store.(storeAdapter)
	if !ok || adapter.Store == nil {
		return errExecutionHostRefused
	}
	capability, err := brokerpipe.AuthorizePeer(ctx, peer)
	if err != nil {
		return err
	}

	initial, err := func() (initial bool, resultErr error) {
		if !e.busy.TryLock() {
			return false, errExecutionHostRefused
		}
		defer e.busy.Unlock()
		runtime.LockOSThread()
		defer runtime.UnlockOSThread()
		if err := errors.Join(adapter.Validate(), capability.Validate(ctx)); err != nil {
			return false, err
		}
		raw, err := adapter.ReadRegistration(peer.SID)
		if err != nil {
			return false, err
		}
		r, err := parseRegistration(raw, peer.SID)
		if err != nil || request.StageID != r.Installed.Generation {
			return false, errors.Join(errExecutionHostRefused, err)
		}
		registrationDigest := executionOriginalDigest(raw)
		baseline, err := adapter.OpenReadOnlyGeneration(ctx, peer.SID, r.Installed.Generation)
		if err != nil {
			return false, err
		}
		defer func() { resultErr = errors.Join(resultErr, baseline.Close()) }()
		if baseline.Purpose() != brokerstore.GenerationPurposeApprovedBaseline {
			return false, errExecutionHostRefused
		}
		authorization := &nativeExecutionAuthorization{store: adapter.Store}
		if err = authorization.authorize(ctx, baseline, nil); err != nil {
			return false, err
		}
		image, err := executionHostExecutor(ctx, baseline)
		if err != nil {
			return false, err
		}
		// This pure comparison uses actual retained peer numbers and consumes no
		// one-use session binding. Initial qualification binds its own retained
		// original after all inspection owners/locks below have been released.
		if err = capability.WithPeer(ctx, func(identity brokerpipe.PeerIdentity) error {
			if identity.SID != r.ClientSID {
				return errExecutionHostRefused
			}
			return image.MatchesProcess(ctx, identity.ProcessID, identity.ProcessCreation)
		}); err != nil {
			return false, err
		}
		observed, stock, err := e.observe(ctx, r)
		if stock != nil {
			defer func() { resultErr = errors.Join(resultErr, stock.Close()) }()
		}
		if err != nil || stock == nil || observed != stockBaseline {
			return false, errors.Join(errExecutionHostRefused, err)
		}
		entries, err := adapter.EnumerateExecutionJournals(ctx, peer.SID)
		if err != nil {
			return false, err
		}
		inventory, err := executionSelectionInventoryDigest(peer.SID, entries)
		if err != nil {
			return false, err
		}
		validate := func() error {
			after, err := adapter.ReadRegistration(peer.SID)
			if err != nil || executionOriginalDigest(after) != registrationDigest {
				return errors.Join(errExecutionHostRefused, err)
			}
			if err = errors.Join(capability.Validate(ctx), authorization.authorize(ctx, baseline, nil), stock.validate()); err != nil {
				return err
			}
			afterEntries, err := adapter.EnumerateExecutionJournals(ctx, peer.SID)
			if err != nil {
				return err
			}
			afterInventory, err := executionSelectionInventoryDigest(peer.SID, afterEntries)
			if err != nil || afterInventory != inventory {
				return errors.Join(errExecutionHostRefused, err)
			}
			return ctx.Err()
		}
		if len(entries) == 0 {
			if err = validate(); err != nil {
				return false, err
			}
			return true, nil
		}
		selected, err := captureSelectedExecutionNative(ctx, adapter.Store, r)
		if err != nil {
			return false, err
		} // Pending, partial or forked history refuses.
		defer func() { resultErr = errors.Join(resultErr, selected.Close()) }()
		if selected.inventory != inventory {
			return false, errExecutionHostRefused
		}
		if err = session.BindExecutorImage(ctx, image); err != nil {
			return false, err
		}
		if err = validate(); err != nil {
			return false, err
		}
		if err = relaySelectedExecutionNative(ctx, capability, selected, session); err != nil {
			return false, err
		}
		if err = errors.Join(validate(), selected.validate()); err != nil {
			return false, err
		}
		_, err = session.ExchangeCommand(ctx, brokerpipe.ExecutionDirective{Phase: brokerpipe.FinishExecution})
		return false, err
	}()
	if err != nil || !initial {
		return err
	}
	// Pure inspection has fully drained. The initial-only qualifier MUST recheck
	// exact empty original history under its own operation lock before any effect;
	// the same start-selected intent is preserved, never rewritten as an update.
	providers, closeProviders := nativeExecutionProviders(adapter.Store)
	defer func() { resultErr = errors.Join(resultErr, closeProviders()) }()
	return e.handleNativeExecution(ctx, peer, request, session, providers)
}
