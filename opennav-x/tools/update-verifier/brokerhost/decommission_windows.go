//go:build windows

package brokerhost

import (
	"context"
	"errors"
	"runtime"

	"example.com/opennav-update-verifier/brokerpipe"
	"example.com/opennav-update-verifier/brokerstore"
	"example.com/opennav-update-verifier/launcher"
)

// Service-only preparation publishes a permanent startup fence. The separate
// protected uninstall retires original selected application and registration
// custody before returning the authenticated witness; it never touches profiles.
// The same service-local original application must have exited normally;
// restart-lost custody refuses. The current profile is frozen through publication/readback and never written. All
// histories, code, trust floors and the untouched stock remain recovery backup.
func (e *engine) handlePrepareDecommission(ctx context.Context, peer brokerpipe.Peer, q brokerpipe.MaintenanceRequest) (result brokerpipe.MaintenanceResult, resultErr error) {
	if q.Operation != brokerpipe.PrepareDecommission {
		return result, errDecommissionRefused
	}
	return e.prepareOriginalDecommission(ctx, peer, q, nil)
}

func (e *engine) handleNativeProtectedUninstall(ctx context.Context, peer brokerpipe.Peer, q brokerpipe.MaintenanceRequest, publishFence func(context.Context, brokerpipe.MaintenanceResult) error) (brokerpipe.MaintenanceResult, error) {
	if q.Operation != brokerpipe.PrepareUninstall || publishFence == nil {
		return brokerpipe.MaintenanceResult{}, errDecommissionRefused
	}
	return e.prepareOriginalDecommission(ctx, peer, q, publishFence)
}

func (e *engine) prepareOriginalDecommission(ctx context.Context, peer brokerpipe.Peer, q brokerpipe.MaintenanceRequest, publishFence func(context.Context, brokerpipe.MaintenanceResult) error) (result brokerpipe.MaintenanceResult, resultErr error) {
	defer func() {
		if resultErr != nil {
			result = brokerpipe.MaintenanceResult{}
			resultErr = errDecommissionRefused
		}
	}()
	if e == nil || ctx == nil || ctx.Err() != nil || e.observe == nil || !validDecommissionRequest(q) {
		return result, errDecommissionRefused
	}
	adapter, ok := e.store.(storeAdapter)
	if !ok || adapter.Store == nil {
		return result, errDecommissionRefused
	}
	capability, err := brokerpipe.AuthorizePeer(ctx, peer)
	if err != nil || !e.busy.TryLock() {
		return result, errDecommissionRefused
	}
	defer e.busy.Unlock()
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if err = errors.Join(executionNoImpersonation(), adapter.Validate()); err != nil {
		return result, err
	}
	raw, err := adapter.ReadRegistration(peer.SID)
	if err != nil {
		return result, err
	}
	registered, err := parseRegistration(raw, peer.SID)
	if err != nil {
		return result, err
	}
	registrationDigest := executionOriginalDigest(raw)
	if err = adapter.Store.ValidateSingletonRegistration(ctx, registered.ClientSID, registrationDigest); err != nil {
		return result, err
	}
	err = capability.WithMaintenanceOperatorLocationChecked(ctx, registered.ClientSID, func(_ brokerpipe.PeerIdentity, localAppData string, checkOperator func(context.Context) error) (callbackErr error) {
		barrier, err := launcher.CaptureMaintenanceBarrierNative(ctx, localAppData)
		if err != nil {
			return err
		}
		defer func() { callbackErr = errors.Join(callbackErr, barrier.Close()) }()
		selected, err := captureSelectedExecutionNative(ctx, adapter.Store, registered)
		if err != nil {
			return err
		}
		defer func() { callbackErr = errors.Join(callbackErr, selected.Close()) }()
		if selected.selected.head != q.ExpectedSelectedHeadSHA256 {
			return errDecommissionRefused
		}
		original, state, err := e.observeApplicationOwnerLocked(ctx, registered.ClientSID)
		if err != nil || state != executionApplicationExited || original == nil || original.generation() != selected.selected.generation {
			return errors.Join(errDecommissionRefused, err)
		}
		// This service-global action accepts exactly the original selected client. An
		// extra/uncertain service-local owner is not inferred absent from a PID list.
		if len(e.applications.entries) != 1 {
			return errDecommissionRefused
		}
		observed, stock, err := e.observe(ctx, registered)
		if stock != nil {
			defer func() { callbackErr = errors.Join(callbackErr, stock.Close()) }()
		}
		if err != nil || stock == nil || observed != stockBaseline {
			return errors.Join(errDecommissionRefused, err)
		}
		source, err := captureProfileTreeNative(ctx)
		if err != nil {
			return err
		}
		profile := &nativeManualRollbackProfile{source: source, snapshot: brokerstore.ExecutionJournalSnapshot{ClientSID: registered.ClientSID, ExecutionID: q.RequestID, Generation: selected.selected.generation, CompletionSHA256: selected.selected.completion}}
		defer func() { callbackErr = errors.Join(callbackErr, profile.close()) }()
		frozen, err := profile.validate(ctx)
		if err != nil {
			return err
		}
		// This standalone guard never calls ordinary Store/generation/journal checks:
		// those are intentionally denied as soon as any terminal fence exists.
		standalone := func(ctx context.Context) error {
			if ctx == nil || ctx.Err() != nil {
				return errDecommissionRefused
			}
			if err := errors.Join(checkOperator(ctx), executionNoImpersonation(), barrier.Validate(ctx), stock.validate(), profileNoNamedProcesses(ctx)); err != nil {
				return err
			}
			owner, state, err := e.observeApplicationOwnerLocked(ctx, registered.ClientSID)
			if err != nil || owner != original || state != executionApplicationExited || owner.generation() != selected.selected.generation || len(e.applications.entries) != 1 {
				return errors.Join(errDecommissionRefused, err)
			}
			current, err := profile.validate(ctx)
			if err != nil || current != frozen {
				return errors.Join(errDecommissionRefused, err)
			}
			return errors.Join(checkOperator(ctx), ctx.Err())
		}
		guard := func(ctx context.Context) error {
			if err := errors.Join(standalone(ctx), adapter.Validate(), selected.validate()); err != nil {
				return err
			}
			current, err := adapter.ReadRegistration(registered.ClientSID)
			if err != nil || executionOriginalDigest(current) != registrationDigest {
				return errors.Join(errDecommissionRefused, err)
			}
			return standalone(ctx)
		}
		binding := brokerstore.DecommissionBinding{RequestID: q.RequestID, ClientSID: registered.ClientSID, RegistrationSHA256: registrationDigest, SelectedHeadSHA256: selected.selected.head, GenerationID: selected.selected.generation, CompletionSHA256: selected.selected.completion, ProfileSHA256: frozen.contents, ProfileIdentitySHA256: frozen.identity}
		if q.Operation == brokerpipe.PrepareUninstall {
			// Capture original physical identities and protected parent custody while
			// the full selected owner still validates. Only then release its read-only
			// generation/journal handles, which cannot coexist with DELETE custody.
			preparation, err := adapter.Store.PrepareRetiringDecommission(ctx, binding, guard)
			if err != nil {
				return err
			}
			defer func() { callbackErr = errors.Join(callbackErr, preparation.Close()) }()
			if err = errors.Join(preparation.Validate(ctx), guard(ctx)); err != nil {
				return err
			}
			// Commit must match the previously captured original identities before the
			// terminal fence; no pathname scan or post-fence reopening grants authority.
			fence, retirement, err := commitPreparedStartupRetirement(ctx, selected.Close, func(ctx context.Context, drain func(context.Context, string) error) (retirementReceipt, error) {
				return preparation.CommitWithStartupDrain(ctx, standalone, drain)
			}, standalone, func(ctx context.Context, fence string) error {
				// The interim witness permits original shortcut restoration and child
				// handle drainage only. The same pipe waits for that acknowledgement
				// before Store can rename; no final witness or SCM authority exists yet.
				return publishFence(ctx, brokerpipe.MaintenanceResult{Schema: 1, RequestID: q.RequestID, State: "uninstall-fenced", Generation: binding.GenerationID, CompletionSHA256: binding.CompletionSHA256, SelectedHeadSHA256: binding.SelectedHeadSHA256, ClientSID: binding.ClientSID, RegistrationSHA256: binding.RegistrationSHA256, CurrentProfileSHA256: binding.ProfileSHA256, CurrentProfileIdentitySHA256: binding.ProfileIdentitySHA256, FenceSHA256: fence})
			})
			if err != nil {
				return err
			}
			result = brokerpipe.MaintenanceResult{Schema: 1, RequestID: q.RequestID, State: "uninstall-prepared", Generation: binding.GenerationID, CompletionSHA256: binding.CompletionSHA256, SelectedHeadSHA256: binding.SelectedHeadSHA256, ClientSID: binding.ClientSID, RegistrationSHA256: binding.RegistrationSHA256, CurrentProfileSHA256: binding.ProfileSHA256, CurrentProfileIdentitySHA256: binding.ProfileIdentitySHA256, FenceSHA256: fence, RetirementSHA256: retirement}
			return nil
		}
		receipt, err := adapter.Store.CommitDecommission(ctx, binding, guard)
		if err != nil {
			return err
		}
		// Store owns the original receipt until host shutdown. Releasing selected
		// readers uses only its sealed terminal custody; ordinary validation refuses.
		fence, err := finishDecommissionPreparation(ctx, receipt, selected.readers, standalone)
		if err != nil {
			return err
		}
		result = brokerpipe.MaintenanceResult{Schema: 1, RequestID: q.RequestID, State: "decommission-prepared", Generation: binding.GenerationID, CompletionSHA256: binding.CompletionSHA256, SelectedHeadSHA256: binding.SelectedHeadSHA256, ClientSID: binding.ClientSID, RegistrationSHA256: binding.RegistrationSHA256, CurrentProfileSHA256: binding.ProfileSHA256, CurrentProfileIdentitySHA256: binding.ProfileIdentitySHA256, FenceSHA256: fence}
		return nil
	})
	return result, err
}
