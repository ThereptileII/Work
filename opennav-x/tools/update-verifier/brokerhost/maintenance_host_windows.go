//go:build windows

package brokerhost

import (
	"context"
	"errors"
	"runtime"

	"example.com/opennav-update-verifier/brokerpipe"
	"example.com/opennav-update-verifier/launcher"
)

// Manual selection is a separate elevated operator action. It never stops an
// ordinary application, restores an old profile, resets trust, or launches the
// historical version. Current data and original process-exit custody are kept
// through committed-history readback. Missing custody after SCM restart refuses.
func (e *engine) handleNativeMaintenance(ctx context.Context, peer brokerpipe.Peer, q brokerpipe.MaintenanceRequest) (result brokerpipe.MaintenanceResult, resultErr error) {
	if q.Operation == brokerpipe.PrepareDecommission || q.Operation == brokerpipe.PrepareUninstall {
		return e.handlePrepareDecommission(ctx, peer, q)
	}
	if q.Operation == brokerpipe.SelectedHistory {
		return e.handleSelectedHistoryDiagnostic(ctx, peer, q)
	}
	defer func() {
		if resultErr != nil {
			result = brokerpipe.MaintenanceResult{}
			resultErr = ErrManualRollbackRefused // Do not disclose native paths/errors.
		}
	}()
	if e == nil || ctx == nil || ctx.Err() != nil || e.observe == nil || q.Schema != 1 || q.Operation != brokerpipe.ManualRollback {
		return result, ErrManualRollbackRefused
	}
	request := ManualRollbackRequest{RequestID: q.RequestID, ExpectedSelectedHeadSHA256: q.ExpectedSelectedHeadSHA256,
		TargetGeneration: q.TargetGeneration, TargetCompletionSHA256: q.TargetCompletionSHA256}
	if !validManualRollbackRequest(request) {
		return result, ErrManualRollbackRefused
	}
	adapter, ok := e.store.(storeAdapter)
	if !ok || adapter.Store == nil {
		return result, ErrManualRollbackRefused
	}
	capability, err := brokerpipe.AuthorizePeer(ctx, peer)
	if err != nil || !e.busy.TryLock() {
		return result, ErrManualRollbackRefused
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
	err = capability.WithMaintenanceOperatorLocationChecked(ctx, registered.ClientSID, func(identity brokerpipe.PeerIdentity, localAppData string, checkOperator func(context.Context) error) (callbackErr error) {
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
		if selected.selected.head != request.ExpectedSelectedHeadSHA256 {
			return ErrManualRollbackRefused
		}
		original, state, err := e.observeApplicationOwnerLocked(ctx, registered.ClientSID)
		if err != nil || state != executionApplicationExited || original == nil || original.generation() != selected.selected.generation {
			return errors.Join(ErrManualRollbackRefused, err)
		}
		observed, stock, err := e.observe(ctx, registered)
		if stock != nil {
			defer func() { callbackErr = errors.Join(callbackErr, stock.Close()) }()
		}
		if err != nil || stock == nil || observed != stockBaseline {
			return errors.Join(ErrManualRollbackRefused, err)
		}
		guard := func(ctx context.Context) error {
			if ctx == nil || ctx.Err() != nil {
				return ErrManualRollbackRefused
			}
			if err := errors.Join(checkOperator(ctx), executionNoImpersonation(), adapter.Validate(), barrier.Validate(ctx), stock.validate(), profileNoNamedProcesses(ctx)); err != nil {
				return err
			}
			current, err := adapter.ReadRegistration(registered.ClientSID)
			if err != nil || executionOriginalDigest(current) != registrationDigest {
				return errors.Join(ErrManualRollbackRefused, err)
			}
			owner, state, err := e.observeApplicationOwnerLocked(ctx, registered.ClientSID)
			if err != nil || owner != original || state != executionApplicationExited || owner.generation() != selected.selected.generation {
				return errors.Join(ErrManualRollbackRefused, err)
			}
			return errors.Join(checkOperator(ctx), ctx.Err())
		}
		if err = guard(ctx); err != nil {
			return err
		}
		transaction, err := prepareExecutionManualRollbackNative(ctx, selected, request)
		if err != nil {
			return err
		}
		defer func() { callbackErr = errors.Join(callbackErr, transaction.Close()) }()
		// The native writer retains the CURRENT frozen profile. Recheck every outer
		// guard alongside its original journal/history check before each fact.
		validate := transaction.run.validate
		transaction.run.validate = func(ctx context.Context) error {
			return errors.Join(guard(ctx), validate(ctx))
		}
		if err = transaction.execute(ctx); err != nil {
			return err
		}
		outcome, err := transaction.outcome(ctx)
		if err != nil {
			return err
		}
		if err = guard(ctx); err != nil {
			return errors.Join(ErrManualRollbackUncertain, err)
		}
		if err = e.releaseApplicationOwnerLocked(ctx, registered.ClientSID); err != nil {
			return errors.Join(ErrManualRollbackUncertain, err)
		}
		result = brokerpipe.MaintenanceResult{Schema: 1, RequestID: outcome.RequestID, State: "manual-selected",
			Generation: outcome.Generation, CompletionSHA256: outcome.CompletionSHA256, SelectedHeadSHA256: outcome.SelectedHeadSHA256}
		return nil
	})
	return result, err
}
