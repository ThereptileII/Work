//go:build windows

package brokerhost

import (
	"context"
	"errors"
	"runtime"

	"example.com/opennav-update-verifier/brokerpipe"
	"example.com/opennav-update-verifier/brokerstore"
)

// This endpoint returns only data observed under the original service Store,
// operation lock and live elevated registered peer. It never opens the profile,
// grants a maintenance capability or authorizes a later request.
func (e *engine) handleSelectedHistoryDiagnostic(ctx context.Context, peer brokerpipe.Peer, q brokerpipe.MaintenanceRequest) (result brokerpipe.MaintenanceResult, resultErr error) {
	defer func() {
		if resultErr != nil {
			result = brokerpipe.MaintenanceResult{}
			resultErr = errExecutionSelection
		}
	}()
	if e == nil || ctx == nil || ctx.Err() != nil || (q.Operation != brokerpipe.SelectedHistory && q.Operation != brokerpipe.PreviousQualified) || q.Schema != 1 || (q.Operation == brokerpipe.SelectedHistory && q.Page == nil) {
		return result, errExecutionSelection
	}
	adapter, ok := e.store.(storeAdapter)
	if !ok || adapter.Store == nil {
		return result, errExecutionSelection
	}
	capability, err := brokerpipe.AuthorizePeer(ctx, peer)
	if err != nil || !e.busy.TryLock() {
		return result, errExecutionSelection
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
	err = capability.WithMaintenanceOperatorChecked(ctx, registered.ClientSID, func(_ brokerpipe.PeerIdentity, checkOperator func(context.Context) error) (callbackErr error) {
		if err := checkOperator(ctx); err != nil {
			return err
		}
		selected, err := captureSelectedExecutionNative(ctx, adapter.Store, registered)
		if err != nil {
			return err
		}
		defer func() { callbackErr = errors.Join(callbackErr, selected.Close()) }()
		if q.Operation == brokerpipe.SelectedHistory && *q.Page != 0 && (selected.selected.head != q.ExpectedSelectedHeadSHA256 || selected.inventory != q.InventorySHA256) {
			return errExecutionSelection
		}
		attempts, err := selectedHistoryOriginalAttempts(selected)
		if err != nil {
			return err
		}
		if q.Operation == brokerpipe.PreviousQualified {
			result, err = previousQualified(attempts, registered.Installed.Generation, selected.selected, q.RequestID)
		} else {
			result, err = selectedHistoryPage(attempts, registered.Installed.Generation, selected.selected, selected.inventory, q.RequestID, *q.Page)
		}
		if err != nil {
			return err
		}
		current, err := adapter.ReadRegistration(registered.ClientSID)
		if err != nil || executionOriginalDigest(current) != registrationDigest {
			return errExecutionSelection
		}
		return errors.Join(checkOperator(ctx), selected.validate(), adapter.Validate(), ctx.Err())
	})
	return result, err
}

func selectedHistoryOriginalAttempts(selected *nativeSelectedExecution) ([]executionSelectionAttempt, error) {
	if selected == nil || selected.closed || selected.store == nil || selected.ctx == nil || selected.ctx.Err() != nil {
		return nil, errExecutionSelection
	}
	attempts := make([]executionSelectionAttempt, 0, len(selected.readers))
	for _, reader := range selected.readers {
		snapshot, err := reader.Snapshot()
		if err != nil || snapshot.ClientSID != selected.registration.ClientSID {
			return nil, errExecutionSelection
		}
		facts := make([]executionSelectionFact, 0, len(snapshot.Facts))
		for _, fact := range snapshot.Facts {
			facts = append(facts, executionSelectionFact{fact.Kind, fact.Original})
		}
		attempt, err := parseExecutionSelectionAttempt(snapshot.ClientSID, snapshot.ExecutionID, snapshot.Generation,
			snapshot.CompletionSHA256, snapshot.GenerationPurpose, snapshot.JournalSHA256, facts)
		if err != nil {
			return nil, err
		}
		attempts = append(attempts, attempt)
	}
	if len(attempts) > 128 {
		return nil, brokerstore.ErrExecutionJournalUncertain
	}
	return attempts, selected.validate()
}
