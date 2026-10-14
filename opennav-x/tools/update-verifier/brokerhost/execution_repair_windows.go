//go:build windows

package brokerhost

import (
	"context"
	"encoding/json"
	"errors"
	"runtime"

	"example.com/opennav-update-verifier/brokerpipe"
	"example.com/opennav-update-verifier/brokerstore"
	"example.com/opennav-update-verifier/launcher"
)

// This is a no-launch same-version restoration. Original ordinary-child exit
// custody (or explicit native frozen cold custody), actual elevated operator,
// stock and CURRENT frozen profile are held
// across copying and committed-history readback. No trust or profile is written.
func (e *engine) handleNativeRepair(ctx context.Context, peer brokerpipe.Peer, q brokerpipe.MaintenanceRequest) (result brokerpipe.MaintenanceResult, resultErr error) {
	defer func() {
		if resultErr != nil {
			result = brokerpipe.MaintenanceResult{}
			resultErr = errExecutionSelection
		}
	}()
	if e == nil || ctx == nil || ctx.Err() != nil || e.observe == nil || !brokerpipe.ValidMaintenanceClientRequest(q) || (q.Operation != brokerpipe.RepairInspect && q.Operation != brokerpipe.RepairCurrent) {
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
	err = capability.WithMaintenanceOperatorLocationChecked(ctx, registered.ClientSID, func(_ brokerpipe.PeerIdentity, local string, checkOperator func(context.Context) error) (callbackErr error) {
		selected, err := captureRepairSelectionNative(ctx, adapter.Store, registered)
		if err != nil {
			return err
		}
		defer func() { callbackErr = errors.Join(callbackErr, selected.Close()) }()
		source, err := selected.repairSource.Snapshot(ctx)
		if err != nil {
			return err
		}
		attempts, err := manualRollbackOriginalAttempts(selected)
		if err != nil {
			return err
		}
		var ancestor *executionSelectionAttempt
		for i := range attempts {
			if q.Operation == brokerpipe.RepairCurrent && attempts[i].begin.ExecutionID == q.RequestID {
				return errExecutionSelection
			}
			a := &attempts[i]
			if a.repair == nil && a.manual == nil && a.recovery == nil && a.terminal == "selected" && a.begin.Generation == source.SourceGeneration && a.begin.CompletionSHA256 == source.SourceCompletionSHA256 {
				if ancestor != nil {
					return errExecutionSelection
				}
				ancestor = a
			}
		}
		if ancestor == nil || ancestor.begin.Version != source.Identity.Version || ancestor.begin.Commit != source.Identity.Commit {
			return errExecutionSelection
		}
		if err = errors.Join(selected.validate(), checkOperator(ctx)); err != nil {
			return err
		}
		if q.Operation == brokerpipe.RepairInspect {
			result = brokerpipe.MaintenanceResult{Schema: 1, RequestID: q.RequestID, State: brokerpipe.RepairInspect, Generation: selected.selected.generation, CompletionSHA256: selected.selected.completion, SelectedHeadSHA256: selected.selected.head, Version: source.Identity.Version, Commit: source.Identity.Commit}
			return nil
		}
		if selected.selected.head != q.ExpectedSelectedHeadSHA256 {
			return errExecutionSelection
		}
		barrier, err := launcher.CaptureMaintenanceBarrierNative(ctx, local)
		if err != nil {
			return err
		}
		defer func() { callbackErr = errors.Join(callbackErr, barrier.Close()) }()
		observed, stock, err := e.observe(ctx, registered)
		if stock != nil {
			defer func() { callbackErr = errors.Join(callbackErr, stock.Close()) }()
		}
		if err != nil || stock == nil || observed != stockBaseline {
			return errExecutionSelection
		}
		profileSource, err := captureProfileTreeNative(ctx)
		if err != nil {
			return err
		}
		profile := &nativeManualRollbackProfile{source: profileSource, snapshot: brokerstore.ExecutionJournalSnapshot{ClientSID: registered.ClientSID, ExecutionID: q.RequestID, Generation: selected.selected.generation, CompletionSHA256: selected.selected.completion}}
		defer func() { callbackErr = errors.Join(callbackErr, profile.close()) }()
		custody, err := captureRepairApplicationCustodyLocked(ctx, e, registered.ClientSID, selected.selected.generation, profileSource)
		if err != nil {
			return err
		}
		defer func() { callbackErr = errors.Join(callbackErr, custody.close()) }()
		guard := func(ctx context.Context) error {
			if err := errors.Join(checkOperator(ctx), executionNoImpersonation(), adapter.Validate(), barrier.Validate(ctx), stock.validate(), profileNoNamedProcesses(ctx)); err != nil {
				return err
			}
			if err := custody.validate(ctx); err != nil {
				return err
			}
			return selected.validateCustody()
		}
		if err = guard(ctx); err != nil {
			return err
		}

		before, err := profile.validate(ctx)
		if err != nil {
			return err
		}
		check := func() error {
			after, err := profile.validate(ctx)
			if err != nil || after != before {
				return errors.Join(errExecutionSelection, err)
			}
			return guard(ctx)
		}
		if err = check(); err != nil {
			return err
		}
		// A failure from here may have durable CREATE_NEW state. Preserve it, never
		// delete, reuse the request ID, re-enroll or automatically attempt again.
		var repaired *brokerstore.ReadOnlyGeneration
		if selected.interruptedRepair != nil {
			// This exact comparison came from original protected metadata,
			// never a customer target/path. Full member/ID/namespace reopening
			// is mandatory: modified or unknown additions are not adopted.
			original, err := adapter.Store.OpenReadOnlyGeneration(ctx, registered.ClientSID, selected.interruptedRepair.Generation)
			if original != nil {
				defer func() { callbackErr = errors.Join(callbackErr, original.Close()) }()
			}
			if err != nil {
				return err
			}
			repaired, err = original.Repaired()
			if err != nil {
				return err
			}
			completion, e := repaired.Completion()
			if e != nil || completion.CompletionSHA256 != selected.interruptedRepair.CompletionSHA256 {
				return errExecutionSelection
			}
		} else {
			generation, err := adapter.Store.MaterializeRepairFresh(ctx, selected.repairSource, q.RequestID, selected.selected.head)
			if generation != nil {
				defer func() { callbackErr = errors.Join(callbackErr, generation.Close()) }()
			}
			if err != nil {
				return err
			}
			repaired, err = generation.TransitionReadOnly(ctx)
			if err != nil {
				return err
			}
			defer func() { callbackErr = errors.Join(callbackErr, repaired.Close()) }()
		}
		completion, err := repaired.Completion()
		if err != nil {
			return err
		}
		if err = check(); err != nil {
			return err
		}
		var journal *brokerstore.ExecutionJournal
		if selected.repairPending != nil {
			var prior *brokerstore.ExecutionJournalReader
			for _, reader := range selected.readers {
				snapshot, e := reader.Snapshot()
				if e != nil {
					return e
				}
				if snapshot.ExecutionID == selected.repairPending.begin.ExecutionID && snapshot.JournalSHA256 == selected.repairPending.head {
					if prior != nil {
						return errExecutionSelection
					}
					prior = reader
				}
			}
			if prior == nil {
				return errExecutionSelection
			}
			journal, err = adapter.Store.BeginRepairRecoveryJournal(ctx, repaired, q.RequestID, prior)
		} else {
			journal, err = adapter.Store.BeginRepairJournal(ctx, repaired, q.RequestID)
		}
		if journal != nil {
			defer func() { callbackErr = errors.Join(callbackErr, journal.Close()) }()
		}
		if err != nil {
			return err
		}
		b := executionRepairBegin{executionSelectionBegin: executionSelectionBegin{Schema: 1, Purpose: "broker-selected-repair-v1", ExecutionID: q.RequestID, ClientSID: registered.ClientSID, Generation: completion.Generation, Version: source.Identity.Version, Commit: source.Identity.Commit, CompletionSHA256: completion.CompletionSHA256, PreviousGeneration: selected.selected.generation, PreviousHeadSHA256: selected.selected.head}, PreviousCompletionSHA256: selected.selected.completion, SourceGeneration: source.SourceGeneration, SourceCompletionSHA256: source.SourceCompletionSHA256, SourceQualifiedHeadSHA256: ancestor.head, CurrentProfileSHA256: before.contents, CurrentProfileIdentitySHA256: before.identity}
		if selected.repairPending != nil {
			b.Purpose = "broker-selected-repair-recovery-v1"
			b.PriorExecutionID = selected.repairPending.begin.ExecutionID
			b.PriorJournalSHA256 = selected.repairPending.head
		}
		appendFact := func(kind brokerstore.ExecutionFactKind, record any) error {
			if err := check(); err != nil {
				return err
			}
			snapshot, err := journal.Snapshot()
			if err != nil {
				return err
			}
			entries, enumErr := adapter.Store.EnumerateExecutionJournals(ctx, registered.ClientSID)
			if err = executionHostInventory(registered.ClientSID, q.RequestID, selected.inventory, entries, enumErr, &snapshot); err != nil {
				return err
			}
			raw, err := json.Marshal(record)
			if err != nil {
				return err
			}
			return journal.AppendFact(ctx, kind, raw)
		}
		if err = appendFact(brokerstore.ExecutionPreparation, b); err != nil {
			return err
		}
		for _, step := range []string{"payload-restored", "repair-selected"} {
			if err = appendFact(brokerstore.ExecutionRestoration, executionSelectionStep{Schema: 1, Purpose: "broker-selected-repair-step-v1", ExecutionID: q.RequestID, Step: step, Generation: completion.Generation, EvidenceSHA256: completion.CompletionSHA256}); err != nil {
				return err
			}
		}
		snapshot, err := journal.Snapshot()
		if err != nil {
			return err
		}
		if err = journal.Close(); err != nil {
			return err
		}
		resolved, err := captureSelectedExecutionNative(ctx, adapter.Store, registered)
		if err != nil {
			return err
		}
		defer func() { callbackErr = errors.Join(callbackErr, resolved.Close()) }()
		if resolved.selected.generation != completion.Generation || resolved.selected.completion != completion.CompletionSHA256 || resolved.selected.head != snapshot.JournalSHA256 || resolved.selected.installed != selected.selected.installed {
			return errExecutionSelection
		}
		if err = errors.Join(check(), resolved.validate()); err != nil {
			return err
		}
		if err = custody.finish(ctx); err != nil {
			return err
		}
		result = brokerpipe.MaintenanceResult{Schema: 1, RequestID: q.RequestID, State: "repair-selected", Generation: resolved.selected.generation, CompletionSHA256: resolved.selected.completion, SelectedHeadSHA256: resolved.selected.head, Version: source.Identity.Version, Commit: source.Identity.Commit}
		return nil
	})
	return result, err
}
