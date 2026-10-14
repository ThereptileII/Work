package brokerhost

import (
	"context"
	"errors"

	"example.com/opennav-update-verifier/brokerstore"
	"example.com/opennav-update-verifier/releasepolicy"
)

// The actual SCM host retains original journal readers for the whole staging
// operation. This owner is private; neither pipe reports nor a selected JSON file
// can construct it. Store's exclusive lifetime also excludes another writer.
type nativeSelectedExecution struct {
	store              *brokerstore.Store
	ctx                context.Context
	inventory          string
	registration       registration
	registrationDigest string
	readers            []*brokerstore.ExecutionJournalReader
	generation         *brokerstore.ReopenedGeneration
	repairSource       *brokerstore.RepairSource
	selected           executionSelectedVersion
	closed             bool
	closeErr           error
}

func captureSelectedExecutionNative(ctx context.Context, store *brokerstore.Store, r registration) (result *nativeSelectedExecution, resultErr error) {
	return captureSelectedOwnerNative(ctx, store, r, false)
}
func captureRepairSelectionNative(ctx context.Context, store *brokerstore.Store, r registration) (*nativeSelectedExecution, error) {
	return captureSelectedOwnerNative(ctx, store, r, true)
}
func captureSelectedOwnerNative(ctx context.Context, store *brokerstore.Store, r registration, repairOnly bool) (result *nativeSelectedExecution, resultErr error) {
	if ctx == nil || store == nil || ctx.Err() != nil {
		return nil, errExecutionSelection
	}
	owner := &nativeSelectedExecution{store: store, ctx: ctx, registration: r}
	defer func() {
		if resultErr != nil {
			resultErr = errors.Join(errExecutionSelection, resultErr, owner.Close())
		}
	}()
	raw, err := store.ReadRegistration(r.ClientSID)
	if err != nil {
		return nil, err
	}
	parsed, err := parseRegistration(raw, r.ClientSID)
	if err != nil || parsed.ApprovedRequestSHA256 != r.ApprovedRequestSHA256 || parsed.Installed != r.Installed {
		return nil, errExecutionSelection
	}
	owner.registrationDigest = executionOriginalDigest(raw)
	entries, err := store.EnumerateExecutionJournals(ctx, r.ClientSID)
	if err != nil {
		return nil, err
	}
	owner.inventory, err = executionSelectionInventoryDigest(r.ClientSID, entries)
	if err != nil {
		return nil, err
	}
	attempts := make([]executionSelectionAttempt, 0, len(entries))
	for _, entry := range entries {
		if !entry.Readable {
			return nil, errExecutionSelection
		}
		reader, err := store.OpenExecutionJournal(ctx, r.ClientSID, entry.ExecutionID)
		if reader != nil {
			owner.readers = append(owner.readers, reader)
		}
		if err != nil || reader == nil {
			return nil, errExecutionSelection
		}
		snapshot, err := reader.Snapshot()
		if err != nil || snapshot.ClientSID != r.ClientSID || snapshot.ExecutionID != entry.ExecutionID {
			return nil, errExecutionSelection
		}
		facts := make([]executionSelectionFact, 0, len(snapshot.Facts))
		for _, fact := range snapshot.Facts {
			facts = append(facts, executionSelectionFact{fact.Kind, fact.Original})
		}
		attempt, err := parseExecutionSelectionAttempt(r.ClientSID, snapshot.ExecutionID, snapshot.Generation, snapshot.CompletionSHA256, snapshot.GenerationPurpose, snapshot.JournalSHA256, facts)
		if err != nil {
			return nil, err
		}
		attempts = append(attempts, attempt)
	}
	owner.selected, _, err = foldExecutionSelection(attempts, r.Installed.Generation)
	if err != nil {
		return nil, err
	}
	repairHistory := map[string]string{}
	for _, a := range attempts {
		if a.repair != nil && a.terminal == "repair-selected" {
			repairHistory[a.begin.Generation] = a.begin.CompletionSHA256
		}
	}
	if repairOnly {
		owner.repairSource, err = store.OpenRepairSource(ctx, r.ClientSID, owner.selected.generation)
		if err != nil {
			return nil, err
		}
		if err = owner.repairSource.ValidateHistory(ctx, repairHistory); err != nil {
			return nil, err
		}
		c, e := owner.repairSource.Snapshot(ctx)
		if e != nil || c.CompletionSHA256 != owner.selected.completion || c.Identity.Version != owner.selected.installed.Version || c.Identity.Commit != owner.selected.installed.Commit {
			return nil, errExecutionSelection
		}
		if err = owner.validate(); err != nil {
			return nil, err
		}
		return owner, nil
	}
	owner.generation, err = store.OpenReadOnlyGeneration(ctx, r.ClientSID, owner.selected.generation)
	if err != nil || owner.generation == nil {
		return nil, errExecutionSelection
	}
	switch owner.generation.Purpose() {
	case brokerstore.GenerationPurposeApprovedBaseline:
		baseline, err := owner.generation.ApprovedBaseline()
		if err != nil {
			return nil, err
		}
		completion, err := baseline.Completion()
		if err != nil || completion.ApprovedRequestSHA256 != r.ApprovedRequestSHA256 || completion.CompletionSHA256 != owner.selected.completion || completion.Identity.Version != owner.selected.installed.Version || completion.Identity.Commit != owner.selected.installed.Commit {
			return nil, errExecutionSelection
		}
	case brokerstore.GenerationPurposeSignedCandidate, brokerstore.GenerationPurposeRepaired:
		var candidate *brokerstore.ReadOnlyGeneration
		var err error
		if owner.generation.Purpose() == brokerstore.GenerationPurposeRepaired {
			candidate, err = owner.generation.Repaired()
		} else {
			candidate, err = owner.generation.Candidate()
		}
		if err != nil {
			return nil, err
		}
		completion, err := candidate.Completion()
		if err != nil || completion.CompletionSHA256 != owner.selected.completion || completion.Identity.Version != owner.selected.installed.Version || completion.Identity.Commit != owner.selected.installed.Commit {
			return nil, errExecutionSelection
		}
	default:
		return nil, errExecutionSelection
	}
	if err = owner.generation.ValidateRepairHistory(ctx, repairHistory); err != nil {
		return nil, err
	}
	if owner.generation.Purpose() == brokerstore.GenerationPurposeRepaired {
		origin, e := owner.generation.RepairOrigin(ctx)
		if e != nil {
			return nil, e
		}
		matched := false
		for _, a := range attempts {
			if a.repair != nil && a.begin.Generation == owner.selected.generation && a.begin.CompletionSHA256 == owner.selected.completion {
				if matched || origin.SourceCompletionSHA256 != a.repair.SourceCompletionSHA256 || origin.PreviousGeneration != a.begin.PreviousGeneration || origin.PreviousCompletionSHA256 != a.repair.PreviousCompletionSHA256 || origin.PreviousSelectedHeadSHA256 != a.begin.PreviousHeadSHA256 {
					return nil, errExecutionSelection
				}
				matched = true
			}
		}
		if !matched {
			return nil, errExecutionSelection
		}
	}
	if err = owner.validate(); err != nil {
		return nil, err
	}
	if err = ctx.Err(); err != nil {
		return nil, err
	}
	return owner, nil
}

func (s *nativeSelectedExecution) installed() releasepolicy.Installed { return s.selected.installed }
func (s *nativeSelectedExecution) validate() error {
	if s == nil || s.closed || s.store == nil || s.ctx == nil || s.ctx.Err() != nil || (s.generation == nil && s.repairSource == nil) || s.selected.generation == "" {
		return errExecutionSelection
	}
	if err := s.validateCustody(); err != nil {
		return err
	}
	entries, err := s.store.EnumerateExecutionJournals(s.ctx, s.registration.ClientSID)
	if err != nil {
		return err
	}
	digest, err := executionSelectionInventoryDigest(s.registration.ClientSID, entries)
	if err != nil || digest != s.inventory {
		return errExecutionSelection
	}
	return s.ctx.Err()
}
func (s *nativeSelectedExecution) validateCustody() error {
	if s == nil || s.closed || s.ctx == nil || s.ctx.Err() != nil {
		return errExecutionSelection
	}
	if err := s.store.Validate(); err != nil {
		return err
	}
	raw, err := s.store.ReadRegistration(s.registration.ClientSID)
	if err != nil || executionOriginalDigest(raw) != s.registrationDigest {
		return errExecutionSelection
	}
	for _, reader := range s.readers {
		if err := reader.Validate(); err != nil {
			return err
		}
	}
	if s.generation != nil {
		if err := s.generation.Validate(); err != nil {
			return err
		}
	}
	if s.repairSource != nil {
		if err := s.repairSource.Validate(s.ctx); err != nil {
			return err
		}
	}

	return s.ctx.Err()
}
func (s *nativeSelectedExecution) Close() error {
	if s == nil {
		return nil
	}
	if s.closed {
		return s.closeErr
	}
	s.closed = true
	if s.repairSource != nil {
		s.closeErr = errors.Join(s.closeErr, s.repairSource.Close())
	}
	if s.generation != nil {
		s.closeErr = errors.Join(s.closeErr, s.generation.Close())
	}
	for i := len(s.readers) - 1; i >= 0; i-- {
		s.closeErr = errors.Join(s.closeErr, s.readers[i].Close())
	}
	return s.closeErr
}
