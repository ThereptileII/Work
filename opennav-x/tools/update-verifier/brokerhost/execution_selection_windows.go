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
	selected           executionSelectedVersion
	closed             bool
	closeErr           error
}

func captureSelectedExecutionNative(ctx context.Context, store *brokerstore.Store, r registration) (result *nativeSelectedExecution, resultErr error) {
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
	case brokerstore.GenerationPurposeSignedCandidate:
		candidate, err := owner.generation.Candidate()
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
	if s == nil || s.closed || s.store == nil || s.ctx == nil || s.ctx.Err() != nil || s.generation == nil || s.selected.generation == "" {
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
	if err := s.generation.Validate(); err != nil {
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
func (s *nativeSelectedExecution) Close() error {
	if s == nil {
		return nil
	}
	if s.closed {
		return s.closeErr
	}
	s.closed = true
	if s.generation != nil {
		s.closeErr = errors.Join(s.closeErr, s.generation.Close())
	}
	for i := len(s.readers) - 1; i >= 0; i-- {
		s.closeErr = errors.Join(s.closeErr, s.readers[i].Close())
	}
	return s.closeErr
}
