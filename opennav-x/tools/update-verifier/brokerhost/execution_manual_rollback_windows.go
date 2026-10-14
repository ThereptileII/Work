//go:build windows

package brokerhost

import (
	"context"
	"encoding/json"
	"errors"
	"runtime"

	"example.com/opennav-update-verifier/brokerstore"
)

// This is a service-internal transaction, intentionally not installed dispatch.
// The eventual maintenance handler must authenticate the independently approved
// elevated operator and retain its native peer, hold engine.busy, and enforce the
// actual armed/ordinary-child/installation exclusion gates BEFORE preparation.
// The same live Store already holds the broker SID's exclusive store.lock.
// Historical health below is NOT fresh current-profile compatibility or authority
// to start the older program. This transaction sends no creator/restore command.
type nativeExecutionManualRollback struct {
	selected  *nativeSelectedExecution // borrowed; handler retains original owner
	target    *brokerstore.ReopenedGeneration
	journal   *brokerstore.ExecutionJournal
	profile   *nativeManualRollbackProfile
	resolved  *nativeSelectedExecution
	run       *executionManualRollbackRun
	succeeded bool
	reserved  bool
	closed    bool
	closeErr  error
}

type nativeManualRollbackProfile struct {
	source   profileCaptureSource
	snapshot brokerstore.ExecutionJournalSnapshot
	closed   bool
	closeErr error
}

// The outer native transaction owns the frozen source through outcome readback.
// The portable writer only borrows it; returning must not open a writer gap.
type borrowedManualRollbackProfile struct{ owner *nativeManualRollbackProfile }

func (p borrowedManualRollbackProfile) validate(ctx context.Context) (manualRollbackProfileBinding, error) {
	return p.owner.validate(ctx)
}
func (p borrowedManualRollbackProfile) close() error { return nil }

func (p *nativeManualRollbackProfile) validate(ctx context.Context) (manualRollbackProfileBinding, error) {
	if p == nil || p.closed || p.source == nil || ctx == nil {
		return manualRollbackProfileBinding{}, ErrManualRollbackRefused
	}
	if err := p.source.validate(ctx, true); err != nil {
		return manualRollbackProfileBinding{}, err
	}
	manifest, raw, _, err := profileManifest(p.snapshot, p.source)
	if err != nil {
		return manualRollbackProfileBinding{}, err
	}
	// Byte-content identity is separate from current native ancestry, full file
	// IDs and metadata. Both come solely from the original frozen native source.
	type content struct {
		Path      string
		Directory bool
		Bytes     int64
		SHA256    string
	}
	contents := make([]content, 0, len(manifest.Entries))
	for _, e := range manifest.Entries {
		contents = append(contents, content{e.Path, e.Directory, e.Bytes, e.SHA256})
	}
	bytes, err := json.Marshal(contents)
	if err != nil {
		return manualRollbackProfileBinding{}, err
	}
	if err = p.source.validate(ctx, true); err != nil {
		return manualRollbackProfileBinding{}, err
	}
	return manualRollbackProfileBinding{captureDigest(bytes), captureDigest(raw)}, nil
}

func (p *nativeManualRollbackProfile) close() error {
	if p == nil {
		return nil
	}
	if p.closed {
		return p.closeErr
	}
	p.closed = true
	p.closeErr = p.source.close()
	return p.closeErr
}

// prepareExecutionManualRollbackNative can only consume an existing native
// selected owner. It reopens the exact retained ancestor, freezes CURRENT real
// navigation data, and reserves an exclusive fresh journal. It never initializes
// registration, verifier trust or a profile backup, and never appends an old file.
func prepareExecutionManualRollbackNative(ctx context.Context, selected *nativeSelectedExecution, q ManualRollbackRequest) (result *nativeExecutionManualRollback, resultErr error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if ctx == nil || ctx.Err() != nil || selected == nil || selected.closed || selected.store == nil || !validManualRollbackRequest(q) ||
		q.ExpectedSelectedHeadSHA256 != selected.selected.head || q.TargetGeneration == selected.selected.generation {
		return nil, ErrManualRollbackRefused
	}
	if err := errors.Join(executionNoImpersonation(), selected.validate()); err != nil {
		return nil, err
	}
	attempts, err := manualRollbackOriginalAttempts(selected)
	if err != nil {
		return nil, err
	}
	ancestor, err := manualRollbackAncestor(attempts, selected.registration.Installed.Generation, q)
	if err != nil {
		return nil, err
	}
	h := &nativeExecutionManualRollback{selected: selected}
	defer func() {
		if resultErr != nil {
			resultErr = errors.Join(resultErr, h.Close())
			if h.reserved {
				resultErr = errors.Join(ErrManualRollbackUncertain, resultErr)
			}
		}
	}()
	h.target, err = selected.store.OpenReadOnlyGeneration(ctx, selected.registration.ClientSID, q.TargetGeneration)
	if err != nil {
		return nil, err
	}
	completion, version, commit, err := manualRollbackGenerationIdentity(h.target)
	if err != nil || completion != ancestor.begin.CompletionSHA256 || version != ancestor.begin.Version || commit != ancestor.begin.Commit {
		return nil, errors.Join(ErrManualRollbackRefused, err)
	}
	source, err := captureProfileTreeNative(ctx)
	if err != nil {
		return nil, err
	}
	h.profile = &nativeManualRollbackProfile{source: source, snapshot: brokerstore.ExecutionJournalSnapshot{
		ClientSID: selected.registration.ClientSID, ExecutionID: q.RequestID, Generation: q.TargetGeneration, CompletionSHA256: q.TargetCompletionSHA256}}
	profile, err := h.profile.validate(ctx)
	if err != nil {
		return nil, err
	}
	b := executionManualRollbackBegin{executionSelectionBegin: executionSelectionBegin{
		Schema: 1, Purpose: "broker-manual-rollback-v1", ExecutionID: q.RequestID, ClientSID: selected.registration.ClientSID,
		Generation: q.TargetGeneration, Version: version, Commit: commit, CompletionSHA256: completion,
		PreviousGeneration: selected.selected.generation, PreviousHeadSHA256: selected.selected.head},
		QualifiedHeadSHA256: ancestor.head, QualificationSHA256: ancestor.health, QualificationClosedSHA256: ancestor.closed,
		CurrentProfileSHA256: profile.contents, CurrentProfileIdentitySHA256: profile.identity}
	if err = errors.Join(selected.validate(), h.target.Validate()); err != nil {
		return nil, err
	}
	h.reserved = true // a failed reservation may already have durable names
	switch h.target.Purpose() {
	case brokerstore.GenerationPurposeApprovedBaseline:
		baseline, e := h.target.ApprovedBaseline()
		if e != nil {
			return nil, e
		}
		h.journal, err = selected.store.BeginBaselineQualificationJournal(ctx, baseline, q.RequestID)
	case brokerstore.GenerationPurposeSignedCandidate:
		candidate, e := h.target.Candidate()
		if e != nil {
			return nil, e
		}
		h.journal, err = selected.store.BeginExecutionJournal(ctx, candidate, q.RequestID)
	default:
		return nil, ErrManualRollbackRefused
	}
	if err != nil {
		return nil, err
	}
	h.run = &executionManualRollbackRun{begin: b, journal: h.journal, profile: borrowedManualRollbackProfile{h.profile}, validate: h.validate,
		qualification: ancestor.health, qualificationClosed: ancestor.closed, resolve: h.resolve}
	if err = h.validate(ctx); err != nil {
		return nil, err
	}
	return h, nil
}

func manualRollbackOriginalAttempts(selected *nativeSelectedExecution) ([]executionSelectionAttempt, error) {
	attempts := make([]executionSelectionAttempt, 0, len(selected.readers))
	for _, reader := range selected.readers {
		s, err := reader.Snapshot()
		if err != nil || s.ClientSID != selected.registration.ClientSID {
			return nil, errors.Join(ErrManualRollbackRefused, err)
		}
		facts := make([]executionSelectionFact, 0, len(s.Facts))
		for _, f := range s.Facts {
			facts = append(facts, executionSelectionFact{f.Kind, f.Original})
		}
		a, err := parseExecutionSelectionAttempt(s.ClientSID, s.ExecutionID, s.Generation, s.CompletionSHA256, s.GenerationPurpose, s.JournalSHA256, facts)
		if err != nil {
			return nil, err
		}
		attempts = append(attempts, a)
	}
	return attempts, nil
}

func manualRollbackGenerationIdentity(g *brokerstore.ReopenedGeneration) (completion, version, commit string, err error) {
	switch g.Purpose() {
	case brokerstore.GenerationPurposeApprovedBaseline:
		b, e := g.ApprovedBaseline()
		if e != nil {
			return "", "", "", e
		}
		c, e := b.Completion()
		return c.CompletionSHA256, c.Identity.Version, c.Identity.Commit, e
	case brokerstore.GenerationPurposeSignedCandidate, brokerstore.GenerationPurposeRepaired:
		var b *brokerstore.ReadOnlyGeneration
		var e error
		if g.Purpose() == brokerstore.GenerationPurposeRepaired {
			b, e = g.Repaired()
		} else {
			b, e = g.Candidate()
		}
		if e != nil {
			return "", "", "", e
		}
		c, e := b.Completion()
		return c.CompletionSHA256, c.Identity.Version, c.Identity.Commit, e
	}
	return "", "", "", ErrManualRollbackRefused
}

func (h *nativeExecutionManualRollback) validate(ctx context.Context) error {
	if h == nil || h.closed || ctx == nil || ctx.Err() != nil || h.selected == nil || h.target == nil || h.journal == nil {
		return ErrManualRollbackRefused
	}
	if err := errors.Join(executionNoImpersonation(), h.selected.store.Validate(), h.target.Validate(), h.selected.generation.Validate()); err != nil {
		return err
	}
	raw, err := h.selected.store.ReadRegistration(h.selected.registration.ClientSID)
	if err != nil || executionOriginalDigest(raw) != h.selected.registrationDigest {
		return errors.Join(ErrManualRollbackRefused, err)
	}
	for _, reader := range h.selected.readers {
		if err = reader.Validate(); err != nil {
			return err
		}
	}
	if h.resolved != nil {
		return h.resolved.validate()
	}
	snapshot, err := h.journal.Snapshot()
	if err != nil {
		return err
	}
	if h.run == nil || snapshot.ExecutionID != h.run.begin.ExecutionID || snapshot.ClientSID != h.run.begin.ClientSID ||
		snapshot.Generation != h.run.begin.Generation || snapshot.CompletionSHA256 != h.run.begin.CompletionSHA256 {
		return ErrManualRollbackRefused
	}
	entries, enumerationErr := h.selected.store.EnumerateExecutionJournals(ctx, h.selected.registration.ClientSID)
	if err = executionHostInventory(h.selected.registration.ClientSID, snapshot.ExecutionID, h.selected.inventory, entries, enumerationErr, &snapshot); err != nil {
		return err
	}
	after, err := h.journal.Snapshot()
	if err != nil || after.JournalSHA256 != snapshot.JournalSHA256 {
		return errors.Join(ErrManualRollbackRefused, err)
	}
	return ctx.Err()
}

func (h *nativeExecutionManualRollback) execute(ctx context.Context) error {
	if h == nil || h.closed || h.run == nil {
		return ErrManualRollbackRefused
	}
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	err := h.run.execute(ctx)
	h.succeeded = err == nil
	if err != nil && h.reserved {
		// Reservation already changed the durable namespace, even if the
		// writer's first custody check failed before its preparation append.
		err = errors.Join(ErrManualRollbackUncertain, err)
	}
	return err
}

func (h *nativeExecutionManualRollback) outcome(ctx context.Context) (ManualRollbackOutcome, error) {
	if h == nil || h.closed || !h.succeeded || h.resolved == nil || h.run == nil {
		return ManualRollbackOutcome{}, ErrManualRollbackRefused
	}
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if err := h.run.check(ctx); err != nil {
		h.succeeded = false
		return ManualRollbackOutcome{}, errors.Join(ErrManualRollbackUncertain, err)
	}
	b := h.run.begin
	return ManualRollbackOutcome{b.ExecutionID, b.Generation, b.CompletionSHA256, h.resolved.selected.head, b.QualifiedHeadSHA256, b.CurrentProfileSHA256, b.CurrentProfileIdentitySHA256}, nil
}

func (h *nativeExecutionManualRollback) resolve(ctx context.Context) error {
	s, err := h.journal.Snapshot()
	if err != nil {
		return err
	}
	if err = h.journal.Close(); err != nil {
		return err
	}
	h.resolved, err = captureSelectedExecutionNative(ctx, h.selected.store, h.selected.registration)
	if err != nil {
		return err
	}
	b := h.run.begin
	if h.resolved.selected.head != s.JournalSHA256 || h.resolved.selected.generation != b.Generation ||
		h.resolved.selected.completion != b.CompletionSHA256 || h.resolved.selected.installed != ancestorInstalled(executionSelectionAttempt{begin: b.executionSelectionBegin}) {
		return ErrManualRollbackRefused
	}
	return h.resolved.validate()
}

func (h *nativeExecutionManualRollback) Close() error {
	if h == nil {
		return nil
	}
	if h.closed {
		return h.closeErr
	}
	h.closed = true
	if h.profile != nil {
		h.closeErr = errors.Join(h.closeErr, h.profile.close())
	}
	if h.journal != nil {
		h.closeErr = errors.Join(h.closeErr, h.journal.Close())
	}
	if h.resolved != nil {
		h.closeErr = errors.Join(h.closeErr, h.resolved.Close())
	}
	if h.target != nil {
		h.closeErr = errors.Join(h.closeErr, h.target.Close())
	}
	if h.reserved && !h.succeeded {
		// Releasing a prepared but uncommitted owner never silently cancels its
		// durable journal pair or makes a fresh retry safe.
		h.closeErr = errors.Join(h.closeErr, ErrManualRollbackUncertain)
	}
	if h.closeErr != nil {
		h.closeErr = errors.Join(ErrManualRollbackUncertain, h.closeErr)
	}
	return h.closeErr
}
