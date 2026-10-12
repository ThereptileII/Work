package brokerstore

import (
	"context"
	"errors"
	"runtime"
)

var ErrGenerationHistoryRequiresReadOnly = errors.New("live generation history requires atomic readonly materialization")

// A prepared append owns independent readonly history handles. Preparing must
// not mutate the recipient. Installation transfers those handles without IO;
// Close drains only handles that were not transferred. Neither a directory scan
// nor serialized completion bytes may stand in for the live successor owner.
type generationSiblingAppend interface {
	install()
	close() error
}
type generationHistoryBackend interface {
	prepareSibling(context.Context, generationBackend) (generationSiblingAppend, error)
}

// MaterializeCandidateReadOnlyFresh publishes and transfers a signed candidate
// under one Store lease, then appends only that exact live successor to retained
// readonly sibling histories. It grants no selection, launch or health authority.
// A failure after attempted reservation poisons all affected live owners; durable
// partial/completed bytes are never deleted, retried or adopted by a later scan.
func (s *Store) MaterializeCandidateReadOnlyFresh(ctx context.Context, previous *InstalledBaseline, candidate *Candidate) (*ReadOnlyGeneration, error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	s.mu.Lock()
	defer s.mu.Unlock()
	if candidate == nil || candidate.store != s {
		return nil, errors.New("live same-store candidate required")
	}
	return s.materializeReadOnlyHistoryLocked(ctx, candidate.clientSID, func(onMutation func()) (*InstalledGeneration, error) {
		return s.materializeCandidateFreshLocked(ctx, previous, candidate, onMutation)
	})
}

// MaterializeApprovedBaselineReadOnlyFresh has the same atomic history lifetime
// for the independently approved initial baseline. It preserves that distinct
// provenance; it does not turn operator approval into a signed update policy.
func (s *Store) MaterializeApprovedBaselineReadOnlyFresh(ctx context.Context, previous *InstalledBaseline, rawRequest []byte, approvedSHA string) (*ReadOnlyApprovedBaselineGeneration, error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	s.mu.Lock()
	defer s.mu.Unlock()
	if previous == nil || previous.store != s {
		return nil, errors.New("live same-store approved baseline required")
	}
	r, err := s.materializeReadOnlyHistoryLocked(ctx, previous.record.ClientSID, func(onMutation func()) (*InstalledGeneration, error) {
		b, err := s.materializeApprovedBaselineFreshLocked(ctx, previous, rawRequest, approvedSHA, onMutation)
		if err != nil {
			return nil, err
		}
		return b.generation, nil
	})
	if err != nil {
		return nil, err
	}
	return &ReadOnlyApprovedBaselineGeneration{owner: r}, nil
}

// Caller holds Store.mu. Close and every public generation/member operation use
// Store.mu -> generation.mu, so these owners cannot be revoked or observe an
// intermediate namespace while publication/adoption is in progress.
func (s *Store) hasLiveGenerationLocked(client string) bool {
	for _, g := range s.generations {
		if !g.closed && g.record.ClientSID == client {
			return true
		}
	}
	return false
}

func (s *Store) materializeReadOnlyHistoryLocked(ctx context.Context, client string, create func(func()) (*InstalledGeneration, error)) (result *ReadOnlyGeneration, resultErr error) {
	if s.closed.Load() {
		return nil, ErrClosed
	}
	if ctx == nil {
		return nil, errors.New("live history context required")
	}
	ctx = registrationContext{Context: ctx, store: s}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	var owners []*InstalledGeneration
	for _, g := range s.generations {
		if g.closed || g.record.ClientSID != client {
			continue
		}
		g.mu.Lock()
		err := g.ready()
		if err == nil && (!g.transferred || !g.readOnly) {
			err = ErrGenerationHistoryRequiresReadOnly
		}
		if err == nil {
			if _, ok := g.backend.(generationHistoryBackend); !ok {
				err = ErrNativeRequired
			}
		}
		if err == nil {
			err = g.validateContents(ctx)
			// Cancellation before reservation does not establish a custody failure.
			if err != nil && (ctx.Err() == nil || !errors.Is(err, ctx.Err())) {
				g.failure = errors.Join(g.failure, err)
			}
		}
		g.mu.Unlock()
		if err != nil {
			return nil, err
		}
		owners = append(owners, g)
	}
	var successor *InstalledGeneration
	var additions []generationSiblingAppend
	mutation := false
	defer func() {
		for _, addition := range additions {
			resultErr = errors.Join(resultErr, addition.close())
		}
		if resultErr == nil {
			return
		}
		result = nil
		if !mutation {
			return
		}
		resultErr = errors.Join(ErrInstalledGenerationUncertain, resultErr)
		for _, g := range owners {
			g.mu.Lock()
			g.failure = errors.Join(g.failure, resultErr)
			g.mu.Unlock()
		}
		if successor != nil {
			successor.mu.Lock()
			successor.failure = errors.Join(successor.failure, resultErr)
			resultErr = errors.Join(resultErr, successor.closeLocked())
			successor.mu.Unlock()
		}
	}()
	var err error
	successor, err = create(func() { mutation = true })
	if err != nil {
		return nil, err
	}
	successor.mu.Lock()
	r, err := successor.transitionReadOnlyLocked(ctx)
	successor.mu.Unlock()
	if err != nil {
		return nil, err
	}
	// The new private writers are now closed, so independent readonly history
	// handles can be acquired without sharing violations or adopting a writer.
	for _, g := range owners {
		g.mu.Lock()
		addition, err := g.backend.(generationHistoryBackend).prepareSibling(ctx, successor.backend)
		g.mu.Unlock()
		if addition != nil {
			additions = append(additions, addition)
		}
		if err != nil {
			return nil, err
		}
		if addition == nil {
			return nil, errors.New("exact prepared native sibling append required")
		}
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	// All preparations succeeded before any recipient map changes. Installation
	// has no IO/failure point; Store.mu hides the batch until final validation.
	for _, addition := range additions {
		addition.install()
	}
	for _, g := range append(owners, successor) {
		g.mu.Lock()
		err := g.validateContents(ctx)
		g.mu.Unlock()
		if err != nil {
			return nil, err
		}
	}
	return r, nil
}
