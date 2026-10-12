package brokerstore

import (
	"context"
	"errors"
	"runtime"
)

const generationExecutorMember = "app/skager-start.exe"

type generationExecutorBackend interface {
	matchExecutorProcess(context.Context, uint32, uint64) error
}

// GenerationExecutorImage is a revocable capability for the one fixed original
// interactive launcher in a retained read-only generation. It cannot be minted
// from a path, hash, completion receipt or process ID. Its process comparison is
// only one part of executor authentication: brokerpipe must independently retain
// and validate the actual peer process, pipe, token and logon session.
//
// This capability does not select a version, grant a spawn, commission a profile
// or establish application health. The original generation/Store owns custody.
type GenerationExecutorImage struct{ owner *ReadOnlyGeneration }

func (r *ReadOnlyGeneration) ExecutorImage(ctx context.Context) (*GenerationExecutorImage, error) {
	if r == nil || r.generation == nil || ctx == nil {
		return nil, ErrClosed
	}
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	g := r.generation
	g.store.mu.Lock()
	defer g.store.mu.Unlock()
	g.mu.Lock()
	defer g.mu.Unlock()
	if err := r.ready(); err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if _, ok := g.backend.(generationExecutorBackend); !ok {
		return nil, ErrNativeRequired
	}
	if member, ok := g.record.Members[generationExecutorMember]; !ok || member.Length <= 0 || g.files[generationExecutorMember] == nil {
		return nil, errors.New("fixed original interactive launcher required")
	}
	if err := g.validateContents(registrationContext{Context: ctx, store: g.store}); err != nil {
		g.failure = errors.Join(g.failure, err)
		return nil, err
	}
	return &GenerationExecutorImage{owner: r}, nil
}

func (r *ReadOnlyApprovedBaselineGeneration) ExecutorImage(ctx context.Context) (*GenerationExecutorImage, error) {
	if r == nil || r.owner == nil {
		return nil, ErrClosed
	}
	return r.owner.ExecutorImage(ctx)
}

// MatchesProcess compares the actual live native process image to the retained
// original launcher, including full physical file identity and original process
// creation FILETIME. Callers must derive both numbers from their retained native
// peer, never from JSON. A successful comparison grants no execution authority.
func (image *GenerationExecutorImage) MatchesProcess(ctx context.Context, processID uint32, creationFileTime uint64) error {
	if image == nil || image.owner == nil || image.owner.generation == nil || ctx == nil || processID == 0 || creationFileTime == 0 {
		return ErrClosed
	}
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	g := image.owner.generation
	g.store.mu.Lock()
	defer g.store.mu.Unlock()
	g.mu.Lock()
	defer g.mu.Unlock()
	if err := image.owner.ready(); err != nil {
		return err
	}
	ctx = registrationContext{Context: ctx, store: g.store}
	if err := ctx.Err(); err != nil {
		return err
	}
	backend, ok := g.backend.(generationExecutorBackend)
	if !ok {
		return ErrNativeRequired
	}
	if err := g.validateContents(ctx); err != nil {
		g.failure = errors.Join(g.failure, err)
		return err
	}
	// A mismatched/dead peer does not poison otherwise sound generation custody.
	if err := backend.matchExecutorProcess(ctx, processID, creationFileTime); err != nil {
		return err
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	if err := g.validateContents(ctx); err != nil {
		g.failure = errors.Join(g.failure, err)
		return err
	}
	return nil
}
