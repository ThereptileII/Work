package brokerstore

import (
	"context"
	"errors"
	"runtime"
)

const generationApplicationMember = "app/opencpn.exe"

type generationApplicationBackend interface {
	matchApplicationProcess(context.Context, uint32, uint64) error
}

// GenerationApplicationIdentity is informational original-byte provenance. Only
// its live GenerationApplicationImage can corroborate an actual native process.
// It grants no selected-chain, spawn, commissioning or startup-health authority.
type GenerationApplicationIdentity struct{ ClientSID, Generation, Commit, ExecutableSHA256 string }

// GenerationApplicationImage can only be derived from the fixed original app
// member of a live readonly generation. It exposes no path/native/write handle.
// The same Store/generation lifetime owns and revokes it, including after reopen.
type GenerationApplicationImage struct{ owner *ReadOnlyGeneration }

func (r *ReadOnlyGeneration) ApplicationImage(ctx context.Context) (*GenerationApplicationImage, error) {
	image := &GenerationApplicationImage{owner: r}
	if _, err := image.Identity(ctx); err != nil {
		return nil, err
	}
	return image, nil
}
func (r *ReadOnlyApprovedBaselineGeneration) ApplicationImage(ctx context.Context) (*GenerationApplicationImage, error) {
	if r == nil {
		return nil, ErrClosed
	}
	return r.owner.ApplicationImage(ctx)
}
func (image *GenerationApplicationImage) Identity(ctx context.Context) (GenerationApplicationIdentity, error) {
	var empty GenerationApplicationIdentity
	if image == nil || image.owner == nil || image.owner.generation == nil || ctx == nil {
		return empty, ErrClosed
	}
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	g := image.owner.generation
	g.store.mu.Lock()
	defer g.store.mu.Unlock()
	g.mu.Lock()
	defer g.mu.Unlock()
	if err := image.owner.ready(); err != nil {
		return empty, err
	}
	ctx = registrationContext{Context: ctx, store: g.store}
	if err := ctx.Err(); err != nil {
		return empty, err
	}
	if _, ok := g.backend.(generationApplicationBackend); !ok {
		return empty, ErrNativeRequired
	}
	m, ok := g.record.Members[generationApplicationMember]
	if !ok || m.Length <= 0 || g.files[generationApplicationMember] == nil || m.SHA256 != g.record.Identity.ExecutableSHA256 {
		return empty, errors.New("fixed original application member required")
	}
	if err := g.validateContents(ctx); err != nil {
		g.failure = errors.Join(g.failure, err)
		return empty, err
	}
	return GenerationApplicationIdentity{g.record.ClientSID, g.record.Generation, g.record.Identity.Commit, m.SHA256}, nil
}

// MatchesProcess requires native-derived PID/creation evidence from a retained
// live process owner. These numbers alone never mint this image capability.
func (image *GenerationApplicationImage) MatchesProcess(ctx context.Context, pid uint32, created uint64) error {
	if image == nil || image.owner == nil || image.owner.generation == nil || ctx == nil || pid == 0 || created == 0 {
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
	b, ok := g.backend.(generationApplicationBackend)
	if !ok {
		return ErrNativeRequired
	}
	if err := g.validateContents(ctx); err != nil {
		g.failure = errors.Join(g.failure, err)
		return err
	}
	if err := b.matchApplicationProcess(ctx, pid, created); err != nil {
		return err
	}
	if err := g.validateContents(ctx); err != nil {
		g.failure = errors.Join(g.failure, err)
		return err
	}
	return ctx.Err()
}
func (*GenerationApplicationImage) MarshalJSON() ([]byte, error) { return nil, ErrNativeRequired }
func (*GenerationApplicationImage) UnmarshalJSON([]byte) error   { return ErrNativeRequired }
