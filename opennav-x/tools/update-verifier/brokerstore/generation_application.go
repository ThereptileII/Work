package brokerstore

import (
	"context"
	"errors"
	"io"
	"runtime"
)

const generationApplicationMember = "app/opencpn.exe"
const generationQualificationMember = "app/update-qualification.json"
const maxGenerationQualificationBytes = 4096

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

// QualificationBytes returns copied original capability evidence from the fixed
// retained generation sidecar. It cannot qualify a caller-supplied path or bytes,
// and grants no health, commissioning, selected-chain or spawn authority.
// Historical producers lacking this original member deliberately refuse.
func (image *GenerationApplicationImage) QualificationBytes(ctx context.Context) (result []byte, resultErr error) {
	before, err := image.Identity(ctx)
	if err != nil {
		return nil, err
	}
	member, err := image.owner.OpenMember(generationQualificationMember)
	if err != nil {
		return nil, err
	}
	defer func() {
		resultErr = errors.Join(resultErr, member.Close())
		if resultErr != nil {
			result = nil
		}
	}()
	identity, err := member.Identity()
	if err != nil {
		return nil, err
	}
	if identity.RelativePath != generationQualificationMember || identity.Length <= 0 || identity.Length > maxGenerationQualificationBytes {
		return nil, errors.New("bounded original qualification sidecar required")
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	original := make([]byte, int(identity.Length)+1)
	n, readErr := member.ReadAt(original, 0)
	if readErr != nil && !errors.Is(readErr, io.EOF) {
		return nil, readErr
	}
	if int64(n) != identity.Length || registrationDigest(original[:n]) != identity.SHA256 {
		return nil, errors.New("qualification sidecar original length/digest changed")
	}
	afterMember, err := member.Identity()
	if err != nil || afterMember != identity {
		return nil, errors.Join(errors.New("qualification sidecar original identity changed"), err)
	}
	after, err := image.Identity(ctx)
	if err != nil || after != before {
		return nil, errors.Join(errors.New("qualification application original identity changed"), err)
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	return original[:n], nil
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
