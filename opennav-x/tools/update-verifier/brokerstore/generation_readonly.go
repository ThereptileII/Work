package brokerstore

import (
	"context"
	"errors"
	"io"
	"runtime"
)

var ErrGenerationTransferred = errors.New("generation writer permanently transferred to readonly custody")
var ErrGenerationReadOnly = errors.New("readonly generation member")

const maxGenerationMemberRead = 4 << 20

type generationReadOnlyBackend interface {
	// Replace files incrementally, removing each writer before its single Close
	// attempt. Even failure must leave every still-owned file in this map so the
	// common lifetime owner drains readers and remaining writers exactly once.
	transitionReadOnly(context.Context, map[string]candidateFile, map[string]baselineMember) error
}

type generationReadOnlyFile struct {
	candidateFile
	reader io.ReaderAt
}

func (*generationReadOnlyFile) Write([]byte) (int, error) { return 0, ErrGenerationReadOnly }
func (f *generationReadOnlyFile) ReadAt(p []byte, offset int64) (int, error) {
	return f.reader.ReadAt(p, offset)
}

// ReadOnlyGeneration is a live capability to retained, authenticated original
// bytes. It grants no selection, launch, installation or healthy-start authority.
// Its Store and original InstalledGeneration remain lifetime/revocation owners:
// closing any of them revokes this capability and all member views. It cannot be
// reconstructed from Completion, a path, a receipt or current permissions.
type ReadOnlyGeneration struct {
	generation *InstalledGeneration
	children   map[*GenerationReadHandle]bool
}

// GenerationMemberIdentity is informational. Only a live GenerationReadHandle
// permits reading its member; serializing this identity grants no authority.
type GenerationMemberIdentity struct {
	RelativePath string
	Length       int64
	SHA256       string
}

// GenerationReadHandle is an owned, revocable readonly view of one exact member.
// It exposes neither a writable/native handle nor a caller-selected disk path.
// Close revokes this view; physical custody stays with ReadOnlyGeneration until
// that owner (or its Store/original generation) closes.
type GenerationReadHandle struct {
	owner    *ReadOnlyGeneration
	relative string
	closed   bool
}

// TransitionReadOnly consumes a completed writer exactly once. It actually
// closes native writer file objects and reopens readonly files; reduced-access
// duplicates are not assumed to change Windows sharing or loader constraints.
// The intermediate reopen is safe only because this retained namespace and its
// exact descriptors were protected at birth, with untrusted clients never given
// write/delete rights. Directory/full-ID/history custody remains held throughout.
// Failure poisons and drains the owner with retained uncertainty, never permits
// retry/adoption, and never deletes durable completed or partial bytes. Cancellation
// before success is refused; it cannot undo the existing completed generation.
func (g *InstalledGeneration) TransitionReadOnly(ctx context.Context) (result *ReadOnlyGeneration, resultErr error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	g.store.mu.Lock()
	defer g.store.mu.Unlock()
	g.mu.Lock()
	defer g.mu.Unlock()
	if err := g.ready(); err != nil {
		return nil, err
	}
	if g.transferred {
		return nil, ErrGenerationTransferred
	}
	g.transferred = true
	defer func() {
		if resultErr != nil {
			g.failure = errors.Join(g.failure, resultErr)
			resultErr = errors.Join(ErrInstalledGenerationUncertain, resultErr, g.closeLocked())
		}
	}()
	if ctx == nil {
		return nil, errors.New("live readonly transition context required")
	}
	ctx = registrationContext{Context: ctx, store: g.store}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if err := g.validateContents(ctx); err != nil {
		return nil, err
	}
	backend, ok := g.backend.(generationReadOnlyBackend)
	if !ok {
		return nil, ErrNativeRequired
	}
	if err := backend.transitionReadOnly(ctx, g.files, g.record.Members); err != nil {
		return nil, err
	}
	for name := range g.record.Members {
		f, ok := g.files[name].(*generationReadOnlyFile)
		if !ok || f == nil || f.reader == nil {
			return nil, errors.New("actual readonly retained member required")
		}
	}
	if err := g.validateContents(ctx); err != nil {
		return nil, err
	}
	g.readOnly = true
	return &ReadOnlyGeneration{generation: g, children: map[*GenerationReadHandle]bool{}}, nil
}
func (r *ReadOnlyGeneration) ready() error {
	if r == nil || r.generation == nil {
		return ErrClosed
	}
	g := r.generation
	if err := g.ready(); err != nil {
		return err
	}
	if !g.transferred || !g.readOnly {
		return ErrGenerationReadOnly
	}
	return nil
}
func (r *ReadOnlyGeneration) Validate() error {
	if r == nil || r.generation == nil {
		return ErrClosed
	}
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	g := r.generation
	g.mu.Lock()
	defer g.mu.Unlock()
	if err := r.ready(); err != nil {
		return err
	}
	if err := g.validateContents(registrationContext{Context: context.Background(), store: g.store}); err != nil {
		g.failure = errors.Join(g.failure, err)
		return errors.Join(ErrInstalledGenerationUncertain, err)
	}
	return nil
}
func (r *ReadOnlyGeneration) Completion() (GenerationCompletion, error) {
	if r == nil || r.generation == nil {
		return GenerationCompletion{}, ErrClosed
	}
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	g := r.generation
	g.mu.Lock()
	defer g.mu.Unlock()
	if err := r.ready(); err != nil {
		return GenerationCompletion{}, err
	}
	if err := g.validateContents(registrationContext{Context: context.Background(), store: g.store}); err != nil {
		g.failure = errors.Join(g.failure, err)
		return GenerationCompletion{}, errors.Join(ErrInstalledGenerationUncertain, err)
	}
	return g.completionRecord()
}
func (r *ReadOnlyGeneration) OpenMember(relative string) (*GenerationReadHandle, error) {
	if r == nil || r.generation == nil {
		return nil, ErrClosed
	}
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	g := r.generation
	g.mu.Lock()
	defer g.mu.Unlock()
	if err := r.ready(); err != nil {
		return nil, err
	}
	m, ok := g.record.Members[relative]
	if !ok || len(r.children) >= MaxFiles {
		return nil, errors.New("exact retained relative member and bounded live views required")
	}
	if err := g.backend.check(); err != nil {
		g.failure = errors.Join(g.failure, err)
		return nil, err
	}
	if err := g.files[relative].check(m.Length); err != nil {
		g.failure = errors.Join(g.failure, err)
		return nil, err
	}
	h := &GenerationReadHandle{owner: r, relative: relative}
	r.children[h] = true
	return h, nil
}
func (h *GenerationReadHandle) Identity() (GenerationMemberIdentity, error) {
	if h == nil || h.owner == nil || h.owner.generation == nil {
		return GenerationMemberIdentity{}, ErrClosed
	}
	g := h.owner.generation
	g.mu.Lock()
	defer g.mu.Unlock()
	if h.closed {
		return GenerationMemberIdentity{}, ErrClosed
	}
	if err := h.owner.ready(); err != nil {
		return GenerationMemberIdentity{}, err
	}
	m := g.record.Members[h.relative]
	return GenerationMemberIdentity{h.relative, m.Length, m.SHA256}, nil
}
func (h *GenerationReadHandle) ReadAt(p []byte, offset int64) (int, error) {
	if h == nil || h.owner == nil || h.owner.generation == nil {
		return 0, ErrClosed
	}
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	g := h.owner.generation
	g.mu.Lock()
	defer g.mu.Unlock()
	if h.closed {
		return 0, ErrClosed
	}
	if err := h.owner.ready(); err != nil {
		return 0, err
	}
	if offset < 0 || len(p) > maxGenerationMemberRead {
		return 0, errors.New("bounded member read required")
	}
	m := g.record.Members[h.relative]
	if len(p) == 0 {
		return 0, nil
	}
	if offset >= m.Length {
		return 0, io.EOF
	}
	if err := g.backend.check(); err != nil {
		g.failure = errors.Join(g.failure, err)
		return 0, err
	}
	f := g.files[h.relative].(*generationReadOnlyFile)
	if err := f.check(m.Length); err != nil {
		g.failure = errors.Join(g.failure, err)
		return 0, err
	}
	n, err := f.ReadAt(p, offset)
	if checkErr := f.check(m.Length); checkErr != nil {
		g.failure = errors.Join(g.failure, checkErr)
		return n, errors.Join(err, checkErr)
	}
	return n, err
}
func (h *GenerationReadHandle) Close() error {
	if h == nil || h.owner == nil || h.owner.generation == nil {
		return nil
	}
	g := h.owner.generation
	g.mu.Lock()
	defer g.mu.Unlock()
	if !h.closed {
		h.closed = true
		delete(h.owner.children, h)
	}
	return nil
}
func (r *ReadOnlyGeneration) Close() error {
	if r == nil || r.generation == nil {
		return nil
	}
	return r.generation.Close()
}
