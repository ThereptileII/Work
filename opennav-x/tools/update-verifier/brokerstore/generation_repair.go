package brokerstore

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"io"
	"runtime"
	"sort"
	"sync"

	"example.com/opennav-update-verifier/baselineevidence"
	"example.com/opennav-update-verifier/brokerregistration"
)

const GenerationPurposeRepaired GenerationPurpose = "repaired-generation"

// RepairOrigin identifies exact historical source bytes, never new health or a
// new signed Upgrade. Source is the original nonrepair completed generation.
type RepairOrigin struct {
	SourceCompletion           []byte `json:"sourceCompletion"`
	SourceCompletionSHA256     string `json:"sourceCompletionSha256"`
	PreviousGeneration         string `json:"previousGeneration"`
	PreviousCompletionSHA256   string `json:"previousCompletionSha256"`
	PreviousSelectedHeadSHA256 string `json:"previousSelectedHeadSha256"`
}
type RepairSourceSnapshot struct {
	ClientSID, Generation, CompletionSHA256, SourceGeneration, SourceCompletionSHA256 string
	Identity                                                                          baselineevidence.Identity
}
type repairSourceBackend interface {
	check() error
	files() map[string]candidateFile
	create(context.Context, string) (generationBackend, error)
	close() error
}
type repairStoreBackend interface {
	openRepairSource(context.Context, string, string) (repairSourceBackend, generationRecord, []byte, generationRecord, []byte, error)
}

// RepairSource can only be produced from retained protected creation metadata
// and independently opened ORIGINAL private source IDs/hashes. Damaged public
// payload is never source data, adopted ownership or permission to overwrite.
type RepairSource struct {
	mu                      sync.Mutex
	store                   *Store
	backend                 repairSourceBackend
	current, original       generationRecord
	currentRaw, originalRaw []byte
	attempted, closed       bool
	closeErr                error
}

func (s *Store) OpenRepairSource(ctx context.Context, client, generation string) (result *RepairSource, resultErr error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	s.mu.Lock()
	defer s.mu.Unlock()
	if ctx == nil || s.closed.Load() || !brokerregistration.IsCanonicalUserSID(client) || !validHex(generation, 32) {
		return nil, ErrClosed
	}
	ctx = registrationContext{Context: ctx, store: s}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	live := 0
	kept := s.repairs[:0]
	for _, r := range s.repairs {
		r.mu.Lock()
		closed, err := r.closed, r.closeErr
		r.mu.Unlock()
		if err != nil {
			return nil, errors.Join(ErrPoisoned, err)
		}
		if !closed {
			live++
			kept = append(kept, r)
		}
	}
	s.repairs = kept
	if live >= MaxCandidates {
		return nil, ErrInstalledGenerationUncertain
	}
	b, ok := s.backend.(repairStoreBackend)
	if !ok {
		return nil, ErrNativeRequired
	}
	native, current, currentRaw, original, originalRaw, err := b.openRepairSource(ctx, client, generation)
	owner := &RepairSource{store: s, backend: native, current: current, original: original, currentRaw: bytes.Clone(currentRaw), originalRaw: bytes.Clone(originalRaw)}
	if err != nil || native == nil {
		if native != nil {
			err = errors.Join(err, native.close())
		}
		return nil, errors.Join(ErrInstalledGenerationUncertain, err)
	}
	if current.ClientSID != client || current.Generation != generation {
		return nil, errors.Join(ErrInstalledGenerationUncertain, native.close())
	}
	if err = owner.validateLocked(ctx); err != nil {
		return nil, errors.Join(err, native.close())
	}
	s.repairs = append(s.repairs, owner)
	return owner, nil
}
func (r *RepairSource) validateLocked(ctx context.Context) error {
	if r == nil || r.closed || r.store.closed.Load() || r.backend == nil || ctx == nil {
		return ErrClosed
	}
	if err := errors.Join(ctx.Err(), r.store.backend.check(), r.backend.check()); err != nil {
		return err
	}
	if r.current.ClientSID != r.original.ClientSID || r.current.Identity != r.original.Identity || r.original.Repair != nil ||
		(r.original.Purpose != string(GenerationPurposeSignedCandidate) && r.original.Purpose != approvedBaselineGenerationPurpose) ||
		len(r.currentRaw) == 0 || len(r.originalRaw) == 0 || len(r.backend.files()) != len(r.original.Members) {
		return ErrInstalledGenerationUncertain
	}
	a, e := json.Marshal(r.current)
	b, f := json.Marshal(r.original)
	if e != nil || f != nil || !bytes.Equal(a, r.currentRaw) || !bytes.Equal(b, r.originalRaw) || validateGenerationPurpose(r.current) != nil || validateGenerationPurpose(r.original) != nil {
		return ErrInstalledGenerationUncertain
	}
	if r.current.Repair == nil {
		if !bytes.Equal(r.currentRaw, r.originalRaw) {
			return ErrInstalledGenerationUncertain
		}
	} else if !bytes.Equal(r.current.Repair.SourceCompletion, r.originalRaw) {
		return ErrInstalledGenerationUncertain
	}
	for name, m := range r.original.Members {
		if r.backend.files()[name] == nil {
			return ErrInstalledGenerationUncertain
		}
		if err := verifyFile(ctx, r.store, r.backend.files()[name], m.Length, m.SHA256); err != nil {
			return err
		}
	}
	// Validate the exact source sidecars against the complete restored payload set.
	return ctx.Err()
}
func (r *RepairSource) Validate(ctx context.Context) error {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if r == nil || ctx == nil {
		return ErrClosed
	}
	r.store.mu.Lock()
	defer r.store.mu.Unlock()
	r.mu.Lock()
	defer r.mu.Unlock()
	return r.validateLocked(registrationContext{Context: ctx, store: r.store})
}
func (r *RepairSource) Snapshot(ctx context.Context) (RepairSourceSnapshot, error) {
	if err := r.Validate(ctx); err != nil {
		return RepairSourceSnapshot{}, err
	}
	return RepairSourceSnapshot{r.current.ClientSID, r.current.Generation, registrationDigest(r.currentRaw), r.original.Generation, registrationDigest(r.originalRaw), r.original.Identity}, nil
}
func (r *RepairSource) Close() error {
	if r == nil {
		return nil
	}
	r.mu.Lock()
	defer r.mu.Unlock()
	if r.closed {
		return r.closeErr
	}
	r.closed = true
	if r.backend != nil {
		r.closeErr = r.backend.close()
	}
	return r.closeErr
}
func (*RepairSource) MarshalJSON() ([]byte, error) { return nil, ErrNativeRequired }
func (*RepairSource) UnmarshalJSON([]byte) error   { return ErrNativeRequired }

// MaterializeRepairFresh creates a new copy only. The old public tree, unknown
// additions, current profile, registration and trust state are never written.
// SelectedHead is comparison-only supplied by the broker's retained journals.
func (s *Store) MaterializeRepairFresh(ctx context.Context, source *RepairSource, generation, selectedHead string) (result *InstalledGeneration, resultErr error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	s.mu.Lock()
	defer s.mu.Unlock()
	if source == nil || source.store != s || ctx == nil || !validHex(generation, 32) || !validHex(selectedHead, 64) {
		return nil, ErrNativeRequired
	}
	source.mu.Lock()
	defer source.mu.Unlock()
	ctx = registrationContext{Context: ctx, store: s}
	if err := source.validateLocked(ctx); err != nil {
		return nil, err
	}
	if source.attempted || generation == source.current.Generation || generation == source.original.Generation {
		return nil, ErrInstalledGenerationUncertain
	}
	source.attempted = true
	backend, err := source.backend.create(ctx, generation)
	if backend == nil && err == nil {
		err = ErrNativeRequired
	}
	if err != nil {
		return nil, err
	}
	original := source.original
	origin := &RepairOrigin{bytes.Clone(source.originalRaw), registrationDigest(source.originalRaw), source.current.Generation, registrationDigest(source.currentRaw), selectedHead}
	g := &InstalledGeneration{store: s, backend: backend, files: map[string]candidateFile{}, record: generationRecord{Schema: 1, Purpose: string(GenerationPurposeRepaired), Repair: origin, ClientSID: original.ClientSID, Generation: generation,
		PreviousCompletion: bytes.Clone(original.PreviousCompletion), PreviousCompletionSHA256: original.PreviousCompletionSHA256, Acquisition: original.Acquisition, Identity: original.Identity,
		SourceDirectories: original.SourceDirectories, SourceMembers: original.SourceMembers, Members: map[string]baselineMember{}}}
	defer func() {
		if resultErr != nil {
			g.failure = errors.Join(g.failure, resultErr)
			resultErr = errors.Join(resultErr, g.closeLocked())
		}
	}()
	raw, err := json.Marshal(g.record)
	if err != nil || len(raw) > maxGenerationRecordBytes {
		return nil, ErrInstalledGenerationUncertain
	}
	if err = backend.reserve(ctx, raw); err != nil {
		return nil, err
	}
	if err = backend.createTree(); err != nil {
		return nil, err
	}
	names := make([]string, 0, len(original.Members))
	for name := range original.Members {
		names = append(names, name)
	}
	sort.Strings(names)
	copier := &Candidate{store: s, backend: backend}
	for _, name := range names {
		if err = ctx.Err(); err != nil {
			return nil, err
		}
		m := original.Members[name]
		f := source.backend.files()[name]
		if _, err = f.Seek(0, io.SeekStart); err != nil {
			return nil, err
		}
		target, e := backend.create(name)
		if e != nil {
			return nil, e
		}
		g.files[name] = target
		if err = copier.copyFile(ctx, target, f, m.Length, m.SHA256); err != nil {
			return nil, err
		}
		g.record.Members[name] = baselineMember{Length: m.Length, SHA256: m.SHA256}
	}
	dirs, members, err := backend.identities(g.record.Members)
	if err != nil || len(members) != len(g.files) {
		return nil, errors.Join(ErrInstalledGenerationUncertain, err)
	}
	g.record.Directories = dirs
	for name, id := range members {
		m := g.record.Members[name]
		m.Identity = id
		g.record.Members[name] = m
	}
	if err = g.validateContents(ctx); err != nil {
		return nil, err
	}
	if err = source.validateLocked(ctx); err != nil {
		return nil, err
	}
	g.raw, err = json.Marshal(g.record)
	if err != nil || len(g.raw) > maxGenerationRecordBytes {
		return nil, ErrInstalledGenerationUncertain
	}
	if err = backend.publish(ctx, g.raw); err != nil {
		return nil, err
	}
	g.complete = true
	s.generations = append(s.generations, g)
	return g, nil
}
func validateRepairOrigin(record generationRecord) error {
	p := record.Repair
	if p == nil || record.ApprovedBaseline != nil || len(p.SourceCompletion) == 0 || len(p.SourceCompletion) > maxGenerationRecordBytes || registrationDigest(p.SourceCompletion) != p.SourceCompletionSHA256 ||
		!validHex(p.PreviousGeneration, 32) || p.PreviousGeneration == record.Generation || !validHex(p.PreviousCompletionSHA256, 64) || !validHex(p.PreviousSelectedHeadSHA256, 64) {
		return ErrInstalledGenerationUncertain
	}
	var original generationRecord
	if json.Unmarshal(p.SourceCompletion, &original) != nil || original.Repair != nil {
		return ErrInstalledGenerationUncertain
	}
	raw, err := json.Marshal(original)
	if err != nil || !bytes.Equal(raw, p.SourceCompletion) || validateGenerationPurpose(original) != nil || original.ClientSID != record.ClientSID || original.Identity != record.Identity || original.Generation == record.Generation ||
		original.Acquisition != record.Acquisition || !bytes.Equal(original.PreviousCompletion, record.PreviousCompletion) || original.PreviousCompletionSHA256 != record.PreviousCompletionSHA256 ||
		!sameGenerationIDs(original.SourceDirectories, record.SourceDirectories) || !sameGenerationIDs(original.SourceMembers, record.SourceMembers) {
		return ErrInstalledGenerationUncertain
	}
	// At reservation the destination set is empty; completed validation below
	// binds every original payload member while allowing new physical IDs only.
	if len(record.Members) != 0 {
		if len(record.Members) != len(original.Members) {
			return ErrInstalledGenerationUncertain
		}
		for name, m := range original.Members {
			n, ok := record.Members[name]
			if !ok || n.Length != m.Length || n.SHA256 != m.SHA256 {
				return ErrInstalledGenerationUncertain
			}
		}
	}
	return nil
}
func (r *ReopenedGeneration) Repaired() (*ReadOnlyGeneration, error) {
	if err := r.checkPurpose(GenerationPurposeRepaired); err != nil {
		return nil, err
	}
	return r.owner, nil
}
func (r *ReopenedGeneration) RepairOrigin(ctx context.Context) (RepairOrigin, error) {
	if err := r.Validate(); err != nil {
		return RepairOrigin{}, err
	}
	if err := ctx.Err(); err != nil {
		return RepairOrigin{}, err
	}
	if r.owner.generation.record.Repair == nil {
		return RepairOrigin{}, ErrNativeRequired
	}
	p := *r.owner.generation.record.Repair
	p.SourceCompletion = bytes.Clone(p.SourceCompletion)
	return p, nil
}
func (s *Store) BeginRepairJournal(ctx context.Context, generation *ReadOnlyGeneration, id string) (*ExecutionJournal, error) {
	return s.beginExecutionJournal(ctx, generation, id, string(GenerationPurposeRepaired))
}

// OriginalBaselineExecutorImage admits only the original intact bootstrap
// launcher. Damaged unrelated B0 payload is never treated as executable code.
// The caller must independently resolve complete selected history before relay;
// this image capability supplies no selection, app launch or qualification.
func (r *RepairSource) OriginalBaselineExecutorImage(ctx context.Context) (*GenerationExecutorImage, error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if ctx == nil || r == nil {
		return nil, ErrClosed
	}
	r.store.mu.Lock()
	defer r.store.mu.Unlock()
	r.mu.Lock()
	defer r.mu.Unlock()
	ctx = registrationContext{Context: ctx, store: r.store}
	if err := r.validateLocked(ctx); err != nil {
		return nil, err
	}
	if r.current.Purpose != approvedBaselineGenerationPurpose || r.current.Generation != r.original.Generation {
		return nil, ErrNativeRequired
	}
	b, ok := r.backend.(interface {
		prepareBaselineExecutor(context.Context, *Store, baselineMember, map[string]diskIdentity) error
	})
	if !ok {
		return nil, ErrNativeRequired
	}
	if err := b.prepareBaselineExecutor(ctx, r.store, r.original.Members[generationExecutorMember], r.original.Directories); err != nil {
		return nil, err
	}
	return &GenerationExecutorImage{repairBootstrap: r}, nil
}
func (r *RepairSource) matchBaselineExecutor(ctx context.Context, pid uint32, created uint64) error {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if r == nil || ctx == nil || pid == 0 || created == 0 {
		return ErrClosed
	}
	r.store.mu.Lock()
	defer r.store.mu.Unlock()
	r.mu.Lock()
	defer r.mu.Unlock()
	ctx = registrationContext{Context: ctx, store: r.store}
	if err := r.validateLocked(ctx); err != nil {
		return err
	}
	b, ok := r.backend.(interface {
		matchBaselineExecutor(context.Context, *Store, uint32, uint64) error
	})
	if !ok {
		return ErrNativeRequired
	}
	if err := b.matchBaselineExecutor(ctx, r.store, pid, created); err != nil {
		return err
	}
	return r.validateLocked(ctx)
}

// ValidateHistory closes the crash boundary between completed copying and its
// first journal append. Comparison values must come from original authenticated
// journal owners. An orphan completed repair is preserved and blocks another
// attempt, rather than being silently treated as an unused staged Upgrade.
func (r *RepairSource) ValidateHistory(ctx context.Context, completed map[string]string) error {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if err := r.Validate(ctx); err != nil {
		return err
	}
	r.store.mu.Lock()
	defer r.store.mu.Unlock()
	r.mu.Lock()
	defer r.mu.Unlock()
	b, ok := r.backend.(interface {
		validateRepairHistory(context.Context, map[string]string) error
	})
	if !ok {
		return ErrNativeRequired
	}
	return b.validateRepairHistory(ctx, completed)
}
func (r *ReopenedGeneration) ValidateRepairHistory(ctx context.Context, completed map[string]string) error {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if ctx == nil {
		return ErrClosed
	}
	if err := r.Validate(); err != nil {
		return err
	}
	g := r.owner.generation
	g.store.mu.Lock()
	defer g.store.mu.Unlock()
	g.mu.Lock()
	defer g.mu.Unlock()
	b, ok := g.backend.(interface {
		validateRepairHistory(context.Context, map[string]string) error
	})
	if !ok {
		return ErrNativeRequired
	}
	return b.validateRepairHistory(ctx, completed)
}
func validateRepairHistoryRecords(records [][]byte, completed map[string]string) error {
	if len(records) > MaxCandidates+1 || len(completed) > 128 {
		return ErrInstalledGenerationUncertain
	}
	seen := map[string]bool{}
	for _, raw := range records {
		var record generationRecord
		if json.Unmarshal(raw, &record) != nil {
			return ErrInstalledGenerationUncertain
		}
		if record.Purpose != string(GenerationPurposeRepaired) {
			continue
		}
		canonical, err := json.Marshal(record)
		if err != nil || !bytes.Equal(canonical, raw) || validateRepairOrigin(record) != nil || seen[record.Generation] || completed[record.Generation] != registrationDigest(raw) {
			return ErrInstalledGenerationUncertain
		}
		seen[record.Generation] = true
	}
	return nil
}
