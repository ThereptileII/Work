package brokerstore

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"runtime"
	"sync"

	"example.com/opennav-update-verifier/brokerregistration"
)

const (
	maxExecutionJournals       = 128
	maxExecutionFacts          = 64
	maxExecutionFactBytes      = 64 << 10
	maxExecutionFenceBytes     = 24 << 20
	maxExecutionJournalBytes   = 32 << 20
	maxExecutionNamespaceFiles = 4096
)

var ErrExecutionJournalUncertain = errors.New("execution journal custody or durable append uncertain")

// ExecutionFactKind labels observations retained by the actual broker host.
// Neither a kind nor its original bytes authenticates the observation. In
// particular HealthObserved is not a healthy commit or selection permission.
type ExecutionFactKind string

const (
	ExecutionPreparation      ExecutionFactKind = "preparation"
	ExecutionCommissioning    ExecutionFactKind = "commissioning"
	ExecutionSpawnIntent      ExecutionFactKind = "spawn-intent"
	ExecutionChildCreated     ExecutionFactKind = "child-created"
	ExecutionHealthObserved   ExecutionFactKind = "health-observed"
	ExecutionTeardown         ExecutionFactKind = "teardown"
	ExecutionRestoration      ExecutionFactKind = "restoration"
	ExecutionOperationFailure ExecutionFactKind = "operation-failure"
)

func (k ExecutionFactKind) valid() bool {
	switch k {
	case ExecutionPreparation, ExecutionCommissioning, ExecutionSpawnIntent, ExecutionChildCreated,
		ExecutionHealthObserved, ExecutionTeardown, ExecutionRestoration, ExecutionOperationFailure:
		return true
	}
	return false
}

type executionJournalIDs struct {
	Parents map[string]diskIdentity `json:"parents"`
	Fence   diskIdentity            `json:"fence"`
	Journal diskIdentity            `json:"journal"`
}

func (i executionJournalIDs) valid() bool {
	if len(i.Parents) == 0 || len(i.Parents) > 128 || !i.Fence.valid() || !i.Journal.valid() || i.Fence == i.Journal {
		return false
	}
	for name, id := range i.Parents {
		if name == "" || len(name) > 32768 || !id.valid() || id.Volume != i.Fence.Volume || id.Volume != i.Journal.Volume {
			return false
		}
	}
	return true
}

func sameExecutionJournalIDs(a, b executionJournalIDs) bool {
	if a.Fence != b.Fence || a.Journal != b.Journal || len(a.Parents) != len(b.Parents) {
		return false
	}
	for name, id := range a.Parents {
		if b.Parents[name] != id {
			return false
		}
	}
	return true
}

type executionJournalOrigin struct {
	Schema             int                 `json:"schema"`
	Purpose            string              `json:"purpose"`
	ExecutionID        string              `json:"executionId"`
	ClientSID          string              `json:"clientSid"`
	Generation         string              `json:"generation"`
	GenerationPurpose  string              `json:"generationPurpose"`
	OriginalCompletion []byte              `json:"originalCompletion"`
	CompletionSHA256   string              `json:"completionSha256"`
	Identities         executionJournalIDs `json:"identities"`
}

type executionJournalRecord struct {
	Schema         int               `json:"schema"`
	Sequence       int               `json:"sequence"`
	PreviousSHA256 string            `json:"previousSha256"`
	Kind           ExecutionFactKind `json:"kind"`
	Original       []byte            `json:"original"`
}

// Exact re-encoding closes the schema, rejects duplicate/unknown keys and
// aliases, and preserves the original byte chain rather than trusting a parsed
// summary. This parser alone authenticates neither disk custody nor fact truth.
func validateExecutionJournalBytes(fence, raw []byte) error {
	if len(fence) == 0 || len(fence) > maxExecutionFenceBytes || len(raw) > maxExecutionJournalBytes || len(raw) < len(fence)+1 || !bytes.Equal(raw[:len(fence)], fence) || raw[len(fence)] != '\n' {
		return errors.New("invalid execution journal origin framing/bounds")
	}
	var origin executionJournalOrigin
	if err := json.Unmarshal(fence, &origin); err != nil {
		return err
	}
	canonical, err := json.Marshal(origin)
	if err != nil || !bytes.Equal(canonical, fence) || origin.Schema != 1 || origin.Purpose != "protected-execution-facts" || !validHex(origin.ExecutionID, 32) || !brokerregistration.IsCanonicalUserSID(origin.ClientSID) || !validHex(origin.Generation, 32) ||
		(origin.GenerationPurpose != "signed-candidate-generation" && origin.GenerationPurpose != approvedBaselineGenerationPurpose) || len(origin.OriginalCompletion) == 0 || len(origin.OriginalCompletion) > maxGenerationRecordBytes || origin.CompletionSHA256 != registrationDigest(origin.OriginalCompletion) || !origin.Identities.valid() {
		return errors.New("noncanonical or unbound execution journal origin")
	}
	prefix := len(fence) + 1
	for sequence := 1; prefix < len(raw); sequence++ {
		end := bytes.IndexByte(raw[prefix:], '\n')
		if end < 0 || sequence > maxExecutionFacts || end > 2*maxExecutionFactBytes {
			return errors.New("execution journal record framing/count/bounds")
		}
		line := raw[prefix : prefix+end]
		var e executionJournalRecord
		if err := json.Unmarshal(line, &e); err != nil {
			return err
		}
		canonical, err := json.Marshal(e)
		if err != nil || !bytes.Equal(canonical, line) || e.Schema != 1 || e.Sequence != sequence || e.PreviousSHA256 != registrationDigest(raw[:prefix]) || !e.Kind.valid() || len(e.Original) == 0 || len(e.Original) > maxExecutionFactBytes {
			return errors.New("noncanonical, reordered or changed execution journal chain")
		}
		prefix += end + 1
	}
	return nil
}

type executionJournalStoreBackend interface {
	prepareExecutionJournal(context.Context, string, string) (executionJournalBackend, error)
}
type executionJournalBackend interface {
	identities() (executionJournalIDs, error)
	initialize(context.Context, []byte, []byte) error
	append(context.Context, []byte, []byte, []byte) error
	validate([]byte, []byte) error
	close() error
}

// ExecutionJournal owns fresh broker-private native files and depends on a live
// retained readonly generation. It is an append-only storage primitive, not an execution
// reducer or authorization API. Facts can contain inaccurate caller assertions;
// the future actual host must independently corroborate them with live owners.
// No selected pointer is exposed or derived from PreviousGeneration. Only a
// future authenticated healthy-commit operation may establish selection.
// There is no append reopening, disk adoption, deletion or retry of a fence.
type ExecutionJournal struct {
	mu          sync.Mutex
	store       *Store
	generation  *ReadOnlyGeneration
	backend     executionJournalBackend
	origin      executionJournalOrigin
	fence, raw  []byte
	records     []executionJournalRecord
	initialized bool
	closed      bool
	failure     error
	closeErr    error
}

// ExecutionJournalSnapshot is an informational copy of original protected
// records. It cannot be converted back into a live owner or launch permission.
type ExecutionJournalSnapshot struct {
	ExecutionID      string
	ClientSID        string
	Generation       string
	CompletionSHA256 string
	JournalSHA256    string
	OriginalFence    []byte
	OriginalJournal  []byte
	FactCount        int
}

func (s *Store) BeginExecutionJournal(ctx context.Context, generation *ReadOnlyGeneration, executionID string) (*ExecutionJournal, error) {
	return s.beginExecutionJournal(ctx, generation, executionID, "signed-candidate-generation")
}

// BeginBaselineQualificationJournal retains facts about an initial baseline.
// An approved baseline is not assumed selected, commissioned or healthy.
func (s *Store) BeginBaselineQualificationJournal(ctx context.Context, generation *ReadOnlyApprovedBaselineGeneration, executionID string) (*ExecutionJournal, error) {
	if generation == nil {
		return nil, errors.New("live readonly approved baseline required")
	}
	return s.beginExecutionJournal(ctx, generation.owner, executionID, approvedBaselineGenerationPurpose)
}

func (s *Store) beginExecutionJournal(ctx context.Context, generation *ReadOnlyGeneration, executionID, purpose string) (result *ExecutionJournal, resultErr error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	s.mu.Lock()
	defer s.mu.Unlock()
	if s.closed.Load() {
		return nil, ErrClosed
	}
	if ctx == nil || !validHex(executionID, 32) || generation == nil || generation.generation == nil || generation.generation.store != s || len(s.journals) >= maxExecutionJournals {
		return nil, errors.New("bounded fresh execution identity and same-store readonly generation required")
	}
	g := generation.generation
	g.mu.Lock()
	defer g.mu.Unlock()
	ctx = registrationContext{Context: ctx, store: s}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if err := generation.ready(); err != nil {
		return nil, err
	}
	if g.record.Purpose != purpose {
		return nil, errors.New("generation provenance differs from journal operation")
	}
	if err := g.validateContents(ctx); err != nil {
		return nil, err
	}
	if len(g.raw) == 0 || len(g.raw) > maxGenerationRecordBytes {
		return nil, errors.New("bounded original generation completion required")
	}
	b, ok := s.backend.(executionJournalStoreBackend)
	if !ok {
		return nil, ErrNativeRequired
	}
	native, err := b.prepareExecutionJournal(ctx, g.record.ClientSID, executionID)
	if err != nil {
		return nil, err
	}
	if native == nil {
		return nil, errors.New("native execution journal owner absent")
	}
	j := &ExecutionJournal{store: s, generation: generation, backend: native}
	s.journals = append(s.journals, j)
	defer func() {
		if resultErr != nil {
			j.failure = errors.Join(j.failure, resultErr)
			resultErr = errors.Join(ErrExecutionJournalUncertain, resultErr, j.closeLocked())
		}
	}()
	ids, err := native.identities()
	if err != nil || !ids.valid() {
		return nil, errors.Join(errors.New("original full journal and ancestry identities required"), err)
	}
	j.origin = executionJournalOrigin{Schema: 1, Purpose: "protected-execution-facts", ExecutionID: executionID, ClientSID: g.record.ClientSID, Generation: g.record.Generation,
		GenerationPurpose: purpose, OriginalCompletion: bytes.Clone(g.raw), CompletionSHA256: registrationDigest(g.raw), Identities: ids}
	j.fence, err = json.Marshal(j.origin)
	if err != nil || len(j.fence) > maxExecutionFenceBytes {
		return nil, errors.New("execution fence exceeds fixed bound")
	}
	j.raw = append(bytes.Clone(j.fence), '\n')
	if err = native.initialize(ctx, j.fence, j.raw); err != nil {
		return nil, err
	}
	j.initialized = true
	if err = ctx.Err(); err != nil {
		return nil, err
	}
	return j, nil
}

// Caller holds Store.mu, journal.mu and generation.mu, in that order.
func (j *ExecutionJournal) ready(ctx context.Context) error {
	if j.closed || j.store.closed.Load() {
		return ErrClosed
	}
	if j.failure != nil {
		return errors.Join(ErrPoisoned, j.failure)
	}
	if ctx == nil {
		return errors.New("live execution journal context required")
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	if err := j.generation.ready(); err != nil {
		return err
	}
	g := j.generation.generation
	if err := g.validateContents(registrationContext{Context: ctx, store: j.store}); err != nil {
		return err
	}
	if !bytes.Equal(g.raw, j.origin.OriginalCompletion) {
		return errors.New("original execution generation completion changed")
	}
	return j.backend.validate(j.fence, j.raw)
}

// AppendFact durably retains exact bounded original bytes after the prior chain.
// Return success means only flush/readback and live custody succeeded. A
// spawn-intent fact is not a one-use launch grant and health-observed is not a
// healthy commit. Any error consumes this live owner and preserves its fence.
func (j *ExecutionJournal) AppendFact(ctx context.Context, kind ExecutionFactKind, original []byte) (resultErr error) {
	if j == nil || j.store == nil || j.generation == nil || j.generation.generation == nil || j.backend == nil {
		return ErrClosed
	}
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	j.store.mu.Lock()
	defer j.store.mu.Unlock()
	j.mu.Lock()
	defer j.mu.Unlock()
	j.generation.generation.mu.Lock()
	defer j.generation.generation.mu.Unlock()
	if j.closed || j.store.closed.Load() {
		return ErrClosed
	}
	defer func() {
		if resultErr != nil {
			j.failure = errors.Join(j.failure, resultErr)
			resultErr = errors.Join(ErrExecutionJournalUncertain, resultErr, j.closeLocked())
		}
	}()
	if err := j.ready(ctx); err != nil {
		return err
	}
	if !kind.valid() || len(original) == 0 || len(original) > maxExecutionFactBytes || len(j.records) >= maxExecutionFacts {
		return errors.New("invalid or exhausted execution fact bounds")
	}
	e := executionJournalRecord{Schema: 1, Sequence: len(j.records) + 1, PreviousSHA256: registrationDigest(j.raw), Kind: kind, Original: bytes.Clone(original)}
	line, err := json.Marshal(e)
	if err != nil {
		return err
	}
	next := make([]byte, 0, len(j.raw)+len(line)+1)
	next = append(next, j.raw...)
	next = append(next, line...)
	next = append(next, '\n')
	if len(next) > maxExecutionJournalBytes {
		return errors.New("execution journal fixed byte bound exhausted")
	}
	if err = j.backend.append(ctx, j.fence, j.raw, next); err != nil {
		return err
	}
	if err = ctx.Err(); err != nil {
		return err
	}
	j.raw = next
	j.records = append(j.records, e)
	return nil
}

func (j *ExecutionJournal) Validate() error {
	_, err := j.Snapshot()
	return err
}

func (j *ExecutionJournal) Snapshot() (result ExecutionJournalSnapshot, resultErr error) {
	if j == nil || j.store == nil || j.generation == nil || j.generation.generation == nil || j.backend == nil {
		return result, ErrClosed
	}
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	j.store.mu.Lock()
	defer j.store.mu.Unlock()
	j.mu.Lock()
	defer j.mu.Unlock()
	j.generation.generation.mu.Lock()
	defer j.generation.generation.mu.Unlock()
	if j.closed || j.store.closed.Load() {
		return result, ErrClosed
	}
	if err := j.ready(context.Background()); err != nil {
		j.failure = errors.Join(j.failure, err)
		return result, errors.Join(ErrExecutionJournalUncertain, err, j.closeLocked())
	}
	return ExecutionJournalSnapshot{ExecutionID: j.origin.ExecutionID, ClientSID: j.origin.ClientSID, Generation: j.origin.Generation, CompletionSHA256: j.origin.CompletionSHA256,
		JournalSHA256: registrationDigest(j.raw), OriginalFence: bytes.Clone(j.fence), OriginalJournal: bytes.Clone(j.raw), FactCount: len(j.records)}, nil
}

func (j *ExecutionJournal) Close() error {
	if j == nil || j.backend == nil {
		return nil
	}
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	j.store.mu.Lock()
	defer j.store.mu.Unlock()
	j.mu.Lock()
	defer j.mu.Unlock()
	return j.closeLocked()
}

func (j *ExecutionJournal) closeLocked() error {
	if j.closed {
		return j.closeErr
	}
	j.closed = true
	j.closeErr = j.failure
	if j.initialized {
		j.closeErr = errors.Join(j.closeErr, j.backend.validate(j.fence, j.raw))
	}
	j.closeErr = errors.Join(j.closeErr, j.backend.close())
	if j.closeErr != nil {
		j.closeErr = errors.Join(ErrExecutionJournalUncertain, j.closeErr)
	}
	return j.closeErr
}
