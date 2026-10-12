package brokerstore

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"io"
	"runtime"
	"strings"
	"sync"
)

const maxProfileBackupMarkerBytes = 2*maxExecutionFenceBytes + (64 << 10)

var ErrExecutionProfileBackupUncertain = errors.New("execution profile backup incomplete or custody uncertain")

type executionProfileBackupOrigin struct {
	Schema               int                 `json:"schema"`
	Purpose              string              `json:"purpose"`
	OriginalJournalFence []byte              `json:"originalJournalFence"`
	Identities           executionJournalIDs `json:"identities"`
}
type executionProfileBackupCompletion struct {
	Schema       int    `json:"schema"`
	Purpose      string `json:"purpose"`
	OriginSHA256 string `json:"originSha256"`
	Bytes        int64  `json:"bytes"`
	SHA256       string `json:"sha256"`
}

type executionProfileBackupStoreBackend interface {
	createExecutionProfileBackup(context.Context, string, string) (executionProfileBackupBackend, error)
	openExecutionProfileBackup(context.Context, string, string) (executionProfileBackupBackend, []byte, error)
}
type executionProfileBackupBackend interface {
	identities() (executionJournalIDs, error)
	initialize(context.Context, []byte) error
	write(context.Context, io.Reader, int64, string) error
	complete(context.Context, []byte, []byte, int64, string) error
	validate([]byte, int64, string) error
	readAt([]byte, int64) (int, error)
	close() error
}

// ExecutionProfileBackup is a fresh broker-private bounded opaque archive owner.
// Storage authenticates original bytes and native custody, not a full source-tree
// snapshot, coldness, metadata meaning, commissioning, selection or restoration.
// The host must independently capture and corroborate the real fixed profile.
// It cannot reopen for writes, delete, repair or retry its permanent reservation.
type ExecutionProfileBackup struct{ state *executionProfileBackupState }

// ExecutionProfileBackupReader retains only a completed fixed backup bound to a
// live same-Store journal reader. It exposes no path, raw handle or write API.
type ExecutionProfileBackupReader struct{ state *executionProfileBackupState }
type executionProfileBackupState struct {
	mu                                      sync.Mutex
	store                                   *Store
	owner                                   context.Context
	journal                                 *ExecutionJournal
	reader                                  *ExecutionJournalReader
	backend                                 executionProfileBackupBackend
	fence, marker                           []byte
	origin                                  executionJournalOrigin
	size                                    int64
	digest                                  string
	initialized, written, completed, closed bool
	failure, closeErr                       error
}

// ExecutionProfileBackupSnapshot is copied storage evidence only. An archive's
// digest does not attest what the host captured or authorize overwriting data.
type ExecutionProfileBackupSnapshot struct {
	ClientSID, ExecutionID, Generation, CompletionSHA256, JournalFenceSHA256 string
	Bytes                                                                    int64
	SHA256                                                                   string
	OriginalMarker                                                           []byte
}

// Ignore other storage purposes, but reject every alias/unknown member for this
// exact backup identity. Profile entries never acquire execution-journal meaning.
func executionProfileBackupPairNames(client, id string, names []string, fresh bool) error {
	prefix := client + ".profile." + id
	found := map[string]bool{}
	for _, name := range names {
		if len(name) < len(prefix) || !strings.EqualFold(name[:len(prefix)], prefix) {
			continue
		}
		if len(name) > len(prefix) && name[len(prefix)] != '.' {
			continue
		}
		if fresh || (name != prefix+".once" && name != prefix+".backup") || found[name] {
			return errors.New("profile backup namespace already exists, is unknown or aliased")
		}
		found[name] = true
	}
	if !fresh && (!found[prefix+".once"] || !found[prefix+".backup"]) {
		return errors.New("profile backup permanent pair incomplete")
	}
	return nil
}

func decodeExecutionProfileBackup(marker, journalFence []byte) (executionProfileBackupOrigin, executionProfileBackupCompletion, error) {
	var origin executionProfileBackupOrigin
	var completion executionProfileBackupCompletion
	lines := bytes.Split(marker, []byte{'\n'})
	if len(marker) > maxProfileBackupMarkerBytes || len(lines) != 3 || len(lines[2]) != 0 {
		return origin, completion, errors.New("profile backup completion missing or outside bound")
	}
	if err := json.Unmarshal(lines[0], &origin); err != nil {
		return origin, completion, err
	}
	raw, err := json.Marshal(origin)
	if err != nil || !bytes.Equal(raw, lines[0]) || origin.Schema != 1 || origin.Purpose != "protected-execution-profile-backup" || !bytes.Equal(origin.OriginalJournalFence, journalFence) || !origin.Identities.valid() {
		return origin, completion, errors.New("profile backup origin differs from original execution")
	}
	if err := validateExecutionJournalBytes(journalFence, append(bytes.Clone(journalFence), '\n')); err != nil {
		return origin, completion, err
	}
	if err := json.Unmarshal(lines[1], &completion); err != nil {
		return origin, completion, err
	}
	raw, err = json.Marshal(completion)
	if err != nil || !bytes.Equal(raw, lines[1]) || completion.Schema != 1 || completion.Purpose != "completed-execution-profile-backup" || completion.OriginSHA256 != registrationDigest(lines[0]) || completion.Bytes <= 0 || completion.Bytes > MaxPayloadBytes || !validHex(completion.SHA256, 64) {
		return origin, completion, errors.New("noncanonical or unbound profile backup completion")
	}
	return origin, completion, nil
}

func (s *Store) BeginExecutionProfileBackupFresh(ctx context.Context, journal *ExecutionJournal) (result *ExecutionProfileBackup, resultErr error) {
	if s == nil {
		return nil, ErrClosed
	}
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	s.mu.Lock()
	defer s.mu.Unlock()
	if s.closed.Load() {
		return nil, ErrClosed
	}
	if ctx == nil || journal == nil || journal.store != s || journal.generation == nil || journal.generation.generation == nil || len(s.profileBackups) >= maxExecutionJournals {
		return nil, errors.New("bounded live same-store journal required")
	}
	p := &executionProfileBackupState{store: s, owner: registrationContext{Context: ctx, store: s}, journal: journal}
	if err := p.journalReady(ctx); err != nil {
		return nil, err
	}
	b, ok := s.backend.(executionProfileBackupStoreBackend)
	if !ok {
		return nil, ErrNativeRequired
	}
	native, err := b.createExecutionProfileBackup(ctx, journal.origin.ClientSID, journal.origin.ExecutionID)
	if err != nil {
		if native != nil {
			err = errors.Join(err, native.close())
		}
		return nil, errors.Join(ErrExecutionProfileBackupUncertain, err)
	}
	if native == nil {
		return nil, errors.Join(ErrExecutionProfileBackupUncertain, ErrNativeRequired)
	}
	p.backend = native
	p.origin = journal.origin
	p.fence = bytes.Clone(journal.fence)
	s.profileBackups = append(s.profileBackups, p)
	defer func() {
		if resultErr != nil {
			resultErr = p.fail(resultErr)
		}
	}()
	ids, err := native.identities()
	if err != nil || !ids.valid() {
		return nil, errors.Join(errors.New("original backup identities required"), err)
	}
	origin := executionProfileBackupOrigin{1, "protected-execution-profile-backup", bytes.Clone(p.fence), ids}
	raw, err := json.Marshal(origin)
	if err != nil || len(raw)+1 > maxProfileBackupMarkerBytes {
		return nil, errors.New("profile backup origin bound exceeded")
	}
	p.marker = append(raw, '\n')
	if err = native.initialize(ctx, p.marker); err != nil {
		return nil, err
	}
	p.initialized = true
	if err = ctx.Err(); err != nil {
		return nil, err
	}
	return &ExecutionProfileBackup{p}, nil
}

func (s *Store) OpenExecutionProfileBackup(ctx context.Context, journal *ExecutionJournalReader) (result *ExecutionProfileBackupReader, resultErr error) {
	if s == nil {
		return nil, ErrClosed
	}
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	s.mu.Lock()
	defer s.mu.Unlock()
	if s.closed.Load() {
		return nil, ErrClosed
	}
	if ctx == nil || journal == nil || journal.state == nil || journal.state.store != s || len(s.profileBackups) >= maxExecutionJournals {
		return nil, errors.New("bounded live same-store journal reader required")
	}
	p := &executionProfileBackupState{store: s, owner: registrationContext{Context: ctx, store: s}, reader: journal}
	if err := p.journalReady(ctx); err != nil {
		return nil, err
	}
	b, ok := s.backend.(executionProfileBackupStoreBackend)
	if !ok {
		return nil, ErrNativeRequired
	}
	native, marker, err := b.openExecutionProfileBackup(ctx, journal.state.origin.ClientSID, journal.state.origin.ExecutionID)
	if err != nil {
		if native != nil {
			err = errors.Join(err, native.close())
		}
		return nil, errors.Join(ErrExecutionProfileBackupUncertain, err)
	}
	if native == nil {
		return nil, errors.Join(ErrExecutionProfileBackupUncertain, ErrNativeRequired)
	}
	p.backend = native
	p.origin = journal.state.origin
	p.fence = bytes.Clone(journal.state.fence)
	p.marker = bytes.Clone(marker)
	s.profileBackups = append(s.profileBackups, p)
	defer func() {
		if resultErr != nil {
			resultErr = p.fail(resultErr)
		}
	}()
	origin, completion, err := decodeExecutionProfileBackup(p.marker, p.fence)
	if err != nil {
		return nil, err
	}
	ids, err := native.identities()
	if err != nil || !sameExecutionJournalIDs(ids, origin.Identities) {
		return nil, errors.Join(errors.New("backup original native identity changed"), err)
	}
	p.size = completion.Bytes
	p.digest = completion.SHA256
	p.initialized = true
	p.written = true
	p.completed = true
	if err = p.ready(ctx); err != nil {
		return nil, err
	}
	return &ExecutionProfileBackupReader{p}, nil
}

// Caller holds Store.mu then backup.mu. Journal/generation locks are acquired
// only beneath them; no journal operation acquires a backup lock.
func (p *executionProfileBackupState) journalReady(ctx context.Context) error {
	if ctx == nil {
		return errors.New("live profile backup context required")
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	if p.journal != nil {
		j := p.journal
		j.mu.Lock()
		defer j.mu.Unlock()
		j.generation.generation.mu.Lock()
		defer j.generation.generation.mu.Unlock()
		if err := j.ready(ctx); err != nil {
			return err
		}
		if len(p.fence) > 0 && !bytes.Equal(p.fence, j.fence) {
			return errors.New("original journal fence changed")
		}
		return nil
	}
	if p.reader != nil {
		j := p.reader.state
		j.mu.Lock()
		defer j.mu.Unlock()
		if err := j.ready(); err != nil {
			return err
		}
		if len(p.fence) > 0 && !bytes.Equal(p.fence, j.fence) {
			return errors.New("original reader fence changed")
		}
		return nil
	}
	return ErrClosed
}
func (p *executionProfileBackupState) ready(ctx context.Context) error {
	if p.closed || p.store.closed.Load() {
		return ErrClosed
	}
	if p.failure != nil {
		return errors.Join(ErrPoisoned, p.failure)
	}
	if err := p.owner.Err(); err != nil {
		return err
	}
	if err := p.journalReady(ctx); err != nil {
		return err
	}
	return errors.Join(p.backend.validate(p.marker, p.size, p.digest), p.owner.Err(), ctx.Err())
}
func (p *executionProfileBackupState) fail(err error) error {
	p.failure = errors.Join(p.failure, err)
	return errors.Join(ErrExecutionProfileBackupUncertain, err, p.closeLocked())
}

// Write streams exactly one bounded archive and verifies its exact length/hash.
// Hashes supplied here bind storage bytes only and never prove a source snapshot.
func (b *ExecutionProfileBackup) Write(ctx context.Context, source io.Reader, size int64, digest string) (resultErr error) {
	if b == nil || b.state == nil {
		return ErrClosed
	}
	p := b.state
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	p.store.mu.Lock()
	defer p.store.mu.Unlock()
	p.mu.Lock()
	defer p.mu.Unlock()
	if p.closed {
		return ErrClosed
	}
	defer func() {
		if resultErr != nil {
			resultErr = p.fail(resultErr)
		}
	}()
	if err := p.ready(ctx); err != nil {
		return err
	}
	if p.written || source == nil || size <= 0 || size > MaxPayloadBytes || !validHex(digest, 64) {
		return errors.New("fresh bounded exact profile archive required")
	}
	if err := p.backend.write(ctx, source, size, digest); err != nil {
		return err
	}
	p.size = size
	p.digest = digest
	p.written = true
	return p.ready(ctx)
}

// Complete publishes completion only after exact archive readback and all live
// preconditions. Once final publication begins, successful durable completion
// wins over late cancellation. Actual publication/check failure is uncertain:
// valid completed bytes may already exist, and must never be retried or erased.
func (b *ExecutionProfileBackup) Complete(ctx context.Context) (resultErr error) {
	if b == nil || b.state == nil {
		return ErrClosed
	}
	p := b.state
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	p.store.mu.Lock()
	defer p.store.mu.Unlock()
	p.mu.Lock()
	defer p.mu.Unlock()
	if p.closed {
		return ErrClosed
	}
	defer func() {
		if resultErr != nil {
			resultErr = p.fail(resultErr)
		}
	}()
	if err := p.ready(ctx); err != nil {
		return err
	}
	if !p.written || p.completed {
		return errors.New("one fresh completed archive required")
	}
	completion := executionProfileBackupCompletion{1, "completed-execution-profile-backup", registrationDigest(bytes.TrimSuffix(p.marker, []byte{'\n'})), p.size, p.digest}
	raw, err := json.Marshal(completion)
	if err != nil {
		return err
	}
	next := append(append(bytes.Clone(p.marker), raw...), '\n')
	if err = errors.Join(ctx.Err(), p.owner.Err()); err != nil {
		return err
	}
	if err = p.backend.complete(ctx, p.marker, next, p.size, p.digest); err != nil {
		return err
	}
	p.marker = next
	p.completed = true
	return nil
}
func (p *executionProfileBackupState) snapshot(ctx context.Context) (result ExecutionProfileBackupSnapshot, resultErr error) {
	if p == nil {
		return result, ErrClosed
	}
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	p.store.mu.Lock()
	defer p.store.mu.Unlock()
	p.mu.Lock()
	defer p.mu.Unlock()
	if p.closed || p.store.closed.Load() {
		return result, ErrClosed
	}
	if err := p.ready(ctx); err != nil {
		return result, p.fail(err)
	}
	if !p.completed {
		return result, ErrExecutionProfileBackupUncertain
	}
	return ExecutionProfileBackupSnapshot{p.origin.ClientSID, p.origin.ExecutionID, p.origin.Generation, p.origin.CompletionSHA256, registrationDigest(p.fence), p.size, p.digest, bytes.Clone(p.marker)}, nil
}
func (b *ExecutionProfileBackup) Snapshot(ctx context.Context) (ExecutionProfileBackupSnapshot, error) {
	if b == nil {
		return ExecutionProfileBackupSnapshot{}, ErrClosed
	}
	return b.state.snapshot(ctx)
}
func (b *ExecutionProfileBackupReader) Snapshot(ctx context.Context) (ExecutionProfileBackupSnapshot, error) {
	if b == nil {
		return ExecutionProfileBackupSnapshot{}, ErrClosed
	}
	return b.state.snapshot(ctx)
}
func (b *ExecutionProfileBackup) Validate(ctx context.Context) error {
	_, err := b.Snapshot(ctx)
	return err
}
func (b *ExecutionProfileBackupReader) Validate(ctx context.Context) error {
	_, err := b.Snapshot(ctx)
	return err
}

// ReadAt reads retained immutable archive bytes, never a caller-selected path.
// Complete-archive hash verification occurs at Open/Validate/Snapshot; this
// bounded read rechecks original identities and exact marker before and after.
func (p *executionProfileBackupState) readAt(ctx context.Context, dst []byte, offset int64) (n int, resultErr error) {
	if p == nil {
		return 0, ErrClosed
	}
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	p.store.mu.Lock()
	defer p.store.mu.Unlock()
	p.mu.Lock()
	defer p.mu.Unlock()
	if p.closed || p.store.closed.Load() {
		return 0, ErrClosed
	}
	if !p.completed || ctx == nil || offset < 0 || len(dst) > maxExecutionFactBytes {
		return 0, errors.New("bounded backup read required")
	}
	if err := p.owner.Err(); err != nil {
		return 0, p.fail(err)
	}
	if err := p.journalReady(ctx); err != nil {
		return 0, p.fail(err)
	}
	n, resultErr = p.backend.readAt(dst, offset)
	if resultErr != nil && !errors.Is(resultErr, io.EOF) {
		return n, p.fail(resultErr)
	}
	if err := errors.Join(ctx.Err(), p.owner.Err(), p.journalReady(ctx)); err != nil {
		return n, p.fail(err)
	}
	return n, resultErr
}
func (b *ExecutionProfileBackup) ReadAt(ctx context.Context, dst []byte, offset int64) (int, error) {
	if b == nil {
		return 0, ErrClosed
	}
	return b.state.readAt(ctx, dst, offset)
}
func (b *ExecutionProfileBackupReader) ReadAt(ctx context.Context, dst []byte, offset int64) (int, error) {
	if b == nil {
		return 0, ErrClosed
	}
	return b.state.readAt(ctx, dst, offset)
}

func (p *executionProfileBackupState) closeLocked() error {
	if p.closed {
		return p.closeErr
	}
	p.closed = true
	p.closeErr = p.failure
	if p.initialized {
		p.closeErr = errors.Join(p.closeErr, p.backend.validate(p.marker, p.size, p.digest))
	}
	p.closeErr = errors.Join(p.closeErr, p.backend.close())
	if p.closeErr != nil {
		p.closeErr = errors.Join(ErrExecutionProfileBackupUncertain, p.closeErr)
	}
	return p.closeErr
}
func (p *executionProfileBackupState) close() error {
	if p == nil {
		return nil
	}
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	p.store.mu.Lock()
	defer p.store.mu.Unlock()
	p.mu.Lock()
	defer p.mu.Unlock()
	return p.closeLocked()
}
func (b *ExecutionProfileBackup) Close() error {
	if b == nil {
		return nil
	}
	return b.state.close()
}
func (b *ExecutionProfileBackupReader) Close() error {
	if b == nil {
		return nil
	}
	return b.state.close()
}
func (*ExecutionProfileBackup) MarshalJSON() ([]byte, error)       { return nil, ErrNativeRequired }
func (*ExecutionProfileBackup) UnmarshalJSON([]byte) error         { return ErrNativeRequired }
func (*ExecutionProfileBackupReader) MarshalJSON() ([]byte, error) { return nil, ErrNativeRequired }
func (*ExecutionProfileBackupReader) UnmarshalJSON([]byte) error   { return ErrNativeRequired }
