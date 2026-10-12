package brokerstore

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"runtime"
	"strings"
)

const maxStageSealOriginBytes = 32 << 20
const maxStageSealRecordBytes = 4096

var ErrGenerationStageSealUncertain = errors.New("generation stage seal incomplete or uncertain")

type stageSealOrigin struct {
	Schema             int                 `json:"schema"`
	Purpose            string              `json:"purpose"`
	OriginalCompletion []byte              `json:"originalCompletion"`
	Identities         executionJournalIDs `json:"identities"`
}
type stageSealRecord struct {
	Schema       int            `json:"schema"`
	Purpose      string         `json:"purpose"`
	OriginSHA256 string         `json:"originSha256"`
	Receipt      baselineMember `json:"receipt"`
}
type stageSealStoreBackend interface {
	prepareStageSeal(context.Context, generationBackend, candidateBackend, generationRecord, []byte) (stageSealWriter, error)
	openStageSeal(context.Context, generationBackend, generationRecord, []byte) (stageSealReaderBackend, []byte, []byte, []byte, error)
}
type stageSealWriter interface {
	complete(context.Context, fileRecord) error
	close() error
}
type stageSealReaderBackend interface{ validate(context.Context) error }

// GenerationStageSealSnapshot copies completed storage evidence. Neither these
// bytes nor a staging receipt grant selection, launch or commissioning authority.
type GenerationStageSealSnapshot struct {
	Receipt       []byte
	OriginalSeal  []byte
	SealSHA256    string
	ReceiptSHA256 string
	Completion    GenerationCompletion
}

// GenerationStageSeal is a revocable view attached to one live same-Store
// ReopenedGeneration. Native readonly files and ancestor pins belong to that
// generation and drain when it or its Store closes. Close revokes this view;
// like GenerationReadHandle, it does not release its owner's physical custody.
type GenerationStageSeal struct{ state *generationStageSealState }
type generationStageSealState struct {
	owner                   *ReopenedGeneration
	context                 context.Context
	backend                 stageSealReaderBackend
	marker, record, receipt []byte
	closed                  bool
	closeErr                error
}

func stageSealPairNames(client, stage string, names []string, fresh bool) error {
	prefix := client + ".stage." + stage
	seen := map[string]bool{}
	count := 0
	for _, name := range names {
		lower := strings.ToLower(name)
		if !strings.HasPrefix(lower, strings.ToLower(prefix)) {
			continue
		}
		tail := lower[len(prefix):]
		if tail != "" && !strings.HasPrefix(tail, ".") {
			continue
		}
		count++
		if fresh || (name != prefix+".once" && name != prefix+".json") || seen[name] {
			return ErrGenerationStageSealUncertain
		}
		seen[name] = true
	}
	if !fresh && (count != 2 || !seen[prefix+".once"] || !seen[prefix+".json"]) {
		return ErrGenerationStageSealUncertain
	}
	return nil
}
func decodeStageSeal(marker, raw, completion []byte) (stageSealOrigin, stageSealRecord, error) {
	var origin stageSealOrigin
	var record stageSealRecord
	lines := bytes.Split(marker, []byte{'\n'})
	if len(marker) > maxStageSealOriginBytes+66 || len(raw) == 0 || len(raw) > maxStageSealRecordBytes || len(lines) != 3 || len(lines[2]) != 0 {
		return origin, record, ErrGenerationStageSealUncertain
	}
	if json.Unmarshal(lines[0], &origin) != nil || json.Unmarshal(raw, &record) != nil {
		return origin, record, ErrGenerationStageSealUncertain
	}
	canonical, _ := json.Marshal(origin)
	canonicalRecord, _ := json.Marshal(record)
	if !bytes.Equal(canonical, lines[0]) || !bytes.Equal(canonicalRecord, raw) || origin.Schema != 1 || origin.Purpose != "original-generation-stage-seal" || !bytes.Equal(origin.OriginalCompletion, completion) || !origin.Identities.valid() || record.Schema != 1 || record.Purpose != "completed-generation-stage-seal" || record.OriginSHA256 != registrationDigest(canonical) || !bytes.Equal(lines[1], []byte(registrationDigest(raw))) || !record.Receipt.Identity.valid() || record.Receipt.Length <= 0 || record.Receipt.Length > MaxReceiptBytes || !validHex(record.Receipt.SHA256, 64) {
		return origin, record, ErrGenerationStageSealUncertain
	}
	return origin, record, nil
}

// Called before acquiring Store.mu. Reopen identities are derived only from the
// retained candidate, never supplied by a serialized receipt. A materialized
// candidate cannot fall back to the historical unmaterialized Seal behavior.
func (c *Candidate) stageSealGeneration(ctx context.Context) (*ReopenedGeneration, error) {
	c.mu.Lock()
	err := c.ready()
	materialized, client, stage := c.materialized, c.clientSID, c.stageID
	c.mu.Unlock()
	if err != nil {
		return nil, err
	}
	if !materialized {
		return nil, nil
	}
	return c.store.OpenReadOnlyGeneration(ctx, client, stage)
}
func (c *Candidate) sealStage(ctx context.Context, receipt []byte) (resultErr error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	reopened, err := c.stageSealGeneration(ctx)
	if err != nil {
		return err
	}
	if reopened != nil {
		defer func() {
			if err := reopened.Close(); err != nil {
				resultErr = errors.Join(ErrGenerationStageSealUncertain, resultErr, err)
				c.mu.Lock()
				c.poisoned = true
				c.mu.Unlock()
			}
		}()
	}
	c.store.mu.Lock()
	defer c.store.mu.Unlock()
	c.mu.Lock()
	defer c.mu.Unlock()
	if err = c.ready(); err != nil {
		return err
	}
	defer func() {
		if resultErr != nil {
			c.poisoned = true
		}
	}()
	if ctx == nil || len(receipt) == 0 || len(receipt) > MaxReceiptBytes || len(c.files) == 0 || c.sealed {
		return errors.New("nonempty fresh payload and bounded receipt required")
	}
	ctx = registrationContext{Context: ctx, store: c.store}
	if err = ctx.Err(); err != nil {
		return err
	}
	if c.recoveryEvidence == nil {
		return errors.New("complete bound original recovery evidence required")
	}
	if err = c.validateRecoveryEvidence(ctx, *c.recoveryEvidence); err != nil {
		return err
	}
	var writer stageSealWriter
	if c.materialized {
		if reopened == nil || reopened.purpose != GenerationPurposeSignedCandidate || reopened.owner == nil {
			return ErrGenerationStageSealUncertain
		}
		g := reopened.owner.generation
		g.mu.Lock()
		defer g.mu.Unlock()
		if err = reopened.owner.ready(); err != nil {
			return err
		}
		if g.record.ClientSID != c.clientSID || g.record.Generation != c.stageID || g.record.Identity != *c.recoveryEvidence {
			return ErrGenerationStageSealUncertain
		}
		if err = g.validateContents(ctx); err != nil {
			return err
		}
		b, ok := c.store.backend.(stageSealStoreBackend)
		if !ok {
			return ErrNativeRequired
		}
		writer, err = b.prepareStageSeal(ctx, g.backend, c.backend, g.record, g.raw)
		if err != nil {
			if writer != nil {
				err = errors.Join(err, writer.close())
			}
			return errors.Join(ErrGenerationStageSealUncertain, err)
		}
		if writer == nil {
			return ErrGenerationStageSealUncertain
		}
		defer func() {
			resultErr = errors.Join(resultErr, writer.close())
			if resultErr != nil {
				resultErr = errors.Join(ErrGenerationStageSealUncertain, resultErr)
				c.poisoned = true
			}
		}()
	}
	file, err := c.backend.create(ReceiptFilename)
	if err != nil {
		return err
	}
	original := fileRecord{file: file, length: int64(len(receipt)), digest: registrationDigest(receipt)}
	c.files = append(c.files, original)
	if err = c.copyFile(ctx, file, bytes.NewReader(receipt), original.length, original.digest); err != nil {
		return err
	}
	if writer != nil {
		if err = writer.complete(ctx, original); err != nil {
			return errors.Join(ErrGenerationStageSealUncertain, err)
		}
	}
	c.sealed = true
	return nil
}

// OpenGenerationStageSeal reads only a completed protected seal for the given
// live signed-candidate owner. It refuses historical absence, partials, aliases,
// mismatched originals and cleanup uncertainty without writing or repairing.
func (s *Store) OpenGenerationStageSeal(ctx context.Context, owner *ReopenedGeneration) (result *GenerationStageSeal, resultErr error) {
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
	if ctx == nil || owner == nil || owner.owner == nil || owner.owner.generation == nil || owner.owner.generation.store != s || owner.purpose != GenerationPurposeSignedCandidate {
		return nil, errors.New("live same-Store reopened signed candidate required")
	}
	g := owner.owner.generation
	g.mu.Lock()
	defer g.mu.Unlock()
	ctx = registrationContext{Context: ctx, store: s}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if err := owner.owner.ready(); err != nil {
		return nil, err
	}
	if g.record.Purpose != string(GenerationPurposeSignedCandidate) {
		return nil, ErrGenerationStageSealUncertain
	}
	if err := g.validateContents(ctx); err != nil {
		return nil, err
	}
	b, ok := s.backend.(stageSealStoreBackend)
	if !ok {
		return nil, ErrNativeRequired
	}
	// Any backend failure, including failed acquisition cleanup, revokes the
	// supplied lifetime owner. Partial storage is never adopted on another use.
	defer func() {
		if resultErr != nil {
			g.failure = errors.Join(g.failure, resultErr)
			resultErr = errors.Join(ErrGenerationStageSealUncertain, resultErr, g.closeLocked())
		}
	}()
	backend, marker, raw, receipt, err := b.openStageSeal(ctx, g.backend, g.record, g.raw)
	if err != nil {
		return nil, err
	}
	_, record, err := decodeStageSeal(marker, raw, g.raw)
	if err != nil || int64(len(receipt)) != record.Receipt.Length || registrationDigest(receipt) != record.Receipt.SHA256 || backend == nil {
		return nil, errors.Join(ErrGenerationStageSealUncertain, err)
	}
	if err = errors.Join(backend.validate(ctx), ctx.Err()); err != nil {
		return nil, err
	}
	return &GenerationStageSeal{state: &generationStageSealState{owner: owner, context: ctx, backend: backend, marker: bytes.Clone(marker), record: bytes.Clone(raw), receipt: bytes.Clone(receipt)}}, nil
}
func (r *GenerationStageSeal) Snapshot(ctx context.Context) (result GenerationStageSealSnapshot, resultErr error) {
	if r == nil || r.state == nil {
		return result, ErrClosed
	}
	p := r.state
	g := p.owner.owner.generation
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	g.store.mu.Lock()
	defer g.store.mu.Unlock()
	g.mu.Lock()
	defer g.mu.Unlock()
	if p.closed {
		return result, ErrClosed
	}
	if err := p.owner.owner.ready(); err != nil {
		return result, err
	}
	defer func() {
		if resultErr != nil {
			p.closed = true
			g.failure = errors.Join(g.failure, resultErr)
			resultErr = errors.Join(ErrGenerationStageSealUncertain, resultErr, g.closeLocked())
			p.closeErr = resultErr
		}
	}()
	if ctx == nil {
		return result, errors.New("live seal snapshot context required")
	}
	if err := errors.Join(ctx.Err(), p.context.Err()); err != nil {
		return result, err
	}
	if err := g.validateContents(registrationContext{Context: ctx, store: g.store}); err != nil {
		return result, err
	}
	if err := p.backend.validate(ctx); err != nil {
		return result, err
	}
	_, record, err := decodeStageSeal(p.marker, p.record, g.raw)
	if err != nil {
		return result, err
	}
	completion, err := g.completionRecord()
	if err != nil {
		return result, err
	}
	if err = errors.Join(ctx.Err(), p.context.Err()); err != nil {
		return result, err
	}
	return GenerationStageSealSnapshot{Receipt: bytes.Clone(p.receipt), OriginalSeal: bytes.Clone(p.record), SealSHA256: registrationDigest(p.record), ReceiptSHA256: record.Receipt.SHA256, Completion: completion}, nil
}
func (r *GenerationStageSeal) Validate(ctx context.Context) error {
	_, err := r.Snapshot(ctx)
	return err
}
func (r *GenerationStageSeal) Close() error {
	if r == nil || r.state == nil {
		return nil
	}
	p := r.state
	g := p.owner.owner.generation
	g.store.mu.Lock()
	defer g.store.mu.Unlock()
	g.mu.Lock()
	defer g.mu.Unlock()
	p.closed = true
	return p.closeErr
}
func (*GenerationStageSeal) MarshalJSON() ([]byte, error) { return nil, ErrNativeRequired }
func (*GenerationStageSeal) UnmarshalJSON([]byte) error   { return ErrNativeRequired }

func stageSealDirectoryName(name string) error {
	if len(name) == 0 || len(name) > 240 || strings.Contains(name, `\`) {
		return ErrGenerationStageSealUncertain
	}
	for _, part := range strings.Split(name, "/") {
		if !safeComponent(part) {
			return ErrGenerationStageSealUncertain
		}
	}
	return nil
}
