package brokerstore

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"runtime"
	"sort"
	"strings"
	"sync"

	"example.com/opennav-update-verifier/brokerregistration"
)

// ExecutionJournalFact is a copied original storage fact, not authenticated
// evidence of its assertion, health, selection, one-use spawn or restoration.
type ExecutionJournalFact struct {
	Kind     ExecutionFactKind
	Original []byte
}

// ExecutionJournalReadSnapshot retains the original schema and ordered bytes.
// No parsed field, original completion or fact can reconstruct a native owner.
type ExecutionJournalReadSnapshot struct {
	ExecutionJournalSnapshot
	RepairRecovery     *RepairRecoveryLink
	GenerationPurpose  string
	OriginalCompletion []byte
	Facts              []ExecutionJournalFact
}

type executionJournalReadBackend interface {
	validate([]byte, []byte) error
	close() error
}
type executionJournalReopenStoreBackend interface {
	executionJournalNames(context.Context, string) ([]string, error)
	openExecutionJournal(context.Context, string, string) (executionJournalReadBackend, []byte, []byte, error)
}

// ExecutionJournalReader is a read-only native custody capability, with shared
// revocation across copied wrappers. Only OpenExecutionJournal can establish one
// from fixed protected files. It exposes no append/path/adoption/launch API.
type ExecutionJournalReader struct{ state *executionJournalReaderState }
type executionJournalReaderState struct {
	mu                sync.Mutex
	store             *Store
	owner             context.Context
	backend           executionJournalReadBackend
	origin            executionJournalOrigin
	fence, raw        []byte
	facts             []ExecutionJournalFact
	closed            bool
	failure, closeErr error
}

func decodeReadExecutionJournal(client, id string, fence, raw []byte) (executionJournalOrigin, []ExecutionJournalFact, error) {
	var origin executionJournalOrigin
	if err := validateExecutionJournalBytes(fence, raw); err != nil {
		return origin, nil, err
	}
	if err := json.Unmarshal(fence, &origin); err != nil {
		return origin, nil, err
	}
	if origin.ClientSID != client || origin.ExecutionID != id {
		return origin, nil, errors.New("fixed journal name differs from original origin")
	}
	var facts []ExecutionJournalFact
	for _, line := range bytes.Split(raw[len(fence)+1:], []byte{'\n'}) {
		if len(line) == 0 {
			continue
		} // validator already requires every record's newline
		var record executionJournalRecord
		if err := json.Unmarshal(line, &record); err != nil {
			return origin, nil, err
		}
		facts = append(facts, ExecutionJournalFact{record.Kind, bytes.Clone(record.Original)})
	}
	return origin, facts, nil
}

// OpenExecutionJournal reopens only this real canonical client's fixed permanent
// execution pair. It never resumes append, repairs a partial, trusts supplied
// bytes/path/receipt or infers selection from journal facts. Missing/partial or
// changed pairs return ErrExecutionJournalUncertain and are left untouched.
func (s *Store) OpenExecutionJournal(ctx context.Context, client, id string) (result *ExecutionJournalReader, resultErr error) {
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
	if ctx == nil || !brokerregistration.IsCanonicalUserSID(client) || !validHex(id, 32) {
		return nil, errors.New("canonical client and execution identity required")
	}
	ctx = registrationContext{Context: ctx, store: s}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	// Cleanly closed readers grant no authority and need not consume the live
	// owner bound forever. Keep failed cleanup owners so Store.Close retains their
	// exact uncertainty; pruning never changes disk bytes or permits append.
	retained := s.journalReaders[:0]
	for _, reader := range s.journalReaders {
		if !reader.state.closed || reader.state.closeErr != nil {
			retained = append(retained, reader)
		}
	}
	clear(s.journalReaders[len(retained):])
	s.journalReaders = retained
	if len(s.journalReaders) >= maxExecutionJournals {
		return nil, errors.New("bounded retained execution reader capacity exhausted")
	}
	b, ok := s.backend.(executionJournalReopenStoreBackend)
	if !ok {
		return nil, ErrNativeRequired
	}
	native, fence, raw, err := b.openExecutionJournal(ctx, client, id)
	if err != nil {
		if native != nil {
			err = errors.Join(err, native.close())
		}
		return nil, errors.Join(ErrExecutionJournalUncertain, err)
	}
	if native == nil {
		return nil, errors.Join(ErrExecutionJournalUncertain, errors.New("native execution reader absent"))
	}
	p := &executionJournalReaderState{store: s, owner: ctx, backend: native, fence: bytes.Clone(fence), raw: bytes.Clone(raw)}
	reader := &ExecutionJournalReader{state: p}
	defer func() {
		if resultErr != nil {
			p.failure = resultErr
			resultErr = errors.Join(ErrExecutionJournalUncertain, resultErr, p.closeLocked())
		}
	}()
	p.origin, p.facts, err = decodeReadExecutionJournal(client, id, p.fence, p.raw)
	if err != nil {
		return nil, err
	}
	if err = p.ready(); err != nil {
		return nil, err
	}
	s.journalReaders = append(s.journalReaders, reader)
	return reader, nil
}
func (p *executionJournalReaderState) ready() error {
	if p.closed {
		return errors.Join(ErrClosed, p.closeErr)
	}
	if p.failure != nil {
		return errors.Join(ErrPoisoned, p.failure)
	}
	if p.store.closed.Load() {
		return ErrClosed
	}
	if err := p.owner.Err(); err != nil {
		return err
	}
	return errors.Join(p.backend.validate(p.fence, p.raw), p.owner.Err())
}
func (r *ExecutionJournalReader) Validate() error { _, err := r.Snapshot(); return err }
func (r *ExecutionJournalReader) Snapshot() (result ExecutionJournalReadSnapshot, resultErr error) {
	if r == nil || r.state == nil {
		return result, ErrClosed
	}
	p := r.state
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	p.store.mu.Lock()
	defer p.store.mu.Unlock()
	p.mu.Lock()
	defer p.mu.Unlock()
	if err := p.ready(); err != nil {
		if p.closed || p.store.closed.Load() {
			return result, err
		}
		p.failure = errors.Join(ErrExecutionJournalUncertain, err)
		return result, errors.Join(p.failure, p.closeLocked())
	}
	result = ExecutionJournalReadSnapshot{ExecutionJournalSnapshot: ExecutionJournalSnapshot{
		ExecutionID: p.origin.ExecutionID, ClientSID: p.origin.ClientSID, Generation: p.origin.Generation,
		CompletionSHA256: p.origin.CompletionSHA256, JournalSHA256: registrationDigest(p.raw),
		OriginalFence: bytes.Clone(p.fence), OriginalJournal: bytes.Clone(p.raw), FactCount: len(p.facts)},
		GenerationPurpose: p.origin.GenerationPurpose, OriginalCompletion: bytes.Clone(p.origin.OriginalCompletion)}
	if p.origin.RepairRecovery != nil {
		copy := *p.origin.RepairRecovery
		result.RepairRecovery = &copy
	}
	for _, fact := range p.facts {
		result.Facts = append(result.Facts, ExecutionJournalFact{fact.Kind, bytes.Clone(fact.Original)})
	}
	return result, nil
}
func (r *ExecutionJournalReader) Close() error {
	if r == nil || r.state == nil {
		return nil
	}
	p := r.state
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	p.store.mu.Lock()
	defer p.store.mu.Unlock()
	p.mu.Lock()
	defer p.mu.Unlock()
	return p.closeLocked()
}
func (p *executionJournalReaderState) closeLocked() error {
	if p.closed {
		return p.closeErr
	}
	p.closed = true
	p.closeErr = errors.Join(p.failure, p.backend.validate(p.fence, p.raw), p.backend.close())
	if p.closeErr != nil {
		p.closeErr = errors.Join(ErrExecutionJournalUncertain, p.closeErr)
	}
	return p.closeErr
}
func (*ExecutionJournalReader) MarshalJSON() ([]byte, error) { return nil, ErrNativeRequired }
func (*ExecutionJournalReader) UnmarshalJSON([]byte) error   { return ErrNativeRequired }

// ExecutionJournalEntry describes every observed attempt in this client's fixed
// namespace. Readable describes storage integrity only, never transaction state.
// Noncanonical names have no ExecutionID; EntryNames retain them as inert data.
type ExecutionJournalEntry struct {
	ExecutionID string
	EntryNames  []string
	Readable    bool
	Problem     string
}

// ExecutionJournalNamespaceError preserves incomplete enumeration as a typed
// uncertain result. Entries are independent copies, never native capabilities.
type ExecutionJournalNamespaceError struct {
	ClientSID string
	Entries   []ExecutionJournalEntry
	Cause     error
}

func (e *ExecutionJournalNamespaceError) Error() string {
	return fmt.Sprintf("protected execution namespace for %s incomplete or uncertain: %v", e.ClientSID, e.Cause)
}
func (e *ExecutionJournalNamespaceError) Unwrap() error {
	return errors.Join(ErrExecutionJournalUncertain, e.Cause)
}
func cloneExecutionJournalEntries(entries []ExecutionJournalEntry) []ExecutionJournalEntry {
	result := append([]ExecutionJournalEntry(nil), entries...)
	for i := range result {
		result[i].EntryNames = append([]string(nil), result[i].EntryNames...)
	}
	return result
}
func executionJournalEntries(client string, names []string) []ExecutionJournalEntry {
	prefix := client + ".execution"
	groups := map[string]*ExecutionJournalEntry{}
	var entries []ExecutionJournalEntry
	for _, name := range names {
		if len(name) < len(prefix) || !strings.EqualFold(name[:len(prefix)], prefix) {
			continue
		}
		remainder := strings.TrimPrefix(name, prefix+".")
		id, suffix, found := strings.Cut(remainder, ".")
		canonical := strings.HasPrefix(name, prefix+".") && found && validHex(id, 32) && (suffix == "once" || suffix == "journal")
		if !canonical {
			entries = append(entries, ExecutionJournalEntry{EntryNames: []string{name}, Problem: "noncanonical execution namespace entry"})
			continue
		}
		entry := groups[id]
		if entry == nil {
			entry = &ExecutionJournalEntry{ExecutionID: id}
			groups[id] = entry
		}
		entry.EntryNames = append(entry.EntryNames, name)
	}
	for _, entry := range groups {
		sort.Strings(entry.EntryNames)
		if len(entry.EntryNames) != 2 || entry.EntryNames[0] != prefix+"."+entry.ExecutionID+".journal" || entry.EntryNames[1] != prefix+"."+entry.ExecutionID+".once" {
			entry.Problem = "permanent reservation/journal pair missing or duplicate"
		}
		entries = append(entries, *entry)
	}
	sort.Slice(entries, func(i, j int) bool { return entries[i].EntryNames[0] < entries[j].EntryNames[0] })
	return entries
}

// The original pair cannot coexist with a case alias, suffix fork or third
// member for the same execution ID. Other attempts remain visible separately.
func executionJournalPairNames(client, id string, names []string) error {
	prefix := client + ".execution." + id
	var matching []string
	for _, name := range names {
		if len(name) >= len(prefix) && strings.EqualFold(name[:len(prefix)], prefix) {
			matching = append(matching, name)
		}
	}
	sort.Strings(matching)
	if len(matching) != 2 || matching[0] != prefix+".journal" || matching[1] != prefix+".once" {
		return errors.New("original execution namespace pair missing/aliased/forked")
	}
	return nil
}

// EnumerateExecutionJournals returns all this canonical client's attempts,
// including orphaned/empty/noncanonical/aliased entries. Any incomplete entry
// accompanies a typed ErrExecutionJournalUncertain; valid entries are not
// silently discarded. It is a copied point-in-time inventory, not a namespace
// capability or selected history. Host policy must serialize effects and reopen
// individual readers before using retained original facts. No files are changed.
func (s *Store) EnumerateExecutionJournals(ctx context.Context, client string) (entries []ExecutionJournalEntry, resultErr error) {
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
	if ctx == nil || !brokerregistration.IsCanonicalUserSID(client) {
		return nil, errors.New("canonical real client required")
	}
	ctx = registrationContext{Context: ctx, store: s}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	b, ok := s.backend.(executionJournalReopenStoreBackend)
	if !ok {
		return nil, ErrNativeRequired
	}
	names, err := b.executionJournalNames(ctx, client)
	if err != nil {
		return nil, &ExecutionJournalNamespaceError{ClientSID: client, Cause: err}
	}
	if len(names) > maxExecutionNamespaceFiles {
		return nil, &ExecutionJournalNamespaceError{ClientSID: client, Cause: errors.New("execution namespace exceeds fixed enumeration bound")}
	}
	entries = executionJournalEntries(client, names)
	var problems error
	if len(entries) > maxExecutionJournals {
		problems = errors.New("execution attempt capacity exceeded")
	}
	for i := range entries {
		entry := &entries[i]
		if entry.Problem != "" {
			problems = errors.Join(problems, errors.New(entry.Problem))
			continue
		}
		if err := ctx.Err(); err != nil {
			entry.Problem = "enumeration canceled before original custody"
			problems = errors.Join(problems, err)
			continue
		}
		native, fence, raw, err := b.openExecutionJournal(ctx, client, entry.ExecutionID)
		if err == nil && native == nil {
			err = errors.New("native original journal custody absent")
		}
		if err == nil {
			_, _, err = decodeReadExecutionJournal(client, entry.ExecutionID, fence, raw)
		}
		if err == nil {
			err = native.validate(fence, raw)
		}
		if native != nil {
			err = errors.Join(err, native.close())
		}
		if err != nil {
			entry.Problem = "original execution pair incomplete or custody uncertain"
			problems = errors.Join(problems, err)
			continue
		}
		entry.Readable = true
	}
	if err := ctx.Err(); err != nil {
		problems = errors.Join(problems, err)
	}
	if problems != nil {
		return entries, &ExecutionJournalNamespaceError{ClientSID: client, Entries: cloneExecutionJournalEntries(entries), Cause: problems}
	}
	return entries, nil
}
