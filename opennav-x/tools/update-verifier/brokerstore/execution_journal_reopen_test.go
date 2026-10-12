package brokerstore

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"strings"
	"sync"
	"testing"
)

type memoryExecutionReadPair struct{ fence, raw []byte }
type memoryExecutionReader struct {
	pair              *memoryExecutionReadPair
	fault, closeFault error
	cancel            context.CancelFunc
	closes            int
	events            *[]string
}

func (r *memoryExecutionReader) validate(fence, raw []byte) error {
	if r.cancel != nil {
		r.cancel()
	}
	if r.fault != nil {
		return r.fault
	}
	if !bytes.Equal(fence, r.pair.fence) || !bytes.Equal(raw, r.pair.raw) {
		return testFault
	}
	return validateExecutionJournalBytes(fence, raw)
}
func (r *memoryExecutionReader) close() error {
	r.closes++
	if r.events != nil {
		*r.events = append(*r.events, "reader-close")
	}
	return r.closeFault
}

type memoryExecutionReopenStore struct {
	*memoryStore
	names                                              []string
	pairs                                              map[string]*memoryExecutionReadPair
	readers                                            []*memoryExecutionReader
	openFault, enumerateFault, readerFault, closeFault error
	absent                                             bool
	cancel                                             context.CancelFunc
	events                                             []string
	opens                                              int
}

func (b *memoryExecutionReopenStore) executionJournalNames(ctx context.Context, client string) ([]string, error) {
	return append([]string(nil), b.names...), errors.Join(ctx.Err(), b.enumerateFault)
}
func (b *memoryExecutionReopenStore) openExecutionJournal(ctx context.Context, client, id string) (executionJournalReadBackend, []byte, []byte, error) {
	b.opens++
	pair := b.pairs[id]
	if b.absent {
		return nil, nil, nil, b.openFault
	}
	if pair == nil {
		return nil, nil, nil, testFault
	}
	r := &memoryExecutionReader{pair: pair, fault: b.readerFault, closeFault: b.closeFault, cancel: b.cancel, events: &b.events}
	b.readers = append(b.readers, r)
	return r, bytes.Clone(pair.fence), bytes.Clone(pair.raw), errors.Join(ctx.Err(), b.openFault)
}
func (b *memoryExecutionReopenStore) close() error {
	b.events = append(b.events, "store-close")
	return b.memoryStore.close()
}
func executionReopenFixture(t *testing.T) (*Store, *memoryExecutionReopenStore) {
	t.Helper()
	old, _, writer, persisted := beginExecutionFixture(t)
	for _, kind := range []ExecutionFactKind{ExecutionPreparation, ExecutionSpawnIntent, ExecutionOperationFailure} {
		if err := writer.AppendFact(context.Background(), kind, []byte("original fact\x00\n\xff"+string(kind))); err != nil {
			t.Fatal(err)
		}
	}
	if err := old.Close(); err != nil {
		t.Fatal(err)
	}
	pair := &memoryExecutionReadPair{bytes.Clone(persisted.next.fence), bytes.Clone(persisted.next.raw)}
	b := &memoryExecutionReopenStore{memoryStore: &memoryStore{}, pairs: map[string]*memoryExecutionReadPair{stageA: pair}, names: []string{registrationTestSID + ".execution." + stageA + ".once", registrationTestSID + ".execution." + stageA + ".journal"}}
	s := &Store{backend: b}
	t.Cleanup(func() { s.Close() })
	return s, b
}
func TestExecutionJournalReopenAfterRestartPreservesOrderedOriginalFacts(t *testing.T) {
	s, b := executionReopenFixture(t)
	reader, err := s.OpenExecutionJournal(context.Background(), registrationTestSID, stageA)
	if err != nil {
		t.Fatal(err)
	}
	first, err := reader.Snapshot()
	if err != nil {
		t.Fatal(err)
	}
	if first.ClientSID != registrationTestSID || first.ExecutionID != stageA || first.GenerationPurpose != "signed-candidate-generation" || first.FactCount != 3 || len(first.Facts) != 3 || !bytes.Equal(first.OriginalFence, b.pairs[stageA].fence) || !bytes.Equal(first.OriginalJournal, b.pairs[stageA].raw) {
		t.Fatal("original provenance/chain was reconstructed")
	}
	for i, kind := range []ExecutionFactKind{ExecutionPreparation, ExecutionSpawnIntent, ExecutionOperationFailure} {
		if first.Facts[i].Kind != kind || !bytes.Equal(first.Facts[i].Original, []byte("original fact\x00\n\xff"+string(kind))) {
			t.Fatal("original fact order/bytes changed")
		}
	}
	first.OriginalFence[0] ^= 1
	first.OriginalJournal[0] ^= 1
	first.OriginalCompletion[0] ^= 1
	first.Facts[0].Original[0] ^= 1
	first.Facts[0].Kind = ExecutionHealthObserved
	second, err := reader.Snapshot()
	if err != nil || second.Facts[0].Kind != ExecutionPreparation || !bytes.Equal(second.OriginalJournal, b.pairs[stageA].raw) {
		t.Fatal("snapshot mutation escaped informational copies", err)
	}
	copy := *reader
	if err := copy.Close(); err != nil {
		t.Fatal(err)
	}
	if err := reader.Validate(); !errors.Is(err, ErrClosed) {
		t.Fatal("copied capability extended revocation", err)
	}
	if err := reader.Close(); err != nil || b.readers[0].closes != 1 {
		t.Fatal("cleanup repeated", err)
	}
}
func TestExecutionJournalReopenRefusesForgedPartialAndForkedOriginals(t *testing.T) {
	for _, name := range []string{"empty-fence", "empty-journal", "missing-newline", "unknown-origin", "other-id", "other-sid", "wrong-completion", "forked-record", "foreign-owner-error", "nil-owner", "native-validation", "late-cancel"} {
		t.Run(name, func(t *testing.T) {
			s, b := executionReopenFixture(t)
			pair := b.pairs[stageA]
			switch name {
			case "empty-fence":
				pair.fence = nil
			case "empty-journal":
				pair.raw = nil
			case "missing-newline":
				pair.raw = pair.raw[:len(pair.raw)-1]
			case "unknown-origin":
				pair.fence = append(pair.fence, byte(' '))
				pair.raw = append(append(bytes.Clone(pair.fence), '\n'), pair.raw...)
			case "other-id", "other-sid", "wrong-completion":
				var origin executionJournalOrigin
				json.Unmarshal(pair.fence, &origin)
				if name == "other-id" {
					origin.ExecutionID = strings.Repeat("b", 32)
				} else if name == "other-sid" {
					origin.ClientSID = "S-1-5-21-123-456-789-1002"
				} else {
					origin.OriginalCompletion = []byte("replaced")
				}
				pair.fence, _ = json.Marshal(origin)
				pair.raw = append(bytes.Clone(pair.fence), '\n')
			case "forked-record":
				pair.raw = bytes.Replace(pair.raw, []byte("spawn-intent"), []byte("child-created"), 1)
			case "foreign-owner-error":
				b.openFault = testFault
				b.closeFault = errors.New("retained close error")
			case "nil-owner":
				b.absent = true
			case "native-validation":
				b.readerFault = testFault
			case "late-cancel":
				ctx, cancel := context.WithCancel(context.Background())
				b.cancel = cancel
				r, err := s.OpenExecutionJournal(ctx, registrationTestSID, stageA)
				if r != nil || !errors.Is(err, context.Canceled) || !errors.Is(err, ErrExecutionJournalUncertain) {
					t.Fatal("late cancellation minted capability", err)
				}
				if b.readers[0].closes != 1 {
					t.Fatal("late cancellation leaked reader")
				}
				return
			}
			fence, raw := bytes.Clone(pair.fence), bytes.Clone(pair.raw)
			r, err := s.OpenExecutionJournal(context.Background(), registrationTestSID, stageA)
			if r != nil || !errors.Is(err, ErrExecutionJournalUncertain) {
				t.Fatal("uncertain originals minted custody", err)
			}
			if !bytes.Equal(pair.fence, fence) || !bytes.Equal(pair.raw, raw) {
				t.Fatal("failed reopening repaired originals")
			}
			for _, owner := range b.readers {
				if owner.closes != 1 {
					t.Fatal("partial native owner leaked")
				}
			}
			if name == "foreign-owner-error" && !errors.Is(err, b.closeFault) {
				t.Fatal("close evidence discarded")
			}
		})
	}
}
func TestExecutionJournalReaderMutationAndCancellationPermanentlyRevoke(t *testing.T) {
	for _, name := range []string{"bytes", "native", "context", "close-error"} {
		t.Run(name, func(t *testing.T) {
			s, b := executionReopenFixture(t)
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			r, err := s.OpenExecutionJournal(ctx, registrationTestSID, stageA)
			if err != nil {
				t.Fatal(err)
			}
			owner := b.readers[0]
			switch name {
			case "bytes":
				b.pairs[stageA].raw[len(b.pairs[stageA].raw)-2] ^= 1
			case "native":
				owner.fault = testFault
			case "context":
				cancel()
			case "close-error":
				owner.fault = testFault
				owner.closeFault = errors.New("retained read close error")
			}
			if err := r.Validate(); !errors.Is(err, ErrExecutionJournalUncertain) {
				t.Fatal("mutation/cancellation survived", err)
			}
			first, second := r.Close(), r.Close()
			if first != second || owner.closes != 1 {
				t.Fatal("cleanup uncertainty not retained")
			}
			if name == "close-error" && !errors.Is(first, owner.closeFault) {
				t.Fatal("close error lost")
			}
		})
	}
}
func TestExecutionJournalEnumerationRetainsEveryIncompleteAttempt(t *testing.T) {
	s, b := executionReopenFixture(t)
	other := "S-1-5-21-123-456-789-1002"
	b.names = append(b.names,
		registrationTestSID+".execution."+strings.Repeat("b", 32)+".once",
		registrationTestSID+".execution."+strings.Repeat("c", 32)+".journal",
		registrationTestSID+".execution."+strings.Repeat("d", 32)+".once",
		registrationTestSID+".execution."+strings.Repeat("d", 32)+".journal",
		strings.ToLower(registrationTestSID)+".execution."+strings.Repeat("e", 32)+".once",
		registrationTestSID+".execution."+strings.Repeat("F", 32)+".once",
		registrationTestSID+".execution."+strings.Repeat("f", 32)+".journal.extra",
		other+".execution."+strings.Repeat("f", 32)+".once",
		registrationTestSID+".json")
	b.pairs[strings.Repeat("d", 32)] = &memoryExecutionReadPair{}
	entries, err := s.EnumerateExecutionJournals(context.Background(), registrationTestSID)
	var uncertain *ExecutionJournalNamespaceError
	if !errors.As(err, &uncertain) || !errors.Is(err, ErrExecutionJournalUncertain) || len(entries) != 7 || len(uncertain.Entries) != 7 {
		t.Fatal("partial namespace silently ignored", len(entries), err)
	}
	valid := 0
	for _, entry := range entries {
		if entry.Readable {
			valid++
			if entry.ExecutionID != stageA {
				t.Fatal("invalid attempt called readable")
			}
		} else if entry.Problem == "" {
			t.Fatal("partial has no refusal")
		}
	}
	if valid != 1 {
		t.Fatal("valid history discarded alongside partial attempts")
	}
	entries[0].EntryNames[0] = "caller mutation"
	if uncertain.Entries[0].EntryNames[0] == "caller mutation" {
		t.Fatal("error evidence shared mutable names")
	}
	if len(s.journalReaders) != 0 {
		t.Fatal("enumeration retained hidden owners")
	}
	for _, r := range b.readers {
		if r.closes != 1 {
			t.Fatal("enumeration leaked temporary custody")
		}
	}
}
func TestExecutionJournalPairNamesRejectAliasesForksAndMissingSiblings(t *testing.T) {
	prefix := registrationTestSID + ".execution." + stageA
	for _, name := range []string{"missing", "duplicate", "case-alias", "suffix-fork", "extra-journal", "other-attempt"} {
		t.Run(name, func(t *testing.T) {
			names := []string{prefix + ".once", prefix + ".journal"}
			switch name {
			case "missing":
				names = names[:1]
			case "duplicate":
				names = append(names, prefix+".once")
			case "case-alias":
				names[0] = strings.ToLower(registrationTestSID) + ".execution." + stageA + ".once"
			case "suffix-fork":
				names = append(names, prefix+".fork")
			case "extra-journal":
				names = append(names, prefix+".journal.extra")
			case "other-attempt":
				names = append(names, registrationTestSID+".execution."+strings.Repeat("b", 32)+".once")
			}
			err := executionJournalPairNames(registrationTestSID, stageA, names)
			if (name == "other-attempt") != (err == nil) {
				t.Fatal("exact same-attempt namespace rule failed", err)
			}
		})
	}
}
func TestExecutionJournalReaderStoreCloseAndConcurrentCopiesDrainOnce(t *testing.T) {
	s, b := executionReopenFixture(t)
	r, err := s.OpenExecutionJournal(context.Background(), registrationTestSID, stageA)
	if err != nil {
		t.Fatal(err)
	}
	var wg sync.WaitGroup
	for i := 0; i < 8; i++ {
		wg.Add(1)
		go func() { defer wg.Done(); copy := *r; copy.Validate(); copy.Close() }()
	}
	wg.Add(1)
	go func() { defer wg.Done(); s.Close() }()
	wg.Wait()
	if b.readers[0].closes != 1 || b.memoryStore.closes != 1 {
		t.Fatal("concurrent cleanup repeated")
	}
	if len(b.events) != 2 || b.events[0] != "reader-close" || b.events[1] != "store-close" {
		t.Fatal("native roots closed before reader", b.events)
	}
	if err := r.Validate(); !errors.Is(err, ErrClosed) {
		t.Fatal("Store.Close left live reader")
	}
}
func TestExecutionJournalReopenInvalidInputsNeverTouchNative(t *testing.T) {
	s, b := executionReopenFixture(t)
	ctx, cancel := context.WithCancel(context.Background())
	cancel()
	for _, test := range []struct {
		ctx     context.Context
		sid, id string
	}{{nil, registrationTestSID, stageA}, {context.Background(), "S-1-5-18", stageA}, {context.Background(), strings.ToLower(registrationTestSID), stageA}, {context.Background(), registrationTestSID, strings.Repeat("A", 32)}, {ctx, registrationTestSID, stageA}} {
		if r, err := s.OpenExecutionJournal(test.ctx, test.sid, test.id); r != nil || err == nil {
			t.Fatal("invalid input accepted")
		}
	}
	if b.opens != 0 {
		t.Fatal("invalid declaration reached native opener")
	}
	zero := &ExecutionJournalReader{}
	if zero.Validate() == nil || zero.Close() != nil {
		t.Fatal("zero owner provides authority")
	}
	if _, err := json.Marshal(zero); err == nil || json.Unmarshal([]byte("{}"), zero) == nil {
		t.Fatal("serialized capability accepted")
	}
	plain, _ := testStore()
	if _, err := plain.OpenExecutionJournal(context.Background(), registrationTestSID, stageA); !errors.Is(err, ErrNativeRequired) {
		t.Fatal("portable authority fallback", err)
	}
}

func TestExecutionJournalReopenCleanReadsDoNotExhaustRetainedOwnerBound(t *testing.T) {
	s, b := executionReopenFixture(t)
	for i := 0; i <= maxExecutionJournals; i++ {
		reader, err := s.OpenExecutionJournal(context.Background(), registrationTestSID, stageA)
		if err != nil {
			t.Fatal("clean informational read exhausted capacity", i, err)
		}
		if err := reader.Close(); err != nil {
			t.Fatal(err)
		}
	}
	if len(s.journalReaders) > 1 {
		t.Fatal("closed clean capabilities accumulated")
	}
	for _, reader := range b.readers {
		if reader.closes != 1 {
			t.Fatal("pruning repeated native cleanup")
		}
	}
}
