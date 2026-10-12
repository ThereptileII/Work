package brokerstore

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"strings"
	"sync"
	"testing"
)

type memoryExecutionJournal struct {
	ids        executionJournalIDs
	fence, raw []byte
	fault      string
	events     []string
	closes     int
	closeFault error
	cancel     context.CancelFunc
	onClose    func()
}

func (b *memoryExecutionJournal) step(name string) error {
	b.events = append(b.events, name)
	if b.fault == name {
		return testFault
	}
	return nil
}
func (b *memoryExecutionJournal) identities() (executionJournalIDs, error) {
	return b.ids, b.step("identities")
}
func (b *memoryExecutionJournal) initialize(ctx context.Context, fence, raw []byte) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	for _, name := range []string{"fence-write", "fence-flush", "fence-readback", "journal-write", "journal-flush", "journal-readback"} {
		if name == "fence-write" {
			b.fence = bytes.Clone(fence)
		}
		if name == "journal-write" {
			b.raw = bytes.Clone(raw)
		}
		if err := b.step(name); err != nil {
			return err
		}
	}
	if b.cancel != nil {
		b.cancel()
	}
	return ctx.Err()
}
func (b *memoryExecutionJournal) append(ctx context.Context, fence, previous, next []byte) error {
	if err := b.validate(fence, previous); err != nil {
		return err
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	// Retain the partial mutation even if the write failed. No caller can
	// reinterpret a failing return as absence of durable bytes.
	if b.fault == "append-write" {
		b.raw = bytes.Clone(next[:len(next)-1])
		return b.step("append-write")
	}
	b.raw = bytes.Clone(next)
	for _, name := range []string{"append-write", "append-flush", "append-readback"} {
		if err := b.step(name); err != nil {
			return err
		}
	}
	if b.cancel != nil {
		b.cancel()
	}
	return ctx.Err()
}
func (b *memoryExecutionJournal) validate(fence, raw []byte) error {
	if !bytes.Equal(fence, b.fence) || !bytes.Equal(raw, b.raw) {
		return testFault
	}
	return validateExecutionJournalBytes(fence, raw)
}
func (b *memoryExecutionJournal) close() error {
	b.closes++
	if b.onClose != nil {
		b.onClose()
	}
	return b.closeFault
}

type memoryExecutionStore struct {
	storeBackend
	next     *memoryExecutionJournal
	reserved map[string]bool
	prepares int
}

func (b *memoryExecutionStore) prepareExecutionJournal(ctx context.Context, client, id string) (executionJournalBackend, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	b.prepares++
	key := client + ".execution." + id
	if b.reserved[key] {
		return nil, testFault
	}
	b.reserved[key] = true
	return b.next, nil
}

func newMemoryExecutionJournal() *memoryExecutionJournal {
	return &memoryExecutionJournal{ids: executionJournalIDs{Parents: map[string]diskIdentity{"retained-parent": baselineTestID(900)}, Fence: baselineTestID(901), Journal: baselineTestID(902)}}
}

func executionJournalFixture(t *testing.T) (*Store, *ReadOnlyGeneration, *memoryExecutionStore) {
	t.Helper()
	s, g, _ := readOnlyFixture(t)
	r, err := g.TransitionReadOnly(context.Background())
	if err != nil {
		t.Fatal(err)
	}
	b := &memoryExecutionStore{storeBackend: s.backend, next: newMemoryExecutionJournal(), reserved: map[string]bool{}}
	s.backend = b
	return s, r, b
}

func beginExecutionFixture(t *testing.T) (*Store, *ReadOnlyGeneration, *ExecutionJournal, *memoryExecutionStore) {
	t.Helper()
	s, r, b := executionJournalFixture(t)
	j, err := s.BeginExecutionJournal(context.Background(), r, stageA)
	if err != nil {
		t.Fatal(err)
	}
	return s, r, j, b
}

func TestExecutionJournalRetainsExactOriginalsAndFactsWithoutSelection(t *testing.T) {
	_, r, j, b := beginExecutionFixture(t)
	original := []byte("\x00original observation\nwith exact bytes\xff")
	for _, kind := range []ExecutionFactKind{ExecutionPreparation, ExecutionSpawnIntent, ExecutionHealthObserved} {
		if err := j.AppendFact(context.Background(), kind, original); err != nil {
			t.Fatal(err)
		}
	}
	original[0] = 'x'
	snapshot, err := j.Snapshot()
	if err != nil || snapshot.FactCount != 3 || snapshot.Generation != r.generation.record.Generation || snapshot.CompletionSHA256 != registrationDigest(r.generation.raw) || snapshot.JournalSHA256 != registrationDigest(snapshot.OriginalJournal) {
		t.Fatal("original informational snapshot changed", err)
	}
	var origin executionJournalOrigin
	if err = json.Unmarshal(snapshot.OriginalFence, &origin); err != nil || !bytes.Equal(origin.OriginalCompletion, r.generation.raw) {
		t.Fatal("original generation completion lost", err)
	}
	if bytes.Contains(snapshot.OriginalFence, []byte(`"selected"`)) || bytes.Contains(snapshot.OriginalFence, []byte(`"previousGeneration"`)) {
		t.Fatal("journal invented selected or previous authority")
	}
	for _, e := range j.records {
		if e.Original[0] != 0 {
			t.Fatal("caller mutated retained original")
		}
	}
	snapshot.OriginalJournal[0] ^= 1
	snapshot.OriginalFence[0] ^= 1
	if err = j.Validate(); err != nil {
		t.Fatal("snapshot mutated live storage", err)
	}
	if err = j.Close(); err != nil {
		t.Fatal(err)
	}
	if err = j.Close(); err != nil || b.next.closes != 1 {
		t.Fatal("cleanup repeated", err, b.next.closes)
	}
	if _, err = j.Snapshot(); !errors.Is(err, ErrClosed) {
		t.Fatal("closed snapshot retained custody", err)
	}
	if _, err = j.store.BeginExecutionJournal(context.Background(), r, stageA); err == nil {
		t.Fatal("permanent fence retried")
	}
}

func TestExecutionJournalBaselineUsesDistinctLiveQualificationOwner(t *testing.T) {
	s, p, view, _, raw := baselineViewFixture(t)
	g, err := s.MaterializeApprovedBaselineFresh(context.Background(), p, raw, registrationDigest(raw))
	if err != nil {
		t.Fatal(err)
	}
	for name, file := range view.files {
		view.old[name] = file
	}
	r, err := g.TransitionReadOnly(context.Background())
	if err != nil {
		t.Fatal(err)
	}
	b := &memoryExecutionStore{storeBackend: s.backend, next: newMemoryExecutionJournal(), reserved: map[string]bool{}}
	s.backend = b
	if _, err = s.BeginExecutionJournal(context.Background(), r.owner, stageA); err == nil || b.prepares != 0 {
		t.Fatal("approved baseline accepted as signed candidate")
	}
	j, err := s.BeginBaselineQualificationJournal(context.Background(), r, stageA)
	if err != nil || j.origin.GenerationPurpose != approvedBaselineGenerationPurpose {
		t.Fatal("baseline qualification origin lost", err)
	}
	if err = j.AppendFact(context.Background(), ExecutionCommissioning, []byte("observation only")); err != nil {
		t.Fatal(err)
	}
}

func TestExecutionJournalBeginRefusesInvalidOrRevokedOwnersBeforeMutation(t *testing.T) {
	var zero ExecutionJournal
	if err := zero.AppendFact(context.Background(), ExecutionPreparation, []byte("no native owner")); !errors.Is(err, ErrClosed) {
		t.Fatal("empty owner permitted append", err)
	}
	if _, err := zero.Snapshot(); !errors.Is(err, ErrClosed) {
		t.Fatal("empty owner permitted snapshot", err)
	}
	if err := zero.Close(); err != nil {
		t.Fatal("empty close", err)
	}
	for _, fault := range []string{"nil-context", "cancelled", "bad-id", "nil-owner", "foreign-store", "closed-generation", "unsupported", "capacity"} {
		t.Run(fault, func(t *testing.T) {
			s, r, b := executionJournalFixture(t)
			ctx := context.Background()
			id := stageA
			switch fault {
			case "nil-context":
				ctx = nil
			case "cancelled":
				var cancel context.CancelFunc
				ctx, cancel = context.WithCancel(ctx)
				cancel()
			case "bad-id":
				id = strings.ToUpper(stageA)
			case "nil-owner":
				r = nil
			case "foreign-store":
				s = &Store{backend: b}
			case "closed-generation":
				if err := r.Close(); err != nil {
					t.Fatal(err)
				}
			case "unsupported":
				s.backend = b.storeBackend
			case "capacity":
				s.journals = make([]*ExecutionJournal, maxExecutionJournals)
				defer func() { s.journals = nil }()
			}
			if j, err := s.BeginExecutionJournal(ctx, r, id); j != nil || err == nil || b.prepares != 0 || len(b.reserved) != 0 {
				t.Fatal("invalid begin mutated journal", err, b.prepares)
			}
		})
	}
}

func TestExecutionJournalEveryInitializationFailureRetainsFenceAndDrainsOnce(t *testing.T) {
	for _, fault := range []string{"identities", "fence-write", "fence-flush", "fence-readback", "journal-write", "journal-flush", "journal-readback", "cancel-after-write"} {
		t.Run(fault, func(t *testing.T) {
			s, r, b := executionJournalFixture(t)
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			if fault == "cancel-after-write" {
				b.next.cancel = cancel
			} else {
				b.next.fault = fault
			}
			b.next.closeFault = errors.New("original close uncertainty")
			j, err := s.BeginExecutionJournal(ctx, r, stageA)
			if j != nil || !errors.Is(err, ErrExecutionJournalUncertain) || !errors.Is(err, b.next.closeFault) || b.next.closes != 1 || len(b.reserved) != 1 {
				t.Fatal("failed initialization lost reservation/cleanup", err)
			}
			if fault == "cancel-after-write" && !errors.Is(err, context.Canceled) {
				t.Fatal("cancellation identity lost", err)
			}
			if fault != "cancel-after-write" && !errors.Is(err, testFault) {
				t.Fatal("original initialization failure lost", err)
			}
			if _, err = s.BeginExecutionJournal(context.Background(), r, stageA); err == nil || b.next.closes != 1 {
				t.Fatal("failed reservation retried or close repeated", err)
			}
		})
	}
}

func TestExecutionJournalAppendFailurePoisonsDrainsAndPreservesDurablePartial(t *testing.T) {
	for _, fault := range []string{"append-write", "append-flush", "append-readback", "cancel-after-write", "cancel-before", "tampered-fence", "tampered-journal", "generation-changed", "generation-closed", "invalid-kind", "empty-original", "oversize"} {
		t.Run(fault, func(t *testing.T) {
			_, r, j, b := beginExecutionFixture(t)
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			kind, original := ExecutionSpawnIntent, []byte("exact original process-creation request")
			before := bytes.Clone(b.next.raw)
			switch fault {
			case "cancel-after-write":
				b.next.cancel = cancel
			case "cancel-before":
				cancel()
			case "tampered-fence":
				b.next.fence[0] ^= 1
			case "tampered-journal":
				b.next.raw[0] ^= 1
			case "generation-changed":
				r.generation.raw[0] ^= 1
			case "generation-closed":
				if err := r.Close(); err != nil {
					t.Fatal(err)
				}
			case "invalid-kind":
				kind = "healthy-selected"
			case "empty-original":
				original = nil
			case "oversize":
				original = make([]byte, maxExecutionFactBytes+1)
			default:
				b.next.fault = fault
			}
			b.next.closeFault = errors.New("retained failed close")
			err := j.AppendFact(ctx, kind, original)
			if !errors.Is(err, ErrExecutionJournalUncertain) || !errors.Is(err, b.next.closeFault) || b.next.closes != 1 {
				t.Fatal("failed append reported usable custody", err)
			}
			if strings.HasPrefix(fault, "cancel-") && !errors.Is(err, context.Canceled) {
				t.Fatal("append cancellation identity lost", err)
			}
			if strings.HasPrefix(fault, "append-") || fault == "cancel-after-write" {
				if len(b.next.raw) <= len(before) || !bytes.HasPrefix(b.next.raw, before) {
					t.Fatal("partial/durable append erased")
				}
			}
			if err = j.AppendFact(context.Background(), ExecutionTeardown, []byte("retry")); !errors.Is(err, ErrClosed) || b.next.closes != 1 {
				t.Fatal("failed journal retried", err)
			}
			if err = j.Close(); !errors.Is(err, b.next.closeFault) || b.next.closes != 1 {
				t.Fatal("cached failed close became success", err)
			}
		})
	}
}

func TestExecutionJournalStrictCanonicalChainRefusesMalformedOriginals(t *testing.T) {
	_, _, j, _ := beginExecutionFixture(t)
	if err := j.AppendFact(context.Background(), ExecutionHealthObserved, []byte("not an authority")); err != nil {
		t.Fatal(err)
	}
	for _, fault := range []string{"unknown-origin", "duplicate-origin", "origin-hash", "unknown-record", "duplicate-record", "sequence", "previous-hash", "kind", "empty", "missing-newline", "extra-newline", "whitespace"} {
		t.Run(fault, func(t *testing.T) {
			fence, raw := bytes.Clone(j.fence), bytes.Clone(j.raw)
			switch fault {
			case "unknown-origin":
				fence = append([]byte(`{"extra":0,`), fence[1:]...)
			case "duplicate-origin":
				fence = append([]byte(`{"schema":1,`), fence[1:]...)
			case "origin-hash":
				var origin executionJournalOrigin
				json.Unmarshal(fence, &origin)
				origin.CompletionSHA256 = strings.Repeat("a", 64)
				fence, _ = json.Marshal(origin)
			case "missing-newline":
				raw = raw[:len(raw)-1]
			case "extra-newline":
				raw = append(raw, '\n')
			case "whitespace":
				raw = append(raw[:len(j.fence)+1], append([]byte(" "), raw[len(j.fence)+1:]...)...)
			default:
				e := j.records[0]
				switch fault {
				case "sequence":
					e.Sequence++
				case "previous-hash":
					e.PreviousSHA256 = strings.Repeat("a", 64)
				case "kind":
					e.Kind = "healthy-selected"
				case "empty":
					e.Original = nil
				}
				line, _ := json.Marshal(e)
				if fault == "unknown-record" {
					line = append([]byte(`{"extra":0,`), line[1:]...)
				}
				if fault == "duplicate-record" {
					line = append([]byte(`{"schema":1,`), line[1:]...)
				}
				raw = append(append(append(bytes.Clone(fence), '\n'), line...), '\n')
			}
			if strings.Contains(fault, "origin") {
				raw = append(bytes.Clone(fence), '\n')
			}
			if err := validateExecutionJournalBytes(fence, raw); err == nil {
				t.Fatal("malformed chain accepted", fault)
			}
		})
	}
}

func TestExecutionJournalParallelAppendsRemainOneOriginalChain(t *testing.T) {
	_, _, j, _ := beginExecutionFixture(t)
	var wg sync.WaitGroup
	errorsOut := make(chan error, 32)
	for i := 0; i < 32; i++ {
		wg.Add(1)
		go func(i int) {
			defer wg.Done()
			errorsOut <- j.AppendFact(context.Background(), ExecutionPreparation, []byte(fmt.Sprintf("original-%d", i)))
		}(i)
	}
	wg.Wait()
	close(errorsOut)
	for err := range errorsOut {
		if err != nil {
			t.Fatal(err)
		}
	}
	seen := map[string]bool{}
	for i, e := range j.records {
		if e.Sequence != i+1 || seen[string(e.Original)] {
			t.Fatal("append repeated or lost original")
		}
		seen[string(e.Original)] = true
	}
	if len(seen) != 32 || validateExecutionJournalBytes(j.fence, j.raw) != nil {
		t.Fatal("parallel append chain invalid")
	}
}

func TestExecutionJournalBoundsAndStoreCloseDrainBeforeGeneration(t *testing.T) {
	for _, fault := range []string{"facts", "close-failure", "store-close"} {
		t.Run(fault, func(t *testing.T) {
			s, r, j, b := beginExecutionFixture(t)
			b.next.onClose = func() {
				if r.generation.closed {
					t.Error("generation closed before journal drain")
				}
			}
			if fault == "facts" {
				for i := 0; i < maxExecutionFacts; i++ {
					if err := j.AppendFact(context.Background(), ExecutionPreparation, []byte("bounded fact")); err != nil {
						t.Fatal(err)
					}
				}
				if err := j.AppendFact(context.Background(), ExecutionPreparation, []byte("exhausted")); !errors.Is(err, ErrExecutionJournalUncertain) || len(j.records) != maxExecutionFacts {
					t.Fatal("fact count bound relaxed", err)
				}
			} else if fault == "close-failure" {
				b.next.closeFault = testFault
			}
			err := s.Close()
			if fault == "store-close" && err != nil {
				t.Fatal(err)
			}
			if fault != "store-close" && !errors.Is(err, ErrExecutionJournalUncertain) {
				t.Fatal("store lost child cleanup uncertainty", err)
			}
			if b.next.closes != 1 || !r.generation.closed {
				t.Fatal("store did not drain once")
			}
			if retry := s.Close(); (err == nil) != (retry == nil) || b.next.closes != 1 {
				t.Fatal("store close retry changed outcome", retry)
			}
			if err = j.AppendFact(context.Background(), ExecutionTeardown, []byte("after close")); !errors.Is(err, ErrClosed) {
				t.Fatal("closed store allowed append", err)
			}
		})
	}
}

func TestExecutionJournalConcurrentAppendAndStoreCloseDrainOnce(t *testing.T) {
	s, _, j, b := beginExecutionFixture(t)
	start := make(chan struct{})
	errorsOut := make(chan error, 33)
	var wg sync.WaitGroup
	for i := 0; i < 32; i++ {
		wg.Add(1)
		go func(i int) {
			defer wg.Done()
			<-start
			errorsOut <- j.AppendFact(context.Background(), ExecutionPreparation, []byte(fmt.Sprintf("concurrent-%d", i)))
		}(i)
	}
	wg.Add(1)
	go func() {
		defer wg.Done()
		<-start
		errorsOut <- s.Close()
	}()
	close(start)
	wg.Wait()
	close(errorsOut)
	for err := range errorsOut {
		if err != nil && !errors.Is(err, ErrClosed) {
			t.Fatal("concurrent close falsely reported custody uncertainty", err)
		}
	}
	if b.next.closes != 1 || !bytes.Equal(j.raw, b.next.raw) || validateExecutionJournalBytes(b.next.fence, b.next.raw) != nil {
		t.Fatal("concurrent drain lost original chain or repeated cleanup")
	}
}
