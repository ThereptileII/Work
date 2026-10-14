package brokerstore

import (
	"bytes"
	"context"
	"encoding/json"
	"strings"
	"testing"
)

type repairRecoveryMemoryStore struct {
	*memoryExecutionStore
	pairs map[string]*memoryExecutionReadPair
}

func (b *repairRecoveryMemoryStore) executionJournalNames(context.Context, string) ([]string, error) {
	names := []string{}
	for id := range b.pairs {
		base := registrationTestSID + ".execution." + id
		names = append(names, base+".once", base+".journal")
	}
	return names, nil
}
func (b *repairRecoveryMemoryStore) openExecutionJournal(_ context.Context, _, id string) (executionJournalReadBackend, []byte, []byte, error) {
	pair := b.pairs[id]
	if pair == nil {
		return nil, nil, nil, testFault
	}
	return &memoryExecutionReader{pair: pair}, bytes.Clone(pair.fence), bytes.Clone(pair.raw), nil
}
func recoveryJournalFixture(t *testing.T) (*Store, *ReadOnlyGeneration, *repairRecoveryMemoryStore) {
	t.Helper()
	s, g, b := executionJournalFixture(t)
	// Isolate original-reader binding with the existing readonly generation seam.
	// Actual Repair copying/physical identity is covered separately, including NTFS.
	original := bytes.Clone(g.generation.raw)
	g.generation.record.Purpose = string(GenerationPurposeRepaired)
	g.generation.record.Repair = &RepairOrigin{SourceCompletion: original, SourceCompletionSHA256: registrationDigest(original), PreviousGeneration: g.generation.record.Generation, PreviousCompletionSHA256: registrationDigest(original), PreviousSelectedHeadSHA256: strings.Repeat("5", 64)}
	g.generation.record.Generation = strings.Repeat("4", 32)
	raw, err := json.Marshal(g.generation.record)
	if err != nil {
		t.Fatal(err)
	}
	g.generation.raw = raw
	backend := &repairRecoveryMemoryStore{b, map[string]*memoryExecutionReadPair{}}
	s.backend = backend
	return s, g, backend
}
func TestRepairRecoveryJournalBornLinkSurvivesZeroFactsAndFurtherContinuation(t *testing.T) {
	s, g, b := recoveryJournalFixture(t)
	ctx := context.Background()
	prior, err := s.BeginRepairJournal(ctx, g, strings.Repeat("6", 32))
	if err != nil {
		t.Fatal(err)
	}
	if err = prior.AppendFact(ctx, ExecutionPreparation, []byte("inert original preparation, no launch or health")); err != nil {
		t.Fatal(err)
	}
	first, err := prior.Snapshot()
	if err != nil {
		t.Fatal(err)
	}
	if err = prior.Close(); err != nil {
		t.Fatal(err)
	}
	b.pairs[first.ExecutionID] = &memoryExecutionReadPair{first.OriginalFence, first.OriginalJournal}
	reader, err := s.OpenExecutionJournal(ctx, registrationTestSID, first.ExecutionID)
	if err != nil {
		t.Fatal(err)
	}
	b.next = newMemoryExecutionJournal()
	next, err := s.BeginRepairRecoveryJournal(ctx, g, strings.Repeat("7", 32), reader)
	if err != nil {
		t.Fatal(err)
	}
	born, err := next.Snapshot()
	if err != nil || born.FactCount != 0 {
		t.Fatal("not actual pre-Preparation boundary", err)
	}
	if err = next.Close(); err != nil {
		t.Fatal(err)
	}
	b.pairs[born.ExecutionID] = &memoryExecutionReadPair{born.OriginalFence, born.OriginalJournal}
	empty, err := s.OpenExecutionJournal(ctx, registrationTestSID, born.ExecutionID)
	if err != nil {
		t.Fatal(err)
	}
	observed, err := empty.Snapshot()
	if err != nil || observed.FactCount != 0 || observed.RepairRecovery == nil || observed.RepairRecovery.PriorExecutionID != first.ExecutionID || observed.RepairRecovery.PriorJournalSHA256 != first.JournalSHA256 {
		t.Fatal("durable empty continuation lost exact prior link", err)
	}
	b.next = newMemoryExecutionJournal()
	last, err := s.BeginRepairRecoveryJournal(ctx, g, strings.Repeat("8", 32), empty)
	if err != nil {
		t.Fatal(err)
	}
	final, err := last.Snapshot()
	if err != nil || final.FactCount != 0 {
		t.Fatal(err)
	}
	var origin executionJournalOrigin
	if json.Unmarshal(final.OriginalFence, &origin) != nil || origin.RepairRecovery == nil || origin.RepairRecovery.PriorExecutionID != born.ExecutionID || origin.RepairRecovery.PriorJournalSHA256 != born.JournalSHA256 {
		t.Fatal("second empty continuation not exactly linked")
	}
	if !bytes.Equal(b.pairs[first.ExecutionID].raw, first.OriginalJournal) || !bytes.Equal(b.pairs[born.ExecutionID].raw, born.OriginalJournal) {
		t.Fatal("old journal was reopened for append")
	}
}
func TestRepairRecoveryJournalRejectsUnownedClosedOrChangedPriorBeforeReservation(t *testing.T) {
	for _, fault := range []string{"nil", "other-store", "closed", "original-bytes"} {
		t.Run(fault, func(t *testing.T) {
			s, g, b := recoveryJournalFixture(t)
			ctx := context.Background()
			prior, err := s.BeginRepairJournal(ctx, g, strings.Repeat("6", 32))
			if err != nil {
				t.Fatal(err)
			}
			snapshot, err := prior.Snapshot()
			if err != nil {
				t.Fatal(err)
			}
			prior.Close()
			b.pairs[snapshot.ExecutionID] = &memoryExecutionReadPair{snapshot.OriginalFence, snapshot.OriginalJournal}
			reader, err := s.OpenExecutionJournal(ctx, registrationTestSID, snapshot.ExecutionID)
			if err != nil {
				t.Fatal(err)
			}
			switch fault {
			case "nil":
				reader = nil
			case "other-store":
				reader = &ExecutionJournalReader{state: &executionJournalReaderState{store: &Store{}}}
			case "closed":
				reader.Close()
			case "original-bytes":
				b.pairs[snapshot.ExecutionID].raw[0] ^= 1
			}
			before := b.prepares
			if _, err = s.BeginRepairRecoveryJournal(ctx, g, strings.Repeat("7", 32), reader); err == nil || b.prepares != before {
				t.Fatal("invalid prior reserved a recovery journal")
			}
		})
	}
}
