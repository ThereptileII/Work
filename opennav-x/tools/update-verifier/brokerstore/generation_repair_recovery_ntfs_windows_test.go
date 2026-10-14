//go:build windows

package brokerstore

import (
	"context"
	"runtime"
	"strings"
	"testing"
)

// Only existing temporary principal/root factories differ. Both birth and read
// use Store's real production journal body and actual protected NTFS pairs.
// This is filesystem custody evidence, not installed/UAC/process qualification.
type nativeRepairRecoveryNTFSStore struct{ *nativeRepairNTFSStore }

func (b *nativeRepairRecoveryNTFSStore) prepareExecutionJournal(ctx context.Context, client, id string) (executionJournalBackend, error) {
	return createNativeExecutionJournal(ctx, b.nativeStore, client, id)
}
func (b *nativeRepairRecoveryNTFSStore) executionJournalNames(ctx context.Context, _ string) ([]string, error) {
	return nativeExecutionJournalNames(ctx, b.nativeStore)
}
func (b *nativeRepairRecoveryNTFSStore) openExecutionJournal(ctx context.Context, client, id string) (executionJournalReadBackend, []byte, []byte, error) {
	return openNativeExecutionJournal(ctx, b.nativeStore, client, id)
}
func TestNativeRepairRecoveryJournalRetainsBornLinkBeforeFirstPreparation(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	f := repairNTFSFixture(t, GenerationPurposeApprovedBaseline)
	ctx := context.Background()
	source, err := f.store.OpenRepairSource(ctx, f.client, f.generation)
	if err != nil {
		t.Fatal(err)
	}
	copied, err := f.store.MaterializeRepairFresh(ctx, source, strings.Repeat("4", 32), strings.Repeat("5", 64))
	if err != nil {
		t.Fatal(err)
	}
	readonly, err := copied.TransitionReadOnly(ctx)
	if err != nil {
		t.Fatal(err)
	}
	f.store.backend = &nativeRepairRecoveryNTFSStore{f.backend}
	prior, err := f.store.BeginRepairJournal(ctx, readonly, strings.Repeat("6", 32))
	if err != nil {
		t.Fatal(err)
	}
	if err = prior.AppendFact(ctx, ExecutionPreparation, []byte("inert retained preparation; no launch/health assertion")); err != nil {
		t.Fatal(err)
	}
	first, err := prior.Snapshot()
	if err != nil {
		t.Fatal(err)
	}
	if err = prior.Close(); err != nil {
		t.Fatal(err)
	}
	original, err := f.store.OpenExecutionJournal(ctx, f.client, first.ExecutionID)
	if err != nil {
		t.Fatal(err)
	}
	next, err := f.store.BeginRepairRecoveryJournal(ctx, readonly, strings.Repeat("7", 32), original)
	if err != nil {
		t.Fatal(err)
	}
	born, err := next.Snapshot()
	if err != nil || born.FactCount != 0 {
		t.Fatal("not pre-Preparation origin", err)
	}
	// Close/reopen at the precise fully durable boundary, with no preparation.
	if err = next.Close(); err != nil {
		t.Fatal(err)
	}
	empty, err := f.store.OpenExecutionJournal(ctx, f.client, born.ExecutionID)
	if err != nil {
		t.Fatal(err)
	}
	observed, err := empty.Snapshot()
	if err != nil || observed.FactCount != 0 || observed.RepairRecovery == nil || observed.RepairRecovery.PriorExecutionID != first.ExecutionID || observed.RepairRecovery.PriorJournalSHA256 != first.JournalSHA256 {
		t.Fatal("empty original NTFS journal lost born prior link", err)
	}
	last, err := f.store.BeginRepairRecoveryJournal(ctx, readonly, strings.Repeat("8", 32), empty)
	if err != nil {
		t.Fatal(err)
	}
	final, err := last.Snapshot()
	if err != nil || final.FactCount != 0 {
		t.Fatal(err)
	}
	if err = last.Close(); err != nil {
		t.Fatal(err)
	}
	finalReader, err := f.store.OpenExecutionJournal(ctx, f.client, final.ExecutionID)
	if err != nil {
		t.Fatal(err)
	}
	finalOriginal, err := finalReader.Snapshot()
	if err != nil || finalOriginal.RepairRecovery == nil || finalOriginal.RepairRecovery.PriorExecutionID != born.ExecutionID || finalOriginal.RepairRecovery.PriorJournalSHA256 != born.JournalSHA256 {
		t.Fatal("new continuation did not bind exact empty NTFS original", err)
	}
	if err = original.Validate(); err != nil {
		t.Fatal("prior original changed", err)
	}
	if err = empty.Validate(); err != nil {
		t.Fatal("empty original changed", err)
	}
}
