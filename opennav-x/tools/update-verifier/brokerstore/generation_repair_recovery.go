package brokerstore

import (
	"bytes"
	"context"
	"encoding/json"
	"runtime"

	"example.com/opennav-update-verifier/baselineevidence"
	"example.com/opennav-update-verifier/brokerregistration"
)

// RepairCompletionBinding is comparison data parsed from already retained
// protected original bytes. It is never native custody or permission to select.
type RepairCompletionBinding struct {
	ClientSID, Generation, CompletionSHA256, SourceGeneration string
	Identity                                                  baselineevidence.Identity
	Origin                                                    RepairOrigin
}

func DescribeRepairCompletion(raw []byte) (RepairCompletionBinding, error) {
	var r generationRecord
	if len(raw) == 0 || len(raw) > maxGenerationRecordBytes || json.Unmarshal(raw, &r) != nil {
		return RepairCompletionBinding{}, ErrInstalledGenerationUncertain
	}
	canonical, err := json.Marshal(r)
	if err != nil || !bytes.Equal(canonical, raw) || r.Schema != 1 || r.Purpose != string(GenerationPurposeRepaired) || !brokerregistration.IsCanonicalUserSID(r.ClientSID) || !validHex(r.Generation, 32) || len(r.Members) == 0 || len(r.Directories) == 0 || validateRepairOrigin(r) != nil {
		return RepairCompletionBinding{}, ErrInstalledGenerationUncertain
	}
	var original generationRecord
	if json.Unmarshal(r.Repair.SourceCompletion, &original) != nil {
		return RepairCompletionBinding{}, ErrInstalledGenerationUncertain
	}
	origin := *r.Repair
	origin.SourceCompletion = bytes.Clone(origin.SourceCompletion)
	return RepairCompletionBinding{r.ClientSID, r.Generation, registrationDigest(raw), original.Generation, r.Identity, origin}, nil
}

// FindInterruptedRepair only identifies one exact owned completed copy. It
// neither adopts a namespace nor writes/deletes/launches. The host must fully
// reopen this exact generation and author a fresh linked journal before use.
func (r *RepairSource) FindInterruptedRepair(ctx context.Context, selectedHead string, completed map[string]string) (*RepairCompletionBinding, error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if ctx == nil || !validHex(selectedHead, 64) {
		return nil, ErrInstalledGenerationUncertain
	}
	if err := r.Validate(ctx); err != nil {
		return nil, err
	}
	r.store.mu.Lock()
	defer r.store.mu.Unlock()
	r.mu.Lock()
	defer r.mu.Unlock()
	b, ok := r.backend.(interface {
		repairHistoryRecords(context.Context) ([][]byte, error)
	})
	if !ok {
		return nil, ErrNativeRequired
	}
	records, err := b.repairHistoryRecords(ctx)
	if err != nil {
		return nil, err
	}
	return findInterruptedRepairRecord(records, r.current, r.currentRaw, r.originalRaw, selectedHead, completed)
}
func findInterruptedRepairRecord(records [][]byte, current generationRecord, currentRaw, sourceRaw []byte, head string, completed map[string]string) (*RepairCompletionBinding, error) {
	if len(records) > MaxCandidates+1 || len(completed) > 128 || !validHex(head, 64) {
		return nil, ErrInstalledGenerationUncertain
	}
	var found *RepairCompletionBinding
	seen := map[string]bool{}
	for _, raw := range records {
		var probe struct {
			Purpose string `json:"purpose"`
		}
		if json.Unmarshal(raw, &probe) != nil {
			return nil, ErrInstalledGenerationUncertain
		}
		if probe.Purpose != string(GenerationPurposeRepaired) {
			continue
		}
		c, err := DescribeRepairCompletion(raw)
		if err != nil || seen[c.Generation] {
			return nil, ErrInstalledGenerationUncertain
		}
		seen[c.Generation] = true
		if digest, exists := completed[c.Generation]; exists {
			if digest != c.CompletionSHA256 {
				return nil, ErrInstalledGenerationUncertain
			}
			continue
		}
		if found != nil || c.ClientSID != current.ClientSID || c.Identity != current.Identity || c.Generation == current.Generation || c.Origin.PreviousGeneration != current.Generation || c.Origin.PreviousCompletionSHA256 != registrationDigest(currentRaw) || c.Origin.PreviousSelectedHeadSHA256 != head || !bytes.Equal(c.Origin.SourceCompletion, sourceRaw) {
			return nil, ErrInstalledGenerationUncertain
		}
		candidate := c
		found = &candidate
	}
	return found, nil
}

// RepairRecoveryLink is informational original-fence data, not a reader owner.
// The native author can create it only by consuming the retained prior reader.
type RepairRecoveryLink struct {
	PriorExecutionID   string `json:"priorExecutionId"`
	PriorJournalSHA256 string `json:"priorJournalSha256"`
}

// BeginRepairRecoveryJournal binds the actual original reader into the initial
// durable fence BEFORE the first preparation fact. No serialized link can call
// this API, reopen an append writer or authorize recovery/selection by itself.
func (s *Store) BeginRepairRecoveryJournal(ctx context.Context, generation *ReadOnlyGeneration, id string, prior *ExecutionJournalReader) (*ExecutionJournal, error) {
	if prior == nil {
		return nil, ErrNativeRequired
	}
	return s.beginExecutionJournalWithRepairRecovery(ctx, generation, id, string(GenerationPurposeRepaired), prior)
}
