package brokerstore

import (
	"bytes"
	"context"
	"encoding/json"
	"strings"
	"testing"
)

func TestInterruptedRepairDiscoveryOnlyAcceptsOneExactOwnedCopy(t *testing.T) {
	for _, fault := range []string{"", "wrong-head", "wrong-source", "wrong-predecessor", "tampered-completion", "duplicate-copy", "already-selected", "wrong-selected-digest"} {
		t.Run(fault, func(t *testing.T) {
			s, b := repairFixture(t, GenerationPurposeSignedCandidate)
			ctx := context.Background()
			source, err := s.OpenRepairSource(ctx, registrationTestSID, b.current.Generation)
			if err != nil {
				t.Fatal(err)
			}
			generation, err := s.MaterializeRepairFresh(ctx, source, strings.Repeat("4", 32), strings.Repeat("5", 64))
			if err != nil {
				t.Fatal(err)
			}
			record := generation.record
			raw := bytes.Clone(generation.raw)
			head := strings.Repeat("5", 64)
			completed := map[string]string{}
			switch fault {
			case "wrong-head":
				head = strings.Repeat("6", 64)
			case "wrong-source":
				record.Repair.SourceCompletion = bytes.Clone(record.Repair.SourceCompletion)
				record.Repair.SourceCompletion[0] ^= 1
				raw, _ = json.Marshal(record)
			case "wrong-predecessor":
				record.Repair.PreviousGeneration = strings.Repeat("6", 32)
				raw, _ = json.Marshal(record)
			case "tampered-completion":
				raw = append(raw, ' ')
			case "already-selected":
				completed[record.Generation] = registrationDigest(raw)
			case "wrong-selected-digest":
				completed[record.Generation] = strings.Repeat("f", 64)
			}
			records := [][]byte{b.originalRaw, raw}
			if fault == "duplicate-copy" {
				var other generationRecord
				json.Unmarshal(raw, &other)
				other.Generation = strings.Repeat("7", 32)
				second, _ := json.Marshal(other)
				records = append(records, second)
			}
			candidate, err := findInterruptedRepairRecord(records, b.current, b.currentRaw, b.originalRaw, head, completed)
			if fault == "" {
				if err != nil || candidate == nil || candidate.Generation != record.Generation || candidate.CompletionSHA256 != registrationDigest(raw) {
					t.Fatal("exact orphan not available for explicit continuation", err)
				}
				return
			}
			if fault == "already-selected" {
				if err != nil || candidate != nil {
					t.Fatal("committed repair misidentified as interrupted", err)
				}
				return
			}
			if err == nil || candidate != nil {
				t.Fatal("ambiguous/corrupt/unrelated copy adopted")
			}
		})
	}
}
func TestInterruptedRepairStillRequiresStrictPublicPayloadReopen(t *testing.T) {
	for _, fault := range []string{"", "corrupt-member", "missing-member", "changed-member-id", "unknown-addition"} {
		t.Run(fault, func(t *testing.T) {
			s, b := repairFixture(t, GenerationPurposeApprovedBaseline)
			ctx := context.Background()
			source, err := s.OpenRepairSource(ctx, registrationTestSID, b.current.Generation)
			if err != nil {
				t.Fatal(err)
			}
			generation, err := s.MaterializeRepairFresh(ctx, source, strings.Repeat("4", 32), strings.Repeat("5", 64))
			if err != nil {
				t.Fatal(err)
			}
			interrupted, err := findInterruptedRepairRecord([][]byte{b.originalRaw, generation.raw}, b.current, b.currentRaw, b.originalRaw, strings.Repeat("5", 64), nil)
			if err != nil {
				t.Fatal(err)
			}
			record, raw := generation.record, bytes.Clone(generation.raw)
			originals := map[string][]byte{}
			for name, f := range b.source.destination.files {
				originals[name] = bytes.Clone(f.data)
			}
			switch fault {
			case "corrupt-member":
				originals["app/opencpn.exe"][0] ^= 1
			case "missing-member":
				delete(originals, "app/opencpn.exe")
			case "unknown-addition":
				originals["app/customer-added.bin"] = []byte("must remain untouched, never adopt")
			case "changed-member-id":
				m := record.Members["app/opencpn.exe"]
				m.Identity = baselineTestID(9999)
				record.Members["app/opencpn.exe"] = m
			}
			if err = generation.Close(); err != nil {
				t.Fatal(err)
			}
			b.memoryReopenStore.b = &memoryReopenedGeneration{memoryGeneration: &memoryGeneration{memoryCandidate: &memoryCandidate{files: map[string]*memoryFile{}}, completed: true}, originals: originals, record: record}
			b.memoryReopenStore.evidence.completion = raw
			b.reopenOverride = &repairReopenCustody{b.memoryReopenStore.b}
			reopened, err := s.OpenReadOnlyGeneration(ctx, registrationTestSID, interrupted.Generation)
			if (fault == "") != (err == nil) {
				t.Fatal("orphan data was adopted without strict complete byte/ID check", err)
			}
			if reopened != nil {
				reopened.Close()
			}
		})
	}
}

type repairReopenCustody struct{ *memoryReopenedGeneration }

func (b *repairReopenCustody) identities(_ map[string]baselineMember) (map[string]diskIdentity, map[string]diskIdentity, error) {
	observed := map[string]diskIdentity{}
	for name, m := range b.record.Members {
		observed[name] = m.Identity
	}
	return b.record.Directories, observed, nil
}
func (b *repairReopenCustody) inventory(names []string, complete bool) error {
	if err := b.memoryReopenedGeneration.inventory(names, complete); err != nil {
		return err
	}
	if len(b.originals) != len(names) {
		return testFault
	}
	for _, name := range names {
		if _, ok := b.originals[name]; !ok {
			return testFault
		}
	}
	return nil
}
