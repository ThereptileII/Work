package brokerstore

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"io"
	"strings"
	"sync"
	"testing"
)

type memoryStageSealStore struct {
	*memoryReopenStore
	writer *memoryStageSeal
	opens  int
}
type memoryStageSeal struct {
	marker, raw, receipt []byte
	origin               stageSealOrigin
	fault                string
	closeFault           error
	closes               int
	events               []string
	candidate            *memoryCandidate
	cancel               context.CancelFunc
	beforeCommitCancel   context.CancelFunc
}

func (m *memoryStageSeal) step(name string) error {
	m.events = append(m.events, name)
	if m.fault == name {
		return testFault
	}
	return nil
}
func (m *memoryStageSealStore) prepareStageSeal(ctx context.Context, _ generationBackend, c candidateBackend, _ generationRecord, raw []byte) (stageSealWriter, error) {
	b := m.writer
	if len(b.marker) > 0 {
		return nil, testFault
	}
	b.candidate = c.(*memoryCandidate)
	b.origin = stageSealOrigin{1, "original-generation-stage-seal", bytes.Clone(raw), executionJournalIDs{Parents: map[string]diskIdentity{"protected": baselineTestID(700)}, Fence: baselineTestID(701), Journal: baselineTestID(702)}}
	origin, _ := json.Marshal(b.origin)
	b.marker = append(origin, '\n')
	for _, name := range []string{"reserve", "origin-sync", "origin-readback"} {
		if err := b.step(name); err != nil {
			return b, err
		}
	}
	return b, ctx.Err()
}
func (b *memoryStageSeal) complete(ctx context.Context, f fileRecord) error {
	if b.candidate.files[ReceiptFilename] != f.file {
		return testFault
	}
	if _, err := f.file.Seek(0, io.SeekStart); err != nil {
		return err
	}
	raw, err := io.ReadAll(f.file)
	if err != nil {
		return err
	}
	if int64(len(raw)) != f.length || registrationDigest(raw) != f.digest {
		return testFault
	}
	b.receipt = bytes.Clone(raw)
	record, _ := json.Marshal(stageSealRecord{1, "completed-generation-stage-seal", registrationDigest(bytes.TrimSuffix(b.marker, []byte{'\n'})), baselineMember{Length: f.length, SHA256: f.digest, Identity: baselineTestID(703)}})
	b.raw = record
	for _, name := range []string{"record-write", "record-sync", "record-readback"} {
		if err = b.step(name); err != nil {
			return err
		}
	}
	if b.beforeCommitCancel != nil {
		b.beforeCommitCancel()
	}
	if err = ctx.Err(); err != nil {
		return err
	}
	b.marker = append(append(bytes.Clone(b.marker), []byte(registrationDigest(record))...), '\n')
	for _, name := range []string{"commit-write", "commit-sync", "commit-readback"} {
		if err = b.step(name); err != nil {
			return err
		}
	}
	if b.cancel != nil {
		b.cancel()
	}
	return nil
}
func (b *memoryStageSeal) close() error { b.closes++; return b.closeFault }
func (m *memoryStageSealStore) openStageSeal(ctx context.Context, _ generationBackend, _ generationRecord, _ []byte) (stageSealReaderBackend, []byte, []byte, []byte, error) {
	m.opens++
	if m.writer.fault == "open" {
		return nil, nil, nil, nil, testFault
	}
	return m.writer, bytes.Clone(m.writer.marker), bytes.Clone(m.writer.raw), bytes.Clone(m.writer.receipt), ctx.Err()
}
func (b *memoryStageSeal) validate(ctx context.Context) error {
	if b.fault == "validate" {
		return testFault
	}
	return ctx.Err()
}
func stageSealFixture(t *testing.T) (*Store, *Candidate, *memoryStageSealStore) {
	t.Helper()
	s, reopen, f := reopenFixture(t, GenerationPurposeSignedCandidate)
	candidate := &Candidate{store: s, backend: &memoryCandidate{files: map[string]*memoryFile{}}, clientSID: registrationTestSID, stageID: reopen.b.record.Generation, names: map[string]string{}, fileNames: map[string]bool{}}
	writeCandidateEvidence(t, candidate, f)
	if err := candidate.CompleteRecoveryEvidence(context.Background(), f.identity); err != nil {
		t.Fatal(err)
	}
	candidate.materialized = true
	s.candidates = append(s.candidates, candidate)
	b := &memoryStageSealStore{memoryReopenStore: reopen, writer: &memoryStageSeal{}}
	s.backend = b
	return s, candidate, b
}
func sealAndReopenFixture(t *testing.T) (*Store, *ReopenedGeneration, *GenerationStageSeal, *memoryStageSealStore) {
	t.Helper()
	s, c, b := stageSealFixture(t)
	if err := c.Seal(context.Background(), []byte("original signed staging receipt\n")); err != nil {
		t.Fatal(err)
	}
	if err := c.Close(); err != nil {
		t.Fatal(err)
	}
	owner, err := s.OpenReadOnlyGeneration(context.Background(), registrationTestSID, c.stageID)
	if err != nil {
		t.Fatal(err)
	}
	reader, err := s.OpenGenerationStageSeal(context.Background(), owner)
	if err != nil {
		t.Fatal(err)
	}
	return s, owner, reader, b
}
func TestGenerationStageSealOriginalReceiptAndLifetime(t *testing.T) {
	s, owner, reader, b := sealAndReopenFixture(t)
	snapshot, err := reader.Snapshot(context.Background())
	if err != nil {
		t.Fatal(err)
	}
	if !bytes.Equal(snapshot.Receipt, b.writer.receipt) || snapshot.ReceiptSHA256 != registrationDigest(b.writer.receipt) || snapshot.SealSHA256 != registrationDigest(b.writer.raw) || snapshot.Completion.Generation != owner.owner.generation.record.Generation {
		t.Fatal("original provenance changed")
	}
	snapshot.Receipt[0] ^= 1
	snapshot.OriginalSeal[0] ^= 1
	if err = reader.Validate(context.Background()); err != nil {
		t.Fatal("copy modified original", err)
	}
	if got := strings.Join(b.writer.events, ","); got != "reserve,origin-sync,origin-readback,record-write,record-sync,record-readback,commit-write,commit-sync,commit-readback" {
		t.Fatal(got)
	}
	if b.writer.closes != 1 {
		t.Fatal("writer not drained once")
	}
	copy := *reader
	if err = copy.Close(); err != nil {
		t.Fatal(err)
	}
	if err = reader.Validate(context.Background()); !errors.Is(err, ErrClosed) {
		t.Fatal("copied view extended lifetime", err)
	}
	if err = owner.Validate(); err != nil {
		t.Fatal("closing view revoked owner", err)
	}
	if _, err = json.Marshal(reader); !errors.Is(err, ErrNativeRequired) {
		t.Fatal("serialized view accepted", err)
	}
	if err = s.Close(); err != nil {
		t.Fatal(err)
	}
}
func TestGenerationStageSealPublicationFailuresNeverRetry(t *testing.T) {
	for _, fault := range []string{"reserve", "origin-sync", "origin-readback", "receipt-create", "record-write", "record-sync", "record-readback", "commit-write", "commit-sync", "commit-readback", "close"} {
		t.Run(fault, func(t *testing.T) {
			s, c, b := stageSealFixture(t)
			if fault == "receipt-create" {
				c.backend.(*memoryCandidate).createFault = testFault
			} else if fault == "close" {
				b.writer.closeFault = testFault
			} else {
				b.writer.fault = fault
			}
			err := c.Seal(context.Background(), []byte("original"))
			if !errors.Is(err, testFault) || !errors.Is(err, ErrGenerationStageSealUncertain) || c.sealed && fault != "close" {
				t.Fatal("failed publication accepted", err)
			}
			if b.writer.closes != 1 {
				t.Fatal("failed writer not drained once", b.writer.closes)
			}
			if err = c.Seal(context.Background(), []byte("retry")); !errors.Is(err, ErrPoisoned) {
				t.Fatal("failed stage retried", err)
			}
			if b.writer.closes != 1 {
				t.Fatal("cleanup retried")
			}
			s.Close()
		})
	}
}
func TestGenerationStageSealReaderRefusesChangedOriginals(t *testing.T) {
	for _, fault := range []string{"missing", "partial", "marker", "record", "receipt", "generation", "identity", "open"} {
		t.Run(fault, func(t *testing.T) {
			s, c, b := stageSealFixture(t)
			if err := c.Seal(context.Background(), []byte("original")); err != nil {
				t.Fatal(err)
			}
			c.Close()
			owner, err := s.OpenReadOnlyGeneration(context.Background(), registrationTestSID, c.stageID)
			if err != nil {
				t.Fatal(err)
			}
			switch fault {
			case "missing":
				b.writer.marker = nil
			case "partial":
				b.writer.marker = bytes.SplitAfterN(b.writer.marker, []byte{'\n'}, 2)[0]
			case "marker":
				b.writer.marker[0] ^= 1
			case "record":
				b.writer.raw[0] ^= 1
			case "receipt":
				b.writer.receipt[0] ^= 1
			case "generation":
				owner.owner.generation.raw[0] ^= 1
			case "identity":
				record := stageSealRecord{}
				json.Unmarshal(b.writer.raw, &record)
				record.Receipt.Identity = diskIdentity{}
				b.writer.raw, _ = json.Marshal(record)
				origin := bytes.SplitN(b.writer.marker, []byte{'\n'}, 2)[0]
				b.writer.marker = append(append(append([]byte{}, origin...), '\n'), []byte(registrationDigest(b.writer.raw)+"\n")...)
			case "open":
				b.writer.fault = "open"
			}
			if _, err = s.OpenGenerationStageSeal(context.Background(), owner); !errors.Is(err, ErrGenerationStageSealUncertain) {
				t.Fatal("changed seal reopened", err)
			}
			if err = owner.Validate(); !errors.Is(err, ErrClosed) {
				t.Fatal("failed acquire did not drain owner", err)
			}
			s.Close()
		})
	}
}
func TestGenerationStageSealRevocationAndParallelClose(t *testing.T) {
	for _, fault := range []string{"context", "validation", "owner", "store", "view"} {
		t.Run(fault, func(t *testing.T) {
			s, owner, reader, b := sealAndReopenFixture(t)
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			switch fault {
			case "context":
				cancel()
			case "validation":
				b.writer.fault = "validate"
			case "owner":
				owner.Close()
			case "store":
				s.Close()
			case "view":
				reader.Close()
			}
			if err := reader.Validate(ctx); err == nil {
				t.Fatal("revoked view accepted")
			}
			var wg sync.WaitGroup
			for range 8 {
				wg.Add(1)
				go func() { defer wg.Done(); reader.Close() }()
			}
			wg.Wait()
			s.Close()
		})
	}
}
func TestGenerationStageSealSchemaNamespaceAndPreconditions(t *testing.T) {
	s, owner, reader, b := sealAndReopenFixture(t)
	defer s.Close()
	for _, fault := range []string{"duplicate", "unknown", "case", "completion", "bound"} {
		marker, raw, completion := bytes.Clone(b.writer.marker), bytes.Clone(b.writer.raw), bytes.Clone(owner.owner.generation.raw)
		switch fault {
		case "duplicate":
			raw = bytes.Replace(raw, []byte(`"schema":1`), []byte(`"schema":1,"schema":1`), 1)
		case "unknown":
			raw = append(raw[:len(raw)-1], []byte(`,"unknown":1}`)...)
		case "case":
			raw = bytes.Replace(raw, []byte(`"schema"`), []byte(`"Schema"`), 1)
		case "completion":
			completion[0] ^= 1
		case "bound":
			raw = make([]byte, maxStageSealRecordBytes+1)
		}
		if _, _, err := decodeStageSeal(marker, raw, completion); err == nil {
			t.Fatal("invalid canonical record", fault)
		}
	}
	prefix := registrationTestSID + ".stage." + owner.owner.generation.record.Generation
	for _, names := range [][]string{{prefix + ".once"}, {prefix + ".once", prefix + ".json", prefix + ".extra"}, {strings.ToUpper(prefix) + ".once", prefix + ".json"}} {
		if stageSealPairNames(registrationTestSID, owner.owner.generation.record.Generation, names, false) == nil {
			t.Fatal("ambiguous seal namespace")
		}
	}
	if err := stageSealPairNames(registrationTestSID, owner.owner.generation.record.Generation, []string{prefix + ".once", prefix + ".json"}, false); err != nil {
		t.Fatal(err)
	}
	for _, bad := range []string{"../escape", "installer/../escape", "/absolute", `app\wrong`, "app/trailing."} {
		if stageSealDirectoryName(bad) == nil {
			t.Fatal("unsafe source parent", bad)
		}
	}
	if _, err := s.OpenGenerationStageSeal(nil, owner); err == nil {
		t.Fatal("nil context accepted")
	}
	if _, err := (&Store{}).OpenGenerationStageSeal(context.Background(), owner); err == nil {
		t.Fatal("foreign store accepted")
	}
	owner.purpose = GenerationPurposeApprovedBaseline
	if _, err := s.OpenGenerationStageSeal(context.Background(), owner); err == nil {
		t.Fatal("baseline accepted")
	}
	owner.purpose = GenerationPurposeSignedCandidate
	reader.Close()
}

func TestGenerationStageSealCancellationCommitPoint(t *testing.T) {
	for _, phase := range []string{"before-receipt-readback", "before-commit", "after-commit"} {
		t.Run(phase, func(t *testing.T) {
			s, c, b := stageSealFixture(t)
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			switch phase {
			case "before-receipt-readback":
				c.backend.(*memoryCandidate).configure = func(file *memoryFile) { file.onRead = cancel }
			case "before-commit":
				b.writer.beforeCommitCancel = cancel
			case "after-commit":
				b.writer.cancel = cancel
			}
			err := c.Seal(ctx, []byte("original receipt"))
			if phase == "after-commit" {
				if err != nil || !c.sealed {
					t.Fatal("successful durable completion lost to late cancellation", err)
				}
			} else {
				if !errors.Is(err, context.Canceled) || !errors.Is(err, ErrGenerationStageSealUncertain) || c.sealed {
					t.Fatal("cancelled partial accepted", err)
				}
				if _, _, err = decodeStageSeal(b.writer.marker, b.writer.raw, b.evidence.completion); err == nil {
					t.Fatal("cancelled partial marked complete")
				}
			}
			if b.writer.closes != 1 {
				t.Fatal("cancelled writer not drained once")
			}
			s.Close()
		})
	}
}

func TestGenerationStageSealConcurrentSnapshotAndOwnerClose(t *testing.T) {
	s, owner, reader, _ := sealAndReopenFixture(t)
	start := make(chan struct{})
	var wg sync.WaitGroup
	for range 8 {
		wg.Add(1)
		go func() {
			defer wg.Done()
			<-start
			snapshot, err := reader.Snapshot(context.Background())
			if err != nil && !errors.Is(err, ErrClosed) {
				t.Error("concurrent custody failure", err)
			}
			if err == nil && !bytes.Equal(snapshot.Receipt, []byte("original signed staging receipt\n")) {
				t.Error("partial snapshot during close")
			}
		}()
	}
	wg.Add(1)
	go func() {
		defer wg.Done()
		<-start
		if err := owner.Close(); err != nil {
			t.Error(err)
		}
	}()
	close(start)
	wg.Wait()
	if err := reader.Validate(context.Background()); !errors.Is(err, ErrClosed) {
		t.Fatal("closed owner revived", err)
	}
	if err := s.Close(); err != nil {
		t.Fatal(err)
	}
}
