package brokerstore

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"io"
	"strings"
	"testing"
)

type memoryProfileBackup struct {
	validateCancel, readCancel context.CancelFunc
	ids                        executionJournalIDs
	marker, data               []byte
	fault                      string
	events                     []string
	closes                     int
	closeFault                 error
	readFault                  error
	cancel                     context.CancelFunc
}

func (b *memoryProfileBackup) step(name string) error {
	b.events = append(b.events, name)
	if b.fault == name {
		return testFault
	}
	return nil
}
func (b *memoryProfileBackup) identities() (executionJournalIDs, error) {
	return b.ids, b.step("identities")
}
func (b *memoryProfileBackup) initialize(ctx context.Context, marker []byte) error {
	b.marker = bytes.Clone(marker)
	for _, event := range []string{"started-write", "started-sync", "started-readback"} {
		if err := b.step(event); err != nil {
			return err
		}
	}
	return ctx.Err()
}
func (b *memoryProfileBackup) write(ctx context.Context, source io.Reader, size int64, digest string) error {
	var err error
	b.data, err = io.ReadAll(io.LimitReader(source, size+1))
	if err != nil {
		return err
	}
	for _, event := range []string{"archive-write", "archive-sync", "archive-readback"} {
		if err := b.step(event); err != nil {
			return err
		}
	}
	if int64(len(b.data)) != size || registrationDigest(b.data) != digest {
		return testFault
	}
	return ctx.Err()
}
func (b *memoryProfileBackup) complete(ctx context.Context, previous, next []byte, size int64, digest string) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	if err := b.validate(previous, size, digest); err != nil {
		return err
	}
	b.marker = bytes.Clone(next)
	for _, event := range []string{"completed-write", "completed-sync", "completed-readback"} {
		if err := b.step(event); err != nil {
			return err
		}
	}
	if b.cancel != nil {
		b.cancel()
	}
	return nil
}
func (b *memoryProfileBackup) validate(marker []byte, size int64, digest string) error {
	if b.validateCancel != nil {
		b.validateCancel()
	}
	if b.fault == "validate" || !bytes.Equal(marker, b.marker) || int64(len(b.data)) != size || (digest != "" && registrationDigest(b.data) != digest) {
		return testFault
	}
	return nil
}
func (b *memoryProfileBackup) readAt(dst []byte, offset int64) (int, error) {
	if b.readCancel != nil {
		b.readCancel()
	}
	n, err := bytes.NewReader(b.data).ReadAt(dst, offset)
	if b.readFault != nil {
		return n, errors.Join(err, b.readFault)
	}
	return n, err
}
func (b *memoryProfileBackup) close() error {
	b.closes++
	b.events = append(b.events, "close")
	return b.closeFault
}

type memoryProfileStore struct {
	storeBackend
	backup         *memoryProfileBackup
	reserved       bool
	creates, opens int
}

func (b *memoryProfileStore) createExecutionProfileBackup(ctx context.Context, client, id string) (executionProfileBackupBackend, error) {
	b.creates++
	if b.reserved {
		return nil, testFault
	}
	b.reserved = true
	return b.backup, ctx.Err()
}
func (b *memoryProfileStore) openExecutionProfileBackup(ctx context.Context, client, id string) (executionProfileBackupBackend, []byte, error) {
	b.opens++
	return b.backup, bytes.Clone(b.backup.marker), ctx.Err()
}
func profileBackupFixture(t *testing.T) (*Store, *ExecutionJournal, *memoryProfileStore) {
	t.Helper()
	s, _, j, _ := beginExecutionFixture(t)
	b := &memoryProfileStore{storeBackend: s.backend, backup: &memoryProfileBackup{ids: executionJournalIDs{Parents: map[string]diskIdentity{"protected-parent": baselineTestID(910)}, Fence: baselineTestID(911), Journal: baselineTestID(912)}}}
	s.backend = b
	return s, j, b
}
func beginProfileFixture(t *testing.T) (*Store, *ExecutionJournal, *ExecutionProfileBackup, *memoryProfileStore) {
	t.Helper()
	s, j, b := profileBackupFixture(t)
	p, err := s.BeginExecutionProfileBackupFresh(context.Background(), j)
	if err != nil {
		t.Fatal(err)
	}
	return s, j, p, b
}
func completeProfileFixture(t *testing.T) (*Store, *ExecutionJournal, *ExecutionProfileBackup, *memoryProfileStore, []byte) {
	t.Helper()
	s, j, p, b := beginProfileFixture(t)
	raw := []byte("opaque original entire profile archive\x00\xff\n")
	if err := p.Write(context.Background(), bytes.NewReader(raw), int64(len(raw)), registrationDigest(raw)); err != nil {
		t.Fatal(err)
	}
	if err := p.Complete(context.Background()); err != nil {
		t.Fatal(err)
	}
	return s, j, p, b, raw
}
func profileReaderFixture(t *testing.T) (*Store, *ExecutionProfileBackupReader, *memoryProfileStore, []byte) {
	t.Helper()
	_, j, p, b, raw := completeProfileFixture(t)
	if err := p.Close(); err != nil {
		t.Fatal(err)
	}
	// A restarted store opens actual original journal storage. Private fakes mint
	// no exported capability and never appear in production factories.
	pair := &memoryExecutionReadPair{bytes.Clone(j.fence), bytes.Clone(j.raw)}
	jb := &memoryExecutionReopenStore{memoryStore: &memoryStore{}, pairs: map[string]*memoryExecutionReadPair{stageA: pair}}
	reopened := &memoryProfileStore{storeBackend: jb, backup: &memoryProfileBackup{ids: b.backup.ids, marker: bytes.Clone(b.backup.marker), data: bytes.Clone(raw)}}
	s := &Store{backend: reopened}
	r, err := s.OpenExecutionJournal(context.Background(), registrationTestSID, stageA)
	if err != nil {
		t.Fatal(err)
	}
	reader, err := s.OpenExecutionProfileBackup(context.Background(), r)
	if err != nil {
		t.Fatal(err)
	}
	return s, reader, reopened, raw
}
func TestExecutionProfileBackupStreamsOriginalsAndReopensReadOnly(t *testing.T) {
	s, j, p, b, raw := completeProfileFixture(t)
	snapshot, err := p.Snapshot(context.Background())
	if err != nil {
		t.Fatal(err)
	}
	if snapshot.Generation != j.origin.Generation || snapshot.CompletionSHA256 != j.origin.CompletionSHA256 || snapshot.JournalFenceSHA256 != registrationDigest(j.fence) || snapshot.SHA256 != registrationDigest(raw) {
		t.Fatal("original lineage or archive changed")
	}
	snapshot.OriginalMarker[0] ^= 1
	if err = p.Validate(context.Background()); err != nil {
		t.Fatal("snapshot mutated owner", err)
	}
	if err = j.AppendFact(context.Background(), ExecutionPreparation, []byte("later native host fact")); err != nil {
		t.Fatal(err)
	}
	if err = p.Validate(context.Background()); err != nil {
		t.Fatal("later journal append invalidated original fence binding", err)
	}
	if got := strings.Join(b.backup.events, ","); got != "identities,started-write,started-sync,started-readback,archive-write,archive-sync,archive-readback,completed-write,completed-sync,completed-readback" {
		t.Fatal(got)
	}
	if err = s.Close(); err != nil {
		t.Fatal(err)
	}
	if b.backup.closes != 1 {
		t.Fatal("store failed to drain backup exactly once")
	}
	s, r, reopened, want := profileReaderFixture(t)
	defer s.Close()
	got := make([]byte, len(want)+1)
	n, err := r.ReadAt(context.Background(), got, 0)
	if err != io.EOF || !bytes.Equal(got[:n], want) {
		t.Fatal("original read-only archive bytes changed", err)
	}
	if err = r.Validate(context.Background()); err != nil || reopened.backup.closes != 0 {
		t.Fatal("pure EOF revoked read custody", err)
	}
	copy := *r
	if err = copy.Close(); err != nil {
		t.Fatal(err)
	}
	if err = r.Validate(context.Background()); !errors.Is(err, ErrClosed) {
		t.Fatal("copied capability extended lifetime", err)
	}
	if _, err = json.Marshal(r); !errors.Is(err, ErrNativeRequired) {
		t.Fatal("serialized capability accepted", err)
	}
}
func TestExecutionProfileBackupEOFWithIntegrityFailureDrainsOnce(t *testing.T) {
	s, r, b, raw := profileReaderFixture(t)
	b.backup.readFault = testFault
	n, err := r.ReadAt(context.Background(), make([]byte, len(raw)+1), 0)
	if n != len(raw) || !errors.Is(err, io.EOF) || !errors.Is(err, testFault) || !errors.Is(err, ErrExecutionProfileBackupUncertain) {
		t.Fatal("EOF hid the post-read integrity failure", n, err)
	}
	if b.backup.closes != 1 {
		t.Fatal("integrity failure did not drain exactly once", b.backup.closes)
	}
	if _, err = r.ReadAt(context.Background(), make([]byte, 1), 0); !errors.Is(err, ErrClosed) {
		t.Fatal("read retried a revoked owner", err)
	}
	if err = r.Validate(context.Background()); !errors.Is(err, ErrClosed) {
		t.Fatal("integrity failure left owner valid", err)
	}
	first := r.Close()
	if !errors.Is(first, testFault) || !errors.Is(first, ErrExecutionProfileBackupUncertain) || r.Close() != first {
		t.Fatal("integrity failure cleanup was not cached", first)
	}
	if err = s.Close(); !errors.Is(err, testFault) || !errors.Is(err, ErrExecutionProfileBackupUncertain) || b.backup.closes != 1 {
		t.Fatal("store cleanup lost the failure or retried close", err, b.backup.closes)
	}
}
func TestExecutionProfileBackupPreconditionsRefuseBeforeReservation(t *testing.T) {
	for _, fault := range []string{"nil-context", "cancel", "nil-journal", "foreign-journal", "closed-journal", "capacity", "unsupported"} {
		t.Run(fault, func(t *testing.T) {
			s, j, b := profileBackupFixture(t)
			ctx := context.Background()
			switch fault {
			case "nil-context":
				ctx = nil
			case "cancel":
				var cancel context.CancelFunc
				ctx, cancel = context.WithCancel(ctx)
				cancel()
			case "nil-journal":
				j = nil
			case "foreign-journal":
				j.store = &Store{}
			case "closed-journal":
				j.closed = true
			case "capacity":
				s.profileBackups = make([]*executionProfileBackupState, maxExecutionJournals)
			case "unsupported":
				s.backend = b.storeBackend
			}
			if p, err := s.BeginExecutionProfileBackupFresh(ctx, j); err == nil || p != nil || b.creates != 0 {
				t.Fatal("invalid owner mutated namespace", err)
			}
		})
	}
}
func TestExecutionProfileBackupPartialAndCommitFailuresRemainPermanent(t *testing.T) {
	for _, fault := range []string{"identities", "started-write", "started-sync", "started-readback", "archive-write", "archive-sync", "archive-readback", "completed-write", "completed-sync", "completed-readback"} {
		t.Run(fault, func(t *testing.T) {
			s, j, b := profileBackupFixture(t)
			b.backup.fault = fault
			p, err := s.BeginExecutionProfileBackupFresh(context.Background(), j)
			raw := []byte("originals")
			if err == nil {
				err = p.Write(context.Background(), bytes.NewReader(raw), int64(len(raw)), registrationDigest(raw))
			}
			if err == nil {
				err = p.Complete(context.Background())
			}
			if !errors.Is(err, ErrExecutionProfileBackupUncertain) || !errors.Is(err, testFault) || b.backup.closes != 1 {
				t.Fatal("uncertainty or once-cleanup lost", err, b.backup.closes)
			}
			if retry, err := s.BeginExecutionProfileBackupFresh(context.Background(), j); err == nil || retry != nil {
				t.Fatal("partial reservation retried")
			}
			// A completed-write/check error may leave perfectly valid completion bytes;
			// error never promises that a later protected reader must reject them.
			if strings.HasPrefix(fault, "completed-") {
				if _, _, err := decodeExecutionProfileBackup(b.backup.marker, j.fence); err != nil {
					t.Fatal("uncertain publication discarded durable completion", err)
				}
			}
		})
	}
}
func TestExecutionProfileBackupCancellationCommitPointAndCleanup(t *testing.T) {
	for _, late := range []bool{false, true} {
		t.Run(map[bool]string{false: "before-publication", true: "after-publication"}[late], func(t *testing.T) {
			_, _, p, b := beginProfileFixture(t)
			raw := []byte("original")
			if err := p.Write(context.Background(), bytes.NewReader(raw), int64(len(raw)), registrationDigest(raw)); err != nil {
				t.Fatal(err)
			}
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			if late {
				b.backup.cancel = cancel
			} else {
				cancel()
			}
			err := p.Complete(ctx)
			if late {
				if err != nil || !p.state.completed || p.state.failure != nil {
					t.Fatal("durable commit poisoned by late cancel", err)
				}
				if err = p.Close(); err != nil {
					t.Fatal(err)
				}
			} else {
				if !errors.Is(err, context.Canceled) || p.state.completed || bytes.Contains(b.backup.marker, []byte("completed-execution-profile-backup")) {
					t.Fatal("precommit cancellation published", err)
				}
			}
		})
	}
	_, _, p, b, _ := completeProfileFixture(t)
	b.backup.closeFault = testFault
	first := p.Close()
	if !errors.Is(first, ErrExecutionProfileBackupUncertain) || !errors.Is(first, testFault) || p.Close() != first || b.backup.closes != 1 {
		t.Fatal("cleanup uncertainty not cached", first)
	}
}
func TestExecutionProfileBackupRejectsArchiveAndReopenedMutations(t *testing.T) {
	for _, fault := range []string{"short", "extra", "digest", "second-write", "cancel", "nil-reader", "oversize"} {
		t.Run(fault, func(t *testing.T) {
			_, _, p, _ := beginProfileFixture(t)
			raw := []byte("original")
			var input io.Reader = bytes.NewReader(raw)
			size := int64(len(raw))
			digest := registrationDigest(raw)
			ctx := context.Background()
			switch fault {
			case "short":
				size++
			case "extra":
				size--
			case "digest":
				digest = strings.Repeat("0", 64)
			case "second-write":
				if err := p.Write(ctx, input, size, digest); err != nil {
					t.Fatal(err)
				}
				input = bytes.NewReader(raw)
			case "cancel":
				var cancel context.CancelFunc
				ctx, cancel = context.WithCancel(ctx)
				cancel()
			case "nil-reader":
				input = nil
			case "oversize":
				size = MaxPayloadBytes + 1
			}
			if err := p.Write(ctx, input, size, digest); !errors.Is(err, ErrExecutionProfileBackupUncertain) {
				t.Fatal("invalid archive accepted", err)
			}
		})
	}
	for _, fault := range []string{"bytes", "marker", "journal", "close"} {
		t.Run("reopen-"+fault, func(t *testing.T) {
			_, r, b, _ := profileReaderFixture(t)
			switch fault {
			case "bytes":
				b.backup.data[0] ^= 1
			case "marker":
				b.backup.marker[0] ^= 1
			case "journal":
				r.state.reader.state.closed = true
			case "close":
				if err := r.Close(); err != nil {
					t.Fatal(err)
				}
			}
			if err := r.Validate(context.Background()); err == nil {
				t.Fatal("revoked or mutated read custody accepted")
			}
		})
	}
}
func TestExecutionProfileBackupClosedSchemaAndNamespaceIsolation(t *testing.T) {
	_, j, p, _, _ := completeProfileFixture(t)
	for _, fault := range []string{"partial", "alias", "duplicate", "unknown", "wrong-fence", "wrong-hash", "wrong-id"} {
		t.Run(fault, func(t *testing.T) {
			raw := bytes.Clone(p.state.marker)
			fence := j.fence
			switch fault {
			case "partial":
				raw = raw[:bytes.IndexByte(raw, '\n')+1]
			case "alias":
				raw = bytes.Replace(raw, []byte(`"schema"`), []byte(`"Schema"`), 1)
			case "duplicate":
				raw = bytes.Replace(raw, []byte(`"schema":1`), []byte(`"schema":1,"schema":1`), 1)
			case "unknown":
				raw = bytes.Replace(raw, []byte(`"schema":1`), []byte(`"schema":1,"unknown":1`), 1)
			case "wrong-fence":
				fence = []byte("forged")
			case "wrong-hash":
				raw = bytes.Replace(raw, []byte(`"originSha256":"`), []byte(`"originSha256":"0`), 1)
			case "wrong-id":
				raw = bytes.Replace(raw, []byte(`"fileId":"`), []byte(`"fileId":"0`), 1)
				if bytes.Equal(raw, p.state.marker) {
					raw = []byte("wrong native identity")
				}
			}
			if _, _, err := decodeExecutionProfileBackup(raw, fence); err == nil {
				t.Fatal("noncanonical backup accepted")
			}
		})
	}
	prefix := registrationTestSID + ".profile." + stageA
	names := []string{prefix + ".once", prefix + ".backup"}
	if err := executionProfileBackupPairNames(registrationTestSID, stageA, names, false); err != nil {
		t.Fatal(err)
	}
	if got := executionJournalEntries(registrationTestSID, names); len(got) != 0 {
		t.Fatal("backup reinterpreted as execution journal")
	}
	for _, extra := range []string{prefix + ".extra", strings.ToUpper(prefix) + ".once", prefix} {
		if err := executionProfileBackupPairNames(registrationTestSID, stageA, append(names, extra), false); err == nil {
			t.Fatal("unknown backup member adopted", extra)
		}
	}
}

func (b *memoryProfileStore) openExecutionJournal(ctx context.Context, client, id string) (executionJournalReadBackend, []byte, []byte, error) {
	return b.storeBackend.(executionJournalReopenStoreBackend).openExecutionJournal(ctx, client, id)
}
func (b *memoryProfileStore) executionJournalNames(ctx context.Context, client string) ([]string, error) {
	return b.storeBackend.(executionJournalReopenStoreBackend).executionJournalNames(ctx, client)
}

func TestExecutionProfileBackupRevocationDuringValidationAndRead(t *testing.T) {
	t.Run("original-context-during-validation", func(t *testing.T) {
		s, j, b := profileBackupFixture(t)
		ctx, cancel := context.WithCancel(context.Background())
		defer cancel()
		p, err := s.BeginExecutionProfileBackupFresh(ctx, j)
		if err != nil {
			t.Fatal(err)
		}
		raw := []byte("original")
		if err = p.Write(ctx, bytes.NewReader(raw), int64(len(raw)), registrationDigest(raw)); err != nil {
			t.Fatal(err)
		}
		if err = p.Complete(ctx); err != nil {
			t.Fatal(err)
		}
		b.backup.validateCancel = cancel
		if _, err = p.Snapshot(context.Background()); !errors.Is(err, context.Canceled) || !errors.Is(err, ErrExecutionProfileBackupUncertain) {
			t.Fatal("revoked original context retained authority", err)
		}
	})
	t.Run("original-context-during-read", func(t *testing.T) {
		_, r, b, raw := profileReaderFixture(t)
		ctx, cancel := context.WithCancel(context.Background())
		defer cancel()
		r.state.owner = ctx
		b.backup.readCancel = cancel
		if _, err := r.ReadAt(context.Background(), make([]byte, len(raw)), 0); !errors.Is(err, context.Canceled) || !errors.Is(err, ErrExecutionProfileBackupUncertain) {
			t.Fatal("read returned authority after original revocation", err)
		}
	})
}
