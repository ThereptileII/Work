package brokerstore

import (
	"bytes"
	"context"
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"errors"
	"io"
	"strings"

	"golang.org/x/sys/windows"
)

type nativeExecutionProfileBackup struct {
	pair       *nativeExecutionJournal
	marker     []byte
	size       int64
	digest     string
	readonly   bool
	completed  bool
	client, id string
}

func executionProfileBackupComponent(client, id string) string { return client + ".profile." + id }
func (n *nativeStore) createExecutionProfileBackup(ctx context.Context, client, id string) (executionProfileBackupBackend, error) {
	if err := n.check(); err != nil {
		return nil, err
	}
	if _, err := clientSID(client, n.broker); err != nil {
		return nil, err
	}
	return createNativeExecutionProfileBackup(ctx, n, client, id)
}
func (n *nativeStore) openExecutionProfileBackup(ctx context.Context, client, id string) (executionProfileBackupBackend, []byte, error) {
	if err := n.check(); err != nil {
		return nil, nil, err
	}
	if _, err := clientSID(client, n.broker); err != nil {
		return nil, nil, err
	}
	return openNativeExecutionProfileBackup(ctx, n, client, id)
}
func profileBackupNames(ctx context.Context, n *nativeStore, client, id string, fresh bool) error {
	names, err := nativeExecutionJournalNames(ctx, n)
	if err != nil {
		return err
	}
	count := 0
	for _, name := range names {
		if strings.Contains(strings.ToLower(name), ".profile.") {
			count++
		}
	}
	if err := executionProfileBackupPairNames(client, id, names, fresh); err != nil {
		return err
	}
	if fresh && count+2 > 2*maxExecutionJournals {
		return errors.New("permanent profile backup capacity exhausted")
	}
	return nil
}
func profileBackupPair(n *nativeStore) (*nativeExecutionJournal, error) {
	j := &nativeExecutionJournal{store: n, ids: executionJournalIDs{Parents: map[string]diskIdentity{}}, parents: map[string]*nativePin{}}
	for _, pin := range n.pins {
		id, err := fileIdentity(pin.handle)
		if err != nil {
			return nil, err
		}
		j.ids.Parents[pin.name] = id
		j.parents[pin.name] = pin
	}
	return j, nil
}

// The private lower constructor permits temporary current-user native fixtures;
// production always enters through actual broker principal and real SID checks.
func createNativeExecutionProfileBackup(ctx context.Context, n *nativeStore, client, id string) (result executionProfileBackupBackend, resultErr error) {
	n.beginMu.Lock()
	defer n.beginMu.Unlock()
	if ctx == nil || !validHex(id, 32) {
		return nil, errors.New("live bounded execution identity required")
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if err := n.check(); err != nil {
		return nil, err
	}
	if err := profileBackupNames(ctx, n, client, id, true); err != nil {
		return nil, err
	}
	sd, err := descriptor(n.broker, nil, false)
	if err != nil {
		return nil, err
	}
	pair, err := profileBackupPair(n)
	if err != nil {
		return nil, err
	}
	b := &nativeExecutionProfileBackup{pair: pair, client: client, id: id}
	mutating := false
	defer func() {
		if resultErr != nil {
			resultErr = errors.Join(resultErr, b.close())
			if mutating {
				resultErr = errors.Join(ErrExecutionProfileBackupUncertain, resultErr)
			}
		}
	}()
	component := executionProfileBackupComponent(client, id)
	if err = ctx.Err(); err != nil {
		return nil, err
	}
	mutating = true
	pair.fence, err = createRegistrationFile(n.registrations, component+".once", sd)
	if err != nil {
		return nil, err
	}
	if err = pair.fence.Sync(); err != nil {
		return nil, err
	}
	if err = executionReadback(pair.fence, nil); err != nil {
		return nil, err
	}
	pair.ids.Fence, err = fileIdentity(pair.fence.pin.handle)
	if err != nil {
		return nil, err
	}
	if err = ctx.Err(); err != nil {
		return nil, err
	}
	pair.journal, err = createRegistrationFile(n.registrations, component+".backup", sd)
	if err != nil {
		return nil, err
	}
	pair.ids.Journal, err = fileIdentity(pair.journal.pin.handle)
	if err != nil || !pair.ids.valid() {
		return nil, errors.Join(errors.New("original backup identities unavailable"), err)
	}
	return b, nil
}
func openNativeExecutionProfileBackup(ctx context.Context, n *nativeStore, client, id string) (result executionProfileBackupBackend, marker []byte, resultErr error) {
	n.beginMu.Lock()
	defer n.beginMu.Unlock()
	if ctx == nil || !validHex(id, 32) {
		return nil, nil, errors.New("live bounded execution identity required")
	}
	if err := ctx.Err(); err != nil {
		return nil, nil, err
	}
	if err := n.check(); err != nil {
		return nil, nil, err
	}
	if err := profileBackupNames(ctx, n, client, id, false); err != nil {
		return nil, nil, err
	}
	sd, err := descriptor(n.broker, nil, false)
	if err != nil {
		return nil, nil, err
	}
	pair, err := profileBackupPair(n)
	if err != nil {
		return nil, nil, err
	}
	b := &nativeExecutionProfileBackup{pair: pair, readonly: true, completed: true, client: client, id: id}
	defer func() {
		if resultErr != nil {
			resultErr = errors.Join(ErrExecutionProfileBackupUncertain, resultErr, b.close())
		}
	}()
	component := executionProfileBackupComponent(client, id)
	pair.fence, err = openAcquisitionFile(n.registrations, component+".once", sd, windows.GENERIC_READ|windows.READ_CONTROL, windows.FILE_SHARE_READ, windows.OPEN_EXISTING)
	if err != nil {
		return nil, nil, err
	}
	pair.ids.Fence, err = fileIdentity(pair.fence.pin.handle)
	if err != nil {
		return nil, nil, err
	}
	length := int64(uint64(pair.fence.pin.info.FileSizeHigh)<<32 | uint64(pair.fence.pin.info.FileSizeLow))
	if length <= 0 || length > maxProfileBackupMarkerBytes {
		return nil, nil, errors.New("profile backup permanent marker partial or outside bound")
	}
	marker, err = io.ReadAll(io.LimitReader(pair.fence, length+1))
	if err != nil || int64(len(marker)) != length {
		return nil, nil, errors.Join(errors.New("original backup marker read failed"), err)
	}
	// Only the exact self-contained original fence is used to parse storage here;
	// the public opener separately requires equality to the retained journal.
	var origin executionProfileBackupOrigin
	first := bytes.IndexByte(marker, '\n')
	if first < 0 {
		return nil, nil, errors.New("profile backup completion absent")
	}
	if err = json.Unmarshal(marker[:first], &origin); err != nil {
		return nil, nil, err
	}
	_, completion, err := decodeExecutionProfileBackup(marker, origin.OriginalJournalFence)
	if err != nil {
		return nil, nil, err
	}
	pair.journal, err = openAcquisitionFile(n.registrations, component+".backup", sd, windows.GENERIC_READ|windows.READ_CONTROL, windows.FILE_SHARE_READ, windows.OPEN_EXISTING)
	if err != nil {
		return nil, nil, err
	}
	pair.ids.Journal, err = fileIdentity(pair.journal.pin.handle)
	if err != nil {
		return nil, nil, err
	}
	if !sameExecutionJournalIDs(pair.ids, origin.Identities) {
		return nil, nil, errors.New("backup original full file/parent identities changed")
	}
	b.marker = bytes.Clone(marker)
	b.size = completion.Bytes
	b.digest = completion.SHA256
	if err = b.validate(marker, b.size, b.digest); err != nil {
		return nil, nil, err
	}
	if err = ctx.Err(); err != nil {
		return nil, nil, err
	}
	return b, marker, nil
}
func (b *nativeExecutionProfileBackup) identities() (executionJournalIDs, error) {
	return b.pair.identities()
}
func (b *nativeExecutionProfileBackup) initialize(ctx context.Context, marker []byte) error {
	if b.readonly {
		return ErrClosed
	}
	if err := b.pair.checkIdentities(); err != nil {
		return err
	}
	if err := executionWrite(ctx, b.pair.fence, nil, marker); err != nil {
		return err
	}
	b.marker = bytes.Clone(marker)
	return b.validate(marker, 0, "")
}
func profileArchiveHash(file *nativeFile, size int64, digest string) error {
	if err := file.check(size); err != nil {
		return err
	}
	if _, err := file.Seek(0, io.SeekStart); err != nil {
		return err
	}
	hash := sha256.New()
	count, err := io.CopyBuffer(hash, io.LimitReader(file, size+1), make([]byte, 64<<10))
	if err != nil || count != size || hex.EncodeToString(hash.Sum(nil)) != digest {
		return errors.Join(errors.New("original profile archive length/hash changed"), err)
	}
	return file.check(size)
}
func (b *nativeExecutionProfileBackup) write(ctx context.Context, source io.Reader, size int64, digest string) error {
	if b.readonly || b.digest != "" {
		return ErrClosed
	}
	if err := b.validate(b.marker, 0, ""); err != nil {
		return err
	}
	if _, err := b.pair.journal.Seek(0, io.SeekStart); err != nil {
		return err
	}
	hash := sha256.New()
	buf := make([]byte, 64<<10)
	left := size
	for left > 0 {
		if err := ctx.Err(); err != nil {
			return err
		}
		limit := int64(len(buf))
		if left < limit {
			limit = left
		}
		count, err := io.ReadFull(source, buf[:int(limit)])
		if err != nil {
			return err
		}
		written, err := b.pair.journal.Write(buf[:count])
		if err != nil {
			return err
		}
		if written != count {
			return io.ErrShortWrite
		}
		_, _ = hash.Write(buf[:count])
		left -= int64(count)
	}
	var extra [1]byte
	count, err := io.ReadFull(source, extra[:])
	if count != 0 || (err != io.EOF) {
		return errors.Join(errors.New("profile archive source exceeded declared length"), err)
	}
	if hex.EncodeToString(hash.Sum(nil)) != digest {
		return errors.New("profile archive source hash differs")
	}
	if err = b.pair.journal.Sync(); err != nil {
		return err
	}
	if err = profileArchiveHash(b.pair.journal, size, digest); err != nil {
		return err
	}
	b.size = size
	b.digest = digest
	return ctx.Err()
}
func (b *nativeExecutionProfileBackup) complete(ctx context.Context, previous, next []byte, size int64, digest string) error {
	if b.readonly {
		return ErrClosed
	}
	if err := b.validate(previous, size, digest); err != nil {
		return err
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	// Irrevocable publication: final successful Sync/readback cannot be reported
	// as noncommit merely because the operation context was cancelled meanwhile.
	if err := executionWrite(context.WithoutCancel(ctx), b.pair.fence, previous, next); err != nil {
		return err
	}
	b.marker = bytes.Clone(next)
	b.completed = true
	return b.validate(next, size, digest)
}
func (b *nativeExecutionProfileBackup) metadata(marker []byte, size int64) error {
	if err := profileBackupNames(context.Background(), b.pair.store, b.client, b.id, false); err != nil {
		return err
	}
	if err := b.pair.checkIdentities(); err != nil {
		return err
	}
	if !bytes.Equal(marker, b.marker) {
		return errors.New("original profile backup marker changed")
	}
	if err := executionReadback(b.pair.fence, marker); err != nil {
		return err
	}
	return b.pair.journal.check(size)
}
func (b *nativeExecutionProfileBackup) validate(marker []byte, size int64, digest string) error {
	if err := b.metadata(marker, size); err != nil {
		return err
	}
	if digest == "" {
		if size != 0 {
			return errors.New("profile archive digest absent")
		}
		return nil
	}
	return profileArchiveHash(b.pair.journal, size, digest)
}
func (b *nativeExecutionProfileBackup) readAt(dst []byte, offset int64) (int, error) {
	if !b.completed {
		return 0, ErrClosed
	}
	if err := b.metadata(b.marker, b.size); err != nil {
		return 0, err
	}
	n, err := b.pair.journal.ReadAt(dst, offset)
	return n, errors.Join(err, b.metadata(b.marker, b.size))
}
func (b *nativeExecutionProfileBackup) close() error { return b.pair.close() }
