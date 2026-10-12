package brokerhost

import (
	"bytes"
	"context"
	"crypto/sha256"
	"encoding/binary"
	"encoding/hex"
	"encoding/json"
	"errors"
	"io"
	"strconv"
	"strings"
	"unicode/utf8"

	"example.com/opennav-update-verifier/brokerstore"
)

// OpenExecutionProfileCaptureNative reopens only the completed original archive
// belonging to a live same-Store journal reader. The caller retains that reader.
// It grants cold native inspection, not historical release, child-termination,
// writable restoration, health or selection authority. In particular a consumed
// attempt cannot recover its original native child handle from persisted bytes.
func OpenExecutionProfileCaptureNative(ctx context.Context, store *brokerstore.Store, journal *brokerstore.ExecutionJournalReader) (*ExecutionProfileCapture, error) {
	if ctx == nil || store == nil || journal == nil {
		return nil, ErrExecutionProfileCaptureRefused
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	original, err := journal.Snapshot()
	if err != nil {
		return nil, err
	}
	archive, err := store.OpenExecutionProfileBackup(ctx, journal)
	if err != nil {
		return nil, err
	}
	return openProfileCaptureArchive(ctx, archive, original.ExecutionJournalSnapshot, reopenProfileSourceNative)
}

// This private wrapper deliberately does not embed its source or implement
// profileRestoreFactory. No reopened archive can reacquire writable custody or
// manufacture the original child authority needed by restoreOriginalNative.
type reopenedProfileSource struct {
	original profileCaptureManifest
	native   profileCaptureSource
}

func (s *reopenedProfileSource) originals() ([]profileCaptureEntry, []string) {
	return append([]profileCaptureEntry(nil), s.original.Entries...), append([]string(nil), s.original.Ancestors...)
}
func (*reopenedProfileSource) readers() ([]io.Reader, error) {
	return nil, ErrExecutionProfileCaptureRefused
}
func (s *reopenedProfileSource) validate(ctx context.Context, frozen bool) error {
	if frozen || s.native == nil {
		return ErrExecutionProfileCaptureRefused
	}
	return s.native.validate(ctx, false)
}
func (*reopenedProfileSource) release(context.Context) error {
	return ErrExecutionProfileCaptureRefused
}
func (s *reopenedProfileSource) inspect(ctx context.Context) (profileCaptureSource, error) {
	return s.native.inspect(ctx)
}
func (s *reopenedProfileSource) close() error { return s.native.close() }

func bindReopenedProfileSource(ctx context.Context, m profileCaptureManifest, native profileCaptureSource) (profileCaptureSource, error) {
	_, ancestors := native.originals()
	if len(ancestors) != len(m.Ancestors) {
		return nil, errors.Join(ErrExecutionProfileCaptureRefused, native.close())
	}
	for i := range ancestors {
		if ancestors[i] != m.Ancestors[i] {
			return nil, errors.Join(ErrExecutionProfileCaptureRefused, native.close())
		}
	}
	if err := native.release(ctx); err != nil {
		return nil, errors.Join(err, native.close())
	}
	return &reopenedProfileSource{original: m, native: native}, nil
}

func profileArchiveReadExact(ctx context.Context, archive profileCaptureArchive, dst []byte, offset int64) error {
	for len(dst) > 0 {
		if err := ctx.Err(); err != nil {
			return err
		}
		chunk := len(dst)
		if chunk > 64<<10 {
			chunk = 64 << 10
		}
		n, err := archive.ReadAt(ctx, dst[:chunk], offset)
		if n != chunk {
			return errors.Join(ErrExecutionProfileCaptureRefused, io.ErrUnexpectedEOF, err)
		}
		if err != nil {
			return err
		}
		dst = dst[chunk:]
		offset += int64(chunk)
	}
	return ctx.Err()
}
func parseProfileCaptureID(raw string) (executionProfileID, error) {
	var id executionProfileID
	if len(raw) != 66 || raw[16] != ' ' || raw[49] != ' ' {
		return id, ErrExecutionProfileCaptureRefused
	}
	var err error
	id.volume, err = strconv.ParseUint(raw[:16], 16, 64)
	if err != nil {
		return id, ErrExecutionProfileCaptureRefused
	}
	file, err := hex.DecodeString(raw[17:49])
	if err != nil {
		return id, ErrExecutionProfileCaptureRefused
	}
	copy(id.file[:], file)
	id.created, err = strconv.ParseUint(raw[50:], 16, 64)
	if err != nil || id.volume == 0 || id.file == [16]byte{} || id.created == 0 || id.wire() != raw {
		return id, ErrExecutionProfileCaptureRefused
	}
	return id, nil
}

func validateProfileArchiveManifest(m profileCaptureManifest, raw []byte, journal brokerstore.ExecutionJournalSnapshot, snapshot brokerstore.ExecutionProfileBackupSnapshot) (int64, error) {
	if m.Schema != 1 || m.Purpose != "protected-opencpn-profile-capture-v1" || m.ClientSID != journal.ClientSID || m.ExecutionID != journal.ExecutionID || m.Generation != journal.Generation || m.CompletionSHA256 != journal.CompletionSHA256 || snapshot.ClientSID != journal.ClientSID || snapshot.ExecutionID != journal.ExecutionID || snapshot.Generation != journal.Generation || snapshot.CompletionSHA256 != journal.CompletionSHA256 || len(journal.OriginalFence) == 0 || snapshot.JournalFenceSHA256 != captureDigest(journal.OriginalFence) {
		return 0, ErrExecutionProfileCaptureRefused
	}
	// Reuse the producer's bounded path/parent/hash rules; exact encoding also
	// rejects duplicate/unknown JSON fields, aliases, unsorted entries and whitespace.
	source := &reopenedProfileSource{original: m}
	_, canonical, total, err := profileManifest(journal, source)
	if err != nil || !bytes.Equal(raw, canonical) || total != snapshot.Bytes {
		return 0, errors.Join(ErrExecutionProfileCaptureRefused, err)
	}
	var volume uint64
	parents := map[executionProfileID]bool{}
	for _, wire := range m.Ancestors {
		id, err := parseProfileCaptureID(wire)
		if err != nil || parents[id] || (volume != 0 && id.volume != volume) {
			return 0, ErrExecutionProfileCaptureRefused
		}
		volume = id.volume
		parents[id] = true
	}
	members := map[[16]byte]bool{}
	const ordinaryAttributes = uint32(1 | 2 | 4 | 16 | 32 | 128 | 8192)
	for i, e := range m.Entries {
		id, err := parseProfileCaptureID(e.Identity)
		if err != nil || id.volume != volume || id.created != e.Created || members[id.file] || e.Attributes & ^ordinaryAttributes != 0 || (e.Attributes&16 != 0) != e.Directory || len(e.Security) > 64<<10 || !utf8.ValidString(e.Security) || strings.ContainsRune(e.Security, 0) || !utf8.ValidString(e.Path) {
			return 0, ErrExecutionProfileCaptureRefused
		}
		members[id.file] = true
		if i == 0 && (e.Path != "." || e.Identity != m.Ancestors[len(m.Ancestors)-1]) {
			return 0, ErrExecutionProfileCaptureRefused
		}
	}
	return int64(len(raw)) + 8, nil
}

func openProfileCaptureArchive(ctx context.Context, archive profileCaptureArchive, journal brokerstore.ExecutionJournalSnapshot, sourceFactory func(context.Context, profileCaptureManifest) (profileCaptureSource, error)) (result *ExecutionProfileCapture, resultErr error) {
	var source profileCaptureSource
	defer func() {
		if resultErr != nil {
			if source != nil {
				resultErr = errors.Join(resultErr, source.close())
			}
			resultErr = errors.Join(ErrExecutionProfileCaptureRefused, resultErr, archive.Close())
		}
	}()
	if ctx == nil || sourceFactory == nil {
		return nil, ErrExecutionProfileCaptureRefused
	}
	snapshot, err := archive.Snapshot(ctx)
	if err != nil {
		return nil, err
	}
	if snapshot.Bytes < 8 || snapshot.Bytes > brokerstore.MaxPayloadBytes {
		return nil, ErrExecutionProfileCaptureRefused
	}
	var header [8]byte
	if err = profileArchiveReadExact(ctx, archive, header[:], 0); err != nil {
		return nil, err
	}
	length := binary.BigEndian.Uint64(header[:])
	if length == 0 || length > maxProfileManifestBytes || length > uint64(snapshot.Bytes-8) {
		return nil, ErrExecutionProfileCaptureRefused
	}
	raw := make([]byte, int(length))
	if err = profileArchiveReadExact(ctx, archive, raw, 8); err != nil {
		return nil, err
	}
	var manifest profileCaptureManifest
	if err = json.Unmarshal(raw, &manifest); err != nil {
		return nil, err
	}
	offset, err := validateProfileArchiveManifest(manifest, raw, journal, snapshot)
	if err != nil {
		return nil, err
	}
	buffer := make([]byte, 64<<10)
	for _, entry := range manifest.Entries {
		if entry.Directory {
			continue
		}
		hash := sha256.New()
		remaining := entry.Bytes
		for remaining > 0 {
			amount := int64(len(buffer))
			if remaining < amount {
				amount = remaining
			}
			if err = profileArchiveReadExact(ctx, archive, buffer[:int(amount)], offset); err != nil {
				return nil, err
			}
			_, _ = hash.Write(buffer[:int(amount)])
			offset += amount
			remaining -= amount
		}
		if hex.EncodeToString(hash.Sum(nil)) != entry.SHA256 {
			return nil, ErrExecutionProfileCaptureRefused
		}
	}
	if offset != snapshot.Bytes {
		return nil, ErrExecutionProfileCaptureRefused
	}
	if err = archive.Validate(ctx); err != nil {
		return nil, err
	}
	source, err = sourceFactory(ctx, manifest)
	if err != nil {
		return nil, err
	}
	p := &profileCaptureState{owner: ctx, source: source, archive: archive, manifest: manifest, original: bytes.Clone(raw), released: true, reopened: true}
	if err = p.ready(ctx); err != nil {
		return nil, err
	}
	return &ExecutionProfileCapture{state: p}, nil
}
