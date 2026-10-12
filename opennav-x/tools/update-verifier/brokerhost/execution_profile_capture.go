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
	"sort"
	"strings"
	"sync"

	"example.com/opennav-update-verifier/brokerstore"
)

const maxProfileManifestBytes = 8 << 20

var ErrExecutionProfileCaptureRefused = errors.New("native entire-profile capture custody refused")
var ErrExecutionProfileRestoreReviewRequired = errors.New("profile changes exceed the closed automatic startup restore policy; review required")

type profileCaptureEntry struct {
	Path       string `json:"path"`
	Directory  bool   `json:"directory"`
	Bytes      int64  `json:"bytes"`
	SHA256     string `json:"sha256"`
	Identity   string `json:"identity"`
	Attributes uint32 `json:"attributes"`
	Created    uint64 `json:"created"`
	LastAccess uint64 `json:"lastAccess"`
	LastWrite  uint64 `json:"lastWrite"`
	Security   string `json:"ownerGroupDacl"`
}
type profileCaptureManifest struct {
	Schema           int                   `json:"schema"`
	Purpose          string                `json:"purpose"`
	ClientSID        string                `json:"clientSid"`
	ExecutionID      string                `json:"executionId"`
	Generation       string                `json:"generation"`
	CompletionSHA256 string                `json:"completionSha256"`
	Ancestors        []string              `json:"ancestors"`
	Entries          []profileCaptureEntry `json:"entries"`
}
type profileCaptureSource interface {
	originals() ([]profileCaptureEntry, []string)
	readers() ([]io.Reader, error)
	validate(context.Context, bool) error
	release(context.Context) error
	inspect(context.Context) (profileCaptureSource, error)
	close() error
}
type profileCaptureArchive interface {
	ReadAt(context.Context, []byte, int64) (int, error)
	Validate(context.Context) error
	Snapshot(context.Context) (brokerstore.ExecutionProfileBackupSnapshot, error)
	Close() error
}

type profileCaptureArchiveWriter interface {
	profileCaptureArchive
	Write(context.Context, io.Reader, int64, string) error
	Complete(context.Context) error
}

// ExecutionProfileCapture owns the fixed native full profile and its completed
// broker-private original archive. It is neither commissioning nor launch/health
// authority. Original owner/group/DACL, attributes and times are retained; audit
// descriptors are never replaced or synthesized by capture/in-place restoration.
type ExecutionProfileCapture struct{ state *profileCaptureState }
type profileCaptureState struct {
	mu                sync.Mutex
	owner             context.Context
	source            profileCaptureSource
	archive           profileCaptureArchive
	manifest          profileCaptureManifest
	original          []byte
	released, closed  bool
	reopened          bool
	restoreAttempted  bool
	failure, closeErr error
	inspections       []*ExecutionProfileRestoreInspection
}
type ExecutionProfileCaptureSnapshot struct {
	OriginalManifest []byte
	ManifestSHA256   string
	Archive          brokerstore.ExecutionProfileBackupSnapshot
	// Reopening original data never establishes historical qualification release.
	ReleasedForQualification bool
}

// ExecutionProfileChange is copied inspection data, never restore permission.
type ExecutionProfileChange struct{ Path, Kind string }
type ExecutionProfileRestoreSnapshot struct {
	OriginalManifestSHA256, CurrentManifestSHA256 string
	Changes                                       []ExecutionProfileChange
	MatchesOriginal                               bool
}

// ExecutionProfileRestoreInspection retains a new native cold current-tree
// inspection. Reports cannot construct it. Changed bytes are never overwritten,
// missing originals recreated, or new navigation data deleted by this provider.
type ExecutionProfileRestoreInspection struct {
	capture           *profileCaptureState
	source            profileCaptureSource
	original, current []byte
	changes           []ExecutionProfileChange
	closed            bool
	failure, closeErr error
}

// CaptureExecutionProfileBackupNative accepts no path or supplied profile proof.
// It permanently reserves the live same-Store execution's archive before reading
// the actual KnownFolder ProgramData/opencpn tree. Failures preserve that fence.
func CaptureExecutionProfileBackupNative(ctx context.Context, store *brokerstore.Store, journal *brokerstore.ExecutionJournal) (result *ExecutionProfileCapture, resultErr error) {
	if ctx == nil || store == nil || journal == nil {
		return nil, ErrExecutionProfileCaptureRefused
	}
	archive, err := store.BeginExecutionProfileBackupFresh(ctx, journal)
	if err != nil {
		return nil, err
	}
	defer func() {
		if resultErr != nil {
			resultErr = errors.Join(ErrExecutionProfileCaptureRefused, brokerstore.ErrExecutionProfileBackupUncertain, resultErr, archive.Close())
		}
	}()
	source, err := captureProfileTreeNative(ctx)
	if err != nil {
		return nil, err
	}
	snapshot, err := journal.Snapshot()
	if err != nil {
		return nil, errors.Join(err, source.close())
	}
	return completeProfileCapture(ctx, source, archive, snapshot)
}
func captureDigest(raw []byte) string {
	digest := sha256.Sum256(raw)
	return hex.EncodeToString(digest[:])
}
func profileCapturePath(name string) bool {
	if name == "." {
		return true
	}
	if name == "" || len(name) > 1024 || strings.ContainsAny(name, "\\:\x00") || strings.HasPrefix(name, "/") {
		return false
	}
	for _, part := range strings.Split(name, "/") {
		if part == "" || part == "." || part == ".." || strings.HasSuffix(part, ".") || strings.HasSuffix(part, " ") {
			return false
		}
		stem := strings.ToUpper(strings.Split(part, ".")[0])
		if stem == "CON" || stem == "PRN" || stem == "AUX" || stem == "NUL" || (len(stem) == 4 && (strings.HasPrefix(stem, "COM") || strings.HasPrefix(stem, "LPT")) && stem[3] >= '1' && stem[3] <= '9') {
			return false
		}
	}
	return true
}
func profileManifest(snapshot brokerstore.ExecutionJournalSnapshot, source profileCaptureSource) (profileCaptureManifest, []byte, int64, error) {
	entries, parents := source.originals()
	m := profileCaptureManifest{1, "protected-opencpn-profile-capture-v1", snapshot.ClientSID, snapshot.ExecutionID, snapshot.Generation, snapshot.CompletionSHA256, append([]string(nil), parents...), append([]profileCaptureEntry(nil), entries...)}
	if len(entries) == 0 || len(entries) > brokerstore.MaxFiles || len(parents) == 0 || len(parents) > 128 {
		return m, nil, 0, ErrExecutionProfileCaptureRefused
	}
	sort.Slice(m.Entries, func(i, j int) bool {
		if m.Entries[i].Path == "." || m.Entries[j].Path == "." {
			return m.Entries[i].Path == "." && m.Entries[j].Path != "."
		}
		return m.Entries[i].Path < m.Entries[j].Path
	})
	names := map[string]bool{}
	directories := map[string]bool{}
	var total int64
	for _, e := range m.Entries {
		key := strings.ToLower(e.Path)
		if !profileCapturePath(e.Path) || names[key] || e.Identity == "" || e.Security == "" || e.Created == 0 || e.Attributes == 0 || e.Bytes < 0 || e.Bytes > brokerstore.MaxFileBytes {
			return m, nil, 0, ErrExecutionProfileCaptureRefused
		}
		names[key] = true
		if e.Path == "." && !e.Directory {
			return m, nil, 0, ErrExecutionProfileCaptureRefused
		}
		if e.Path != "." {
			parent := "."
			if at := strings.LastIndexByte(e.Path, '/'); at >= 0 {
				parent = e.Path[:at]
			}
			if !directories[parent] {
				return m, nil, 0, ErrExecutionProfileCaptureRefused
			}
		}
		if e.Directory {
			if e.Bytes != 0 || e.SHA256 != "" {
				return m, nil, 0, ErrExecutionProfileCaptureRefused
			}
			directories[e.Path] = true
		} else {
			if len(e.SHA256) != 64 || strings.Trim(e.SHA256, "0123456789abcdef") != "" || e.Bytes > brokerstore.MaxPayloadBytes-total {
				return m, nil, 0, ErrExecutionProfileCaptureRefused
			}
			total += e.Bytes
		}
	}
	if !directories["."] {
		return m, nil, 0, ErrExecutionProfileCaptureRefused
	}
	raw, err := json.Marshal(m)
	if err != nil || len(raw) > maxProfileManifestBytes || total > brokerstore.MaxPayloadBytes-int64(len(raw))-8 {
		return m, nil, 0, ErrExecutionProfileCaptureRefused
	}
	return m, raw, total + int64(len(raw)) + 8, nil
}
func profileArchiveReader(source profileCaptureSource, raw []byte) (io.Reader, error) {
	readers, err := source.readers()
	if err != nil {
		return nil, err
	}
	var prefix [8]byte
	binary.BigEndian.PutUint64(prefix[:], uint64(len(raw)))
	return io.MultiReader(append([]io.Reader{bytes.NewReader(prefix[:]), bytes.NewReader(raw)}, readers...)...), nil
}
func completeProfileCapture(ctx context.Context, source profileCaptureSource, archive profileCaptureArchiveWriter, journal brokerstore.ExecutionJournalSnapshot) (result *ExecutionProfileCapture, resultErr error) {
	p := &profileCaptureState{owner: ctx, source: source, archive: archive}
	defer func() {
		if resultErr != nil {
			p.failure = resultErr
			resultErr = errors.Join(ErrExecutionProfileCaptureRefused, resultErr, p.closeLocked())
		}
	}()
	if ctx == nil {
		return nil, ErrExecutionProfileCaptureRefused
	}
	if err := source.validate(ctx, true); err != nil {
		return nil, err
	}
	m, raw, size, err := profileManifest(journal, source)
	if err != nil {
		return nil, err
	}
	p.manifest = m
	p.original = raw
	reader, err := profileArchiveReader(source, raw)
	if err != nil {
		return nil, err
	}
	hash := sha256.New()
	count, err := io.Copy(hash, io.LimitReader(reader, size+1))
	if err != nil || count != size {
		return nil, errors.Join(ErrExecutionProfileCaptureRefused, err)
	}
	digest := hex.EncodeToString(hash.Sum(nil))
	if err = source.validate(ctx, true); err != nil {
		return nil, err
	}
	reader, err = profileArchiveReader(source, raw)
	if err != nil {
		return nil, err
	}
	if err = archive.Write(ctx, reader, size, digest); err != nil {
		return nil, err
	}
	if err = source.validate(ctx, true); err != nil {
		return nil, err
	}
	if err = archive.Complete(ctx); err != nil {
		return nil, err
	}
	if err = source.validate(ctx, true); err != nil {
		return nil, errors.Join(brokerstore.ErrExecutionProfileBackupUncertain, err)
	}
	return &ExecutionProfileCapture{p}, nil
}
func (p *profileCaptureState) ready(ctx context.Context) error {
	if p.closed {
		return errors.Join(ErrExecutionProfileCaptureRefused, p.closeErr)
	}
	if p.failure != nil {
		return errors.Join(ErrExecutionProfileCaptureRefused, p.failure)
	}
	if ctx == nil {
		return ErrExecutionProfileCaptureRefused
	}
	if err := errors.Join(ctx.Err(), p.owner.Err()); err != nil {
		return err
	}
	return errors.Join(p.archive.Validate(ctx), p.source.validate(ctx, !p.released), p.owner.Err(), ctx.Err())
}
func (p *ExecutionProfileCapture) Validate(ctx context.Context) error {
	_, err := p.Snapshot(ctx)
	return err
}
func (p *ExecutionProfileCapture) Snapshot(ctx context.Context) (result ExecutionProfileCaptureSnapshot, resultErr error) {
	if p == nil || p.state == nil {
		return result, ErrExecutionProfileCaptureRefused
	}
	s := p.state
	s.mu.Lock()
	defer s.mu.Unlock()
	if err := s.ready(ctx); err != nil {
		s.failure = err
		return result, err
	}
	archive, err := s.archive.Snapshot(ctx)
	if err != nil {
		s.failure = err
		return result, err
	}
	if err := errors.Join(ctx.Err(), s.owner.Err()); err != nil {
		s.failure = err
		return result, err
	}
	return ExecutionProfileCaptureSnapshot{bytes.Clone(s.original), captureDigest(s.original), archive, s.released && !s.reopened}, nil
}

// Evidence binds retained original capture/storage facts. It does not prove
// commissioning, child teardown, restoration, health or selection.
func (p *ExecutionProfileCapture) Evidence(ctx context.Context) (string, error) {
	snapshot, err := p.Snapshot(ctx)
	if err != nil {
		return "", err
	}
	original := struct {
		Schema                                                                                                                         int
		Purpose, ManifestSHA256, ArchiveSHA256, MarkerSHA256, JournalFenceSHA256, ClientSID, ExecutionID, Generation, CompletionSHA256 string
	}{1, "native-original-profile-capture-evidence-v1", snapshot.ManifestSHA256, snapshot.Archive.SHA256, captureDigest(snapshot.Archive.OriginalMarker), snapshot.Archive.JournalFenceSHA256, snapshot.Archive.ClientSID, snapshot.Archive.ExecutionID, snapshot.Archive.Generation, snapshot.Archive.CompletionSHA256}
	raw, err := json.Marshal(original)
	if err != nil {
		return "", err
	}
	return captureDigest(raw), nil
}

// ReleaseForQualification is one-way. It checks the entire frozen tree and
// completed archive before releasing original file deny-writer handles/watch.
// Ancestor custody remains. Subsequent application writes are expected data,
// never permission to erase them or proof that the application closed normally.
func (p *ExecutionProfileCapture) ReleaseForQualification(ctx context.Context) (resultErr error) {
	if p == nil || p.state == nil {
		return ErrExecutionProfileCaptureRefused
	}
	s := p.state
	s.mu.Lock()
	defer s.mu.Unlock()
	if err := s.ready(ctx); err != nil {
		s.failure = err
		return err
	}
	if s.released {
		return ErrExecutionProfileCaptureRefused
	}
	if err := s.source.release(ctx); err != nil {
		s.failure = err
		return err
	}
	s.released = true
	return nil
}
func profileChanges(original, current []profileCaptureEntry) []ExecutionProfileChange {
	old := map[string]profileCaptureEntry{}
	now := map[string]profileCaptureEntry{}
	for _, e := range original {
		old[e.Path] = e
	}
	for _, e := range current {
		now[e.Path] = e
	}
	var result []ExecutionProfileChange
	for name, before := range old {
		after, ok := now[name]
		if !ok {
			result = append(result, ExecutionProfileChange{name, "missing"})
			continue
		}
		// Reads themselves can update last-access time; retain the original value in
		// the archive, but it cannot authorize or obstruct a byte/identity restore.
		before.LastAccess = 0
		after.LastAccess = 0
		// wxFileConfig can legitimately atomically replace the INI. Current
		// physical IDs are owned by the inspection; original IDs prove capture
		// lineage and are not a requirement for restored original contents.
		before.Identity = ""
		after.Identity = ""
		if before != after {
			result = append(result, ExecutionProfileChange{name, "changed"})
		}
	}
	for name := range now {
		if _, ok := old[name]; !ok {
			result = append(result, ExecutionProfileChange{name, "added"})
		}
	}
	sort.Slice(result, func(i, j int) bool { return result[i].Path < result[j].Path })
	return result
}

// InspectRestore independently freezes the current native fixed tree after
// named-process checks. It cannot prove termination of a particular owned child;
// the host must separately retain and corroborate that native process handle.
func (p *ExecutionProfileCapture) InspectRestore(ctx context.Context) (result *ExecutionProfileRestoreInspection, resultErr error) {
	if p == nil || p.state == nil {
		return nil, ErrExecutionProfileCaptureRefused
	}
	s := p.state
	s.mu.Lock()
	defer s.mu.Unlock()
	if err := s.ready(ctx); err != nil {
		return nil, err
	}
	if !s.released {
		return nil, ErrExecutionProfileCaptureRefused
	}
	if len(s.inspections) >= 8 {
		return nil, ErrExecutionProfileCaptureRefused
	}
	source, err := s.source.inspect(ctx)
	if err != nil {
		return nil, err
	}
	i := &ExecutionProfileRestoreInspection{capture: s, source: source, original: bytes.Clone(s.original)}
	defer func() {
		if resultErr != nil {
			resultErr = errors.Join(resultErr, i.closeLocked())
		}
	}()
	_, raw, _, err := profileManifest(brokerstore.ExecutionJournalSnapshot{ClientSID: s.manifest.ClientSID, ExecutionID: s.manifest.ExecutionID, Generation: s.manifest.Generation, CompletionSHA256: s.manifest.CompletionSHA256}, source)
	if err != nil {
		return nil, err
	}
	entries, _ := source.originals()
	i.current = raw
	i.changes = profileChanges(s.manifest.Entries, entries)
	if err = source.validate(ctx, true); err != nil {
		return nil, err
	}
	s.inspections = append(s.inspections, i)
	return i, nil
}
func (i *ExecutionProfileRestoreInspection) Snapshot(ctx context.Context) (result ExecutionProfileRestoreSnapshot, resultErr error) {
	if i == nil || i.capture == nil {
		return result, ErrExecutionProfileCaptureRefused
	}
	s := i.capture
	s.mu.Lock()
	defer s.mu.Unlock()
	if i.closed {
		return result, ErrExecutionProfileCaptureRefused
	}
	if err := s.ready(ctx); err != nil {
		return result, err
	}
	if err := i.source.validate(ctx, true); err != nil {
		i.failure = err
		return result, err
	}
	if err := errors.Join(ctx.Err(), s.owner.Err()); err != nil {
		i.failure = err
		return result, err
	}
	return ExecutionProfileRestoreSnapshot{captureDigest(i.original), captureDigest(i.current), append([]ExecutionProfileChange(nil), i.changes...), len(i.changes) == 0}, nil
}
func (i *ExecutionProfileRestoreInspection) Validate(ctx context.Context) error {
	_, err := i.Snapshot(ctx)
	return err
}

// VerifyRestoredOriginal verifies an independently restored or unchanged profile.
// Every unreviewed change refuses; it never writes, renames, recreates or deletes
// profile data, changes COM8, treats READY as restoration, or grants health.
func (p *ExecutionProfileCapture) VerifyRestoredOriginal(ctx context.Context) (resultErr error) {
	i, err := p.InspectRestore(ctx)
	if err != nil {
		return err
	}
	defer func() { resultErr = errors.Join(resultErr, i.Close()) }()
	snapshot, err := i.Snapshot(ctx)
	if err != nil {
		return err
	}
	if !snapshot.MatchesOriginal {
		return ErrExecutionProfileRestoreReviewRequired
	}
	return nil
}

// RestoreOriginal is the no-proof compatibility inspection: only an unchanged
// original can succeed here. Actual changed-profile restoration uses the private
// restoreOriginalNative boundary with native original-child teardown, closed
// source-derived startup policy, writable current custody and durable intent.
// A public call/client report cannot mint that authority or authorize an overwrite.
func (p *ExecutionProfileCapture) RestoreOriginal(ctx context.Context) error {
	return p.VerifyRestoredOriginal(ctx)
}

func (i *ExecutionProfileRestoreInspection) closeLocked() error {
	if i.closed {
		return i.closeErr
	}
	i.closed = true
	i.closeErr = errors.Join(i.failure, i.source.close())
	return i.closeErr
}
func (i *ExecutionProfileRestoreInspection) Close() error {
	if i == nil || i.capture == nil {
		return nil
	}
	i.capture.mu.Lock()
	defer i.capture.mu.Unlock()
	return i.closeLocked()
}
func (p *profileCaptureState) closeLocked() error {
	if p.closed {
		return p.closeErr
	}
	p.closed = true
	p.closeErr = p.failure
	for _, i := range p.inspections {
		p.closeErr = errors.Join(p.closeErr, i.closeLocked())
	}
	p.closeErr = errors.Join(p.closeErr, p.source.close(), p.archive.Close())
	return p.closeErr
}
func (p *ExecutionProfileCapture) Close() error {
	if p == nil || p.state == nil {
		return nil
	}
	p.state.mu.Lock()
	defer p.state.mu.Unlock()
	return p.state.closeLocked()
}
func (*ExecutionProfileCapture) MarshalJSON() ([]byte, error) {
	return nil, ErrExecutionProfileCaptureRefused
}
func (*ExecutionProfileCapture) UnmarshalJSON([]byte) error { return ErrExecutionProfileCaptureRefused }
func (*ExecutionProfileRestoreInspection) MarshalJSON() ([]byte, error) {
	return nil, ErrExecutionProfileCaptureRefused
}
func (*ExecutionProfileRestoreInspection) UnmarshalJSON([]byte) error {
	return ErrExecutionProfileCaptureRefused
}
