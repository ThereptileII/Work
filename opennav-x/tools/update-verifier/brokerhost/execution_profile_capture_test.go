package brokerhost

import (
	"bytes"
	"context"
	"encoding/binary"
	"encoding/json"
	"errors"
	"io"
	"sort"
	"strings"
	"testing"

	"example.com/opennav-update-verifier/brokerstore"
)

var profileCaptureTestFault = errors.New("profile capture fixture fault")

type memoryProfileCaptureSource struct {
	entries                                []profileCaptureEntry
	files                                  map[string][]byte
	next                                   *memoryProfileCaptureSource
	validateCalls, failValidate, closes    int
	readersFault, releaseFault, closeFault error
	released                               bool
	onReaders                              func()
}

func profileCaptureTestSource() *memoryProfileCaptureSource {
	s := &memoryProfileCaptureSource{files: map[string][]byte{"navigation/navobj.xml": []byte("original navigation\x00\xff"), "opencpn.ini": []byte("[Connections]\r\nCOM8=unchanged\r\n")}}
	for _, name := range []string{".", "empty", "navigation", "navigation/navobj.xml", "opencpn.ini"} {
		directory := name == "." || name == "empty" || name == "navigation"
		e := profileCaptureEntry{Path: name, Directory: directory, Identity: "original-id-" + name, Attributes: 32, Created: 1, LastAccess: 2, LastWrite: 3, Security: "original owner/group/DACL"}
		if directory {
			e.Attributes = 16
		} else {
			e.Bytes = int64(len(s.files[name]))
			e.SHA256 = captureDigest(s.files[name])
		}
		s.entries = append(s.entries, e)
	}
	return s
}
func (s *memoryProfileCaptureSource) originals() ([]profileCaptureEntry, []string) {
	return append([]profileCaptureEntry(nil), s.entries...), []string{"physical-original-volume", "physical-original-parent", "physical-original-root"}
}
func (s *memoryProfileCaptureSource) readers() ([]io.Reader, error) {
	if s.onReaders != nil {
		s.onReaders()
	}
	if s.readersFault != nil {
		return nil, s.readersFault
	}
	var result []io.Reader
	entries := append([]profileCaptureEntry(nil), s.entries...)
	sort.Slice(entries, func(i, j int) bool { return entries[i].Path < entries[j].Path })
	for _, e := range entries {
		if !e.Directory {
			result = append(result, bytes.NewReader(s.files[e.Path]))
		}
	}
	return result, nil
}
func (s *memoryProfileCaptureSource) validate(ctx context.Context, frozen bool) error {
	s.validateCalls++
	if ctx == nil {
		return profileCaptureTestFault
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	if s.validateCalls == s.failValidate || (frozen && s.released) {
		return profileCaptureTestFault
	}
	if frozen {
		for _, e := range s.entries {
			if !e.Directory && (int64(len(s.files[e.Path])) != e.Bytes || captureDigest(s.files[e.Path]) != e.SHA256) {
				return profileCaptureTestFault
			}
		}
	}
	return nil
}
func (s *memoryProfileCaptureSource) release(ctx context.Context) error {
	if s.releaseFault != nil {
		return s.releaseFault
	}
	s.released = true
	return ctx.Err()
}
func (s *memoryProfileCaptureSource) inspect(ctx context.Context) (profileCaptureSource, error) {
	if s.next == nil {
		s.next = profileCaptureTestSource()
	}
	return s.next, ctx.Err()
}
func (s *memoryProfileCaptureSource) close() error { s.closes++; return s.closeFault }

type memoryProfileCaptureArchive struct {
	onSnapshot func()
	raw        []byte
	digest     string
	events     []string
	closes     int
	fault      string
	closeFault error
	cancel     context.CancelFunc
}

func (b *memoryProfileCaptureArchive) Write(ctx context.Context, r io.Reader, size int64, digest string) error {
	b.events = append(b.events, "write")
	var err error
	b.raw, err = io.ReadAll(io.LimitReader(r, size+1))
	if err != nil {
		return err
	}
	b.digest = digest
	if b.fault == "write" || int64(len(b.raw)) != size || captureDigest(b.raw) != digest {
		return profileCaptureTestFault
	}
	return ctx.Err()
}
func (b *memoryProfileCaptureArchive) Complete(ctx context.Context) error {
	b.events = append(b.events, "complete")
	if b.fault == "complete" {
		return profileCaptureTestFault
	}
	if b.cancel != nil {
		b.cancel()
	}
	return nil
}
func (b *memoryProfileCaptureArchive) Validate(ctx context.Context) error {
	if b.fault == "validate" || captureDigest(b.raw) != b.digest {
		return profileCaptureTestFault
	}
	return ctx.Err()
}
func (b *memoryProfileCaptureArchive) Snapshot(ctx context.Context) (brokerstore.ExecutionProfileBackupSnapshot, error) {
	if b.onSnapshot != nil {
		b.onSnapshot()
	}
	return brokerstore.ExecutionProfileBackupSnapshot{Bytes: int64(len(b.raw)), SHA256: b.digest, OriginalMarker: []byte("owned original storage completion")}, b.Validate(ctx)
}
func (b *memoryProfileCaptureArchive) Close() error { b.closes++; return b.closeFault }
func profileCaptureFixture(t *testing.T) (*ExecutionProfileCapture, *memoryProfileCaptureSource, *memoryProfileCaptureArchive) {
	t.Helper()
	s := profileCaptureTestSource()
	b := &memoryProfileCaptureArchive{}
	p, err := completeProfileCapture(context.Background(), s, b, brokerstore.ExecutionJournalSnapshot{ClientSID: "S-1-5-21-1-2-3-1001", ExecutionID: strings.Repeat("a", 32), Generation: strings.Repeat("b", 32), CompletionSHA256: strings.Repeat("c", 64)})
	if err != nil {
		t.Fatal(err)
	}
	return p, s, b
}
func TestExecutionProfileCaptureStreamsEveryOriginalAndMetadata(t *testing.T) {
	p, s, b := profileCaptureFixture(t)
	defer p.Close()
	header := binary.BigEndian.Uint64(b.raw[:8])
	var manifest profileCaptureManifest
	if err := json.Unmarshal(b.raw[8:8+header], &manifest); err != nil {
		t.Fatal(err)
	}
	if len(manifest.Entries) != 5 || manifest.Entries[1].Path != "empty" || !manifest.Entries[1].Directory || manifest.Entries[0].Security != "original owner/group/DACL" {
		t.Fatal("complete original tree/metadata lost")
	}
	remaining := b.raw[8+header:]
	for _, entry := range manifest.Entries {
		if entry.Directory {
			continue
		}
		part := remaining[:entry.Bytes]
		remaining = remaining[entry.Bytes:]
		if !bytes.Equal(part, s.files[entry.Path]) || captureDigest(part) != entry.SHA256 {
			t.Fatal("archive member/manifest byte binding lost", entry.Path)
		}
	}
	if len(remaining) != 0 || strings.Join(b.events, ",") != "write,complete" {
		t.Fatal("archive contains unknown bytes or wrong publication order")
	}
	snapshot, err := p.Snapshot(context.Background())
	if err != nil {
		t.Fatal(err)
	}
	snapshot.OriginalManifest[0] ^= 1
	snapshot.Archive.OriginalMarker[0] ^= 1
	if err = p.Validate(context.Background()); err != nil {
		t.Fatal("informational copy mutated custody", err)
	}
	if evidence, err := p.Evidence(context.Background()); err != nil || len(evidence) != 64 {
		t.Fatal("native capture evidence unavailable", err)
	}
	if !bytes.Contains(b.raw, []byte("COM8=unchanged")) {
		t.Fatal("profile was transformed")
	}
	if _, err = json.Marshal(p); !errors.Is(err, ErrExecutionProfileCaptureRefused) {
		t.Fatal("serialized owner accepted")
	}
}
func TestExecutionProfileCaptureRefusesIncompleteUnsafeAndChangedInputs(t *testing.T) {
	for _, fault := range []string{"missing-root", "case-alias", "bad-relative", "missing-parent", "too-many", "negative-size", "hash", "archive-write", "archive-complete", "source-read", "source-before-write", "source-after-write", "source-after-complete"} {
		t.Run(fault, func(t *testing.T) {
			s := profileCaptureTestSource()
			b := &memoryProfileCaptureArchive{}
			switch fault {
			case "missing-root":
				s.entries = s.entries[1:]
			case "case-alias":
				e := s.entries[4]
				e.Path = "OPENCPN.INI"
				s.entries = append(s.entries, e)
			case "bad-relative":
				s.entries[4].Path = "../opencpn.ini"
			case "missing-parent":
				s.entries = append(s.entries[:2], s.entries[3:]...)
			case "too-many":
				s.entries = make([]profileCaptureEntry, brokerstore.MaxFiles+1)
			case "negative-size":
				s.entries[4].Bytes = -1
			case "hash":
				s.entries[4].SHA256 = strings.Repeat("0", 64)
			case "archive-write":
				b.fault = "write"
			case "archive-complete":
				b.fault = "complete"
			case "source-read":
				s.readersFault = profileCaptureTestFault
			case "source-before-write":
				s.failValidate = 2
			case "source-after-write":
				s.failValidate = 3
			case "source-after-complete":
				s.failValidate = 4
			}
			p, err := completeProfileCapture(context.Background(), s, b, brokerstore.ExecutionJournalSnapshot{})
			if p != nil || err == nil || s.closes != 1 || b.closes != 1 {
				t.Fatal("unsafe full-profile capture accepted or cleanup repeated", err, s.closes, b.closes)
			}
			if fault == "source-after-complete" && !errors.Is(err, brokerstore.ErrExecutionProfileBackupUncertain) {
				t.Fatal("durable archive uncertainty hidden", err)
			}
		})
	}
}
func TestExecutionProfileCaptureOneWayReleaseAndFreshNativeRestoreInspection(t *testing.T) {
	p, s, _ := profileCaptureFixture(t)
	if _, err := p.InspectRestore(context.Background()); err == nil {
		t.Fatal("inspection before release accepted")
	}
	if err := p.ReleaseForQualification(context.Background()); err != nil {
		t.Fatal(err)
	}
	if err := p.ReleaseForQualification(context.Background()); err == nil {
		t.Fatal("release retried")
	}
	if err := p.Validate(context.Background()); err != nil {
		t.Fatal("normal released state rejected", err)
	}
	if err := p.RestoreOriginal(context.Background()); err != nil {
		t.Fatal("exact original independently verified no-op refused", err)
	}
	// Atomic wx replacement is allowed as original contents, but the new full ID
	// is retained independently by the native current-tree inspection.
	current := profileCaptureTestSource()
	current.entries[4].Identity = "wx-replaced-current-native-id"
	s.next = current
	if err := p.VerifyRestoredOriginal(context.Background()); err != nil {
		t.Fatal("restore incorrectly required old file ID", err)
	}
	if err := p.Close(); err != nil {
		t.Fatal(err)
	}
	if err := p.Validate(context.Background()); err == nil {
		t.Fatal("closed owner granted authority")
	}
}
func TestExecutionProfileCaptureNeverRestoresUnreviewedNavigationChanges(t *testing.T) {
	for _, kind := range []string{"changed-navigation", "added", "missing", "changed-acl", "changed-current-after-inspect"} {
		t.Run(kind, func(t *testing.T) {
			p, s, _ := profileCaptureFixture(t)
			defer p.Close()
			if err := p.ReleaseForQualification(context.Background()); err != nil {
				t.Fatal(err)
			}
			current := profileCaptureTestSource()
			s.next = current
			switch kind {
			case "changed-navigation":
				current.files["navigation/navobj.xml"] = []byte("new navigation data")
				current.entries[3].Bytes = int64(len(current.files["navigation/navobj.xml"]))
				current.entries[3].SHA256 = captureDigest(current.files["navigation/navobj.xml"])
			case "added":
				e := current.entries[4]
				e.Path = "new-voyage.gpx"
				current.files[e.Path] = bytes.Clone(current.files["opencpn.ini"])
				current.entries = append(current.entries, e)
			case "missing":
				current.entries = current.entries[:4]
			case "changed-acl":
				current.entries[4].Security = "unreviewed owner/group/DACL"
			}
			i, err := p.InspectRestore(context.Background())
			if err != nil {
				t.Fatal(err)
			}
			if kind == "changed-current-after-inspect" {
				current.files["opencpn.ini"] = []byte("changed after native freeze")
				if err = i.Validate(context.Background()); err == nil {
					t.Fatal("stale restore inspection accepted")
				}
				return
			}
			snapshot, err := i.Snapshot(context.Background())
			if err != nil || snapshot.MatchesOriginal || len(snapshot.Changes) == 0 {
				t.Fatal("unreviewed full-tree change disappeared", err)
			}
			snapshot.Changes[0].Kind = "fake-restored"
			snapshot, err = i.Snapshot(context.Background())
			if err != nil || snapshot.Changes[0].Kind == "fake-restored" {
				t.Fatal("copied report minted restore evidence", err)
			}
			if err = p.RestoreOriginal(context.Background()); !errors.Is(err, ErrExecutionProfileRestoreReviewRequired) {
				t.Fatal("unreviewed navigation overwritten or inferred restored", err)
			}
		})
	}
}
func TestExecutionProfileCaptureCancellationRevocationAndCleanup(t *testing.T) {
	s := profileCaptureTestSource()
	b := &memoryProfileCaptureArchive{}
	ctx, cancel := context.WithCancel(context.Background())
	b.cancel = cancel
	if p, err := completeProfileCapture(ctx, s, b, brokerstore.ExecutionJournalSnapshot{}); p != nil || !errors.Is(err, context.Canceled) || !errors.Is(err, brokerstore.ErrExecutionProfileBackupUncertain) {
		t.Fatal("late cancellation lost durable uncertainty", err)
	}
	p, s, b := profileCaptureFixture(t)
	b.closeFault = profileCaptureTestFault
	s.closeFault = profileCaptureTestFault
	first := p.Close()
	if !errors.Is(first, profileCaptureTestFault) || p.Close() != first || s.closes != 1 || b.closes != 1 {
		t.Fatal("cleanup outcome not retained", first)
	}
	p2, _, b2 := profileCaptureFixture(t)
	owned, cancelOwned := context.WithCancel(context.Background())
	defer cancelOwned()
	p2.state.owner = owned
	b2.onSnapshot = cancelOwned
	if _, err := p2.Snapshot(context.Background()); !errors.Is(err, context.Canceled) {
		t.Fatal("cancelled original owner returned snapshot", err)
	}
	p2.Close()
	var zero ExecutionProfileCapture
	if err := zero.Validate(context.Background()); err == nil {
		t.Fatal("zero capability accepted")
	}
	if _, err := CaptureExecutionProfileBackupNative(context.Background(), nil, nil); err == nil {
		t.Fatal("unowned supplied storage accepted")
	}
}

func (b *memoryProfileCaptureArchive) ReadAt(ctx context.Context, dst []byte, offset int64) (int, error) {
	if err := ctx.Err(); err != nil {
		return 0, err
	}
	return bytes.NewReader(b.raw).ReadAt(dst, offset)
}
