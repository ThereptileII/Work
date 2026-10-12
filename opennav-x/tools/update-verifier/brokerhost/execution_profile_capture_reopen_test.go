package brokerhost

import (
	"bytes"
	"context"
	"encoding/binary"
	"encoding/json"
	"errors"
	"io"
	"strings"
	"testing"

	"example.com/opennav-update-verifier/brokerstore"
)

// Implements only the reader interface: reopening cannot depend on Write/Complete.
type reopenMemoryArchive struct {
	raw           []byte
	snapshot      brokerstore.ExecutionProfileBackupSnapshot
	closes, reads int
	closed        bool
	closeFault    error
	onRead        func(int)
}

func (a *reopenMemoryArchive) Validate(ctx context.Context) error {
	if a.closed || captureDigest(a.raw) != a.snapshot.SHA256 {
		return profileCaptureTestFault
	}
	return ctx.Err()
}
func (a *reopenMemoryArchive) Snapshot(ctx context.Context) (brokerstore.ExecutionProfileBackupSnapshot, error) {
	snapshot := a.snapshot
	snapshot.OriginalMarker = bytes.Clone(snapshot.OriginalMarker)
	return snapshot, a.Validate(ctx)
}
func (a *reopenMemoryArchive) ReadAt(ctx context.Context, dst []byte, offset int64) (int, error) {
	a.reads++
	if a.onRead != nil {
		a.onRead(a.reads)
	}
	if err := a.Validate(ctx); err != nil {
		return 0, err
	}
	if len(dst) > 64<<10 {
		return 0, profileCaptureTestFault
	}
	return bytes.NewReader(a.raw).ReadAt(dst, offset)
}
func (a *reopenMemoryArchive) Close() error {
	if !a.closed {
		a.closed = true
		a.closes++
	}
	return a.closeFault
}

type reopenMemorySource struct {
	manifest         profileCaptureManifest
	closed, released bool
	closes           int
}

func (s *reopenMemorySource) originals() ([]profileCaptureEntry, []string) {
	return s.manifest.Entries, s.manifest.Ancestors
}
func (*reopenMemorySource) readers() ([]io.Reader, error) { return nil, profileCaptureTestFault }
func (s *reopenMemorySource) validate(ctx context.Context, frozen bool) error {
	if s.closed || frozen && s.released {
		return profileCaptureTestFault
	}
	return ctx.Err()
}
func (s *reopenMemorySource) release(ctx context.Context) error { s.released = true; return ctx.Err() }
func (s *reopenMemorySource) inspect(ctx context.Context) (profileCaptureSource, error) {
	return &reopenMemorySource{manifest: s.manifest}, ctx.Err()
}
func (s *reopenMemorySource) close() error { s.closed = true; s.closes++; return nil }

func reopenFixtureID(n byte) executionProfileID {
	id := executionProfileID{volume: 1, created: 123}
	id.file[0] = n
	return id
}
func reopenProfileFixture(t *testing.T) (*reopenMemoryArchive, brokerstore.ExecutionJournalSnapshot, profileCaptureManifest, []byte) {
	t.Helper()
	journal := brokerstore.ExecutionJournalSnapshot{ClientSID: "S-1-5-21-1-2-3-1001", ExecutionID: strings.Repeat("a", 32), Generation: strings.Repeat("b", 32), CompletionSHA256: strings.Repeat("c", 64), OriginalFence: []byte("exact protected original journal origin")}
	payload := []byte("[Connections]\r\nCOM8=preserved\r\n\x00\xff")
	m := profileCaptureManifest{Schema: 1, Purpose: "protected-opencpn-profile-capture-v1", ClientSID: journal.ClientSID, ExecutionID: journal.ExecutionID, Generation: journal.Generation, CompletionSHA256: journal.CompletionSHA256, Ancestors: []string{reopenFixtureID(1).wire(), reopenFixtureID(2).wire()}, Entries: []profileCaptureEntry{
		{Path: ".", Directory: true, Identity: reopenFixtureID(2).wire(), Attributes: 16, Created: 123, LastAccess: 234, LastWrite: 345, Security: "O:SYG:SYD:P(A;;FA;;;SY)"},
		{Path: "opencpn.ini", Bytes: int64(len(payload)), SHA256: captureDigest(payload), Identity: reopenFixtureID(3).wire(), Attributes: 32, Created: 123, LastAccess: 234, LastWrite: 345, Security: "O:SYG:SYD:P(A;;FA;;;SY)"},
	}}
	a := &reopenMemoryArchive{snapshot: brokerstore.ExecutionProfileBackupSnapshot{ClientSID: journal.ClientSID, ExecutionID: journal.ExecutionID, Generation: journal.Generation, CompletionSHA256: journal.CompletionSHA256, JournalFenceSHA256: captureDigest(journal.OriginalFence), OriginalMarker: []byte("retained completed storage marker")}}
	resetReopenArchive(t, a, m, payload)
	return a, journal, m, payload
}
func resetReopenArchive(t *testing.T, a *reopenMemoryArchive, m profileCaptureManifest, payload []byte) {
	t.Helper()
	raw, err := json.Marshal(m)
	if err != nil {
		t.Fatal(err)
	}
	var header [8]byte
	binary.BigEndian.PutUint64(header[:], uint64(len(raw)))
	a.raw = append(append(header[:], raw...), payload...)
	a.snapshot.Bytes = int64(len(a.raw))
	a.snapshot.SHA256 = captureDigest(a.raw)
}
func reopenFixtureFactory(s *reopenMemorySource) func(context.Context, profileCaptureManifest) (profileCaptureSource, error) {
	return func(ctx context.Context, m profileCaptureManifest) (profileCaptureSource, error) {
		return bindReopenedProfileSource(ctx, m, s)
	}
}
func TestExecutionProfileCaptureReopenOriginalReadOnlyInspection(t *testing.T) {
	a, j, m, _ := reopenProfileFixture(t)
	source := &reopenMemorySource{manifest: m}
	p, err := openProfileCaptureArchive(context.Background(), a, j, reopenFixtureFactory(source))
	if err != nil {
		t.Fatal(err)
	}
	snapshot, err := p.Snapshot(context.Background())
	if err != nil || snapshot.ReleasedForQualification {
		t.Fatal("invented historical release", err)
	}
	original := bytes.Clone(snapshot.OriginalManifest)
	snapshot.OriginalManifest[0] = 'x'
	next, err := p.Snapshot(context.Background())
	if err != nil || !bytes.Equal(next.OriginalManifest, original) {
		t.Fatal("mutable original manifest", err)
	}
	if err = p.VerifyRestoredOriginal(context.Background()); err != nil {
		t.Fatal("cold unchanged inspection refused", err)
	}
	if err = p.ReleaseForQualification(context.Background()); err == nil {
		t.Fatal("reopened capture released historical custody")
	}
	if _, ok := p.state.source.(profileRestoreFactory); ok {
		t.Fatal("reopened source acquired writable factory")
	}
	authority := &memoryProfileRestoreAuthority{}
	if _, err = p.restoreOriginalNative(context.Background(), authority); err == nil || authority.intent.Schema != 0 {
		t.Fatal("restart recreated changed-profile authority", err)
	}
	if err = p.Close(); err != nil {
		t.Fatal(err)
	}
	if err = p.Close(); err != nil || a.closes != 1 || source.closes != 1 {
		t.Fatal("cleanup not cached", err, a.closes, source.closes)
	}
	if err = p.Validate(context.Background()); err == nil {
		t.Fatal("closed capture accepted")
	}
}
func TestExecutionProfileCaptureReopenMalformedArchiveRefuses(t *testing.T) {
	for _, kind := range []string{"short-header", "zero-header", "overflow-header", "truncated-manifest", "truncated-member", "trailing", "duplicate-json", "unknown-json", "whitespace", "bad-member-hash"} {
		t.Run(kind, func(t *testing.T) {
			a, j, m, payload := reopenProfileFixture(t)
			switch kind {
			case "short-header":
				a.raw = a.raw[:7]
			case "zero-header":
				binary.BigEndian.PutUint64(a.raw[:8], 0)
			case "overflow-header":
				binary.BigEndian.PutUint64(a.raw[:8], ^uint64(0))
			case "truncated-manifest":
				a.raw = a.raw[:20]
			case "truncated-member":
				a.raw = a.raw[:len(a.raw)-1]
			case "trailing":
				a.raw = append(a.raw, 0)
			case "bad-member-hash":
				a.raw[len(a.raw)-1] ^= 1
			default:
				raw, _ := json.Marshal(m)
				switch kind {
				case "duplicate-json":
					raw = append([]byte(`{"schema":1,`), raw[1:]...)
				case "unknown-json":
					raw = append([]byte(`{"unknown":0,`), raw[1:]...)
				case "whitespace":
					raw = append([]byte(" "), raw...)
				}
				var header [8]byte
				binary.BigEndian.PutUint64(header[:], uint64(len(raw)))
				a.raw = append(append(header[:], raw...), payload...)
			}
			a.snapshot.Bytes = int64(len(a.raw))
			a.snapshot.SHA256 = captureDigest(a.raw)
			calls := 0
			_, err := openProfileCaptureArchive(context.Background(), a, j, func(context.Context, profileCaptureManifest) (profileCaptureSource, error) {
				calls++
				return nil, profileCaptureTestFault
			})
			if err == nil || calls != 0 || a.closes != 1 {
				t.Fatal("malformed original reached native factory", err, calls, a.closes)
			}
		})
	}
}
func TestExecutionProfileCaptureReopenManifestIdentityRefuses(t *testing.T) {
	for _, kind := range []string{"schema", "purpose", "sid", "execution", "generation", "completion", "archive-rebind", "fence", "partial-parent", "duplicate-member", "alias", "path", "negative-size", "overflow-size", "digest", "directory-hash", "identity-case", "identity-zero", "identity-created", "identity-duplicate", "identity-volume", "parent-duplicate", "root-parent", "attributes", "directory-attributes", "security-nul"} {
		t.Run(kind, func(t *testing.T) {
			a, j, m, payload := reopenProfileFixture(t)
			switch kind {
			case "schema":
				m.Schema = 2
			case "purpose":
				m.Purpose = "untrusted"
			case "sid":
				m.ClientSID += "0"
			case "execution":
				m.ExecutionID = strings.Repeat("d", 32)
			case "generation":
				m.Generation = strings.Repeat("d", 32)
			case "completion":
				m.CompletionSHA256 = strings.Repeat("d", 64)
			case "archive-rebind":
				a.snapshot.Generation = strings.Repeat("d", 32)
			case "fence":
				j.OriginalFence = append(j.OriginalFence, '!')
			case "partial-parent":
				m.Entries[1].Path = "missing/opencpn.ini"
			case "duplicate-member":
				m.Entries = append(m.Entries, m.Entries[1])
			case "alias":
				e := m.Entries[1]
				e.Path = "OPENCPN.INI"
				m.Entries = append(m.Entries, e)
			case "path":
				m.Entries[1].Path = "../opencpn.ini"
			case "negative-size":
				m.Entries[1].Bytes = -1
			case "overflow-size":
				m.Entries[1].Bytes = 1 << 62
			case "digest":
				m.Entries[1].SHA256 = strings.Repeat("g", 64)
			case "directory-hash":
				m.Entries[0].SHA256 = captureDigest(nil)
			case "identity-case":
				m.Entries[1].Identity = strings.Replace(m.Entries[1].Identity, "7b", "7B", 1)
			case "identity-zero":
				m.Entries[1].Identity = strings.Repeat("0", 16) + m.Entries[1].Identity[16:]
			case "identity-created":
				m.Entries[1].Created++
			case "identity-duplicate":
				m.Entries[1].Identity = m.Entries[0].Identity
			case "identity-volume":
				id := reopenFixtureID(3)
				id.volume = 2
				m.Entries[1].Identity = id.wire()
			case "parent-duplicate":
				m.Ancestors = append(m.Ancestors[:1], m.Ancestors...)
			case "root-parent":
				m.Ancestors[1] = reopenFixtureID(4).wire()
			case "attributes":
				m.Entries[1].Attributes |= 0x400
			case "directory-attributes":
				m.Entries[1].Attributes = 16
			case "security-nul":
				m.Entries[1].Security += "\x00"
			}
			resetReopenArchive(t, a, m, payload)
			calls := 0
			_, err := openProfileCaptureArchive(context.Background(), a, j, func(context.Context, profileCaptureManifest) (profileCaptureSource, error) {
				calls++
				return nil, profileCaptureTestFault
			})
			if err == nil || calls != 0 || a.closes != 1 {
				t.Fatal("invalid binding reached native factory", err, calls, a.closes)
			}
		})
	}
}
func TestExecutionProfileCaptureReopenCustodyCancelAndCleanup(t *testing.T) {
	for _, kind := range []string{"cancel-before", "cancel-reading", "source-rebind", "source-release", "archive-revoke", "close-error"} {
		t.Run(kind, func(t *testing.T) {
			a, j, m, _ := reopenProfileFixture(t)
			source := &reopenMemorySource{manifest: m}
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			switch kind {
			case "cancel-before":
				cancel()
			case "cancel-reading":
				a.onRead = func(n int) {
					if n == 2 {
						cancel()
					}
				}
			case "source-rebind":
				source.manifest.Ancestors = append([]string(nil), m.Ancestors...)
				source.manifest.Ancestors[0] = reopenFixtureID(9).wire()
			case "source-release":
				source.closed = true
			case "close-error":
				a.closeFault = profileCaptureTestFault
			}
			p, err := openProfileCaptureArchive(ctx, a, j, reopenFixtureFactory(source))
			if kind == "archive-revoke" || kind == "close-error" {
				if err != nil {
					t.Fatal(err)
				}
				if kind == "archive-revoke" {
					if err = a.Close(); err != nil {
						t.Fatal(err)
					}
					if err = p.Validate(ctx); err == nil {
						t.Fatal("revoked original archive remained usable")
					}
				}
				first := p.Close()
				second := p.Close()
				if a.closes != 1 || source.closes != 1 || (kind == "close-error" && (!errors.Is(first, profileCaptureTestFault) || !errors.Is(second, profileCaptureTestFault))) {
					t.Fatal("close result/custody not retained", first, second, a.closes, source.closes)
				}
			} else if err == nil || a.closes != 1 {
				t.Fatal("failed reopen accepted/reused", err, a.closes)
			}
		})
	}
}
