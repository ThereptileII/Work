package brokerstore

import (
	"bytes"
	"context"
	"crypto/sha256"
	"encoding/hex"
	"errors"
	"io"
	"strings"
	"testing"
	"time"
)

var testFault = errors.New("injected custody failure")

const stageA = "0123456789abcdef0123456789abcdef"

type memoryStore struct {
	stages   map[string]bool
	children []*memoryCandidate
	fault    error
	closes   int
}

func (m *memoryStore) check() error { return m.fault }
func (m *memoryStore) begin(id, _ string) (candidateBackend, error) {
	if m.stages[id] {
		return nil, testFault
	}
	m.stages[id] = true
	c := &memoryCandidate{files: map[string]*memoryFile{}}
	m.children = append(m.children, c)
	return c, nil
}
func (*memoryStore) registration(string) ([]byte, error) { return []byte("fixed registration"), nil }
func (*memoryStore) verifierDirectories(string, string) (string, string, error) {
	return "fixed state", "fixed artifacts", nil
}
func (m *memoryStore) close() error { m.closes++; return m.fault }

type memoryCandidate struct {
	configure                      func(*memoryFile)
	files                          map[string]*memoryFile
	fault, createFault, closeFault error
	closes                         int
}

func (m *memoryCandidate) check() error { return m.fault }
func (m *memoryCandidate) create(name string) (candidateFile, error) {
	if m.createFault != nil {
		return nil, m.createFault
	}
	if _, ok := m.files[name]; ok {
		return nil, testFault
	}
	f := &memoryFile{}
	if m.configure != nil {
		m.configure(f)
	}
	m.files[name] = f
	return f, nil
}
func (m *memoryCandidate) candidateInventory(names []string, _ bool) error {
	if len(names) != len(m.files) {
		return testFault
	}
	for _, name := range names {
		if m.files[name] == nil {
			return testFault
		}
	}
	return m.fault
}
func (m *memoryCandidate) close() error { m.closes++; return m.closeFault }

type memoryFile struct {
	data                         []byte
	pos                          int64
	fault, syncFault, closeFault error
	onRead                       func()
	closes                       int
}

func (f *memoryFile) Read(p []byte) (int, error) {
	if f.onRead != nil {
		f.onRead()
	}
	if f.pos >= int64(len(f.data)) {
		return 0, io.EOF
	}
	n := copy(p, f.data[f.pos:])
	f.pos += int64(n)
	return n, nil
}
func (f *memoryFile) Write(p []byte) (int, error) {
	f.data = append(f.data, p...)
	f.pos += int64(len(p))
	return len(p), nil
}
func (f *memoryFile) Seek(offset int64, whence int) (int64, error) {
	if whence != io.SeekStart || offset < 0 {
		return 0, testFault
	}
	f.pos = offset
	return offset, nil
}
func (f *memoryFile) Sync() error { return f.syncFault }
func (f *memoryFile) check(n int64) error {
	if f.fault != nil {
		return f.fault
	}
	if int64(len(f.data)) != n {
		return testFault
	}
	return nil
}
func (f *memoryFile) Close() error { f.closes++; return f.closeFault }
func testStore() (*Store, *memoryStore) {
	m := &memoryStore{stages: map[string]bool{}}
	return &Store{backend: m}, m
}
func testCandidate(t *testing.T) (*Store, *Candidate, *memoryCandidate) {
	t.Helper()
	s, m := testStore()
	c, e := s.Begin(stageA, "client")
	if e != nil {
		t.Fatal(e)
	}
	t.Cleanup(func() { s.Close() })
	return s, c, m.children[0]
}
func digest(s string) string { h := sha256.Sum256([]byte(s)); return hex.EncodeToString(h[:]) }
func write(c *Candidate, name, data string) error {
	return c.WriteFile(context.Background(), name, strings.NewReader(data), int64(len(data)), digest(data))
}

func TestCloseRetainsCleanupFailuresWithoutRetry(t *testing.T) {
	s, m := testStore()
	c, err := s.Begin(stageA, "client")
	if err != nil {
		t.Fatal(err)
	}
	if err = write(c, "app/a", "payload"); err != nil {
		t.Fatal(err)
	}
	fileFault := errors.New("file close uncertainty")
	candidateFault := errors.New("candidate close uncertainty")
	storeFault := errors.New("store close uncertainty")
	child := m.children[0]
	file := child.files["app/a"]
	file.closeFault, child.closeFault = fileFault, candidateFault
	first := c.Close()
	if !errors.Is(first, fileFault) || !errors.Is(first, candidateFault) {
		t.Fatalf("first close lost cleanup failure: %v", first)
	}
	file.closeFault, child.closeFault = nil, nil
	if next := c.Close(); next != first {
		t.Fatalf("repeated candidate close changed original result: %v", next)
	}
	m.fault = storeFault
	closed := s.Close()
	for _, fault := range []error{fileFault, candidateFault, storeFault} {
		if !errors.Is(closed, fault) {
			t.Fatalf("store close lost earlier cleanup uncertainty: %v", closed)
		}
	}
	m.fault = nil
	if next := s.Close(); next != closed {
		t.Fatalf("repeated store close changed original result: %v", next)
	}
	if file.closes != 1 || child.closes != 1 || m.closes != 1 {
		t.Fatalf("native cleanup retried: file=%d candidate=%d store=%d", file.closes, child.closes, m.closes)
	}
}

type waitingCloseStore struct {
	memoryStore
	entered, release chan struct{}
}

func (m *waitingCloseStore) close() error {
	close(m.entered)
	<-m.release
	return m.memoryStore.close()
}

func TestConcurrentStoreCloseWaitsForOriginalCleanup(t *testing.T) {
	m := &waitingCloseStore{memoryStore: memoryStore{fault: testFault}, entered: make(chan struct{}), release: make(chan struct{})}
	s := &Store{backend: m}
	first, second := make(chan error, 1), make(chan error, 1)
	go func() { first <- s.Close() }()
	select {
	case <-m.entered:
	case <-time.After(5 * time.Second):
		t.Fatal("first close did not reach backend")
	}
	started := make(chan struct{})
	go func() { close(started); second <- s.Close() }()
	<-started
	select {
	case err := <-second:
		close(m.release)
		<-first
		t.Fatalf("second close returned before cleanup finished: %v", err)
	case <-time.After(50 * time.Millisecond):
	}
	close(m.release)
	one, two := <-first, <-second
	if !errors.Is(one, testFault) || two != one || m.closes != 1 {
		t.Fatalf("concurrent close result changed/retried: first=%v second=%v closes=%d", one, two, m.closes)
	}
}

func TestPayloadAndSeal(t *testing.T) {
	_, c, m := testCandidate(t)
	f := baselineData(t)
	writeCandidateEvidence(t, c, f)
	if err := c.CompleteRecoveryEvidence(context.Background(), f.identity); err != nil {
		t.Fatal(err)
	}
	receipt := []byte(`{"stage":"bound by caller"}`)
	if e := c.Seal(context.Background(), receipt); e != nil {
		t.Fatal(e)
	}
	if !bytes.Equal(m.files[ReceiptFilename].data, receipt) {
		t.Fatal("receipt changed")
	}
	if e := write(c, "app/late.dll", "x"); e == nil {
		t.Fatal("write after seal")
	}
	if e := c.Seal(context.Background(), receipt); e == nil {
		t.Fatal("second seal")
	}
}
func TestMemberPaths(t *testing.T) {
	for _, name := range []string{"app/a.exe", "docs/licenses/z.txt", "app/a_b-1.dll"} {
		if e := memberPath(name); e != nil {
			t.Errorf("%s: %v", name, e)
		}
	}
	for _, name := range []string{"", "app", "App/a", "other/a", "/app/a", "C:/app/a", "app/../a", "app//a", `app\a`, "app/a:stream", "app/a.", "app/a ", "app/CON.txt", "app/COM1.dll", "app/LPT9", "app/NUL", "app/é", "app/" + strings.Repeat("a", 240)} {
		t.Run(name, func(t *testing.T) {
			if memberPath(name) == nil {
				t.Fatal("unsafe path accepted")
			}
		})
	}
}
func TestCollisionPoisons(t *testing.T) {
	for _, pair := range [][2]string{{"app/A.dll", "app/a.dll"}, {"app/a.dll", "app/a.dll"}, {"app/sub/a", "app/Sub/b"}, {"app/sub/a", "app/sub"}, {"app/sub", "app/sub/a"}} {
		t.Run(strings.Join(pair[:], " -> "), func(t *testing.T) {
			_, c, _ := testCandidate(t)
			if e := write(c, pair[0], "x"); e != nil {
				t.Fatal(e)
			}
			if e := write(c, pair[1], "x"); e == nil {
				t.Fatal("collision accepted")
			}
			if e := write(c, "docs/after", "x"); !errors.Is(e, ErrPoisoned) {
				t.Fatal(e)
			}
		})
	}
}
func TestWriteFailuresPoison(t *testing.T) {
	tests := []struct {
		name    string
		content io.Reader
		length  int64
		hash    string
		setup   func(*memoryCandidate)
	}{
		{"short", strings.NewReader("a"), 2, digest("ab"), nil}, {"long", strings.NewReader("ab"), 1, digest("a"), nil}, {"digest", strings.NewReader("a"), 1, digest("b"), nil}, {"negative", strings.NewReader(""), -1, digest(""), nil}, {"file bound", strings.NewReader(""), MaxFileBytes + 1, digest(""), nil}, {"bad hash", strings.NewReader(""), 0, strings.Repeat("A", 64), nil}, {"create", strings.NewReader("a"), 1, digest("a"), func(m *memoryCandidate) { m.createFault = testFault }},
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			_, c, m := testCandidate(t)
			if tt.setup != nil {
				tt.setup(m)
			}
			if e := c.WriteFile(context.Background(), "app/a", tt.content, tt.length, tt.hash); e == nil {
				t.Fatal("failure accepted")
			}
			if e := write(c, "app/retry", "x"); !errors.Is(e, ErrPoisoned) {
				t.Fatal(e)
			}
			if e := c.Seal(context.Background(), []byte("receipt")); !errors.Is(e, ErrPoisoned) {
				t.Fatal(e)
			}
		})
	}
}
func TestCustodyFailurePoisons(t *testing.T) {
	_, c, m := testCandidate(t)
	m.fault = testFault
	if e := write(c, "app/a", "x"); !errors.Is(e, testFault) {
		t.Fatal(e)
	}
	m.fault = nil
	if e := write(c, "app/a", "x"); !errors.Is(e, ErrPoisoned) {
		t.Fatal(e)
	}
}
func TestSealRehashesHeldBytes(t *testing.T) {
	_, c, m := testCandidate(t)
	f := baselineData(t)
	writeCandidateEvidence(t, c, f)
	if err := c.CompleteRecoveryEvidence(context.Background(), f.identity); err != nil {
		t.Fatal(err)
	}
	m.files["app/opencpn.exe"].data[0] ^= 1
	if e := c.Seal(context.Background(), []byte("receipt")); e == nil {
		t.Fatal("changed held bytes accepted")
	}
	if _, ok := m.files[ReceiptFilename]; ok {
		t.Fatal("receipt created before verification")
	}
}
func TestSealBoundsAndEmptyPayload(t *testing.T) {
	for _, which := range []string{"empty payload", "empty receipt", "large receipt"} {
		t.Run(which, func(t *testing.T) {
			_, c, _ := testCandidate(t)
			receipt := []byte("x")
			if which != "empty payload" {
				if e := write(c, "app/a", ""); e != nil {
					t.Fatal(e)
				}
			}
			if which == "empty receipt" {
				receipt = nil
			}
			if which == "large receipt" {
				receipt = make([]byte, MaxReceiptBytes+1)
			}
			if e := c.Seal(context.Background(), receipt); e == nil {
				t.Fatal("invalid seal accepted")
			}
		})
	}
}
func TestOperationBounds(t *testing.T) {
	for _, kind := range []string{"files", "bytes"} {
		t.Run(kind, func(t *testing.T) {
			_, c, m := testCandidate(t)
			if kind == "files" {
				c.files = make([]fileRecord, MaxFiles)
				t.Cleanup(func() { c.mu.Lock(); c.files = nil; c.mu.Unlock() })
			} else {
				c.bytes = MaxPayloadBytes
			}
			if e := write(c, "app/a", "x"); e == nil {
				t.Fatal("operation bound exceeded")
			}
			if len(m.files) != 0 {
				t.Fatal("created beyond bound")
			}
		})
	}
}
func TestCancellationDuringHeldRehash(t *testing.T) {
	_, c, m := testCandidate(t)
	f := baselineData(t)
	writeCandidateEvidence(t, c, f)
	if err := c.CompleteRecoveryEvidence(context.Background(), f.identity); err != nil {
		t.Fatal(err)
	}
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	m.files["app/opencpn.exe"].onRead = cancel
	if e := c.Seal(ctx, []byte("receipt")); !errors.Is(e, context.Canceled) {
		t.Fatal(e)
	}
	if _, ok := m.files[ReceiptFilename]; ok {
		t.Fatal("receipt after cancellation")
	}
}

type cancelReader struct {
	cancel context.CancelFunc
	called bool
}

func (r *cancelReader) Read(p []byte) (int, error) {
	if r.called {
		return 0, io.EOF
	}
	r.called = true
	r.cancel()
	copy(p, "x")
	return 1, nil
}
func TestCancellationDuringStreaming(t *testing.T) {
	_, c, _ := testCandidate(t)
	ctx, cancel := context.WithCancel(context.Background())
	r := &cancelReader{cancel: cancel}
	if e := c.WriteFile(ctx, "app/a", r, 2, digest("xx")); !errors.Is(e, context.Canceled) {
		t.Fatal(e)
	}
}
func TestPersistentReservationAndClose(t *testing.T) {
	s, m := testStore()
	c, e := s.Begin(stageA, "client")
	if e != nil {
		t.Fatal(e)
	}
	if e = write(c, "app/a", "x"); e != nil {
		t.Fatal(e)
	}
	f := m.children[0].files["app/a"]
	f.closeFault = testFault
	m.children[0].closeFault = io.ErrClosedPipe
	m.fault = io.ErrUnexpectedEOF
	e = s.Close()
	for _, want := range []error{testFault, io.ErrClosedPipe, io.ErrUnexpectedEOF} {
		if !errors.Is(e, want) {
			t.Fatal(e)
		}
	}
	if f.closes != 1 || m.children[0].closes != 1 || m.closes != 1 {
		t.Fatal("independent cleanup skipped")
	}
	if next := s.Close(); next != e {
		t.Fatalf("repeat close lost original cleanup result: %v", next)
	}
	if e = write(c, "app/b", "x"); !errors.Is(e, ErrClosed) {
		t.Fatal(e)
	}
	m.fault = nil
	s2 := &Store{backend: m}
	defer s2.Close()
	if _, e = s2.Begin(stageA, "client"); e == nil {
		t.Fatal("abandoned reservation reused")
	}
}
func TestStoreAPIValidation(t *testing.T) {
	s, m := testStore()
	defer s.Close()
	for _, id := range []string{"", strings.ToUpper(stageA), stageA + "a", "../" + stageA} {
		if _, e := s.Begin(id, "client"); e == nil {
			t.Fatal("bad stage accepted")
		}
	}
	if _, _, e := s.VerifierDirectories("client", "../stable"); e == nil {
		t.Fatal("arbitrary channel")
	}
	m.fault = testFault
	if !errors.Is(s.Validate(), testFault) {
		t.Fatal("Validate hid failure")
	}
	if _, e := s.ReadRegistration("client"); !errors.Is(e, testFault) {
		t.Fatal(e)
	}
	s.Close()
	if !errors.Is(s.Validate(), ErrClosed) {
		t.Fatal("closed validation")
	}
	if _, _, e := s.VerifierDirectories("client", "beta"); !errors.Is(e, ErrClosed) {
		t.Fatal(e)
	}
}

func TestFinalWriteChecksPoison(t *testing.T) {
	for _, kind := range []string{"sync", "identity", "rehash", "final custody"} {
		t.Run(kind, func(t *testing.T) {
			_, c, m := testCandidate(t)
			m.configure = func(f *memoryFile) {
				switch kind {
				case "sync":
					f.syncFault = testFault
				case "identity":
					f.fault = testFault
				case "rehash":
					f.onRead = func() { f.data = []byte("different") }
				case "final custody":
					f.onRead = func() { m.fault = testFault }
				}
			}
			if err := write(c, "app/a", "original"); err == nil {
				t.Fatal("uncertain final write accepted")
			}
			m.fault = nil
			if err := write(c, "app/retry", "x"); !errors.Is(err, ErrPoisoned) {
				t.Fatal(err)
			}
		})
	}
}
