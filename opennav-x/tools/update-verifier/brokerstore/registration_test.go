package brokerstore

import (
	"bytes"
	"context"
	"errors"
	"io"
	"reflect"
	"strings"
	"testing"
)

const registrationTestSID = "S-1-5-21-123-456-789-1001"

var registrationTestApproval = strings.Repeat("a", 64)

type recordingRegistration struct {
	events                                      []string
	failAt                                      int
	fault, cleanup                              error
	reserved, user, trust, artifacts, completed bool
	closes                                      int
	completeData                                []byte
	directoriesFault                            error
}

func (r *recordingRegistration) step(name string) error {
	r.events = append(r.events, name)
	if len(r.events) == r.failAt {
		return r.fault
	}
	return nil
}
func (r *recordingRegistration) check() error { return r.step("check") }
func (r *recordingRegistration) reserve(ctx context.Context) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	if err := r.step("reserve-sync"); err != nil {
		return err
	}
	r.reserved = true
	return nil
}
func (r *recordingRegistration) createUser() error {
	if !r.reserved {
		panic("client tree before durable reservation")
	}
	if err := r.step("user"); err != nil {
		return err
	}
	r.user = true
	return nil
}
func (r *recordingRegistration) createTrust() error {
	if !r.user {
		panic("trust before fresh user")
	}
	if err := r.step("trust"); err != nil {
		return err
	}
	r.trust = true
	return nil
}
func (r *recordingRegistration) createArtifacts() error {
	if !r.trust {
		panic("artifacts before fresh trust parent")
	}
	if err := r.step("artifacts"); err != nil {
		return err
	}
	r.artifacts = true
	return nil
}
func (r *recordingRegistration) directories() (string, string, error) {
	if r.directoriesFault != nil {
		return "", "", r.directoriesFault
	}
	return "fixed-absent-channel", "fixed-existing-artifacts", nil
}
func (r *recordingRegistration) complete(ctx context.Context, data []byte) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	if !(r.reserved && r.user && r.trust && r.artifacts) {
		panic("completion before fresh tree")
	}
	if err := r.step("complete"); err != nil {
		return err
	}
	r.completeData = append([]byte(nil), data...)
	r.completed = true
	return nil
}
func (r *recordingRegistration) close() error {
	r.closes++
	r.events = append(r.events, "close")
	return r.cleanup
}

type registrationMemoryStore struct {
	*memoryStore
	creation registrationBackend
	prepares int
}

func (s *registrationMemoryStore) prepareRegistration(_, _, _ string) (registrationBackend, error) {
	s.prepares++
	return s.creation, nil
}
func registrationOwnerFixture(t *testing.T) (*Store, *Registration, *recordingRegistration) {
	t.Helper()
	b := &recordingRegistration{fault: testFault}
	s := &Store{backend: &registrationMemoryStore{memoryStore: &memoryStore{}, creation: b}}
	r, err := s.BeginRegistrationFresh(context.Background(), registrationTestSID, "beta", registrationTestApproval)
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { s.Close() })
	return s, r, b
}

func TestRegistrationPermanentReservationPrecedesClientTree(t *testing.T) {
	b := &recordingRegistration{}
	if err := beginRegistration(context.Background(), b); err != nil {
		t.Fatal(err)
	}
	want := []string{"check", "reserve-sync", "check", "user", "check", "trust", "check", "artifacts", "check"}
	if !reflect.DeepEqual(b.events, want) || b.completed || b.closes != 0 {
		t.Fatalf("creation order/ownership: %+v", b)
	}
}
func TestRegistrationEveryBeginFaultStopsRetainsPartialStateAndJoinsClose(t *testing.T) {
	cleanup := errors.New("uncertain close")
	for failure := 1; failure <= 9; failure++ {
		b := &recordingRegistration{failAt: failure, fault: testFault, cleanup: cleanup}
		err := beginRegistration(context.Background(), b)
		if !errors.Is(err, testFault) || !errors.Is(err, cleanup) {
			t.Fatalf("step %d lost errors: %v", failure, err)
		}
		if len(b.events) != failure+1 || b.events[failure] != "close" || b.closes != 1 {
			t.Fatalf("step %d continued: %v", failure, b.events)
		}
		if b.reserved != (failure > 2) || b.user != (failure > 4) || b.trust != (failure > 6) || b.artifacts != (failure > 8) || b.completed {
			t.Fatalf("step %d erased/premature state: %+v", failure, b)
		}
	}
}
func TestRegistrationUnsupportedAndInvalidBeginDoNotCreate(t *testing.T) {
	s, _ := testStore()
	if _, err := s.BeginRegistrationFresh(context.Background(), registrationTestSID, "beta", registrationTestApproval); !errors.Is(err, ErrNativeRequired) {
		t.Fatal(err)
	}
	b := &registrationMemoryStore{memoryStore: &memoryStore{}, creation: &recordingRegistration{}}
	s = &Store{backend: b}
	for _, tc := range []struct {
		ctx                context.Context
		sid, channel, hash string
	}{
		{nil, registrationTestSID, "beta", registrationTestApproval},
		{context.Background(), "", "beta", registrationTestApproval},
		{context.Background(), registrationTestSID, "Beta", registrationTestApproval},
		{context.Background(), registrationTestSID, "beta", strings.Repeat("A", 64)},
	} {
		if _, err := s.BeginRegistrationFresh(tc.ctx, tc.sid, tc.channel, tc.hash); err == nil {
			t.Fatal("invalid begin accepted")
		}
	}
	ctx, cancel := context.WithCancel(context.Background())
	cancel()
	if _, err := s.BeginRegistrationFresh(ctx, registrationTestSID, "beta", registrationTestApproval); !errors.Is(err, context.Canceled) {
		t.Fatal(err)
	}
	if b.prepares != 0 {
		t.Fatal("invalid input consumed native state")
	}
}
func TestRegistrationOwnerCompletionAndStoreCloseDrainOnce(t *testing.T) {
	s, r, b := registrationOwnerFixture(t)
	state, artifacts, err := r.VerifierDirectories()
	if err != nil || state != "fixed-absent-channel" || artifacts != "fixed-existing-artifacts" {
		t.Fatalf("paths: %q %q %v", state, artifacts, err)
	}
	raw := []byte(`{"approved":"opaque to storage"}`)
	if err = r.Complete(context.Background(), raw); err != nil {
		t.Fatal(err)
	}
	raw[0] = 'x'
	if b.completeData[0] != '{' {
		t.Fatal("borrowed input retained")
	}
	if err = r.Complete(context.Background(), []byte("different")); err == nil {
		t.Fatal("completion repeated")
	}
	cleanup := errors.New("retained owner close uncertainty")
	b.cleanup = cleanup
	for _, close := range []func() error{s.Close, r.Close, r.Close, s.Close} {
		if err = close(); !errors.Is(err, cleanup) {
			t.Fatalf("close uncertainty lost: %v", err)
		}
	}
	if b.closes != 1 {
		t.Fatalf("owner drained %d times", b.closes)
	}
	if _, _, err = r.VerifierDirectories(); !errors.Is(err, ErrClosed) {
		t.Fatal(err)
	}
}
func TestRegistrationCompletionFailurePoisonsOwner(t *testing.T) {
	for _, mode := range []string{"nil-context", "cancelled", "empty", "oversize", "native-fault", "directory-fault"} {
		t.Run(mode, func(t *testing.T) {
			_, r, b := registrationOwnerFixture(t)
			ctx := context.Background()
			data := []byte("approved")
			switch mode {
			case "nil-context":
				ctx = nil
			case "cancelled":
				var cancel context.CancelFunc
				ctx, cancel = context.WithCancel(ctx)
				cancel()
			case "empty":
				data = nil
			case "oversize":
				data = make([]byte, MaxRegistrationBytes+1)
			case "native-fault":
				b.failAt = len(b.events) + 2
			case "directory-fault":
				b.directoriesFault = testFault
				if _, _, err := r.VerifierDirectories(); !errors.Is(err, testFault) {
					t.Fatal(err)
				}
			}
			if err := r.Complete(ctx, data); err == nil {
				t.Fatal("completion fault accepted")
			}
			if err := r.Complete(context.Background(), []byte("retry")); !errors.Is(err, ErrPoisoned) {
				t.Fatalf("owner retry: %v", err)
			}
			if b.completed {
				t.Fatal("failed owner completed")
			}
		})
	}
}
func TestRegistrationMarkerRequiresExactCompleteClientAndBoundHash(t *testing.T) {
	raw := []byte(`{"schema":1}`)
	digest := registrationDigest(raw)
	complete := registrationMarker(registrationTestSID, "beta", registrationTestApproval, digest, true)
	started := registrationMarker(registrationTestSID, "beta", registrationTestApproval, "", false)
	if len(started) != len(complete) {
		t.Fatal("completion changes marker width")
	}
	if got, err := completedRegistrationDigest(complete, registrationTestSID); err != nil || got != digest {
		t.Fatalf("complete: %q %v", got, err)
	}
	for _, bad := range [][]byte{started, complete[:len(complete)-1], append(append([]byte(nil), complete...), 0), bytes.Replace(complete, []byte("\nbeta\n"), []byte("\nBeta\n"), 1), bytes.Replace(complete, []byte(registrationTestApproval), []byte(strings.Repeat("A", 64)), 1)} {
		if _, err := completedRegistrationDigest(bad, registrationTestSID); err == nil {
			t.Fatalf("bad marker accepted: %q", bad)
		}
	}
	if _, err := completedRegistrationDigest(complete, "another-client"); err == nil {
		t.Fatal("other client's completion accepted")
	}
}

type registrationFileFixture struct {
	data           []byte
	position       int64
	events         []string
	failAt         int
	short, corrupt bool
	onEvent        func(string)
}

func (f *registrationFileFixture) step(name string) error {
	f.events = append(f.events, name)
	if f.onEvent != nil {
		f.onEvent(name)
	}
	if len(f.events) == f.failAt {
		return testFault
	}
	return nil
}
func (f *registrationFileFixture) Read(p []byte) (int, error) {
	if err := f.step("read"); err != nil {
		return 0, err
	}
	if f.position >= int64(len(f.data)) {
		return 0, io.EOF
	}
	n := copy(p, f.data[f.position:])
	f.position += int64(n)
	if f.corrupt && n > 0 {
		p[0] ^= 1
	}
	return n, nil
}
func (f *registrationFileFixture) Write(p []byte) (int, error) {
	if err := f.step("write"); err != nil {
		return 0, err
	}
	n := len(p)
	if f.short {
		n--
	}
	end := f.position + int64(n)
	if end > int64(len(f.data)) {
		f.data = append(f.data, make([]byte, end-int64(len(f.data)))...)
	}
	copy(f.data[f.position:end], p[:n])
	f.position = end
	return n, nil
}
func (f *registrationFileFixture) Seek(offset int64, whence int) (int64, error) {
	if err := f.step("seek"); err != nil {
		return 0, err
	}
	if whence != io.SeekStart {
		return 0, testFault
	}
	f.position = offset
	return offset, nil
}
func (f *registrationFileFixture) Sync() error  { return f.step("sync") }
func (f *registrationFileFixture) Close() error { return f.step("close") }
func (f *registrationFileFixture) check(n int64) error {
	if err := f.step("file-check"); err != nil {
		return err
	}
	if int64(len(f.data)) != n {
		return testFault
	}
	return nil
}
func TestRegistrationWriteFlushSameHandleReadbackAndEveryIOFault(t *testing.T) {
	data := []byte("approved registration")
	good := &registrationFileFixture{}
	check := func() error { return good.step("tree-check") }
	if err := writeRegistrationBytes(context.Background(), good, 0, data, check); err != nil {
		t.Fatal(err)
	}
	want := []string{"tree-check", "file-check", "seek", "write", "sync", "file-check", "seek", "read", "read", "file-check", "tree-check"}
	if !reflect.DeepEqual(good.events, want) || !bytes.Equal(good.data, data) {
		t.Fatalf("write order: %v", good.events)
	}
	for failure := 1; failure <= len(want); failure++ {
		f := &registrationFileFixture{failAt: failure}
		err := writeRegistrationBytes(context.Background(), f, 0, data, func() error { return f.step("tree-check") })
		if !errors.Is(err, testFault) || len(f.events) != failure {
			t.Fatalf("I/O fault %d continued/lost: %v %v", failure, err, f.events)
		}
	}
	for _, f := range []*registrationFileFixture{{short: true}, {corrupt: true}} {
		if err := writeRegistrationBytes(context.Background(), f, 0, data, func() error { return nil }); err == nil {
			t.Fatal("short/tampered held bytes accepted")
		}
	}
	// Completion replaces fixed-width bytes through the same owner, not a rename.
	f := &registrationFileFixture{data: []byte("started")}
	if err := writeRegistrationBytes(context.Background(), f, 7, []byte("done!!!"), func() error { return nil }); err != nil || string(f.data) != "done!!!" {
		t.Fatalf("marker overwrite: %q %v", f.data, err)
	}
}

type committingRegistration struct {
	*recordingRegistration
	marker            *registrationFileFixture
	cancelAfterCommit context.CancelFunc
}

func (b *committingRegistration) complete(ctx context.Context, raw []byte) error {
	marker := registrationMarker(registrationTestSID, "beta", registrationTestApproval, registrationDigest(raw), true)
	if err := publishRegistrationCompletion(ctx, b.marker, int64(len(b.marker.data)), marker, b.check); err != nil {
		return err
	}
	b.completed = true
	if b.cancelAfterCommit != nil {
		b.cancelAfterCommit()
	}
	return nil
}
func TestRegistrationCompletionCommitIgnoresCancellationDuringAndAfterPublication(t *testing.T) {
	for _, point := range []string{"write", "sync", "read", "after-commit"} {
		t.Run(point, func(t *testing.T) {
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			marker := &registrationFileFixture{data: registrationMarker(registrationTestSID, "beta", registrationTestApproval, "", false)}
			b := &committingRegistration{recordingRegistration: &recordingRegistration{}, marker: marker}
			if point == "after-commit" {
				b.cancelAfterCommit = cancel
			} else {
				marker.onEvent = func(event string) {
					if event == point {
						cancel()
					}
				}
			}
			s := &Store{backend: &registrationMemoryStore{memoryStore: &memoryStore{}, creation: b}}
			r, err := s.BeginRegistrationFresh(ctx, registrationTestSID, "beta", registrationTestApproval)
			if err != nil {
				t.Fatal(err)
			}
			defer s.Close()
			raw := []byte(`{"approved":"committed"}`)
			if err = r.Complete(ctx, raw); err != nil {
				t.Fatalf("durable commit relabeled failed after %s cancellation: %v", point, err)
			}
			if !errors.Is(ctx.Err(), context.Canceled) || !r.completed || r.poisoned {
				t.Fatalf("committed owner state: cancelled=%v completed=%v poisoned=%v", ctx.Err(), r.completed, r.poisoned)
			}
			if got, err := completedRegistrationDigest(marker.data, registrationTestSID); err != nil || got != registrationDigest(raw) {
				t.Fatalf("committed evidence differs: %q %v", got, err)
			}
			if _, _, err = r.VerifierDirectories(); err != nil {
				t.Fatalf("successful committed owner poisoned: %v", err)
			}
		})
	}
}
func TestRegistrationCompletionPrecommitCancellationAndPublicationUncertainty(t *testing.T) {
	started := registrationMarker(registrationTestSID, "beta", registrationTestApproval, "", false)
	raw := []byte(`{"approved":"exact"}`)
	completed := registrationMarker(registrationTestSID, "beta", registrationTestApproval, registrationDigest(raw), true)
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	before := &registrationFileFixture{data: append([]byte(nil), started...)}
	before.onEvent = func(event string) {
		if event == "seek" {
			cancel()
		}
	}
	if err := publishRegistrationCompletion(ctx, before, int64(len(started)), completed, func() error { return nil }); !errors.Is(err, context.Canceled) || errors.Is(err, ErrRegistrationUncertain) {
		t.Fatalf("precommit cancellation classification: %v", err)
	}
	if !bytes.Equal(before.data, started) {
		t.Fatal("precommit cancellation published marker")
	}
	if _, err := completedRegistrationDigest(before.data, registrationTestSID); err == nil {
		t.Fatal("precommit cancellation became readable completion")
	}
	// A custody failure AFTER a successful write/flush may coexist with valid
	// completed evidence. Preserve the underlying cause and never claim rollback.
	file := &registrationFileFixture{data: append([]byte(nil), started...)}
	checks := 0
	check := func() error {
		checks++
		if checks == 2 {
			return testFault
		}
		return nil
	}
	err := publishRegistrationCompletion(context.Background(), file, int64(len(started)), completed, check)
	if !errors.Is(err, ErrRegistrationUncertain) || !errors.Is(err, testFault) {
		t.Fatalf("postpublication uncertainty lost: %v", err)
	}
	if got, parseErr := completedRegistrationDigest(file.data, registrationTestSID); parseErr != nil || got != registrationDigest(raw) {
		t.Fatalf("expected potentially durable completion evidence: %q %v", got, parseErr)
	}
	for _, kind := range []string{"write", "sync", "read"} {
		t.Run(kind, func(t *testing.T) {
			f := &registrationFileFixture{data: append([]byte(nil), started...)}
			f.onEvent = func(event string) {
				if event == kind {
					f.failAt = len(f.events)
				}
			}
			err := publishRegistrationCompletion(context.Background(), f, int64(len(started)), completed, func() error { return nil })
			if !errors.Is(err, ErrRegistrationUncertain) || !errors.Is(err, testFault) {
				t.Fatalf("%s publication failure not uncertain: %v", kind, err)
			}
		})
	}
}
