package brokerstore

import (
	"bytes"
	"context"
	"errors"
	"io"
	"sort"
	"testing"
)

// The portable seam models separate file objects, including removal before the
// sole writer-close attempt. Native fixtures prove actual Windows sharing.
type readOnlyMemoryBackend struct {
	*memoryGeneration
	old, readers map[string]*memoryFile
	attempts     int
	fault        string
	cancel       context.CancelFunc
}
type memoryMemberReader struct{ file *memoryFile }

func (r memoryMemberReader) ReadAt(p []byte, off int64) (int, error) {
	if off < 0 {
		return 0, testFault
	}
	if off >= int64(len(r.file.data)) {
		return 0, io.EOF
	}
	n := copy(p, r.file.data[off:])
	if n < len(p) {
		return n, io.EOF
	}
	return n, nil
}
func (b *readOnlyMemoryBackend) transitionReadOnly(ctx context.Context, files map[string]candidateFile, members map[string]baselineMember) error {
	b.attempts++
	names := make([]string, 0, len(files))
	for name := range files {
		names = append(names, name)
	}
	sort.Strings(names)
	for _, name := range names {
		if err := ctx.Err(); err != nil {
			return err
		}
		old := b.old[name]
		delete(files, name)
		delete(b.files, name)
		if b.fault == "writer-close" {
			old.closeFault = testFault
		}
		if err := old.Close(); err != nil {
			return err
		}
		if b.fault == "after-close" {
			return testFault
		}
		next := &memoryFile{data: bytes.Clone(old.data)}
		b.readers[name] = next
		b.files[name] = next
		files[name] = &generationReadOnlyFile{candidateFile: next, reader: memoryMemberReader{next}}
		if b.fault == "after-reopen" {
			return testFault
		}
		if b.fault == "tampered-reopen" {
			next.data[0] ^= 1
		}
		if b.cancel != nil {
			b.cancel()
			return ctx.Err()
		}
	}
	return nil
}
func readOnlyFixture(t *testing.T) (*Store, *InstalledGeneration, *readOnlyMemoryBackend) {
	t.Helper()
	s, p, c, b, _ := generationFixture(t)
	g, err := s.MaterializeCandidateFresh(context.Background(), p, c)
	if err != nil {
		t.Fatal(err)
	}
	backend := &readOnlyMemoryBackend{memoryGeneration: b, old: map[string]*memoryFile{}, readers: map[string]*memoryFile{}}
	for name, f := range b.files {
		backend.old[name] = f
	}
	g.backend = backend
	return s, g, backend
}
func TestGenerationReadOnlyTransfersOriginalsAndRevokesWriter(t *testing.T) {
	_, g, b := readOnlyFixture(t)
	before, err := g.Completion()
	if err != nil {
		t.Fatal(err)
	}
	raw := bytes.Clone(g.raw)
	r, err := g.TransitionReadOnly(context.Background())
	if err != nil {
		t.Fatal(err)
	}
	if !bytes.Equal(raw, g.raw) {
		t.Fatal("transition rewrote completion")
	}
	after, err := r.Completion()
	if err != nil || after != before {
		t.Fatal("original completion changed", err)
	}
	if !errors.Is(g.Validate(), ErrGenerationTransferred) {
		t.Fatal("writer retained authority")
	}
	if _, err = g.Completion(); !errors.Is(err, ErrGenerationTransferred) {
		t.Fatal("writer summary retained authority", err)
	}
	if _, err = g.TransitionReadOnly(context.Background()); !errors.Is(err, ErrGenerationTransferred) {
		t.Fatal("transition repeated", err)
	}
	for name, old := range b.old {
		if old.closes != 1 {
			t.Fatal("writer not closed once", name, old.closes)
		}
		h, err := r.OpenMember(name)
		if err != nil {
			t.Fatal(err)
		}
		id, err := h.Identity()
		if err != nil || id.RelativePath != name || id.SHA256 != registrationDigest(old.data) || id.Length != int64(len(old.data)) {
			t.Fatal("identity changed", name, err)
		}
		got := make([]byte, len(old.data)+1)
		n, err := h.ReadAt(got, 0)
		if n != len(old.data) || !errors.Is(err, io.EOF) || !bytes.Equal(got[:n], old.data) {
			t.Fatal("original read differs", name, err)
		}
		if _, err = g.files[name].Write([]byte("mutation")); !errors.Is(err, ErrGenerationReadOnly) {
			t.Fatal("readonly file permits write")
		}
		if err = h.Close(); err != nil {
			t.Fatal(err)
		}
		if err = h.Close(); err != nil {
			t.Fatal(err)
		}
		if _, err = h.ReadAt(got, 0); !errors.Is(err, ErrClosed) {
			t.Fatal("closed view readable")
		}
	}
	for _, name := range []string{"../app/opencpn.exe", "app\\opencpn.exe", "APP/opencpn.exe", "app/unknown"} {
		if _, err = r.OpenMember(name); err == nil {
			t.Fatal("nonexact member accepted", name)
		}
	}
	if err = r.Validate(); err != nil {
		t.Fatal(err)
	}
	if err = r.Close(); err != nil {
		t.Fatal(err)
	}
	if err = g.Close(); err != nil {
		t.Fatal(err)
	}
	for name, f := range b.readers {
		if f.closes != 1 {
			t.Fatal("reader close repeated", name, f.closes)
		}
	}
	if b.closes != 1 || b.attempts != 1 {
		t.Fatal("backend cleanup/retry differs", b.closes, b.attempts)
	}
}
func TestGenerationReadOnlyFailedTransitionDrainsOnceWithoutRetry(t *testing.T) {
	for _, fault := range []string{"cancel-before", "writer-close", "after-close", "after-reopen", "tampered-reopen", "cancel-partial"} {
		t.Run(fault, func(t *testing.T) {
			_, g, b := readOnlyFixture(t)
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			if fault == "cancel-before" {
				cancel()
			} else if fault == "cancel-partial" {
				b.cancel = cancel
			} else {
				b.fault = fault
			}
			r, err := g.TransitionReadOnly(ctx)
			if r != nil || !errors.Is(err, ErrInstalledGenerationUncertain) {
				t.Fatal("failed transition not uncertain", err)
			}
			if fault == "cancel-before" || fault == "cancel-partial" {
				if !errors.Is(err, context.Canceled) {
					t.Fatal("cancellation lost", err)
				}
			}
			attempts := b.attempts
			if _, err = g.TransitionReadOnly(context.Background()); err == nil || b.attempts != attempts {
				t.Fatal("failed transition retried")
			}
			first := g.Close()
			if first == nil || g.Close() != first {
				t.Fatal("cleanup result not retained")
			}
			for name, f := range b.old {
				if f.closes != 1 {
					t.Fatal("writer close not exact once", name, f.closes)
				}
			}
			for name, f := range b.readers {
				if f.closes != 1 {
					t.Fatal("reader close not exact once", name, f.closes)
				}
			}
			if b.closes != 1 || !b.completed || len(g.raw) == 0 {
				t.Fatal("cleanup erased durable completion or repeated")
			}
		})
	}
}
func TestGenerationReadOnlyOwnerRevocationBoundsAndPoison(t *testing.T) {
	for _, scenario := range []string{"writer-close", "readonly-close", "store-close", "tamper", "check-fault", "cleanup-fault"} {
		t.Run(scenario, func(t *testing.T) {
			s, g, b := readOnlyFixture(t)
			r, err := g.TransitionReadOnly(context.Background())
			if err != nil {
				t.Fatal(err)
			}
			h, err := r.OpenMember("app/opencpn.exe")
			if err != nil {
				t.Fatal(err)
			}
			if _, err = h.ReadAt(make([]byte, maxGenerationMemberRead+1), 0); err == nil {
				t.Fatal("unbounded read")
			}
			if _, err = h.ReadAt(make([]byte, 1), -1); err == nil {
				t.Fatal("negative read")
			}
			if err = r.Validate(); err != nil {
				t.Fatal("invalid read poisoned owner", err)
			}
			switch scenario {
			case "writer-close":
				err = g.Close()
			case "readonly-close":
				err = r.Close()
			case "store-close":
				err = s.Close()
			case "tamper":
				b.readers["app/opencpn.exe"].data[0] ^= 1
				err = r.Validate()
			case "check-fault":
				b.readers["app/opencpn.exe"].fault = testFault
				_, err = h.ReadAt(make([]byte, 1), 0)
			case "cleanup-fault":
				b.readers["app/opencpn.exe"].closeFault = testFault
				err = r.Close()
				if !errors.Is(err, testFault) || !errors.Is(err, ErrInstalledGenerationUncertain) {
					t.Fatal("cleanup uncertainty lost", err)
				}
				if r.Close() != err {
					t.Fatal("close not cached")
				}
			}
			if (scenario == "tamper" || scenario == "check-fault") && err == nil {
				t.Fatal("custody mutation accepted")
			}
			if _, err = h.ReadAt(make([]byte, 1), 0); err == nil {
				t.Fatal("revoked or poisoned view readable")
			}
			if _, err = h.Identity(); err == nil {
				t.Fatal("revoked or poisoned view identity accepted")
			}
			h.Close()
			h.Close()
		})
	}
}
