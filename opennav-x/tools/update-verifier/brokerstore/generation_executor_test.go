package brokerstore

import (
	"context"
	"errors"
	"testing"
)

type memoryExecutorBackend struct {
	*readOnlyMemoryBackend
	calls int
	after func()
}

func (b *memoryExecutorBackend) matchExecutorProcess(ctx context.Context, pid uint32, created uint64) error {
	b.calls++
	if err := ctx.Err(); err != nil {
		return err
	}
	if pid != 333 || created != 444 {
		return testFault
	}
	if b.after != nil {
		b.after()
	}
	return nil
}

func executorFixture(t *testing.T) (*Store, *ReadOnlyGeneration, *memoryExecutorBackend) {
	t.Helper()
	s, g, b := readOnlyFixture(t)
	backend := &memoryExecutorBackend{readOnlyMemoryBackend: b}
	g.backend = backend
	r, err := g.TransitionReadOnly(context.Background())
	if err != nil {
		t.Fatal(err)
	}
	return s, r, backend
}

func TestGenerationExecutorRejectsForgedIdentityWithoutRevokingOriginal(t *testing.T) {
	_, r, backend := executorFixture(t)
	defer r.Close()
	image, err := r.ExecutorImage(context.Background())
	if err != nil {
		t.Fatal(err)
	}
	for _, pair := range [][2]uint64{{0, 444}, {333, 0}, {334, 444}, {333, 445}} {
		if err := image.MatchesProcess(context.Background(), uint32(pair[0]), pair[1]); err == nil {
			t.Fatal("different process/creation accepted", pair)
		}
	}
	if err := image.MatchesProcess(context.Background(), 333, 444); err != nil {
		t.Fatal("mismatched peer poisoned sound generation", err)
	}
	if backend.calls != 3 {
		t.Fatal("invalid zero identity reached native seam", backend.calls)
	}
	if err := (&GenerationExecutorImage{}).MatchesProcess(context.Background(), 333, 444); !errors.Is(err, ErrClosed) {
		t.Fatal("zero-value serialized image became a capability", err)
	}
}

func TestGenerationExecutorLifetimeAndCustodyRemainBound(t *testing.T) {
	for _, fault := range []string{"owner-close", "store-close", "before-compare", "during-compare", "cancelled"} {
		t.Run(fault, func(t *testing.T) {
			s, r, backend := executorFixture(t)
			defer s.Close()
			image, err := r.ExecutorImage(context.Background())
			if err != nil {
				t.Fatal(err)
			}
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			switch fault {
			case "owner-close":
				if err := r.Close(); err != nil {
					t.Fatal(err)
				}
			case "store-close":
				if err := s.Close(); err != nil {
					t.Fatal(err)
				}
			case "before-compare":
				backend.readers[generationExecutorMember].data[0] ^= 1
			case "during-compare":
				backend.after = func() { backend.readers[generationExecutorMember].data[0] ^= 1 }
			case "cancelled":
				cancel()
			}
			if err := image.MatchesProcess(ctx, 333, 444); err == nil {
				t.Fatal("revoked/changed original retained authority", fault)
			}
			if fault != "during-compare" && backend.calls != 0 {
				t.Fatal("refused original reached native comparison")
			}
		})
	}
}

func TestGenerationExecutorCannotMintFromNonNativeOwner(t *testing.T) {
	_, g, _ := readOnlyFixture(t)
	r, err := g.TransitionReadOnly(context.Background())
	if err != nil {
		t.Fatal(err)
	}
	defer r.Close()
	if _, err := r.ExecutorImage(context.Background()); !errors.Is(err, ErrNativeRequired) {
		t.Fatal("non-native retained generation minted executor image", err)
	}
}
