package brokerstore

import (
	"context"
	"encoding/json"
	"errors"
	"testing"
)

type memoryApplicationBackend struct {
	*memoryExecutorBackend
	applications int
}

func (b *memoryApplicationBackend) matchApplicationProcess(ctx context.Context, pid uint32, created uint64) error {
	b.applications++
	return b.matchExecutorProcess(ctx, pid, created)
}
func applicationFixture(t *testing.T) (*Store, *ReadOnlyGeneration, *memoryApplicationBackend, *GenerationApplicationImage) {
	t.Helper()
	s, r, b := executorFixture(t)
	backend := &memoryApplicationBackend{memoryExecutorBackend: b}
	r.generation.backend = backend
	image, err := r.ApplicationImage(context.Background())
	if err != nil {
		t.Fatal(err)
	}
	return s, r, backend, image
}
func TestGenerationApplicationRequiresLiveFixedOriginalAndNativeProcess(t *testing.T) {
	_, r, b, image := applicationFixture(t)
	identity, err := image.Identity(context.Background())
	if err != nil {
		t.Fatal(err)
	}
	if identity.ClientSID != r.generation.record.ClientSID || identity.Generation != r.generation.record.Generation || identity.Commit != r.generation.record.Identity.Commit || identity.ExecutableSHA256 != r.generation.record.Members[generationApplicationMember].SHA256 {
		t.Fatal("image identity was declared independently of original")
	}
	for _, pair := range [][2]uint64{{0, 444}, {333, 0}, {334, 444}, {333, 445}} {
		if err = image.MatchesProcess(context.Background(), uint32(pair[0]), pair[1]); err == nil {
			t.Fatal("wrong native process accepted")
		}
	}
	if err = image.MatchesProcess(context.Background(), 333, 444); err != nil {
		t.Fatal("mismatch poisoned sound owner", err)
	}
	if b.applications != 3 {
		t.Fatal("invalid zero metadata reached native seam")
	}
	if _, err = json.Marshal(image); err == nil {
		t.Fatal("image serialized as authority")
	}
	if err = (&GenerationApplicationImage{}).MatchesProcess(context.Background(), 333, 444); !errors.Is(err, ErrClosed) {
		t.Fatal(err)
	}
}
func TestGenerationApplicationOriginalCustodyAndLifetimeRevokes(t *testing.T) {
	for _, fault := range []string{"store-close", "owner-close", "before-match", "during-match", "cancelled"} {
		t.Run(fault, func(t *testing.T) {
			s, r, b, image := applicationFixture(t)
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			switch fault {
			case "store-close":
				if err := s.Close(); err != nil {
					t.Fatal(err)
				}
			case "owner-close":
				if err := r.Close(); err != nil {
					t.Fatal(err)
				}
			case "before-match":
				b.readers[generationApplicationMember].data[0] ^= 1
			case "during-match":
				b.after = func() { b.readers[generationApplicationMember].data[0] ^= 1 }
			case "cancelled":
				cancel()
			}
			if err := image.MatchesProcess(ctx, 333, 444); err == nil {
				t.Fatal("revoked/changed original accepted")
			}
			if fault != "during-match" && b.applications != 0 {
				t.Fatal("invalid custody reached native image check")
			}
			if _, err := image.Identity(ctx); err == nil {
				t.Fatal("revoked image identity remained usable")
			}
		})
	}
}
func TestGenerationApplicationNonNativeCannotMintAndBaselineSharesOwner(t *testing.T) {
	_, g, _ := readOnlyFixture(t)
	r, err := g.TransitionReadOnly(context.Background())
	if err != nil {
		t.Fatal(err)
	}
	if _, err = r.ApplicationImage(context.Background()); !errors.Is(err, ErrNativeRequired) {
		t.Fatal("non-native minted image", err)
	}
	_, r, _, _ = applicationFixture(t)
	baseline := &ReadOnlyApprovedBaselineGeneration{owner: r}
	image, err := baseline.ApplicationImage(context.Background())
	if err != nil {
		t.Fatal(err)
	}
	if err = baseline.Close(); err != nil {
		t.Fatal(err)
	}
	if _, err = image.Identity(context.Background()); !errors.Is(err, ErrClosed) {
		t.Fatal("baseline wrapper copied custody", err)
	}
}
