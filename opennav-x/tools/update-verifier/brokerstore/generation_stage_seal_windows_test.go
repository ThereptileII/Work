package brokerstore

import (
	"bytes"
	"context"
	"errors"
	"os"
	"runtime"
	"strings"
	"testing"

	"golang.org/x/sys/windows"
)

// These current-user fresh temporary fixtures bypass only production broker
// principal/fixed-root discovery. Actual CREATE_NEW, protected descriptors,
// Flush/readback, full IDs and no-write/delete sharing remain native.
type nativeStageSealFixtureStore struct {
	*nativeGenerationReopenFixtureStore
}

func (n *nativeStageSealFixtureStore) prepareStageSeal(ctx context.Context, backend generationBackend, source candidateBackend, record generationRecord, raw []byte) (stageSealWriter, error) {
	return createNativeStageSeal(ctx, backend.(*nativeReopenedGeneration), source.(*nativeCandidate), record, raw)
}
func (n *nativeStageSealFixtureStore) openStageSeal(ctx context.Context, backend generationBackend, record generationRecord, raw []byte) (stageSealReaderBackend, []byte, []byte, []byte, error) {
	return openNativeStageSeal(ctx, backend.(*nativeReopenedGeneration), n.sid, record, raw)
}
func nativeStageSealFixture(t *testing.T) (*Store, *Candidate, *nativeStageSealFixtureStore) {
	t.Helper()
	s, previous, view, f, _ := nativeBaselineViewFixture(t)
	view.nativeStore.candidates = view.candidates
	wrapper := &nativeStageSealFixtureStore{&nativeGenerationReopenFixtureStore{view}}
	s.backend = wrapper
	c := nativeHistoryCandidate(t, s, previous, f, strings.Repeat("9", 32))
	generation, err := s.MaterializeCandidateFresh(context.Background(), previous, c)
	if err != nil {
		t.Fatal(err)
	}
	if err = generation.Close(); err != nil {
		t.Fatal(err)
	}
	if err = previous.Close(); err != nil {
		t.Fatal(err)
	}
	return s, c, wrapper
}
func TestNativeGenerationStageSealFreshReceiptAndRetainedAncestry(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	s, c, _ := nativeStageSealFixture(t)
	original := []byte("original staged-awaiting-qualification receipt\n")
	if err := c.Seal(context.Background(), original); err != nil {
		t.Fatal(err)
	}
	if err := c.Close(); err != nil {
		t.Fatal(err)
	}
	owner, err := s.OpenReadOnlyGeneration(context.Background(), c.clientSID, c.stageID)
	if err != nil {
		t.Fatal(err)
	}
	reader, err := s.OpenGenerationStageSeal(context.Background(), owner)
	if err != nil {
		t.Fatal(err)
	}
	snapshot, err := reader.Snapshot(context.Background())
	if err != nil || !bytes.Equal(snapshot.Receipt, original) {
		t.Fatal("original receipt changed", err)
	}
	backend := reader.state.backend.(*nativeStageSeal)
	for _, file := range []*nativeFile{backend.pair.fence, backend.pair.journal, backend.receiptFile} {
		for _, access := range []uint32{windows.GENERIC_WRITE, windows.DELETE} {
			if err = nativeAttemptFile(t, physical(file.pin.name), access); !errors.Is(err, windows.ERROR_SHARING_VIOLATION) {
				t.Fatal("retained readonly seal sharing missing", err)
			}
		}
	}
	root := backend.source.directories[""]
	before := root.name
	moved := before + "-rename-probe"
	if err = os.Rename(physical(before), physical(moved)); err == nil {
		os.Rename(physical(moved), physical(before))
		t.Fatal("original source root renamed while retained")
	}
	if err = reader.Validate(context.Background()); err != nil {
		t.Fatal(err)
	}
	if err = reader.Close(); err != nil {
		t.Fatal(err)
	}
	if err = owner.Validate(); err != nil {
		t.Fatal("view close revoked generation", err)
	}
	if err = owner.Close(); err != nil {
		t.Fatal(err)
	}
	if err = os.Rename(physical(before), physical(moved)); err != nil {
		t.Fatal("source root not released", err)
	}
	if err = os.Rename(physical(moved), physical(before)); err != nil {
		t.Fatal(err)
	}
}
func TestNativeGenerationStageSealTamperPartialAndAliasRefuse(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	for _, fault := range []string{"receipt", "receipt-hardlink", "marker", "record", "pair-alias", "source-id", "partial"} {
		t.Run(fault, func(t *testing.T) {
			s, c, n := nativeStageSealFixture(t)
			component := stageSealComponent(c.clientSID, c.stageID)
			if fault == "partial" {
				owner, err := s.OpenReadOnlyGeneration(context.Background(), c.clientSID, c.stageID)
				if err != nil {
					t.Fatal(err)
				}
				writer, err := n.prepareStageSeal(context.Background(), owner.owner.generation.backend, c.backend, owner.owner.generation.record, owner.owner.generation.raw)
				if err != nil {
					t.Fatal(err)
				}
				if err = writer.close(); err != nil {
					t.Fatal(err)
				}
				owner.Close()
				if err = c.Seal(context.Background(), []byte("retry")); !errors.Is(err, ErrGenerationStageSealUncertain) {
					t.Fatal("partial was retried", err)
				}
			} else {
				if err := c.Seal(context.Background(), []byte("original receipt")); err != nil {
					t.Fatal(err)
				}
			}
			root := c.backend.(*nativeCandidate).directories[""].name
			receiptPath := physical(root + `\` + ReceiptFilename)
			if err := c.Close(); err != nil {
				t.Fatal(err)
			}
			switch fault {
			case "receipt":
				if err := os.WriteFile(receiptPath, []byte("changed receipt!"), 0600); err != nil {
					t.Fatal(err)
				}
			case "receipt-hardlink":
				if err := os.Link(receiptPath, receiptPath+"-hardlink"); err != nil {
					t.Fatal(err)
				}
			case "marker":
				if err := os.WriteFile(physical(n.registrations.name+`\`+component+".once"), []byte("partial"), 0600); err != nil {
					t.Fatal(err)
				}
			case "record":
				if err := os.WriteFile(physical(n.registrations.name+`\`+component+".json"), []byte("{}"), 0600); err != nil {
					t.Fatal(err)
				}
			case "pair-alias":
				if err := os.WriteFile(physical(n.registrations.name+`\`+component+".extra"), []byte("fork"), 0600); err != nil {
					t.Fatal(err)
				}
			}
			owner, err := s.OpenReadOnlyGeneration(context.Background(), c.clientSID, c.stageID)
			if err != nil {
				t.Fatal(err)
			}
			if fault == "source-id" {
				// Private fixture mutation changes the retained original expected ID, not
				// production provenance. The original tree must still be refused.
				owner.owner.generation.record.SourceDirectories["root-absent"] = baselineTestID(999)
			}
			if _, err = s.OpenGenerationStageSeal(context.Background(), owner); !errors.Is(err, ErrGenerationStageSealUncertain) {
				t.Fatal("changed/partial original adopted", fault, err)
			}
			if _, err = os.Stat(physical(n.registrations.name + `\` + component + ".once")); err != nil {
				t.Fatal("permanent refusal marker deleted", err)
			}
		})
	}
}
