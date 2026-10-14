//go:build windows

package brokerstore

import (
	"bytes"
	"context"
	"errors"
	"os"
	"path/filepath"
	"runtime"
	"strings"
	"testing"

	"golang.org/x/sys/windows"
)

// Only principal and Applications discovery use the existing temporary native
// fixture factories. Production and this fixture share original Repair source
// acquisition; materialization/copy/hash/publication and strict reopen are real.
// No SCM, installed roots, account changes, profile writes or launch are involved.
type nativeRepairNTFSStore struct {
	*nativeGenerationReopenFixtureStore
}
type nativeRepairNTFSSource struct {
	*nativeRepairSource
	fixture *nativeRepairNTFSStore
}

func (s *nativeRepairNTFSStore) openRepairSource(ctx context.Context, client, generation string) (repairSourceBackend, generationRecord, []byte, generationRecord, []byte, error) {
	source, current, raw, original, originalRaw, err := openNativeRepairSource(ctx, s.nativeStore, s.sid, client, generation, s.openReadOnlyGeneration)
	if err != nil {
		return source, current, raw, original, originalRaw, err
	}
	return &nativeRepairNTFSSource{source.(*nativeRepairSource), s}, current, raw, original, originalRaw, nil
}
func (s *nativeRepairNTFSSource) create(ctx context.Context, generation string) (generationBackend, error) {
	if err := errors.Join(ctx.Err(), s.check()); err != nil {
		return nil, err
	}
	fixture := s.fixture
	g, err := prepareNativeInstalledGeneration(fixture.nativeStore, nil, fixture.sid, s.current.client, generation)
	if err != nil {
		return nil, err
	}
	g.applications, err = openPin(physical(fixture.applications.name), fixture.applications.name, fixture.applications.security)
	if err != nil {
		return nil, errors.Join(err, g.close())
	}
	g.ancestors = append(g.ancestors, g.applications)
	if err = g.pin(g.applications); err != nil {
		return nil, errors.Join(err, g.close())
	}
	return g, nil
}

type nativeRepairNTFSFixture struct {
	store                               *Store
	backend                             *nativeRepairNTFSStore
	client, generation, public, private string
	data                                baselineFixtureData
	original                            generationRecord
}

func repairNTFSFixture(t *testing.T, purpose GenerationPurpose) nativeRepairNTFSFixture {
	t.Helper()
	s, p, view, f, raw := nativeBaselineViewFixture(t)
	view.nativeStore.candidates = view.candidates
	var generation, private string
	var record generationRecord
	if purpose == GenerationPurposeApprovedBaseline {
		owner, err := s.MaterializeApprovedBaselineReadOnlyFresh(context.Background(), p, raw, registrationDigest(raw))
		if err != nil {
			t.Fatal(err)
		}
		generation, record = stageA, owner.owner.generation.record
		private = physical(p.backend.(*nativeInstalledBaseline).tree.directories["app"].name + `\opencpn.exe`)
	} else {
		s.backend = &nativeStageSealFixtureStore{&nativeGenerationReopenFixtureStore{view}}
		generation = strings.Repeat("9", 32)
		c := nativeHistoryCandidate(t, s, p, f, generation)
		owner, err := s.MaterializeCandidateFresh(context.Background(), p, c)
		if err != nil {
			t.Fatal(err)
		}
		record = owner.record
		private = physical(c.backend.(*nativeCandidate).directories["app"].name + `\opencpn.exe`)
		if err = owner.Close(); err != nil {
			t.Fatal(err)
		}
		if err = c.Seal(context.Background(), []byte("original inert stage receipt")); err != nil {
			t.Fatal(err)
		}
		if err = c.Close(); err != nil {
			t.Fatal(err)
		}
	}
	client := p.record.ClientSID
	public := filepath.Join(physical(view.applications.name), client, "generations", generation)
	reopened := restartNativeGenerationFixture(t, s, view)
	base := reopened.backend.(*nativeGenerationReopenFixtureStore)
	base.nativeStore.candidates = view.candidates
	backend := &nativeRepairNTFSStore{base}
	reopened.backend = backend
	var fs [32]uint16
	var serial, component, flags uint32
	if err := windows.GetVolumeInformationByHandle(base.root.handle, nil, 0, &serial, &component, &flags, &fs[0], uint32(len(fs))); err != nil || windows.UTF16ToString(fs[:]) != "NTFS" {
		t.Fatal("native Repair fixture requires actual NTFS", err)
	}
	return nativeRepairNTFSFixture{reopened, backend, client, generation, public, private, f, record}
}

func TestNativeRepairNTFSRestoresDamagedPublicPayloadFromOriginalPrivateSource(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	for _, purpose := range []GenerationPurpose{GenerationPurposeApprovedBaseline, GenerationPurposeSignedCandidate} {
		t.Run(string(purpose), func(t *testing.T) {
			f := repairNTFSFixture(t, purpose)
			ctx := context.Background()
			damaged := bytes.Repeat([]byte("X"), len(f.data.payload["app/opencpn.exe"]))
			publicApp := filepath.Join(f.public, "app", "opencpn.exe")
			deleted := filepath.Join(f.public, "docs", "PRODUCT_BUILD.json")
			unknown := filepath.Join(f.public, "app", "customer-notes.txt")
			note := []byte("customer addition remains in the damaged original")
			if err := os.WriteFile(publicApp, damaged, 0600); err != nil {
				t.Fatal(err)
			}
			if err := os.Remove(deleted); err != nil {
				t.Fatal(err)
			}
			if err := os.WriteFile(unknown, note, 0600); err != nil {
				t.Fatal(err)
			}
			if original, err := f.store.OpenReadOnlyGeneration(ctx, f.client, f.generation); err == nil || original != nil {
				t.Fatal("ordinary full reopen accepted actual damaged public payload", err)
			}
			source, err := f.store.OpenRepairSource(ctx, f.client, f.generation)
			if err != nil {
				t.Fatal("original private source acquisition", err)
			}
			t.Cleanup(func() {
				if err := source.Close(); err != nil {
					t.Error(err)
				}
			})
			snapshot, err := source.Snapshot(ctx)
			if err != nil || snapshot.SourceGeneration != f.generation || snapshot.Identity != f.original.Identity {
				t.Fatal("original source provenance changed", err)
			}
			for _, access := range []uint32{windows.GENERIC_WRITE, windows.DELETE} {
				if err = nativeAttemptFile(t, f.private, access); !errors.Is(err, windows.ERROR_SHARING_VIOLATION) {
					t.Fatal("original private source lost native deny-write/delete custody", err)
				}
			}
			freshID := strings.Repeat("4", 32)
			gen, err := f.store.MaterializeRepairFresh(ctx, source, freshID, strings.Repeat("5", 64))
			if err != nil {
				t.Fatal("real native fresh copy", err)
			}
			completion, err := gen.Completion()
			if err != nil || completion.Identity != f.original.Identity || completion.Generation != freshID {
				t.Fatal("repaired code identity differs", err)
			}
			if gen.record.Repair == nil || gen.record.Repair.SourceCompletionSHA256 != snapshot.SourceCompletionSHA256 {
				t.Fatal("repair lost original protected completion binding")
			}
			for name, old := range f.original.Members {
				got := gen.record.Members[name]
				if got.Identity == old.Identity || got.Identity == f.original.SourceMembers[name] {
					t.Fatal("repair adopted old public/private file identity", name)
				}
			}
			if err = gen.Close(); err != nil {
				t.Fatal(err)
			}
			if err = source.Close(); err != nil {
				t.Fatal(err)
			}
			repaired, err := f.store.OpenReadOnlyGeneration(ctx, f.client, freshID)
			if err != nil {
				t.Fatal("independent strict restored-generation reopen", err)
			}
			t.Cleanup(func() {
				if err := repaired.Close(); err != nil {
					t.Error(err)
				}
			})
			if _, err = repaired.Repaired(); err != nil {
				t.Fatal(err)
			}
			if _, err = repaired.Candidate(); err == nil {
				t.Fatal("repair relabeled as signed candidate")
			}
			expected := map[string][]byte{"FILE_SHA256.json": f.data.inventory, "installer/package.json": f.data.manifest}
			for name, raw := range f.data.payload {
				expected[name] = raw
			}
			for name, want := range expected {
				member, err := repaired.OpenMember(name)
				if err != nil {
					t.Fatal(name, err)
				}
				got := make([]byte, len(want))
				_, readErr := member.ReadAt(got, 0)
				if err = errors.Join(readErr, member.Close()); err != nil || !bytes.Equal(got, want) {
					t.Fatal("restored source bytes differ", name, err)
				}
			}
			if got, err := os.ReadFile(publicApp); err != nil || !bytes.Equal(got, damaged) {
				t.Fatal("damaged original was overwritten", err)
			}
			if _, err := os.Stat(deleted); !errors.Is(err, os.ErrNotExist) {
				t.Fatal("deleted original was recreated", err)
			}
			if got, err := os.ReadFile(unknown); err != nil || !bytes.Equal(got, note) {
				t.Fatal("unknown original addition changed", err)
			}
			if _, err := os.Stat(filepath.Join(filepath.Dir(f.public), freshID, "app", "customer-notes.txt")); !errors.Is(err, os.ErrNotExist) {
				t.Fatal("unknown addition adopted into authenticated repair", err)
			}
			if err = repaired.Close(); err != nil {
				t.Fatal(err)
			}
			if err = f.store.Close(); err != nil {
				t.Fatal(err)
			}
			for name := range f.original.Members {
				for _, root := range []string{filepath.Dir(filepath.Dir(f.private)), filepath.Join(filepath.Dir(f.public), freshID)} {
					path := filepath.Join(root, filepath.FromSlash(name))
					if err = nativeAttemptFile(t, path, windows.GENERIC_WRITE); err != nil {
						t.Fatal("closed source/copy leaked native handle", name, err)
					}
				}
			}
		})
	}
}

func TestNativeRepairNTFSRefusesTamperedOriginalPrivateSource(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	for _, purpose := range []GenerationPurpose{GenerationPurposeApprovedBaseline, GenerationPurposeSignedCandidate} {
		for _, fault := range []string{"bytes", "same-bytes-replacement-id"} {
			t.Run(string(purpose)+"/"+fault, func(t *testing.T) {
				f := repairNTFSFixture(t, purpose)
				original := f.data.payload["app/opencpn.exe"]
				if fault == "bytes" {
					if err := os.WriteFile(f.private, bytes.Repeat([]byte("Z"), len(original)), 0600); err != nil {
						t.Fatal("actual private source mutation", err)
					}
				} else {
					if err := os.Remove(f.private); err != nil {
						t.Fatal(err)
					}
					var client *windows.SID
					if purpose == GenerationPurposeSignedCandidate {
						client = f.backend.sid
					}
					dirSD, err := descriptor(f.backend.broker, client, true)
					if err != nil {
						t.Fatal(err)
					}
					fileSD, err := descriptor(f.backend.broker, client, false)
					if err != nil {
						t.Fatal(err)
					}
					parent, err := openPin(filepath.Dir(f.private), "", dirSD)
					if err != nil {
						t.Fatal(err)
					}
					replacement, err := baselineFile(parent, filepath.Base(f.private), fileSD, true)
					if err != nil {
						windows.CloseHandle(parent.handle)
						t.Fatal(err)
					}
					replacementID := replacement.identity
					_, writeErr := replacement.Write(original)
					if err = errors.Join(writeErr, replacement.Close(), windows.CloseHandle(parent.handle)); err != nil {
						t.Fatal(err)
					}
					if replacementID == f.original.SourceMembers["app/opencpn.exe"] {
						t.Fatal("fixture did not replace original private physical ID")
					}
					if got, err := os.ReadFile(f.private); err != nil || !bytes.Equal(got, original) {
						t.Fatal("identity fixture changed original bytes", err)
					}
				}
				source, err := f.store.OpenRepairSource(context.Background(), f.client, f.generation)
				if err == nil || source != nil {
					t.Fatal("tampered native source accepted", err)
				}
				if _, err := os.Stat(filepath.Join(filepath.Dir(f.public), strings.Repeat("4", 32))); !errors.Is(err, os.ErrNotExist) {
					t.Fatal("failed source acquisition created a repair", err)
				}
				for name := range f.original.Members {
					path := filepath.Join(filepath.Dir(filepath.Dir(f.private)), filepath.FromSlash(name))
					if err := nativeAttemptFile(t, path, windows.GENERIC_WRITE); err != nil {
						t.Fatal("failed acquisition leaked private member", name, err)
					}
				}
			})
		}
	}
}
