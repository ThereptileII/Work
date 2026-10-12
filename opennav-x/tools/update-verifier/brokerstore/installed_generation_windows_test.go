package brokerstore

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"os"
	"path/filepath"
	"runtime"
	"strings"
	"testing"

	"golang.org/x/sys/windows"
)

// Inert current-user fixture only. These lower helpers perform actual protected
// file creation, byte copying, physical-ID checks, permanent reservations and
// completed parent reopen in a fresh temporary directory. They never access
// Program Files, provision a broker, launch any bytes or qualify execution.
type nativeGenerationFixtureStore struct {
	*nativeBaselineFixtureStore
	candidates, applications *nativePin
}

func (s *nativeGenerationFixtureStore) begin(stage, client string) (candidateBackend, error) {
	dir, err := descriptor(s.broker, s.sid, true)
	if err != nil {
		return nil, err
	}
	file, err := descriptor(s.broker, s.sid, false)
	if err != nil {
		return nil, err
	}
	c := &nativeCandidate{store: s.nativeStore, client: client, stageID: stage, directories: map[string]*nativePin{}, securityDir: dir, securityFile: file}
	root, err := c.newDirectory(s.candidates, stage)
	if err != nil {
		return nil, errors.Join(err, c.close())
	}
	c.directories[""] = root
	return c, nil
}
func (s *nativeGenerationFixtureStore) prepareInstalledGeneration(_ installedBaselineBackend, source candidateBackend, _ *Acquisition, client, stage string) (generationBackend, error) {
	c, ok := source.(*nativeCandidate)
	if !ok {
		return nil, errors.New("native source required")
	}
	g, err := prepareNativeInstalledGeneration(s.nativeStore, c, s.sid, client, stage)
	if err != nil {
		return nil, err
	}
	g.applications, err = openPin(physical(s.applications.name), s.applications.name, s.applications.security)
	if err != nil {
		return nil, errors.Join(err, g.close())
	}
	g.ancestors = append(g.ancestors, g.applications)
	if err = g.pin(g.applications); err != nil {
		return nil, errors.Join(err, g.close())
	}
	return g, nil
}
func nativeGenerationFixture(t *testing.T) (*Store, *InstalledBaseline, *nativeGenerationFixtureStore, baselineFixtureData) {
	t.Helper()
	s, p, b, _ := nativeBaselineFixture(t)
	f := baselineEngineData(t)
	writeBaseline(t, p, f)
	if err := p.Complete(context.Background(), f.identity); err != nil {
		t.Fatal(err)
	}
	private, err := descriptor(b.store.broker, nil, true)
	if err != nil {
		t.Fatal(err)
	}
	container := &nativeCandidate{store: b.store, securityDir: private}
	candidates, err := container.newDirectory(b.store.root, "generation-source-candidates")
	if err != nil {
		t.Fatal(err)
	}
	applications, err := container.newDirectory(b.store.root, "fixture-Applications")
	if err != nil {
		t.Fatal(err)
	}
	wrapper := &nativeGenerationFixtureStore{nativeBaselineFixtureStore: s.backend.(*nativeBaselineFixtureStore), candidates: candidates, applications: applications}
	s.backend = wrapper
	t.Cleanup(func() {
		s.Close()
		if err := container.close(); err != nil {
			t.Error(err)
		}
	})
	return s, p, wrapper, f
}
func nativeGenerationSource(t *testing.T, s *Store, f baselineFixtureData, stage string) *Candidate {
	t.Helper()
	c, err := s.Begin(stage, s.backend.(*nativeGenerationFixtureStore).sid.String())
	if err != nil {
		t.Fatal(err)
	}
	acquireGenerationArtifacts(t, c, f)
	return c
}
func TestNativeGenerationCopiesOriginalHelpersAndAuthenticatedParentsSupportSuccessors(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	s, p, _, f := nativeGenerationFixture(t)
	var firstParent map[string]diskIdentity
	for _, stage := range []string{strings.Repeat("9", 32), strings.Repeat("8", 32)} {
		c := nativeGenerationSource(t, s, f, stage)
		g, err := s.MaterializeCandidateFresh(context.Background(), p, c)
		if err != nil {
			t.Fatal(err)
		}
		n := g.backend.(*nativeInstalledGeneration)
		if firstParent == nil {
			firstParent = n.parentIDs
		} else if !sameGenerationIDs(firstParent, n.parentIDs) || len(n.siblings) != 1 {
			t.Fatal("completed historical parent was not authenticated/reused")
		}
		originals := map[string][]byte{"FILE_SHA256.json": f.inventory, "installer/package.json": f.manifest}
		for name, raw := range f.payload {
			originals[name] = raw
		}
		for name, want := range originals {
			member := n.members[name]
			raw, err := readBaselineFile(context.Background(), member, int64(len(want)))
			if err != nil || !bytes.Equal(raw, want) {
				t.Fatal("exact original copy differs", name, err)
			}
			path := physical(member.pin.name)
			for _, access := range []uint32{windows.GENERIC_WRITE, windows.DELETE} {
				if err = nativeAttemptFile(t, path, access); !errors.Is(err, windows.ERROR_SHARING_VIOLATION) {
					t.Fatal("generation custody missing", name, err)
				}
			}
		}
		if err = g.Validate(); err != nil {
			t.Fatal(err)
		}
		completion, err := g.Completion()
		if err != nil || completion.Generation != stage || completion.Identity != f.identity {
			t.Fatal("informational completion differs", err)
		}
		if err = g.Close(); err != nil {
			t.Fatal(err)
		}
	}
}
func TestNativeGenerationPartialParentAndGenerationRefuseWithoutAdoption(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	for _, scenario := range []string{"existing-client-without-record", "partial-parent-marker", "partial-generation-marker", "unknown-public-generation", "unknown-private-record"} {
		t.Run(scenario, func(t *testing.T) {
			s, p, w, f := nativeGenerationFixture(t)
			if scenario == "existing-client-without-record" {
				dir, err := descriptor(w.broker, w.sid, true)
				if err != nil {
					t.Fatal(err)
				}
				tree := &nativeCandidate{store: w.nativeStore, securityDir: dir}
				if _, err = tree.newDirectory(w.applications, w.sid.String()); err != nil {
					t.Fatal(err)
				}
				if err = tree.close(); err != nil {
					t.Fatal(err)
				}
			} else if scenario == "partial-parent-marker" {
				sd, err := descriptor(w.broker, nil, false)
				if err != nil {
					t.Fatal(err)
				}
				marker, err := baselineFile(w.registrations, w.sid.String()+".launch-parents.once", sd, true)
				if err != nil {
					t.Fatal(err)
				}
				if err = marker.Close(); err != nil {
					t.Fatal(err)
				}
			} else {
				first := nativeGenerationSource(t, s, f, strings.Repeat("9", 32))
				g, err := s.MaterializeCandidateFresh(context.Background(), p, first)
				if err != nil {
					t.Fatal(err)
				}
				n := g.backend.(*nativeInstalledGeneration)
				public, private := n.generations.name, n.registry.name
				if err = g.Close(); err != nil {
					t.Fatal(err)
				}
				switch scenario {
				case "partial-generation-marker":
					if err = os.WriteFile(physical(private+`\`+strings.Repeat("7", 32)+".once"), []byte("partial"), 0600); err != nil {
						t.Fatal(err)
					}
				case "unknown-public-generation":
					if err = os.Mkdir(physical(public+`\`+strings.Repeat("7", 32)), 0700); err != nil {
						t.Fatal(err)
					}
				case "unknown-private-record":
					if err = os.WriteFile(physical(private+`\unknown.json`), []byte("{}"), 0600); err != nil {
						t.Fatal(err)
					}
				}
			}
			c := nativeGenerationSource(t, s, f, strings.Repeat("8", 32))
			if _, err := s.MaterializeCandidateFresh(context.Background(), p, c); err == nil {
				t.Fatal("unknown/partial historical namespace adopted")
			}
			if _, err := os.Stat(filepath.Join(physical(w.applications.name), w.sid.String(), "generations", strings.Repeat("8", 32))); !errors.Is(err, os.ErrNotExist) {
				t.Fatal("new generation exposed after refusal", err)
			}
		})
	}
}
func TestNativeGenerationRetainedDescriptorInventoryAndPrivateBytesRefuseMutation(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	for _, scenario := range []string{"additional-member", "additional-root", "helper-hardlink", "private-record-bytes", "parent-physical-id", "directory-descriptor"} {
		t.Run(scenario, func(t *testing.T) {
			s, p, _, f := nativeGenerationFixture(t)
			c := nativeGenerationSource(t, s, f, strings.Repeat("9", 32))
			g, err := s.MaterializeCandidateFresh(context.Background(), p, c)
			if err != nil {
				t.Fatal(err)
			}
			n := g.backend.(*nativeInstalledGeneration)
			switch scenario {
			case "additional-member":
				err = os.WriteFile(physical(n.tree.directories["app"].name+`\extra`), []byte("extra"), 0600)
			case "additional-root":
				err = os.Mkdir(physical(n.generations.name+`\unexpected`), 0700)
			case "helper-hardlink":
				member := n.members["installer/engine/Lifecycle.ps1"]
				if err = member.Close(); err != nil {
					t.Fatal(err)
				}
				source, destination := physical(member.pin.name), physical(n.store.root.name+`\helper-link`)
				err = os.Link(source, destination)
				if err == nil {
					if opened, e := baselineFile(member.parent, "Lifecycle.ps1", n.tree.securityFile, false); e == nil {
						opened.Close()
						t.Fatal("hardlinked helper reopened")
					}
				}
			case "private-record-bytes":
				_, err = n.record.WriteAt([]byte("x"), 0)
			case "parent-physical-id":
				n.pins[n.generations] = baselineTestID(999)
			case "directory-descriptor":
				sd, e := descriptor(n.store.broker, nil, true)
				if e != nil {
					t.Fatal(e)
				}
				// Mutate the actual fixture directory through an owner-authorized handle;
				// retained validation must compare its original final descriptor exactly.
				// Request READ_CONTROL as well as WRITE_DAC: corroborating the actual
				// changed DACL requires a security read through this same handle.
				acl, _, e := sd.DACL()
				if e != nil {
					t.Fatal(e)
				}
				path, e := windows.UTF16PtrFromString(physical(n.tree.directories["app"].name))
				if e != nil {
					t.Fatal(e)
				}
				handle, e := windows.CreateFile(path, windows.READ_CONTROL|windows.WRITE_DAC, windows.FILE_SHARE_READ|windows.FILE_SHARE_WRITE|windows.FILE_SHARE_DELETE, nil, windows.OPEN_EXISTING, windows.FILE_FLAG_BACKUP_SEMANTICS|windows.FILE_FLAG_OPEN_REPARSE_POINT, 0)
				if e != nil {
					t.Fatal(e)
				}
				setErr := windows.SetSecurityInfo(handle, windows.SE_FILE_OBJECT, windows.DACL_SECURITY_INFORMATION|windows.PROTECTED_DACL_SECURITY_INFORMATION, nil, nil, acl, nil)
				var readbackErr error
				if setErr == nil {
					readbackErr = checkSecurity(handle, sd)
				}
				closeErr := windows.CloseHandle(handle)
				if setErr != nil {
					t.Fatalf("SetSecurityInfo actual descriptor mutation: %v (close: %v)", setErr, closeErr)
				}
				if readbackErr != nil {
					t.Fatalf("actual mutated descriptor readback: %v (close: %v)", readbackErr, closeErr)
				}
				err = closeErr
			}
			if err != nil {
				t.Fatal(err)
			}
			if err = g.Validate(); err == nil {
				t.Fatal("native custody mutation accepted")
			}
		})
	}
}
func TestNativeGenerationCompletedParentRecordBindsPhysicalIdentity(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	s, p, w, f := nativeGenerationFixture(t)
	c := nativeGenerationSource(t, s, f, strings.Repeat("9", 32))
	g, err := s.MaterializeCandidateFresh(context.Background(), p, c)
	if err != nil {
		t.Fatal(err)
	}
	if err = g.Close(); err != nil {
		t.Fatal(err)
	}
	// The private record is mutable by the fixture owner after all retained
	// generation handles close. Rehashing its marker still cannot invent birth IDs.
	sd, err := descriptor(w.broker, nil, false)
	if err != nil {
		t.Fatal(err)
	}
	record, err := openAcquisitionFile(w.registrations, w.sid.String()+".launch-parents.json", sd, windows.GENERIC_READ|windows.GENERIC_WRITE|windows.READ_CONTROL, 0, windows.OPEN_EXISTING)
	if err != nil {
		t.Fatal(err)
	}
	raw, err := os.ReadFile(record.Name())
	if err == nil {
		t.Fatal("exclusive writer unexpectedly allowed external read")
	}
	length := int64(uint64(record.pin.info.FileSizeHigh)<<32 | uint64(record.pin.info.FileSizeLow))
	raw = make([]byte, length)
	if _, err = record.ReadAt(raw, 0); err != nil {
		t.Fatal(err)
	}
	var parents generationParents
	if err = json.Unmarshal(raw, &parents); err != nil {
		t.Fatal(err)
	}
	id := parents.Directories["@generations"]
	id.Created++
	parents.Directories["@generations"] = id
	raw, err = json.Marshal(parents)
	if err != nil {
		t.Fatal(err)
	}
	if err = record.Truncate(0); err != nil {
		t.Fatal(err)
	}
	if _, err = record.WriteAt(raw, 0); err != nil {
		t.Fatal(err)
	}
	if err = record.Sync(); err != nil {
		t.Fatal(err)
	}
	if err = record.Close(); err != nil {
		t.Fatal(err)
	}
	marker, err := openAcquisitionFile(w.registrations, w.sid.String()+".launch-parents.once", sd, windows.GENERIC_READ|windows.GENERIC_WRITE|windows.READ_CONTROL, 0, windows.OPEN_EXISTING)
	if err != nil {
		t.Fatal(err)
	}
	if _, err = marker.WriteAt(parentMarker(w.sid.String(), registrationDigest(raw), true), 0); err != nil {
		t.Fatal(err)
	}
	if err = marker.Sync(); err != nil {
		t.Fatal(err)
	}
	if err = marker.Close(); err != nil {
		t.Fatal(err)
	}
	next := nativeGenerationSource(t, s, f, strings.Repeat("8", 32))
	if _, err = s.MaterializeCandidateFresh(context.Background(), p, next); err == nil {
		t.Fatal("current descriptor substituted for original creation identity")
	}
}
