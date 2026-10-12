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

// Real temporary protected files/current account only. The private adapter
// bypasses production broker/client principal and KnownFolder discovery, never
// provisions a service, touches installed products or claims health/selection.
type nativeExecutionReopenFixtureBackend struct {
	storeBackend
	n *nativeStore
}

func (b *nativeExecutionReopenFixtureBackend) executionJournalNames(ctx context.Context, _ string) ([]string, error) {
	return nativeExecutionJournalNames(ctx, b.n)
}
func (b *nativeExecutionReopenFixtureBackend) openExecutionJournal(ctx context.Context, client, id string) (executionJournalReadBackend, []byte, []byte, error) {
	return openNativeExecutionJournal(ctx, b.n, client, id)
}
func reopenNativeExecutionStoreFixture(t *testing.T, old *nativeStore) *nativeStore {
	t.Helper()
	sd, err := descriptor(old.broker, nil, true)
	if err != nil {
		t.Fatal(err)
	}
	fileSD, err := descriptor(old.broker, nil, false)
	if err != nil {
		t.Fatal(err)
	}
	n := &nativeStore{broker: old.broker, rootDOS: old.rootDOS}
	for _, component := range []string{"", "users", "registrations"} {
		path := filepath.Join(n.rootDOS, component)
		pin, err := openPin(path, "", sd)
		if err != nil {
			t.Fatal(err)
		}
		n.pins = append(n.pins, pin)
		switch component {
		case "":
			n.root = pin
			pin.dos = path
		case "users":
			n.users = pin
		case "registrations":
			n.registrations = pin
		}
	}
	n.lock, err = acquireStoreLock(n.root, fileSD)
	if err != nil {
		n.close()
		t.Fatal(err)
	}
	t.Cleanup(func() {
		if err := n.close(); err != nil {
			t.Error(err)
		}
	})
	return n
}
func TestNativeExecutionJournalReopenAcrossStoreRestartReadonlyCustody(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	old, j, fence, raw := nativeExecutionFixture(t)
	next := nativeExecutionNext(t, raw)
	if err := j.append(context.Background(), fence, raw, next); err != nil {
		t.Fatal(err)
	}
	if err := j.close(); err != nil {
		t.Fatal(err)
	}
	if err := old.close(); err != nil {
		t.Fatal(err)
	}
	n := reopenNativeExecutionStoreFixture(t, old)
	s := &Store{backend: &nativeExecutionReopenFixtureBackend{storeBackend: n, n: n}}
	defer s.Close()
	entries, err := s.EnumerateExecutionJournals(context.Background(), n.broker.String())
	if err != nil || len(entries) != 1 || !entries[0].Readable {
		t.Fatal("durable original pair not enumerated", entries, err)
	}
	reader, err := s.OpenExecutionJournal(context.Background(), n.broker.String(), stageA)
	if err != nil {
		t.Fatal(err)
	}
	snapshot, err := reader.Snapshot()
	if err != nil || !bytes.Equal(snapshot.OriginalFence, fence) || !bytes.Equal(snapshot.OriginalJournal, next) || len(snapshot.Facts) != 1 || snapshot.Facts[0].Kind != ExecutionSpawnIntent {
		t.Fatal("restart reconstructed original facts", err)
	}
	native := reader.state.backend.(*nativeExecutionJournalReader)
	for _, file := range []*nativeFile{native.journal.fence, native.journal.journal} {
		if _, err := file.Write([]byte("must refuse")); err == nil {
			t.Fatal("readonly original handle admitted writes")
		}
		pointer, err := windows.UTF16PtrFromString(physical(file.pin.name))
		if err != nil {
			t.Fatal(err)
		}
		for _, access := range []uint32{windows.GENERIC_WRITE, windows.DELETE} {
			handle, err := windows.CreateFile(pointer, access, windows.FILE_SHARE_READ|windows.FILE_SHARE_WRITE|windows.FILE_SHARE_DELETE, nil, windows.OPEN_EXISTING, windows.FILE_FLAG_OPEN_REPARSE_POINT, 0)
			if err == nil {
				windows.CloseHandle(handle)
				t.Fatal("readonly custody admitted competing mutation")
			}
			if !errors.Is(err, windows.ERROR_SHARING_VIOLATION) {
				t.Fatal("unexpected mutation refusal", err)
			}
		}
	}
	if err := reader.Validate(); err != nil {
		t.Fatal("write refusal changed originals", err)
	}
	if err := s.Close(); err != nil {
		t.Fatal(err)
	}
	if err := reader.Validate(); !errors.Is(err, ErrClosed) {
		t.Fatal("Store.Close left native reader live")
	}
}
func TestNativeExecutionJournalReopenRefusesOriginalMutationAndFork(t *testing.T) {
	for _, kind := range []string{"bytes", "replacement-full-id", "forged-parent-id", "hardlink", "suffix-fork", "case-alias", "directory", "descriptor"} {
		t.Run(kind, func(t *testing.T) {
			runtime.LockOSThread()
			defer runtime.UnlockOSThread()
			n, j, fence, raw := nativeExecutionFixture(t)
			if err := j.close(); err != nil {
				t.Fatal(err)
			}
			component := executionJournalComponent(n.broker.String(), stageA)
			fencePath := filepath.Join(n.rootDOS, "registrations", component+".once")
			journalPath := filepath.Join(n.rootDOS, "registrations", component+".journal")
			sd, err := descriptor(n.broker, nil, false)
			if err != nil {
				t.Fatal(err)
			}
			switch kind {
			case "bytes":
				raw[0] ^= 1
				if err := os.WriteFile(journalPath, raw, 0600); err != nil {
					t.Fatal(err)
				}
			case "replacement-full-id":
				if err := os.Remove(fencePath); err != nil {
					t.Fatal(err)
				}
				file, err := createRegistrationFile(n.registrations, component+".once", sd)
				if err != nil {
					t.Fatal(err)
				}
				if err := executionWrite(context.Background(), file, nil, fence); err != nil {
					file.Close()
					t.Fatal(err)
				}
				if err := file.Close(); err != nil {
					t.Fatal(err)
				}
			case "forged-parent-id":
				var origin executionJournalOrigin
				if err := json.Unmarshal(fence, &origin); err != nil {
					t.Fatal(err)
				}
				id := origin.Identities.Parents[n.root.name]
				id.Created++
				origin.Identities.Parents[n.root.name] = id
				fence, err = json.Marshal(origin)
				if err != nil {
					t.Fatal(err)
				}
				raw = append(bytes.Clone(fence), '\n')
				if err := os.WriteFile(fencePath, fence, 0600); err != nil {
					t.Fatal(err)
				}
				if err := os.WriteFile(journalPath, raw, 0600); err != nil {
					t.Fatal(err)
				}
			case "hardlink":
				if err := os.Link(journalPath, filepath.Join(n.rootDOS, "original-journal-hardlink")); err != nil {
					t.Fatal(err)
				}
			case "suffix-fork":
				if err := os.WriteFile(journalPath+".fork", []byte("inert fork residue"), 0600); err != nil {
					t.Fatal(err)
				}
			case "case-alias":
				// Windows cannot hold both case spellings. Rename itself must be refused as
				// origin authority even if it still resolves through a case-insensitive open.
				alias := filepath.Join(n.rootDOS, "registrations", strings.ToLower(component)+".once")
				if err := os.Rename(fencePath, alias); err != nil {
					t.Fatal(err)
				}
			case "directory":
				if err := os.Remove(journalPath); err != nil {
					t.Fatal(err)
				}
				if err := os.Mkdir(journalPath, 0700); err != nil {
					t.Fatal(err)
				}
			case "descriptor":
				pointer, err := windows.UTF16PtrFromString(fencePath)
				if err != nil {
					t.Fatal(err)
				}
				// Retain security-read rights for the protected DACL update and actual
				// readback, matching the native generation mutation fixture.
				handle, err := windows.CreateFile(pointer, windows.READ_CONTROL|windows.WRITE_DAC, windows.FILE_SHARE_READ|windows.FILE_SHARE_WRITE|windows.FILE_SHARE_DELETE, nil, windows.OPEN_EXISTING, windows.FILE_FLAG_OPEN_REPARSE_POINT, 0)
				if err != nil {
					t.Fatal(err)
				}
				altered, err := windows.SecurityDescriptorFromString("O:" + n.broker.String() + "D:P(A;;FA;;;" + n.broker.String() + ")(A;;FA;;;SY)(A;;FR;;;WD)")
				if err != nil {
					windows.CloseHandle(handle)
					t.Fatal(err)
				}
				acl, _, err := altered.DACL()
				if err == nil {
					err = windows.SetSecurityInfo(handle, windows.SE_FILE_OBJECT, windows.DACL_SECURITY_INFORMATION|windows.PROTECTED_DACL_SECURITY_INFORMATION, nil, nil, acl, nil)
				}
				if err == nil {
					// Confirm the intended changed ACE bytes on the real file before
					// asking the original-custody reader to refuse that descriptor.
					err = nativeGenerationMutatedDACL(handle, acl)
				}
				closeErr := windows.CloseHandle(handle)
				if err != nil || closeErr != nil {
					t.Fatal("actual descriptor mutation/readback", err, closeErr)
				}
			}
			owner, _, _, err := openNativeExecutionJournal(context.Background(), n, n.broker.String(), stageA)
			if owner != nil {
				owner.close()
				t.Fatal("mutated/forked pair minted native read custody")
			}
			if err == nil {
				t.Fatal("mutated/forked pair accepted")
			}
			if kind != "directory" && kind != "case-alias" {
				if _, err := os.Stat(fencePath); err != nil {
					t.Fatal("refusal removed permanent origin", err)
				}
			}
		})
	}
}
func TestNativeExecutionJournalReopenRetainsPartialReservationsAndLiveForkRefusal(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	n, j, fence, raw := nativeExecutionFixture(t)
	if err := j.close(); err != nil {
		t.Fatal(err)
	}
	owner, _, _, err := openNativeExecutionJournal(context.Background(), n, n.broker.String(), stageA)
	if err != nil {
		t.Fatal(err)
	}
	component := executionJournalComponent(n.broker.String(), stageA)
	if err := os.WriteFile(filepath.Join(n.rootDOS, "registrations", component+".journal.fork"), []byte("unknown residue"), 0600); err != nil {
		owner.close()
		t.Fatal(err)
	}
	if err := owner.validate(fence, raw); err == nil {
		owner.close()
		t.Fatal("live readonly owner ignored same-ID namespace addition")
	}
	if err := owner.close(); err != nil {
		t.Fatal(err)
	}
	sd, err := descriptor(n.broker, nil, false)
	if err != nil {
		t.Fatal(err)
	}
	id := strings.Repeat("b", 32)
	partialName := executionJournalComponent(n.broker.String(), id) + ".once"
	partial, err := createRegistrationFile(n.registrations, partialName, sd)
	if err != nil {
		t.Fatal(err)
	}
	if err := partial.Sync(); err != nil {
		partial.Close()
		t.Fatal(err)
	}
	if err := partial.Close(); err != nil {
		t.Fatal(err)
	}
	s := &Store{backend: &nativeExecutionReopenFixtureBackend{storeBackend: n, n: n}}
	entries, err := s.EnumerateExecutionJournals(context.Background(), n.broker.String())
	var uncertain *ExecutionJournalNamespaceError
	if !errors.As(err, &uncertain) || !errors.Is(err, ErrExecutionJournalUncertain) || len(entries) != 3 {
		t.Fatal("native partial/fork attempt silently ignored", entries, err)
	}
	if got, err := os.ReadFile(filepath.Join(n.rootDOS, "registrations", partialName)); err != nil || len(got) != 0 {
		t.Fatal("partial reservation repaired/deleted", err)
	}
	if got, err := os.ReadFile(filepath.Join(n.rootDOS, "registrations", component+".once")); err != nil || !bytes.Equal(got, fence) {
		t.Fatal("original fence changed", err)
	}
}
