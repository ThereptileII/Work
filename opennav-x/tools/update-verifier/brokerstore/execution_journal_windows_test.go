package brokerstore

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"os"
	"path/filepath"
	"runtime"
	"testing"

	"golang.org/x/sys/windows"
)

// Inert current-user fixture only, using the same fixed native file constructor
// beneath a temporary protected store. This does not provision/impersonate the
// broker, touch installed products, launch children, or qualify health/selection.
func nativeExecutionFixture(t *testing.T) (*nativeStore, *nativeExecutionJournal, []byte, []byte) {
	t.Helper()
	n, _ := nativeRegistrationFixture(t)
	b, err := createNativeExecutionJournal(context.Background(), n, n.broker.String(), stageA)
	if err != nil {
		t.Fatal(err)
	}
	j := b.(*nativeExecutionJournal)
	t.Cleanup(func() {
		if err := j.close(); err != nil {
			t.Error(err)
		}
	})
	ids, err := j.identities()
	if err != nil {
		t.Fatal(err)
	}
	original := []byte(`{"inertNativeFixture":true}`)
	origin := executionJournalOrigin{Schema: 1, Purpose: "protected-execution-facts", ExecutionID: stageA, ClientSID: n.broker.String(), Generation: stageA,
		GenerationPurpose: "signed-candidate-generation", OriginalCompletion: original, CompletionSHA256: registrationDigest(original), Identities: ids}
	fence, err := json.Marshal(origin)
	if err != nil {
		t.Fatal(err)
	}
	raw := append(bytes.Clone(fence), '\n')
	if err = j.initialize(context.Background(), fence, raw); err != nil {
		t.Fatal(err)
	}
	return n, j, fence, raw
}

func nativeExecutionNext(t *testing.T, previous []byte) []byte {
	t.Helper()
	e := executionJournalRecord{Schema: 1, Sequence: 1, PreviousSHA256: registrationDigest(previous), Kind: ExecutionSpawnIntent, Original: []byte("inert original request, not launch authority")}
	line, err := json.Marshal(e)
	if err != nil {
		t.Fatal(err)
	}
	return append(append(bytes.Clone(previous), line...), '\n')
}

func TestNativeExecutionJournalFreshProtectionReadbackAndPermanentReplayRefusal(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	n, j, fence, raw := nativeExecutionFixture(t)
	for _, f := range []*nativeFile{j.fence, j.journal} {
		if err := checkSecurity(f.pin.handle, f.pin.security); err != nil {
			t.Fatal("at-creation exact private descriptor", err)
		}
		for _, access := range []uint32{windows.GENERIC_READ, windows.GENERIC_WRITE, windows.DELETE} {
			pointer, err := windows.UTF16PtrFromString(physical(f.pin.name))
			if err != nil {
				t.Fatal(err)
			}
			handle, err := windows.CreateFile(pointer, access, windows.FILE_SHARE_READ|windows.FILE_SHARE_WRITE|windows.FILE_SHARE_DELETE, nil, windows.OPEN_EXISTING, windows.FILE_FLAG_OPEN_REPARSE_POINT, 0)
			if err == nil {
				windows.CloseHandle(handle)
				t.Fatal("live original journal admitted a competing data/delete handle", access)
			}
			if !errors.Is(err, windows.ERROR_SHARING_VIOLATION) {
				t.Fatal("unexpected exclusive-custody refusal", err)
			}
		}
	}
	next := nativeExecutionNext(t, raw)
	if err := j.append(context.Background(), fence, raw, next); err != nil {
		t.Fatal(err)
	}
	if err := j.validate(fence, next); err != nil {
		t.Fatal(err)
	}
	if err := j.close(); err != nil {
		t.Fatal(err)
	}
	component := executionJournalComponent(n.broker.String(), stageA)
	for suffix, want := range map[string][]byte{".once": fence, ".journal": next} {
		got, err := os.ReadFile(filepath.Join(n.rootDOS, "registrations", component+suffix))
		if err != nil || !bytes.Equal(got, want) {
			t.Fatal("original durable bytes changed after close", suffix, err)
		}
	}
	if retry, err := createNativeExecutionJournal(context.Background(), n, n.broker.String(), stageA); retry != nil || !errors.Is(err, ErrExecutionJournalUncertain) {
		t.Fatal("permanent native fence adopted/retried", err)
	}
	if err := j.validate(fence, next); !errors.Is(err, ErrClosed) {
		t.Fatal("closed native owner remained valid", err)
	}
}

func TestNativeExecutionJournalMutationIdentityAndCancellationRefuse(t *testing.T) {
	for _, fault := range []string{"bytes", "full-file-id", "full-parent-id", "cancel-before-append"} {
		t.Run(fault, func(t *testing.T) {
			runtime.LockOSThread()
			defer runtime.UnlockOSThread()
			_, j, fence, raw := nativeExecutionFixture(t)
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			switch fault {
			case "bytes":
				if _, err := j.journal.Seek(0, 0); err != nil {
					t.Fatal(err)
				}
				if _, err := j.journal.Write([]byte("x")); err != nil {
					t.Fatal(err)
				}
			case "full-file-id":
				j.ids.Journal = baselineTestID(999)
			case "full-parent-id":
				for name := range j.ids.Parents {
					j.ids.Parents[name] = baselineTestID(999)
					break
				}
			case "cancel-before-append":
				cancel()
			}
			err := j.append(ctx, fence, raw, nativeExecutionNext(t, raw))
			if err == nil {
				t.Fatal("invalid original native custody accepted")
			}
			if fault == "cancel-before-append" {
				if !errors.Is(err, context.Canceled) {
					t.Fatal("native cancellation identity lost", err)
				}
				if err := executionReadback(j.journal, raw); err != nil {
					t.Fatal("cancel-before-append mutated journal", err)
				}
			}
		})
	}
}

func TestNativeExecutionJournalPartialCreationNeverDeletesOrAdopts(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	n, _ := nativeRegistrationFixture(t)
	sd, err := descriptor(n.broker, nil, false)
	if err != nil {
		t.Fatal(err)
	}
	component := executionJournalComponent(n.broker.String(), stageA)
	foreign, err := createRegistrationFile(n.registrations, component+".journal", sd)
	if err != nil {
		t.Fatal(err)
	}
	original := []byte("preexisting protected bytes are never historical proof")
	if err = executionWrite(context.Background(), foreign, nil, original); err != nil {
		t.Fatal(err)
	}
	if err = foreign.Close(); err != nil {
		t.Fatal(err)
	}
	for attempt := 0; attempt < 2; attempt++ {
		b, err := createNativeExecutionJournal(context.Background(), n, n.broker.String(), stageA)
		if b != nil || !errors.Is(err, ErrExecutionJournalUncertain) {
			t.Fatal("partial native creation accepted or retried", err)
		}
	}
	marker, err := os.ReadFile(filepath.Join(n.rootDOS, "registrations", component+".once"))
	if err != nil || len(marker) != 0 {
		t.Fatal("partial permanent fence deleted or rewritten", err)
	}
	got, err := os.ReadFile(filepath.Join(n.rootDOS, "registrations", component+".journal"))
	if err != nil || !bytes.Equal(got, original) {
		t.Fatal("preexisting original journal adopted/mutated", err)
	}
}
