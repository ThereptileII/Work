package brokerstore

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"golang.org/x/sys/windows"
	"io"
	"os"
	"path/filepath"
	"runtime"
	"testing"
)

func nativeProfileBackupFixture(t *testing.T) (*nativeStore, *nativeExecutionProfileBackup, []byte, []byte) {
	t.Helper()
	n, _, fence, _ := nativeExecutionFixture(t)
	backend, err := createNativeExecutionProfileBackup(context.Background(), n, n.broker.String(), stageA)
	if err != nil {
		t.Fatal(err)
	}
	b := backend.(*nativeExecutionProfileBackup)
	t.Cleanup(func() {
		if err := b.close(); err != nil {
			t.Error(err)
		}
	})
	ids, err := b.identities()
	if err != nil {
		t.Fatal(err)
	}
	origin := executionProfileBackupOrigin{1, "protected-execution-profile-backup", fence, ids}
	raw, err := json.Marshal(origin)
	if err != nil {
		t.Fatal(err)
	}
	marker := append(raw, '\n')
	if err = b.initialize(context.Background(), marker); err != nil {
		t.Fatal(err)
	}
	return n, b, marker, fence
}
func nativeCompleteProfileBackup(t *testing.T, b *nativeExecutionProfileBackup, marker []byte) ([]byte, []byte) {
	t.Helper()
	raw := bytes.Repeat([]byte("entire original opaque archive\x00\xff\n"), 5000)
	digest := registrationDigest(raw)
	if err := b.write(context.Background(), bytes.NewReader(raw), int64(len(raw)), digest); err != nil {
		t.Fatal(err)
	}
	complete := executionProfileBackupCompletion{1, "completed-execution-profile-backup", registrationDigest(bytes.TrimSuffix(marker, []byte{'\n'})), int64(len(raw)), digest}
	line, err := json.Marshal(complete)
	if err != nil {
		t.Fatal(err)
	}
	next := append(append(bytes.Clone(marker), line...), '\n')
	if err = b.complete(context.Background(), marker, next, int64(len(raw)), digest); err != nil {
		t.Fatal(err)
	}
	return next, raw
}

// Inert fixtures use temporary current-user stores and actual native sharing,
// bytes and full-ID checks. No real profile/service/application is touched.
func TestNativeExecutionProfileBackupFreshStreamingReadOnlyReopen(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	n, b, marker, fence := nativeProfileBackupFixture(t)
	marker, raw := nativeCompleteProfileBackup(t, b, marker)
	if _, _, err := decodeExecutionProfileBackup(marker, fence); err != nil {
		t.Fatal(err)
	}
	if err := b.close(); err != nil {
		t.Fatal(err)
	}
	backend, gotMarker, err := openNativeExecutionProfileBackup(context.Background(), n, n.broker.String(), stageA)
	if err != nil {
		t.Fatal(err)
	}
	reader := backend.(*nativeExecutionProfileBackup)
	defer reader.close()
	if !bytes.Equal(gotMarker, marker) {
		t.Fatal("original marker reinterpreted")
	}
	got := make([]byte, len(raw)+1)
	count, err := reader.readAt(got, 0)
	if !errors.Is(err, io.EOF) || !bytes.Equal(got[:count], raw) {
		t.Fatal("exact full original archive changed", err)
	}
	for _, f := range []*nativeFile{reader.pair.fence, reader.pair.journal} {
		if err := checkSecurity(f.pin.handle, f.pin.security); err != nil {
			t.Fatal("final broker-only descriptor", err)
		}
		for _, access := range []uint32{windows.GENERIC_WRITE, windows.DELETE} {
			name, err := windows.UTF16PtrFromString(physical(f.pin.name))
			if err != nil {
				t.Fatal(err)
			}
			h, err := windows.CreateFile(name, access, windows.FILE_SHARE_READ|windows.FILE_SHARE_WRITE|windows.FILE_SHARE_DELETE, nil, windows.OPEN_EXISTING, windows.FILE_FLAG_OPEN_REPARSE_POINT, 0)
			if err == nil {
				windows.CloseHandle(h)
				t.Fatal("readonly custody admitted new write/delete", access)
			}
			if !errors.Is(err, windows.ERROR_SHARING_VIOLATION) {
				t.Fatal(err)
			}
		}
	}
	if retry, err := createNativeExecutionProfileBackup(context.Background(), n, n.broker.String(), stageA); err == nil || retry != nil {
		t.Fatal("completed reservation retried")
	}
	if err := reader.write(context.Background(), bytes.NewReader(raw), int64(len(raw)), registrationDigest(raw)); !errors.Is(err, ErrClosed) {
		t.Fatal("readonly archive gained writer", err)
	}
}
func TestNativeExecutionProfileBackupMutationAndUnknownNamespaceRefuse(t *testing.T) {
	for _, fault := range []string{"archive-bytes", "marker-bytes", "full-file-id", "full-parent-id", "extra", "hardlink"} {
		t.Run(fault, func(t *testing.T) {
			runtime.LockOSThread()
			defer runtime.UnlockOSThread()
			n, b, marker, _ := nativeProfileBackupFixture(t)
			marker, raw := nativeCompleteProfileBackup(t, b, marker)
			switch fault {
			case "archive-bytes":
				if _, err := b.pair.journal.WriteAt([]byte("X"), 0); err != nil {
					t.Fatal(err)
				}
			case "marker-bytes":
				if _, err := b.pair.fence.WriteAt([]byte("X"), 0); err != nil {
					t.Fatal(err)
				}
			case "full-file-id":
				b.pair.ids.Journal = baselineTestID(999)
			case "full-parent-id":
				for name := range b.pair.ids.Parents {
					b.pair.ids.Parents[name] = baselineTestID(999)
					break
				}
			case "extra":
				path := filepath.Join(n.rootDOS, "registrations", executionProfileBackupComponent(n.broker.String(), stageA)+".extra")
				if err := os.WriteFile(path, []byte("unknown residue"), 0600); err != nil {
					t.Fatal(err)
				}
			case "hardlink":
				if err := b.close(); err != nil {
					t.Fatal(err)
				}
				path := filepath.Join(n.rootDOS, "registrations", executionProfileBackupComponent(n.broker.String(), stageA)+".backup")
				if err := os.Link(path, filepath.Join(n.rootDOS, "outside-backup-link")); err != nil {
					t.Fatal(err)
				}
				opened, _, err := openNativeExecutionProfileBackup(context.Background(), n, n.broker.String(), stageA)
				if err == nil {
					opened.close()
					t.Fatal("multiply-linked backup reopened")
				}
				return
			}
			if err := b.validate(marker, int64(len(raw)), registrationDigest(raw)); err == nil {
				t.Fatal("mutated original backup custody accepted")
			}
		})
	}
}
func TestNativeExecutionProfileBackupPartialAndCancellationNeverAdopt(t *testing.T) {
	for _, fault := range []string{"empty-reservation", "started-only", "written-not-completed", "cancel-before-write", "cancel-before-complete"} {
		t.Run(fault, func(t *testing.T) {
			runtime.LockOSThread()
			defer runtime.UnlockOSThread()
			n, b, marker, _ := nativeProfileBackupFixture(t)
			if fault == "empty-reservation" {
				if err := b.pair.fence.Truncate(0); err != nil {
					t.Fatal(err)
				}
			}
			raw := []byte("original")
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			if fault == "cancel-before-write" {
				cancel()
				if err := b.write(ctx, bytes.NewReader(raw), int64(len(raw)), registrationDigest(raw)); !errors.Is(err, context.Canceled) {
					t.Fatal(err)
				}
			}
			if fault == "written-not-completed" || fault == "cancel-before-complete" {
				if err := b.write(context.Background(), bytes.NewReader(raw), int64(len(raw)), registrationDigest(raw)); err != nil {
					t.Fatal(err)
				}
				if fault == "cancel-before-complete" {
					cancel()
					if err := b.complete(ctx, marker, []byte("untrusted completion"), int64(len(raw)), registrationDigest(raw)); !errors.Is(err, context.Canceled) {
						t.Fatal(err)
					}
				}
			}
			if err := b.close(); err != nil {
				t.Fatal(err)
			}
			if reader, _, err := openNativeExecutionProfileBackup(context.Background(), n, n.broker.String(), stageA); err == nil {
				reader.close()
				t.Fatal("partial backup adopted")
			}
			if retry, err := createNativeExecutionProfileBackup(context.Background(), n, n.broker.String(), stageA); err == nil || retry != nil {
				t.Fatal("partial reservation retried")
			}
			if _, err := os.Stat(filepath.Join(n.rootDOS, "registrations", executionProfileBackupComponent(n.broker.String(), stageA)+".once")); err != nil {
				t.Fatal("permanent partial reservation erased", err)
			}
		})
	}
}
