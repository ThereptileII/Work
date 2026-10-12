package brokerstore

import (
	"bytes"
	"context"
	"errors"
	"io"
	"os"
	"path/filepath"
	"runtime"
	"testing"
	"unsafe"

	"golang.org/x/sys/windows"
)

// Fixtures exercise native protection and retained-handle mechanics under the
// current user in a temporary directory. They never provision/impersonate a broker
// or qualify its actual account, fixed ProgramData location or trust state. The
// production constructor accepts none of these fixture-selected paths/principals.
func nativeRegistrationFixture(t *testing.T) (*nativeStore, *nativeRegistration) {
	t.Helper()
	user, err := windows.GetCurrentProcessToken().GetTokenUser()
	if err != nil {
		t.Fatal(err)
	}
	directorySD, err := descriptor(user.User.Sid, nil, true)
	if err != nil {
		t.Fatal(err)
	}
	fileSD, err := descriptor(user.User.Sid, nil, false)
	if err != nil {
		t.Fatal(err)
	}
	path := filepath.Join(t.TempDir(), "protected-registration-root")
	makeDirectory := func(name string) *nativePin {
		pointer, err := windows.UTF16PtrFromString(name)
		if err != nil {
			t.Fatal(err)
		}
		attributes := windows.SecurityAttributes{Length: uint32(unsafe.Sizeof(windows.SecurityAttributes{})), SecurityDescriptor: directorySD}
		if err = windows.CreateDirectory(pointer, &attributes); err != nil {
			t.Fatal(err)
		}
		pin, err := openPin(name, "", directorySD)
		if err != nil {
			t.Fatal(err)
		}
		return pin
	}
	root := makeDirectory(path)
	root.dos = path
	n := &nativeStore{broker: user.User.Sid, rootDOS: path, root: root, pins: []*nativePin{root}}
	t.Cleanup(func() {
		if err := n.close(); err != nil {
			t.Error(err)
		}
	})
	n.users = makeDirectory(filepath.Join(path, "users"))
	n.pins = append(n.pins, n.users)
	n.registrations = makeDirectory(filepath.Join(path, "registrations"))
	n.pins = append(n.pins, n.registrations)
	n.lock, err = acquireStoreLock(root, fileSD)
	if err != nil {
		t.Fatal(err)
	}
	client := user.User.Sid.String()
	r := &nativeRegistration{store: n, client: client, channel: "beta", approved: registrationTestApproval, directorySD: directorySD, fileSD: fileSD,
		stateDOS: filepath.Join(path, "users", client, "trust", "beta"), artifactsDOS: filepath.Join(path, "users", client, "artifacts")}
	t.Cleanup(func() {
		if err := r.close(); err != nil {
			t.Error(err)
		}
	})
	return n, r
}
func createNativeTestChannel(t *testing.T, r *nativeRegistration) {
	t.Helper()
	pointer, err := windows.UTF16PtrFromString(r.stateDOS)
	if err != nil {
		t.Fatal(err)
	}
	attributes := windows.SecurityAttributes{Length: uint32(unsafe.Sizeof(windows.SecurityAttributes{})), SecurityDescriptor: r.directorySD}
	if err = windows.CreateDirectory(pointer, &attributes); err != nil {
		t.Fatal(err)
	}
}
func retryNativeRegistration(r *nativeRegistration) *nativeRegistration {
	return &nativeRegistration{store: r.store, client: r.client, channel: r.channel, approved: r.approved, directorySD: r.directorySD, fileSD: r.fileSD, stateDOS: r.stateDOS, artifactsDOS: r.artifactsDOS}
}
func TestNativeClientRegistrationFinalDescriptorsAbsentStateAndCompletionReadback(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	n, r := nativeRegistrationFixture(t)
	if err := beginRegistration(context.Background(), r); err != nil {
		t.Fatal(err)
	}
	state, artifacts, err := r.directories()
	if err != nil || state != r.stateDOS || artifacts != r.artifactsDOS {
		t.Fatalf("fixed paths: %q %q %v", state, artifacts, err)
	}
	if _, err = os.Stat(state); !errors.Is(err, os.ErrNotExist) {
		t.Fatalf("state precreated: %v", err)
	}
	if err = r.marker.check(r.markerLength); err != nil {
		t.Fatal("at-creation marker protection:", err)
	}
	for _, pin := range []*nativePin{r.user, r.trust, r.artifacts} {
		if err = pin.check(); err != nil {
			t.Fatal("at-creation directory protection:", err)
		}
	}
	if _, err = readCompletedRegistration(n.registrations, r.client, r.fileSD); !errors.Is(err, windows.ERROR_SHARING_VIOLATION) {
		t.Fatalf("live writer was readable: %v", err)
	}
	createNativeTestChannel(t, r)
	raw := []byte(`{"schema":1,"nativeFixture":true}`)
	if err = r.complete(context.Background(), raw); err != nil {
		t.Fatal(err)
	}
	if err = r.registration.check(int64(len(raw))); err != nil {
		t.Fatal(err)
	}
	if err = r.close(); err != nil {
		t.Fatal(err)
	}
	got, err := readCompletedRegistration(n.registrations, r.client, r.fileSD)
	if err != nil || !bytes.Equal(got, raw) {
		t.Fatalf("completed readback: %q %v", got, err)
	}
	marker, err := readRegistrationEvidence(n.registrations, r.client+".once", r.fileSD, maxRegistrationMarkerBytes)
	if err != nil {
		t.Fatal(err)
	}
	want := registrationMarker(r.client, "beta", registrationTestApproval, registrationDigest(raw), true)
	if !bytes.Equal(marker, want) {
		t.Fatalf("approval/hash completion differs: %q", marker)
	}
	// Test-only deletion cannot reset the permanent reservation or authorize retry.
	if err = os.Remove(filepath.Join(n.rootDOS, "registrations", r.client+".json")); err != nil {
		t.Fatal(err)
	}
	retry := retryNativeRegistration(r)
	if err = beginRegistration(context.Background(), retry); !errors.Is(err, windows.ERROR_FILE_EXISTS) && !errors.Is(err, windows.ERROR_ALREADY_EXISTS) {
		t.Fatalf("permanent retry fence: %v", err)
	}
	if _, err = readCompletedRegistration(n.registrations, r.client, r.fileSD); err == nil {
		t.Fatal("deleted registration silently recreated")
	}
}

type failNativeRegistrationArtifacts struct{ *nativeRegistration }

func (r failNativeRegistrationArtifacts) createArtifacts() error { return testFault }
func TestNativeClientPartialCreationRetainsStartedMarkerAndRefusesRetry(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	n, r := nativeRegistrationFixture(t)
	if err := beginRegistration(context.Background(), failNativeRegistrationArtifacts{r}); !errors.Is(err, testFault) {
		t.Fatal(err)
	}
	marker, err := readRegistrationEvidence(n.registrations, r.client+".once", r.fileSD, maxRegistrationMarkerBytes)
	if err != nil {
		t.Fatal(err)
	}
	if !bytes.Equal(marker, registrationMarker(r.client, r.channel, r.approved, "", false)) {
		t.Fatalf("started evidence lost: %q", marker)
	}
	if _, err = os.Stat(filepath.Dir(r.stateDOS)); err != nil {
		t.Fatalf("partial trust tree removed: %v", err)
	}
	if _, err = readCompletedRegistration(n.registrations, r.client, r.fileSD); err == nil {
		t.Fatal("partial registration readable")
	}
	if err = beginRegistration(context.Background(), retryNativeRegistration(r)); err == nil {
		t.Fatal("partial client tree adopted")
	}
}
func TestNativeClientCompleteRefusesMissingOrUnsafeChannel(t *testing.T) {
	for _, unsafeChannel := range []bool{false, true} {
		t.Run(map[bool]string{false: "missing", true: "wrong-acl"}[unsafeChannel], func(t *testing.T) {
			runtime.LockOSThread()
			defer runtime.UnlockOSThread()
			n, r := nativeRegistrationFixture(t)
			if err := beginRegistration(context.Background(), r); err != nil {
				t.Fatal(err)
			}
			if unsafeChannel {
				if err := os.Mkdir(r.stateDOS, 0700); err != nil {
					t.Fatal(err)
				}
			}
			if err := r.complete(context.Background(), []byte(`{"approved":true}`)); err == nil {
				t.Fatal("missing/unsafe initialized state accepted")
			}
			if err := r.close(); err != nil {
				t.Fatal(err)
			}
			if _, err := readCompletedRegistration(n.registrations, r.client, r.fileSD); err == nil {
				t.Fatal("failed completion readable")
			}
			if _, err := os.Stat(filepath.Join(n.rootDOS, "registrations", r.client+".json")); !errors.Is(err, os.ErrNotExist) {
				t.Fatalf("JSON published before channel protection: %v", err)
			}
		})
	}
}
func TestNativeClientCompletedBytesTamperingAndHardLinksRefuse(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	n, r := nativeRegistrationFixture(t)
	if err := beginRegistration(context.Background(), r); err != nil {
		t.Fatal(err)
	}
	createNativeTestChannel(t, r)
	raw := []byte(`{"approved":true}`)
	if err := r.complete(context.Background(), raw); err != nil {
		t.Fatal(err)
	}
	if err := r.close(); err != nil {
		t.Fatal(err)
	}
	name := filepath.Join(n.rootDOS, "registrations", r.client+".json")
	f, err := os.OpenFile(name, os.O_WRONLY, 0)
	if err != nil {
		t.Fatal(err)
	}
	if _, err = f.WriteAt([]byte("X"), 0); err != nil {
		t.Fatal(err)
	}
	if err = f.Close(); err != nil {
		t.Fatal(err)
	}
	if _, err = readCompletedRegistration(n.registrations, r.client, r.fileSD); err == nil {
		t.Fatal("same-length bytes differ from completed marker accepted")
	}
	if err = os.Link(name, name+".alias"); err != nil {
		t.Fatal(err)
	}
	if _, err = readRegistrationEvidence(n.registrations, r.client+".json", r.fileSD, MaxRegistrationBytes); err == nil {
		t.Fatal("multi-link registration accepted")
	}
}
func TestNativeClientExistingRegistrationNeverTruncatedOrAdopted(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	n, r := nativeRegistrationFixture(t)
	file, err := createRegistrationFile(n.registrations, r.client+".json", r.fileSD)
	if err != nil {
		t.Fatal(err)
	}
	data := []byte("historical foreign bytes")
	if err = writeRegistrationBytes(context.Background(), file, 0, data, n.check); err != nil {
		t.Fatal(err)
	}
	info := file.pin.info
	if err = file.Close(); err != nil {
		t.Fatal(err)
	}
	if err = beginRegistration(context.Background(), r); err == nil {
		t.Fatal("historical registration accepted")
	}
	got, err := readRegistrationEvidence(n.registrations, r.client+".json", r.fileSD, MaxRegistrationBytes)
	if err != nil || !bytes.Equal(got, data) {
		t.Fatalf("historical bytes mutated: %q %v", got, err)
	}
	pointer, err := windows.UTF16PtrFromString(filepath.Join(n.rootDOS, "registrations", r.client+".json"))
	if err != nil {
		t.Fatal(err)
	}
	h, err := windows.CreateFile(pointer, windows.GENERIC_READ, windows.FILE_SHARE_READ, nil, windows.OPEN_EXISTING, windows.FILE_FLAG_OPEN_REPARSE_POINT, 0)
	if err != nil {
		t.Fatal(err)
	}
	defer windows.CloseHandle(h)
	var current windows.ByHandleFileInformation
	if err = windows.GetFileInformationByHandle(h, &current); err != nil || !sameIdentity(info, current) {
		t.Fatalf("historical identity replaced: %v", err)
	}
	if _, err = os.Stat(filepath.Join(n.rootDOS, "registrations", r.client+".once")); !errors.Is(err, os.ErrNotExist) {
		t.Fatalf("historical file consumed fresh fence: %v", err)
	}
}
func TestNativeClientStartedAndMalformedCompletionRefuse(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	n, r := nativeRegistrationFixture(t)
	if err := beginRegistration(context.Background(), r); err != nil {
		t.Fatal(err)
	}
	if _, err := r.marker.Seek(0, io.SeekStart); err != nil {
		t.Fatal(err)
	}
	if err := r.close(); err != nil {
		t.Fatal(err)
	}
	if _, err := readCompletedRegistration(n.registrations, r.client, r.fileSD); err == nil {
		t.Fatal("started marker accepted")
	}
	path := filepath.Join(n.rootDOS, "registrations", r.client+".once")
	if err := os.WriteFile(path, []byte("malformed"), 0600); err != nil {
		t.Fatal(err)
	}
	if _, err := readCompletedRegistration(n.registrations, r.client, r.fileSD); err == nil {
		t.Fatal("malformed marker accepted")
	}
}

func TestNativeClientExistingSIDTreeNeverAdopted(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	n, r := nativeRegistrationFixture(t)
	userPath := filepath.Dir(filepath.Dir(r.stateDOS))
	if err := os.Mkdir(userPath, 0700); err != nil {
		t.Fatal(err)
	}
	before, err := openPin(userPath, "", nil)
	if err != nil {
		t.Fatal(err)
	}
	identity := before.info
	if err = windows.CloseHandle(before.handle); err != nil {
		t.Fatal(err)
	}
	if err = beginRegistration(context.Background(), r); err == nil {
		t.Fatal("existing SID namespace adopted")
	}
	after, err := openPin(userPath, "", nil)
	if err != nil {
		t.Fatal(err)
	}
	defer windows.CloseHandle(after.handle)
	if !sameIdentity(identity, after.info) {
		t.Fatal("existing SID namespace replaced")
	}
	marker, err := readRegistrationEvidence(n.registrations, r.client+".once", r.fileSD, maxRegistrationMarkerBytes)
	if err != nil || !bytes.Equal(marker, registrationMarker(r.client, r.channel, r.approved, "", false)) {
		t.Fatalf("started reservation lost: %q %v", marker, err)
	}
	if _, err = os.Stat(filepath.Dir(r.stateDOS)); !errors.Is(err, os.ErrNotExist) {
		t.Fatalf("existing SID namespace mutated: %v", err)
	}
}

func TestNativeClientSIDRequiresCanonicalActualUser(t *testing.T) {
	user, err := windows.GetCurrentProcessToken().GetTokenUser()
	if err != nil {
		t.Fatal(err)
	}
	system, err := windows.StringToSid("S-1-5-18")
	if err != nil {
		t.Fatal(err)
	}
	for _, value := range []string{user.User.Sid.String(), "S-1-5-18", "S-1-5-32-544", "s-1-5-21-1-2-3-1001", "S-1-5-21-1-2-3-99999999", `S-1-5-21-1-2-3-1001\..`} {
		if _, err = clientSID(value, user.User.Sid); err == nil {
			t.Fatalf("unsafe/non-user client accepted: %q", value)
		}
	}
	if !user.User.Sid.Equals(system) {
		sid, err := clientSID(user.User.Sid.String(), system)
		if err != nil || !sid.Equals(user.User.Sid) {
			t.Fatalf("actual canonical user refused: %v", err)
		}
	}
}

type cancelNativeRegistrationWrite struct {
	candidateFile
	cancel context.CancelFunc
}

func (f cancelNativeRegistrationWrite) Write(data []byte) (int, error) {
	n, err := f.candidateFile.Write(data)
	f.cancel()
	return n, err
}
func TestNativeClientCancellationDuringCommittedMarkerRemainsReadableAfterReopen(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	n, r := nativeRegistrationFixture(t)
	if err := beginRegistration(context.Background(), r); err != nil {
		t.Fatal(err)
	}
	createNativeTestChannel(t, r)
	raw := []byte(`{"approved":"native-commit-cancellation"}`)
	var err error
	r.registration, err = createRegistrationFile(n.registrations, r.client+".json", r.fileSD)
	if err != nil {
		t.Fatal(err)
	}
	if err = writeRegistrationBytes(context.Background(), r.registration, 0, raw, r.check); err != nil {
		t.Fatal(err)
	}
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	completed := registrationMarker(r.client, r.channel, r.approved, registrationDigest(raw), true)
	if err = publishRegistrationCompletion(ctx, cancelNativeRegistrationWrite{candidateFile: r.marker, cancel: cancel}, r.markerLength, completed, r.check); err != nil {
		t.Fatalf("native durable completion relabeled cancelled: %v", err)
	}
	if !errors.Is(ctx.Err(), context.Canceled) {
		t.Fatal("fixture did not cancel during final publication")
	}
	if err = r.close(); err != nil {
		t.Fatal(err)
	}
	got, err := readCompletedRegistration(n.registrations, r.client, r.fileSD)
	if err != nil || !bytes.Equal(got, raw) {
		t.Fatalf("reopened completion after cancellation: %q %v", got, err)
	}
}
