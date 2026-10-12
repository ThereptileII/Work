package brokerstore

import (
	"context"
	"errors"
	"fmt"
	"golang.org/x/sys/windows"
	"os"
	"os/exec"
	"path/filepath"
	"strings"
	"testing"
	"time"
	"unsafe"
)

// These fixtures do not provision or impersonate the broker. The temp-directory
// test independently exercises native protection and handle sharing; it does not
// qualify the service account, fixed ProgramData layout or service DOS namespace.
func TestNativeProtectedDirectoryAndRenamePin(t *testing.T) {
	user, err := windows.GetCurrentProcessToken().GetTokenUser()
	if err != nil {
		t.Fatal(err)
	}
	sd, err := descriptor(user.User.Sid, nil, true)
	if err != nil {
		t.Fatal(err)
	}
	path := filepath.Join(t.TempDir(), "protected")
	ptr, err := windows.UTF16PtrFromString(path)
	if err != nil {
		t.Fatal(err)
	}
	attributes := windows.SecurityAttributes{Length: uint32(unsafe.Sizeof(windows.SecurityAttributes{})), SecurityDescriptor: sd}
	if err = windows.CreateDirectory(ptr, &attributes); err != nil {
		t.Fatal(err)
	}
	pin, err := openPin(path, "", sd)
	if err != nil {
		t.Fatal(err)
	}
	closed := false
	defer func() {
		if !closed {
			windows.CloseHandle(pin.handle)
		}
	}()
	if err = pin.check(); err != nil {
		t.Fatal(err)
	}
	altered, err := windows.SecurityDescriptorFromString("O:" + user.User.Sid.String() + "D:P(A;OICI;FA;;;" + user.User.Sid.String() + ")")
	if err != nil {
		t.Fatal(err)
	}
	if err = checkSecurity(pin.handle, altered); err == nil {
		t.Fatal("inexact ACL accepted")
	}
	if err = os.Rename(path, path+"-moved"); !errors.Is(err, windows.ERROR_SHARING_VIOLATION) {
		t.Fatalf("rename while directory data-read pin held: want sharing violation, got %v", err)
	}
	if err = pin.check(); err != nil {
		t.Fatal(err)
	}
	if err = windows.CloseHandle(pin.handle); err != nil {
		t.Fatal(err)
	}
	closed = true
	if err = os.Rename(path, path+"-moved"); err != nil {
		t.Fatalf("rename after pin released: %v", err)
	}
}

func TestNativeExistingDeleteHandleRefusesPin(t *testing.T) {
	path := filepath.Join(t.TempDir(), "preexisting-delete")
	if err := os.Mkdir(path, 0700); err != nil {
		t.Fatal(err)
	}
	pointer, err := windows.UTF16PtrFromString(path)
	if err != nil {
		t.Fatal(err)
	}
	handle, err := windows.CreateFile(pointer, windows.DELETE, windows.FILE_SHARE_READ|windows.FILE_SHARE_WRITE|windows.FILE_SHARE_DELETE, nil, windows.OPEN_EXISTING, windows.FILE_FLAG_BACKUP_SEMANTICS|windows.FILE_FLAG_OPEN_REPARSE_POINT, 0)
	if err != nil {
		t.Fatal(err)
	}
	closed := false
	defer func() {
		if !closed {
			windows.CloseHandle(handle)
		}
	}()
	pin, err := openPin(path, "", nil)
	if pin != nil {
		windows.CloseHandle(pin.handle)
		t.Fatal("pin acquired despite pre-existing delete-access handle")
	}
	if !errors.Is(err, windows.ERROR_SHARING_VIOLATION) {
		t.Fatalf("pin with pre-existing delete access: want sharing violation, got %v", err)
	}
	if err = windows.CloseHandle(handle); err != nil {
		t.Fatal(err)
	}
	closed = true
	pin, err = openPin(path, "", nil)
	if err != nil {
		t.Fatalf("pin after conflicting handle released: %v", err)
	}
	if err = windows.CloseHandle(pin.handle); err != nil {
		t.Fatal(err)
	}
}
func TestNativeOpenRefusesOrdinaryPrincipal(t *testing.T) {
	user, err := windows.GetCurrentProcessToken().GetTokenUser()
	if err != nil {
		t.Fatal(err)
	}
	broker, _, _, err := windows.LookupSID("", BrokerAccount)
	if err == nil && user.User.Sid.Equals(broker) {
		t.Skip("ordinary-user refusal requires an ordinary token")
	}
	store, err := Open(`C:\not-the-fixed-broker-root`)
	if store != nil || !errors.Is(err, ErrPrincipal) {
		t.Fatalf("unexpected ordinary principal result: %v %v", store, err)
	}
}
func TestPhysicalNamespaceValidation(t *testing.T) {
	for _, name := range []string{`\Device\HarddiskVolume3`, `\Device\HarddiskVolume3\ProgramData\SKAGER`} {
		if _, err := physicalParts(name); err != nil {
			t.Fatal(err)
		}
	}
	for _, name := range []string{`C:\ProgramData`, `\Device\Mup\server\share`, `\Device\HarddiskVolumeX`, `\Device\HarddiskVolume3\..`, `\Device\HarddiskVolume3\a:stream`, `\Device\HarddiskVolume3\a.`, `\Device\HarddiskVolume3\\a`} {
		if _, err := physicalParts(name); err == nil {
			t.Fatalf("unsafe physical name %q", name)
		}
	}
	if got := physical(`\Device\HarddiskVolume3`); !strings.HasSuffix(got, `\`) {
		t.Fatal("volume root missing separator")
	}
}

func TestNativeCandidateCapacityCountsForeignReservations(t *testing.T) {
	user, err := windows.GetCurrentProcessToken().GetTokenUser()
	if err != nil {
		t.Fatal(err)
	}
	sd, err := descriptor(user.User.Sid, nil, true)
	if err != nil {
		t.Fatal(err)
	}
	path := filepath.Join(t.TempDir(), "candidates")
	ptr, err := windows.UTF16PtrFromString(path)
	if err != nil {
		t.Fatal(err)
	}
	attributes := windows.SecurityAttributes{Length: uint32(unsafe.Sizeof(windows.SecurityAttributes{})), SecurityDescriptor: sd}
	if err = windows.CreateDirectory(ptr, &attributes); err != nil {
		t.Fatal(err)
	}
	pin, err := openPin(path, "", sd)
	if err != nil {
		t.Fatal(err)
	}
	defer windows.CloseHandle(pin.handle)
	store := &nativeStore{candidates: pin}
	if err = store.candidateCapacity(); err != nil {
		t.Fatal(err)
	}
	for i := 0; i < MaxCandidates; i++ {
		// A foreign file counts just as an incomplete candidate directory does.
		if err = os.WriteFile(filepath.Join(path, fmt.Sprintf("foreign-%03d", i)), nil, 0600); err != nil {
			t.Fatal(err)
		}
		if i == MaxCandidates-2 {
			if err = store.candidateCapacity(); err != nil {
				t.Fatal(err)
			}
		}
	}
	if err = store.candidateCapacity(); err == nil {
		t.Fatal("durable cap allowed another reservation")
	}
	entries, err := os.ReadDir(path)
	if err != nil || len(entries) != MaxCandidates {
		t.Fatalf("capacity check mutated reservations: %d %v", len(entries), err)
	}
}

func TestAncestorMutationPolicy(t *testing.T) {
	trusted := map[string]bool{"S-1-5-18": true, "S-1-5-32-544": true}
	for _, tc := range []struct {
		name, sd   string
		strict, ok bool
	}{
		{"ordinary read", "O:SYD:(A;;FA;;;SY)(A;;FRFX;;;BU)", false, true},
		{"general ancestor child additions", "O:SYD:(A;;FA;;;SY)(A;;0x00000006;;;BU)", false, true},
		{"SKAGER forbids additions", "O:SYD:(A;;FA;;;SY)(A;;0x00000006;;;BU)", true, false},
		{"inherited only does not affect current", "O:SYD:(A;;FA;;;SY)(A;OICIIO;FA;;;BU)", true, true},
		{"untrusted owner", "O:BUD:(A;;FA;;;SY)(A;;FRFX;;;BU)", false, false},
		{"null DACL", "O:SYD:NO_ACCESS_CONTROL", false, false},
		{"delete child", "O:SYD:(A;;0x00000040;;;BU)", false, false},
		{"attributes", "O:SYD:(A;;0x00000100;;;BU)", false, false},
		{"EA", "O:SYD:(A;;0x00000010;;;BU)", false, false},
		{"ACL", "O:SYD:(A;;WD;;;BU)", false, false},
		{"owner", "O:SYD:(A;;WO;;;BU)", false, false},
		{"delete", "O:SYD:(A;;SD;;;BU)", false, false},
		{"generic write", "O:SYD:(A;;GW;;;BU)", false, false},
		{"generic all", "O:SYD:(A;;GA;;;BU)", false, false},
		{"deny never excuses grant", "O:SYD:(D;;FA;;;BU)(A;;FA;;;BU)", false, false},
		{"unsupported effective object ACE", "O:SYD:(OA;;FR;00112233-4455-6677-8899-aabbccddeeff;;BU)", false, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			sd, err := windows.SecurityDescriptorFromString(tc.sd)
			if err != nil {
				t.Fatal(err)
			}
			err = checkAncestorDescriptor(sd, &ancestorPolicy{trusted: trusted, forbidChildAdds: tc.strict})
			if (err == nil) != tc.ok {
				t.Fatalf("accepted=%v want=%v: %v", err == nil, tc.ok, err)
			}
		})
	}
}

func storeLockFixture(t *testing.T) (*nativePin, *windows.SECURITY_DESCRIPTOR) {
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
	name := filepath.Join(t.TempDir(), "protected-lock-root")
	pointer, err := windows.UTF16PtrFromString(name)
	if err != nil {
		t.Fatal(err)
	}
	attributes := windows.SecurityAttributes{Length: uint32(unsafe.Sizeof(windows.SecurityAttributes{})), SecurityDescriptor: directorySD}
	if err = windows.CreateDirectory(pointer, &attributes); err != nil {
		t.Fatal(err)
	}
	root, err := openPin(name, "", directorySD)
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() {
		if root.handle != windows.InvalidHandle {
			if err := windows.CloseHandle(root.handle); err != nil {
				t.Error(err)
			}
		}
	})
	return root, fileSD
}

func TestNativeStoreLifetimeLockExcludesOtherOwnerAndReacquires(t *testing.T) {
	root, sd := storeLockFixture(t)
	lock, err := acquireStoreLock(root, sd)
	if err != nil {
		t.Fatal(err)
	}
	closed := false
	defer func() {
		if !closed {
			windows.CloseHandle(lock.handle)
		}
	}()
	other, err := acquireStoreLock(root, sd)
	if other != nil {
		windows.CloseHandle(other.handle)
		t.Fatal("second owner acquired store lock")
	}
	if !errors.Is(err, windows.ERROR_SHARING_VIOLATION) {
		t.Fatalf("want sharing violation, got %v", err)
	}
	if err = lock.check(); err != nil {
		t.Fatal(err)
	}
	if err = windows.CloseHandle(lock.handle); err != nil {
		t.Fatal(err)
	}
	closed = true
	next, err := acquireStoreLock(root, sd)
	if err != nil {
		t.Fatal("reacquire existing protected synchronization file:", err)
	}
	defer windows.CloseHandle(next.handle)
	if !sameIdentity(lock.info, next.info) {
		t.Fatal("reacquisition replaced the durable synchronization file")
	}
}

// Only the parent fixture selects this inert test-binary child; no service or
// product is started, and this helper does not bypass Open's broker principal.
func TestNativeStoreLockChild(t *testing.T) {
	name := os.Getenv("SKAGER_STORE_LOCK_TEST_ROOT")
	if name == "" {
		return
	}
	user, err := windows.GetCurrentProcessToken().GetTokenUser()
	if err != nil {
		t.Fatal(err)
	}
	dirSD, err := descriptor(user.User.Sid, nil, true)
	if err != nil {
		t.Fatal(err)
	}
	fileSD, err := descriptor(user.User.Sid, nil, false)
	if err != nil {
		t.Fatal(err)
	}
	root, err := openPin(name, "", dirSD)
	if err != nil {
		t.Fatal(err)
	}
	defer windows.CloseHandle(root.handle)
	lock, err := acquireStoreLock(root, fileSD)
	if os.Getenv("SKAGER_STORE_LOCK_TEST_MODE") == "busy" {
		if lock != nil {
			windows.CloseHandle(lock.handle)
			t.Fatal("cross-process lock acquired while held")
		}
		if !errors.Is(err, windows.ERROR_SHARING_VIOLATION) {
			t.Fatalf("cross-process refusal: %v", err)
		}
		return
	}
	if os.Getenv("SKAGER_STORE_LOCK_TEST_MODE") != "acquire" {
		t.Fatal("unexpected inert fixture mode")
	}
	if err != nil {
		t.Fatal(err)
	}
	if err = lock.check(); err != nil {
		windows.CloseHandle(lock.handle)
		t.Fatal(err)
	}
	if err = windows.CloseHandle(lock.handle); err != nil {
		t.Fatal(err)
	}
}

func TestNativeStoreLifetimeLockExcludesAnotherProcess(t *testing.T) {
	root, sd := storeLockFixture(t)
	lock, err := acquireStoreLock(root, sd)
	if err != nil {
		t.Fatal(err)
	}
	closed := false
	defer func() {
		if !closed {
			windows.CloseHandle(lock.handle)
		}
	}()
	executable, err := os.Executable()
	if err != nil {
		t.Fatal(err)
	}
	run := func(mode string) {
		t.Helper()
		ctx, cancel := context.WithTimeout(context.Background(), 30*time.Second)
		defer cancel()
		command := exec.CommandContext(ctx, executable, "-test.run=^TestNativeStoreLockChild$", "-test.count=1")
		command.Env = append(os.Environ(), "SKAGER_STORE_LOCK_TEST_ROOT="+physical(root.name), "SKAGER_STORE_LOCK_TEST_MODE="+mode)
		if out, err := command.CombinedOutput(); err != nil {
			t.Fatalf("inert lock child %s: %v %s", mode, err, out)
		}
	}
	run("busy")
	if err = windows.CloseHandle(lock.handle); err != nil {
		t.Fatal(err)
	}
	closed = true
	run("acquire")
}

func TestNativeStoreLifetimeLockRefusesUnsafeExistingObjects(t *testing.T) {
	for _, kind := range []string{"directory", "nonempty", "hardlink", "wrong DACL"} {
		t.Run(kind, func(t *testing.T) {
			root, sd := storeLockFixture(t)
			name := physical(root.name + `\` + storeLockFilename)
			if kind == "directory" {
				if err := os.Mkdir(name, 0700); err != nil {
					t.Fatal(err)
				}
			} else {
				lock, err := acquireStoreLock(root, sd)
				if err != nil {
					t.Fatal(err)
				}
				if err = windows.CloseHandle(lock.handle); err != nil {
					t.Fatal(err)
				}
				switch kind {
				case "nonempty":
					if err = os.WriteFile(name, []byte("not synchronization state"), 0600); err != nil {
						t.Fatal(err)
					}
				case "hardlink":
					if err = os.Link(name, physical(root.name+`\other-lock-name`)); err != nil {
						t.Fatal(err)
					}
				case "wrong DACL":
					user, err := windows.GetCurrentProcessToken().GetTokenUser()
					if err != nil {
						t.Fatal(err)
					}
					wrong, err := windows.SecurityDescriptorFromString("O:" + user.User.Sid.String() + "D:P(A;;FA;;;" + user.User.Sid.String() + ")")
					if err != nil {
						t.Fatal(err)
					}
					acl, _, err := wrong.DACL()
					if err != nil {
						t.Fatal(err)
					}
					if err = windows.SetNamedSecurityInfo(name, windows.SE_FILE_OBJECT, windows.DACL_SECURITY_INFORMATION|windows.PROTECTED_DACL_SECURITY_INFORMATION, nil, nil, acl, nil); err != nil {
						t.Fatal(err)
					}
				}
			}
			lock, err := acquireStoreLock(root, sd)
			if lock != nil {
				windows.CloseHandle(lock.handle)
				t.Fatal("unsafe synchronization object accepted")
			}
			if err == nil {
				t.Fatal("unsafe lock refusal missing")
			}
		})
	}
}

func TestNativeStoreLifetimeLockRechecksHeldMetadataAndReleasesAfterCleanupError(t *testing.T) {
	root, sd := storeLockFixture(t)
	lock, err := acquireStoreLock(root, sd)
	if err != nil {
		t.Fatal(err)
	}
	var written uint32
	if err = windows.WriteFile(lock.handle, []byte{1}, &written, nil); err != nil || written != 1 {
		windows.CloseHandle(lock.handle)
		t.Fatalf("inert lock mutation: %d %v", written, err)
	}
	if err = lock.check(); err == nil {
		windows.CloseHandle(lock.handle)
		t.Fatal("held nonzero synchronization file not refused")
	}
	if _, err = windows.SetFilePointer(lock.handle, 0, nil, windows.FILE_BEGIN); err != nil {
		windows.CloseHandle(lock.handle)
		t.Fatal(err)
	}
	if err = windows.SetEndOfFile(lock.handle); err != nil {
		windows.CloseHandle(lock.handle)
		t.Fatal(err)
	}
	if err = windows.CloseHandle(root.handle); err != nil {
		windows.CloseHandle(lock.handle)
		t.Fatal(err)
	}
	root.handle = windows.InvalidHandle // The fixture no longer owns the closed real handle.
	// InvalidHandle (-1) is also the current-process pseudo-handle; closing it
	// may succeed. Inject null only into a separate pin, leaving fixture cleanup
	// disarmed and never passing a reusable closed handle to production cleanup.
	failedPin := *root
	failedPin.handle = 0
	store := &Store{backend: &nativeStore{pins: []*nativePin{&failedPin}, lock: lock}}
	firstClose := store.Close()
	if !errors.Is(firstClose, windows.ERROR_INVALID_HANDLE) {
		t.Fatalf("cleanup error lost: %v", firstClose)
	}
	if again := store.Close(); again != firstClose {
		t.Fatalf("completed cleanup error not retained: %v", again)
	}
	// Only metadata on this released fixed file is inspected here; a new owner
	// must subsequently run the complete protected-root acquisition again.
	file, err := os.OpenFile(physical(lock.name), os.O_RDWR, 0)
	if err != nil {
		t.Fatal("lock not released after ancestor cleanup error:", err)
	}
	if err = file.Close(); err != nil {
		t.Fatal(err)
	}
}
