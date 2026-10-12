package brokerstore

import (
	"errors"
	"os"
	"path/filepath"
	"runtime"
	"testing"
	"unsafe"

	"golang.org/x/sys/windows"
)

// These private helper fixtures use a current-user protected temp parent and
// never create/impersonate the broker, touch ProgramData or provision SCM. They
// exercise native creation mechanics, not caller provisioning-history evidence.
func freshFixture(t *testing.T, path string, create bool) *nativeFreshCreation {
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
	if create {
		pointer, err := windows.UTF16PtrFromString(path)
		if err != nil {
			t.Fatal(err)
		}
		attributes := windows.SecurityAttributes{Length: uint32(unsafe.Sizeof(windows.SecurityAttributes{})), SecurityDescriptor: directorySD}
		if err = windows.CreateDirectory(pointer, &attributes); err != nil {
			t.Fatal(err)
		}
	}
	parent, err := openPin(path, "", directorySD)
	if err != nil {
		t.Fatal(err)
	}
	parent.dos = path
	parent.ancestor = &ancestorPolicy{trusted: map[string]bool{user.User.Sid.String(): true, "S-1-5-18": true}, forbidChildAdds: true}
	// No production constructor accepts a caller-supplied SID/path or a nil broker.
	creation := &nativeFreshCreation{parent: parent, pins: []*nativePin{parent}, directorySD: directorySD, fileSD: fileSD}
	t.Cleanup(func() {
		if err := creation.close(); err != nil {
			t.Error(err)
		}
	})
	return creation
}

func TestNativeFreshInitializationFinalCreationDescriptorsAndFenceRetry(t *testing.T) {
	parentPath := filepath.Join(t.TempDir(), "protected-parent")
	c := freshFixture(t, parentPath, true)
	if err := initializeFresh(c); err != nil {
		t.Fatal(err)
	}
	contents, err := os.ReadFile(filepath.Join(parentPath, initializationFenceFilename))
	if err != nil || string(contents) != initializationCompleted {
		t.Fatalf("completed fence: %q %v", contents, err)
	}
	for _, relative := range []string{"UpdateBroker", `UpdateBroker\candidates`, `UpdateBroker\registrations`, `UpdateBroker\users`} {
		pin, err := openPin(filepath.Join(parentPath, relative), "", c.directorySD)
		if err != nil {
			t.Fatalf("final at-creation descriptor %s: %v", relative, err)
		}
		if err = windows.CloseHandle(pin.handle); err != nil {
			t.Fatal(err)
		}
	}
	// A fresh read-only completion check also verifies exact fence SD/identity.
	check := freshFixture(t, parentPath, false)
	if err = check.validateCompleted(); err != nil {
		t.Fatal(err)
	}
	if err = check.close(); err != nil {
		t.Fatal(err)
	}
	retry := freshFixture(t, parentPath, false)
	if err = initializeFresh(retry); err == nil {
		t.Fatal("existing root accepted")
	}
	// Root removal cannot reset the permanent fence. Test-only deletion is local
	// fixture teardown; production never deletes either fence or namespace.
	if err = os.RemoveAll(filepath.Join(parentPath, "UpdateBroker")); err != nil {
		t.Fatal(err)
	}
	retry = freshFixture(t, parentPath, false)
	err = initializeFresh(retry)
	if !errors.Is(err, windows.ERROR_FILE_EXISTS) && !errors.Is(err, windows.ERROR_ALREADY_EXISTS) {
		t.Fatalf("fence retry after root deletion: %v", err)
	}
	if _, err = os.Stat(filepath.Join(parentPath, "UpdateBroker")); !errors.Is(err, os.ErrNotExist) {
		t.Fatalf("retry recreated root: %v", err)
	}
}

func TestNativeCompletedFenceRequiresCompleteExistingTree(t *testing.T) {
	for _, missing := range []string{"registrations", "users"} {
		t.Run(missing, func(t *testing.T) {
			parentPath := filepath.Join(t.TempDir(), "protected-parent")
			c := freshFixture(t, parentPath, true)
			if err := initializeFresh(c); err != nil {
				t.Fatal(err)
			}
			missingPath := filepath.Join(parentPath, "UpdateBroker", missing)
			if err := os.Remove(missingPath); err != nil {
				t.Fatal(err)
			}
			check := freshFixture(t, parentPath, false)
			if err := check.validateCompleted(); err == nil {
				t.Fatal("completed fence accepted missing required child")
			}
			if err := check.close(); err != nil {
				t.Fatal(err)
			}
			if _, err := os.Stat(missingPath); !errors.Is(err, os.ErrNotExist) {
				t.Fatalf("validation repaired missing child: %v", err)
			}
		})
	}
}

type faultCandidates struct{ *nativeFreshCreation }

func (c faultCandidates) createCandidates() error { return errors.New("injected before candidates") }

func TestNativeFreshPartialFailureRetainsStartedFenceAndRoot(t *testing.T) {
	parentPath := filepath.Join(t.TempDir(), "protected-parent")
	c := freshFixture(t, parentPath, true)
	if err := initializeFresh(faultCandidates{c}); err == nil {
		t.Fatal("injected failure lost")
	}
	contents, err := os.ReadFile(filepath.Join(parentPath, initializationFenceFilename))
	if err != nil || string(contents) != initializationStarted {
		t.Fatalf("started fence: %q %v", contents, err)
	}
	if _, err = os.Stat(filepath.Join(parentPath, "UpdateBroker")); err != nil {
		t.Fatalf("partial root removed: %v", err)
	}
	check := freshFixture(t, parentPath, false)
	if err = check.readCompletedFence(); err == nil {
		t.Fatal("partial attempt accepted as complete")
	}
	if err = check.close(); err != nil {
		t.Fatal(err)
	}
	retry := freshFixture(t, parentPath, false)
	if err = initializeFresh(retry); err == nil {
		t.Fatal("partial namespace adopted")
	}
}

func TestNativeFreshExistingRootDoesNotCreateFence(t *testing.T) {
	parentPath := filepath.Join(t.TempDir(), "protected-parent")
	c := freshFixture(t, parentPath, true)
	if err := os.Mkdir(filepath.Join(parentPath, "UpdateBroker"), 0700); err != nil {
		t.Fatal(err)
	}
	if err := initializeFresh(c); err == nil {
		t.Fatal("preexisting root adopted")
	}
	if _, err := os.Stat(filepath.Join(parentPath, initializationFenceFilename)); !errors.Is(err, os.ErrNotExist) {
		t.Fatalf("fence created despite existing root: %v", err)
	}
}

func TestNativeFreshUnsafeParentRefusesBeforeFence(t *testing.T) {
	parentPath := filepath.Join(t.TempDir(), "protected-parent")
	c := freshFixture(t, parentPath, true)
	owner, _, err := c.directorySD.Owner()
	if err != nil {
		t.Fatal(err)
	}
	unsafeSD, err := windows.SecurityDescriptorFromString("O:" + owner.String() + "D:P(A;OICI;FA;;;" + owner.String() + ")(A;OICI;FA;;;SY)(A;;0x00000006;;;BU)")
	if err != nil {
		t.Fatal(err)
	}
	acl, _, err := unsafeSD.DACL()
	if err != nil {
		t.Fatal(err)
	}
	if err = windows.SetNamedSecurityInfo(parentPath, windows.SE_FILE_OBJECT, windows.DACL_SECURITY_INFORMATION|windows.PROTECTED_DACL_SECURITY_INFORMATION, nil, nil, acl, nil); err != nil {
		t.Fatal(err)
	}
	// Parent fixture exact check and strict ancestor policy both reject the grant.
	if err = checkAncestor(c.parent.handle, c.parent.ancestor); err == nil {
		t.Fatal("unsafe child-add grant accepted")
	}
	if err = initializeFresh(c); err == nil {
		t.Fatal("unsafe parent initialized")
	}
	if _, err = os.Stat(filepath.Join(parentPath, initializationFenceFilename)); !errors.Is(err, os.ErrNotExist) {
		t.Fatalf("unsafe parent consumed fence: %v", err)
	}
}

func TestNativeFreshOperationsRefuseOrdinaryPrincipal(t *testing.T) {
	user, err := windows.GetCurrentProcessToken().GetTokenUser()
	if err != nil {
		t.Fatal(err)
	}
	broker, _, _, err := windows.LookupSID("", BrokerAccount)
	if err == nil && user.User.Sid.Equals(broker) {
		t.Skip("ordinary token required")
	}
	for _, operation := range []func() error{InitializeFresh, ValidateInitialization} {
		if err = operation(); !errors.Is(err, ErrPrincipal) {
			t.Fatalf("ordinary principal result: %v", err)
		}
	}
}

func TestNativeFreshBrokerCheckRefusesSelfImpersonation(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	var impersonation windows.Token
	if err := windows.DuplicateTokenEx(windows.GetCurrentProcessToken(), windows.TOKEN_QUERY|windows.TOKEN_IMPERSONATE, nil, windows.SecurityImpersonation, windows.TokenImpersonation, &impersonation); err != nil {
		t.Fatal(err)
	}
	defer impersonation.Close()
	user, err := windows.GetCurrentProcessToken().GetTokenUser()
	if err != nil {
		t.Fatal(err)
	}
	if err = windows.SetThreadToken(nil, impersonation); err != nil {
		t.Fatal(err)
	}
	defer func() {
		if err := windows.RevertToSelf(); err != nil {
			t.Fatal(err)
		}
	}()
	if err = currentBroker(user.User.Sid); !errors.Is(err, ErrPrincipal) {
		t.Fatalf("same-user impersonation accepted: %v", err)
	}
}
