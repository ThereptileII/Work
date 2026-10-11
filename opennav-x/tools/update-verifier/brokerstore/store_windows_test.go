package brokerstore

import (
	"errors"
	"fmt"
	"golang.org/x/sys/windows"
	"os"
	"path/filepath"
	"strings"
	"testing"
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
