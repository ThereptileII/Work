//go:build windows

package brokerenroll

import (
	"bytes"
	"context"
	"errors"
	"os"
	"path/filepath"
	"strings"
	"testing"
	"unsafe"

	"golang.org/x/sys/windows"
)

// These fixtures touch only an inert test-owned temporary file, never the fixed
// KnownFolders, service manager, request pipe or production constructors. They
// exercise native mechanics; cross compilation does not qualify enrollment.
func TestClientRequestNativeReadonlyCustody(t *testing.T) {
	user, err := windows.GetCurrentProcessToken().GetTokenUser()
	if err != nil {
		t.Fatal(err)
	}
	sd, err := windows.SecurityDescriptorFromString("O:" + user.User.Sid.String() + "D:P(A;;FA;;;" + user.User.Sid.String() + ")")
	if err != nil {
		t.Fatal(err)
	}
	path := filepath.Join(t.TempDir(), "inert-request.json")
	p, err := windows.UTF16PtrFromString(path)
	if err != nil {
		t.Fatal(err)
	}
	sa := windows.SecurityAttributes{Length: uint32(unsafe.Sizeof(windows.SecurityAttributes{})), SecurityDescriptor: sd}
	h, err := windows.CreateFile(p, windows.GENERIC_READ|windows.GENERIC_WRITE|windows.READ_CONTROL, windows.FILE_SHARE_READ, &sa, windows.CREATE_NEW, windows.FILE_FLAG_OPEN_REPARSE_POINT, 0)
	if err != nil {
		t.Fatal(err)
	}
	file := os.NewFile(uintptr(h), path)
	open := true
	defer func() {
		if open {
			_ = file.Close()
		}
	}()
	body := []byte("INERT original approved public bytes")
	if _, err := file.Write(body); err != nil {
		t.Fatal(err)
	}
	if err := file.Sync(); err != nil {
		t.Fatal(err)
	}
	physical, err := enrollmentPhysicalPath(h)
	if err != nil {
		t.Skipf("local native fixture unavailable: %v", err)
	}
	pin := &enrollmentPin{handle: h, physical: physical, descriptor: sd, broker: user.User.Sid.String(), file: true, length: int64(len(body))}
	pin.info, pin.id, err = enrollmentInfo(h)
	if err != nil {
		t.Fatal(err)
	}
	if got, err := readClientRequestPin(context.Background(), pin); err != nil || !bytes.Equal(got, body) {
		t.Fatalf("writer readback: %q %v", got, err)
	}
	if err := file.Close(); err != nil {
		t.Fatal(err)
	}
	open = false
	if err := reopenClientRequest(pin); err != nil {
		t.Fatal(err)
	}
	defer windows.CloseHandle(pin.handle)
	pin.dos = path
	if got, err := readClientRequestPin(context.Background(), pin); err != nil || !bytes.Equal(got, body) {
		t.Fatalf("readonly identity/readback: %q %v", got, err)
	}
	for _, access := range []uint32{windows.GENERIC_WRITE, windows.DELETE} {
		h, err := windows.CreateFile(p, access, windows.FILE_SHARE_READ|windows.FILE_SHARE_WRITE|windows.FILE_SHARE_DELETE, nil, windows.OPEN_EXISTING, windows.FILE_FLAG_OPEN_REPARSE_POINT, 0)
		if err == nil {
			windows.CloseHandle(h)
			t.Fatal("retained request permits writer/delete custody")
		}
		if !errors.Is(err, windows.ERROR_SHARING_VIOLATION) {
			t.Fatalf("expected sharing refusal: %v", err)
		}
	}
	if h, err := windows.CreateFile(p, windows.GENERIC_WRITE, windows.FILE_SHARE_READ, &sa, windows.CREATE_NEW, windows.FILE_FLAG_OPEN_REPARSE_POINT, 0); err == nil {
		windows.CloseHandle(h)
		t.Fatal("partial namespace adopted")
	}
}

func TestClientRequestDescriptorExcludesClientAndBrokerMutation(t *testing.T) {
	broker, err := enrollmentBrokerSID()
	if err != nil {
		t.Fatal(err)
	}
	for _, directory := range []bool{false, true} {
		sd, err := clientRequestDescriptor(broker.String(), directory)
		if err != nil {
			t.Fatal(err)
		}
		if err := enrollmentSecurityDescriptor(sd, sd, false, broker.String()); err != nil {
			t.Fatal(err)
		}
		acl, _, err := sd.DACL()
		if err != nil || acl.AceCount != 3 {
			t.Fatal("exact request ACL differs")
		}
		var ace *windows.ACCESS_ALLOWED_ACE
		if err := windows.GetAce(acl, 2, &ace); err != nil {
			t.Fatal(err)
		}
		const fileDeleteChild = 0x40
		if !(*windows.SID)(unsafe.Pointer(&ace.SidStart)).Equals(broker) || ace.Mask&(windows.GENERIC_WRITE|windows.GENERIC_ALL|windows.WRITE_DAC|windows.WRITE_OWNER|windows.DELETE|windows.FILE_WRITE_DATA|windows.FILE_APPEND_DATA|fileDeleteChild) != 0 {
			t.Fatal("request broker mutation allowed")
		}
		broadened, err := windows.SecurityDescriptorFromString(strings.TrimSuffix(sd.String(), ")") + ")(A;;FR;;;BU)")
		if err != nil {
			t.Fatal(err)
		}
		if err := enrollmentSecurityDescriptor(broadened, sd, false, broker.String()); err == nil {
			t.Fatal("ordinary client request rights accepted")
		}
	}
}

func TestClientRequestNativeRefusesOperatorConsoleFallback(t *testing.T) {
	broker, err := enrollmentBrokerSID()
	if err != nil {
		t.Fatal(err)
	}
	user, err := windows.GetCurrentProcessToken().GetTokenUser()
	if err != nil {
		t.Fatal(err)
	}
	if user.User.Sid.Equals(broker) {
		t.Skip("fixture is actual broker")
	}
	if raw, err := ReadApprovedClientRequestNative(context.Background(), inertClientSID, strings.Repeat("a", 64)); err == nil || raw != nil {
		t.Fatal("operator console granted broker request authority")
	}
}

func TestClientServiceArgumentsRefuseBeforeNativeStart(t *testing.T) {
	n := &nativeServiceBackend{}
	for _, values := range [][2]string{{"S-1-5-18", strings.Repeat("a", 64)}, {inertClientSID, ""}, {inertClientSID, strings.Repeat("A", 64)}, {inertClientSID + "\\escape", strings.Repeat("a", 64)}} {
		if err := n.startClientRegistration(values[0], values[1]); err == nil {
			t.Fatal("invalid SCM arguments accepted")
		}
	}
}
