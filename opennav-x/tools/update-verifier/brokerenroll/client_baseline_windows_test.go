//go:build windows

package brokerenroll

import (
	"bytes"
	"context"
	"errors"
	"io"
	"os"
	"path/filepath"
	"testing"
	"unsafe"

	"golang.org/x/sys/windows"
)

// Only inert test-owned temporary bytes are used. The fixture deliberately
// supplies its own principal/descriptor to isolate retained native mechanics;
// it does not call either production constructor, SCM or a fixed machine path.
func TestClientBaselineNativeReadAtCustody(t *testing.T) {
	user, err := windows.GetCurrentProcessToken().GetTokenUser()
	if err != nil {
		t.Fatal(err)
	}
	sd, err := windows.SecurityDescriptorFromString("O:" + user.User.Sid.String() + "D:P(A;;FA;;;" + user.User.Sid.String() + ")")
	if err != nil {
		t.Fatal(err)
	}
	path := filepath.Join(t.TempDir(), "inert-baseline.zip")
	pointer, err := windows.UTF16PtrFromString(path)
	if err != nil {
		t.Fatal(err)
	}
	sa := windows.SecurityAttributes{Length: uint32(unsafe.Sizeof(windows.SecurityAttributes{})), SecurityDescriptor: sd}
	h, err := windows.CreateFile(pointer, windows.GENERIC_READ|windows.GENERIC_WRITE|windows.READ_CONTROL, windows.FILE_SHARE_READ, &sa, windows.CREATE_NEW, windows.FILE_FLAG_OPEN_REPARSE_POINT, 0)
	if err != nil {
		t.Fatal(err)
	}
	file := os.NewFile(uintptr(h), path)
	writerOpen := true
	defer func() {
		if writerOpen {
			_ = file.Close()
		}
	}()
	body := []byte("INERT retained baseline native read-at bytes")
	if _, err := file.Write(body); err != nil {
		t.Fatal(err)
	}
	if err := file.Sync(); err != nil {
		t.Fatal(err)
	}
	physical, err := enrollmentPhysicalPath(h)
	if err != nil {
		t.Skipf("native local fixture unavailable: %v", err)
	}
	pin := &enrollmentPin{handle: h, physical: physical, descriptor: sd, broker: user.User.Sid.String(), file: true, length: int64(len(body))}
	pin.info, pin.id, err = enrollmentInfo(h)
	if err != nil {
		t.Fatal(err)
	}
	originalID := pin.id
	if err := file.Close(); err != nil {
		t.Fatal(err)
	}
	writerOpen = false
	if err := reopenClientRequest(pin); err != nil {
		t.Fatal(err)
	}
	pin.dos = path
	owner := &nativeClientBaseline{ctx: context.Background(), files: &nativeEnrollmentFiles{pins: []*enrollmentPin{pin}}, pin: pin, broker: user.User.Sid, size: pin.length}
	defer func() {
		if err := owner.Close(); err != nil {
			t.Error(err)
		}
	}()
	if err := owner.Validate(); err != nil || pin.id != originalID {
		t.Fatalf("original identity lost: %v", err)
	}
	for _, off := range []int64{7, 0, 13, 2} {
		buffer := make([]byte, 5)
		if n, err := owner.ReadAt(buffer, off); n != len(buffer) || err != nil || !bytes.Equal(buffer, body[off:off+5]) {
			t.Fatalf("read-at offset=%d n=%d err=%v", off, n, err)
		}
	}
	if n, err := owner.ReadAt(make([]byte, 3), owner.Size()-1); n != 1 || !errors.Is(err, io.EOF) {
		t.Fatalf("EOF n=%d err=%v", n, err)
	}
	if _, err := owner.ReadAt(make([]byte, 1), -1); err == nil {
		t.Fatal("negative offset accepted")
	}
	for _, access := range []uint32{windows.GENERIC_WRITE, windows.DELETE} {
		other, err := windows.CreateFile(pointer, access, windows.FILE_SHARE_READ|windows.FILE_SHARE_WRITE|windows.FILE_SHARE_DELETE, nil, windows.OPEN_EXISTING, windows.FILE_FLAG_OPEN_REPARSE_POINT, 0)
		if err == nil {
			windows.CloseHandle(other)
			t.Fatal("retained baseline permits write/delete sharing")
		}
		if !errors.Is(err, windows.ERROR_SHARING_VIOLATION) {
			t.Fatalf("expected sharing refusal: %v", err)
		}
	}
	if err := owner.Close(); err != nil {
		t.Fatal(err)
	}
	if err := owner.Close(); err != nil {
		t.Fatal("cached successful close changed")
	}
	if owner.Validate() == nil {
		t.Fatal("closed custody validated")
	}
	if _, err := owner.ReadAt(make([]byte, 1), 0); err == nil {
		t.Fatal("closed custody read")
	}
}

func TestClientBaselineNativeCachedCloseFailure(t *testing.T) {
	want := errors.Join(ErrCleanup, errors.New("inert prior native close failure"))
	files := &nativeEnrollmentFiles{closed: true, closeErr: want}
	owner := &nativeClientBaseline{files: files}
	if err := owner.Close(); !errors.Is(err, want) {
		t.Fatal("close failure lost")
	}
	files.closeErr = nil
	if err := owner.Close(); !errors.Is(err, want) {
		t.Fatal("close retried or reported false success")
	}
}

func TestClientBaselineNativeHardlinkRefusal(t *testing.T) {
	path := filepath.Join(t.TempDir(), "inert-baseline.zip")
	if err := os.WriteFile(path, []byte("INERT"), 0600); err != nil {
		t.Fatal(err)
	}
	if err := os.Link(path, path+".alias"); err != nil {
		t.Skipf("native hardlink fixture unavailable: %v", err)
	}
	pointer, err := windows.UTF16PtrFromString(path)
	if err != nil {
		t.Fatal(err)
	}
	h, err := windows.CreateFile(pointer, windows.GENERIC_READ|windows.READ_CONTROL, windows.FILE_SHARE_READ, nil, windows.OPEN_EXISTING, windows.FILE_FLAG_OPEN_REPARSE_POINT, 0)
	if err != nil {
		t.Fatal(err)
	}
	defer windows.CloseHandle(h)
	physical, err := enrollmentPhysicalPath(h)
	if err != nil {
		t.Skipf("native local fixture unavailable: %v", err)
	}
	pin := &enrollmentPin{handle: h, physical: physical, file: true, length: 5}
	pin.info, pin.id, err = enrollmentInfo(h)
	if err != nil {
		t.Fatal(err)
	}
	if pin.info.NumberOfLinks != 2 {
		t.Fatal("fixture did not create an actual hardlink")
	}
	if err := pin.check(); err == nil {
		t.Fatal("hardlinked baseline accepted")
	}
}

func TestClientBaselineNativeRefusesOperatorConsoleFallback(t *testing.T) {
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
	if owner, err := OpenApprovedClientBaselineNative(context.Background(), inertClientSID); err == nil || owner != nil {
		t.Fatal("operator console granted broker baseline authority")
	}
}

func TestClientBaselinePartialPublicationCannotLookLikeNoClient(t *testing.T) {
	files := &nativeEnrollmentFiles{}
	if sid, sha := files.clientRegistration(); sid != "" || sha != "" {
		t.Fatal("no-client enrollment changed")
	}
	files.clientAttempted = true
	if sid, sha := files.clientRegistration(); sid == "" && sha == "" || canonicalClientSID(sid) && validPin(sha) {
		t.Fatal("partial request/baseline publication did not refuse SCM binding")
	}
	files.clientSID, files.clientSHA = inertClientSID, digest([]byte("approved original request"))
	if sid, sha := files.clientRegistration(); canonicalClientSID(sid) && validPin(sha) {
		t.Fatal("request without baseline permitted registration")
	}
	files.clientBaselineReady = true
	if sid, sha := files.clientRegistration(); sid != files.clientSID || sha != files.clientSHA {
		t.Fatal("completed baseline lost original request binding")
	}
}
