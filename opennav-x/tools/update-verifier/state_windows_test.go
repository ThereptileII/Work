package verifier

import (
	"path/filepath"
	"testing"

	"golang.org/x/sys/windows"
)

func TestPrivateStateRejectsBroadenedWindowsPermissions(t *testing.T) {
	dir := filepath.Join(t.TempDir(), "private")
	if err := makePrivateStateDirectory(dir); err != nil {
		t.Fatal(err)
	}
	sd, err := windows.SecurityDescriptorFromString("D:P(A;OICI;FA;;;WD)")
	if err != nil {
		t.Fatal(err)
	}
	dacl, _, err := sd.DACL()
	if err != nil {
		t.Fatal(err)
	}
	if err := windows.SetNamedSecurityInfo(dir, windows.SE_FILE_OBJECT,
		windows.DACL_SECURITY_INFORMATION|windows.PROTECTED_DACL_SECURITY_INFORMATION,
		nil, nil, dacl, nil); err != nil {
		t.Fatal(err)
	}
	if err := assertPrivateStateDirectory(dir); err == nil {
		t.Fatal("world-writable retained trust store accepted")
	}
}
