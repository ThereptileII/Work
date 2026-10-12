//go:build windows

package brokerenroll

import (
	"encoding/binary"
	"errors"
	"os"
	"path/filepath"
	"testing"
	"unicode/utf16"

	"golang.org/x/sys/windows"
)

// SCRUM-311: exercise actual NTFS semantics underlying a proposed retained-child
// ancestry guard. All effects are confined to these newly created temp paths;
// this test grants no enrollment authority and changes no production ACL policy.
func TestNativeEnrollmentAncestorRetainedChildBlocksReparseAndRemoval(t *testing.T) {
	base := t.TempDir()
	ancestor := filepath.Join(base, "ancestor")
	target := filepath.Join(base, "target")
	for _, path := range []string{ancestor, target} {
		if err := os.Mkdir(path, 0700); err != nil {
			t.Fatal(err)
		}
	}
	user, err := windows.GetCurrentProcessToken().GetTokenUser()
	if err != nil {
		t.Fatal(err)
	}
	files := &nativeEnrollmentFiles{}
	t.Cleanup(func() {
		if err := files.Close(); err != nil {
			t.Error("original temp directory owners close:", err)
		}
	})
	parent, err := files.pinDirectory(ancestor, "", ancestor, nil, true, user.User.Sid.String())
	if err != nil {
		t.Fatal(err)
	}
	var serial, component, flags uint32
	fs := make([]uint16, 32)
	if err := windows.GetVolumeInformationByHandle(parent.handle, nil, 0, &serial, &component, &flags, &fs[0], uint32(len(fs))); err != nil ||
		windows.UTF16ToString(fs) != "NTFS" || serial != parent.info.VolumeSerialNumber {
		t.Fatal("actual native NTFS temp fixture required:", err)
	}
	pointer, err := windows.UTF16PtrFromString(ancestor)
	if err != nil {
		t.Fatal(err)
	}
	// Request only WA: success cannot be attributed to requesting WRITE_DATA,
	// GENERIC_WRITE or DELETE. Metadata opens are not blocked by sharing modes.
	attributes, err := windows.CreateFile(pointer, windows.FILE_WRITE_ATTRIBUTES,
		windows.FILE_SHARE_READ|windows.FILE_SHARE_WRITE, nil, windows.OPEN_EXISTING,
		windows.FILE_FLAG_OPEN_REPARSE_POINT|windows.FILE_FLAG_BACKUP_SEMANTICS, 0)
	if err != nil {
		t.Fatal("attribute-only original directory open:", err)
	}
	active := false
	t.Cleanup(func() {
		if active {
			if err := ancestorFixtureReparse(attributes, false, nil); err != nil {
				t.Error("remove only original temp junction metadata:", err)
			}
		}
		if err := windows.CloseHandle(attributes); err != nil {
			t.Error("attribute-only handle close:", err)
		}
	})
	buffer := ancestorFixtureMountPoint(t, target)
	if err := ancestorFixtureReparse(attributes, true, buffer); err != nil {
		t.Fatal("empty directory must accept valid WA-only mount-point control:", err)
	}
	active = true
	info, id, err := enrollmentInfo(parent.handle)
	if err != nil || id != parent.id || info.FileAttributes&windows.FILE_ATTRIBUTE_REPARSE_POINT == 0 {
		t.Fatal("actual original directory reparse transition not observed:", err)
	}
	if err := parent.check(); err == nil {
		t.Fatal("original owner accepted actual reparse mutation")
	}
	if err := ancestorFixtureReparse(attributes, false, nil); err != nil {
		t.Fatal("restore only test-created junction metadata:", err)
	}
	active = false
	if err := parent.check(); err != nil {
		t.Fatal("original directory identity after junction removal:", err)
	}

	childPath := filepath.Join(ancestor, "child")
	if err := os.Mkdir(childPath, 0700); err != nil {
		t.Fatal(err)
	}
	child, err := files.pinDirectory(childPath, parent.physical+`\child`, childPath, nil, false, user.User.Sid.String())
	if err != nil {
		t.Fatal(err)
	}
	if child.id.Volume != parent.id.Volume || child.id == parent.id {
		t.Fatal("original immediate child requires distinct same-volume full identity")
	}
	for _, corrupt := range []func(*enrollmentPin){
		func(p *enrollmentPin) { p.id.ID[0] ^= 1 },
		func(p *enrollmentPin) { p.id.Volume ^= 1 },
		func(p *enrollmentPin) { p.physical = parent.physical + `\other` },
		func(p *enrollmentPin) { p.dos = target },
	} {
		wrong := *child
		corrupt(&wrong)
		if err := wrong.check(); err == nil {
			t.Fatal("original child accepted corrupted full identity, volume or parent/path binding")
		}
	}
	if err := ancestorFixtureReparse(attributes, true, buffer); !errors.Is(err, windows.ERROR_DIR_NOT_EMPTY) {
		active = err == nil
		t.Fatal("retained immediate child must prevent same WA-only reparse operation:", err)
	}
	if err := parent.check(); err != nil {
		t.Fatal("failed reparse operation changed original ancestor:", err)
	}
	from, err := windows.UTF16PtrFromString(childPath)
	if err != nil {
		t.Fatal(err)
	}
	to, err := windows.UTF16PtrFromString(filepath.Join(ancestor, "renamed"))
	if err != nil {
		t.Fatal(err)
	}
	if err := windows.MoveFileEx(from, to, 0); !errors.Is(err, windows.ERROR_SHARING_VIOLATION) {
		t.Fatal("original retained child rename must fail sharing:", err)
	}
	if err := windows.RemoveDirectory(from); !errors.Is(err, windows.ERROR_SHARING_VIOLATION) {
		t.Fatal("original retained child removal must fail sharing:", err)
	}
	if err := child.check(); err != nil {
		t.Fatal("original child changed after denied rename/removal:", err)
	}
	if err := files.Close(); err != nil {
		t.Fatal(err)
	}
	if err := child.check(); err == nil {
		t.Fatal("closed original child retained apparent custody")
	}
	if err := windows.MoveFileEx(from, to, 0); err != nil {
		t.Fatal("original child rename after owner close:", err)
	}
	if err := windows.RemoveDirectory(to); err != nil {
		t.Fatal("original child removal after owner close:", err)
	}
}

func ancestorFixtureReparse(handle windows.Handle, set bool, buffer []byte) error {
	control := uint32(windows.FSCTL_SET_REPARSE_POINT)
	if !set {
		control = windows.FSCTL_DELETE_REPARSE_POINT
		buffer = make([]byte, 8)
		binary.LittleEndian.PutUint32(buffer, windows.IO_REPARSE_TAG_MOUNT_POINT)
	}
	var returned uint32
	return windows.DeviceIoControl(handle, control, &buffer[0], uint32(len(buffer)), nil, 0, &returned, nil)
}

func ancestorFixtureMountPoint(t *testing.T, target string) []byte {
	t.Helper()
	substitute := utf16.Encode([]rune(`\??\` + target))
	printName := utf16.Encode([]rune(target))
	buffer := make([]byte, 16+2*(len(substitute)+1+len(printName)+1))
	if len(buffer) > 16384 {
		t.Fatal("bounded temp junction target required")
	}
	binary.LittleEndian.PutUint32(buffer, windows.IO_REPARSE_TAG_MOUNT_POINT)
	binary.LittleEndian.PutUint16(buffer[4:], uint16(len(buffer)-8))
	binary.LittleEndian.PutUint16(buffer[10:], uint16(2*len(substitute)))
	binary.LittleEndian.PutUint16(buffer[12:], uint16(2*(len(substitute)+1)))
	binary.LittleEndian.PutUint16(buffer[14:], uint16(2*len(printName)))
	for i, unit := range append(append(append(substitute, 0), printName...), 0) {
		binary.LittleEndian.PutUint16(buffer[16+2*i:], unit)
	}
	return buffer
}
