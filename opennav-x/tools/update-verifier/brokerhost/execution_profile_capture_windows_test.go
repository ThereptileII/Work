package brokerhost

import (
	"bytes"
	"context"
	"encoding/binary"
	"errors"
	"os"
	"path/filepath"
	"runtime"
	"testing"
	"unsafe"

	"example.com/opennav-update-verifier/brokerstore"
	"golang.org/x/sys/windows"
)

// Inert current-user lower-boundary fixture. It bypasses fixed ProgramData and
// named-process checks only for a private temporary source tree; production's
// factory has no path/descriptor/coldness-fake constructor. No real profile,
// installed application, process, plugin, service or physical output is changed.
func nativeProfileCaptureFixture(t *testing.T) (string, *nativeProfileCapture) {
	t.Helper()
	root := t.TempDir()
	if err := os.Mkdir(filepath.Join(root, "empty"), 0700); err != nil {
		t.Fatal(err)
	}
	if err := os.Mkdir(filepath.Join(root, "navigation"), 0700); err != nil {
		t.Fatal(err)
	}
	for name, raw := range map[string][]byte{"opencpn.ini": []byte("[Connections]\r\nCOM8=untouched\r\n"), "navigation/navobj.xml": []byte("original voyage\x00\xff"), "zero.bin": {}} {
		if err := os.WriteFile(filepath.Join(root, filepath.FromSlash(name)), raw, 0600); err != nil {
			t.Fatal(err)
		}
	}
	source, err := captureProfileTreePath(context.Background(), root, false)
	if err != nil {
		var watch *nativeProfileWatchDiagnostic
		if errors.As(err, &watch) {
			t.Logf("inert profile watch: stage=%s bytes=%d malformed=%t cause=%v notifications=%d", watch.stage, watch.bytes, watch.malformed, watch.cause, len(watch.notifications))
			for _, event := range watch.notifications {
				t.Logf("inert profile notification: action=%d name=%q", event.action, event.name)
			}
		}
		t.Fatal(err)
	}
	b := source.(*nativeProfileCapture)
	t.Cleanup(func() {
		if err := b.close(); err != nil {
			t.Error(err)
		}
	})
	return root, b
}
func TestNativeProfileWatchNotificationDecode(t *testing.T) {
	// A single FILE_NOTIFY_INFORMATION record for an inert test member.
	buf := make([]byte, 12+len("test.ini")*2)
	binary.LittleEndian.PutUint32(buf[4:], windows.FILE_ACTION_MODIFIED)
	binary.LittleEndian.PutUint32(buf[8:], uint32(len(buf)-12))
	for index, char := range "test.ini" {
		binary.LittleEndian.PutUint16(buf[12+index*2:], uint16(char))
	}
	events, malformed := profileWatchNotifications(buf)
	if malformed || len(events) != 1 || events[0].action != windows.FILE_ACTION_MODIFIED || events[0].name != "test.ini" {
		t.Fatalf("notification decoded incorrectly: %+v malformed=%t", events, malformed)
	}
	binary.LittleEndian.PutUint32(buf[8:], uint32(len(buf)))
	if _, malformed = profileWatchNotifications(buf); !malformed {
		t.Fatal("out-of-bounds notification name accepted")
	}
}
func nativeCompleteCapture(t *testing.T, source *nativeProfileCapture) (*ExecutionProfileCapture, *memoryProfileCaptureArchive) {
	t.Helper()
	archive := &memoryProfileCaptureArchive{}
	p, err := completeProfileCapture(context.Background(), source, archive, brokerstore.ExecutionJournalSnapshot{ClientSID: "S-1-5-21-1-2-3-1001", ExecutionID: "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa", Generation: "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb", CompletionSHA256: "cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc"})
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() {
		if err := p.Close(); err != nil {
			t.Error(err)
		}
	})
	return p, archive
}
func TestNativeExecutionProfileCaptureEntireTreeLocksAndReleasedInspection(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	root, source := nativeProfileCaptureFixture(t)
	p, archive := nativeCompleteCapture(t, source)
	entries, _ := source.originals()
	if len(entries) != 6 || !bytes.Contains(archive.raw, []byte("original voyage\x00\xff")) || !bytes.Contains(archive.raw, []byte("COM8=untouched")) {
		t.Fatal("entire native tree originals not retained")
	}
	file := filepath.Join(root, "opencpn.ini")
	if err := os.WriteFile(file, []byte("unreviewed writer"), 0600); !errors.Is(err, windows.ERROR_SHARING_VIOLATION) {
		t.Fatal("frozen original admitted writer", err)
	}
	if err := os.Remove(file); !errors.Is(err, windows.ERROR_SHARING_VIOLATION) {
		t.Fatal("frozen original admitted delete", err)
	}
	if err := p.ReleaseForQualification(context.Background()); err != nil {
		t.Fatal(err)
	}
	if err := p.VerifyRestoredOriginal(context.Background()); err != nil {
		t.Fatal("independently inspected original no-op", err)
	}
	// A normal application may replace INI atomically: original IDs are capture
	// history, and current source IDs must be owned by the fresh restore inspection.
	stage := filepath.Join(root, "atomic-new.ini")
	if err := os.WriteFile(stage, []byte("unreviewed changed profile"), 0600); err != nil {
		t.Fatal(err)
	}
	from, err := windows.UTF16PtrFromString(stage)
	if err != nil {
		t.Fatal(err)
	}
	to, err := windows.UTF16PtrFromString(file)
	if err != nil {
		t.Fatal(err)
	}
	if err = windows.MoveFileEx(from, to, windows.MOVEFILE_REPLACE_EXISTING|windows.MOVEFILE_WRITE_THROUGH); err != nil {
		t.Fatal(err)
	}
	if err = p.RestoreOriginal(context.Background()); !errors.Is(err, ErrExecutionProfileRestoreReviewRequired) {
		t.Fatal("changed native profile inferred restored", err)
	}
	raw, err := os.ReadFile(file)
	if err != nil || string(raw) != "unreviewed changed profile" {
		t.Fatal("refusal erased unreviewed user bytes", err)
	}
}
func TestNativeExecutionProfileCaptureMutationWatchAndMetadataRefuse(t *testing.T) {
	for _, kind := range []string{"namespace-add", "security", "full-id"} {
		t.Run(kind, func(t *testing.T) {
			runtime.LockOSThread()
			defer runtime.UnlockOSThread()
			root, source := nativeProfileCaptureFixture(t)
			switch kind {
			case "namespace-add":
				if err := os.WriteFile(filepath.Join(root, "new-navigation.gpx"), []byte("new voyage"), 0600); err != nil {
					t.Fatal(err)
				}
			case "security":
				// The retained file is intentionally opened without write sharing.
				// Change a held directory's DACL instead: directories allow write
				// sharing for namespace observation, and its security remains in
				// the same frozen capture metadata and subtree watch.
				user, err := windows.GetCurrentProcessToken().GetTokenUser()
				if err != nil {
					t.Fatal(err)
				}
				sd, err := windows.SecurityDescriptorFromString("D:P(A;;FA;;;" + user.User.Sid.String() + ")(A;;FR;;;AU)")
				if err != nil {
					t.Fatal(err)
				}
				acl, _, err := sd.DACL()
				if err != nil {
					t.Fatal(err)
				}
				pointer, err := windows.UTF16PtrFromString(filepath.Join(root, "navigation"))
				if err != nil {
					t.Fatal(err)
				}
				// Protected-DACL mutation and its actual readback require both
				// security read and write rights on this test-only handle.
				h, err := windows.CreateFile(pointer, windows.READ_CONTROL|windows.WRITE_DAC, windows.FILE_SHARE_READ|windows.FILE_SHARE_WRITE, nil, windows.OPEN_EXISTING, windows.FILE_FLAG_OPEN_REPARSE_POINT|windows.FILE_FLAG_BACKUP_SEMANTICS, 0)
				if err != nil {
					t.Fatal(err)
				}
				setErr := windows.SetSecurityInfo(h, windows.SE_FILE_OBJECT, windows.DACL_SECURITY_INFORMATION|windows.PROTECTED_DACL_SECURITY_INFORMATION, nil, nil, acl, nil)
				var readbackErr error
				if setErr == nil {
					readbackErr = nativeProfileMutatedDACL(h, acl)
				}
				closeErr := windows.CloseHandle(h)
				if setErr != nil {
					t.Fatalf("SetSecurityInfo actual directory DACL mutation: %v (close: %v)", setErr, closeErr)
				}
				if readbackErr != nil {
					t.Fatalf("actual mutated directory DACL readback: %v (close: %v)", readbackErr, closeErr)
				}
				if closeErr != nil {
					t.Fatal("directory DACL mutator close", closeErr)
				}
				found := false
				for _, member := range source.members {
					if member.entry.Path == "navigation" {
						found = true
						actual, err := profileMemberMetadata(member.file, member.entry.Path, true)
						if err != nil || actual.Security == member.entry.Security {
							t.Fatal("directory DACL mutation not independently observed", err)
						}
						break
					}
				}
				if !found {
					t.Fatal("held navigation directory missing")
				}
			case "full-id":
				source.members[len(source.members)-1].entry.Identity = "forged original full identity"
			}
			if err := source.validate(context.Background(), true); err == nil {
				t.Fatal("native full tree/watch/metadata mutation accepted")
			}
		})
	}
}

// Corroborate the requested mutation through the actual mutator handle. Windows
// may add AUTO_INHERITED control; compare protected DACL ACE bytes here while
// the retained production metadata still compares its original descriptor.
func nativeProfileMutatedDACL(handle windows.Handle, expected *windows.ACL) error {
	actual, err := windows.GetSecurityInfo(handle, windows.SE_FILE_OBJECT, windows.DACL_SECURITY_INFORMATION)
	if err != nil {
		return err
	}
	control, _, err := actual.Control()
	const required = windows.SE_DACL_PRESENT | windows.SE_DACL_PROTECTED
	if err != nil || control&required != required || control&(windows.SE_DACL_DEFAULTED|windows.SE_DACL_AUTO_INHERIT_REQ) != 0 {
		return errors.New("mutated profile DACL must remain present, protected and nondefaulted")
	}
	acl, _, err := actual.DACL()
	if err != nil || acl == nil || expected == nil || acl.AceCount != expected.AceCount {
		return errors.New("actual mutated profile DACL entry count differs")
	}
	for i := uint32(0); i < uint32(acl.AceCount); i++ {
		var ace, want *windows.ACCESS_ALLOWED_ACE
		if err := windows.GetAce(acl, i, &ace); err != nil {
			return err
		}
		if err := windows.GetAce(expected, i, &want); err != nil {
			return err
		}
		size := int(ace.Header.AceSize)
		if size < int(unsafe.Sizeof(windows.ACE_HEADER{})) || size != int(want.Header.AceSize) || !bytes.Equal(unsafe.Slice((*byte)(unsafe.Pointer(ace)), size), unsafe.Slice((*byte)(unsafe.Pointer(want)), size)) {
			return errors.New("actual mutated profile DACL ACE bytes differ")
		}
	}
	runtime.KeepAlive(actual)
	runtime.KeepAlive(expected)
	return nil
}

func TestNativeExecutionProfileCaptureHardlinksStreamsAndCancellationRefuse(t *testing.T) {
	for _, kind := range []string{"hardlink", "named-stream", "cancelled"} {
		t.Run(kind, func(t *testing.T) {
			runtime.LockOSThread()
			defer runtime.UnlockOSThread()
			root, source := nativeProfileCaptureFixture(t)
			if err := source.close(); err != nil {
				t.Fatal(err)
			}
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			switch kind {
			case "hardlink":
				if err := os.Link(filepath.Join(root, "opencpn.ini"), filepath.Join(root, "second-ini-link")); err != nil {
					t.Fatal(err)
				}
			case "named-stream":
				if err := os.WriteFile(filepath.Join(root, "opencpn.ini")+":unreviewed", []byte("unknown alternate stream"), 0600); err != nil {
					t.Fatal(err)
				}
			case "cancelled":
				cancel()
			}
			if owned, err := captureProfileTreePath(ctx, root, false); err == nil {
				owned.close()
				t.Fatal("unsafe native full profile captured")
			}
		})
	}
}
