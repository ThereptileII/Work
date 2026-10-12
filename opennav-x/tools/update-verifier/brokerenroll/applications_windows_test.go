//go:build windows

package brokerenroll

import (
	"errors"
	"path/filepath"
	"testing"
	"unsafe"

	"golang.org/x/sys/windows"
)

// Native SDDL/ACL inspection only: these tests do not impersonate the broker,
// launch any image, provision ProgramFiles or qualify real enrollment execution.
func TestEnrollmentApplicationsAndCodeNativeAccessPolicy(t *testing.T) {
	broker, err := enrollmentBrokerSID()
	if err != nil {
		t.Fatal(err)
	}
	full := uint32(0x001f01ff) // FILE_ALL_ACCESS, native SDDL FA expansion.
	readExecute := uint32(windows.FILE_GENERIC_READ | windows.FILE_GENERIC_EXECUTE)
	concreteMutation := uint32(windows.DELETE|windows.WRITE_DAC|windows.WRITE_OWNER|windows.FILE_WRITE_DATA|windows.FILE_APPEND_DATA|windows.FILE_WRITE_EA|windows.FILE_WRITE_ATTRIBUTES) | 0x40
	mutation := concreteMutation | windows.GENERIC_ALL | windows.GENERIC_WRITE
	for _, scope := range []string{"SKAGER-parent", "UpdateBroker-directory", "broker-image", "Applications"} {
		t.Run(scope, func(t *testing.T) {
			directory := scope != "broker-image"
			sd, err := enrollmentCodeDescriptor(broker, directory)
			brokerMask := readExecute
			if scope == "Applications" {
				sd, err = enrollmentApplicationsDescriptor(broker)
				brokerMask = full
			}
			if err != nil {
				t.Fatal(err)
			}
			owner, _, err := sd.Owner()
			if err != nil || owner == nil || owner.String() != "S-1-5-32-544" {
				t.Fatal("final administrator ownership missing", err)
			}
			control, _, err := sd.Control()
			if err != nil || control&windows.SE_DACL_PROTECTED == 0 {
				t.Fatal("final DACL is not protected", err)
			}
			acl, _, err := sd.DACL()
			if err != nil || acl == nil || acl.AceCount != 4 {
				t.Fatal("exact final DACL absent", err)
			}
			wanted := map[string]uint32{"S-1-5-18": full, "S-1-5-32-544": full, "S-1-5-11": readExecute, broker.String(): brokerMask}
			seen := map[string]bool{}
			for i := uint32(0); i < uint32(acl.AceCount); i++ {
				var ace *windows.ACCESS_ALLOWED_ACE
				if err := windows.GetAce(acl, i, &ace); err != nil {
					t.Fatal(err)
				}
				sid := (*windows.SID)(unsafe.Pointer(&ace.SidStart))
				if !sid.IsValid() || ace.Header.AceType != windows.ACCESS_ALLOWED_ACE_TYPE || seen[sid.String()] {
					t.Fatal("unknown or repeated allow entry")
				}
				mask, ok := wanted[sid.String()]
				if !ok || uint32(ace.Mask) != mask {
					t.Fatalf("unexpected rights for %s: %x", sid.String(), ace.Mask)
				}
				flags := byte(0)
				if directory {
					flags = windows.OBJECT_INHERIT_ACE | windows.CONTAINER_INHERIT_ACE
				}
				if ace.Header.AceFlags != flags {
					t.Fatal("child inheritance differs")
				}
				if sid.String() == "S-1-5-11" && uint32(ace.Mask)&mutation != 0 {
					t.Fatal("ordinary user has namespace or image mutation rights")
				}
				if sid.String() == broker.String() {
					if scope == "Applications" && uint32(ace.Mask)&concreteMutation != concreteMutation {
						t.Fatal("broker cannot deploy inside Applications")
					}
					if scope != "Applications" && uint32(ace.Mask)&mutation != 0 {
						t.Fatal("broker can mutate its image or parent namespace")
					}
				}
				seen[sid.String()] = true
			}
			if err := enrollmentSecurityDescriptor(sd, sd, false, broker.String()); err != nil {
				t.Fatal("exact at-creation descriptor refused", err)
			}
		})
	}
	code, err := enrollmentCodeDescriptor(broker, true)
	if err != nil {
		t.Fatal(err)
	}
	applications, err := enrollmentApplicationsDescriptor(broker)
	if err != nil {
		t.Fatal(err)
	}
	if enrollmentSecurityDescriptor(applications, code, false, broker.String()) == nil || enrollmentSecurityDescriptor(code, applications, false, broker.String()) == nil {
		t.Fatal("application write rights accepted on code or omitted from launch root")
	}
	if _, err := enrollmentApplicationsDescriptor(nil); err == nil {
		t.Fatal("missing broker identity accepted")
	}
}

func TestEnrollmentApplicationsFreshNativePinAndNoAdoption(t *testing.T) {
	// Disposable inert directory only. Creating the specified BA owner requires
	// the same elevated operator prerequisite as real enrollment; no service or
	// machine KnownFolder is created or modified here.
	if !windows.GetCurrentProcessToken().IsElevated() {
		t.Skip("native fresh BA-owned directory fixture requires elevated operator")
	}
	if err := enrollmentOperator(); err != nil {
		t.Skip(err)
	}
	broker, err := enrollmentBrokerSID()
	if err != nil {
		t.Fatal(err)
	}
	code, err := enrollmentCodeDescriptor(broker, true)
	if err != nil {
		t.Fatal(err)
	}
	applications, err := enrollmentApplicationsDescriptor(broker)
	if err != nil {
		t.Fatal(err)
	}
	parentPath := filepath.Join(t.TempDir(), "SKAGER")
	pointer, err := windows.UTF16PtrFromString(parentPath)
	if err != nil {
		t.Fatal(err)
	}
	attributes := windows.SecurityAttributes{Length: uint32(unsafe.Sizeof(windows.SecurityAttributes{})), SecurityDescriptor: code}
	if err := windows.CreateDirectory(pointer, &attributes); err != nil {
		t.Fatal(err)
	}
	files := &nativeEnrollmentFiles{}
	t.Cleanup(func() {
		if err := files.Close(); err != nil {
			t.Error(err)
		}
	})
	parent, err := files.pinDirectory(parentPath, "", parentPath, code, false, "")
	if err != nil {
		t.Fatal(err)
	}
	root, err := files.newDirectory(parent, "Applications", applications, broker.String())
	if err != nil {
		t.Fatal(err)
	}
	if err := files.Check(); err != nil {
		t.Fatal("fresh launch root pin not valid", err)
	}
	if root.dos != filepath.Join(parentPath, "Applications") || root.descriptor != applications || root.broker != broker.String() {
		t.Fatal("fixed sibling identity or descriptor not retained")
	}
	if _, err := files.newDirectory(parent, "Applications", applications, broker.String()); !errors.Is(err, windows.ERROR_ALREADY_EXISTS) {
		t.Fatalf("existing launch root adopted: %v", err)
	}
	// The original pin remains valid after the exclusive-creation refusal.
	if err := files.Check(); err != nil {
		t.Fatal(err)
	}
	if err := files.Close(); err != nil {
		t.Fatal(err)
	}
	if err := files.Check(); err == nil {
		t.Fatal("closed root still usable")
	}
	if err := root.check(); err == nil {
		t.Fatal("launch root handle was not drained")
	}
}
