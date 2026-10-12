//go:build windows

package brokerenroll

import (
	"context"
	"os"
	"testing"

	"golang.org/x/sys/windows"
)

// Read-only observations of the test process and inert temporary files only.
// No installed service, machine store, application or real profile is changed.
func TestNativeMaintenanceProcessRetainsActualIdentityWithoutSCM(t *testing.T) {
	identity, err := maintenanceToken(windows.GetCurrentProcessToken())
	if err != nil {
		t.Fatal(err)
	}
	path, err := os.Executable()
	if err != nil {
		t.Fatal(err)
	}
	name, err := windows.UTF16PtrFromString(path)
	if err != nil {
		t.Fatal(err)
	}
	handle, err := windows.CreateFile(name, windows.GENERIC_READ|windows.READ_CONTROL, windows.FILE_SHARE_READ, nil, windows.OPEN_EXISTING, windows.FILE_FLAG_OPEN_REPARSE_POINT, 0)
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() {
		if err := windows.CloseHandle(handle); err != nil {
			t.Error(err)
		}
	})
	image := &enrollmentPin{handle: handle, file: true, broker: identity.sid, dos: path}
	image.info, image.id, err = enrollmentInfo(handle)
	if err != nil {
		t.Fatal(err)
	}
	image.length = int64(uint64(image.info.FileSizeHigh)<<32 | uint64(image.info.FileSizeLow))
	image.physical, err = enrollmentPhysicalPath(handle)
	if err != nil {
		t.Fatal(err)
	}
	image.descriptor, err = windows.GetSecurityInfo(handle, windows.SE_FILE_OBJECT, windows.OWNER_SECURITY_INFORMATION|windows.DACL_SECURITY_INFORMATION)
	if err != nil {
		t.Fatal(err)
	}
	process, err := openMaintenanceProcess(windows.GetCurrentProcessId(), image, identity.sid, identity.session)
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() {
		if err := process.close(); err != nil {
			t.Error(err)
		}
	})
	native := process.(*nativeMaintenanceProcess)
	if err := native.live(); err != nil {
		t.Fatal(err)
	}
	if exited, _, err := native.exit(); err != nil || exited {
		t.Fatal("live current process misclassified as exited", exited, err)
	}
	for _, mutation := range []string{"pid", "creation", "token", "image"} {
		t.Run(mutation, func(t *testing.T) {
			copy := *native
			switch mutation {
			case "pid":
				copy.pid++
			case "creation":
				copy.created.LowDateTime ^= 1
			case "token":
				copy.identity.tokenID.LowPart ^= 1
			case "image":
				changed := *image
				changed.physical += ".different"
				copy.image = &changed
			}
			if err := copy.live(); err == nil {
				t.Fatal("changed original process identity accepted")
			}
		})
	}
	if other, err := openMaintenanceProcess(windows.GetCurrentProcessId(), image, identity.sid, identity.session+1); err == nil || other != nil {
		t.Fatal("different process session accepted")
	}
}

func TestNativeMaintenanceInertProtectedBytesKeepReadonlyCustody(t *testing.T) {
	owner, _, recovery, sd := startupNativeFixture(t)
	raw := []byte("INERT-MAINTENANCE-APPROVED-BROKER-BYTES")
	if _, err := startupWriteFile(context.Background(), owner.files, recovery, "inert-broker.exe", raw, sd, false); err != nil {
		t.Fatal(err)
	}
	pin, err := maintenanceHoldFile(owner.files, recovery, "inert-broker.exe", sd, maxBrokerBytes)
	if err != nil {
		t.Fatal(err)
	}
	if err := startupHashPin(context.Background(), pin, digest(raw)); err != nil {
		t.Fatal(err)
	}
	if err := startupHashPin(context.Background(), pin, digest([]byte("DIFFERENT"))); err == nil {
		t.Fatal("self-selected or changed broker hash accepted")
	}
	if err := os.WriteFile(pin.dos, []byte("REPLACEMENT"), 0600); err == nil {
		t.Fatal("retained maintenance bytes allowed an unowned writer")
	}
	if err := os.Remove(pin.dos); err == nil {
		t.Fatal("retained maintenance file allowed deletion")
	}
	if err := startupHashPin(context.Background(), pin, digest(raw)); err != nil {
		t.Fatal("original inert bytes lost", err)
	}
}
