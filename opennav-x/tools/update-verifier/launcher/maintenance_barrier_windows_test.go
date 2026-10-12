package launcher

import (
	"bytes"
	"context"
	"encoding/binary"
	"errors"
	"os"
	"path/filepath"
	"runtime"
	"strings"
	"testing"
	"unsafe"

	"golang.org/x/sys/windows"
)

func TestNativeMaintenanceBarrierExactLegacyDenialAndRetainedLifetime(t *testing.T) {
	for _, kind := range []string{"absent-root", "existing-unarmed", "empty-marker", "garbage-marker", "directory-marker", "file-root", "missing-parent"} {
		t.Run(kind, func(t *testing.T) {
			local := t.TempDir()
			old := filepath.Join(local, "OpenNavXAlpha1")
			switch kind {
			case "existing-unarmed", "empty-marker", "garbage-marker", "directory-marker":
				if err := os.Mkdir(old, 0700); err != nil {
					t.Fatal(err)
				}
			case "file-root":
				if err := os.WriteFile(old, []byte("inert root"), 0600); err != nil {
					t.Fatal(err)
				}
			case "missing-parent":
				local = filepath.Join(local, "missing")
			}
			switch kind {
			case "empty-marker", "garbage-marker":
				raw := []byte(nil)
				if kind == "garbage-marker" {
					raw = []byte("inert denial only; never parse/adopt")
				}
				if err := os.WriteFile(filepath.Join(old, armedUpdateMarker), raw, 0600); err != nil {
					t.Fatal(err)
				}
			case "directory-marker":
				if err := os.Mkdir(filepath.Join(old, armedUpdateMarker), 0700); err != nil {
					t.Fatal(err)
				}
			}
			b, err := CaptureMaintenanceBarrierNative(context.Background(), local)
			if kind != "absent-root" && kind != "existing-unarmed" {
				if b != nil || !errors.Is(err, errArmedUpdateBarrier) {
					t.Fatal("present or ambiguous legacy marker accepted", err)
				}
				return
			}
			if err != nil {
				t.Fatal("exact absent marker refused", err)
			}
			defer b.Close()
			if len(b.pins) < 2 {
				t.Fatal("physical ancestor custody absent")
			}
			// A rename must not replace the captured LocalAppData directory while held.
			if err = os.Rename(local, local+"-replacement"); err == nil {
				os.Rename(local+"-replacement", local)
				t.Fatal("original LocalAppData was replaceable")
			}
			if err = b.Validate(context.Background()); err != nil {
				t.Fatal(err)
			}
			if err = b.Close(); err != nil {
				t.Fatal(err)
			}
			if err = b.Close(); err != nil {
				t.Fatal("repeat cleanup", err)
			}
			if err = b.Validate(context.Background()); !errors.Is(err, errArmedUpdateBarrier) {
				t.Fatal("drained directory owner reused", err)
			}
		})
	}
}

func TestNativeMaintenanceBarrierRechecksOriginalMarkerAndRevokesDenial(t *testing.T) {
	local := t.TempDir()
	b, err := CaptureMaintenanceBarrierNative(context.Background(), local)
	if err != nil {
		t.Fatal(err)
	}
	defer b.Close()
	old := filepath.Join(local, "OpenNavXAlpha1")
	if err = os.Mkdir(old, 0700); err != nil {
		t.Fatal(err)
	}
	marker := filepath.Join(old, armedUpdateMarker)
	raw := []byte("original denial evidence")
	if err = os.WriteFile(marker, raw, 0600); err != nil {
		t.Fatal(err)
	}
	if err = b.Validate(context.Background()); !errors.Is(err, errArmedUpdateBarrier) {
		t.Fatal("marker created during held custody ignored", err)
	}
	actual, err := os.ReadFile(marker)
	if err != nil || string(actual) != string(raw) {
		t.Fatal("denial marker repaired/deleted", err)
	}
	if err = os.Remove(marker); err != nil {
		t.Fatal(err)
	}
	if err = b.Validate(context.Background()); !errors.Is(err, errArmedUpdateBarrier) {
		t.Fatal("observed denial was silently restored")
	}
}

func TestNativeMaintenanceBarrierAliasPreservesOriginalIdentity(t *testing.T) {
	local := filepath.Join(t.TempDir(), "Original Local Application Data With Long Name")
	if err := os.Mkdir(local, 0700); err != nil {
		t.Fatal(err)
	}
	pointer, err := windows.UTF16PtrFromString(local)
	if err != nil {
		t.Fatal(err)
	}
	buffer := make([]uint16, 32768)
	n, err := windows.GetShortPathName(pointer, &buffer[0], uint32(len(buffer)))
	if err != nil || n == 0 || n >= uint32(len(buffer)) {
		t.Fatal("short alias lookup", err)
	}
	alias := windows.UTF16ToString(buffer[:n])
	if strings.EqualFold(alias, local) {
		t.Skip("fixture volume does not allocate short names; ordinary directory cases remain required")
	}
	b, err := CaptureMaintenanceBarrierNative(context.Background(), alias)
	if err != nil {
		t.Fatal("physical alias rejected", err)
	}
	defer b.Close()
	if err = b.Validate(context.Background()); err != nil {
		t.Fatal(err)
	}
	old := filepath.Join(local, "OpenNavXAlpha1")
	if err = os.Mkdir(old, 0700); err != nil {
		t.Fatal(err)
	}
	if err = os.WriteFile(filepath.Join(old, armedUpdateMarker), nil, 0600); err != nil {
		t.Fatal(err)
	}
	if err = b.Validate(context.Background()); !errors.Is(err, errArmedUpdateBarrier) {
		t.Fatal("alias concealed original physical marker", err)
	}
}

func TestNativeMaintenanceBarrierSubstRemapCannotReplaceOriginalDirectory(t *testing.T) {
	requireNativeOrdinarySubst(t)
	first, second := t.TempDir(), t.TempDir()
	device, remap := nativeOrdinarySubstMapping(t, first)
	b, err := CaptureMaintenanceBarrierNative(context.Background(), device+`\`)
	if err != nil {
		t.Fatal(err)
	}
	defer b.Close()
	other, err := openNativeArmedDirectory(second)
	if err != nil {
		t.Fatal(err)
	}
	physical := other.name
	if err = windows.CloseHandle(other.handle); err != nil {
		t.Fatal(err)
	}
	remap(physical)
	if err = b.Validate(context.Background()); !errors.Is(err, errArmedUpdateBarrier) {
		t.Fatal("DOS remap replaced retained original LocalAppData", err)
	}
}

func TestNativeMaintenanceBarrierInaccessibleLegacyRootRefuses(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()

	local := t.TempDir()
	old := filepath.Join(local, "OpenNavXAlpha1")
	if err := os.Mkdir(old, 0700); err != nil {
		t.Fatal(err)
	}
	b, err := CaptureMaintenanceBarrierNative(context.Background(), local)
	if err != nil {
		t.Fatal(err)
	}
	defer b.Close()
	pointer, err := windows.UTF16PtrFromString(old)
	if err != nil {
		t.Fatal(err)
	}
	control, err := windows.CreateFile(pointer, windows.READ_CONTROL|windows.WRITE_DAC, windows.FILE_SHARE_READ|windows.FILE_SHARE_WRITE|windows.FILE_SHARE_DELETE, nil, windows.OPEN_EXISTING, windows.FILE_FLAG_BACKUP_SEMANTICS|windows.FILE_FLAG_OPEN_REPARSE_POINT, 0)
	if err != nil {
		t.Fatal("fixture security-control owner", err)
	}
	defer windows.CloseHandle(control)
	original, err := windows.GetSecurityInfo(control, windows.SE_FILE_OBJECT, windows.DACL_SECURITY_INFORMATION)
	if err != nil {
		t.Fatal(err)
	}
	originalACL, _, err := original.DACL()
	if err != nil {
		t.Fatal(err)
	}
	defer func() {
		if err := windows.SetSecurityInfo(control, windows.SE_FILE_OBJECT, windows.DACL_SECURITY_INFORMATION|windows.UNPROTECTED_DACL_SECURITY_INFORMATION, nil, nil, originalACL, nil); err != nil {
			t.Error("restore inert ACL", err)
		}
	}()
	user, err := windows.GetCurrentProcessToken().GetTokenUser()
	if err != nil {
		t.Fatal(err)
	}
	denied, err := windows.SecurityDescriptorFromString("D:P(D;;FA;;;" + user.User.Sid.String() + ")")
	if err != nil {
		t.Fatal(err)
	}
	deniedACL, _, err := denied.DACL()
	if err != nil {
		t.Fatal(err)
	}
	if err = windows.SetSecurityInfo(control, windows.SE_FILE_OBJECT, windows.DACL_SECURITY_INFORMATION|windows.PROTECTED_DACL_SECURITY_INFORMATION, nil, nil, deniedACL, nil); err != nil {
		t.Fatal("inert root ACL denial", err)
	}
	// Independently read back the exact deny DACL. A successful setter alone is
	// insufficient, and backup semantics can bypass a DACL under a privileged CI
	// executor. Only a private duplicate is changed; the primary token is untouched.
	applied, err := windows.GetSecurityInfo(control, windows.SE_FILE_OBJECT, windows.DACL_SECURITY_INFORMATION)
	if err != nil {
		t.Fatal("inert root security readback", err)
	}
	appliedACL, appliedErr := brokerQueryACLBytes(applied)
	expectedACL, expectedErr := brokerQueryACLBytes(denied)
	appliedControl, _, controlErr := applied.Control()
	if appliedErr != nil || expectedErr != nil || controlErr != nil || appliedControl&windows.SE_DACL_PROTECTED == 0 || !bytes.Equal(appliedACL, expectedACL) {
		t.Fatal("fixture protected deny DACL differs", appliedErr, expectedErr, controlErr)
	}
	fixtureToken := maintenanceBarrierUnprivilegedToken(t)
	if err = windows.SetThreadToken(nil, fixtureToken); err != nil {
		t.Fatal("fixture token activation", err)
	}
	defer func() {
		if err := windows.RevertToSelf(); err != nil {
			t.Error("fixture token revert", err)
		}
	}()
	// The production observer has no generic impersonation gate. Confirm the
	// retained parent remains accessible so the test reaches legacy-root denial.
	parent, err := openNativeArmedDirectory(local)
	if err != nil {
		t.Fatal("fixture parent became inaccessible", err)
	}
	if err := windows.CloseHandle(parent.handle); err != nil {
		t.Fatal(err)
	}
	if probe, queryErr := openNativeArmedDirectory(old); queryErr == nil {
		windows.CloseHandle(probe.handle)
		t.Fatal("fixture legacy directory remained queryable")
	} else if !errors.Is(queryErr, windows.ERROR_ACCESS_DENIED) {
		t.Fatal("fixture did not establish native ACL denial", queryErr)
	}
	if err = b.Validate(context.Background()); !errors.Is(err, errArmedUpdateBarrier) {
		t.Fatal("inaccessible legacy directory treated as absence", err)
	}
}

func maintenanceBarrierJunctionFixture(t *testing.T, link, target string) {
	t.Helper()
	if err := os.Mkdir(link, 0700); err != nil {
		t.Fatal(err)
	}
	substitute, err := windows.UTF16FromString(`\??\` + target)
	if err != nil {
		t.Fatal(err)
	}
	printable, err := windows.UTF16FromString(target)
	if err != nil {
		t.Fatal(err)
	}
	// Native mount-point reparse buffer. Unlike a symlink, this inert directory
	// junction fixture does not require the caller's symlink creation privilege.
	names := append(substitute, printable...)
	buffer := make([]byte, 16+len(names)*2)
	binary.LittleEndian.PutUint32(buffer, windows.IO_REPARSE_TAG_MOUNT_POINT)
	binary.LittleEndian.PutUint16(buffer[4:], uint16(len(buffer)-8))
	binary.LittleEndian.PutUint16(buffer[10:], uint16((len(substitute)-1)*2))
	binary.LittleEndian.PutUint16(buffer[12:], uint16(len(substitute)*2))
	binary.LittleEndian.PutUint16(buffer[14:], uint16((len(printable)-1)*2))
	for index, value := range names {
		binary.LittleEndian.PutUint16(buffer[16+index*2:], value)
	}
	pointer, err := windows.UTF16PtrFromString(link)
	if err != nil {
		t.Fatal(err)
	}
	handle, err := windows.CreateFile(pointer, windows.GENERIC_WRITE, 0, nil, windows.OPEN_EXISTING, windows.FILE_FLAG_BACKUP_SEMANTICS|windows.FILE_FLAG_OPEN_REPARSE_POINT, 0)
	if err != nil {
		t.Fatal(err)
	}
	var used uint32
	setErr := windows.DeviceIoControl(handle, windows.FSCTL_SET_REPARSE_POINT, &buffer[0], uint32(len(buffer)), nil, 0, &used, nil)
	closeErr := windows.CloseHandle(handle)
	if setErr != nil || closeErr != nil {
		t.Fatal("inert junction fixture", setErr, closeErr)
	}
	attrs, err := windows.GetFileAttributes(pointer)
	if err != nil || attrs&windows.FILE_ATTRIBUTE_REPARSE_POINT == 0 {
		t.Fatal("junction fixture was not a native reparse point", err)
	}
}

func TestNativeMaintenanceBarrierReparseLocalOrLegacyDirectoryRefuses(t *testing.T) {
	for _, kind := range []string{"local", "ancestor", "legacy"} {
		t.Run(kind, func(t *testing.T) {
			base, target := t.TempDir(), t.TempDir()
			local := filepath.Join(base, "local")
			switch kind {
			case "local":
				maintenanceBarrierJunctionFixture(t, local, target)
			case "ancestor":
				maintenanceBarrierJunctionFixture(t, local, target)
				if err := os.Mkdir(filepath.Join(target, "child"), 0700); err != nil {
					t.Fatal(err)
				}
				local = filepath.Join(local, "child")
			case "legacy":
				if err := os.Mkdir(local, 0700); err != nil {
					t.Fatal(err)
				}
				maintenanceBarrierJunctionFixture(t, filepath.Join(local, "OpenNavXAlpha1"), target)
			}
			b, err := CaptureMaintenanceBarrierNative(context.Background(), local)
			if b != nil || !errors.Is(err, errArmedUpdateBarrier) {
				if b != nil {
					b.Close()
				}
				t.Fatal("reparse directory accepted as original plain custody", err)
			}
		})
	}
}

func TestNativeMaintenanceBarrierCancellationAndLostOriginalCustodyRefuse(t *testing.T) {
	for _, kind := range []string{"cancelled-capture", "cancelled-validation", "closed-original"} {
		t.Run(kind, func(t *testing.T) {
			local := t.TempDir()
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			if kind == "cancelled-capture" {
				cancel()
				b, err := CaptureMaintenanceBarrierNative(ctx, local)
				if b != nil || !errors.Is(err, context.Canceled) {
					t.Fatal("cancelled capture supplied custody", err)
				}
				return
			}
			b, err := CaptureMaintenanceBarrierNative(ctx, local)
			if err != nil {
				t.Fatal(err)
			}
			defer b.Close()
			if kind == "cancelled-validation" {
				cancel()
			} else {
				if err = windows.CloseHandle(b.pins[0].handle); err != nil {
					t.Fatal(err)
				}
				b.pins[0].handle = 0 // simulate missing original handle, not a PID/path reopen
			}
			if err = b.Validate(ctx); !errors.Is(err, errArmedUpdateBarrier) {
				t.Fatal("lost original custody or cancellation accepted", err)
			}
			if err = b.Validate(context.Background()); !errors.Is(err, errArmedUpdateBarrier) {
				t.Fatal("revoked owner revived from a fresh path lookup", err)
			}
		})
	}
}

// Duplicate only this test process token, disable privileges on the duplicate,
// and retain it for a single locked-thread fixture. No original token privilege,
// production grant, service, installation or profile is changed.
func maintenanceBarrierUnprivilegedToken(t *testing.T) windows.Token {
	t.Helper()
	var existing windows.Token
	err := windows.OpenThreadToken(windows.CurrentThread(), windows.TOKEN_QUERY, true, &existing)
	if err == nil {
		existing.Close()
		t.Fatal("fixture inherited impersonation")
	}
	if !errors.Is(err, windows.ERROR_NO_TOKEN) {
		t.Fatal(err)
	}
	var primary windows.Token
	if err = windows.OpenProcessToken(windows.CurrentProcess(), windows.TOKEN_QUERY|windows.TOKEN_DUPLICATE, &primary); err != nil {
		t.Fatal(err)
	}
	defer func() {
		if err := primary.Close(); err != nil {
			t.Error("fixture primary query close", err)
		}
	}()
	var token windows.Token
	if err = windows.DuplicateTokenEx(primary, windows.TOKEN_QUERY|windows.TOKEN_ADJUST_PRIVILEGES|windows.TOKEN_IMPERSONATE, nil, windows.SecurityImpersonation, windows.TokenImpersonation, &token); err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() {
		if err := token.Close(); err != nil {
			t.Error("fixture token close", err)
		}
	})
	if err = windows.AdjustTokenPrivileges(token, true, nil, 0, nil, nil); err != nil {
		t.Fatal("fixture privilege disable", err)
	}
	// Inspect the actual duplicate, rather than relying on adjustment success.
	var needed uint32
	err = windows.GetTokenInformation(token, windows.TokenPrivileges, nil, 0, &needed)
	if !errors.Is(err, windows.ERROR_INSUFFICIENT_BUFFER) || needed < 4 || needed > 64<<10 {
		t.Fatal("fixture privilege query size", needed, err)
	}
	buffer := make([]byte, needed)
	var used uint32
	if err = windows.GetTokenInformation(token, windows.TokenPrivileges, &buffer[0], needed, &used); err != nil || used < 4 || used > needed {
		t.Fatal("fixture privilege query", err)
	}
	privileges := (*windows.Tokenprivileges)(unsafe.Pointer(&buffer[0]))
	if uint64(privileges.PrivilegeCount) > uint64(used-4)/uint64(unsafe.Sizeof(windows.LUIDAndAttributes{})) {
		t.Fatal("fixture privilege query bound")
	}
	for _, privilege := range privileges.AllPrivileges() {
		if privilege.Attributes&windows.SE_PRIVILEGE_ENABLED != 0 {
			t.Fatal("fixture duplicate retained enabled privilege")
		}
	}
	return token
}
