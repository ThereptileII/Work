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

	"example.com/opennav-update-verifier/brokerpipe"
	"golang.org/x/sys/windows"
)

func preparedRetirementSnapshotForNative(n *nativeStartupRestoration) brokerpipe.DecommissionSnapshot {
	return brokerpipe.DecommissionSnapshot{RequestID: strings.Repeat("1", 32), ClientSID: n.clientSID,
		Generation: n.record.Generation, RetirementSHA256: strings.Repeat("a", 64)}
}

// Exercise the real producer boundary: acquire a DELETE owner while the
// consumer's original anchor is still open, then non-replacing native rename
// relative to the SAME held parent. No executable, SCM or real profile is used.
func retireStartupFixtureGeneration(t *testing.T, a *startupRetirementAnchor, requestID string) {
	t.Helper()
	pointer, err := windows.UTF16PtrFromString(enrollmentPhysical(a.pins[0].physical))
	if err != nil {
		t.Fatal(err)
	}
	handle, err := windows.CreateFile(pointer, windows.DELETE|windows.READ_CONTROL|windows.FILE_READ_ATTRIBUTES, windows.FILE_SHARE_READ, nil, windows.OPEN_EXISTING, windows.FILE_FLAG_OPEN_REPARSE_POINT|windows.FILE_FLAG_BACKUP_SEMANTICS, 0)
	if err != nil {
		t.Fatal("original preflight anchor prevented broker DELETE acquisition", err)
	}
	t.Cleanup(func() {
		if err := windows.CloseHandle(handle); err != nil {
			t.Error(err)
		}
	})
	_, id, err := enrollmentInfo(handle)
	if err != nil || id != a.pins[0].id {
		t.Fatal("producer did not retain original generation", err)
	}
	name := "retired-" + requestID + "-" + a.generation
	component, err := windows.UTF16FromString(name)
	if err != nil {
		t.Fatal(err)
	}
	component = component[:len(component)-1]
	type renameInfo struct {
		ReplaceIfExists uint32
		RootDirectory   windows.Handle
		FileNameLength  uint32
		FileName        [1]uint16
	}
	offset := unsafe.Offsetof(renameInfo{}.FileName)
	buffer := make([]byte, int(offset)+2*len(component))
	header := (*renameInfo)(unsafe.Pointer(&buffer[0]))
	header.RootDirectory, header.FileNameLength = a.parent.handle, uint32(2*len(component))
	copy(unsafe.Slice((*uint16)(unsafe.Pointer(&buffer[offset])), len(component)), component)
	var status windows.IO_STATUS_BLOCK
	if err := windows.NtSetInformationFile(handle, &status, &buffer[0], uint32(len(buffer)), windows.FileRenameInformation); err != nil {
		t.Fatal("original preflight anchor prevented native generation retirement", err)
	}
}

func TestNativePreparedStartupRetirementAndRestorationCompose(t *testing.T) {
	n, request := startupRestorationNativeFixtureMode(t, true)
	ctx := context.Background()
	// Only the existing fake SCM/process observation is substituted. Startup
	// preflight, retained handles, native generation move, relocation checks and
	// native shortcut restoration are the actual production implementation.
	d := newDecommissionFixture(t)
	d.req.ClientRegistrationRequest, d.req.ApprovedClientRequestSHA256 = request.OriginalBytes(), request.RequestSHA256
	d.op.authenticate = func(sid string) error {
		if sid != request.ClientSID {
			return errors.New("wrong native fixture client")
		}
		return nil
	}
	d.op.openDecommission = func(_ context.Context, authority maintenanceAuthority) (maintenanceBackend, error) {
		if !bytes.Equal(authority.client.OriginalBytes(), request.OriginalBytes()) {
			t.Fatal("fixture original client changed")
		}
		return d, nil
	}
	owner, err := openMaintenanceModeWith(ctx, d.req, d.op, true)
	if err != nil {
		t.Fatal(err)
	}
	defer owner.Close()
	approved, err := ParseApprovedStartupPublication(n.raw, digest(n.raw), request.OriginalBytes(), request.RequestSHA256)
	if err != nil {
		t.Fatal(err)
	}
	if err := prepareStartupRestorationWith(owner, ctx, approved, func(_ context.Context, authority maintenanceAuthority, record startupBindingRecord, raw []byte) (startupRestorationBackend, error) {
		if record.Generation != n.record.Generation || !bytes.Equal(raw, n.raw) {
			t.Fatal("prepared native publication changed")
		}
		return n, nil
	}); err != nil {
		t.Fatal(err)
	}
	var originalHandles [3]windows.Handle
	var originalIDs [3]enrollmentFileID
	for i, pin := range n.anchor.pins {
		originalHandles[i], originalIDs[i] = pin.handle, pin.id
	}
	originalLink, err := startupReadOriginal(ctx, n.original.pin)
	if err != nil {
		t.Fatal(err)
	}
	snapshot := preparedRetirementSnapshot(t, owner)
	retireStartupFixtureGeneration(t, n.anchor, snapshot.RequestID)
	if _, err := os.Stat(filepath.Join(n.anchor.parent.dos, n.record.Generation)); !errors.Is(err, os.ErrNotExist) {
		t.Fatal("original generation still at old name", err)
	}
	if err := n.restore(ctx, func(context.Context) error { return nil }); err == nil || n.retired != [3]bool{} {
		t.Fatal("native restoration preceded authenticated retirement binding")
	}
	witness := &decommissionFenceFixture{events: &d.events}
	if err := bindDecommissionSnapshotWith(owner, ctx, witness, snapshot.ClientSID, snapshot.RegistrationSHA256, &snapshot); err != nil {
		t.Fatal(err)
	}
	for i, pin := range n.anchor.pins {
		if pin.handle != originalHandles[i] || pin.id != originalIDs[i] {
			t.Fatal("retirement reopened/adopted startup custody")
		}
	}
	result, err := owner.Decommission(ctx)
	if err != nil || result.Service != ServiceMarkedForRemoval || d.disables != 1 || d.stops != 1 || d.deletes != 1 {
		t.Fatal("joined prepared retirement/restoration did not finish", result, err)
	}
	raw, err := os.ReadFile(filepath.Join(n.menu.dos, "Skager.lnk"))
	if err != nil || !bytes.Equal(raw, originalLink) {
		t.Fatal("exact original normal shortcut not restored", err)
	}
	for i := range n.links {
		if !n.retired[i] {
			t.Fatal("protected shortcut still published")
		}
	}
}

func TestNativeStartupRetirementAnchorRefusesWrongLocationAndChangedOriginal(t *testing.T) {
	for _, fault := range []string{"no-rename", "wrong-destination", "wrong-request", "wrong-generation", "service-only", "descendant-creation"} {
		t.Run(fault, func(t *testing.T) {
			n, _ := startupRestorationNativeFixtureMode(t, true)
			snapshot := preparedRetirementSnapshotForNative(n)
			if fault == "wrong-destination" {
				retireStartupFixtureGeneration(t, n.anchor, strings.Repeat("f", 32))
			} else if fault != "no-rename" {
				retireStartupFixtureGeneration(t, n.anchor, snapshot.RequestID)
			}
			switch fault {
			case "wrong-request":
				snapshot.RequestID = strings.Repeat("e", 32)
			case "wrong-generation":
				snapshot.Generation = strings.Repeat("f", 32)
			case "service-only":
				snapshot.RetirementSHA256 = ""
			case "descendant-creation":
				// A root rename may tunnel ROOT creation only. A changed launcher
				// creation baseline must not be accepted during that handoff.
				image := n.anchor.pins[2]
				name, err := enrollmentPhysicalPath(image.handle)
				if err != nil {
					t.Fatal(err)
				}
				pointer, err := windows.UTF16PtrFromString(enrollmentPhysical(name))
				if err != nil {
					t.Fatal(err)
				}
				handle, err := windows.CreateFile(pointer, windows.FILE_WRITE_ATTRIBUTES, windows.FILE_SHARE_READ|windows.FILE_SHARE_WRITE|windows.FILE_SHARE_DELETE, nil, windows.OPEN_EXISTING, windows.FILE_FLAG_OPEN_REPARSE_POINT, 0)
				if err != nil {
					t.Fatal(err)
				}
				changed := image.info.CreationTime
				changed.LowDateTime ^= 0x10000
				setErr := windows.SetFileTime(handle, &changed, nil, nil)
				if err := errors.Join(setErr, windows.CloseHandle(handle)); err != nil {
					t.Fatal(err)
				}
			}
			if err := n.bindRetirement(context.Background(), snapshot); err == nil {
				t.Fatal("foreign/changed original retirement was accepted")
			}
			if err := n.restore(context.Background(), func(context.Context) error { return nil }); err == nil || n.retired != [3]bool{} {
				t.Fatal("refused retirement allowed shortcut effects")
			}
			if err := n.bindRetirement(context.Background(), preparedRetirementSnapshotForNative(n)); err == nil {
				t.Fatal("refused anchor binding retried")
			}
		})
	}
}

func TestNativeStartupRetirementAnchorExcludesLauncherWrites(t *testing.T) {
	n, _ := startupRestorationNativeFixtureMode(t, true)
	pointer, err := windows.UTF16PtrFromString(n.anchor.dos[2])
	if err != nil {
		t.Fatal(err)
	}
	handle, err := windows.CreateFile(pointer, windows.GENERIC_WRITE, windows.FILE_SHARE_READ|windows.FILE_SHARE_WRITE|windows.FILE_SHARE_DELETE, nil, windows.OPEN_EXISTING, windows.FILE_FLAG_OPEN_REPARSE_POINT, 0)
	if err == nil {
		_ = windows.CloseHandle(handle)
		t.Fatal("move-aware original launcher reader permitted a writer")
	}
	if !errors.Is(err, windows.ERROR_SHARING_VIOLATION) {
		t.Fatal("write exclusion did not come from retained launcher sharing", err)
	}
	if err := n.check(context.Background()); err != nil {
		t.Fatal(err)
	}
}

func TestNativePreparedStartupRetirementPartialRestorationPreservesRecovery(t *testing.T) {
	n, _ := startupRestorationNativeFixtureMode(t, true)
	snapshot := preparedRetirementSnapshotForNative(n)
	retireStartupFixtureGeneration(t, n.anchor, snapshot.RequestID)
	if err := n.bindRetirement(context.Background(), snapshot); err != nil {
		t.Fatal(err)
	}
	checks := 0
	err := n.restore(context.Background(), func(context.Context) error {
		checks++
		if checks == 4 {
			return errors.New("original broker witness lost before second shortcut move")
		}
		return nil
	})
	if !errors.Is(err, ErrStartupRestorationUncertain) || !n.retired[0] || n.retired[1] || n.restored {
		t.Fatal("partial joined restoration state lost", err)
	}
	if err := n.check(context.Background()); err != nil {
		t.Fatal("same retired launcher/recovery custody lost", err)
	}
	if err := n.restore(context.Background(), func(context.Context) error { return nil }); err == nil {
		t.Fatal("partial joined restoration retried")
	}
	if _, err := os.Stat(filepath.Join(n.recovery.dos, startupRetirementName(startupNormal))); err != nil {
		t.Fatal("retired published normal recovery missing", err)
	}
	if _, err := os.Stat(filepath.Join(n.recovery.dos, "Skager.lnk")); err != nil {
		t.Fatal("original normal backup missing", err)
	}
}
