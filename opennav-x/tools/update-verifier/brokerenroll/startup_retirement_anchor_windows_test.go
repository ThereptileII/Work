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
func prepareStartupFixtureRetirement(t *testing.T, a *startupRetirementAnchor, requestID string) func() error {
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
	return func() error {
		var status windows.IO_STATUS_BLOCK
		return windows.NtSetInformationFile(handle, &status, &buffer[0], uint32(len(buffer)), windows.FileRenameInformation)
	}
}

func nativePreparedStartupMaintenanceFixture(t *testing.T, original ...[]byte) (*nativeStartupRestoration, *MaintenanceOwner, *decommissionFixture) {
	t.Helper()
	n, request := startupRestorationNativeFixtureMode(t, true, original...)
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
	t.Cleanup(func() {
		if err := owner.Close(); err != nil {
			t.Error(err)
		}
	})
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
	return n, owner, d
}

func TestNativePreparedStartupRetirementAndRestorationCompose(t *testing.T) {
	t.Run("baseline selected", func(t *testing.T) { nativePreparedStartupComposition(t, false) })
	t.Run("successor selected keeps baseline anchor", func(t *testing.T) { nativePreparedStartupComposition(t, true) })
}

func nativePreparedStartupComposition(t *testing.T, selectedSuccessor bool) {
	t.Helper()
	n, owner, d := nativePreparedStartupMaintenanceFixture(t)
	ctx := context.Background()
	var originalHandles [3]windows.Handle
	var originalIDs [3]enrollmentFileID
	for i, pin := range n.anchor.pins {
		originalHandles[i], originalIDs[i] = pin.handle, pin.id
	}
	originalLink, err := startupReadOriginal(ctx, n.originals[0].pin)
	if err != nil {
		t.Fatal(err)
	}
	snapshot := preparedRetirementSnapshot(t, owner)
	if selectedSuccessor {
		snapshot.Generation = strings.Repeat("c", 32)
		if snapshot.Generation == n.anchor.generation {
			t.Fatal("fixture successor must differ from original startup")
		}
	}
	rename := prepareStartupFixtureRetirement(t, n.anchor, snapshot.RequestID)
	// Keep the actual failed NTFS boundary as an assertion: descendants open
	// with FILE_SHARE_DELETE still prevent rename of the original directory.
	if err := rename(); err == nil {
		t.Fatal("original app/launcher descendants did not prevent premature retirement")
	}
	if err := restorePreparedFixture(t, owner, snapshot, &d.events); err != nil {
		t.Fatal(err)
	}
	if n.anchor.pins[0].handle != originalHandles[0] || n.anchor.pins[0].id != originalIDs[0] || n.anchor.pins[1] != nil || n.anchor.pins[2] != nil || !n.anchor.drained {
		t.Fatal("original root lost or descendants not drained")
	}
	if n.retired != [3]bool{true, true, true} || !n.restored[0] {
		t.Fatal("startup restoration did not complete before original generation rename")
	}
	if _, err := owner.Decommission(ctx); err == nil || d.disables != 0 || d.stops != 0 || d.deletes != 0 {
		t.Fatal("interim restored/drained state authorized service effects")
	}
	if err := rename(); err != nil {
		t.Fatal("same original broker DELETE/root owners could not retire after child drain", err)
	}
	if _, err := os.Stat(filepath.Join(n.anchor.parent.dos, n.record.Generation)); !errors.Is(err, os.ErrNotExist) {
		t.Fatal("original generation still at old name", err)
	}
	if err := n.restore(ctx, func(context.Context) error { return nil }); err == nil || n.retired != [3]bool{true, true, true} {
		t.Fatal("native restoration replayed after completed original drain")
	}
	witness := &decommissionFenceFixture{events: &d.events}
	if err := bindDecommissionSnapshotWith(owner, ctx, witness, snapshot.ClientSID, snapshot.RegistrationSHA256, &snapshot); err != nil {
		t.Fatal(err)
	}
	if n.anchor.pins[0].handle != originalHandles[0] || n.anchor.pins[0].id != originalIDs[0] || n.anchor.pins[1] != nil || n.anchor.pins[2] != nil {
		t.Fatal("retirement reopened/adopted original root or drained descendants")
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
			n, owner, d := nativePreparedStartupMaintenanceFixture(t)
			snapshot := preparedRetirementSnapshot(t, owner)
			requestID := snapshot.RequestID
			if fault == "wrong-destination" {
				requestID = strings.Repeat("f", 32)
			}
			rename := prepareStartupFixtureRetirement(t, n.anchor, requestID)
			if fault == "descendant-creation" {
				// The launcher must validate before its original handle drains.
				// Root creation tunneling never excuses a changed descendant.
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
			guard := func(context.Context) error { return nil }
			if err := restorePreparedFixture(t, owner, snapshot, &d.events); err != nil {
				if fault != "descendant-creation" || n.retired != [3]bool{} {
					t.Fatal("original startup restoration failed before retirement", err)
				}
			} else {
				if fault == "descendant-creation" {
					t.Fatal("changed original launcher accepted before drain")
				}
				if fault != "no-rename" {
					if err := rename(); err != nil {
						t.Fatal("same original root could not rename after descendant drain", err)
					}
				}
			}
			switch fault {
			case "wrong-request":
				snapshot.RequestID = strings.Repeat("e", 32)
			case "wrong-generation":
				snapshot.Generation = strings.Repeat("f", 32)
			case "service-only":
				snapshot.RetirementSHA256 = ""
			}
			beforeRefusal := n.retired
			if err := bindDecommissionSnapshotWith(owner, context.Background(), &decommissionFenceFixture{events: &d.events}, snapshot.ClientSID, snapshot.RegistrationSHA256, &snapshot); err == nil {
				t.Fatal("foreign/changed original retirement was accepted")
			}
			if err := n.restore(context.Background(), guard); err == nil || n.retired != beforeRefusal {
				t.Fatal("refused retirement allowed repeated shortcut effects")
			}
			if err := bindDecommissionSnapshotWith(owner, context.Background(), &decommissionFenceFixture{events: &d.events}, snapshot.ClientSID, snapshot.RegistrationSHA256, &snapshot); err == nil {
				t.Fatal("refused anchor binding retried")
			}
			if _, err := owner.Decommission(context.Background()); err == nil || d.disables != 0 || d.stops != 0 || d.deletes != 0 || !owner.failed {
				t.Fatal("failed composed native handoff reached SCM or left owner reusable")
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
	rename := prepareStartupFixtureRetirement(t, n.anchor, snapshot.RequestID)
	checks := 0
	err := n.restore(context.Background(), func(context.Context) error {
		checks++
		if checks == 4 {
			return errors.New("original broker witness lost before second shortcut move")
		}
		return nil
	})
	if !errors.Is(err, ErrStartupRestorationUncertain) || !n.retired[0] || n.retired[1] || n.restored[0] {
		t.Fatal("partial joined restoration state lost", err)
	}
	if err := n.check(context.Background()); err != nil {
		t.Fatal("same retired launcher/recovery custody lost", err)
	}
	if err := n.restore(context.Background(), func(context.Context) error { return nil }); err == nil {
		t.Fatal("partial joined restoration retried")
	}
	if err := n.drain(context.Background(), func(context.Context) error { return nil }); err == nil || n.anchor.drained || n.anchor.pins[1] == nil || n.anchor.pins[2] == nil {
		t.Fatal("partial restoration drained original descendants")
	}
	if err := rename(); err == nil {
		t.Fatal("partial restoration permitted native generation retirement")
	}
	if _, err := os.Stat(filepath.Join(n.recovery.dos, startupRetirementName(startupNormal))); err != nil {
		t.Fatal("retired published normal recovery missing", err)
	}
	if _, err := os.Stat(filepath.Join(n.recovery.dos, "Skager.lnk")); err != nil {
		t.Fatal("original normal backup missing", err)
	}
}
