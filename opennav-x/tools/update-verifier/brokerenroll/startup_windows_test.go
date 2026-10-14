//go:build windows

package brokerenroll

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"os"
	"path/filepath"
	"runtime"
	"strings"
	"syscall"
	"testing"
	"unsafe"

	"golang.org/x/sys/windows"
)

// Inert native fixtures only. No SCM, fixed installed tree or real user menu is
// changed, no product is executed, and success is not installed-flow acceptance.
func TestNativeStartupShellLinkPersistsFixedProtectedArguments(t *testing.T) {
	target := filepath.Join(t.TempDir(), "skager-start.exe")
	if err := os.WriteFile(target, []byte("INERT-NOT-EXECUTABLE"), 0600); err != nil {
		t.Fatal(err)
	}
	for _, tc := range []struct {
		mode   startupShortcutMode
		suffix string
	}{{startupNormal, ""}, {startupLegacy, ":legacy"}, {startupSafe, ":safe"}} {
		t.Run(tc.suffix, func(t *testing.T) {
			arguments, err := tc.mode.arguments(strings.Repeat("b", 32))
			if err != nil || arguments != "--broker-start="+strings.Repeat("b", 32)+tc.suffix {
				t.Fatalf("fixed broker arguments %q %v", arguments, err)
			}
			startupAssertPersistedLink(t, target, arguments)
		})
	}
}
func startupAssertPersistedLink(t *testing.T, target, arguments string) {
	t.Helper()
	raw, err := startupLinkBytes(target, arguments, filepath.Dir(target))
	if err != nil {
		t.Fatal(err)
	}
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if err := windows.CoInitializeEx(0, 2); err != nil && !errors.Is(err, syscall.Errno(1)) {
		t.Fatal(err)
	}
	defer startupUninitialize.Call()
	clsid, _ := windows.GUIDFromString("{00021401-0000-0000-C000-000000000046}")
	classIID, _ := windows.GUIDFromString("{00000001-0000-0000-C000-000000000046}")
	linkIID, _ := windows.GUIDFromString("{000214F9-0000-0000-C000-000000000046}")
	persistIID, _ := windows.GUIDFromString("{00000109-0000-0000-C000-000000000046}")
	var factory, link, persist, stream *startupCOM
	hr, _, _ := startupShellFactory.Call(uintptr(unsafe.Pointer(&clsid)), uintptr(unsafe.Pointer(&classIID)), uintptr(unsafe.Pointer(&factory)))
	if int32(hr) < 0 || factory == nil {
		t.Fatalf("actual system ShellLink factory HRESULT %#x", uint32(hr))
	}
	defer startupCOMRelease(factory)
	if err := startupCOMCall(factory, 3, 0, uintptr(unsafe.Pointer(&linkIID)), uintptr(unsafe.Pointer(&link))); err != nil {
		t.Fatal(err)
	}
	defer startupCOMRelease(link)
	if err := startupCOMCall(link, 0, uintptr(unsafe.Pointer(&persistIID)), uintptr(unsafe.Pointer(&persist))); err != nil {
		t.Fatal(err)
	}
	defer startupCOMRelease(persist)
	hr, _, _ = startupMemoryStream.Call(0, 1, uintptr(unsafe.Pointer(&stream)))
	if int32(hr) < 0 || stream == nil {
		t.Fatal("actual native stream unavailable")
	}
	defer startupCOMRelease(stream)
	var used uint32
	if err := startupCOMCall(stream, 4, uintptr(unsafe.Pointer(&raw[0])), uintptr(len(raw)), uintptr(unsafe.Pointer(&used))); err != nil || used != uint32(len(raw)) {
		t.Fatalf("actual stream write %d %v", used, err)
	}
	if err := startupStreamRewind(stream); err != nil {
		t.Fatal(err)
	}
	if err := startupCOMCall(persist, 5, uintptr(unsafe.Pointer(stream))); err != nil {
		t.Fatal(err)
	}
	// IShellLink.GetPath expands an existing target's 8.3 components even with
	// SLGP_RAWPATH. The hosted runner's TempDir can contain RUNNER~1. Derive the
	// expected long path independently from Win32 rather than weakening the
	// assertion to a suffix, basename or case-insensitive string comparison.
	targetUTF16, err := windows.UTF16PtrFromString(target)
	if err != nil {
		t.Fatal(err)
	}
	longTarget := make([]uint16, 32768)
	n, err := windows.GetLongPathName(targetUTF16, &longTarget[0], uint32(len(longTarget)))
	if err != nil || n == 0 || n >= uint32(len(longTarget)) {
		t.Fatalf("independent native long target unavailable: length=%d err=%v", n, err)
	}
	expectedTarget := windows.UTF16ToString(longTarget[:n])
	for _, field := range []struct {
		name   string
		method int
		want   string
	}{{"target", 3, expectedTarget}, {"arguments", 10, arguments}, {"working-directory", 8, filepath.Dir(target)}} {
		// COM methods run on this retained original apartment thread, before t.Run
		// starts an assertion-only goroutine.
		buffer := make([]uint16, 32768)
		args := []uintptr{uintptr(unsafe.Pointer(&buffer[0])), uintptr(len(buffer))}
		if field.method == 3 {
			args = append(args, 0, 4)
		}
		err := startupCOMCall(link, field.method, args...)
		got := windows.UTF16ToString(buffer)
		t.Run(field.name, func(t *testing.T) {
			if err != nil {
				t.Fatal(err)
			}
			if got != field.want {
				t.Fatalf("actual persisted %s differs got=%q want=%q", field.name, got, field.want)
			}
		})
	}

}

func startupNativeFixture(t *testing.T) (*nativeStartupPublication, *enrollmentPin, *enrollmentPin, *windows.SECURITY_DESCRIPTOR) {
	t.Helper()
	if !windows.GetCurrentProcessToken().IsElevated() {
		t.Skip("inert fresh protected-directory fixture requires elevation; not a pass")
	}
	user, err := windows.GetCurrentProcessToken().GetTokenUser()
	if err != nil {
		t.Fatal(err)
	}
	sid := user.User.Sid.String()
	// TempDir cleanup runs after the native owners have closed (Cleanup is LIFO).
	path := t.TempDir()
	files := &nativeEnrollmentFiles{}
	t.Cleanup(func() {
		if err := files.Close(); err != nil {
			t.Errorf("native owners close: %v", err)
		}
	})
	parent, err := files.pinDirectory(path, "", path, nil, false, sid)
	if err != nil {
		t.Fatal(err)
	}
	sd, err := windows.SecurityDescriptorFromString("O:BAD:P(A;OICI;FA;;;SY)(A;OICI;FA;;;BA)(A;OICI;FRFX;;;" + sid + ")")
	if err != nil {
		t.Fatal(err)
	}
	menu, err := files.newDirectory(parent, "menu", sd, sid)
	if err != nil {
		t.Fatal(err)
	}
	recovery, err := files.newDirectory(parent, "recovery", sd, sid)
	if err != nil {
		t.Fatal(err)
	}
	return &nativeStartupPublication{files: files, clientSID: sid}, menu, recovery, sd
}
func TestNativeStartupPublicationPreservesOriginalIdentityAndOtherLinks(t *testing.T) {
	owner, menu, recovery, sd := startupNativeFixture(t)
	old := []byte("ORIGINAL-NORMAL-LINK-EXACT-BYTES")
	next := []byte("NEW-INERT-LINK-EXACT-BYTES")
	var err error
	owner.original, err = startupWriteFile(context.Background(), owner.files, menu, startupShortcutName, old, sd, true)
	if err != nil {
		t.Fatal(err)
	}
	// Release the inert creator before the publisher's first DELETE-owning open.
	if err := windows.CloseHandle(owner.original.handle); err != nil {
		t.Fatal(err)
	}
	owner.files.pins = owner.files.pins[:len(owner.files.pins)-1]
	owner.original = nil
	if err := startupCaptureOriginals(owner, menu); err != nil {
		t.Fatal(err)
	}
	owner.originalSHA = digest(old)
	originalID := owner.original.id
	legacy, err := startupWriteFile(context.Background(), owner.files, menu, "OpenCPN Legacy.lnk", []byte("LEGACY-UNCHANGED"), sd, false)
	if err != nil {
		t.Fatal(err)
	}
	safe, err := startupWriteFile(context.Background(), owner.files, menu, "Skager Safe Mode.lnk", []byte("SAFE-UNCHANGED"), sd, false)
	if err != nil {
		t.Fatal(err)
	}
	legacyID, safeID := legacy.id, safe.id
	if _, err := startupWriteFile(context.Background(), owner.files, recovery, "original.lnk.bytes", old, sd, false); err != nil {
		t.Fatal(err)
	}
	owner.link, err = startupWriteFile(context.Background(), owner.files, recovery, "pending.lnk", next, sd, true)
	if err != nil {
		t.Fatal(err)
	}
	owner.linkSHA = digest(next)
	pendingID := owner.link.id
	if err := startupPublishRenames(context.Background(), owner, menu, recovery); err != nil {
		t.Fatal(err)
	}
	for _, check := range []struct {
		name string
		pin  *enrollmentPin
		sha  string
	}{{"new-normal", owner.link, digest(next)}, {"original-backup", owner.original, digest(old)}, {"legacy", legacy, digest([]byte("LEGACY-UNCHANGED"))}, {"safe", safe, digest([]byte("SAFE-UNCHANGED"))}} {
		t.Run(check.name, func(t *testing.T) {
			if err := startupHashPin(context.Background(), check.pin, check.sha); err != nil {
				t.Fatal(err)
			}
		})
	}
	if owner.link.id != pendingID || owner.original.id != originalID || legacy.id != legacyID || safe.id != safeID {
		t.Fatal("actual original physical IDs changed")
	}
	// A successful explicit rename rebind must not relax later creation-time
	// validation on the ordinary retained pin.
	altered := *owner.link
	altered.info.CreationTime.LowDateTime ^= 1
	if err := altered.check(); err == nil {
		t.Fatal("ordinary native pin accepted changed creation time")
	}
	if _, err := os.Stat(filepath.Join(recovery.dos, "pending.lnk")); !os.IsNotExist(err) {
		t.Fatal("staging link still exists", err)
	}
	if _, err := owner.files.newDirectory(owner.files.pins[0], "recovery", sd, owner.clientSID); err == nil {
		t.Fatal("existing recovery adopted")
	}
}

func TestNativeStartupRenameRebindingRefusesSubstitution(t *testing.T) {
	owner, menu, recovery, sd := startupNativeFixture(t)
	raw := []byte("SAME-INERT-BYTES-DIFFERENT-NATIVE-FILE")
	pending, err := startupWriteFile(context.Background(), owner.files, recovery, "pending.lnk", raw, sd, true)
	if err != nil {
		t.Fatal(err)
	}
	foreign, err := startupWriteFile(context.Background(), owner.files, recovery, "foreign.lnk", raw, sd, true)
	if err != nil {
		t.Fatal(err)
	}
	physical := menu.physical + `\` + startupShortcutName
	dos := filepath.Join(menu.dos, startupShortcutName)
	source, _ := windows.UTF16PtrFromString(enrollmentPhysical(foreign.physical))
	destination, _ := windows.UTF16PtrFromString(enrollmentPhysical(physical))
	if err := windows.MoveFileEx(source, destination, windows.MOVEFILE_WRITE_THROUGH); err != nil {
		t.Fatal(err)
	}
	before := *pending
	if err := startupRebindRenamedPin(context.Background(), pending, physical, dos, digest(raw)); err == nil {
		t.Fatal("same-byte substituted destination adopted")
	}
	if pending.physical != before.physical || pending.dos != before.dos || pending.info != before.info || pending.id != before.id || pending.handle != before.handle {
		t.Fatal("failed rename rebind mutated original custody")
	}
	if err := startupHashPin(context.Background(), pending, digest(raw)); err != nil {
		t.Fatal("original pending file lost", err)
	}
	// A foreign retained handle cannot supply the original pin's identity either.
	forged := before
	forged.handle = foreign.handle
	if err := startupRebindRenamedPin(context.Background(), &forged, physical, dos, digest(raw)); err == nil {
		t.Fatal("substituted retained handle adopted")
	}
}

func TestNativeStartupPublicationRefusesUnknownDestinationAndCancellation(t *testing.T) {
	for _, mode := range []string{"unknown-destination", "canceled"} {
		t.Run(mode, func(t *testing.T) {
			owner, menu, recovery, sd := startupNativeFixture(t)
			old := []byte("UNKNOWN-ORIGINAL-ENTRY")
			next := []byte("NEW-INERT-LINK")
			var err error
			owner.link, err = startupWriteFile(context.Background(), owner.files, recovery, "pending.lnk", next, sd, true)
			if err != nil {
				t.Fatal(err)
			}
			existing, err := startupWriteFile(context.Background(), owner.files, menu, startupShortcutName, old, sd, true)
			if err != nil {
				t.Fatal(err)
			}
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			if mode == "canceled" {
				owner.original = existing
				owner.originalSHA = digest(old)
				cancel()
			}
			if err := startupPublishRenames(ctx, owner, menu, recovery); err == nil {
				t.Fatal("unexpected publication success")
			}
			raw, err := os.ReadFile(filepath.Join(menu.dos, startupShortcutName))
			if err != nil || !bytes.Equal(raw, old) {
				t.Fatalf("original entry changed %q %v", raw, err)
			}
			if err := startupHashPin(context.Background(), owner.link, digest(next)); err != nil {
				t.Fatal("pending bytes not retained", err)
			}
		})
	}
}

func startupThreeLinkFixture(t *testing.T, original ...[]byte) (*nativeStartupPublication, *enrollmentPin, *enrollmentPin, *windows.SECURITY_DESCRIPTOR) {
	t.Helper()
	owner, menu, recovery, sd := startupNativeFixture(t)
	ctx := context.Background()
	var err error
	owner.additional = []*nativeStartupPublication{
		{files: owner.files, clientSID: owner.clientSID, mode: startupLegacy},
		{files: owner.files, clientSID: owner.clientSID, mode: startupSafe},
	}
	originals := [3][]byte{[]byte("ORIGINAL-NORMAL")}
	if len(original) > 3 {
		t.Fatal("too many fixture originals")
	}
	copy(originals[:], original)
	for i, raw := range originals {
		if raw == nil {
			continue
		}
		spec, _ := startupShortcutMode(i).spec()
		created, err := startupWriteFile(ctx, owner.files, menu, spec.name, raw, sd, true)
		if err != nil {
			t.Fatal(err)
		}
		// End the inert Setup writer's custody before the enrollment publisher
		// captures its own original. A second stale path pin must not survive a
		// later publisher rename and masquerade as an enrollment owner.
		if owner.files.pins[len(owner.files.pins)-1] != created {
			t.Fatal("fixture writer pin differs")
		}
		if err := windows.CloseHandle(created.handle); err != nil {
			t.Fatal(err)
		}
		owner.files.pins = owner.files.pins[:len(owner.files.pins)-1]
	}
	// Exercise the same fixed-name capture and pre-mutation evidence producer.
	if err := startupCaptureOriginals(owner, menu); err != nil {
		t.Fatal(err)
	}
	for _, child := range append([]*nativeStartupPublication{owner}, owner.additional...) {
		if err := startupWriteOriginalEvidence(ctx, child, recovery, sd); err != nil {
			t.Fatal(err)
		}
	}
	for _, child := range append([]*nativeStartupPublication{owner}, owner.additional...) {
		spec, err := child.mode.spec()
		if err != nil {
			t.Fatal(err)
		}
		raw := []byte("INERT-LINK-" + spec.name)
		child.link, err = startupWriteFile(ctx, owner.files, recovery, spec.pending, raw, sd, true)
		if err != nil {
			t.Fatal(err)
		}
		child.linkSHA = digest(raw)
	}
	manifest, err := startupPublicationManifest(owner)
	if err != nil {
		t.Fatal(err)
	}
	var evidence struct {
		Schema int
		Links  []struct {
			Name     string
			Pending  string
			Original *struct {
				FileID   enrollmentFileID
				Creation windows.Filetime
				SHA256   string
			}
			New struct {
				FileID   enrollmentFileID
				Creation windows.Filetime
				SHA256   string
			}
		}
	}
	if err := json.Unmarshal(manifest, &evidence); err != nil {
		t.Fatal(err)
	}
	if evidence.Schema != 1 || len(evidence.Links) != 3 {
		t.Fatal("missing three-link recovery evidence")
	}
	for i, child := range append([]*nativeStartupPublication{owner}, owner.additional...) {
		if evidence.Links[i].New.FileID != child.link.id || evidence.Links[i].New.Creation != child.link.info.CreationTime || evidence.Links[i].New.SHA256 != child.linkSHA {
			t.Fatal("new identity/hash evidence differs")
		}
		if child.original != nil {
			if evidence.Links[i].Original == nil || evidence.Links[i].Original.FileID != child.original.id || evidence.Links[i].Original.Creation != child.original.info.CreationTime || evidence.Links[i].Original.SHA256 != child.originalSHA {
				t.Fatal("original evidence lost")
			}
		} else if evidence.Links[i].Original != nil {
			t.Fatal("absent original invented")
		}
	}
	if _, err := startupWriteFile(ctx, owner.files, recovery, "publication.json", manifest, sd, false); err != nil {
		t.Fatal(err)
	}
	return owner, menu, recovery, sd
}
func TestNativeStartupPublishesAllThreeRetainedIdentities(t *testing.T) {
	owner, menu, recovery, _ := startupThreeLinkFixture(t)
	children := append([]*nativeStartupPublication{owner}, owner.additional...)
	ids := []enrollmentFileID{children[0].link.id, children[1].link.id, children[2].link.id}
	originalID := owner.original.id
	if err := startupPublishAllRenames(context.Background(), owner, menu, recovery); err != nil {
		t.Fatal(err)
	}
	if err := owner.Check(context.Background()); err != nil {
		t.Fatal(err)
	}
	for i, child := range children {
		spec, _ := child.mode.spec()
		if child.link.id != ids[i] || child.link.physical != menu.physical+`\`+spec.name {
			t.Fatal("published native identity lost")
		}
		if err := startupHashPin(context.Background(), child.link, child.linkSHA); err != nil {
			t.Fatal(err)
		}
		if _, err := os.Stat(filepath.Join(recovery.dos, spec.pending)); !os.IsNotExist(err) {
			t.Fatal("pending still present", err)
		}
	}
	if owner.original.id != originalID || owner.original.physical != recovery.physical+`\Skager.lnk` {
		t.Fatal("normal original identity lost")
	}
}
func TestNativeStartupAllPreflightRefusesExistingModeTargets(t *testing.T) {
	for _, name := range []string{"Skager Legacy.lnk", "Skager Safe Mode.lnk"} {
		t.Run(name, func(t *testing.T) {
			owner, menu, recovery, sd := startupThreeLinkFixture(t)
			foreign, err := startupWriteFile(context.Background(), owner.files, menu, name, []byte("OLD-LOCALAPPDATA-LINK"), sd, true)
			if err != nil {
				t.Fatal(err)
			}
			originalID := owner.original.id
			if err := startupPublishAllRenames(context.Background(), owner, menu, recovery); err == nil {
				t.Fatal("existing mode target adopted")
			}
			if owner.original.id != originalID || owner.original.physical != menu.physical+`\Skager.lnk` {
				t.Fatal("normal moved before all targets passed")
			}
			if err := startupHashPin(context.Background(), foreign, digest([]byte("OLD-LOCALAPPDATA-LINK"))); err != nil {
				t.Fatal(err)
			}
			for _, child := range append([]*nativeStartupPublication{owner}, owner.additional...) {
				spec, _ := child.mode.spec()
				if child.link.physical != recovery.physical+`\`+spec.pending {
					t.Fatal("pending link moved")
				}
				if err := startupHashPin(context.Background(), child.link, child.linkSHA); err != nil {
					t.Fatal(err)
				}
			}
		})
	}
}
func TestNativeStartupPartialPublicationRetainsIndependentEvidence(t *testing.T) {
	owner, menu, recovery, sd := startupThreeLinkFixture(t)
	children := append([]*nativeStartupPublication{owner}, owner.additional...)
	var foreign *enrollmentPin
	err := startupPublishSequence(context.Background(), func(mode startupShortcutMode) error { return startupPreflightDestination(children[int(mode)], menu) }, func(mode startupShortcutMode) error {
		if mode == startupLegacy {
			var err error
			foreign, err = startupWriteFile(context.Background(), owner.files, menu, "Skager Legacy.lnk", []byte("LATE-UNKNOWN-DESTINATION"), sd, true)
			if err != nil {
				return err
			}
		}
		if err := startupPreflightDestination(children[int(mode)], menu); err != nil {
			return err
		}
		return startupPublishRenames(context.Background(), children[int(mode)], menu, recovery)
	})
	if !errors.Is(err, ErrStartupUncertain) {
		t.Fatalf("partial publication not uncertain: %v", err)
	}
	if owner.link.physical != menu.physical+`\Skager.lnk` || owner.original.physical != recovery.physical+`\Skager.lnk` {
		t.Fatal("normal publication/backup evidence lost")
	}
	for _, child := range children {
		if err := startupHashPin(context.Background(), child.link, child.linkSHA); err != nil {
			t.Fatal(err)
		}
	}
	for _, child := range owner.additional {
		spec, _ := child.mode.spec()
		if child.link.physical != recovery.physical+`\`+spec.pending {
			t.Fatal("later pending moved after refusal")
		}
	}
	if foreign == nil {
		t.Fatal("late collision not exercised")
	}
	if err := startupHashPin(context.Background(), foreign, digest([]byte("LATE-UNKNOWN-DESTINATION"))); err != nil {
		t.Fatal("foreign destination overwritten", err)
	}
	if raw, err := os.ReadFile(filepath.Join(recovery.dos, "original.lnk.bytes")); err != nil || !bytes.Equal(raw, []byte("ORIGINAL-NORMAL")) {
		t.Fatal("original exact byte evidence lost", err)
	}
	if _, err := os.Stat(filepath.Join(recovery.dos, "publication.json")); err != nil {
		t.Fatal("manifest lost", err)
	}
}

func TestNativeStartupModePreflightRefusesAmbiguousEntries(t *testing.T) {
	for _, kind := range []string{"empty-file", "directory", "reparse"} {
		t.Run(kind, func(t *testing.T) {
			owner, menu, _, _ := startupNativeFixture(t)
			child := &nativeStartupPublication{files: owner.files, clientSID: owner.clientSID, mode: startupSafe}
			path := filepath.Join(menu.dos, "Skager Safe Mode.lnk")
			switch kind {
			case "empty-file":
				if err := os.WriteFile(path, nil, 0600); err != nil {
					t.Fatal(err)
				}
			case "directory":
				if err := os.Mkdir(path, 0700); err != nil {
					t.Fatal(err)
				}
			case "reparse":
				target := filepath.Join(menu.dos, "inert-target")
				if err := os.WriteFile(target, []byte("INERT"), 0600); err != nil {
					t.Fatal(err)
				}
				p, _ := windows.UTF16PtrFromString(path)
				q, _ := windows.UTF16PtrFromString(target)
				if err := windows.CreateSymbolicLink(p, q, 0); err != nil {
					t.Fatal("elevated inert reparse fixture unavailable", err)
				}
			}
			if err := startupPreflightDestination(child, menu); err == nil {
				t.Fatal("ambiguous mode target accepted")
			}
			if err := startupCaptureOriginals(child, menu); err == nil {
				t.Fatal("ambiguous mode target captured as rollback original")
			}
			if child.original != nil {
				t.Fatal("ambiguous mode target adopted")
			}
		})
	}
}

func TestNativeStartupOriginalCustodyExcludesLateSubstitution(t *testing.T) {
	for mode := startupNormal; mode <= startupSafe; mode++ {
		spec, _ := mode.spec()
		t.Run(spec.name, func(t *testing.T) {
			owner, menu, recovery, sd := startupThreeLinkFixture(t, []byte("ORIGINAL-NORMAL"), []byte("ORIGINAL-LEGACY"), []byte("ORIGINAL-SAFE"))
			children := append([]*nativeStartupPublication{owner}, owner.additional...)
			child := children[int(mode)]
			originalHandle, originalID := child.original.handle, child.original.id
			foreignRaw := []byte("FOREIGN-MUST-STAY-OUTSIDE-RECOVERY")
			foreign, err := startupWriteFile(context.Background(), owner.files, menu, "foreign.lnk", foreignRaw, sd, true)
			if err != nil {
				t.Fatal(err)
			}
			source, _ := windows.UTF16PtrFromString(enrollmentPhysical(child.original.physical))
			away, _ := windows.UTF16PtrFromString(enrollmentPhysical(menu.physical + `\displaced.lnk`))
			replacement, _ := windows.UTF16PtrFromString(enrollmentPhysical(foreign.physical))
			attempted := false
			// Exercise substitution precisely after all role preflights and before
			// this role's move, the former path-based rename race window.
			err = startupPublishSequence(context.Background(), func(m startupShortcutMode) error {
				return startupPreflightDestination(children[int(m)], menu)
			}, func(m startupShortcutMode) error {
				if m == mode {
					attempted = true
					if err := windows.MoveFileEx(source, away, windows.MOVEFILE_WRITE_THROUGH); !errors.Is(err, windows.ERROR_SHARING_VIOLATION) {
						return errors.New("original rename exclusion missing")
					}
					if err := windows.MoveFileEx(replacement, source, windows.MOVEFILE_REPLACE_EXISTING|windows.MOVEFILE_WRITE_THROUGH); !errors.Is(err, windows.ERROR_SHARING_VIOLATION) {
						return errors.New("foreign replacement exclusion missing")
					}
				}
				return startupPublishRenames(context.Background(), children[int(m)], menu, recovery)
			})
			if err != nil || !attempted {
				t.Fatalf("publication after excluded substitution: %v", err)
			}
			if child.original.handle != originalHandle || child.original.id != originalID {
				t.Fatal("original owner was reopened or substituted")
			}
			if err := startupCheckOriginalLocation(child, recovery, spec.name); err != nil {
				t.Fatal(err)
			}
			if err := startupHashPin(context.Background(), child.original, child.originalSHA); err != nil {
				t.Fatal("original backup changed", err)
			}
			if err := startupHashPin(context.Background(), foreign, digest(foreignRaw)); err != nil || foreign.physical != menu.physical+`\foreign.lnk` {
				t.Fatal("foreign file moved or changed", err)
			}
			if err := owner.Check(context.Background()); err != nil {
				t.Fatal(err)
			}
		})
	}
}

func TestNativeStartupOriginalCaptureRefusesExistingDeleteOwner(t *testing.T) {
	for mode := startupNormal; mode <= startupSafe; mode++ {
		spec, _ := mode.spec()
		t.Run(spec.name, func(t *testing.T) {
			owner, menu, recovery, sd := startupNativeFixture(t)
			owner.mode = mode
			raw := []byte("ORIGINAL-WITH-FOREIGN-DELETE-OWNER")
			created, err := startupWriteFile(context.Background(), owner.files, menu, spec.name, raw, sd, true)
			if err != nil {
				t.Fatal(err)
			}
			if err := windows.CloseHandle(created.handle); err != nil {
				t.Fatal(err)
			}
			owner.files.pins = owner.files.pins[:len(owner.files.pins)-1]
			path, _ := windows.UTF16PtrFromString(enrollmentPhysical(created.physical))
			foreign, err := windows.CreateFile(path, windows.GENERIC_READ|windows.DELETE, windows.FILE_SHARE_READ|windows.FILE_SHARE_DELETE, nil, windows.OPEN_EXISTING, windows.FILE_FLAG_OPEN_REPARSE_POINT, 0)
			if err != nil {
				t.Fatal(err)
			}
			t.Cleanup(func() {
				if err := windows.CloseHandle(foreign); err != nil {
					t.Errorf("foreign fixture DELETE owner close: %v", err)
				}
			})
			if err := startupCaptureOriginals(owner, menu); !errors.Is(err, windows.ERROR_SHARING_VIOLATION) {
				t.Fatalf("foreign DELETE owner not refused: %v", err)
			}
			if owner.original != nil || owner.originalFile != nil {
				t.Fatal("unexcluded original adopted")
			}
			_, id, err := enrollmentInfo(foreign)
			if err != nil || id != created.id {
				t.Fatal("original changed during refusal", err)
			}
			observed := *created
			observed.handle, observed.dos = foreign, ""
			if err := startupHashPin(context.Background(), &observed, digest(raw)); err != nil {
				t.Fatal("refusal changed original bytes", err)
			}
			if _, err := os.Stat(filepath.Join(recovery.dos, spec.name)); !os.IsNotExist(err) {
				t.Fatal("refusal moved an original", err)
			}
		})
	}
}
