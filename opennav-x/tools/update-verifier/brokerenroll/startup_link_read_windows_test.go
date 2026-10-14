//go:build windows

package brokerenroll

import (
	"bytes"
	"errors"
	"path/filepath"
	"runtime"
	"strings"
	"syscall"
	"testing"
	"unsafe"

	"golang.org/x/sys/windows"
)

// In-memory system ShellLink objects only. Targets are inert nonexistent paths;
// no SCM, installed product, real menu, profile or executable is opened/launched.
func TestNativeStartupLinkFieldsRoundTripStockAndIntegration(t *testing.T) {
	for _, tc := range []struct {
		name, target, arguments, working string
	}{
		{"stock", `C:\SKAGER-Inert-Shortcut-Reader\Stock\opencpn.exe`, "", `C:\SKAGER-Inert-Shortcut-Reader\Stock`},
		{"integration", `C:\SKAGER-Inert-Shortcut-Reader\generations\` + strings.Repeat("a", 32) + `\app\skager-start.exe`, "--xnav", `C:\SKAGER-Inert-Shortcut-Reader\generations\` + strings.Repeat("a", 32) + `\app`},
		{"protected", `C:\SKAGER-Inert-Shortcut-Reader\Protected\skager-start.exe`, "--broker-start=" + strings.Repeat("b", 32) + ":legacy", `C:\SKAGER-Inert-Shortcut-Reader\Protected`},
		{"unicode", `C:\SKAGER-Inert-Shortcut-Reader\Sjö\opencpn.exe`, `--name="Ö 🛥"`, `C:\SKAGER-Inert-Shortcut-Reader\Sjö`},
	} {
		t.Run(tc.name, func(t *testing.T) {
			raw, err := startupLinkBytes(tc.target, tc.arguments, tc.working)
			if err != nil {
				t.Fatal(err)
			}
			original := bytes.Clone(raw)
			target, arguments, working, err := startupLinkFields(raw)
			if err != nil || target != tc.target || arguments != tc.arguments || working != tc.working {
				t.Fatalf("original fields differ: target=%q arguments=%q working=%q error=%v", target, arguments, working, err)
			}
			if !bytes.Equal(raw, original) {
				t.Fatal("reader changed original shortcut bytes")
			}
		})
	}
}

func TestNativeStartupLinkFieldsRejectsMalformedAndOversized(t *testing.T) {
	raw, err := startupLinkBytes(`C:\SKAGER-Inert-Shortcut-Reader\Stock\opencpn.exe`, "", `C:\SKAGER-Inert-Shortcut-Reader\Stock`)
	if err != nil {
		t.Fatal(err)
	}
	badHeader, badClass := bytes.Clone(raw), bytes.Clone(raw)
	badHeader[0], badClass[4] = 0, 0xff
	for _, bad := range [][]byte{nil, {}, bytes.Repeat([]byte{0xff}, 76), raw[:75], raw[:20], raw[:76], badHeader, badClass,
		append(bytes.Clone(raw), []byte("ignored-tail")...), make([]byte, maxStartupShortcutBytes+1)} {
		target, arguments, working, err := startupLinkFields(bad)
		if err == nil || target != "" || arguments != "" || working != "" {
			t.Fatalf("malformed input returned fields: bytes=%d target=%q arguments=%q working=%q error=%v", len(bad), target, arguments, working, err)
		}
	}
}

func TestNativeStartupLinkFieldsRefusesTruncatedGetterResults(t *testing.T) {
	// Native run 38514939833 refused SetPath for an overlong fixture before
	// decoding. Use a known-valid original object and constrain each actual COM
	// getter's output buffer instead: truncation must refuse, never classify a
	// prefix as the original target/arguments/working directory.
	target := `C:\SKAGER-Inert-Shortcut-Reader\opencpn.exe`
	arguments, working := "--xnav --preserve", `C:\SKAGER-Inert-Shortcut-Reader`
	raw, err := startupLinkBytes(target, arguments, working)
	if err != nil {
		t.Fatal(err)
	}
	if gotTarget, gotArguments, gotWorking, err := startupLinkFields(raw); err != nil || gotTarget != target || gotArguments != arguments || gotWorking != working {
		t.Fatal("truncation fixture is not a valid original shortcut", err)
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
		t.Fatal("actual system ShellLink factory unavailable")
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
	writeErr := startupCOMCall(stream, 4, uintptr(unsafe.Pointer(&raw[0])), uintptr(len(raw)), uintptr(unsafe.Pointer(&used)))
	runtime.KeepAlive(raw)
	if writeErr != nil || used != uint32(len(raw)) {
		t.Fatal("actual original stream write failed", writeErr)
	}
	if err := startupStreamRewind(stream); err != nil {
		t.Fatal(err)
	}
	if err := startupCOMCall(persist, 5, uintptr(unsafe.Pointer(stream))); err != nil {
		t.Fatal(err)
	}
	for _, field := range []struct {
		method int
		want   string
	}{{3, target}, {10, arguments}, {8, working}} {
		// This fixture is ASCII, so byte and UTF16-unit lengths are equal.
		if value, err := startupReadLinkField(link, field.method, len(field.want)+2); err != nil || value != field.want {
			t.Fatalf("actual getter %d failed positive control: value=%q err=%v", field.method, value, err)
		}
		for _, limit := range []int{1, 8, len(field.want) + 1} {
			if value, err := startupReadLinkField(link, field.method, limit); err == nil || value != "" {
				t.Fatalf("actual getter %d accepted truncated/full buffer %d: value=%q err=%v", field.method, limit, value, err)
			}
		}
	}
}

func TestNativeStartupLinkFieldsPreservesExactSelectionFields(t *testing.T) {
	target := `C:\SKAGER-Inert-Shortcut-Reader\generations\` + strings.Repeat("c", 32) + `\app\opencpn.exe`
	for _, arguments := range []string{"--xnav", "--legacy", "--safe-mode", "--xnav --unknown", ""} {
		working := filepath.Dir(target)
		raw, err := startupLinkBytes(target, arguments, working)
		if err != nil {
			t.Fatal(err)
		}
		actualTarget, actualArguments, actualWorking, err := startupLinkFields(raw)
		if err != nil || actualTarget != target || actualArguments != arguments || actualWorking != working {
			t.Fatal("reader normalized a potentially user-modified selection", err)
		}
	}
}

func TestNativeStartupLinkFieldsUTF16BoundAndTermination(t *testing.T) {
	for _, raw := range [][]uint16{nil, {'a'}, {'a', 0}, {0xffff, 0xffff}, {0xd800, 0, 0}, {0xdc00, 0, 0}, {0xd800, 'a', 0, 0}} {
		if value, err := startupLinkUTF16(raw); err == nil || value != "" {
			t.Fatal("malformed or possibly truncated UTF16 accepted", raw, value, err)
		}
	}
	for _, tc := range []struct {
		raw  []uint16
		want string
	}{
		{[]uint16{0, 0xffff}, ""}, {[]uint16{'Ö', 0, 0xffff}, "Ö"}, {[]uint16{0xd83d, 0xdee5, 0, 0xffff}, "🛥"},
	} {
		if value, err := startupLinkUTF16(tc.raw); err != nil || value != tc.want {
			t.Fatal("valid bounded UTF16 refused", value, tc.want, err)
		}
	}
}
