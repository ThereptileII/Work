//go:build windows

package brokerenroll

import (
	"bytes"
	"path/filepath"
	"strings"
	"testing"
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
	for _, tc := range []struct{ target, arguments, working string }{
		{`C:\SKAGER-Inert-Shortcut-Reader\` + strings.Repeat(`Long-Path-Component\`, 14) + `opencpn.exe`, "", `C:\SKAGER-Inert-Shortcut-Reader`},
		{`C:\SKAGER-Inert-Shortcut-Reader\opencpn.exe`, strings.Repeat("a", 32769), `C:\SKAGER-Inert-Shortcut-Reader`},
	} {
		raw, err := startupLinkBytes(tc.target, tc.arguments, tc.working)
		if err != nil {
			t.Fatal(err)
		}
		if target, arguments, working, err := startupLinkFields(raw); err == nil || target != "" || arguments != "" || working != "" {
			t.Fatal("truncated native field accepted", err)
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
