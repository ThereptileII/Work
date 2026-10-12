package launcher

import (
	"errors"
	"os"
	"path/filepath"
	"strings"
	"testing"
	"unicode/utf16"
	"unsafe"

	"example.com/opennav-update-verifier/brokerpipe"
	"golang.org/x/sys/windows"
)

// Observe the actual suspended original process, independently of the creator's
// argument builder. The inert copied test PE never runs or contacts a service.
func brokerModeNativeCommandLine(t *testing.T, child *nativeOrdinaryChild) []string {
	t.Helper()
	storage := make([]uintptr, (128<<10)/int(unsafe.Sizeof(uintptr(0))))
	var used uint32
	if err := windows.NtQueryInformationProcess(child.process, windows.ProcessCommandLineInformation, unsafe.Pointer(&storage[0]), uint32(len(storage))*uint32(unsafe.Sizeof(uintptr(0))), &used); err != nil {
		t.Fatal(err)
	}
	text := (*windows.NTUnicodeString)(unsafe.Pointer(&storage[0]))
	base := uintptr(unsafe.Pointer(&storage[0]))
	pointer := uintptr(unsafe.Pointer(text.Buffer))
	header := unsafe.Sizeof(*text)
	if used < uint32(header) || used > 128<<10 || text.Length == 0 || text.Length%2 != 0 || pointer < base+header || pointer > base+uintptr(used) || uintptr(text.Length) > base+uintptr(used)-pointer {
		t.Fatal("unbounded native command line")
	}
	args, err := windows.DecomposeCommandLine(text.String())
	if err != nil {
		t.Fatal(err)
	}
	return args
}
func TestNativeBrokerExecutionClosedModeArgumentsAndEnvironment(t *testing.T) {
	for _, mode := range []brokerpipe.StartupMode{brokerpipe.StartupXNav, brokerpipe.StartupLegacy, brokerpipe.StartupSafe} {
		t.Run("current/"+string(mode), func(t *testing.T) {
			a, _ := brokerExecutionFixture()
			c := brokerExecutionCommand(a, brokerpipe.SpawnCurrent)
			c.StartupMode = mode
			brokerExecutionSuspendedCreator(t, func(image *nativeOrdinaryImage) (*nativeOrdinaryChild, error) {
				child, err := image.createBrokerCurrent(c, nil)
				if err != nil {
					return child, err
				}
				kept := false
				defer func() {
					if !kept {
						if err := child.abortSuspended(); err != nil {
							t.Error(err)
						}
						if err := child.Close(); err != nil {
							t.Error(err)
						}
					}
				}()
				args := brokerModeNativeCommandLine(t, child)
				expected := "--xnav"
				if mode == brokerpipe.StartupLegacy {
					expected = "--legacy"
				}
				if mode == brokerpipe.StartupSafe {
					expected = "--safe-mode"
				}
				if len(args) != 2 || args[1] != expected {
					t.Fatalf("actual arguments=%q want=%s", args, expected)
				}
				kept = true
				return child, nil
			})
		})
		t.Run("relay/"+string(mode), func(t *testing.T) {
			spec, receipt := ordinaryNativeFixture(t)
			relay := filepath.Join(spec.directory, "skager-start.exe")
			if err := os.Rename(spec.executable, relay); err != nil {
				t.Fatal(err)
			}
			spec.executable = relay
			acquired, err := acquireNativeRelayImage(spec)
			if err != nil {
				t.Fatal(err)
			}
			image := acquired.(*nativeOrdinaryImage)
			defer image.Close()
			a, _ := brokerExecutionFixture()
			c := brokerExecutionCommand(a, brokerpipe.RelaySelectedExecutor)
			c.StartupMode = mode
			child, err := image.createBrokerRelay(c, nil)
			if err != nil {
				t.Fatal(err)
			}
			defer child.Close()
			defer func() {
				if err := child.abortSuspended(); err != nil {
					t.Error(err)
				}
			}()
			if err := child.corroborate(); err != nil {
				t.Fatal(err)
			}
			args := brokerModeNativeCommandLine(t, child)
			expected := "--broker-current=" + c.ApplicationGeneration
			if mode != brokerpipe.StartupXNav {
				expected += ":" + string(mode)
			}
			if len(args) != 2 || args[1] != expected {
				t.Fatalf("actual relay arguments=%q want=%s", args, expected)
			}
			if _, err := os.Stat(receipt); !errors.Is(err, os.ErrNotExist) {
				t.Fatal("inert fixture ran", err)
			}
		})
		a, _ := brokerExecutionFixture()
		c := brokerExecutionCommand(a, brokerpipe.SpawnCurrent)
		c.StartupMode = mode
		block, err := brokerCurrentEnvironment([]string{"KEEP=ordinary", "SKAGER_UPDATE_QUALIFY_PROFILE=1"}, c)
		if err != nil || strings.Contains(strings.ToUpper(string(utf16.Decode(block))), "SKAGER_UPDATE_") {
			t.Fatal("normal mode retained qualification", err)
		}
	}
	for _, mode := range []brokerpipe.StartupMode{brokerpipe.StartupLegacy, brokerpipe.StartupSafe} {
		a, _ := brokerExecutionFixture()
		c := brokerExecutionCommand(a, brokerpipe.SpawnCandidate)
		c.StartupMode = mode
		if _, err := brokerExecutionEnvironment([]string{"KEEP=ordinary"}, c); err == nil {
			t.Fatal("qualification changed fixed XNav mode")
		}
		c = brokerExecutionCommand(a, brokerpipe.RelayStartupExecutor)
		c.StartupMode = mode
		if _, err := brokerRelayEnvironment([]string{"KEEP=ordinary"}, c); err == nil {
			t.Fatal("recovery mode entered update offer relay")
		}
	}
}
