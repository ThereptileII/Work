package launcher

import (
	"errors"
	"os"
	"path/filepath"
	"strings"
	"testing"
	"unicode/utf16"

	"example.com/opennav-update-verifier/brokerpipe"
)

// An inert copy of the test PE is created suspended and never resumed. This is
// not an actual broker grant, selected image proof or successful relay execution.
func TestNativeBrokerExecutionRelaySuspendedCreatorRetainsOriginalLauncher(t *testing.T) {
	spec, receipt := ordinaryNativeFixture(t)
	for _, name := range []string{"other.exe", "skager-start.exe.more", "skager-start.exe/other.exe"} {
		bad := spec
		bad.executable = filepath.Join(spec.directory, name)
		if _, err := acquireNativeRelayImage(bad); err == nil {
			t.Fatal("relay accepted arbitrary executable", name)
		}
	}
	if _, err := acquireNativeRelayImage(spec); err == nil {
		t.Fatal("relay accepted ordinary executable")
	}
	relay := filepath.Join(spec.directory, "skager-start.exe")
	if err := os.Rename(spec.executable, relay); err != nil {
		t.Fatal(err)
	}
	spec.executable = relay
	if _, err := acquireNativeOrdinaryImage(spec); err == nil {
		t.Fatal("ordinary factory accepted relay executable")
	}
	acquired, err := acquireNativeRelayImage(spec)
	if err != nil {
		t.Fatal(err)
	}
	image := acquired.(*nativeOrdinaryImage)
	defer image.Close()
	a, _ := brokerExecutionFixture()
	c := brokerExecutionCommand(a, brokerpipe.RelaySelectedExecutor)
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
	if err = child.corroborate(); err != nil {
		t.Fatal(err)
	}
	if child.pid == 0 || filetimeValue(child.creation) == 0 || child.resumeAttempted {
		t.Fatal("original suspended identity unavailable")
	}
	if err = image.recheck(); err != nil {
		t.Fatal(err)
	}
	if _, err = os.Stat(receipt); !errors.Is(err, os.ErrNotExist) {
		t.Fatal("inert child executed", err)
	}
}
func TestNativeBrokerExecutionRelayEnvironmentHasNoQualification(t *testing.T) {
	a, _ := brokerExecutionFixture()
	c := brokerExecutionCommand(a, brokerpipe.RelaySelectedExecutor)
	values := []string{"PATH=C:\\Windows", "KEEP=ordinary", "SKAGER_UPDATE_QUALIFY_PROFILE=1", "sKaGeR_UpDaTe_PIPE=untrusted"}
	block, err := brokerRelayEnvironment(values, c)
	if err != nil || strings.Contains(strings.ToUpper(string(utf16.Decode(block))), "SKAGER_UPDATE_") || !strings.Contains(string(utf16.Decode(block)), "KEEP=ordinary") {
		t.Fatal("relay inherited qualification or lost normal environment", err)
	}
	c.HealthChallenge = strings.Repeat("1", 64)
	if _, err = brokerRelayEnvironment(values, c); err == nil {
		t.Fatal("relay accepted health payload")
	}
}
