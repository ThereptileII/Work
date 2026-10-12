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

func TestNativeBrokerExecutionExactQualifiedEnvironmentKeepsOrdinaryStripping(t *testing.T) {
	if unsafe.Sizeof(brokerExecutionTokenStatistics{}) != 56 || unsafe.Offsetof(brokerExecutionTokenStatistics{}.AuthenticationID) != 8 || unsafe.Offsetof(brokerExecutionTokenStatistics{}.ModifiedID) != 48 {
		t.Fatal("native token layout")
	}
	a, _ := brokerExecutionFixture()
	c := brokerExecutionCommand(a, brokerpipe.SpawnCandidate)
	original := []string{"PATH=C:\\Windows", "KEEP=ok", "SKAGER_UPDATE_CHALLENGE=untrusted", "sKaGeR_UpDaTe_QUALIFY_PROFILE=0", "SKAGER_UPDATE_OTHER=untrusted", "=C:=C:\\owned"}
	block, err := brokerExecutionEnvironment(original, c)
	if err != nil {
		t.Fatal(err)
	}
	raw := string(utf16.Decode(block))
	if !strings.HasSuffix(raw, "\x00\x00") {
		t.Fatal("unterminated native environment")
	}
	fields := map[string]string{}
	for _, entry := range strings.Split(strings.TrimSuffix(raw, "\x00\x00"), "\x00") {
		key, value, _ := strings.Cut(entry, "=")
		if strings.HasPrefix(strings.ToUpper(key), "SKAGER_UPDATE_") {
			if fields[key] != "" {
				t.Fatal("duplicate generated field")
			}
			fields[key] = value
		}
	}
	if len(fields) != 4 || fields["SKAGER_UPDATE_PIPE"] != c.HealthPipe || fields["SKAGER_UPDATE_GENERATION"] != c.ApplicationGeneration || fields["SKAGER_UPDATE_CHALLENGE"] != c.HealthChallenge || fields["SKAGER_UPDATE_QUALIFY_PROFILE"] != "1" || strings.Contains(raw, "untrusted") {
		t.Fatal("leaked caller health environment", fields)
	}
	ordinary, err := ordinaryEnvironment(original)
	if err != nil || strings.Contains(strings.ToUpper(string(utf16.Decode(ordinary))), "SKAGER_UPDATE_") {
		t.Fatal("ordinary creation gained qualification", err)
	}
	if _, err = brokerExecutionEnvironment([]string{"BAD=embedded\x00value"}, c); err == nil {
		t.Fatal("invalid base environment")
	}
	if _, err = brokerExecutionEnvironment([]string{"BIG=" + strings.Repeat("x", 32767)}, c); err == nil {
		t.Fatal("oversized native environment")
	}
}
func TestNativeBrokerExecutionOldArmedRootIsOnlyExactDenial(t *testing.T) {
	for _, mode := range []string{"absent-root", "existing-unarmed", "garbage-marker", "empty-marker", "file-root", "missing-parent"} {
		t.Run(mode, func(t *testing.T) {
			directory := t.TempDir()
			parent, err := openNativeArmedDirectory(directory)
			if err != nil {
				t.Fatal(err)
			}
			defer windows.CloseHandle(parent.handle)
			old := filepath.Join(directory, "OpenNavXAlpha1")
			if mode != "absent-root" && mode != "missing-parent" && mode != "file-root" {
				if err = os.Mkdir(old, 0700); err != nil {
					t.Fatal(err)
				}
			}
			if mode == "file-root" {
				if err = os.WriteFile(old, []byte("not a directory"), 0600); err != nil {
					t.Fatal(err)
				}
			}
			if mode == "garbage-marker" || mode == "empty-marker" {
				raw := []byte("not authority")
				if mode == "empty-marker" {
					raw = nil
				}
				if err = os.WriteFile(filepath.Join(old, armedUpdateMarker), raw, 0600); err != nil {
					t.Fatal(err)
				}
			}
			physical := parent.name
			if mode == "missing-parent" {
				physical += `\missing-parent`
			}
			err = brokerExecutionOldMarkerAbsent(physical)
			if mode == "absent-root" || mode == "existing-unarmed" {
				if err != nil {
					t.Fatal("exact absence refused", err)
				}
			} else {
				if !errors.Is(err, errArmedUpdateBarrier) {
					t.Fatal("armed or uncertain root accepted", err)
				}
			}
		})
	}
}

// The helper creates a copy of this test PE suspended, never resumes it and uses
// only original creator handles to abort/drain. It does not exercise an actual
// broker grant, interactive token, startup-health producer or product launch.
func TestNativeBrokerExecutionSuspendedCreatorRetainsOriginalImage(t *testing.T) {
	spec, receipt := ordinaryNativeFixture(t)
	acquired, err := acquireNativeOrdinaryImage(spec)
	if err != nil {
		t.Fatal(err)
	}
	image := acquired.(*nativeOrdinaryImage)
	defer image.Close()
	a, _ := brokerExecutionFixture()
	child, err := image.createBrokerExecution(brokerExecutionCommand(a, brokerpipe.SpawnCandidate), nil)
	if err != nil {
		t.Fatal(err)
	}
	defer child.Close()
	if err = child.corroborate(); err != nil {
		_ = child.abortSuspended()
		t.Fatal(err)
	}
	defer func() {
		if err := child.abortSuspended(); err != nil {
			t.Error(err)
		}
	}()
	if child.pid == 0 || filetimeValue(child.creation) == 0 || child.resumeAttempted {
		t.Fatal("missing original suspended creator identity")
	}
	if err = image.recheck(); err != nil {
		t.Fatal(err)
	}
	if _, err = os.Stat(receipt); !errors.Is(err, os.ErrNotExist) {
		t.Fatal("fixture entrypoint unexpectedly executed", err)
	}
}
