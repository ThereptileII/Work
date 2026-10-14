//go:build windows

package brokerenroll

import (
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"os"
	"os/exec"
	"path/filepath"
	"testing"

	"golang.org/x/sys/windows"
)

// This group executes the real Python ReOpenFile adapter against THIS native Go
// test helper. It does not approve anything or touch SCM/product/profile state.
func TestNativeQualificationChallengePythonAdapterStreaming(t *testing.T) {
	fixture, parent := freshCacheNativeParent(t)
	if err := codeReleasePin(fixture, parent); err != nil {
		t.Fatal(err)
	}
	executable, err := os.Executable()
	if err != nil {
		t.Fatal(err)
	}
	raw, err := os.ReadFile(executable)
	if err != nil {
		t.Fatal(err)
	}
	sum := sha256.Sum256(raw)
	user, err := windows.GetCurrentProcessToken().GetTokenUser()
	if err != nil {
		t.Fatal(err)
	}
	directory, err := os.Getwd()
	if err != nil {
		t.Fatal(err)
	}
	script := filepath.Clean(filepath.Join(directory, "..", "..", "test-installed-updater-coordinator-native-windows.py"))
	python, err := exec.LookPath("python")
	if err != nil {
		t.Fatal("native Python adapter prerequisite unavailable", err)
	}
	evidence, err := os.MkdirTemp("", "SKAGER-native-adapter-evidence-")
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() {
		if !t.Failed() {
			if err := os.RemoveAll(evidence); err != nil {
				t.Error(err)
			}
		}
	})
	command := exec.Command(python, script, executable, hex.EncodeToString(sum[:]), parent.dos, evidence, user.User.Sid.String())
	// No CommandContext timeout kill: the adapter owns/drains its actual child.
	output, err := command.CombinedOutput()
	if err != nil {
		t.Fatalf("actual native adapter refused: %v; %s; retained evidence %s", err, output, evidence)
	}
	var receipt struct {
		Schema                  int    `json:"schema"`
		State                   string `json:"state"`
		ReadyEOFNoEffect        bool   `json:"readyEOFNoEffect"`
		ProtectedParentRetained bool   `json:"protectedParentRetained"`
		ExactConsumedBytes      bool   `json:"exactConsumedBytes"`
		RestartedReplayRefused  bool   `json:"restartedReplayRefused"`
		ProductInstallAttempted bool   `json:"productInstallAttempted"`
	}
	if err := json.Unmarshal(output, &receipt); err != nil || receipt.Schema != 1 || receipt.State != "native-adapter-streaming-passed" || !receipt.ReadyEOFNoEffect || !receipt.ProtectedParentRetained || !receipt.ExactConsumedBytes || !receipt.RestartedReplayRefused || receipt.ProductInstallAttempted {
		t.Fatalf("native adapter evidence incomplete: %v; %s", err, output)
	}
	t.Logf("actual Python/native READY, ReOpenFile streaming, deny-delete parent, exact durable consumption and restarted replay refusal passed: %s", output)
}
