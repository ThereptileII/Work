package launcher

import (
	"errors"
	"example.com/opennav-update-verifier/brokerpipe"
	"os"
	"testing"
)

// Real original process/thread handles are retained through exit. The copied
// test executable is never resumed; only its corroborated suspended creator
// aborts it. This does not claim WM_CLOSE or service execution integration.
func TestNativeBrokerExecutionOriginalExitRequiresRetainedCreator(t *testing.T) {
	spec, receipt := ordinaryNativeFixture(t)
	acquired, err := acquireNativeOrdinaryImage(spec)
	if err != nil {
		t.Fatal(err)
	}
	image := acquired.(*nativeOrdinaryImage)
	defer image.Close()
	a, _ := brokerExecutionFixture()
	child, err := image.createBrokerCurrent(brokerExecutionCommand(a, brokerpipe.SpawnCurrent), nil)
	if err != nil {
		t.Fatal(err)
	}
	defer child.Close()
	if err = child.corroborate(); err != nil {
		t.Fatal(err)
	}
	aborted := false
	defer func() {
		if !aborted {
			if err := child.abortSuspended(); err != nil {
				t.Error(err)
			}
		}
	}()
	created := filetimeValue(child.creation)
	if exited, err := brokerExecutionOriginalExited(child, created); err != nil || exited {
		t.Fatal("live suspended creator claimed closed", exited, err)
	}
	if _, err := brokerExecutionOriginalExited(child, created+1); err == nil {
		t.Fatal("wrong original creation accepted")
	}
	if _, err := brokerExecutionOriginalExited(nil, created); err == nil {
		t.Fatal("missing creator accepted")
	}
	if err = child.abortSuspended(); err != nil {
		t.Fatal(err)
	}
	aborted = true
	if exited, err := brokerExecutionOriginalExited(child, created); err != nil || !exited {
		t.Fatal("actual exited original refused", exited, err)
	}
	if _, err := brokerExecutionOriginalExited(child, created+1); err == nil {
		t.Fatal("signaled handle bypassed creation check")
	}
	// Keep the original fixture's no-entrypoint assertion explicit.
	if _, err := os.Stat(receipt); !errors.Is(err, os.ErrNotExist) {
		t.Fatal("inert entrypoint executed", err)
	}
}
