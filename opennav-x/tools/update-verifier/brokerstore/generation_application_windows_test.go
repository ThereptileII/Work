package brokerstore

import (
	"context"
	"encoding/json"
	"os"
	"runtime"
	"strings"
	"testing"
	"unsafe"

	"golang.org/x/sys/windows"
)

// Native fixture executes no application or test entrypoint. Windows creates a
// suspended process from a protected copy of this test image; it is never resumed
// and only the fixture's original creator handles are used to terminate/drain it.
func TestNativeGenerationApplicationMatchesOnlyRetainedOriginalSuspendedImage(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	s, previous, _, f := nativeGenerationFixture(t)
	path, err := os.Executable()
	if err != nil {
		t.Fatal(err)
	}
	raw, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}
	f.payload[generationApplicationMember] = raw
	var product map[string]any
	if err := json.Unmarshal(f.payload["docs/PRODUCT_BUILD.json"], &product); err != nil {
		t.Fatal(err)
	}
	product["executable_sha256"] = registrationDigest(raw)
	f.payload["docs/PRODUCT_BUILD.json"], err = json.Marshal(product)
	if err != nil {
		t.Fatal(err)
	}
	// Rebind every original sidecar/inventory after replacing the fixture PE.
	f = generationImageData(t, f)
	candidate := nativeGenerationSource(t, s, f, strings.Repeat("9", 32))
	owner, err := s.MaterializeCandidateReadOnlyFresh(context.Background(), previous, candidate)
	if err != nil {
		t.Fatal(err)
	}
	defer owner.Close()
	image, err := owner.ApplicationImage(context.Background())
	if err != nil {
		t.Fatal(err)
	}
	native := nativeHistoryOwner(t, owner)
	executable, err := windows.UTF16PtrFromString(physical(native.members[generationApplicationMember].pin.name))
	if err != nil {
		t.Fatal(err)
	}
	startup := windows.StartupInfo{Cb: uint32(unsafe.Sizeof(windows.StartupInfo{}))}
	var process windows.ProcessInformation
	if err := windows.CreateProcess(executable, nil, nil, nil, false, windows.CREATE_SUSPENDED|windows.CREATE_NO_WINDOW, nil, nil, &startup, &process); err != nil {
		t.Fatal("create inert suspended image", err)
	}
	defer func() {
		if err := windows.TerminateProcess(process.Process, 0); err != nil {
			t.Error("terminate owned suspended fixture", err)
		}
		status, err := windows.WaitForSingleObject(process.Process, 10000)
		if err != nil || status != windows.WAIT_OBJECT_0 {
			t.Error("drain owned suspended fixture", status, err)
		}
		if err := windows.CloseHandle(process.Thread); err != nil {
			t.Error(err)
		}
		if err := windows.CloseHandle(process.Process); err != nil {
			t.Error(err)
		}
	}()
	var created, exited, kernel, user windows.Filetime
	if err := windows.GetProcessTimes(process.Process, &created, &exited, &kernel, &user); err != nil {
		t.Fatal(err)
	}
	creation := uint64(created.HighDateTime)<<32 | uint64(created.LowDateTime)
	if err := image.MatchesProcess(context.Background(), process.ProcessId, creation+1); err == nil {
		t.Fatal("different process creation accepted")
	}
	if err := image.MatchesProcess(context.Background(), windows.GetCurrentProcessId(), creation); err == nil {
		t.Fatal("identical image bytes at another path became original application")
	}
	if err := image.MatchesProcess(context.Background(), process.ProcessId, creation); err != nil {
		t.Fatal("actual retained original suspended image refused", err)
	}
	if err := owner.Close(); err != nil {
		t.Fatal(err)
	}
	if err := image.MatchesProcess(context.Background(), process.ProcessId, creation); err == nil {
		t.Fatal("closed generation left application capability live")
	}
}
