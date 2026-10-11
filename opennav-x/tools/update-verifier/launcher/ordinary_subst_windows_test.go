package launcher

import (
	"errors"
	"os"
	"path/filepath"
	"strings"
	"testing"

	"golang.org/x/sys/windows"
)

// Explicitly opt in only on a disposable hosted Windows runner. This changes
// one previously absent test-session DOS definition, with no broadcast; cleanup
// removes only the exact owned target. It is never run on a boat or this host.
func requireNativeOrdinarySubst(t *testing.T) {
	t.Helper()
	if os.Getenv("SKAGER_LAUNCHER_DISPOSABLE_SUBST_TEST") != "1" {
		t.Skip("requires explicit disposable Windows runner opt-in")
	}
	user, err := windows.GetCurrentProcessToken().GetTokenUser()
	if err != nil || user.User.Sid.IsWellKnown(windows.WinLocalSystemSid) {
		t.Fatal("test requires a confirmed local user DOS namespace, never LocalSystem/global")
	}
}

func nativeOrdinarySubstMapping(t *testing.T, aliasRoot string) (string, func(string)) {
	t.Helper()
	directory, err := os.Open(aliasRoot)
	if err != nil {
		t.Fatal(err)
	}
	target, err := nativePhysicalName(windows.Handle(directory.Fd()))
	directory.Close()
	if err != nil {
		t.Fatal(err)
	}
	var device string
	for candidate := 'Z'; candidate >= 'R'; candidate-- {
		name := string(candidate) + ":"
		pointer, _ := windows.UTF16PtrFromString(name)
		buffer := make([]uint16, 32768)
		_, err := windows.QueryDosDevice(pointer, &buffer[0], uint32(len(buffer)))
		if errors.Is(err, windows.ERROR_FILE_NOT_FOUND) {
			device = name
			break
		}
		if err != nil {
			t.Fatal("test DOS definition absence could not be established")
		}
	}
	if device == "" {
		t.Fatal("no absent DOS letter available for isolated SUBST control")
	}
	name, _ := windows.UTF16PtrFromString(device)
	ownedTarget := ""
	readback := func() string {
		buffer := make([]uint16, 32768)
		n, err := windows.QueryDosDevice(name, &buffer[0], uint32(len(buffer)))
		if err != nil || n == 0 {
			t.Fatal("test DOS definition readback failed")
		}
		// A fresh letter must contain one target and the final MULTI_SZ NUL.
		first := windows.UTF16ToString(buffer[:n])
		if int(n) != len(windows.StringToUTF16(first))+1 {
			t.Fatal("test DOS definition has unexpected stacked targets")
		}
		return first
	}
	remove := func() {
		if ownedTarget == "" {
			return
		}
		pointer, _ := windows.UTF16PtrFromString(ownedTarget)
		if err := windows.DefineDosDevice(windows.DDD_RAW_TARGET_PATH|windows.DDD_REMOVE_DEFINITION|
			windows.DDD_EXACT_MATCH_ON_REMOVE|windows.DDD_NO_BROADCAST_SYSTEM, name, pointer); err != nil {
			t.Error("exact owned test DOS definition removal failed: " + err.Error())
			return
		}
		ownedTarget = ""
		buffer := make([]uint16, 32768)
		if _, err := windows.QueryDosDevice(name, &buffer[0], uint32(len(buffer))); !errors.Is(err, windows.ERROR_FILE_NOT_FOUND) {
			t.Error("test DOS letter absence was not restored after exact removal")
		}
	}
	t.Cleanup(remove)
	define := func(target string) {
		buffer := make([]uint16, 32768)
		if _, err := windows.QueryDosDevice(name, &buffer[0], uint32(len(buffer))); !errors.Is(err, windows.ERROR_FILE_NOT_FOUND) {
			t.Fatal("test DOS letter is no longer confirmed absent before definition")
		}
		pointer, _ := windows.UTF16PtrFromString(target)
		if err := windows.DefineDosDevice(windows.DDD_RAW_TARGET_PATH|windows.DDD_NO_BROADCAST_SYSTEM, name, pointer); err != nil {
			t.Fatal(err)
		}
		ownedTarget = target // cleanup knows the exact successful definition even if readback fails
		if readback() != target {
			t.Fatal("test DOS definition target differs")
		}
	}
	define(target)
	return device, func(target string) {
		remove()
		if ownedTarget != "" {
			t.Fatal("test DOS definition could not be safely remapped")
		}
		define(target)
	}
}

func TestNativeOrdinarySubstRemapAfterAcquire(t *testing.T) {
	requireNativeOrdinarySubst(t)
	spec, marker := ordinaryNativeFixture(t)
	aliasRoot := filepath.Dir(spec.directory)
	device, remap := nativeOrdinarySubstMapping(t, aliasRoot)
	spec.executable = device + `\app\opencpn.exe`
	spec.directory = device + `\app`
	image, err := acquireNativeOrdinaryImage(spec)
	if err != nil {
		t.Fatal(err)
	}
	owner := image.(*nativeOrdinaryImage)
	defer owner.Close()
	aboveAlias := filepath.Dir(aliasRoot)
	if err := os.Rename(aboveAlias, aboveAlias+".moved"); err == nil {
		t.Fatal("ancestor above SUBST root was not pinned")
	}
	// Remap discovery to a different, harmless but wrong image after acquisition.
	other := filepath.Join(t.TempDir(), "other-root")
	if err := os.MkdirAll(filepath.Join(other, "app"), 0700); err != nil {
		t.Fatal(err)
	}
	if err := os.WriteFile(filepath.Join(other, "app", "opencpn.exe"), []byte("not the acquired image"), 0600); err != nil {
		t.Fatal(err)
	}
	f, err := os.Open(other)
	if err != nil {
		t.Fatal(err)
	}
	otherTarget, err := nativePhysicalName(windows.Handle(f.Fd()))
	f.Close()
	if err != nil {
		t.Fatal(err)
	}
	remap(otherTarget)
	// The production state machine receives its already acquired physical owner.
	// Repeating acquisition through the mutable DOS alias is intentionally absent.
	if err := startOrdinary(spec, func(processSpec) (ordinaryImage, error) { return owner, nil }); err != nil {
		t.Fatal(err)
	}
	receipt := readOrdinaryNativeReceipt(t, marker)
	if !strings.EqualFold(receipt.Image, owner.name) || strings.HasPrefix(strings.ToLower(receipt.Image), strings.ToLower(otherTarget)+`\`) {
		t.Fatalf("DOS remap selected a different image: %+v", receipt)
	}
}
