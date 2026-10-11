package launcher

import (
	"encoding/json"
	"errors"
	"fmt"
	"os"
	"path/filepath"
	"reflect"
	"strconv"
	"strings"
	"testing"
	"time"
	"unicode/utf16"
	"unsafe"

	"golang.org/x/sys/windows"
)

type ordinaryNativeReceipt struct {
	Image, Directory string
	UpdateVariables  []string
	InheritedHandle  bool
	PID              uint32
}

// Only an explicitly selected inert child writes this private fixture receipt.
// No product, supervisor, shell, network, profile or device is executed.
func TestNativeOrdinaryChild(t *testing.T) {
	if os.Getenv("SKAGER_ORDINARY_INERT_CHILD") != "1" {
		return
	}
	if len(os.Args) != 4 {
		os.Exit(2)
	}
	buffer := make([]uint16, 32768)
	size := uint32(len(buffer))
	if windows.QueryFullProcessImageName(windows.CurrentProcess(), 1, &buffer[0], &size) != nil {
		os.Exit(3)
	}
	receipt := ordinaryNativeReceipt{Image: windows.UTF16ToString(buffer[:size]), PID: windows.GetCurrentProcessId()}
	dir, err := os.Open(".")
	if err != nil {
		os.Exit(4)
	}
	receipt.Directory, err = nativePhysicalName(windows.Handle(dir.Fd()))
	dir.Close()
	if err != nil {
		os.Exit(5)
	}
	for _, value := range os.Environ() {
		key, _, _ := strings.Cut(value, "=")
		if strings.HasPrefix(strings.ToUpper(key), "SKAGER_UPDATE_") {
			receipt.UpdateVariables = append(receipt.UpdateVariables, key)
		}
	}
	handle, err := strconv.ParseUint(os.Args[3], 10, 64)
	if err != nil {
		os.Exit(6)
	}
	var flags uint32
	valid, _, _ := windows.NewLazySystemDLL("kernel32.dll").NewProc("GetHandleInformation").Call(uintptr(handle), uintptr(unsafe.Pointer(&flags)))
	receipt.InheritedHandle = valid != 0
	data, _ := json.Marshal(receipt)
	if os.WriteFile(os.Args[2], data, 0600) != nil {
		os.Exit(7)
	}
	os.Exit(0)
}

func ordinaryNativeFixture(t *testing.T) (processSpec, string) {
	t.Helper()
	root := t.TempDir()
	directory := filepath.Join(root, "physical-parent", "alias-root", "app")
	if err := os.MkdirAll(directory, 0700); err != nil {
		t.Fatal(err)
	}
	executable, err := os.Executable()
	if err != nil {
		t.Fatal(err)
	}
	data, err := os.ReadFile(executable)
	if err != nil {
		t.Fatal(err)
	}
	image := filepath.Join(directory, "opencpn.exe")
	if err := os.WriteFile(image, data, 0600); err != nil {
		t.Fatal(err)
	}
	attributes := windows.SecurityAttributes{Length: uint32(unsafe.Sizeof(windows.SecurityAttributes{})), InheritHandle: 1}
	event, err := windows.CreateEvent(&attributes, 0, 0, nil)
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { windows.CloseHandle(event) })
	marker := filepath.Join(root, "inert-receipt.json")
	t.Setenv("SKAGER_ORDINARY_INERT_CHILD", "1")
	t.Setenv("SKAGER_UPDATE_CHALLENGE", "not-for-ordinary-children")
	t.Setenv("sKaGeR_UpDaTe_other", "also-stripped")
	return processSpec{executable: image, directory: directory, expectedImageHash: digest(data),
		arguments: []string{"-test.run=^TestNativeOrdinaryChild$", marker, fmt.Sprint(uintptr(event))}}, marker
}

func readOrdinaryNativeReceipt(t *testing.T, name string) ordinaryNativeReceipt {
	t.Helper()
	deadline := time.Now().Add(10 * time.Second)
	for time.Now().Before(deadline) {
		data, err := os.ReadFile(name)
		if err == nil {
			var receipt ordinaryNativeReceipt
			if json.Unmarshal(data, &receipt) == nil {
				// Observation only: never terminate by a receipt/PID. The creator
				// has already closed its owner; allow the inert child to exit
				// before deleting the copied test executable in TempDir cleanup.
				process, err := windows.OpenProcess(windows.SYNCHRONIZE, false, receipt.PID)
				if err == nil {
					status, waitErr := windows.WaitForSingleObject(process, 5000)
					windows.CloseHandle(process)
					if waitErr != nil || status != windows.WAIT_OBJECT_0 {
						t.Fatal("inert ordinary child exit not observed")
					}
				} else if !errors.Is(err, windows.ERROR_INVALID_PARAMETER) {
					t.Fatal(err)
				}
				return receipt
			}
		} else if !errors.Is(err, os.ErrNotExist) {
			t.Fatal(err)
		}
		time.Sleep(20 * time.Millisecond)
	}
	t.Fatal("inert ordinary child did not produce its receipt")
	return ordinaryNativeReceipt{}
}

func TestNativeOrdinaryActualCreator(t *testing.T) {
	spec, marker := ordinaryNativeFixture(t)
	file, err := openNativeRead(spec.executable)
	if err != nil {
		t.Fatal(err)
	}
	expected, err := nativePhysicalName(windows.Handle(file.Fd()))
	file.Close()
	if err != nil {
		t.Fatal(err)
	}
	if err := (nativePlatform{}).start(spec); err != nil {
		t.Fatal(err)
	}
	receipt := readOrdinaryNativeReceipt(t, marker)
	parent := expected[:strings.LastIndex(expected, `\`)]
	if !strings.EqualFold(receipt.Image, expected) || !strings.EqualFold(receipt.Directory, parent) ||
		len(receipt.UpdateVariables) != 0 || receipt.InheritedHandle {
		t.Fatalf("ordinary physical image/CWD/environment/handle control: %+v", receipt)
	}
}

func TestNativeOrdinaryPhysicalCustodyBeforeResume(t *testing.T) {
	spec, marker := ordinaryNativeFixture(t)
	acquired, err := acquireNativeOrdinaryImage(spec)
	if err != nil {
		t.Fatal(err)
	}
	owner := acquired.(*nativeOrdinaryImage)
	defer owner.Close()
	_, expectedParents, err := physicalAncestors(owner.name)
	if err != nil || len(owner.parents) < 4 || len(owner.parents) != len(expectedParents) {
		t.Fatal("true physical volume root/ancestor coverage missing")
	}
	for i, entry := range owner.parents {
		if entry.name != expectedParents[i] {
			t.Fatal("held physical ancestor differs from acquired image ancestry")
		}
	}
	if err := os.WriteFile(spec.executable, []byte("replace held inert image"), 0600); err == nil {
		t.Fatal("image write was not denied under ownership")
	}
	if err := os.Rename(spec.executable, spec.executable+".moved"); err == nil {
		t.Fatal("image rename was not denied under ownership")
	}
	for _, directory := range []string{spec.directory, filepath.Dir(spec.directory), filepath.Dir(filepath.Dir(spec.directory))} {
		if err := os.Rename(directory, directory+".moved"); err == nil {
			t.Fatal("physical ancestor rename was not denied: " + directory)
		}
	}
	childInterface, err := owner.create(spec)
	if err != nil {
		t.Fatal(err)
	}
	child := childInterface.(*nativeOrdinaryChild)
	defer child.Close()
	if err := child.corroborate(); err != nil {
		_ = child.abortSuspended()
		t.Fatal(err)
	}
	if _, err := os.Stat(marker); !errors.Is(err, os.ErrNotExist) {
		_ = child.abortSuspended()
		t.Fatal("inert child executed before the guarded resume")
	}
	if err := owner.recheck(); err != nil {
		_ = child.abortSuspended()
		t.Fatal(err)
	}
	count, err := child.resume()
	if err != nil || count != 1 {
		t.Fatalf("single native resume: count=%d err=%v", count, err)
	}
	if _, err := child.resume(); err == nil {
		t.Fatal("second resume was permitted")
	}
	if err := child.abortSuspended(); err == nil {
		t.Fatal("termination was permitted after a resume attempt")
	}
	receipt := readOrdinaryNativeReceipt(t, marker)
	if !strings.EqualFold(receipt.Image, owner.name) || len(receipt.UpdateVariables) != 0 {
		t.Fatalf("inert custody receipt: %+v", receipt)
	}
}

func TestNativeOrdinaryWrongHashAndHardlinkRefuse(t *testing.T) {
	for _, scenario := range []string{"hash", "hardlink"} {
		t.Run(scenario, func(t *testing.T) {
			spec, marker := ordinaryNativeFixture(t)
			if scenario == "hash" {
				spec.expectedImageHash = strings.Repeat("f", 64)
			} else if err := os.Link(spec.executable, spec.executable+".alias"); err != nil {
				t.Fatal(err)
			}
			if (nativePlatform{}).start(spec) == nil {
				t.Fatal("unowned ordinary image reached execution")
			}
			if _, err := os.Stat(marker); !errors.Is(err, os.ErrNotExist) {
				t.Fatal("refused image executed")
			}
		})
	}
}

func TestNativeOrdinarySuspendedRefusalCleanup(t *testing.T) {
	spec, marker := ordinaryNativeFixture(t)
	image, err := acquireNativeOrdinaryImage(spec)
	if err != nil {
		t.Fatal(err)
	}
	defer image.Close()
	childInterface, err := image.create(spec)
	if err != nil {
		t.Fatal(err)
	}
	child := childInterface.(*nativeOrdinaryChild)
	defer child.Close()
	if err := child.abortSuspended(); err == nil {
		t.Fatal("unproven raw child ownership authorized termination")
	}
	pid := child.pid
	child.pid++
	if err := child.corroborate(); err == nil {
		t.Fatal("mismatched returned PID was accepted")
	}
	if err := child.abortSuspended(); err == nil {
		t.Fatal("mismatched raw child identity authorized termination")
	}
	child.pid = pid
	if err := child.corroborate(); err != nil {
		t.Fatal(err)
	}
	creation := child.creation
	child.creation.LowDateTime ^= 1
	if err := child.abortSuspended(); err == nil {
		t.Fatal("inconsistent raw creation time authorized termination")
	}
	child.creation = creation
	if err := child.abortSuspended(); err != nil {
		t.Fatal(err)
	}
	if _, err := os.Stat(marker); !errors.Is(err, os.ErrNotExist) {
		t.Fatal("refused suspended child executed")
	}
}

func TestNativeOrdinaryNameAndEnvironmentBounds(t *testing.T) {
	root, parents, err := physicalAncestors(`\Device\HarddiskVolume7\above-subst\alias-root\app\opencpn.exe`)
	if err != nil || root != `\Device\HarddiskVolume7` || !reflect.DeepEqual(parents, []string{
		`\Device\HarddiskVolume7`, `\Device\HarddiskVolume7\above-subst`, `\Device\HarddiskVolume7\above-subst\alias-root`, `\Device\HarddiskVolume7\above-subst\alias-root\app`}) {
		t.Fatalf("physical ancestry: %q %v %v", root, parents, err)
	}
	for _, name := range []string{`C:\app\opencpn.exe`, `\Device\Mup\host\app`, `\Device\HarddiskVolume7\app\..\opencpn.exe`, `\Device\HarddiskVolume7\app\`, `\Device\HarddiskVolumeX\app\opencpn.exe`} {
		if _, _, err := physicalAncestors(name); err == nil {
			t.Fatal("unsupported physical spelling accepted: " + name)
		}
	}
	block, err := ordinaryEnvironment([]string{"PATH=C:\\Windows", "SKAGER_UPDATE_CHALLENGE=secret", "sKaGeR_UpDaTe_X=secret", "KEEP=ok", "=C:=C:\\owned"})
	if err != nil || block[len(block)-1] != 0 || block[len(block)-2] != 0 || strings.Contains(strings.ToUpper(string(utf16.Decode(block))), "SKAGER_UPDATE_") {
		t.Fatal("ordinary environment did not strip all update variables or terminate correctly")
	}
	for _, values := range [][]string{{"broken"}, {"A=a\x00B=b"}, {"BIG=" + strings.Repeat("a", 32768)}} {
		if _, err := ordinaryEnvironment(values); err == nil {
			t.Fatal("invalid/unbounded ordinary environment accepted")
		}
	}
}
