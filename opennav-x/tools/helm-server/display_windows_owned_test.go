//go:build windows

package helm

import (
	"errors"
	"fmt"
	"os"
	"os/exec"
	"runtime"
	"strconv"
	"testing"
	"time"
	"unsafe"

	"golang.org/x/sys/windows"
)

// A plain coloured popup window: the class background brush paints it.
func colourWindow(t testing.TB, class string, colour uint32, x, y, w, h int, topmost bool) uintptr {
	brush, _, _ := gdi32.NewProc("CreateSolidBrush").Call(uintptr(colour))
	name, _ := windows.UTF16PtrFromString(class)
	instance, _, _ := kernel32GetModuleHandle.Call(0)
	wc := struct {
		Size, Style             uint32
		Proc                    uintptr
		ClassExtra, WindowExtra int32
		Instance, Icon, Cursor  uintptr
		Background              uintptr
		Menu, Name              *uint16
		IconSmall               uintptr
	}{Proc: user32.NewProc("DefWindowProcW").Addr(), Instance: instance, Background: brush, Name: name}
	wc.Size = uint32(unsafe.Sizeof(wc))
	if atom, _, err := user32.NewProc("RegisterClassExW").Call(uintptr(unsafe.Pointer(&wc))); atom == 0 {
		t.Fatal(err)
	}
	var ex uintptr
	if topmost {
		ex = 0x8
	}
	hwnd, _, err := user32.NewProc("CreateWindowExW").Call(ex, uintptr(unsafe.Pointer(name)), 0,
		0x90000000 /* popup, visible */, uintptr(x), uintptr(y), uintptr(w), uintptr(h), 0, 0, instance, 0)
	if hwnd == 0 {
		t.Fatal(err)
	}
	return hwnd
}

func pumpMessages(d time.Duration) {
	var msg [64]byte
	peek, translate, dispatch := user32.NewProc("PeekMessageW"), user32.NewProc("TranslateMessage"), user32.NewProc("DispatchMessageW")
	for end := time.Now().Add(d); time.Now().Before(end); {
		for {
			if ok, _, _ := peek.Call(uintptr(unsafe.Pointer(&msg[0])), 0, 0, 0, 1); ok == 0 {
				break
			}
			translate.Call(uintptr(unsafe.Pointer(&msg[0])))
			dispatch.Call(uintptr(unsafe.Pointer(&msg[0])))
		}
		time.Sleep(4 * time.Millisecond)
	}
}

// TestMain doubles as "another program": with HELM_TEST_COVER=x,y,w,h it only
// shows a blue topmost window there for a few seconds.
func TestMain(m *testing.M) {
	if spec := os.Getenv("HELM_TEST_COVER"); spec != "" {
		runtime.LockOSThread()
		var x, y, w, h int
		if _, err := fmt.Sscanf(spec, "%d,%d,%d,%d", &x, &y, &w, &h); err != nil {
			os.Exit(2)
		}
		colourWindow(fatalOnly{}, "SkagerHelmOtherProgram", 0x00FF0000 /* blue */, x, y, w, h, true)
		fmt.Println("shown")
		pumpMessages(8 * time.Second)
		os.Exit(0)
	}
	os.Exit(m.Run())
}

type fatalOnly struct{ testing.TB }

func (fatalOnly) Fatal(args ...any) { os.Exit(3) }

// TestWindowsHelmShowsOnlySkager: another program's window over SKAGER is not
// in Helm's picture, and a touch there never reaches that program.
func TestWindowsHelmShowsOnlySkager(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	const x, y, size = 160, 140, 400
	skager := colourWindow(t, "SkagerHelmFakeSkager", 0x000000FF /* red */, x, y, size, size, false)
	defer user32.NewProc("DestroyWindow").Call(skager)
	user32.NewProc("SetForegroundWindow").Call(skager)
	pumpMessages(300 * time.Millisecond)

	d, err := OpenDisplay("0x" + strconv.FormatUint(uint64(skager), 16))
	if err != nil {
		t.Fatal(err)
	}
	defer d.Close()
	frame, err := d.Capture(nil)
	if errors.Is(err, ErrSurfaceUnavailable) && os.Getenv("HELM_REQUIRE_DESKTOP") != "1" {
		t.Skip("no interactive desktop")
	}
	if err != nil {
		t.Fatal(err)
	}
	at := func(px, py int) (uint8, uint8, uint8) {
		i := frame.PixOffset(px, py)
		return frame.Pix[i], frame.Pix[i+1], frame.Pix[i+2]
	}
	if r, g, b := at(size/2, size/2); r < 200 || g > 60 || b > 60 {
		t.Fatalf("SKAGER window not captured: centre is %d,%d,%d", r, g, b)
	}

	// Another program covers the middle of SKAGER.
	cover := exec.Command(os.Args[0], "-test.run", "^$")
	cover.Env = append(os.Environ(), fmt.Sprintf("HELM_TEST_COVER=%d,%d,%d,%d", x+100, y+100, 200, 200))
	out, _ := cover.StdoutPipe()
	if err := cover.Start(); err != nil {
		t.Fatal(err)
	}
	defer cover.Process.Kill()
	buf := make([]byte, 5)
	if _, err := out.Read(buf); err != nil || string(buf) != "shown" {
		t.Fatalf("cover window did not start: %q %v", buf, err)
	}
	pumpMessages(500 * time.Millisecond)

	frame, err = d.Capture(frame)
	if err != nil {
		t.Fatal(err)
	}
	if r, g, b := at(size/2, size/2); b > 100 || r < 200 {
		t.Fatalf("the other program's window was captured: centre is %d,%d,%d", r, g, b)
	}
	// A touch on the covered spot is refused (and SKAGER is brought forward).
	if err := d.PointerMove(size/2, size/2); !errors.Is(err, ErrSurfaceUnavailable) {
		t.Fatalf("a touch over another program was not refused: %v", err)
	}
	// Uncovered parts of SKAGER still take touches.
	if err := d.PointerMove(20, 20); err != nil {
		t.Fatalf("a touch on SKAGER was refused: %v", err)
	}
}
