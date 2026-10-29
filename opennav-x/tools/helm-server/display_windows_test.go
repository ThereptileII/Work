//go:build windows

package helm

import (
	"errors"
	"os"
	"runtime"
	"testing"
	"time"
	"unsafe"

	"golang.org/x/sys/windows"
)

// TestWindowsDesktopCaptureAndPointer runs the real GDI capture and SendInput
// path on an interactive Windows desktop (CI sets HELM_REQUIRE_DESKTOP=1).
func TestWindowsDesktopCaptureAndPointer(t *testing.T) {
	d, err := OpenDisplay("")
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
	w, h := frame.Rect.Dx(), frame.Rect.Dy()
	if w < 640 || h < 480 {
		t.Fatalf("captured %dx%d", w, h)
	}
	opaque := true
	for i := 3; i < len(frame.Pix); i += 4 * 997 {
		opaque = opaque && frame.Pix[i] == 0xff
	}
	if !opaque {
		t.Fatal("captured pixels are not opaque RGBA")
	}
	t.Logf("captured the primary screen: %dx%d", w, h)

	getCursorPos := user32.NewProc("GetCursorPos")
	setCursorPos := user32.NewProc("SetCursorPos")
	// A desktop without interactive input (a CI service session) ignores every
	// cursor change; only then is the SendInput check skipped.
	setCursorPos.Call(37, 41)
	var probe struct{ X, Y int32 }
	getCursorPos.Call(uintptr(unsafe.Pointer(&probe)))
	if probe.X != 37 || probe.Y != 41 {
		if os.Getenv("HELM_REQUIRE_INPUT") == "1" {
			t.Fatalf("this desktop does not accept cursor changes (SetCursorPos gave %d,%d)", probe.X, probe.Y)
		}
		t.Skipf("capture verified; this desktop does not accept cursor changes, so SendInput cannot be checked here")
	}
	for _, target := range [][2]int{{100, 80}, {w / 2, h / 2}, {w - 1, h - 1}} {
		if err := d.PointerMove(target[0], target[1]); err != nil {
			t.Fatal(err)
		}
		// SendInput queues the move; the cursor follows once Windows processes it.
		var p struct{ X, Y int32 }
		for deadline := time.Now().Add(300 * time.Millisecond); ; {
			getCursorPos.Call(uintptr(unsafe.Pointer(&p)))
			if (abs(int(p.X)-target[0]) <= 1 && abs(int(p.Y)-target[1]) <= 1) || time.Now().After(deadline) {
				break
			}
			time.Sleep(5 * time.Millisecond)
		}
		if abs(int(p.X)-target[0]) > 1 || abs(int(p.Y)-target[1]) > 1 {
			t.Fatalf("SendInput moved the cursor to %d,%d, want %d,%d", p.X, p.Y, target[0], target[1])
		}
	}
}

// Messages seen by the touch test window (its window procedure runs on the test thread).
var touchSeen struct{ down, update, up, notify, gesture, pan int }

// TestWindowsTouchSwipeScrolls injects a phone swipe into a window that asks for
// vertical pan gestures, as SKAGER's scrolling lists do (wxTOUCH_VERTICAL_PAN_GESTURE).
// Windows must recognize it as a pan, not a press-move-release.
func TestWindowsTouchSwipeScrolls(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	d, err := OpenDisplay("")
	if err != nil {
		t.Fatal(err)
	}
	defer d.Close()
	wd := d.(*windowsDisplay)
	if wd.touch == nil {
		if os.Getenv("HELM_REQUIRE_TOUCH") == "1" {
			t.Fatal("touch injection is not available")
		}
		t.Skip("touch injection is not available on this Windows")
	}
	if _, err := d.Capture(nil); err != nil {
		if os.Getenv("HELM_REQUIRE_DESKTOP") == "1" {
			t.Fatal(err)
		}
		t.Skip("no interactive desktop")
	}

	const (
		wmPointerUpdate = 0x0245
		wmPointerDown   = 0x0246
		wmPointerUp     = 0x0247
		wmGesture       = 0x0119
		wmGestureNotify = 0x011A
		gidPan          = 4
	)
	defWindowProc := user32.NewProc("DefWindowProcW")
	getGestureInfo := user32.NewProc("GetGestureInfo")
	proc := windows.NewCallback(func(hwnd, msg, wparam, lparam uintptr) uintptr {
		switch msg {
		case wmPointerDown:
			touchSeen.down++
		case wmPointerUpdate:
			touchSeen.update++
		case wmPointerUp:
			touchSeen.up++
		case wmGestureNotify:
			touchSeen.notify++
		case wmGesture:
			touchSeen.gesture++
			// GESTUREINFO: cbSize, dwFlags, dwID, ... (40 + 2 pointers bytes).
			var info [64]byte
			*(*uint32)(unsafe.Pointer(&info[0])) = 40 + 2*ptrSize
			if ok, _, _ := getGestureInfo.Call(lparam, uintptr(unsafe.Pointer(&info[0]))); ok != 0 &&
				*(*uint32)(unsafe.Pointer(&info[8])) == gidPan {
				touchSeen.pan++
			}
		}
		r, _, _ := defWindowProc.Call(hwnd, msg, wparam, lparam)
		return r
	})
	className, _ := windows.UTF16PtrFromString("SkagerHelmTouchTest")
	instance, _, _ := kernel32GetModuleHandle.Call(0)
	class := struct {
		Size, Style             uint32
		Proc                    uintptr
		ClassExtra, WindowExtra int32
		Instance, Icon, Cursor  uintptr
		Background              uintptr
		Menu, Name              *uint16
		IconSmall               uintptr
	}{Proc: proc, Instance: instance, Name: className}
	class.Size = uint32(unsafe.Sizeof(class))
	if atom, _, err := user32.NewProc("RegisterClassExW").Call(uintptr(unsafe.Pointer(&class))); atom == 0 {
		t.Fatal(err)
	}
	const x, y, size = 200, 160, 420
	hwnd, _, err := user32.NewProc("CreateWindowExW").Call(0x8 /* topmost */, uintptr(unsafe.Pointer(className)), 0,
		0x90000000 /* popup, visible */, x, y, size, size, 0, 0, instance, 0)
	if hwnd == 0 {
		t.Fatal(err)
	}
	defer user32.NewProc("DestroyWindow").Call(hwnd)
	// GESTURECONFIG {GID_PAN, want GC_PAN | single finger vertically, block single finger horizontally}
	config := [3]uint32{gidPan, 0x1 | 0x2, 0x4}
	if ok, _, err := user32.NewProc("SetGestureConfig").Call(hwnd, 0, 1, uintptr(unsafe.Pointer(&config)), unsafe.Sizeof(config)); ok == 0 {
		t.Fatal(err)
	}
	user32.NewProc("SetForegroundWindow").Call(hwnd)
	pump := func(d time.Duration) {
		var msg [64]byte
		peek, translate, dispatch := user32.NewProc("PeekMessageW"), user32.NewProc("TranslateMessage"), user32.NewProc("DispatchMessageW")
		for end := time.Now().Add(d); time.Now().Before(end); {
			for {
				if ok, _, _ := peek.Call(uintptr(unsafe.Pointer(&msg[0])), 0, 0, 0, 1 /* remove */); ok == 0 {
					break
				}
				translate.Call(uintptr(unsafe.Pointer(&msg[0])))
				dispatch.Call(uintptr(unsafe.Pointer(&msg[0])))
			}
			time.Sleep(4 * time.Millisecond)
		}
	}
	pump(200 * time.Millisecond)

	// What the session does for a phone swipe: move, press, moves, release.
	cx, cy := x+size/2, y+size-60
	if err := d.PointerMove(cx, cy); err != nil {
		t.Fatal(err)
	}
	if err := d.PointerButton(0, true); err != nil {
		t.Fatal(err)
	}
	for step := 1; step <= 12; step++ {
		pump(16 * time.Millisecond)
		if err := d.PointerMove(cx, cy-step*25); err != nil {
			t.Fatal(err)
		}
	}
	pump(200 * time.Millisecond) // resting finger: refreshed, not timed out
	if err := d.PointerButton(0, false); err != nil {
		t.Fatal(err)
	}
	pump(300 * time.Millisecond)
	t.Logf("pointer down %d, update %d, up %d; gesture notify %d, gestures %d, pan %d",
		touchSeen.down, touchSeen.update, touchSeen.up, touchSeen.notify, touchSeen.gesture, touchSeen.pan)
	if touchSeen.down != 1 || touchSeen.up != 1 || touchSeen.update < 12 {
		t.Fatal("the window did not receive one touch contact")
	}
	// Windows Server without Ink and Handwriting Services has no gesture
	// engine: the contact arrives, but no window is ever offered gestures.
	if touchSeen.notify == 0 && os.Getenv("HELM_REQUIRE_GESTURES") != "1" {
		t.Skip("touch contact verified; this Windows has no gesture engine, so the pan cannot be checked here")
	}
	if touchSeen.pan == 0 {
		t.Fatal("Windows did not recognize the swipe as a pan gesture")
	}
	// The contact is lifted: another press starts a new one.
	if wd.held {
		t.Fatal("contact still held")
	}
}

var kernel32GetModuleHandle = windows.NewLazySystemDLL("kernel32.dll").NewProc("GetModuleHandleW")

func abs(v int) int {
	if v < 0 {
		return -v
	}
	return v
}
