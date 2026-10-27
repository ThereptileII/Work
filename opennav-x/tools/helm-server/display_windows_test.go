//go:build windows

package helm

import (
	"errors"
	"os"
	"testing"
	"unsafe"
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
	for _, target := range [][2]int{{100, 80}, {w / 2, h / 2}, {w - 1, h - 1}} {
		if err := d.PointerMove(target[0], target[1]); err != nil {
			t.Fatal(err)
		}
		var p struct{ X, Y int32 }
		getCursorPos.Call(uintptr(unsafe.Pointer(&p)))
		if abs(int(p.X)-target[0]) > 1 || abs(int(p.Y)-target[1]) > 1 {
			t.Fatalf("SendInput moved the cursor to %d,%d, want %d,%d", p.X, p.Y, target[0], target[1])
		}
	}
}

func abs(v int) int {
	if v < 0 {
		return -v
	}
	return v
}
