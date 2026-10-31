//go:build windows

package helm

import (
	"image"
	"sync"
	"unsafe"

	"golang.org/x/sys/windows"
)

// Helm shows only SKAGER: its main window and the menus and dialogs it opens,
// each drawn from the window's own content (PrintWindow), never from the
// screen. Another program in front of SKAGER is not captured, and Helm input
// never lands on it.

var (
	procEnumWindows              = user32.NewProc("EnumWindows")
	procGetWindowThreadProcessID = user32.NewProc("GetWindowThreadProcessId")
	procPrintWindow              = user32.NewProc("PrintWindow")
	procWindowFromPoint          = user32.NewProc("WindowFromPoint")
	procGetAncestor              = user32.NewProc("GetAncestor")
	procGetForegroundWindow      = user32.NewProc("GetForegroundWindow")
	procSetForegroundWindow      = user32.NewProc("SetForegroundWindow")
	procBringWindowToTop         = user32.NewProc("BringWindowToTop")
	procAttachThreadInput        = user32.NewProc("AttachThreadInput")
	procCreateCompatibleBitmap   = gdi32.NewProc("CreateCompatibleBitmap")
	procPatBlt                   = gdi32.NewProc("PatBlt")
	procGetCurrentThreadID       = windows.NewLazySystemDLL("kernel32.dll").NewProc("GetCurrentThreadId")
)

const (
	pwRenderFullContent = 0x2 // includes DirectX/OpenGL content (Windows 8.1+)
	gaRoot              = 2
	blackness           = 0x00000042
)

// windowProcess returns the process that owns a window (0 when it is gone).
func windowProcess(hwnd uintptr) uint32 {
	var pid uint32
	procGetWindowThreadProcessID.Call(hwnd, uintptr(unsafe.Pointer(&pid)))
	return pid
}

// EnumWindows needs a callback; Windows allows few, so one is shared.
var (
	enumMu       sync.Mutex
	enumFound    []uintptr
	enumCallback = windows.NewCallback(func(hwnd, _ uintptr) uintptr {
		enumFound = append(enumFound, hwnd)
		return 1
	})
)

// topLevelWindows lists every top-level window, front to back.
func topLevelWindows() []uintptr {
	enumMu.Lock()
	defer enumMu.Unlock()
	enumFound = enumFound[:0]
	procEnumWindows.Call(enumCallback, 0)
	return append([]uintptr(nil), enumFound...)
}

// visibleBounds is the window as drawn on screen, without the invisible
// resize border; frame is the whole window, which PrintWindow renders.
func visibleBounds(hwnd uintptr) (visible, frame image.Rectangle, ok bool) {
	var f rect
	if r, _, _ := procGetWindowRect.Call(hwnd, uintptr(unsafe.Pointer(&f))); r == 0 {
		return image.Rectangle{}, image.Rectangle{}, false
	}
	frame = image.Rect(int(f.Left), int(f.Top), int(f.Right), int(f.Bottom))
	var v rect
	if hr, _, _ := procDwmGetWindowAttribute.Call(hwnd, dwmaExtendedFrame, uintptr(unsafe.Pointer(&v)), unsafe.Sizeof(v)); hr != 0 {
		return frame, frame, true
	}
	return image.Rect(int(v.Left), int(v.Top), int(v.Right), int(v.Bottom)).Intersect(frame), frame, true
}

// ownedWindows lists SKAGER's visible top-level windows that overlap the main
// window, back to front, so later ones are drawn on top.
func ownedWindows(main uintptr, pid uint32, region image.Rectangle) []uintptr {
	all := topLevelWindows()
	out := make([]uintptr, 0, 4)
	for i := len(all) - 1; i >= 0; i-- {
		h := all[i]
		if h != main && windowProcess(h) != pid {
			continue
		}
		if visible, _, _ := procIsWindowVisible.Call(h); visible == 0 {
			continue
		}
		if iconic, _, _ := procIsIconic.Call(h); iconic != 0 {
			continue
		}
		if v, _, ok := visibleBounds(h); !ok || !v.Overlaps(region) {
			continue
		}
		out = append(out, h)
	}
	return out
}

// composeOwned draws SKAGER's windows into memDC, whose origin is region.Min.
// It reports false when the main window itself could not be drawn.
func composeOwned(memDC uintptr, main uintptr, pid uint32, region image.Rectangle) bool {
	procPatBlt.Call(memDC, 0, 0, uintptr(region.Dx()), uintptr(region.Dy()), blackness)
	screen, _, _ := procGetDC.Call(0)
	defer procReleaseDC.Call(0, screen)
	drewMain := false
	for _, h := range ownedWindows(main, pid, region) {
		visible, frame, ok := visibleBounds(h)
		if !ok || frame.Dx() <= 0 || frame.Dy() <= 0 || frame.Dx() > 16384 || frame.Dy() > 16384 {
			continue
		}
		dc, _, _ := procCreateCompatibleDC.Call(screen)
		if dc == 0 {
			continue
		}
		bmp, _, _ := procCreateCompatibleBitmap.Call(screen, uintptr(frame.Dx()), uintptr(frame.Dy()))
		if bmp == 0 {
			procDeleteDC.Call(dc)
			continue
		}
		old, _, _ := procSelectObject.Call(dc, bmp)
		if printed, _, _ := procPrintWindow.Call(h, dc, pwRenderFullContent); printed != 0 {
			// Only the visible part: the invisible resize border is not SKAGER's content.
			part := visible.Intersect(region)
			procBitBlt.Call(memDC, uintptr(part.Min.X-region.Min.X), uintptr(part.Min.Y-region.Min.Y),
				uintptr(part.Dx()), uintptr(part.Dy()), dc, uintptr(part.Min.X-frame.Min.X), uintptr(part.Min.Y-frame.Min.Y), srcCopy)
			if h == main {
				drewMain = true
			}
		}
		procSelectObject.Call(dc, old)
		procDeleteObject.Call(bmp)
		procDeleteDC.Call(dc)
	}
	return drewMain
}

// ownsPoint reports whether the window under a screen point belongs to SKAGER.
func ownsPoint(pid uint32, at point) bool {
	// POINT is passed by value: one 64-bit register on x64, two stack words on x86.
	var h uintptr
	if ptrSize == 8 {
		h, _, _ = procWindowFromPoint.Call(uintptr(uint64(uint32(at.X)) | uint64(uint32(at.Y))<<32))
	} else {
		h, _, _ = procWindowFromPoint.Call(uintptr(at.X), uintptr(at.Y))
	}
	if h == 0 {
		return false
	}
	root, _, _ := procGetAncestor.Call(h, gaRoot)
	if root == 0 {
		root = h
	}
	return windowProcess(root) == pid
}

// ownsForeground reports whether keys typed now would reach SKAGER.
func ownsForeground(pid uint32) bool {
	h, _, _ := procGetForegroundWindow.Call()
	return h != 0 && windowProcess(h) == pid
}

// raise brings SKAGER in front of the program covering it, so the next touch
// reaches SKAGER. Windows only lets the foreground thread change the
// foreground, so Helm briefly shares its input state.
func raise(main uintptr) {
	fg, _, _ := procGetForegroundWindow.Call()
	self, _, _ := procGetCurrentThreadID.Call()
	var other uintptr
	if fg != 0 {
		other, _, _ = procGetWindowThreadProcessID.Call(fg, 0)
	}
	if other != 0 && other != self {
		procAttachThreadInput.Call(other, self, 1)
		defer procAttachThreadInput.Call(other, self, 0)
	}
	procBringWindowToTop.Call(main)
	procSetForegroundWindow.Call(main)
}
