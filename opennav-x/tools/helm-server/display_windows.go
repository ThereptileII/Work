//go:build windows

package helm

import (
	"errors"
	"image"
	"strconv"
	"strings"
	"sync"
	"unicode/utf16"
	"unsafe"

	"golang.org/x/sys/windows"
)

// Windows display: GDI BitBlt of the SKAGER window (or the primary screen) and
// SendInput for touches. The process is per-monitor DPI aware, so capture and
// input use physical pixels on every scaling setting.

var (
	user32 = windows.NewLazySystemDLL("user32.dll")
	gdi32  = windows.NewLazySystemDLL("gdi32.dll")
	dwmapi = windows.NewLazySystemDLL("dwmapi.dll")

	procGetDC                         = user32.NewProc("GetDC")
	procReleaseDC                     = user32.NewProc("ReleaseDC")
	procGetSystemMetrics              = user32.NewProc("GetSystemMetrics")
	procIsWindow                      = user32.NewProc("IsWindow")
	procIsIconic                      = user32.NewProc("IsIconic")
	procIsWindowVisible               = user32.NewProc("IsWindowVisible")
	procGetWindowRect                 = user32.NewProc("GetWindowRect")
	procSendInput                     = user32.NewProc("SendInput")
	procSetProcessDpiAwarenessContext = user32.NewProc("SetProcessDpiAwarenessContext")
	procSetProcessDPIAware            = user32.NewProc("SetProcessDPIAware")
	procCreateCompatibleDC            = gdi32.NewProc("CreateCompatibleDC")
	procCreateDIBSection              = gdi32.NewProc("CreateDIBSection")
	procSelectObject                  = gdi32.NewProc("SelectObject")
	procDeleteObject                  = gdi32.NewProc("DeleteObject")
	procDeleteDC                      = gdi32.NewProc("DeleteDC")
	procBitBlt                        = gdi32.NewProc("BitBlt")
	procDwmGetWindowAttribute         = dwmapi.NewProc("DwmGetWindowAttribute")
)

const (
	smCXScreen        = 0
	smCYScreen        = 1
	smXVirtualScreen  = 76
	smYVirtualScreen  = 77
	smCXVirtualScreen = 78
	smCYVirtualScreen = 79
	srcCopy           = 0x00CC0020
	captureBlt        = 0x40000000
	dwmaExtendedFrame = 9

	inputMouse    = 0
	inputKeyboard = 1

	mouseMove      = 0x0001
	mouseLeftDown  = 0x0002
	mouseLeftUp    = 0x0004
	mouseRightDown = 0x0008
	mouseRightUp   = 0x0010
	mouseWheel     = 0x0800
	mouseVirtual   = 0x4000
	mouseAbsolute  = 0x8000
	keyExtended    = 0x0001
	keyUp          = 0x0002
	keyUnicode     = 0x0004
)

type rect struct{ Left, Top, Right, Bottom int32 }

type bitmapInfoHeader struct {
	Size          uint32
	Width         int32
	Height        int32
	Planes        uint16
	BitCount      uint16
	Compression   uint32
	SizeImage     uint32
	XPelsPerMeter int32
	YPelsPerMeter int32
	ClrUsed       uint32
	ClrImportant  uint32
}

// INPUT is a DWORD type followed by a union aligned to the pointer size:
// 40 bytes on 64-bit Windows, 28 on 32-bit (SKAGER ships as 32-bit x86).
const (
	ptrSize   = 4 << (^uintptr(0) >> 63)
	inputPad  = ptrSize - 4 // after Type, so the union is pointer aligned
	keyTail   = 8           // KEYBDINPUT is 8 bytes shorter than MOUSEINPUT
	inputSize = 4 + inputPad + 20 + inputPad + ptrSize
)

type mouseInput struct {
	Type      uint32
	_         [inputPad]byte
	Dx, Dy    int32
	MouseData uint32
	Flags     uint32
	Time      uint32
	ExtraInfo uintptr // Go aligns this to the pointer size, as C does
}

type keyInput struct {
	Type      uint32
	_         [inputPad]byte
	Vk        uint16
	Scan      uint16
	Flags     uint32
	Time      uint32
	ExtraInfo uintptr
	_         [keyTail]byte
}

// Fails to compile if a layout ever differs from Windows' INPUT.
var _ = [1]struct{}{}[unsafe.Sizeof(mouseInput{})-inputSize]
var _ = [1]struct{}{}[unsafe.Sizeof(keyInput{})-inputSize]

var virtualKeys = map[string]struct {
	vk       uint16
	extended bool
}{
	"Enter": {0x0D, false}, "Escape": {0x1B, false}, "Backspace": {0x08, false}, "Tab": {0x09, false},
	"Delete": {0x2E, true}, "ArrowLeft": {0x25, true}, "ArrowUp": {0x26, true}, "ArrowRight": {0x27, true},
	"ArrowDown": {0x28, true}, "Home": {0x24, true}, "End": {0x23, true}, "PageUp": {0x21, true}, "PageDown": {0x22, true},
}

type windowsDisplay struct {
	mu     sync.Mutex
	hwnd   uintptr
	origin image.Point // screen position of the last captured frame
	size   image.Point
	memDC  uintptr
	bitmap uintptr
	old    uintptr
	bits   unsafe.Pointer
	bw, bh int
}

func openPlatformDisplay(target string) (Display, error) {
	if procSetProcessDpiAwarenessContext.Find() == nil {
		procSetProcessDpiAwarenessContext.Call(^uintptr(3)) // DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2 (-4)
	} else {
		procSetProcessDPIAware.Call()
	}
	d := &windowsDisplay{}
	if target != "" {
		v, err := strconv.ParseUint(strings.TrimPrefix(strings.ToLower(target), "0x"), base(target), 64)
		if err != nil || v == 0 {
			return nil, errors.New("invalid window handle")
		}
		d.hwnd = uintptr(v)
	}
	return d, nil
}

func base(s string) int {
	if strings.HasPrefix(strings.ToLower(s), "0x") {
		return 16
	}
	return 10
}

func (d *windowsDisplay) region() (image.Rectangle, error) {
	if d.hwnd == 0 {
		w, _, _ := procGetSystemMetrics.Call(smCXScreen)
		h, _, _ := procGetSystemMetrics.Call(smCYScreen)
		return image.Rect(0, 0, int(w), int(h)), nil
	}
	if ok, _, _ := procIsWindow.Call(d.hwnd); ok == 0 {
		return image.Rectangle{}, ErrSurfaceUnavailable
	}
	if iconic, _, _ := procIsIconic.Call(d.hwnd); iconic != 0 {
		return image.Rectangle{}, ErrSurfaceUnavailable
	}
	if visible, _, _ := procIsWindowVisible.Call(d.hwnd); visible == 0 {
		return image.Rectangle{}, ErrSurfaceUnavailable
	}
	var r rect
	if hr, _, _ := procDwmGetWindowAttribute.Call(d.hwnd, dwmaExtendedFrame, uintptr(unsafe.Pointer(&r)), unsafe.Sizeof(r)); hr != 0 {
		if ok, _, _ := procGetWindowRect.Call(d.hwnd, uintptr(unsafe.Pointer(&r))); ok == 0 {
			return image.Rectangle{}, ErrSurfaceUnavailable
		}
	}
	out := image.Rect(int(r.Left), int(r.Top), int(r.Right), int(r.Bottom))
	if out.Dx() < 64 || out.Dy() < 64 {
		return image.Rectangle{}, ErrSurfaceUnavailable
	}
	return out, nil
}

func (d *windowsDisplay) ensureBitmap(w, h int) error {
	if d.memDC != 0 && d.bw == w && d.bh == h {
		return nil
	}
	d.freeBitmap()
	screen, _, _ := procGetDC.Call(0)
	defer procReleaseDC.Call(0, screen)
	mem, _, _ := procCreateCompatibleDC.Call(screen)
	if mem == 0 {
		return errors.New("CreateCompatibleDC failed")
	}
	info := struct {
		Header bitmapInfoHeader
		Colors [4]byte
	}{Header: bitmapInfoHeader{Width: int32(w), Height: -int32(h), Planes: 1, BitCount: 32}}
	info.Header.Size = uint32(unsafe.Sizeof(info.Header))
	var bits unsafe.Pointer
	bmp, _, _ := procCreateDIBSection.Call(mem, uintptr(unsafe.Pointer(&info)), 0, uintptr(unsafe.Pointer(&bits)), 0, 0)
	if bmp == 0 || bits == nil {
		procDeleteDC.Call(mem)
		return errors.New("CreateDIBSection failed")
	}
	old, _, _ := procSelectObject.Call(mem, bmp)
	d.memDC, d.bitmap, d.old, d.bits, d.bw, d.bh = mem, bmp, old, bits, w, h
	return nil
}

func (d *windowsDisplay) freeBitmap() {
	if d.memDC == 0 {
		return
	}
	procSelectObject.Call(d.memDC, d.old)
	procDeleteObject.Call(d.bitmap)
	procDeleteDC.Call(d.memDC)
	d.memDC, d.bitmap, d.bits = 0, 0, nil
}

func (d *windowsDisplay) Capture(dst *image.RGBA) (*image.RGBA, error) {
	d.mu.Lock()
	defer d.mu.Unlock()
	r, err := d.region()
	if err != nil {
		return dst, err
	}
	w, h := r.Dx(), r.Dy()
	if err := d.ensureBitmap(w, h); err != nil {
		return dst, err
	}
	screen, _, _ := procGetDC.Call(0)
	ok, _, _ := procBitBlt.Call(d.memDC, 0, 0, uintptr(w), uintptr(h), screen, uintptr(r.Min.X), uintptr(r.Min.Y), srcCopy|captureBlt)
	procReleaseDC.Call(0, screen)
	if ok == 0 {
		// The secure desktop (lock screen, UAC prompt) cannot be captured.
		return dst, ErrSurfaceUnavailable
	}
	if dst == nil || dst.Rect.Dx() != w || dst.Rect.Dy() != h {
		dst = image.NewRGBA(image.Rect(0, 0, w, h))
	}
	src := unsafe.Slice((*byte)(d.bits), w*h*4)
	pix := dst.Pix
	for i := 0; i < len(src); i += 4 {
		pix[i], pix[i+1], pix[i+2], pix[i+3] = src[i+2], src[i+1], src[i], 0xff
	}
	d.origin, d.size = r.Min, image.Pt(w, h)
	return dst, nil
}

func (d *windowsDisplay) send(inputs unsafe.Pointer, count int, size uintptr) error {
	n, _, err := procSendInput.Call(uintptr(count), uintptr(inputs), size)
	if int(n) != count {
		return err
	}
	return nil
}

func (d *windowsDisplay) mouse(flags uint32, dx, dy int32, data uint32) error {
	in := mouseInput{Type: inputMouse, Dx: dx, Dy: dy, MouseData: data, Flags: flags, ExtraInfo: HelmInputTag}
	return d.send(unsafe.Pointer(&in), 1, unsafe.Sizeof(in))
}

func (d *windowsDisplay) PointerMove(x, y int) error {
	d.mu.Lock()
	origin, size := d.origin, d.size
	d.mu.Unlock()
	if size.X == 0 {
		return ErrSurfaceUnavailable
	}
	sx, sy := origin.X+clamp(x, 0, size.X-1), origin.Y+clamp(y, 0, size.Y-1)
	vx, _, _ := procGetSystemMetrics.Call(smXVirtualScreen)
	vy, _, _ := procGetSystemMetrics.Call(smYVirtualScreen)
	vw, _, _ := procGetSystemMetrics.Call(smCXVirtualScreen)
	vh, _, _ := procGetSystemMetrics.Call(smCYVirtualScreen)
	if int32(vw) < 2 || int32(vh) < 2 {
		return errors.New("no virtual screen")
	}
	ax := int32((int64(sx-int(int32(vx)))*65535 + int64(int32(vw)-1)/2) / int64(int32(vw)-1))
	ay := int32((int64(sy-int(int32(vy)))*65535 + int64(int32(vh)-1)/2) / int64(int32(vh)-1))
	return d.mouse(mouseMove|mouseAbsolute|mouseVirtual, ax, ay, 0)
}

func (d *windowsDisplay) PointerButton(button int, down bool) error {
	var flag uint32
	switch {
	case button == 2 && down:
		flag = mouseRightDown
	case button == 2:
		flag = mouseRightUp
	case down:
		flag = mouseLeftDown
	default:
		flag = mouseLeftUp
	}
	return d.mouse(flag, 0, 0, 0)
}

func (d *windowsDisplay) Wheel(notches float64) error {
	return d.mouse(mouseWheel, 0, 0, uint32(int32(notches*120)))
}

func (d *windowsDisplay) Key(name string) error {
	k, ok := virtualKeys[name]
	if !ok {
		return errInvalidInput
	}
	var flags uint32
	if k.extended {
		flags = keyExtended
	}
	in := [2]keyInput{{Type: inputKeyboard, Vk: k.vk, Flags: flags, ExtraInfo: HelmInputTag},
		{Type: inputKeyboard, Vk: k.vk, Flags: flags | keyUp, ExtraInfo: HelmInputTag}}
	return d.send(unsafe.Pointer(&in[0]), len(in), unsafe.Sizeof(in[0]))
}

func (d *windowsDisplay) Text(text string) error {
	units := utf16.Encode([]rune(text))
	in := make([]keyInput, 0, len(units)*2)
	for _, u := range units {
		in = append(in, keyInput{Type: inputKeyboard, Scan: u, Flags: keyUnicode, ExtraInfo: HelmInputTag},
			keyInput{Type: inputKeyboard, Scan: u, Flags: keyUnicode | keyUp, ExtraInfo: HelmInputTag})
	}
	if len(in) == 0 {
		return nil
	}
	return d.send(unsafe.Pointer(&in[0]), len(in), unsafe.Sizeof(in[0]))
}

func (d *windowsDisplay) Close() error {
	d.mu.Lock()
	defer d.mu.Unlock()
	d.freeBitmap()
	return nil
}

func clamp(v, lo, hi int) int {
	if v < lo {
		return lo
	}
	if v > hi {
		return hi
	}
	return v
}
