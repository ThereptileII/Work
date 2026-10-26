//go:build linux

package helm

import (
	"errors"
	"image"
	"strconv"
	"strings"
	"sync"

	"github.com/jezek/xgb"
	"github.com/jezek/xgb/xproto"
	"github.com/jezek/xgb/xtest"
)

// X11 display (Linux): GetImage of the SKAGER window region from the root
// window, XTEST for input. Wayland sessions run SKAGER under XWayland or X11.

var x11Keysyms = map[string]xproto.Keysym{
	"Enter": 0xff0d, "Escape": 0xff1b, "Backspace": 0xff08, "Tab": 0xff09, "Delete": 0xffff,
	"ArrowLeft": 0xff51, "ArrowUp": 0xff52, "ArrowRight": 0xff53, "ArrowDown": 0xff54,
	"Home": 0xff50, "End": 0xff57, "PageUp": 0xff55, "PageDown": 0xff56,
}

type x11Display struct {
	mu      sync.Mutex
	conn    *xgb.Conn
	root    xproto.Window
	window  xproto.Window
	origin  image.Point
	size    image.Point
	keymap  map[xproto.Keysym]keyChoice
	shiftKC xproto.Keycode
}

type keyChoice struct {
	code  xproto.Keycode
	shift bool
}

func openPlatformDisplay(target string) (Display, error) {
	conn, err := xgb.NewConn()
	if err != nil {
		return nil, errors.New("no X11 display: " + err.Error())
	}
	if err := xtest.Init(conn); err != nil {
		conn.Close()
		return nil, errors.New("X11 XTEST extension unavailable")
	}
	setup := xproto.Setup(conn)
	d := &x11Display{conn: conn, root: setup.DefaultScreen(conn).Root}
	if target != "" {
		v, err := strconv.ParseUint(strings.TrimPrefix(strings.ToLower(target), "0x"), base(target), 32)
		if err != nil || v == 0 {
			conn.Close()
			return nil, errors.New("invalid window id")
		}
		d.window = xproto.Window(v)
	}
	if err := d.loadKeymap(setup); err != nil {
		conn.Close()
		return nil, err
	}
	return d, nil
}

func base(s string) int {
	if strings.HasPrefix(strings.ToLower(s), "0x") {
		return 16
	}
	return 10
}

func (d *x11Display) loadKeymap(setup *xproto.SetupInfo) error {
	count := byte(int(setup.MaxKeycode) - int(setup.MinKeycode) + 1)
	reply, err := xproto.GetKeyboardMapping(d.conn, setup.MinKeycode, count).Reply()
	if err != nil {
		return err
	}
	per := int(reply.KeysymsPerKeycode)
	d.keymap = map[xproto.Keysym]keyChoice{}
	for i := 0; i < int(count); i++ {
		code := xproto.Keycode(int(setup.MinKeycode) + i)
		for level := 0; level < per && level < 2; level++ {
			sym := reply.Keysyms[i*per+level]
			if sym == 0 {
				continue
			}
			if _, seen := d.keymap[sym]; !seen {
				d.keymap[sym] = keyChoice{code: code, shift: level == 1}
			}
			if sym == 0xffe1 && d.shiftKC == 0 {
				d.shiftKC = code
			}
		}
	}
	return nil
}

func (d *x11Display) region() (image.Rectangle, error) {
	if d.window == 0 {
		g, err := xproto.GetGeometry(d.conn, xproto.Drawable(d.root)).Reply()
		if err != nil {
			return image.Rectangle{}, ErrSurfaceUnavailable
		}
		return image.Rect(0, 0, int(g.Width), int(g.Height)), nil
	}
	attrs, err := xproto.GetWindowAttributes(d.conn, d.window).Reply()
	if err != nil || attrs.MapState != xproto.MapStateViewable {
		return image.Rectangle{}, ErrSurfaceUnavailable
	}
	g, err := xproto.GetGeometry(d.conn, xproto.Drawable(d.window)).Reply()
	if err != nil {
		return image.Rectangle{}, ErrSurfaceUnavailable
	}
	t, err := xproto.TranslateCoordinates(d.conn, d.window, d.root, 0, 0).Reply()
	if err != nil {
		return image.Rectangle{}, ErrSurfaceUnavailable
	}
	r := image.Rect(int(t.DstX), int(t.DstY), int(t.DstX)+int(g.Width), int(t.DstY)+int(g.Height))
	rg, err := xproto.GetGeometry(d.conn, xproto.Drawable(d.root)).Reply()
	if err == nil {
		r = r.Intersect(image.Rect(0, 0, int(rg.Width), int(rg.Height)))
	}
	if r.Dx() < 64 || r.Dy() < 64 {
		return image.Rectangle{}, ErrSurfaceUnavailable
	}
	return r, nil
}

func (d *x11Display) Capture(dst *image.RGBA) (*image.RGBA, error) {
	d.mu.Lock()
	defer d.mu.Unlock()
	r, err := d.region()
	if err != nil {
		return dst, err
	}
	w, h := r.Dx(), r.Dy()
	if dst == nil || dst.Rect.Dx() != w || dst.Rect.Dy() != h {
		dst = image.NewRGBA(image.Rect(0, 0, w, h))
	}
	// Fetch in bands: a full-screen GetImage reply can exceed what one request may return.
	band := max(1, (4<<20)/(w*4))
	for y := 0; y < h; y += band {
		bh := min(band, h-y)
		img, err := xproto.GetImage(d.conn, xproto.ImageFormatZPixmap, xproto.Drawable(d.root),
			int16(r.Min.X), int16(r.Min.Y+y), uint16(w), uint16(bh), 0xffffffff).Reply()
		if err != nil {
			return dst, ErrSurfaceUnavailable
		}
		if len(img.Data) < w*bh*4 {
			return dst, errors.New("X11 display depth not supported (need 24/32-bit)")
		}
		out := dst.Pix[y*dst.Stride:]
		src := img.Data
		for i := 0; i < w*bh*4; i += 4 {
			out[i], out[i+1], out[i+2], out[i+3] = src[i+2], src[i+1], src[i], 0xff
		}
	}
	d.origin, d.size = r.Min, image.Pt(w, h)
	return dst, nil
}

func (d *x11Display) fake(kind byte, detail byte, x, y int16) error {
	return xtest.FakeInputChecked(d.conn, kind, detail, 0, d.root, x, y, 0).Check()
}

func (d *x11Display) PointerMove(x, y int) error {
	d.mu.Lock()
	origin, size := d.origin, d.size
	d.mu.Unlock()
	if size.X == 0 {
		return ErrSurfaceUnavailable
	}
	return d.fake(xproto.MotionNotify, 0, int16(origin.X+clamp(x, 0, size.X-1)), int16(origin.Y+clamp(y, 0, size.Y-1)))
}

func (d *x11Display) PointerButton(button int, down bool) error {
	b := byte(1)
	if button == 2 {
		b = 3
	}
	kind := byte(xproto.ButtonRelease)
	if down {
		kind = xproto.ButtonPress
	}
	return d.fake(kind, b, 0, 0)
}

func (d *x11Display) Wheel(notches float64) error {
	b := byte(4)
	if notches < 0 {
		b, notches = 5, -notches
	}
	for i := 0; i < int(notches+0.5); i++ {
		if err := d.fake(xproto.ButtonPress, b, 0, 0); err != nil {
			return err
		}
		if err := d.fake(xproto.ButtonRelease, b, 0, 0); err != nil {
			return err
		}
	}
	return nil
}

func (d *x11Display) press(sym xproto.Keysym) error {
	k, ok := d.keymap[sym]
	if !ok {
		return nil // not on this keyboard layout; skipped
	}
	if k.shift && d.shiftKC != 0 {
		if err := d.fake(xproto.KeyPress, byte(d.shiftKC), 0, 0); err != nil {
			return err
		}
		defer d.fake(xproto.KeyRelease, byte(d.shiftKC), 0, 0)
	}
	if err := d.fake(xproto.KeyPress, byte(k.code), 0, 0); err != nil {
		return err
	}
	return d.fake(xproto.KeyRelease, byte(k.code), 0, 0)
}

func (d *x11Display) Key(name string) error {
	sym, ok := x11Keysyms[name]
	if !ok {
		return errInvalidInput
	}
	return d.press(sym)
}

func (d *x11Display) Text(text string) error {
	for _, r := range text {
		sym := xproto.Keysym(r)
		if !(r >= 0x20 && r <= 0x7e) && !(r >= 0xa0 && r <= 0xff) {
			sym = xproto.Keysym(0x01000000 | uint32(r))
		}
		if err := d.press(sym); err != nil {
			return err
		}
	}
	return nil
}

func (d *x11Display) Close() error {
	d.conn.Close()
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
