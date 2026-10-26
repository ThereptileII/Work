package helm

import (
	"errors"
	"image"
	"image/color"
	"image/draw"
	"math"
	"sync"
	"time"

	"golang.org/x/image/font"
	"golang.org/x/image/font/basicfont"
	"golang.org/x/image/math/fixed"
)

// Display is the SKAGER surface on this computer: what Helm captures and where
// it injects the phone's touches. Coordinates are pixels of the captured frame;
// the platform maps them to the screen at the moment of injection, so a moved
// window still receives touches where the phone showed them.
type Display interface {
	// Capture returns the current frame, reusing dst when the size is unchanged.
	Capture(dst *image.RGBA) (*image.RGBA, error)
	PointerMove(x, y int) error
	PointerButton(button int, down bool) error
	Wheel(notches float64) error
	Key(name string) error
	Text(text string) error
	Close() error
}

// ErrSurfaceUnavailable means SKAGER is minimized, hidden or the desktop is
// locked. Helm keeps the session and shows the phone that the screen is not available.
var ErrSurfaceUnavailable = errors.New("SKAGER screen is not available")

// HelmInputTag marks every input event Helm injects on Windows (dwExtraInfo,
// readable with GetMessageExtraInfo). SKAGER uses it to refuse remote
// autopilot commands. "SKHL".
const HelmInputTag = 0x534B484C

// OpenDisplay opens the platform display. target selects a window (Windows HWND
// or X11 window id, as decimal or 0x hex); empty means the primary screen.
func OpenDisplay(target string) (Display, error) { return openPlatformDisplay(target) }

// ---------------------------------------------------------------- test pattern

// TestPattern is a synthetic display for tests and the development preview. It
// is labelled TEST PATTERN on every frame so it cannot be mistaken for SKAGER,
// draws a mark where each touch landed and records the input it received.
type TestPattern struct {
	mu      sync.Mutex
	w, h    int
	start   time.Time
	frame   int
	x, y    int
	down    map[int]bool
	marks   []image.Point
	Log     []string
	OnInput func(string)
}

func NewTestPattern(w, h int) *TestPattern {
	return &TestPattern{w: w, h: h, start: time.Now(), down: map[int]bool{}}
}

var (
	tpWater  = color.RGBA{0xd5, 0xe5, 0xe5, 0xff}
	tpLand   = color.RGBA{0xee, 0xee, 0xe2, 0xff}
	tpPanel  = color.RGBA{0x15, 0x23, 0x26, 0xff}
	tpText   = color.RGBA{0xf3, 0xf5, 0xee, 0xff}
	tpActive = color.RGBA{0xb6, 0xef, 0xce, 0xff}
	tpMark   = color.RGBA{0xec, 0x8f, 0x87, 0xff}
	tpRoute  = color.RGBA{0x26, 0x7c, 0x76, 0xff}
)

func (t *TestPattern) Capture(dst *image.RGBA) (*image.RGBA, error) {
	t.mu.Lock()
	defer t.mu.Unlock()
	if dst == nil || dst.Rect.Dx() != t.w || dst.Rect.Dy() != t.h {
		dst = image.NewRGBA(image.Rect(0, 0, t.w, t.h))
	}
	t.frame++
	draw.Draw(dst, dst.Rect, &image.Uniform{tpWater}, image.Point{}, draw.Src)
	// A still "coast", a panel and a slowly moving vessel, so that most tiles
	// stay unchanged between frames as on a real chart.
	draw.Draw(dst, image.Rect(0, t.h*3/5, t.w*2/5, t.h), &image.Uniform{tpLand}, image.Point{}, draw.Src)
	draw.Draw(dst, image.Rect(t.w*3/4, 0, t.w, t.h), &image.Uniform{tpPanel}, image.Point{}, draw.Src)
	for i := 0; i < t.w*3/4; i += 6 {
		fillCircle(dst, i, t.h/3+int(20*math.Sin(float64(i)/60)), 2, tpRoute)
	}
	elapsed := time.Since(t.start).Seconds()
	vx := int(float64(t.w*3/4) * (0.5 + 0.4*math.Sin(elapsed/8)))
	fillCircle(dst, vx, t.h/3+int(20*math.Sin(float64(vx)/60)), 12, tpActive)
	label(dst, t.w*3/4+24, 48, "TEST PATTERN", tpActive)
	label(dst, t.w*3/4+24, 72, "Not SKAGER. Helm development", tpText)
	label(dst, t.w*3/4+24, 96, "and test source only.", tpText)
	label(dst, t.w*3/4+24, 140, time.Now().Format("15:04:05"), tpText)
	for _, m := range t.marks {
		fillCircle(dst, m.X, m.Y, 10, tpMark)
	}
	return dst, nil
}

func (t *TestPattern) record(entry string) {
	t.Log = append(t.Log, entry)
	if t.OnInput != nil {
		t.OnInput(entry)
	}
}

func (t *TestPattern) PointerMove(x, y int) error {
	t.mu.Lock()
	defer t.mu.Unlock()
	moved := x != t.x || y != t.y
	t.x, t.y = x, y
	if t.down[0] && moved {
		t.record("drag")
	}
	return nil
}

func (t *TestPattern) PointerButton(button int, down bool) error {
	t.mu.Lock()
	defer t.mu.Unlock()
	t.down[button] = down
	state := "up"
	if down {
		state = "down"
		if len(t.marks) == 16 {
			t.marks = t.marks[1:]
		}
		t.marks = append(t.marks, image.Pt(t.x, t.y))
	}
	t.record(buttonName(button) + " " + state + " " + itoa(t.x) + "," + itoa(t.y))
	return nil
}

func (t *TestPattern) Wheel(n float64) error {
	t.mu.Lock()
	defer t.mu.Unlock()
	t.record("wheel " + itoa(int(math.Round(n))))
	return nil
}

func (t *TestPattern) Key(name string) error {
	t.mu.Lock()
	defer t.mu.Unlock()
	t.record("key " + name)
	return nil
}

func (t *TestPattern) Text(s string) error {
	t.mu.Lock()
	defer t.mu.Unlock()
	t.record("text " + s)
	return nil
}

func (t *TestPattern) Close() error { return nil }

func (t *TestPattern) Entries() []string {
	t.mu.Lock()
	defer t.mu.Unlock()
	return append([]string(nil), t.Log...)
}

func buttonName(b int) string {
	if b == 2 {
		return "secondary"
	}
	return "primary"
}

func itoa(v int) string {
	if v == 0 {
		return "0"
	}
	neg := v < 0
	if neg {
		v = -v
	}
	var b [20]byte
	i := len(b)
	for v > 0 {
		i--
		b[i] = byte('0' + v%10)
		v /= 10
	}
	if neg {
		i--
		b[i] = '-'
	}
	return string(b[i:])
}

func fillCircle(img *image.RGBA, cx, cy, r int, c color.RGBA) {
	for y := -r; y <= r; y++ {
		for x := -r; x <= r; x++ {
			if x*x+y*y <= r*r && image.Pt(cx+x, cy+y).In(img.Rect) {
				img.SetRGBA(cx+x, cy+y, c)
			}
		}
	}
}

func label(img *image.RGBA, x, y int, s string, c color.RGBA) {
	d := font.Drawer{Dst: img, Src: &image.Uniform{c}, Face: basicfont.Face7x13, Dot: fixed.P(x, y)}
	d.DrawString(s)
}
