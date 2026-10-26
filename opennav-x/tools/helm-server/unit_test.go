package helm

import (
	"context"
	"image"
	"image/color"
	"os"
	"path/filepath"
	"testing"
	"time"
)

func TestInputValidation(t *testing.T) {
	ok := []InputEvent{
		{Type: "pointer_down", X: 0, Y: 1, Sequence: 1},
		{Type: "pointer_up", X: .5, Y: .5, Button: 2, Sequence: 2},
		{Type: "wheel", X: .5, Y: .5, Delta: -1, Sequence: 3},
		{Type: "key", Key: "Enter", Sequence: 4},
		{Type: "text", Text: "Ölandsbron 12", Sequence: 5},
	}
	for _, e := range ok {
		if err := e.Validate(); err != nil {
			t.Errorf("%v refused", e)
		}
	}
	bad := []InputEvent{
		{Type: "pointer_down", X: 1.1, Y: .5, Sequence: 1},
		{Type: "pointer_down", X: .5, Y: .5, Button: 1, Sequence: 1},
		{Type: "pointer_down", X: .5, Y: .5},
		{Type: "wheel", X: .5, Y: .5, Delta: 0, Sequence: 1},
		{Type: "key", Key: "F4", Sequence: 1},
		{Type: "key", Key: "Alt", Sequence: 1},
		{Type: "text", Text: "a\nb", Sequence: 1},
		{Type: "text", Text: "", Sequence: 1},
		{Type: "autopilot", Sequence: 1},
		{Type: "pointer_down", X: .5, Y: .5, Sequence: 1, SentAt: -1},
	}
	for _, e := range bad {
		if err := e.Validate(); err == nil {
			t.Errorf("%+v accepted", e)
		}
	}
	if x, y, err := InputCoordinate(.5, .5, 1281, 801); err != nil || x != 640 || y != 400 {
		t.Fatalf("mapping: %d %d %v", x, y, err)
	}
}

func TestPairingLinkAndKey(t *testing.T) {
	link := PairingLink("srv", "Aurora & co", []string{"192.168.1.20:43872", "10.0.0.5:43872"}, "MFRGGZDFMZTWQ2LK")
	p, err := ParsePairingLink(link)
	if err != nil || p.ServerID != "srv" || p.Name != "Aurora & co" || len(p.Addresses) != 2 || len(p.Secret) != 10 {
		t.Fatalf("%+v %v", p, err)
	}
	for _, typed := range []string{"MFRG-GZDF-MZTW-Q2LK", "mfrg gzdf mztw q2lk"} {
		if b, err := ParsePairingKey(typed); err != nil || string(b) != string(p.Secret) {
			t.Fatalf("%q: %v", typed, err)
		}
	}
	if _, err := ParsePairingKey("MFRG-GZDF"); err == nil {
		t.Fatal("short key accepted")
	}
	if _, err := ParsePairingLink("https://example.com/pair?k=x"); err == nil {
		t.Fatal("foreign link accepted")
	}
	qr, err := EncodeQR(link)
	if err != nil || qr.Size < 25 || len(qr.Rows) != qr.Size {
		t.Fatalf("qr: %d %v", qr.Size, err)
	}
	if groupKey("ABCDEFGHIJKLMNOP") != "ABCD-EFGH-IJKL-MNOP" {
		t.Fatal(groupKey("ABCDEFGHIJKLMNOP"))
	}
}

func TestStorePersistsDevicesPrivately(t *testing.T) {
	path := filepath.Join(t.TempDir(), "helm", "helm-devices.json")
	s, err := OpenStore(path)
	if err != nil {
		t.Fatal(err)
	}
	s.devices["d1"] = Device{ID: "d1", Name: "Phone", Key: make([]byte, 32), PairedAt: time.Now()}
	if err := s.save(); err != nil {
		t.Fatal(err)
	}
	if info, err := os.Stat(path); err != nil || info.Mode().Perm() != 0o600 {
		t.Fatalf("store file permissions: %v %v", info.Mode(), err)
	}
	again, err := OpenStore(path)
	if err != nil || again.ServerID() != s.ServerID() || len(again.Devices()) != 1 {
		t.Fatalf("reload: %v", err)
	}
	if !again.Revoke("d1") || len(again.Devices()) != 0 {
		t.Fatal("revoke")
	}
	if err := os.WriteFile(path, []byte("{"), 0o600); err != nil {
		t.Fatal(err)
	}
	if _, err := OpenStore(path); err == nil {
		t.Fatal("corrupt store accepted")
	}
}

// still is a display whose content only changes when the test says so.
type still struct {
	*TestPattern
	img *image.RGBA
}

func (s *still) Capture(dst *image.RGBA) (*image.RGBA, error) {
	if dst == nil || dst.Rect != s.img.Rect {
		dst = image.NewRGBA(s.img.Rect)
	}
	copy(dst.Pix, s.img.Pix)
	return dst, nil
}

func TestStreamPlanSendsChangesThenRefines(t *testing.T) {
	img := image.NewRGBA(image.Rect(0, 0, 256, 128)) // 4 x 2 tiles
	s := NewStreamer(&still{img: img}, nil, nil, DefaultStreamOptions())
	prev := image.NewRGBA(img.Rect)
	copy(prev.Pix, img.Pix)
	rects, _ := s.plan(img, nil, true)
	if len(rects) != 2 || rects[0] != image.Rect(0, 0, 256, 64) {
		t.Fatalf("full frame runs: %v", rects)
	}
	if rects, _ := s.plan(img, prev, false); len(rects) != 0 {
		t.Fatalf("unchanged screen sent %v", rects)
	}
	img.SetRGBA(70, 10, color.RGBA{1, 2, 3, 255})
	rects, q := s.plan(img, prev, false)
	if len(rects) != 1 || rects[0] != image.Rect(64, 0, 128, 64) || q[0] != s.opts.Quality {
		t.Fatalf("one changed tile: %v %v", rects, q)
	}
	// Motion: most tiles change and go at motion quality, then sharpen when still.
	for x := 0; x < 256; x += 64 {
		img.SetRGBA(x+1, 70, color.RGBA{9, 9, 9, 255})
	}
	rects, q = s.plan(img, prev, false)
	if q[0] != s.opts.MotionQuality {
		t.Fatalf("motion quality: %v", q)
	}
	copy(prev.Pix, img.Pix)
	if rects, _ := s.plan(img, prev, false); len(rects) != 0 {
		t.Fatal("refined too early")
	}
	s.lastMove = time.Now().Add(-time.Second)
	rects, q = s.plan(img, prev, false)
	if len(rects) == 0 || q[0] != s.opts.RefineQuality {
		t.Fatalf("refine: %v %v", rects, q)
	}
	if rects, _ := s.plan(img, prev, false); len(rects) != 0 {
		t.Fatal("refined twice")
	}
}

func TestStreamFlowControl(t *testing.T) {
	tp := NewTestPattern(320, 240)
	var sent int
	s := NewStreamer(tp, func([]byte) error { sent++; return nil }, nil, DefaultStreamOptions())
	s.opts.Interval = 5 * time.Millisecond
	ctx, cancel := context.WithTimeout(context.Background(), 300*time.Millisecond)
	defer cancel()
	_ = s.Run(ctx)
	if f := s.currentFrame(); f != s.opts.MaxInFlight {
		t.Fatalf("sent %d frames without acknowledgement, want %d", f, s.opts.MaxInFlight)
	}
	s.Ack(1)
	s.Ack(99) // not yet sent: ignored
	if s.acked.Load() != 1 {
		t.Fatalf("acked %d", s.acked.Load())
	}
}
