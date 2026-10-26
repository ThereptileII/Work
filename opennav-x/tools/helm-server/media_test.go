package helm

import (
	"testing"
	"time"
)

func TestDefaultMediaPolicyAndCoordinateMapping(t *testing.T) {
	p := DefaultMediaPolicy()
	if err := p.Validate(); err != nil {
		t.Fatal(err)
	}
	x, y, err := InputCoordinate(.5, .5, p.Width, p.Height)
	if err != nil {
		t.Fatal(err)
	}
	if x != 640 || y != 400 {
		t.Fatalf("unexpected mapped center %d,%d", x, y)
	}
	if _, _, err := InputCoordinate(.5, 1.1, p.Width, p.Height); err == nil {
		t.Fatal("invalid coordinate accepted")
	}
	if (FrameObservation{FrameID: 1, CapturedAt: time.Unix(1, 0), EncodedAt: time.Unix(1, 1), SentAt: time.Unix(1, 2), Width: 1280, Height: 800}).Valid() != true {
		t.Fatal("valid frame rejected")
	}
}
