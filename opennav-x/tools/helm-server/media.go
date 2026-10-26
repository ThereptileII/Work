package helm

import (
	"errors"
	"fmt"
	"math"
	"time"
)

// MediaPolicy is deliberately separate from the SKAGER renderer. It describes
// the bounded WebRTC output contract; it does not change chart or navigation
// semantics.
type MediaPolicy struct {
	Width           int
	Height          int
	TargetFPS       int
	MaxBitrateKbps  int
	TargetLatencyMs int
	Codec           string
}

func DefaultMediaPolicy() MediaPolicy {
	return MediaPolicy{Width: 1280, Height: 800, TargetFPS: 60,
		MaxBitrateKbps: 6000, TargetLatencyMs: 100, Codec: "H264"}
}

func (p MediaPolicy) Validate() error {
	if p.Width < 320 || p.Width > 7680 || p.Height < 240 || p.Height > 4320 {
		return errors.New("WebRTC frame dimensions outside safe bounds")
	}
	if p.TargetFPS < 1 || p.TargetFPS > 120 {
		return errors.New("WebRTC frame rate outside safe bounds")
	}
	if p.MaxBitrateKbps < 128 || p.MaxBitrateKbps > 50000 {
		return errors.New("WebRTC bitrate outside safe bounds")
	}
	if p.TargetLatencyMs < 20 || p.TargetLatencyMs > 1000 {
		return errors.New("WebRTC latency target outside safe bounds")
	}
	if p.Codec != "H264" {
		return fmt.Errorf("unsupported Helm codec %q", p.Codec)
	}
	return nil
}

type FrameObservation struct {
	FrameID       uint64
	CapturedAt    time.Time
	EncodedAt     time.Time
	SentAt        time.Time
	Width         int
	Height        int
	KeyFrame      bool
	EncodeLatency time.Duration
}

func (f FrameObservation) Valid() bool {
	return f.FrameID > 0 && !f.CapturedAt.IsZero() && !f.EncodedAt.IsZero() &&
		!f.SentAt.IsZero() && f.Width > 0 && f.Height > 0 &&
		f.EncodeLatency >= 0 && f.EncodeLatency < 10*time.Second
}

func InputCoordinate(x, y float64, width, height int) (int, int, error) {
	if width <= 0 || height <= 0 || math.IsNaN(x) || math.IsNaN(y) ||
		math.IsInf(x, 0) || math.IsInf(y, 0) || x < 0 || x > 1 || y < 0 || y > 1 {
		return 0, 0, errors.New("invalid normalized Helm coordinate")
	}
	return int(math.Round(x * float64(width-1))), int(math.Round(y * float64(height-1))), nil
}
