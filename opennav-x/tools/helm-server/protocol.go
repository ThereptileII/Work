package helm

import (
	"errors"
	"fmt"
	"math"
	"strings"
	"unicode/utf8"
)

var errInvalidInput = errors.New("invalid Helm input event")

// InputEvent is the only input shape accepted from a Helm client. Coordinates
// are normalized to the frame the client is showing; no native screen
// coordinates cross the network boundary.
type InputEvent struct {
	Type      string  `json:"type"`
	X         float64 `json:"x,omitempty"`
	Y         float64 `json:"y,omitempty"`
	Button    int     `json:"button,omitempty"` // 0 primary, 2 secondary
	Delta     float64 `json:"delta,omitempty"`  // wheel notches, positive = away from the user (zoom in)
	Key       string  `json:"key,omitempty"`
	Text      string  `json:"text,omitempty"`
	Sequence  uint64  `json:"sequence"`
	SentAt    float64 `json:"t,omitempty"` // client monotonic milliseconds, for the staleness check
	PointerID int     `json:"pointer_id,omitempty"`
}

// Keys a client may press. Anything else is refused: Helm types into SKAGER's
// fields, it is not a general remote keyboard (no modifiers, no shortcuts).
var allowedKeys = map[string]bool{
	"Enter": true, "Escape": true, "Backspace": true, "Delete": true, "Tab": true,
	"ArrowUp": true, "ArrowDown": true, "ArrowLeft": true, "ArrowRight": true,
	"Home": true, "End": true, "PageUp": true, "PageDown": true,
}

func (e InputEvent) Validate() error {
	switch e.Type {
	case "pointer_down", "pointer_up", "pointer_move":
		if !finiteUnit(e.X) || !finiteUnit(e.Y) || e.PointerID < 0 || (e.Button != 0 && e.Button != 2) {
			return errInvalidInput
		}
	case "wheel":
		if !finiteUnit(e.X) || !finiteUnit(e.Y) || !finiteNumber(e.Delta) || e.Delta == 0 || math.Abs(e.Delta) > 10 {
			return errInvalidInput
		}
	case "key":
		if !allowedKeys[e.Key] {
			return errInvalidInput
		}
	case "text":
		if len(e.Text) == 0 || len(e.Text) > 256 || !utf8.ValidString(e.Text) || strings.ContainsFunc(e.Text, isControl) {
			return errInvalidInput
		}
	default:
		return errInvalidInput
	}
	if e.Sequence == 0 || !finiteNumber(e.SentAt) || e.SentAt < 0 {
		return errInvalidInput
	}
	return nil
}

func isControl(r rune) bool { return r < 0x20 || r == 0x7f || (r >= 0x80 && r < 0xa0) }

func finiteNumber(v float64) bool { return !math.IsNaN(v) && !math.IsInf(v, 0) }
func finiteUnit(v float64) bool   { return finiteNumber(v) && v >= 0 && v <= 1 }

func (e InputEvent) String() string { return fmt.Sprintf("%s/%d", e.Type, e.Sequence) }

// InputCoordinate maps a normalized coordinate onto a frame of width x height pixels.
func InputCoordinate(x, y float64, width, height int) (int, int, error) {
	if width <= 0 || height <= 0 || !finiteUnit(x) || !finiteUnit(y) {
		return 0, 0, errors.New("invalid normalized Helm coordinate")
	}
	return int(math.Round(x * float64(width-1))), int(math.Round(y * float64(height-1))), nil
}
