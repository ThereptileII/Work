package helm

import (
	"bytes"
	"context"
	"encoding/binary"
	"errors"
	"image"
	"image/jpeg"
	"runtime"
	"sync"
	"sync/atomic"
	"time"
)

// Screen stream
//
// SKAGER's screen is mostly still: a chart, instruments and panels that change
// in small regions. Helm therefore sends changed 64-pixel tiles as JPEG over the
// WebRTC "screen" data channel (DTLS, ordered and reliable) instead of a video
// codec. Text stays sharp, nothing is sent when nothing changes, and the stream
// needs no platform encoder. During large changes (panning the chart) tiles go
// at a lower quality and are sent again sharper once the screen is still.
//
// Flow control: at most MaxInFlight frames may be unacknowledged by the phone,
// so a slow link lowers the frame rate instead of building up delay.
//
// Messages (little endian):
//
//	1 begin  frame u32, width u16, height u16, flags u8 (1 = full frame), tiles u16
//	2 tile   frame u32, x u16, y u16, w u16, h u16, JPEG bytes
//	3 end    frame u32, capture ms u16, encode ms u16
const (
	msgBegin = 1
	msgTile  = 2
	msgEnd   = 3

	flagFull = 1
)

type StreamOptions struct {
	Interval      time.Duration // shortest time between frames
	MaxInFlight   uint32
	TileSize      int
	MaxRunTiles   int // tiles merged into one JPEG along a row
	Quality       int
	MotionQuality int
	RefineQuality int
	RefineAfter   time.Duration
	MotionShare   float64 // share of changed tiles that counts as motion
	MaxBuffered   uint64
}

func DefaultStreamOptions() StreamOptions {
	return StreamOptions{Interval: 50 * time.Millisecond, MaxInFlight: 2, TileSize: 64, MaxRunTiles: 8,
		Quality: 82, MotionQuality: 62, RefineQuality: 90, RefineAfter: 600 * time.Millisecond,
		MotionShare: 0.18, MaxBuffered: 2 << 20}
}

// StreamStats are reported to the phone every two seconds.
type StreamStats struct {
	FPS       float64 `json:"fps"`
	Kbps      float64 `json:"kbps"`
	CaptureMs float64 `json:"capture_ms"`
	EncodeMs  float64 `json:"encode_ms"`
	Width     int     `json:"width"`
	Height    int     `json:"height"`
}

type Streamer struct {
	display  Display
	send     func([]byte) error
	buffered func() uint64
	onState  func(available bool)
	onStats  func(StreamStats)
	opts     StreamOptions

	frame    uint32
	size     atomic.Uint64 // width<<32 | height of the last frame sent
	acked    atomic.Uint32
	refresh  atomic.Bool
	wake     chan struct{}
	lowq     []bool
	lastMove time.Time
}

func NewStreamer(display Display, send func([]byte) error, buffered func() uint64, opts StreamOptions) *Streamer {
	return &Streamer{display: display, send: send, buffered: buffered, opts: opts, wake: make(chan struct{}, 1)}
}

func (s *Streamer) OnState(f func(available bool)) { s.onState = f }
func (s *Streamer) OnStats(f func(StreamStats))    { s.onStats = f }

// Ack records that the phone has drawn frame id.
func (s *Streamer) Ack(id uint32) {
	for {
		cur := s.acked.Load()
		if int32(id-cur) <= 0 || int32(id-s.currentFrame()) > 0 || s.acked.CompareAndSwap(cur, id) {
			break
		}
	}
	s.poke()
}

// Size is the width and height of the frame the phone is showing.
func (s *Streamer) Size() (int, int) {
	v := s.size.Load()
	return int(v >> 32), int(v & 0xffffffff)
}

func (s *Streamer) currentFrame() uint32 { return atomic.LoadUint32(&s.frame) }

// Refresh asks for a complete frame (new connection, phone redraw).
func (s *Streamer) Refresh() { s.refresh.Store(true); s.poke() }

func (s *Streamer) poke() {
	select {
	case s.wake <- struct{}{}:
	default:
	}
}

func (s *Streamer) Run(ctx context.Context) error {
	var cur, prev *image.RGBA
	available := true
	stats := struct {
		frames, bytes   int
		capture, encode time.Duration
		since           time.Time
	}{since: time.Now()}
	s.refresh.Store(true)
	ticker := time.NewTicker(s.opts.Interval)
	defer ticker.Stop()
	for {
		select {
		case <-ctx.Done():
			return ctx.Err()
		case <-ticker.C:
		case <-s.wake:
		}
		if s.onStats != nil && time.Since(stats.since) >= 2*time.Second {
			secs := time.Since(stats.since).Seconds()
			st := StreamStats{FPS: float64(stats.frames) / secs, Kbps: float64(stats.bytes) * 8 / 1000 / secs}
			if stats.frames > 0 {
				st.CaptureMs = float64(stats.capture.Milliseconds()) / float64(stats.frames)
				st.EncodeMs = float64(stats.encode.Milliseconds()) / float64(stats.frames)
			}
			if prev != nil {
				st.Width, st.Height = prev.Rect.Dx(), prev.Rect.Dy()
			}
			s.onStats(st)
			stats.frames, stats.bytes, stats.capture, stats.encode, stats.since = 0, 0, 0, 0, time.Now()
		}
		if s.currentFrame()-s.acked.Load() >= s.opts.MaxInFlight {
			continue
		}
		if s.buffered != nil && s.buffered() > s.opts.MaxBuffered {
			continue
		}
		t0 := time.Now()
		next, err := s.display.Capture(cur)
		if err != nil {
			if errors.Is(err, ErrSurfaceUnavailable) {
				if available && s.onState != nil {
					s.onState(false)
				}
				available = false
				continue
			}
			return err
		}
		if !available {
			available = true
			if s.onState != nil {
				s.onState(true)
			}
			s.refresh.Store(true)
		}
		cur = next
		captured := time.Since(t0)
		full := s.refresh.Swap(false) || prev == nil || prev.Rect != cur.Rect
		rects, quality := s.plan(cur, prev, full)
		if len(rects) == 0 {
			continue
		}
		t1 := time.Now()
		tiles, err := encodeTiles(cur, rects, quality)
		if err != nil {
			return err
		}
		encoded := time.Since(t1)
		n, err := s.emit(cur, tiles, full, captured, encoded)
		if err != nil {
			return err
		}
		stats.frames++
		stats.bytes += n
		stats.capture += captured
		stats.encode += encoded
		prev, cur = cur, prev
	}
}

// plan chooses the regions to send and their JPEG quality.
func (s *Streamer) plan(cur, prev *image.RGBA, full bool) ([]image.Rectangle, []int) {
	ts := s.opts.TileSize
	cols, rows := (cur.Rect.Dx()+ts-1)/ts, (cur.Rect.Dy()+ts-1)/ts
	if len(s.lowq) != cols*rows || full {
		s.lowq = make([]bool, cols*rows)
	}
	dirty := make([]bool, cols*rows)
	count := 0
	for ty := 0; ty < rows; ty++ {
		for tx := 0; tx < cols; tx++ {
			if full || tileChanged(cur, prev, tx*ts, ty*ts, ts) {
				dirty[ty*cols+tx] = true
				count++
			}
		}
	}
	q := s.opts.Quality
	now := time.Now()
	switch {
	case full:
	case count == 0:
		// Still screen: resend tiles sent during motion at full sharpness.
		if now.Sub(s.lastMove) < s.opts.RefineAfter {
			return nil, nil
		}
		for i, low := range s.lowq {
			if low {
				dirty[i], s.lowq[i] = true, false
				count++
			}
		}
		if count == 0 {
			return nil, nil
		}
		q = s.opts.RefineQuality
	case float64(count) >= s.opts.MotionShare*float64(cols*rows):
		q = s.opts.MotionQuality
		s.lastMove = now
		for i, d := range dirty {
			if d {
				s.lowq[i] = true
			}
		}
	default:
		s.lastMove = now
		for i, d := range dirty {
			if d {
				s.lowq[i] = false
			}
		}
	}
	var rects []image.Rectangle
	for ty := 0; ty < rows; ty++ {
		for tx := 0; tx < cols; {
			if !dirty[ty*cols+tx] {
				tx++
				continue
			}
			start := tx
			for tx < cols && dirty[ty*cols+tx] && tx-start < s.opts.MaxRunTiles {
				tx++
			}
			rects = append(rects, image.Rect(start*ts, ty*ts, tx*ts, (ty+1)*ts).Intersect(cur.Rect))
		}
	}
	qs := make([]int, len(rects))
	for i := range qs {
		qs[i] = q
	}
	return rects, qs
}

func tileChanged(cur, prev *image.RGBA, x, y, ts int) bool {
	w := min(ts, cur.Rect.Dx()-x) * 4
	for row := y; row < min(y+ts, cur.Rect.Dy()); row++ {
		o := row*cur.Stride + x*4
		if !bytes.Equal(cur.Pix[o:o+w], prev.Pix[o:o+w]) {
			return true
		}
	}
	return false
}

type encodedTile struct {
	rect image.Rectangle
	data []byte
}

func encodeTiles(img *image.RGBA, rects []image.Rectangle, quality []int) ([]encodedTile, error) {
	out := make([]encodedTile, len(rects))
	errs := make([]error, len(rects))
	var wg sync.WaitGroup
	next := atomic.Int64{}
	workers := min(runtime.GOMAXPROCS(0), len(rects))
	for w := 0; w < workers; w++ {
		wg.Add(1)
		go func() {
			defer wg.Done()
			for {
				i := int(next.Add(1) - 1)
				if i >= len(rects) {
					return
				}
				var buf bytes.Buffer
				errs[i] = jpeg.Encode(&buf, img.SubImage(rects[i]), &jpeg.Options{Quality: quality[i]})
				out[i] = encodedTile{rect: rects[i], data: buf.Bytes()}
			}
		}()
	}
	wg.Wait()
	for _, err := range errs {
		if err != nil {
			return nil, err
		}
	}
	return out, nil
}

func (s *Streamer) emit(img *image.RGBA, tiles []encodedTile, full bool, captured, encoded time.Duration) (int, error) {
	id := atomic.AddUint32(&s.frame, 1)
	s.size.Store(uint64(img.Rect.Dx())<<32 | uint64(img.Rect.Dy()))
	var flags byte
	if full {
		flags = flagFull
	}
	begin := make([]byte, 12)
	begin[0] = msgBegin
	binary.LittleEndian.PutUint32(begin[1:], id)
	binary.LittleEndian.PutUint16(begin[5:], uint16(img.Rect.Dx()))
	binary.LittleEndian.PutUint16(begin[7:], uint16(img.Rect.Dy()))
	begin[9] = flags
	binary.LittleEndian.PutUint16(begin[10:], uint16(len(tiles)))
	if err := s.send(begin); err != nil {
		return 0, err
	}
	total := len(begin)
	for _, t := range tiles {
		msg := make([]byte, 13+len(t.data))
		msg[0] = msgTile
		binary.LittleEndian.PutUint32(msg[1:], id)
		binary.LittleEndian.PutUint16(msg[5:], uint16(t.rect.Min.X))
		binary.LittleEndian.PutUint16(msg[7:], uint16(t.rect.Min.Y))
		binary.LittleEndian.PutUint16(msg[9:], uint16(t.rect.Dx()))
		binary.LittleEndian.PutUint16(msg[11:], uint16(t.rect.Dy()))
		copy(msg[13:], t.data)
		if err := s.send(msg); err != nil {
			return 0, err
		}
		total += len(msg)
	}
	end := make([]byte, 9)
	end[0] = msgEnd
	binary.LittleEndian.PutUint32(end[1:], id)
	binary.LittleEndian.PutUint16(end[5:], uint16(min(captured.Milliseconds(), 65535)))
	binary.LittleEndian.PutUint16(end[7:], uint16(min(encoded.Milliseconds(), 65535)))
	if err := s.send(end); err != nil {
		return 0, err
	}
	return total + len(end), nil
}
