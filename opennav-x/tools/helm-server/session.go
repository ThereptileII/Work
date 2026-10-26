package helm

import (
	"context"
	"encoding/json"
	"sync"
	"time"

	"github.com/pion/webrtc/v4"
)

// Input older than this when it reaches the PC is dropped: a touch must never
// land seconds late on a screen that has changed in the meantime. A release of a
// button this session pressed is always delivered.
const staleInput = 400 * time.Millisecond

// Session is one connected phone. Only one session is active at a time.
type Session struct {
	server   *Server
	device   Device
	pc       *webrtc.PeerConnection
	ctx      context.Context
	cancel   context.CancelFunc
	started  time.Time
	streamer *Streamer

	mu         sync.Mutex
	control    *webrtc.DataChannel
	input      *webrtc.DataChannel
	connected  bool
	ended      bool
	pressed    map[int]bool
	suppressed bool
	lastSeq    uint64
	minOffset  float64
	haveOffset bool
	rate       bucket
	lostTimer  *time.Timer
	closed     chan struct{} // closed once the peer connection is closed
}

func newSession(s *Server, device Device, pc *webrtc.PeerConnection) *Session {
	ctx, cancel := context.WithCancel(context.Background())
	sess := &Session{server: s, device: device, pc: pc, ctx: ctx, cancel: cancel, started: time.Now(),
		pressed: map[int]bool{}, rate: bucket{tokens: 200, at: time.Now()}, closed: make(chan struct{})}
	pc.OnConnectionStateChange(sess.onState)
	pc.OnDataChannel(func(dc *webrtc.DataChannel) {
		switch dc.Label() {
		case "screen":
			dc.OnOpen(func() { sess.startStream(dc) })
		case "input":
			sess.mu.Lock()
			sess.input = dc
			sess.mu.Unlock()
			dc.OnMessage(func(m webrtc.DataChannelMessage) { sess.onInput(m.Data) })
		case "control":
			sess.mu.Lock()
			sess.control = dc
			sess.mu.Unlock()
			dc.OnOpen(sess.hello)
			dc.OnMessage(func(m webrtc.DataChannelMessage) { sess.onControl(m.Data) })
		default:
			_ = dc.Close()
		}
	})
	// A phone that never completes the connection does not hold the PC.
	time.AfterFunc(20*time.Second, func() {
		sess.mu.Lock()
		connected := sess.connected
		sess.mu.Unlock()
		if !connected {
			sess.End("connect_timeout", "")
		}
	})
	return sess
}

func (s *Session) Active() bool {
	s.mu.Lock()
	defer s.mu.Unlock()
	return !s.ended
}

func (s *Session) onState(state webrtc.PeerConnectionState) {
	switch state {
	case webrtc.PeerConnectionStateConnected:
		s.mu.Lock()
		first := !s.connected
		s.connected = true
		if s.lostTimer != nil {
			s.lostTimer.Stop()
			s.lostTimer = nil
		}
		s.mu.Unlock()
		if first {
			info := s.device.Info()
			s.server.cfg.Logf("%q connected", s.device.Name)
			s.server.cfg.Events(Event{Event: "session", State: "connected", Device: &info})
			s.server.emitDevices()
		}
	case webrtc.PeerConnectionStateDisconnected:
		// Input stops immediately; the session ends unless the link recovers.
		s.mu.Lock()
		s.connected = false
		s.releaseLocked()
		if s.lostTimer == nil {
			s.lostTimer = time.AfterFunc(8*time.Second, func() { s.End("connection_lost", "") })
		}
		s.mu.Unlock()
	case webrtc.PeerConnectionStateFailed, webrtc.PeerConnectionStateClosed:
		s.End("connection_lost", "")
	}
}

func (s *Session) hello() {
	caps := []string{"screen-tiles", "touch-input", "keyboard-input", "autopilot-status"}
	msg := map[string]any{"type": "hello", "name": s.server.Name(), "server_id": s.server.cfg.Store.ServerID(),
		"software": Version, "capabilities": caps}
	if p := s.server.pilotStatus(); p != nil {
		msg["pilot"] = p
	}
	s.sendControl(msg)
}

func (s *Session) startStream(dc *webrtc.DataChannel) {
	dc.SetBufferedAmountLowThreshold(256 << 10)
	st := NewStreamer(s.server.cfg.Display, dc.Send, dc.BufferedAmount, s.server.cfg.Stream)
	st.OnState(func(available bool) {
		s.sendControl(map[string]any{"type": "screen", "available": available})
		s.server.cfg.Events(Event{Event: "screen", Available: &available})
	})
	st.OnStats(func(stats StreamStats) {
		s.sendControl(map[string]any{"type": "stats", "stats": stats})
	})
	dc.OnBufferedAmountLow(st.poke)
	s.mu.Lock()
	s.streamer = st
	s.mu.Unlock()
	go func() {
		if err := st.Run(s.ctx); err != nil && s.ctx.Err() == nil {
			s.server.cfg.Logf("screen stream stopped: %v", err)
			s.End("stream_error", "")
		}
	}()
}

type controlMessage struct {
	Type  string  `json:"type"`
	Frame uint32  `json:"frame"`
	T     float64 `json:"t"`
}

func (s *Session) onControl(data []byte) {
	var m controlMessage
	if len(data) > 4096 || json.Unmarshal(data, &m) != nil {
		return
	}
	s.mu.Lock()
	st := s.streamer
	s.mu.Unlock()
	switch m.Type {
	case "ack":
		if st != nil {
			st.Ack(m.Frame)
		}
	case "refresh":
		if st != nil {
			st.Refresh()
		}
	case "ping":
		s.sendControl(map[string]any{"type": "pong", "t": m.T})
	case "bye":
		s.End("disconnected_by_phone", "")
	}
}

func (s *Session) sendControl(v any) {
	s.mu.Lock()
	dc := s.control
	s.mu.Unlock()
	if dc == nil || dc.ReadyState() != webrtc.DataChannelStateOpen {
		return
	}
	if data, err := json.Marshal(v); err == nil {
		_ = dc.SendText(string(data))
	}
}

type inputEnvelope struct {
	Type  string     `json:"type"`
	Event InputEvent `json:"event"`
}

type inputAck struct {
	Type     string `json:"type"`
	Sequence uint64 `json:"sequence"`
	Accepted bool   `json:"accepted"`
	Reason   string `json:"reason,omitempty"`
}

func (s *Session) ack(seq uint64, reason string) {
	s.mu.Lock()
	dc := s.input
	s.mu.Unlock()
	if dc == nil || dc.ReadyState() != webrtc.DataChannelStateOpen {
		return
	}
	data, _ := json.Marshal(inputAck{Type: "input_ack", Sequence: seq, Accepted: reason == "", Reason: reason})
	_ = dc.SendText(string(data))
}

func (s *Session) since() float64 { return float64(time.Since(s.started).Microseconds()) / 1000 }

func (s *Session) onInput(data []byte) {
	var env inputEnvelope
	if len(data) > 2048 || json.Unmarshal(data, &env) != nil || env.Type != "input" {
		s.ack(0, "malformed")
		return
	}
	ev := env.Event
	if err := ev.Validate(); err != nil {
		s.ack(ev.Sequence, "invalid")
		return
	}
	reason := s.deliver(ev)
	s.ack(ev.Sequence, reason)
}

// deliver applies one input event and returns why it was not applied ("" = applied).
func (s *Session) deliver(ev InputEvent) string {
	s.mu.Lock()
	defer s.mu.Unlock()
	if s.ended || !s.connected {
		return "disconnected"
	}
	if ev.Sequence <= s.lastSeq {
		return "out_of_order"
	}
	s.lastSeq = ev.Sequence
	now := time.Now()
	s.rate.tokens = min(200, s.rate.tokens+now.Sub(s.rate.at).Seconds()*200)
	s.rate.at = now
	if s.rate.tokens < 1 && ev.Type != "pointer_up" {
		return "rate_limited"
	}
	s.rate.tokens--
	// Staleness: the lowest (arrival - sent) seen in this session is the clock
	// offset plus the fastest delivery; anything much slower than that is late.
	offset := s.since() - ev.SentAt
	if !s.haveOffset || offset < s.minOffset {
		s.minOffset, s.haveOffset = offset, true
	}
	stale := time.Duration((offset-s.minOffset)*1000)*time.Microsecond > staleInput
	var w, h int
	if s.streamer != nil {
		w, h = s.streamer.Size()
	}
	d := s.server.cfg.Display
	at := func() bool {
		if w <= 0 || h <= 0 {
			return false
		}
		x, y, err := InputCoordinate(ev.X, ev.Y, w, h)
		return err == nil && d.PointerMove(x, y) == nil
	}
	switch ev.Type {
	case "pointer_down":
		if stale {
			s.suppressed = true
			return "stale"
		}
		if len(s.pressed) > 0 {
			return "busy"
		}
		s.suppressed = false
		s.notifyInput()
		if !at() {
			return "no_screen"
		}
		if err := d.PointerButton(ev.Button, true); err != nil {
			return "failed"
		}
		s.pressed[ev.Button] = true
	case "pointer_move":
		if s.suppressed || stale {
			return "stale"
		}
		if !at() {
			return "no_screen"
		}
	case "pointer_up":
		s.suppressed = false
		if !s.pressed[ev.Button] {
			return "not_pressed"
		}
		if !stale {
			at()
		}
		delete(s.pressed, ev.Button)
		if err := d.PointerButton(ev.Button, false); err != nil {
			return "failed"
		}
	case "wheel":
		if stale {
			return "stale"
		}
		s.notifyInput()
		if !at() {
			return "no_screen"
		}
		if err := d.Wheel(ev.Delta); err != nil {
			return "failed"
		}
	case "key":
		if stale {
			return "stale"
		}
		s.notifyInput()
		if err := d.Key(ev.Key); err != nil {
			return "failed"
		}
	case "text":
		if stale {
			return "stale"
		}
		s.notifyInput()
		if err := d.Text(ev.Text); err != nil {
			return "failed"
		}
	}
	return ""
}

// notifyInput tells SKAGER, before the event is injected, that the next input
// comes from Helm (used to keep autopilot commands local to the helm).
func (s *Session) notifyInput() { s.server.cfg.Events(Event{Event: "input"}) }

// releaseLocked lifts any button this session holds down. Caller holds s.mu.
func (s *Session) releaseLocked() {
	for b := range s.pressed {
		_ = s.server.cfg.Display.PointerButton(b, false)
		delete(s.pressed, b)
	}
	s.suppressed = false
}

// End closes the session: input stops, held buttons are released and the phone
// is told why.
func (s *Session) End(reason, by string) {
	s.mu.Lock()
	if s.ended {
		s.mu.Unlock()
		return
	}
	s.ended = true
	s.releaseLocked()
	if s.lostTimer != nil {
		s.lostTimer.Stop()
	}
	s.mu.Unlock()
	msg := map[string]any{"type": "bye", "reason": reason}
	if by != "" {
		msg["by"] = by
	}
	s.mu.Lock()
	dc := s.control
	s.mu.Unlock()
	if dc != nil && dc.ReadyState() == webrtc.DataChannelStateOpen {
		if data, err := json.Marshal(msg); err == nil {
			_ = dc.SendText(string(data))
		}
	}
	s.cancel()
	go func() {
		time.Sleep(200 * time.Millisecond) // let the goodbye leave before closing
		_ = s.pc.Close()
		close(s.closed)
	}()
	s.server.cfg.Logf("%q session ended: %s", s.device.Name, reason)
	s.server.sessionEnded(s, reason)
}
