package helm

import (
	"bytes"
	"context"
	"crypto/rand"
	"encoding/binary"
	"encoding/json"
	"image/jpeg"
	"net"
	"net/http"
	"strings"
	"sync"
	"testing"
	"time"

	"github.com/pion/webrtc/v4"
)

// testClient is a minimal Helm client (the protocol the app implements).
type testClient struct {
	t        *testing.T
	base     string
	deviceID string
	key      []byte
}

func nonce() string {
	b := make([]byte, 16)
	_, _ = rand.Read(b)
	return b64(b)
}

func postJSON(t *testing.T, url string, body any, out any) int {
	t.Helper()
	data, _ := json.Marshal(body)
	resp, err := http.Post(url, "application/json", bytes.NewReader(data))
	if err != nil {
		t.Fatal(err)
	}
	defer resp.Body.Close()
	if out != nil {
		_ = json.NewDecoder(resp.Body).Decode(out)
	}
	return resp.StatusCode
}

func pair(t *testing.T, base string, link string, name string) (*testClient, int) {
	t.Helper()
	parsed, err := ParsePairingLink(link)
	if err != nil {
		t.Fatal(err)
	}
	cn := nonce()
	req := PairRequest{Version: 1, DeviceName: name, ClientNonce: cn,
		Proof: MAC(parsed.Secret, pairMessage(parsed.ServerID, cn, name))}
	var resp PairResponse
	status := postJSON(t, base+"/helm/v1/pair", req, &resp)
	if status != 200 {
		return nil, status
	}
	cnb, snb := decodeB64(cn), decodeB64(resp.ServerNonce)
	key, err := DeviceKey(parsed.Secret, cnb, snb, parsed.ServerID, resp.DeviceID)
	if err != nil {
		t.Fatal(err)
	}
	if MAC(key, pairedMessage(resp.DeviceID, cn, resp.ServerNonce)) != resp.Proof {
		t.Fatal("server pairing proof did not verify")
	}
	return &testClient{t: t, base: base, deviceID: resp.DeviceID, key: key}, status
}

func (c *testClient) challenge() string {
	resp, err := http.Get(c.base + "/helm/v1/challenge")
	if err != nil {
		c.t.Fatal(err)
	}
	defer resp.Body.Close()
	var out map[string]string
	_ = json.NewDecoder(resp.Body).Decode(&out)
	return out["challenge"]
}

type peer struct {
	pc      *webrtc.PeerConnection
	screen  *webrtc.DataChannel
	input   *webrtc.DataChannel
	control *webrtc.DataChannel
	mu      sync.Mutex
	frames  []frameSeen
	tiles   int
	acks    map[uint64]string
	ctrl    []map[string]any
	opened  chan struct{}
	seq     uint64
	start   time.Time
}

type frameSeen struct {
	id   uint32
	w, h int
	full bool
}

func newPeer(t *testing.T) *peer {
	pc, err := webrtc.NewPeerConnection(webrtc.Configuration{})
	if err != nil {
		t.Fatal(err)
	}
	p := &peer{pc: pc, acks: map[uint64]string{}, opened: make(chan struct{}), start: time.Now().Add(-time.Minute)}
	ordered := true
	p.screen, _ = pc.CreateDataChannel("screen", &webrtc.DataChannelInit{Ordered: &ordered})
	p.input, _ = pc.CreateDataChannel("input", &webrtc.DataChannelInit{Ordered: &ordered})
	p.control, _ = pc.CreateDataChannel("control", &webrtc.DataChannelInit{Ordered: &ordered})
	var once sync.Once
	p.input.OnOpen(func() { once.Do(func() { close(p.opened) }) })
	p.screen.OnMessage(func(m webrtc.DataChannelMessage) {
		d := m.Data
		p.mu.Lock()
		defer p.mu.Unlock()
		switch d[0] {
		case msgBegin:
			p.frames = append(p.frames, frameSeen{id: binary.LittleEndian.Uint32(d[1:]), w: int(binary.LittleEndian.Uint16(d[5:])),
				h: int(binary.LittleEndian.Uint16(d[7:])), full: d[9]&flagFull != 0})
		case msgTile:
			if _, err := jpeg.Decode(bytes.NewReader(d[13:])); err != nil {
				t.Errorf("tile is not a JPEG: %v", err)
			}
			p.tiles++
		case msgEnd:
			id := binary.LittleEndian.Uint32(d[1:])
			data, _ := json.Marshal(map[string]any{"type": "ack", "frame": id})
			_ = p.control.SendText(string(data))
		}
	})
	p.input.OnMessage(func(m webrtc.DataChannelMessage) {
		var a inputAck
		_ = json.Unmarshal(m.Data, &a)
		p.mu.Lock()
		p.acks[a.Sequence] = a.Reason + "|"
		p.mu.Unlock()
	})
	p.control.OnMessage(func(m webrtc.DataChannelMessage) {
		var v map[string]any
		_ = json.Unmarshal(m.Data, &v)
		p.mu.Lock()
		p.ctrl = append(p.ctrl, v)
		p.mu.Unlock()
	})
	return p
}

func (c *testClient) connect(p *peer, takeover bool) (int, map[string]string) {
	c.t.Helper()
	offer, err := p.pc.CreateOffer(nil)
	if err != nil {
		c.t.Fatal(err)
	}
	gathered := webrtc.GatheringCompletePromise(p.pc)
	if err := p.pc.SetLocalDescription(offer); err != nil {
		c.t.Fatal(err)
	}
	<-gathered
	sdp := p.pc.LocalDescription().SDP
	ch, cn := c.challenge(), nonce()
	req := connectRequest{DeviceID: c.deviceID, Challenge: ch, ClientNonce: cn, SDP: sdp, Takeover: takeover,
		MAC: MAC(c.key, OfferMessage(c.deviceID, ch, cn, sdp, takeover))}
	var out map[string]string
	status := postJSON(c.t, c.base+"/helm/v1/connect", req, &out)
	if status != 200 {
		return status, out
	}
	if MAC(c.key, AnswerMessage(c.deviceID, ch, cn, out["sdp"])) != out["mac"] {
		c.t.Fatal("answer MAC did not verify")
	}
	if err := p.pc.SetRemoteDescription(webrtc.SessionDescription{Type: webrtc.SDPTypeAnswer, SDP: out["sdp"]}); err != nil {
		c.t.Fatal(err)
	}
	return status, out
}

func (p *peer) send(t *testing.T, ev InputEvent, ageMs float64) uint64 {
	p.mu.Lock()
	p.seq++
	ev.Sequence = p.seq
	p.mu.Unlock()
	ev.SentAt = float64(time.Since(p.start).Microseconds())/1000 - ageMs
	data, _ := json.Marshal(inputEnvelope{Type: "input", Event: ev})
	if err := p.input.SendText(string(data)); err != nil {
		t.Fatal(err)
	}
	return ev.Sequence
}

func (p *peer) ackFor(seq uint64) (string, bool) {
	p.mu.Lock()
	defer p.mu.Unlock()
	v, ok := p.acks[seq]
	return strings.TrimSuffix(v, "|"), ok
}

func (p *peer) control_(kind string) map[string]any {
	p.mu.Lock()
	defer p.mu.Unlock()
	for _, m := range p.ctrl {
		if m["type"] == kind {
			return m
		}
	}
	return nil
}

func eventually(t *testing.T, what string, f func() bool) {
	t.Helper()
	deadline := time.Now().Add(8 * time.Second)
	for time.Now().Before(deadline) {
		if f() {
			return
		}
		time.Sleep(20 * time.Millisecond)
	}
	t.Fatalf("timed out waiting for %s", what)
}

type harness struct {
	server  *Server
	display *TestPattern
	base    string
	events  chan Event
	cancel  context.CancelFunc
}

func newHarness(t *testing.T) *harness {
	store, err := OpenStore("")
	if err != nil {
		t.Fatal(err)
	}
	tp := NewTestPattern(640, 400)
	events := make(chan Event, 256)
	s, err := NewServer(Config{ListenAddr: "127.0.0.1:0", Name: "Aurora", Store: store, Display: tp,
		Events: func(e Event) {
			select {
			case events <- e:
			default:
			}
		}})
	if err != nil {
		t.Fatal(err)
	}
	l, err := s.Listen()
	if err != nil {
		t.Fatal(err)
	}
	ctx, cancel := context.WithCancel(context.Background())
	go func() { _ = s.Serve(ctx, l) }()
	t.Cleanup(cancel)
	return &harness{server: s, display: tp, base: "http://" + l.Addr().String(), events: events, cancel: cancel}
}

func (h *harness) link(t *testing.T) string {
	ev, err := h.server.OpenPairing()
	if err != nil {
		t.Fatal(err)
	}
	// The QR code lists LAN addresses; tests reach the loopback listener.
	return ev.Link
}

func TestInfoAndPairing(t *testing.T) {
	h := newHarness(t)
	resp, err := http.Get(h.base + "/helm/v1/info")
	if err != nil {
		t.Fatal(err)
	}
	var info infoResponse
	_ = json.NewDecoder(resp.Body).Decode(&info)
	resp.Body.Close()
	if info.Name != "Aurora" || info.Pairing || info.ServerID == "" {
		t.Fatalf("bad info: %+v", info)
	}
	if _, status := pair(t, h.base, PairingLink(info.ServerID, "x", nil, "AAAAAAAAAAAAAAAA"), "Phone"); status != http.StatusConflict {
		t.Fatalf("pairing without an open window: %d", status)
	}
	link := h.link(t)
	c, status := pair(t, h.base, link, "Kristoffer's phone")
	if status != 200 || c == nil {
		t.Fatalf("pairing failed: %d", status)
	}
	if _, status := pair(t, h.base, link, "Second"); status != http.StatusConflict {
		t.Fatalf("pairing key reused: %d", status)
	}
	devices := h.server.cfg.Store.Devices()
	if len(devices) != 1 || devices[0].Name != "Kristoffer's phone" {
		t.Fatalf("devices: %+v", devices)
	}
}

func TestWrongPairingKeyClosesWindowAfterAttempts(t *testing.T) {
	h := newHarness(t)
	link := h.link(t)
	parsed, _ := ParsePairingLink(link)
	wrong := PairingLink(parsed.ServerID, "x", nil, "BBBBBBBBBBBBBBBB")
	for i := 0; i < pairingAttempts; i++ {
		if _, status := pair(t, h.base, wrong, "Phone"); status != http.StatusUnauthorized {
			t.Fatalf("attempt %d: %d", i, status)
		}
	}
	if _, status := pair(t, h.base, link, "Phone"); status != http.StatusConflict {
		t.Fatalf("window should close after %d wrong keys, got %d", pairingAttempts, status)
	}
}

func TestPairingWindowExpires(t *testing.T) {
	store, _ := OpenStore("")
	now := time.Unix(1000, 0)
	store.now = func() time.Time { return now }
	if _, err := store.OpenPairing("Aurora", nil); err != nil {
		t.Fatal(err)
	}
	now = now.Add(pairingWindow + time.Second)
	if store.PairingOpen() {
		t.Fatal("pairing window still open after expiry")
	}
}

func TestConnectRequiresDeviceKeyAndFreshChallenge(t *testing.T) {
	h := newHarness(t)
	c, _ := pair(t, h.base, h.link(t), "Phone")
	ch := c.challenge()
	cn := nonce()
	bad := connectRequest{DeviceID: c.deviceID, Challenge: ch, ClientNonce: cn, SDP: "v=0",
		MAC: MAC([]byte("wrong key wrong key wrong key 32"), OfferMessage(c.deviceID, ch, cn, "v=0", false))}
	if status := postJSON(t, h.base+"/helm/v1/connect", bad, nil); status != http.StatusUnauthorized {
		t.Fatalf("wrong key: %d", status)
	}
	// The challenge was consumed by the failed attempt.
	bad.MAC = MAC(c.key, OfferMessage(c.deviceID, ch, cn, "v=0", false))
	if status := postJSON(t, h.base+"/helm/v1/connect", bad, nil); status != http.StatusUnauthorized {
		t.Fatalf("replayed challenge: %d", status)
	}
	unknown := connectRequest{DeviceID: "nope", Challenge: c.challenge(), ClientNonce: cn, SDP: "v=0", MAC: "x"}
	var out map[string]string
	if status := postJSON(t, h.base+"/helm/v1/connect", unknown, &out); status != http.StatusUnauthorized || out["error"] != "unpaired" {
		t.Fatalf("unknown device: %d %v", status, out)
	}
}

func TestCORSAllowsOnlyTheApp(t *testing.T) {
	h := newHarness(t)
	for origin, want := range map[string]int{"http://localhost": 204, "capacitor://localhost": 204, "https://evil.example": 403} {
		req, _ := http.NewRequest(http.MethodOptions, h.base+"/helm/v1/connect", nil)
		req.Header.Set("Origin", origin)
		req.Header.Set("Access-Control-Request-Method", "POST")
		resp, err := http.DefaultClient.Do(req)
		if err != nil {
			t.Fatal(err)
		}
		resp.Body.Close()
		if resp.StatusCode != want {
			t.Fatalf("%s: %d", origin, resp.StatusCode)
		}
	}
}

func TestScreenStreamInputAndSessionLifecycle(t *testing.T) {
	h := newHarness(t)
	c, _ := pair(t, h.base, h.link(t), "Phone A")
	p := newPeer(t)
	defer p.pc.Close()
	if status, out := c.connect(p, false); status != 200 {
		t.Fatalf("connect: %d %v", status, out)
	}
	select {
	case <-p.opened:
	case <-time.After(8 * time.Second):
		t.Fatal("data channels did not open")
	}
	eventually(t, "a full frame", func() bool {
		p.mu.Lock()
		defer p.mu.Unlock()
		return len(p.frames) > 0 && p.frames[0].full && p.frames[0].w == 640 && p.frames[0].h == 400 && p.tiles > 0
	})
	eventually(t, "hello", func() bool { return p.control_("hello") != nil })
	if p.control_("hello")["name"] != "Aurora" {
		t.Fatalf("hello: %v", p.control_("hello"))
	}
	// The test pattern changes only a little: later frames are partial.
	eventually(t, "a partial frame", func() bool {
		p.mu.Lock()
		defer p.mu.Unlock()
		return len(p.frames) > 1 && !p.frames[len(p.frames)-1].full
	})

	// A tap lands in the middle of the frame.
	down := p.send(t, InputEvent{Type: "pointer_down", X: .5, Y: .5}, 0)
	up := p.send(t, InputEvent{Type: "pointer_up", X: .5, Y: .5}, 0)
	eventually(t, "tap acks", func() bool { _, a := p.ackFor(down); _, b := p.ackFor(up); return a && b })
	if r, _ := p.ackFor(down); r != "" {
		t.Fatalf("tap refused: %s", r)
	}
	eventually(t, "tap delivered", func() bool {
		e := h.display.Entries()
		return len(e) >= 2 && e[len(e)-2] == "primary down 320,200" && e[len(e)-1] == "primary up 320,200"
	})
	// Late input is dropped instead of landing on a changed screen.
	late := p.send(t, InputEvent{Type: "pointer_down", X: .2, Y: .2}, 2000)
	eventually(t, "late ack", func() bool { _, ok := p.ackFor(late); return ok })
	if r, _ := p.ackFor(late); r != "stale" {
		t.Fatalf("late input: %q", r)
	}
	// Keys outside the allowlist never reach the PC.
	bad := p.send(t, InputEvent{Type: "key", Key: "F4"}, 0)
	eventually(t, "key ack", func() bool { _, ok := p.ackFor(bad); return ok })
	if r, _ := p.ackFor(bad); r != "invalid" {
		t.Fatalf("disallowed key: %q", r)
	}
	text := p.send(t, InputEvent{Type: "text", Text: "Sandhamn"}, 0)
	eventually(t, "text", func() bool { _, ok := p.ackFor(text); return ok })

	// A button held when the phone goes away is released.
	held := p.send(t, InputEvent{Type: "pointer_down", X: .1, Y: .1}, 0)
	eventually(t, "held", func() bool { r, ok := p.ackFor(held); return ok && r == "" })

	// A second phone is told who is connected, then may take over.
	h.server.cfg.Store.secret = nil
	c2, _ := pair(t, h.base, h.link(t), "Phone B")
	p2 := newPeer(t)
	defer p2.pc.Close()
	status, out := c2.connect(p2, false)
	if status != http.StatusConflict || out["by"] != "Phone A" {
		t.Fatalf("busy: %d %v", status, out)
	}
	p2 = newPeer(t)
	if status, _ := c2.connect(p2, true); status != 200 {
		t.Fatalf("takeover: %d", status)
	}
	eventually(t, "first phone told", func() bool {
		bye := p.control_("bye")
		return bye != nil && bye["reason"] == "taken_over" && bye["by"] == "Phone B"
	})
	eventually(t, "held button released", func() bool {
		e := h.display.Entries()
		return e[len(e)-1] == "primary up 64,40"
	})
	select {
	case <-p2.opened:
	case <-time.After(8 * time.Second):
		t.Fatal("second phone did not connect")
	}
	eventually(t, "second phone hello", func() bool { return p2.control_("hello") != nil })
	// Removing the phone in SKAGER ends its session at once.
	h.server.Revoke(c2.deviceID)
	eventually(t, "revoked", func() bool { b := p2.control_("bye"); return b != nil && b["reason"] == "revoked" })
	p3 := newPeer(t)
	defer p3.pc.Close()
	if status, out := c2.connect(p3, false); status != http.StatusUnauthorized || out["error"] != "unpaired" {
		t.Fatalf("revoked phone connected: %d %v", status, out)
	}
}

func TestPilotStatusIsForwarded(t *testing.T) {
	h := newHarness(t)
	h.server.SetPilot(json.RawMessage(`{"state":"standby"}`))
	c, _ := pair(t, h.base, h.link(t), "Phone")
	p := newPeer(t)
	defer p.pc.Close()
	c.connect(p, false)
	eventually(t, "hello with pilot", func() bool { m := p.control_("hello"); return m != nil && m["pilot"] != nil })
	h.server.SetPilot(json.RawMessage(`{"state":"auto","heading":182}`))
	eventually(t, "pilot update", func() bool { return p.control_("pilot") != nil })
}

func TestControlChannel(t *testing.T) {
	h := newHarness(t)
	in, out := net.Pipe()
	var lines []Event
	var mu sync.Mutex
	ctx, cancel := context.WithCancel(context.Background())
	done := make(chan struct{})
	go func() {
		RunControl(ctx, cancel, h.server, out, func(e Event) { mu.Lock(); lines = append(lines, e); mu.Unlock() })
		close(done)
	}()
	_, _ = in.Write([]byte(`{"cmd":"pair"}` + "\n" + `{"cmd":"name","name":"Saga"}` + "\n" + `{"cmd":"bogus"}` + "\n"))
	eventually(t, "events", func() bool { mu.Lock(); defer mu.Unlock(); return len(lines) >= 2 })
	mu.Lock()
	if lines[0].Event != "pairing" || lines[0].QR == nil || lines[0].QR.Size < 21 || !strings.Contains(lines[0].Key, "-") {
		t.Fatalf("pairing event: %+v", lines[0])
	}
	if lines[1].Event != "error" {
		t.Fatalf("unknown command: %+v", lines[1])
	}
	mu.Unlock()
	if h.server.Name() != "Saga" {
		t.Fatal("name not applied")
	}
	in.Close() // SKAGER exited
	select {
	case <-done:
	case <-time.After(2 * time.Second):
		t.Fatal("control did not stop when stdin closed")
	}
	if ctx.Err() == nil {
		t.Fatal("server context not cancelled")
	}
}
