package helm

import (
	"context"
	"encoding/json"
	"errors"
	"net"
	"net/http"
	"regexp"
	"strconv"
	"sync"
	"time"

	"github.com/pion/ice/v4"
	"github.com/pion/webrtc/v4"
)

// Version is set at build time (-ldflags "-X skager.app/helm.Version=…").
var Version = "dev"

const DefaultPort = 43872

// Event is a line on the control channel to SKAGER.
type Event struct {
	Event     string          `json:"event"`
	ServerID  string          `json:"server_id,omitempty"`
	Name      string          `json:"name,omitempty"`
	Port      int             `json:"port,omitempty"`
	Addresses []string        `json:"addresses,omitempty"`
	Key       string          `json:"key,omitempty"`
	Link      string          `json:"link,omitempty"`
	QR        *QRMatrix       `json:"qr,omitempty"`
	Expires   *time.Time      `json:"expires,omitempty"`
	Device    *DeviceInfo     `json:"device,omitempty"`
	Devices   []DeviceInfo    `json:"devices,omitempty"`
	Connected string          `json:"connected,omitempty"`
	State     string          `json:"state,omitempty"`
	Reason    string          `json:"reason,omitempty"`
	Available *bool           `json:"available,omitempty"`
	Message   string          `json:"message,omitempty"`
	Stats     *StreamStats    `json:"stats,omitempty"`
	Raw       json.RawMessage `json:"raw,omitempty"`
}

type Config struct {
	ListenAddr string
	Name       string
	Store      *Store
	Display    Display
	Stream     StreamOptions
	Events     func(Event)
	Logf       func(format string, args ...any)
	// Origins the Helm app's WebView may call from (CORS).
	AllowedOrigins []string
}

var DefaultOrigins = []string{"http://localhost", "https://localhost", "capacitor://localhost"}

// Pages served from this computer itself (the app's browser preview) may call too.
var localOrigin = regexp.MustCompile(`^http://(localhost|127\.0\.0\.1)(:[0-9]{1,5})?$`)

type Server struct {
	cfg     Config
	api     *webrtc.API
	mu      sync.Mutex
	name    string
	session *Session
	pilot   json.RawMessage
	limits  map[string]*bucket
	http    *http.Server
	port    int
}

func NewServer(cfg Config) (*Server, error) {
	if cfg.Store == nil || cfg.Display == nil {
		return nil, errors.New("Helm needs a device store and a display")
	}
	if cfg.ListenAddr == "" {
		cfg.ListenAddr = ":" + strconv.Itoa(DefaultPort)
	}
	if cfg.Stream.Interval == 0 {
		cfg.Stream = DefaultStreamOptions()
	}
	if cfg.Events == nil {
		cfg.Events = func(Event) {}
	}
	if cfg.Logf == nil {
		cfg.Logf = func(string, ...any) {}
	}
	if cfg.AllowedOrigins == nil {
		cfg.AllowedOrigins = DefaultOrigins
	}
	name, ok := cleanName(cfg.Name)
	if !ok {
		name = "SKAGER"
	}
	se := webrtc.SettingEngine{}
	se.SetICEMulticastDNSMode(ice.MulticastDNSModeQueryOnly)
	// A phone that leaves the Wi-Fi is noticed within seconds; input stops at once.
	se.SetICETimeouts(4*time.Second, 12*time.Second, time.Second)
	return &Server{cfg: cfg, api: webrtc.NewAPI(webrtc.WithSettingEngine(se)), name: name, limits: map[string]*bucket{}}, nil
}

func (s *Server) Name() string {
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.name
}

func (s *Server) SetName(name string) {
	if clean, ok := cleanName(name); ok {
		s.mu.Lock()
		s.name = clean
		s.mu.Unlock()
	}
}

// Listen binds the port; Serve then answers on it. Split so SKAGER learns the
// real port (and an occupied port is reported) before any phone connects.
func (s *Server) Listen() (net.Listener, error) {
	l, err := net.Listen("tcp", s.cfg.ListenAddr)
	if err != nil {
		return nil, err
	}
	s.port = l.Addr().(*net.TCPAddr).Port
	return l, nil
}

func (s *Server) Port() int { return s.port }

func (s *Server) Serve(ctx context.Context, l net.Listener) error {
	s.http = &http.Server{Handler: s.Handler(), ReadHeaderTimeout: 5 * time.Second, ReadTimeout: 15 * time.Second,
		WriteTimeout: 20 * time.Second, MaxHeaderBytes: 16 << 10}
	go func() {
		<-ctx.Done()
		// Tell the phone and close the connection before the process exits, so
		// the phone shows "stopped" at once instead of waiting for WebRTC to time out.
		s.mu.Lock()
		current := s.session
		s.mu.Unlock()
		if current != nil {
			current.End("server_stopped", "")
			select {
			case <-current.closed:
			case <-time.After(time.Second):
			}
		}
		shutdown, cancel := context.WithTimeout(context.Background(), 2*time.Second)
		defer cancel()
		_ = s.http.Shutdown(shutdown)
	}()
	err := s.http.Serve(l)
	if errors.Is(err, http.ErrServerClosed) {
		return nil
	}
	return err
}

func (s *Server) Handler() http.Handler {
	mux := http.NewServeMux()
	mux.HandleFunc("/helm/", func(w http.ResponseWriter, r *http.Request) {
		w.Header().Set("Content-Type", "text/plain; charset=utf-8")
		_, _ = w.Write([]byte("SKAGER Helm is running on this computer. Open the SKAGER Helm app on your phone or tablet.\n"))
	})
	mux.HandleFunc("GET /helm/v1/info", s.handleInfo)
	mux.HandleFunc("POST /helm/v1/pair", s.handlePair)
	mux.HandleFunc("GET /helm/v1/challenge", s.handleChallenge)
	mux.HandleFunc("POST /helm/v1/connect", s.handleConnect)
	return s.cors(mux)
}

func (s *Server) cors(next http.Handler) http.Handler {
	return http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		w.Header().Set("Cache-Control", "no-store")
		w.Header().Set("X-Content-Type-Options", "nosniff")
		if origin := r.Header.Get("Origin"); origin != "" {
			allowed := localOrigin.MatchString(origin)
			for _, o := range s.cfg.AllowedOrigins {
				allowed = allowed || o == origin
			}
			if !allowed {
				http.Error(w, "origin not allowed", http.StatusForbidden)
				return
			}
			w.Header().Set("Access-Control-Allow-Origin", origin)
			w.Header().Set("Vary", "Origin")
			if r.Method == http.MethodOptions {
				w.Header().Set("Access-Control-Allow-Methods", "GET, POST")
				w.Header().Set("Access-Control-Allow-Headers", "content-type")
				w.Header().Set("Access-Control-Max-Age", "600")
				if r.Header.Get("Access-Control-Request-Private-Network") == "true" {
					w.Header().Set("Access-Control-Allow-Private-Network", "true")
				}
				w.WriteHeader(http.StatusNoContent)
				return
			}
		}
		next.ServeHTTP(w, r)
	})
}

type infoResponse struct {
	Version  int    `json:"v"`
	ServerID string `json:"server_id"`
	Name     string `json:"name"`
	Pairing  bool   `json:"pairing"`
	Software string `json:"software"`
}

func (s *Server) handleInfo(w http.ResponseWriter, r *http.Request) {
	writeJSON(w, http.StatusOK, infoResponse{Version: 1, ServerID: s.cfg.Store.ServerID(), Name: s.Name(),
		Pairing: s.cfg.Store.PairingOpen(), Software: Version})
}

func (s *Server) handlePair(w http.ResponseWriter, r *http.Request) {
	if !s.allow(r, 10) {
		writeError(w, http.StatusTooManyRequests, "busy", "Too many attempts. Wait a minute and try again.")
		return
	}
	var req PairRequest
	if json.NewDecoder(http.MaxBytesReader(w, r.Body, 4096)).Decode(&req) != nil {
		writeError(w, http.StatusBadRequest, "invalid", "Invalid pairing request.")
		return
	}
	resp, device, err := s.cfg.Store.Pair(req)
	switch {
	case errors.Is(err, errNoPairing):
		writeError(w, http.StatusConflict, "no_pairing", "Pairing is not open on SKAGER. Open Settings → SKAGER Helm → Pair a phone on the SKAGER screen.")
		return
	case errors.Is(err, errPairingKey):
		writeError(w, http.StatusUnauthorized, "wrong_key", "The pairing key does not match the one on the SKAGER screen.")
		s.cfg.Logf("pairing attempt refused from %s", remoteIP(r))
		return
	case errors.Is(err, errTooMany):
		writeError(w, http.StatusConflict, "too_many", err.Error())
		return
	case err != nil:
		writeError(w, http.StatusBadRequest, "invalid", "Invalid pairing request.")
		return
	}
	info := device.Info()
	s.cfg.Logf("paired %q", device.Name)
	s.cfg.Events(Event{Event: "paired", Device: &info})
	s.cfg.Events(Event{Event: "pairing_closed", Reason: "paired"})
	s.emitDevices()
	writeJSON(w, http.StatusOK, resp)
}

func (s *Server) handleChallenge(w http.ResponseWriter, r *http.Request) {
	if !s.allow(r, 30) {
		writeError(w, http.StatusTooManyRequests, "busy", "Too many attempts. Wait a minute and try again.")
		return
	}
	writeJSON(w, http.StatusOK, map[string]string{"challenge": s.cfg.Store.Challenge(), "server_id": s.cfg.Store.ServerID()})
}

type connectRequest struct {
	DeviceID    string `json:"device_id"`
	Challenge   string `json:"challenge"`
	ClientNonce string `json:"client_nonce"`
	SDP         string `json:"sdp"`
	Takeover    bool   `json:"takeover"`
	MAC         string `json:"mac"`
}

func (s *Server) handleConnect(w http.ResponseWriter, r *http.Request) {
	if !s.allow(r, 30) {
		writeError(w, http.StatusTooManyRequests, "busy", "Too many attempts. Wait a minute and try again.")
		return
	}
	var req connectRequest
	if json.NewDecoder(http.MaxBytesReader(w, r.Body, 64<<10)).Decode(&req) != nil || req.SDP == "" {
		writeError(w, http.StatusBadRequest, "invalid", "Invalid connection request.")
		return
	}
	if _, err := decodeNonce(req.ClientNonce); err != nil {
		writeError(w, http.StatusBadRequest, "invalid", "Invalid connection request.")
		return
	}
	device, err := s.cfg.Store.Authenticate(req.DeviceID, req.Challenge,
		OfferMessage(req.DeviceID, req.Challenge, req.ClientNonce, req.SDP, req.Takeover), req.MAC)
	switch {
	case errors.Is(err, errUnknown), errors.Is(err, errBadMAC):
		writeError(w, http.StatusUnauthorized, "unpaired", "This phone is no longer paired with this SKAGER. Pair it again from the SKAGER screen.")
		return
	case err != nil:
		writeError(w, http.StatusUnauthorized, "expired", "The connection attempt expired. Try again.")
		return
	}
	s.mu.Lock()
	current := s.session
	s.mu.Unlock()
	if current != nil && current.Active() && current.device.ID != device.ID && !req.Takeover {
		writeJSON(w, http.StatusConflict, map[string]string{"error": "busy", "by": current.device.Name,
			"message": current.device.Name + " is using SKAGER Helm."})
		return
	}
	sess, answer, err := s.startSession(device, req.SDP)
	if err != nil {
		s.cfg.Logf("connection from %q failed: %v", device.Name, err)
		writeError(w, http.StatusBadRequest, "webrtc", "Could not start the connection.")
		return
	}
	reason := "replaced"
	if current != nil && current.device.ID != device.ID {
		reason = "taken_over"
	}
	s.mu.Lock()
	old := s.session
	s.session = sess
	s.mu.Unlock()
	if old != nil {
		old.End(reason, device.Name)
	}
	s.cfg.Store.Touch(device.ID)
	writeJSON(w, http.StatusOK, map[string]string{"sdp": answer,
		"mac": MAC(device.Key, AnswerMessage(device.ID, req.Challenge, req.ClientNonce, answer))})
}

func (s *Server) startSession(device Device, offer string) (*Session, string, error) {
	pc, err := s.api.NewPeerConnection(webrtc.Configuration{})
	if err != nil {
		return nil, "", err
	}
	sess := newSession(s, device, pc)
	if err := pc.SetRemoteDescription(webrtc.SessionDescription{Type: webrtc.SDPTypeOffer, SDP: offer}); err != nil {
		pc.Close()
		return nil, "", err
	}
	answer, err := pc.CreateAnswer(nil)
	if err != nil {
		pc.Close()
		return nil, "", err
	}
	gathered := webrtc.GatheringCompletePromise(pc)
	if err := pc.SetLocalDescription(answer); err != nil {
		pc.Close()
		return nil, "", err
	}
	select {
	case <-gathered:
	case <-time.After(3 * time.Second):
	}
	local := pc.LocalDescription()
	if local == nil {
		pc.Close()
		return nil, "", errors.New("no local description")
	}
	return sess, local.SDP, nil
}

func (s *Server) sessionEnded(sess *Session, reason string) {
	s.mu.Lock()
	if s.session == sess {
		s.session = nil
	}
	s.mu.Unlock()
	info := sess.device.Info()
	s.cfg.Events(Event{Event: "session", State: "ended", Device: &info, Reason: reason})
	s.emitDevices()
}

// ---------------------------------------------------------------- SKAGER controls

func (s *Server) OpenPairing() (Event, error) {
	addrs := LANAddresses(s.port)
	offer, err := s.cfg.Store.OpenPairing(s.Name(), addrs)
	if err != nil {
		return Event{}, err
	}
	qr, err := EncodeQR(offer.Link)
	if err != nil {
		return Event{}, err
	}
	return Event{Event: "pairing", Key: offer.Key, Link: offer.Link, QR: &qr, Expires: &offer.Expires, Addresses: addrs, Name: s.Name()}, nil
}

func (s *Server) ClosePairing() { s.cfg.Store.ClosePairing() }

func (s *Server) Revoke(deviceID string) {
	s.mu.Lock()
	current := s.session
	s.mu.Unlock()
	if current != nil && current.device.ID == deviceID {
		current.End("revoked", "")
	}
	if s.cfg.Store.Revoke(deviceID) {
		s.emitDevices()
	}
}

func (s *Server) EndSession(reason string) {
	s.mu.Lock()
	current := s.session
	s.mu.Unlock()
	if current != nil {
		current.End(reason, "")
	}
}

// SetPilot forwards SKAGER's autopilot status to the phone. Helm shows it; it
// never sends autopilot commands.
func (s *Server) SetPilot(status json.RawMessage) {
	s.mu.Lock()
	s.pilot = append(json.RawMessage(nil), status...)
	current := s.session
	s.mu.Unlock()
	if current != nil {
		current.sendControl(map[string]any{"type": "pilot", "status": status})
	}
}

func (s *Server) pilotStatus() json.RawMessage {
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.pilot
}

func (s *Server) emitDevices() {
	s.mu.Lock()
	connected := ""
	if s.session != nil && s.session.Active() {
		connected = s.session.device.ID
	}
	s.mu.Unlock()
	s.cfg.Events(Event{Event: "devices", Devices: s.cfg.Store.Devices(), Connected: connected})
}

func (s *Server) EmitDevices() { s.emitDevices() }

// ---------------------------------------------------------------- helpers

type bucket struct {
	tokens float64
	at     time.Time
}

// allow is a per-address token bucket: perMinute requests, bursting to perMinute.
func (s *Server) allow(r *http.Request, perMinute float64) bool {
	ip := remoteIP(r) + r.URL.Path
	s.mu.Lock()
	defer s.mu.Unlock()
	now := time.Now()
	b, ok := s.limits[ip]
	if !ok {
		if len(s.limits) > 1024 {
			s.limits = map[string]*bucket{}
		}
		b = &bucket{tokens: perMinute, at: now}
		s.limits[ip] = b
	}
	b.tokens = min(perMinute, b.tokens+now.Sub(b.at).Minutes()*perMinute)
	b.at = now
	if b.tokens < 1 {
		return false
	}
	b.tokens--
	return true
}

func remoteIP(r *http.Request) string {
	host, _, err := net.SplitHostPort(r.RemoteAddr)
	if err != nil {
		return r.RemoteAddr
	}
	return host
}

func writeJSON(w http.ResponseWriter, status int, value any) {
	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(status)
	_ = json.NewEncoder(w).Encode(value)
}

func writeError(w http.ResponseWriter, status int, code, message string) {
	writeJSON(w, status, map[string]string{"error": code, "message": message})
}
