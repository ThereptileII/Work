package helm

import (
	"context"
	_ "embed"
	"encoding/json"
	"errors"
	"net"
	"net/http"
	"strings"
	"sync"
	"time"

	"github.com/gorilla/websocket"
	"github.com/pion/rtp"
	"github.com/pion/webrtc/v4"
)

//go:embed web/index.html
var helmClientHTML []byte

type ServerConfig struct {
	ListenAddr  string
	ServerID    string
	DisplayName string
	PairingCode string
	MaxClients  int
	Media       MediaPolicy
	TLSRequired bool
	TLSCertFile string
	TLSKeyFile  string
}

type Server struct {
	config   ServerConfig
	registry *SessionRegistry
	track    *webrtc.TrackLocalStaticRTP
	trackMu  sync.Mutex
	onInput  func(SessionInfo, InputEvent)
	upgrader websocket.Upgrader
}

type signalEnvelope struct {
	Type       string                   `json:"type"`
	SDP        string                   `json:"sdp,omitempty"`
	Candidate  *webrtc.ICECandidateInit `json:"candidate,omitempty"`
	ClientName string                   `json:"client_name,omitempty"`
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

func NewServer(cfg ServerConfig) (*Server, error) {
	if cfg.ListenAddr == "" {
		cfg.ListenAddr = "127.0.0.1:43872"
	}
	if cfg.MaxClients == 0 {
		cfg.MaxClients = 1
	}
	if cfg.Media.Width == 0 {
		cfg.Media = DefaultMediaPolicy()
	}
	if err := cfg.Media.Validate(); err != nil {
		return nil, err
	}
	if cfg.PairingCode == "" {
		return nil, errors.New("a pairing code must be provisioned by SKAGER UI")
	}
	r, err := NewSessionRegistry(cfg.ServerID, cfg.DisplayName, cfg.PairingCode, time.Now(), cfg.MaxClients)
	if err != nil {
		return nil, err
	}
	track, err := webrtc.NewTrackLocalStaticRTP(webrtc.RTPCodecCapability{MimeType: webrtc.MimeTypeH264, ClockRate: 90000}, "video", "skager-helm")
	if err != nil {
		return nil, err
	}
	return &Server{config: cfg, registry: r, track: track,
		upgrader: websocket.Upgrader{ReadBufferSize: 8192, WriteBufferSize: 8192,
			CheckOrigin: func(*http.Request) bool { return true }}}, nil
}

func (s *Server) Registry() *SessionRegistry { return s.registry }

func (s *Server) SetInputHandler(handler func(SessionInfo, InputEvent)) { s.onInput = handler }

// PublishRTP publishes an already encoded H264 RTP packet from the platform
// capture/encoder boundary. The server intentionally does not own chart
// rendering or navigation state.
func (s *Server) PublishRTP(packet *rtp.Packet) error {
	if packet == nil {
		return errors.New("nil RTP packet")
	}
	s.trackMu.Lock()
	defer s.trackMu.Unlock()
	return s.track.WriteRTP(packet)
}

func (s *Server) Handler() http.Handler {
	mux := http.NewServeMux()
	mux.HandleFunc("/helm/", func(w http.ResponseWriter, r *http.Request) {
		if r.Method != http.MethodGet {
			http.Error(w, "method not allowed", http.StatusMethodNotAllowed)
			return
		}
		w.Header().Set("Content-Type", "text/html; charset=utf-8")
		_, _ = w.Write(helmClientHTML)
	})
	mux.HandleFunc("/helm/v1/info", s.handleInfo)
	mux.HandleFunc("/helm/v1/pair", s.handlePair)
	mux.HandleFunc("/helm/v1/signal", s.handleSignal)
	return mux
}

func (s *Server) Serve(ctx context.Context) error {
	h := &http.Server{Addr: s.config.ListenAddr, Handler: s.Handler(), ReadHeaderTimeout: 3 * time.Second}
	go func() { <-ctx.Done(); _ = h.Shutdown(context.Background()) }()
	if s.config.TLSCertFile != "" || s.config.TLSKeyFile != "" {
		if s.config.TLSCertFile == "" || s.config.TLSKeyFile == "" {
			return errors.New("both TLS certificate and key are required")
		}
		return h.ListenAndServeTLS(s.config.TLSCertFile, s.config.TLSKeyFile)
	}
	if s.config.TLSRequired || !loopbackListen(s.config.ListenAddr) {
		return errors.New("TLS certificate integration is required before non-loopback deployment")
	}
	return h.ListenAndServe()
}

func loopbackListen(address string) bool {
	host, _, err := net.SplitHostPort(address)
	if err != nil {
		return false
	}
	if host == "localhost" || host == "" {
		return true
	}
	ip := net.ParseIP(host)
	return ip != nil && ip.IsLoopback()
}

func (s *Server) handleInfo(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodGet {
		http.Error(w, "method not allowed", http.StatusMethodNotAllowed)
		return
	}
	writeJSON(w, http.StatusOK, s.registry.Info(time.Now()))
}

func (s *Server) handlePair(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		http.Error(w, "method not allowed", http.StatusMethodNotAllowed)
		return
	}
	var body struct {
		PairingCode string `json:"pairing_code"`
		ClientName  string `json:"client_name"`
	}
	if json.NewDecoder(http.MaxBytesReader(w, r.Body, 4096)).Decode(&body) != nil {
		http.Error(w, "invalid pairing request", http.StatusBadRequest)
		return
	}
	cred, err := s.registry.Pair(body.PairingCode, body.ClientName, time.Now())
	if err != nil {
		http.Error(w, "pairing refused", http.StatusUnauthorized)
		return
	}
	writeJSON(w, http.StatusOK, cred)
}

func (s *Server) handleSignal(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodGet {
		http.Error(w, "method not allowed", http.StatusMethodNotAllowed)
		return
	}
	token := bearerToken(r.Header.Get("Authorization"))
	if token == "" {
		token = r.URL.Query().Get("token")
	}
	info, err := s.registry.Authorize(token, time.Now())
	if err != nil {
		http.Error(w, "unauthorized", http.StatusUnauthorized)
		return
	}
	conn, err := s.upgrader.Upgrade(w, r, nil)
	if err != nil {
		return
	}
	defer conn.Close()
	pc, err := webrtc.NewPeerConnection(webrtc.Configuration{})
	if err != nil {
		return
	}
	defer pc.Close()
	sender, err := pc.AddTrack(s.track)
	if err != nil {
		return
	}
	go func() {
		buf := make([]byte, 1500)
		for {
			if _, _, e := sender.Read(buf); e != nil {
				return
			}
		}
	}()
	pc.OnDataChannel(func(dc *webrtc.DataChannel) {
		if dc.Label() != "input" {
			return
		}
		dc.OnMessage(func(msg webrtc.DataChannelMessage) {
			var env inputEnvelope
			if json.Unmarshal(msg.Data, &env) != nil {
				sendAck(dc, inputAck{Type: "input_ack", Accepted: false, Reason: "malformed"})
				return
			}
			_, e := s.registry.AcceptInput(token, env.Event, time.Now())
			if e == nil && s.onInput != nil {
				s.onInput(info, env.Event)
			}
			ack := inputAck{Type: "input_ack", Sequence: env.Event.Sequence, Accepted: e == nil}
			if e != nil {
				ack.Reason = e.Error()
			}
			sendAck(dc, ack)
		})
	})
	pc.OnConnectionStateChange(func(state webrtc.PeerConnectionState) {
		if state == webrtc.PeerConnectionStateFailed || state == webrtc.PeerConnectionStateClosed || state == webrtc.PeerConnectionStateDisconnected {
			s.registry.Disconnect(info.ID, time.Now())
		}
	})
	for {
		var env signalEnvelope
		if err := conn.ReadJSON(&env); err != nil {
			s.registry.Disconnect(info.ID, time.Now())
			return
		}
		switch env.Type {
		case "offer":
			if env.SDP == "" {
				return
			}
			if err := pc.SetRemoteDescription(webrtc.SessionDescription{Type: webrtc.SDPTypeOffer, SDP: env.SDP}); err != nil {
				return
			}
			answer, err := pc.CreateAnswer(nil)
			if err != nil {
				return
			}
			gather := webrtc.GatheringCompletePromise(pc)
			if err := pc.SetLocalDescription(answer); err != nil {
				return
			}
			<-gather
			local := pc.LocalDescription()
			if local == nil {
				return
			}
			if err := conn.WriteJSON(signalEnvelope{Type: "answer", SDP: local.SDP}); err != nil {
				return
			}
		case "candidate":
			if env.Candidate != nil {
				_ = pc.AddICECandidate(*env.Candidate)
			}
		default:
			return
		}
	}
}

func sendAck(dc *webrtc.DataChannel, ack inputAck) {
	data, err := json.Marshal(ack)
	if err == nil {
		_ = dc.Send(data)
	}
}

func bearerToken(value string) string {
	parts := strings.Fields(value)
	if len(parts) == 2 && strings.EqualFold(parts[0], "bearer") {
		return parts[1]
	}
	return ""
}

func writeJSON(w http.ResponseWriter, status int, value any) {
	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(status)
	_ = json.NewEncoder(w).Encode(value)
}
