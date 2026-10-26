package helm

import (
	"crypto/rand"
	"crypto/sha256"
	"crypto/subtle"
	"encoding/base64"
	"errors"
	"fmt"
	"math"
	"strings"
	"sync"
	"time"
)

var (
	errInvalidPairing = errors.New("invalid or expired pairing code")
	errUnauthorized   = errors.New("session is not authorized")
	errTooManyClients = errors.New("maximum Helm clients reached")
	errInvalidInput   = errors.New("invalid Helm input event")
)

// InputEvent is the only input shape accepted from a Helm client. Coordinates
// are normalized to the current rendered frame; no native screen coordinates
// cross the network boundary.
type InputEvent struct {
	Type      string  `json:"type"`
	X         float64 `json:"x,omitempty"`
	Y         float64 `json:"y,omitempty"`
	Delta     float64 `json:"delta,omitempty"`
	Key       string  `json:"key,omitempty"`
	Text      string  `json:"text,omitempty"`
	Sequence  uint64  `json:"sequence"`
	FrameID   uint64  `json:"frame_id,omitempty"`
	Width     int     `json:"width,omitempty"`
	Height    int     `json:"height,omitempty"`
	PointerID int     `json:"pointer_id,omitempty"`
}

func (e InputEvent) Validate() error {
	switch e.Type {
	case "pointer_down", "pointer_up", "pointer_move":
		if !finiteUnit(e.X) || !finiteUnit(e.Y) || e.PointerID < 0 {
			return errInvalidInput
		}
	case "wheel":
		if !finiteUnit(e.X) || !finiteUnit(e.Y) || !finiteNumber(e.Delta) {
			return errInvalidInput
		}
	case "key":
		if len(e.Key) == 0 || len(e.Key) > 64 {
			return errInvalidInput
		}
	case "text":
		if len(e.Text) == 0 || len(e.Text) > 512 {
			return errInvalidInput
		}
	default:
		return errInvalidInput
	}
	if e.Sequence == 0 {
		return errInvalidInput
	}
	if e.Width < 0 || e.Width > 16384 || e.Height < 0 || e.Height > 16384 {
		return errInvalidInput
	}
	return nil
}

func finiteNumber(v float64) bool { return !math.IsNaN(v) && !math.IsInf(v, 0) }
func finiteUnit(v float64) bool   { return finiteNumber(v) && v >= 0 && v <= 1 }

type PairingInfo struct {
	ServerID       string    `json:"server_id"`
	DisplayName    string    `json:"display_name"`
	PairingExpires time.Time `json:"pairing_expires"`
	Capabilities   []string  `json:"capabilities"`
}

type SessionCredentials struct {
	SessionID string `json:"session_id"`
	Token     string `json:"token"`
}

type SessionInfo struct {
	ID         string    `json:"id"`
	ClientName string    `json:"client_name"`
	Connected  bool      `json:"connected"`
	LastSeen   time.Time `json:"last_seen"`
}

type SessionRegistry struct {
	mu           sync.Mutex
	serverID     string
	displayName  string
	pairingHash  [32]byte
	pairingUntil time.Time
	maxClients   int
	sessions     map[string]session
	capabilities []string
}

type session struct {
	info      SessionInfo
	tokenHash [32]byte
}

func NewSessionRegistry(serverID, displayName, pairingCode string, now time.Time, maxClients int) (*SessionRegistry, error) {
	if strings.TrimSpace(serverID) == "" || strings.TrimSpace(displayName) == "" || len(pairingCode) < 6 {
		return nil, errors.New("invalid Helm registry configuration")
	}
	if maxClients < 1 || maxClients > 8 {
		return nil, errors.New("invalid Helm client limit")
	}
	r := &SessionRegistry{
		serverID:     serverID,
		displayName:  displayName,
		pairingHash:  sha256.Sum256([]byte(pairingCode)),
		pairingUntil: now.Add(10 * time.Minute),
		maxClients:   maxClients,
		sessions:     make(map[string]session),
		capabilities: []string{"webrtc-video", "touch-input", "keyboard-input", "autopilot-status"},
	}
	return r, nil
}

func (r *SessionRegistry) Info(now time.Time) PairingInfo {
	r.mu.Lock()
	defer r.mu.Unlock()
	return PairingInfo{ServerID: r.serverID, DisplayName: r.displayName,
		PairingExpires: r.pairingUntil, Capabilities: append([]string(nil), r.capabilities...)}
}

func (r *SessionRegistry) Pair(pairingCode, clientName string, now time.Time) (SessionCredentials, error) {
	r.mu.Lock()
	defer r.mu.Unlock()
	got := sha256.Sum256([]byte(pairingCode))
	if now.After(r.pairingUntil) || subtle.ConstantTimeCompare(got[:], r.pairingHash[:]) != 1 {
		return SessionCredentials{}, errInvalidPairing
	}
	if len(r.sessions) >= r.maxClients {
		return SessionCredentials{}, errTooManyClients
	}
	if strings.TrimSpace(clientName) == "" || len(clientName) > 80 {
		return SessionCredentials{}, errors.New("invalid client name")
	}
	sessionID, err := randomToken(16)
	if err != nil {
		return SessionCredentials{}, err
	}
	token, err := randomToken(32)
	if err != nil {
		return SessionCredentials{}, err
	}
	r.sessions[sessionID] = session{info: SessionInfo{ID: sessionID, ClientName: clientName, Connected: true, LastSeen: now}, tokenHash: sha256.Sum256([]byte(token))}
	return SessionCredentials{SessionID: sessionID, Token: token}, nil
}

func (r *SessionRegistry) Authorize(token string, now time.Time) (SessionInfo, error) {
	r.mu.Lock()
	defer r.mu.Unlock()
	for id, s := range r.sessions {
		got := sha256.Sum256([]byte(token))
		if subtle.ConstantTimeCompare(got[:], s.tokenHash[:]) == 1 && s.info.Connected {
			s.info.LastSeen = now
			r.sessions[id] = s
			return s.info, nil
		}
	}
	return SessionInfo{}, errUnauthorized
}

func (r *SessionRegistry) Disconnect(sessionID string, now time.Time) {
	r.mu.Lock()
	defer r.mu.Unlock()
	s, ok := r.sessions[sessionID]
	if !ok {
		return
	}
	s.info.Connected = false
	s.info.LastSeen = now
	r.sessions[sessionID] = s
}

func (r *SessionRegistry) Revoke(sessionID string) {
	r.mu.Lock()
	defer r.mu.Unlock()
	delete(r.sessions, sessionID)
}

func (r *SessionRegistry) Sessions() []SessionInfo {
	r.mu.Lock()
	defer r.mu.Unlock()
	out := make([]SessionInfo, 0, len(r.sessions))
	for _, s := range r.sessions {
		out = append(out, s.info)
	}
	return out
}

func (r *SessionRegistry) AcceptInput(token string, event InputEvent, now time.Time) (SessionInfo, error) {
	if err := event.Validate(); err != nil {
		return SessionInfo{}, err
	}
	return r.Authorize(token, now)
}

func randomToken(bytes int) (string, error) {
	b := make([]byte, bytes)
	if _, err := rand.Read(b); err != nil {
		return "", err
	}
	return base64.RawURLEncoding.EncodeToString(b), nil
}

func (e InputEvent) String() string { return fmt.Sprintf("%s/%d", e.Type, e.Sequence) }
