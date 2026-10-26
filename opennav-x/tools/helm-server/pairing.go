package helm

import (
	"crypto/hkdf"
	"crypto/hmac"
	"crypto/rand"
	"crypto/sha256"
	"encoding/base32"
	"encoding/base64"
	"encoding/json"
	"errors"
	"os"
	"path/filepath"
	"sort"
	"strings"
	"sync"
	"time"
	"unicode/utf8"
)

// Pairing and connection security
//
// The phone and the PC share nothing until the owner opens a pairing window on
// the SKAGER screen. The window shows a QR code (and the same pairing key as
// text) carrying an 80-bit one-time secret S. The secret never crosses the
// network: the phone proves it knows S with an HMAC, and both sides derive the
// long-term device key K = HKDF(S, both nonces). A listener on the boat Wi-Fi
// sees only nonces and MACs, and cannot derive K or guess S offline.
//
// Every later connection is a challenge-response under K that also covers both
// WebRTC session descriptions. The descriptions carry the DTLS certificate
// fingerprints, so the DTLS channel that carries the screen and the touches is
// authenticated end to end without TLS certificates on the boat network.

const (
	pairingSecretBytes = 10
	pairingWindow      = 5 * time.Minute
	pairingAttempts    = 5
	maxDevices         = 8
	challengeTTL       = time.Minute
	maxChallenges      = 64
)

var (
	errNoPairing   = errors.New("no pairing window is open on SKAGER")
	errPairingKey  = errors.New("pairing key not accepted")
	errUnknown     = errors.New("this phone is not paired with this SKAGER")
	errBadMAC      = errors.New("request not authenticated")
	errTooMany     = errors.New("too many phones are paired; remove one in SKAGER first")
	errChallenge   = errors.New("connection challenge expired")
	errDeviceName  = errors.New("invalid device name")
	errStoreFormat = errors.New("unreadable Helm device store")
)

var b32 = base32.StdEncoding.WithPadding(base32.NoPadding)

// Device is a paired phone or tablet.
type Device struct {
	ID       string    `json:"id"`
	Name     string    `json:"name"`
	Key      []byte    `json:"key"`
	PairedAt time.Time `json:"paired_at"`
	LastSeen time.Time `json:"last_seen,omitempty"`
}

// DeviceInfo is a Device without its key, for SKAGER and logs.
type DeviceInfo struct {
	ID       string    `json:"id"`
	Name     string    `json:"name"`
	PairedAt time.Time `json:"paired_at"`
	LastSeen time.Time `json:"last_seen,omitempty"`
}

func (d Device) Info() DeviceInfo {
	return DeviceInfo{ID: d.ID, Name: d.Name, PairedAt: d.PairedAt, LastSeen: d.LastSeen}
}

// PairingOffer is what SKAGER shows on its screen while the window is open.
type PairingOffer struct {
	Key     string    `json:"key"` // grouped for reading: ABCD-EFGH-JKLM-NPQR
	Link    string    `json:"link"`
	Expires time.Time `json:"expires"`
}

type storeFile struct {
	ServerID string   `json:"server_id"`
	Devices  []Device `json:"devices"`
}

// Store holds the server identity and paired devices. With a path it persists
// them (owner-only file permissions); without one it is memory-only.
type Store struct {
	mu         sync.Mutex
	path       string
	serverID   string
	devices    map[string]Device
	secret     []byte
	until      time.Time
	attempts   int
	challenges map[string]time.Time
	now        func() time.Time
}

func OpenStore(path string) (*Store, error) {
	s := &Store{path: path, devices: map[string]Device{}, challenges: map[string]time.Time{}, now: time.Now}
	if path != "" {
		data, err := os.ReadFile(path)
		switch {
		case err == nil:
			var f storeFile
			if json.Unmarshal(data, &f) != nil || f.ServerID == "" {
				return nil, errStoreFormat
			}
			s.serverID = f.ServerID
			for _, d := range f.Devices {
				if d.ID != "" && len(d.Key) == 32 {
					s.devices[d.ID] = d
				}
			}
		case errors.Is(err, os.ErrNotExist):
		default:
			return nil, err
		}
	}
	if s.serverID == "" {
		s.serverID = randomID(10)
		if err := s.save(); err != nil {
			return nil, err
		}
	}
	return s, nil
}

func (s *Store) ServerID() string { return s.serverID }

func (s *Store) save() error {
	if s.path == "" {
		return nil
	}
	f := storeFile{ServerID: s.serverID, Devices: make([]Device, 0, len(s.devices))}
	for _, d := range s.devices {
		f.Devices = append(f.Devices, d)
	}
	sort.Slice(f.Devices, func(i, j int) bool { return f.Devices[i].PairedAt.Before(f.Devices[j].PairedAt) })
	data, err := json.MarshalIndent(f, "", "  ")
	if err != nil {
		return err
	}
	if err := os.MkdirAll(filepath.Dir(s.path), 0o700); err != nil {
		return err
	}
	tmp := s.path + ".tmp"
	if err := os.WriteFile(tmp, data, 0o600); err != nil {
		return err
	}
	return os.Rename(tmp, s.path)
}

// OpenPairing starts (or restarts) the pairing window with a fresh secret.
func (s *Store) OpenPairing(serverName string, addresses []string) (PairingOffer, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	if len(s.devices) >= maxDevices {
		return PairingOffer{}, errTooMany
	}
	secret := make([]byte, pairingSecretBytes)
	if _, err := rand.Read(secret); err != nil {
		return PairingOffer{}, err
	}
	s.secret, s.until, s.attempts = secret, s.now().Add(pairingWindow), 0
	key := b32.EncodeToString(secret)
	return PairingOffer{Key: groupKey(key), Link: PairingLink(s.serverID, serverName, addresses, key), Expires: s.until}, nil
}

func (s *Store) ClosePairing() {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.secret = nil
}

func (s *Store) PairingOpen() bool {
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.secret != nil && s.now().Before(s.until)
}

// PairRequest is sent by the phone; Proof = HMAC(S, pairMessage).
type PairRequest struct {
	Version     int    `json:"v"`
	DeviceName  string `json:"device_name"`
	ClientNonce string `json:"client_nonce"`
	Proof       string `json:"proof"`
}

type PairResponse struct {
	DeviceID    string `json:"device_id"`
	ServerID    string `json:"server_id"`
	ServerNonce string `json:"server_nonce"`
	Proof       string `json:"proof"`
}

// Pair consumes the open pairing window when the proof is right. A wrong proof
// costs one of a few attempts; the window then closes and SKAGER must open a new one.
func (s *Store) Pair(req PairRequest) (PairResponse, Device, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	if s.secret == nil || !s.now().Before(s.until) {
		s.secret = nil
		return PairResponse{}, Device{}, errNoPairing
	}
	name, ok := cleanName(req.DeviceName)
	cn, err := decodeNonce(req.ClientNonce)
	if req.Version != 1 || !ok || err != nil {
		return PairResponse{}, Device{}, errDeviceName
	}
	if !hmac.Equal(mac(s.secret, pairMessage(s.serverID, req.ClientNonce, name)), decodeB64(req.Proof)) {
		s.attempts++
		if s.attempts >= pairingAttempts {
			s.secret = nil
		}
		return PairResponse{}, Device{}, errPairingKey
	}
	if len(s.devices) >= maxDevices {
		return PairResponse{}, Device{}, errTooMany
	}
	sn := make([]byte, 16)
	if _, err := rand.Read(sn); err != nil {
		return PairResponse{}, Device{}, err
	}
	id := randomID(10)
	key, err := DeviceKey(s.secret, cn, sn, s.serverID, id)
	if err != nil {
		return PairResponse{}, Device{}, err
	}
	d := Device{ID: id, Name: name, Key: key, PairedAt: s.now().UTC()}
	s.devices[id] = d
	s.secret = nil
	if err := s.save(); err != nil {
		delete(s.devices, id)
		return PairResponse{}, Device{}, err
	}
	snText := b64(sn)
	return PairResponse{DeviceID: id, ServerID: s.serverID, ServerNonce: snText,
		Proof: b64(mac(key, pairedMessage(id, req.ClientNonce, snText)))}, d, nil
}

// Challenge issues a single-use nonce for one connection attempt.
func (s *Store) Challenge() string {
	s.mu.Lock()
	defer s.mu.Unlock()
	now := s.now()
	for n, t := range s.challenges {
		if now.After(t) {
			delete(s.challenges, n)
		}
	}
	if len(s.challenges) >= maxChallenges {
		for n := range s.challenges {
			delete(s.challenges, n)
			break
		}
	}
	n := randomID(16)
	s.challenges[n] = now.Add(challengeTTL)
	return n
}

// Authenticate checks a connection request and consumes its challenge.
func (s *Store) Authenticate(deviceID, challenge string, message string, proof string) (Device, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	until, ok := s.challenges[challenge]
	delete(s.challenges, challenge)
	if !ok || s.now().After(until) {
		return Device{}, errChallenge
	}
	d, ok := s.devices[deviceID]
	if !ok {
		return Device{}, errUnknown
	}
	if !hmac.Equal(mac(d.Key, message), decodeB64(proof)) {
		return Device{}, errBadMAC
	}
	return d, nil
}

func (s *Store) Touch(deviceID string) {
	s.mu.Lock()
	defer s.mu.Unlock()
	if d, ok := s.devices[deviceID]; ok {
		d.LastSeen = s.now().UTC()
		s.devices[deviceID] = d
		_ = s.save()
	}
}

func (s *Store) Revoke(deviceID string) bool {
	s.mu.Lock()
	defer s.mu.Unlock()
	if _, ok := s.devices[deviceID]; !ok {
		return false
	}
	delete(s.devices, deviceID)
	_ = s.save()
	return true
}

func (s *Store) Devices() []DeviceInfo {
	s.mu.Lock()
	defer s.mu.Unlock()
	out := make([]DeviceInfo, 0, len(s.devices))
	for _, d := range s.devices {
		out = append(out, d.Info())
	}
	sort.Slice(out, func(i, j int) bool { return out[i].PairedAt.Before(out[j].PairedAt) })
	return out
}

// ---------------------------------------------------------------- messages
// Each authenticated message is a fixed label and newline-separated fields.
// Names cannot contain control characters, so fields cannot be confused.

func pairMessage(serverID, clientNonce, name string) string {
	return "skager-helm pair v1\n" + serverID + "\n" + clientNonce + "\n" + name
}
func pairedMessage(deviceID, clientNonce, serverNonce string) string {
	return "skager-helm paired v1\n" + deviceID + "\n" + clientNonce + "\n" + serverNonce
}

// OfferMessage and AnswerMessage bind the WebRTC descriptions (and with them the
// DTLS fingerprints) to the device key and this connection's challenge.
func OfferMessage(deviceID, challenge, clientNonce, sdp string, takeover bool) string {
	t := "0"
	if takeover {
		t = "1"
	}
	return "skager-helm offer v1\n" + deviceID + "\n" + challenge + "\n" + clientNonce + "\n" + t + "\n" + sdp
}
func AnswerMessage(deviceID, challenge, clientNonce, sdp string) string {
	return "skager-helm answer v1\n" + deviceID + "\n" + challenge + "\n" + clientNonce + "\n" + sdp
}

// DeviceKey derives the long-term key for one paired device.
func DeviceKey(secret, clientNonce, serverNonce []byte, serverID, deviceID string) ([]byte, error) {
	salt := append(append([]byte(nil), clientNonce...), serverNonce...)
	return hkdf.Key(sha256.New, secret, salt, "skager-helm device v1\n"+serverID+"\n"+deviceID, 32)
}

func mac(key []byte, message string) []byte {
	h := hmac.New(sha256.New, key)
	h.Write([]byte(message))
	return h.Sum(nil)
}

// MAC is exported for tests and tools that act as a client.
func MAC(key []byte, message string) string { return b64(mac(key, message)) }

func b64(b []byte) string { return base64.RawURLEncoding.EncodeToString(b) }
func decodeB64(s string) []byte {
	b, err := base64.RawURLEncoding.DecodeString(s)
	if err != nil {
		return nil
	}
	return b
}

func decodeNonce(s string) ([]byte, error) {
	b := decodeB64(s)
	if len(b) < 16 || len(b) > 32 {
		return nil, errors.New("invalid nonce")
	}
	return b, nil
}

func randomID(n int) string {
	b := make([]byte, n)
	if _, err := rand.Read(b); err != nil {
		panic(err)
	}
	return b64(b)
}

func cleanName(name string) (string, bool) {
	name = strings.TrimSpace(name)
	if name == "" || utf8.RuneCountInString(name) > 60 || !utf8.ValidString(name) || strings.ContainsFunc(name, isControl) {
		return "", false
	}
	return name, true
}

func groupKey(key string) string {
	var parts []string
	for len(key) > 4 {
		parts, key = append(parts, key[:4]), key[4:]
	}
	return strings.Join(append(parts, key), "-")
}

// ParsePairingKey accepts the key as typed: any case, with or without dashes or spaces.
func ParsePairingKey(text string) ([]byte, error) {
	clean := strings.Map(func(r rune) rune {
		switch {
		case r == '-' || r == ' ':
			return -1
		case r == '0':
			return 'O'
		case r == '1':
			return 'I'
		}
		return r
	}, strings.ToUpper(text))
	b, err := b32.DecodeString(clean)
	if err != nil || len(b) != pairingSecretBytes {
		return nil, errPairingKey
	}
	return b, nil
}
