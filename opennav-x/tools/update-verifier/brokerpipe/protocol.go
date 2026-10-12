// Package brokerpipe transports one authenticated, local prepare-and-stage
// request. Neither a response nor its identifiers authorize installation/launch.
package brokerpipe

import (
	"bytes"
	"context"
	"encoding/binary"
	"encoding/json"
	"errors"
	"io"
	"regexp"
	"time"
	"unicode/utf8"

	verifier "example.com/opennav-update-verifier"
	"example.com/opennav-update-verifier/releasepolicy"
)

const (
	ServiceName      = "SkagerUpdateBroker"
	PipeName         = `\\.\pipe\SkagerUpdateBroker.v1`
	Staged           = "staged-awaiting-qualification"
	maxFrame         = 4096
	maxRequests      = 4096
	handshakeTimeout = 5 * time.Second
	operationTimeout = 30 * time.Minute
)

var ErrUnavailable = errors.New("authenticated native broker transport unavailable")
var ErrRefused = errors.New("broker staging request refused or incomplete")
var id32 = regexp.MustCompile(`^[a-f0-9]{32}$`)
var hex64 = regexp.MustCompile(`^[a-f0-9]{64}$`)
var commit40 = regexp.MustCompile(`^[a-f0-9]{40}$`)

// Request accepts no filesystem path, trust configuration or execution command.
// Consent identifies the offer, not proof of a human gesture. RequestID must be
// a fresh cryptographically random 16-byte value, encoded as lowercase hex.
type Request struct {
	Schema    int
	Operation string
	RequestID string
	Consent   verifier.ConsentIdentity
}

type Result struct {
	Schema    int    `json:"schema"`
	RequestID string `json:"requestId"`
	State     string `json:"state"`
	StageID   string `json:"stageId"`
	Version   string `json:"version"`
	Commit    string `json:"commit"`
}

// Peer is derived from the actual pipe token and a retained live process. It is
// never decoded from JSON. SessionID is the Windows terminal-services session;
// the backend also compares the token authentication LUID (logon session). These
// public fields alone grant no execution authority; AuthorizePeer requires the
// live native context capability.
type Peer struct {
	SID       string
	SessionID uint32
	ProcessID uint32
}

// Handler must independently authorize Peer against protected registration,
// and execution joins must additionally obtain/revalidate AuthorizePeer(ctx, Peer).
// persist RequestID before mutation, obey ctx, and return only completed stage
// results. Serve's replay reservation lasts until Serve exits, not across SCM
// restarts. Durable replay protection belongs to the broker store. No handler
// is invoked concurrently, and errors never produce a successful response.
type Handler func(context.Context, Peer, Request) (Result, error)

type consentWire struct {
	Version      string `json:"version"`
	Commit       string `json:"commit"`
	PolicySHA256 string `json:"policySha256"`
}
type requestWire struct {
	Schema    int         `json:"schema"`
	Operation string      `json:"operation"`
	RequestID string      `json:"requestId"`
	Consent   consentWire `json:"consent"`
}

func validRequest(r Request) bool {
	_, err := releasepolicy.ParseSemVer(r.Consent.Version)
	return r.Schema == 1 && r.Operation == "prepare-and-stage" && id32.MatchString(r.RequestID) && err == nil && len(r.Consent.Version) <= 128 && commit40.MatchString(r.Consent.Commit) && hex64.MatchString(r.Consent.PolicySHA256)
}
func validResult(r Result, q Request) bool {
	return r.Schema == 1 && r.RequestID == q.RequestID && r.State == Staged && r.StageID == q.RequestID && r.Version == q.Consent.Version && r.Commit == q.Consent.Commit
}
func encodeRequest(r Request) ([]byte, error) {
	if !validRequest(r) {
		return nil, ErrRefused
	}
	return json.Marshal(requestWire{r.Schema, r.Operation, r.RequestID, consentWire{r.Consent.Version, r.Consent.Commit, r.Consent.PolicySHA256}})
}

// Decode keys ourselves: encoding/json accepts duplicate keys and case aliases.
func exactObject(raw []byte, keys []string, nested bool) (map[string]json.RawMessage, error) {
	d := json.NewDecoder(bytes.NewReader(raw))
	t, e := d.Token()
	if e != nil || t != json.Delim('{') {
		return nil, ErrRefused
	}
	m := map[string]json.RawMessage{}
	for d.More() {
		t, e = d.Token()
		if e != nil {
			return nil, ErrRefused
		}
		k, ok := t.(string)
		if !ok || len(m) >= len(keys) {
			return nil, ErrRefused
		}
		allowed := false
		for _, v := range keys {
			if k == v {
				allowed = true
				break
			}
		}
		if !allowed {
			return nil, ErrRefused
		}
		if _, ok = m[k]; ok {
			return nil, ErrRefused
		}
		var v json.RawMessage
		if d.Decode(&v) != nil {
			return nil, ErrRefused
		}
		m[k] = v
	}
	if t, e = d.Token(); e != nil || t != json.Delim('}') || len(m) != len(keys) {
		return nil, ErrRefused
	}
	if _, e = d.Token(); e != io.EOF {
		return nil, ErrRefused
	}
	if nested {
		if _, e = exactObject(m["consent"], []string{"version", "commit", "policySha256"}, false); e != nil {
			return nil, e
		}
	}
	return m, nil
}
func decodeRequest(raw []byte) (Request, error) {
	var out Request
	if len(raw) == 0 || len(raw) > maxFrame || !utf8.Valid(raw) {
		return out, ErrRefused
	}
	if _, e := exactObject(raw, []string{"schema", "operation", "requestId", "consent"}, true); e != nil {
		return out, e
	}
	var w requestWire
	if json.Unmarshal(raw, &w) != nil {
		return out, ErrRefused
	}
	out = Request{w.Schema, w.Operation, w.RequestID, verifier.ConsentIdentity{Version: w.Consent.Version, Commit: w.Consent.Commit, PolicySHA256: w.Consent.PolicySHA256}}
	if !validRequest(out) {
		return Request{}, ErrRefused
	}
	return out, nil
}
func decodeResult(raw []byte, q Request) (Result, error) {
	var out Result
	if len(raw) == 0 || len(raw) > maxFrame || !utf8.Valid(raw) {
		return out, ErrRefused
	}
	if _, e := exactObject(raw, []string{"schema", "requestId", "state", "stageId", "version", "commit"}, false); e != nil {
		return out, e
	}
	if json.Unmarshal(raw, &out) != nil || !validResult(out, q) {
		return Result{}, ErrRefused
	}
	return out, nil
}

// The final bounded acknowledgement ensures DisconnectNamedPipe cannot discard
// a response the client has not consumed. It carries no new operation/authority.
func encodeAck(q Request) []byte {
	b, _ := json.Marshal(struct {
		Schema    int    `json:"schema"`
		RequestID string `json:"requestId"`
	}{1, q.RequestID})
	return b
}
func validAck(raw []byte, q Request) bool {
	if len(raw) == 0 || len(raw) > maxFrame || !utf8.Valid(raw) {
		return false
	}
	if _, e := exactObject(raw, []string{"schema", "requestId"}, false); e != nil {
		return false
	}
	var a struct {
		Schema    int    `json:"schema"`
		RequestID string `json:"requestId"`
	}
	return json.Unmarshal(raw, &a) == nil && a.Schema == 1 && a.RequestID == q.RequestID
}

func readFrame(r io.Reader) ([]byte, error) {
	var header [4]byte
	if _, e := io.ReadFull(r, header[:]); e != nil {
		return nil, ErrRefused
	}
	n := binary.LittleEndian.Uint32(header[:])
	if n == 0 || n > maxFrame {
		return nil, ErrRefused
	}
	b := make([]byte, int(n))
	if _, e := io.ReadFull(r, b); e != nil {
		return nil, ErrRefused
	}
	return b, nil
}
func writeFrame(w io.Writer, b []byte) error {
	if len(b) == 0 || len(b) > maxFrame {
		return ErrRefused
	}
	var header [4]byte
	binary.LittleEndian.PutUint32(header[:], uint32(len(b)))
	for _, v := range [][]byte{header[:], b} {
		for len(v) > 0 {
			n, e := w.Write(v)
			if e != nil || n <= 0 || n > len(v) {
				return ErrRefused
			}
			v = v[n:]
		}
	}
	return nil
}

type requests struct{ seen map[string]bool }

func (r *requests) invoke(ctx context.Context, h Handler, p Peer, q Request) (Result, error) {
	if h == nil || ctx == nil || ctx.Err() != nil || !validRequest(q) || p.SID == "" || p.ProcessID == 0 || p.SessionID == 0 {
		return Result{}, ErrRefused
	}
	if r.seen == nil {
		r.seen = map[string]bool{}
	}
	key := p.SID + ":" + q.RequestID
	if r.seen[key] || len(r.seen) >= maxRequests {
		return Result{}, ErrRefused
	}
	r.seen[key] = true // errors/cancellation consume too; never evict into reuse
	out, e := h(ctx, p, q)
	if e != nil || ctx.Err() != nil || !validResult(out, q) {
		return Result{}, ErrRefused
	}
	return out, nil
}
