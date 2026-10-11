package brokerpipe

import (
	"bytes"
	"context"
	"encoding/binary"
	"encoding/json"
	"errors"
	"io"
	"strings"
	"testing"

	verifier "example.com/opennav-update-verifier"
)

func requestFixture() Request {
	return Request{Schema: 1, Operation: "prepare-and-stage", RequestID: strings.Repeat("a", 32), Consent: verifier.ConsentIdentity{Version: "2.1.0-beta.2", Commit: strings.Repeat("b", 40), PolicySHA256: strings.Repeat("c", 64)}}
}
func resultFixture(q Request) Result {
	return Result{1, q.RequestID, Staged, q.RequestID, q.Consent.Version, q.Consent.Commit}
}
func TestExactRequestProtocol(t *testing.T) {
	q := requestFixture()
	raw, e := encodeRequest(q)
	if e != nil {
		t.Fatal(e)
	}
	got, e := decodeRequest(raw)
	if e != nil || got != q {
		t.Fatalf("roundtrip: %+v %v", got, e)
	}
	cases := map[string][]byte{
		"duplicate":        bytes.Replace(raw, []byte(`"schema":1`), []byte(`"schema":1,"schema":1`), 1),
		"case alias":       bytes.Replace(raw, []byte(`"schema"`), []byte(`"Schema"`), 1),
		"nested duplicate": bytes.Replace(raw, []byte(`"version":`), []byte(`"version":"2.0.0","version":`), 1),
		"nested case":      bytes.Replace(raw, []byte(`"commit"`), []byte(`"Commit"`), 1),
		"path":             append([]byte(`{"path":"C:/arbitrary",`), raw[1:]...),
		"trust":            append([]byte(`{"trust":{},`), raw[1:]...),
		"trailing":         append(append([]byte{}, raw...), []byte(`{}`)...),
		"execute":          bytes.Replace(raw, []byte("prepare-and-stage"), []byte("execute"), 1),
		"null consent":     []byte(`{"schema":1,"operation":"prepare-and-stage","requestId":"` + q.RequestID + `","consent":null}`),
		"schema float":     bytes.Replace(raw, []byte(`"schema":1`), []byte(`"schema":1.0`), 1),
		"missing":          bytes.Replace(raw, []byte(`"schema":1,`), nil, 1),
		"oversize":         bytes.Repeat([]byte(" "), maxFrame+1),
		"nested array":     bytes.Replace(raw, []byte(`"version":"2.1.0-beta.2"`), []byte(`"version":[]`), 1),
	}
	for name, b := range cases {
		t.Run(name, func(t *testing.T) {
			if _, e := decodeRequest(b); e == nil {
				t.Fatal("accepted malformed/expanded request")
			}
		})
	}
	for _, mutate := range []func(*Request){func(r *Request) { r.Schema = 2 }, func(r *Request) { r.Operation = "install" }, func(r *Request) { r.RequestID = strings.Repeat("A", 32) }, func(r *Request) { r.Consent.PolicySHA256 = "" }, func(r *Request) { r.Consent.Commit = "abc" }, func(r *Request) { r.Consent.Version = "v2.0.0" }} {
		r := q
		mutate(&r)
		if _, e := encodeRequest(r); e == nil {
			t.Fatal("encoded invalid request")
		}
	}
}
func TestResultCannotAuthorizeOtherOperationOrIdentity(t *testing.T) {
	q := requestFixture()
	result := resultFixture(q)
	raw, _ := json.Marshal(result)
	if got, e := decodeResult(raw, q); e != nil || got != result {
		t.Fatal(got, e)
	}
	for name, mutate := range map[string]func(*Result){
		"install success": func(r *Result) { r.State = "installed" }, "launch success": func(r *Result) { r.State = "started" },
		"request": func(r *Result) { r.RequestID = strings.Repeat("d", 32) }, "stage path": func(r *Result) { r.StageID = "C:/stage" },
		"different valid stage ID": func(r *Result) { r.StageID = strings.Repeat("d", 32) },
		"version":                  func(r *Result) { r.Version = "9.9.9" }, "commit": func(r *Result) { r.Commit = strings.Repeat("e", 40) }, "schema": func(r *Result) { r.Schema = 2 },
	} {
		t.Run(name, func(t *testing.T) {
			r := result
			mutate(&r)
			b, _ := json.Marshal(r)
			if _, e := decodeResult(b, q); e == nil {
				t.Fatal("accepted wrong result")
			}
		})
	}
	for _, b := range [][]byte{append([]byte(`{"schema":1,`), raw[1:]...), bytes.Replace(raw, []byte(`"state"`), []byte(`"State"`), 1), append([]byte(`{"authority":true,`), raw[1:]...)} {
		if _, e := decodeResult(b, q); e == nil {
			t.Fatal("accepted result alias/extra")
		}
	}
}
func TestFramingBoundsAndPartialIO(t *testing.T) {
	q := requestFixture()
	raw, _ := encodeRequest(q)
	var b bytes.Buffer
	if e := writeFrame(&b, raw); e != nil {
		t.Fatal(e)
	}
	got, e := readFrame(&b)
	if e != nil || !bytes.Equal(raw, got) {
		t.Fatal(e)
	}
	for _, size := range []uint32{0, maxFrame + 1, ^uint32(0)} {
		var h [4]byte
		binary.LittleEndian.PutUint32(h[:], size)
		if _, e := readFrame(bytes.NewReader(h[:])); e == nil {
			t.Fatal("accepted length", size)
		}
	}
	for _, short := range [][]byte{{1, 2, 3}, {5, 0, 0, 0, 'a'}} {
		if _, e := readFrame(bytes.NewReader(short)); e == nil {
			t.Fatal("accepted truncated frame")
		}
	}
	if e := writeFrame(io.Discard, make([]byte, maxFrame+1)); e == nil {
		t.Fatal("oversized write")
	}
	if e := writeFrame(shortWriter{}, raw); e == nil {
		t.Fatal("zero-progress write")
	}
	one := &oneByteWriter{}
	if e := writeFrame(one, raw); e != nil {
		t.Fatal(e)
	}
	if got, e := readFrame(bytes.NewReader(one.Bytes())); e != nil || !bytes.Equal(raw, got) {
		t.Fatal("partial-write framing", e)
	}
}

type shortWriter struct{}

func (shortWriter) Write([]byte) (int, error) { return 0, nil }

type oneByteWriter struct{ bytes.Buffer }

func (w *oneByteWriter) Write(b []byte) (int, error) { return w.Buffer.Write(b[:1]) }
func TestReservationsConsumeFailureAndNeverEvict(t *testing.T) {
	p := Peer{"S-1-5-21-1-2-3-1001", 1, 42}
	q := requestFixture()
	ctx := context.Background()
	var r requests
	calls := 0
	handler := func(context.Context, Peer, Request) (Result, error) {
		calls++
		return Result{}, errors.New("private handler details")
	}
	if _, e := r.invoke(ctx, handler, p, q); e != ErrRefused {
		t.Fatal(e)
	}
	if _, e := r.invoke(ctx, handler, p, q); e != ErrRefused || calls != 1 {
		t.Fatal("failed request replayed", e, calls)
	}
	q.RequestID = strings.Repeat("d", 32)
	success := func(_ context.Context, got Peer, req Request) (Result, error) {
		calls++
		if got != p {
			t.Fatal("peer changed")
		}
		return resultFixture(req), nil
	}
	if _, e := r.invoke(ctx, success, p, q); e != nil {
		t.Fatal(e)
	}
	if _, e := r.invoke(ctx, success, p, q); e == nil {
		t.Fatal("success replayed")
	}
	q.RequestID = strings.Repeat("e", 32)
	cancelled, cancel := context.WithCancel(ctx)
	cancel()
	if _, e := r.invoke(cancelled, success, p, q); e == nil {
		t.Fatal("cancelled request entered")
	}
	if len(r.seen) != 2 {
		t.Fatal("invalid/cancelled request reserved")
	}
	q.RequestID = strings.Repeat("f", 32)
	bad := func(context.Context, Peer, Request) (Result, error) { return Result{State: "started"}, nil }
	if _, e := r.invoke(ctx, bad, p, q); e == nil {
		t.Fatal("bad handler result accepted")
	}
	if _, e := r.invoke(ctx, success, p, q); e == nil {
		t.Fatal("invalid-result request replayed")
	}
	full := requests{seen: map[string]bool{}}
	for i := 0; i < maxRequests; i++ {
		full.seen[string(rune(i))] = true
	}
	before := calls
	if _, e := full.invoke(ctx, success, p, q); e == nil || calls != before {
		t.Fatal("full reservation set evicted/admitted")
	}
}
func TestHandlerCancellationAndInvalidPeer(t *testing.T) {
	q := requestFixture()
	p := Peer{"S-1-5-21-1-2-3-1001", 1, 42}
	ctx, cancel := context.WithCancel(context.Background())
	var r requests
	handler := func(context.Context, Peer, Request) (Result, error) { cancel(); return resultFixture(q), nil }
	if _, e := r.invoke(ctx, handler, p, q); e == nil {
		t.Fatal("success after cancellation")
	}
	for _, bad := range []Peer{{"", 1, 42}, {p.SID, 0, 42}, {p.SID, 1, 0}} {
		var r requests
		if _, e := r.invoke(context.Background(), handler, bad, q); e == nil || len(r.seen) != 0 {
			t.Fatal("invalid peer admitted")
		}
	}
}

func TestAcknowledgementBindsExactCompletedRequest(t *testing.T) {
	q := requestFixture()
	raw := encodeAck(q)
	if !validAck(raw, q) {
		t.Fatal("valid acknowledgement refused")
	}
	for _, b := range [][]byte{bytes.Replace(raw, []byte(q.RequestID), []byte(strings.Repeat("b", 32)), 1), append([]byte(`{"schema":1,`), raw[1:]...), bytes.Replace(raw, []byte(`"schema"`), []byte(`"Schema"`), 1), append(raw, []byte(`{}`)...)} {
		if validAck(b, q) {
			t.Fatal("accepted mismatched/ambiguous ack")
		}
	}
}

func TestStageIDsAreExactlyRequest32AndJSONMustBeUTF8(t *testing.T) {
	q := requestFixture()
	if len(q.RequestID) != 32 {
		t.Fatal("fixture ID length")
	}
	bad := q
	bad.RequestID = strings.Repeat("a", 64)
	if _, e := encodeRequest(bad); e == nil {
		t.Fatal("accepted SHA-length request ID")
	}
	r := resultFixture(q)
	r.StageID = strings.Repeat("d", 32)
	raw, _ := json.Marshal(r)
	if _, e := decodeResult(raw, q); e == nil {
		t.Fatal("accepted different valid stage ID")
	}
	req, _ := encodeRequest(q)
	res, _ := json.Marshal(resultFixture(q))
	ack := encodeAck(q)
	// Insert actual invalid UTF-8, not a JSON Unicode escape.
	invalid := func(b []byte) []byte {
		return bytes.Replace(b, []byte(q.RequestID), append([]byte{0xff}, []byte(q.RequestID[1:])...), 1)
	}
	if _, e := decodeRequest(invalid(req)); e == nil {
		t.Fatal("invalid UTF-8 request")
	}
	if _, e := decodeResult(invalid(res), q); e == nil {
		t.Fatal("invalid UTF-8 result")
	}
	if validAck(invalid(ack), q) {
		t.Fatal("invalid UTF-8 acknowledgement")
	}
	if !hex64.MatchString(q.Consent.PolicySHA256) {
		t.Fatal("SHA256 bound changed with ID bound")
	}
}
