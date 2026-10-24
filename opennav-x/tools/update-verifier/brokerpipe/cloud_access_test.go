package brokerpipe

import (
	"bytes"
	"context"
	"encoding/binary"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"strings"
	"testing"
)

// Deliberately public, synthetic test data; never a stored account credential.
const cloudTokenFixture = "skg1_0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"

func cloudAccessFixture(t *testing.T) CloudAccess {
	t.Helper()
	a, err := ParseCloudAccess(cloudTokenFixture)
	if err != nil {
		t.Fatal("synthetic token refused")
	}
	return a
}

func TestCloudAccessExactGrammarAndAbsentValue(t *testing.T) {
	if (CloudAccess{}).IsPresent() || (CloudAccess{}).Token() != "" {
		t.Fatal("zero value is not absent")
	}
	a := cloudAccessFixture(t)
	if !a.IsPresent() || a.Token() != cloudTokenFixture {
		t.Fatal("valid credential was altered")
	}
	for name, token := range map[string]string{
		"empty": "", "short": cloudTokenFixture[:68], "long": cloudTokenFixture + "0",
		"upper prefix": "SKG1_" + cloudTokenFixture[5:], "upper hex": "skg1_" + strings.Repeat("A", 64),
		"nonhex": "skg1_" + strings.Repeat("g", 64), "newline": cloudTokenFixture[:68] + "\n",
		"nul": cloudTokenFixture[:68] + "\x00", "bearer": "Bearer " + cloudTokenFixture,
		"unicode": "skg1_" + strings.Repeat("a", 62) + "ä", "space": " " + cloudTokenFixture,
	} {
		t.Run(name, func(t *testing.T) {
			got, err := ParseCloudAccess(token)
			if !errors.Is(err, ErrRefused) || got.IsPresent() || strings.Contains(err.Error(), token) && token != "" {
				t.Fatal("malformed credential accepted or disclosed")
			}
		})
	}
}

func TestCloudAccessDiagnosticsNeverSerializeSecret(t *testing.T) {
	a := cloudAccessFixture(t)
	q := requestFixture()
	q.CloudAccess = a
	x := executionTestRequest()
	x.CloudAccess = a
	for _, value := range []any{a, &a, q, &q, x, &x, struct{ Access CloudAccess }{a}} {
		for _, format := range []string{"%v", "%+v", "%#v", "%s", "%q", "%d", "%x", "%X", "%b", "%f"} {
			if strings.Contains(fmt.Sprintf(format, value), cloudTokenFixture) {
				t.Fatal("generic formatting disclosed credential")
			}
		}
		raw, err := json.Marshal(value)
		if err != nil || bytes.Contains(raw, []byte(cloudTokenFixture)) || bytes.Contains(raw, []byte("cloudAccess")) {
			t.Fatal("generic JSON disclosed credential or wire field")
		}
	}
	for _, raw := range []string{`null`, `{}`, `"` + cloudTokenFixture + `"`} {
		var got CloudAccess
		if json.Unmarshal([]byte(raw), &got) == nil || got.IsPresent() {
			t.Fatal("generic JSON created a credential")
		}
	}
}

func TestCloudAccessStagingAndExecutionWireRoundTrip(t *testing.T) {
	q := requestFixture()
	old, err := encodeRequest(q)
	want := `{"schema":1,"operation":"prepare-and-stage","requestId":"` + q.RequestID + `","consent":{"version":"` + q.Consent.Version + `","commit":"` + q.Consent.Commit + `","policySha256":"` + q.Consent.PolicySHA256 + `"}}`
	if err != nil || string(old) != want {
		t.Fatal("absent-token staging encoding changed")
	}
	q.CloudAccess = cloudAccessFixture(t)
	raw, err := encodeRequest(q)
	if err != nil || bytes.Count(raw, []byte(cloudTokenFixture)) != 1 {
		t.Fatal("explicit staging wire did not contain exactly one credential")
	}
	got, err := decodeRequest(raw)
	if err != nil || got != q {
		t.Fatal("staging credential round-trip failed")
	}
	for _, operation := range []string{"offer-current", "execute-session"} {
		x := executionTestRequest()
		x.Operation = operation
		old, err := encodeExecutionRequest(x)
		want := `{"schema":1,"operation":"` + operation + `","requestId":"` + x.RequestID + `","stageId":"` + x.StageID + `"}`
		if err != nil || string(old) != want {
			t.Fatal("absent-token execution encoding changed")
		}
		x.CloudAccess = q.CloudAccess
		raw, err := encodeExecutionRequest(x)
		if err != nil || bytes.Count(raw, []byte(cloudTokenFixture)) != 1 {
			t.Fatal("explicit execution wire did not contain exactly one credential")
		}
		got, err := decodeExecutionRequest(raw)
		if err != nil || got != x {
			t.Fatal("execution credential round-trip failed")
		}
	}
}

func addCloudField(raw []byte, field string) []byte {
	return []byte(string(raw[:len(raw)-1]) + "," + field + "}")
}

func TestCloudAccessWireRejectsMalformedAndExpandedFields(t *testing.T) {
	staging, _ := encodeRequest(requestFixture())
	execution, _ := encodeExecutionRequest(executionTestRequest())
	for _, protocol := range []struct {
		name   string
		raw    []byte
		decode func([]byte) error
	}{
		{"staging", staging, func(raw []byte) error { _, err := decodeRequest(raw); return err }},
		{"execution", execution, func(raw []byte) error { _, err := decodeExecutionRequest(raw); return err }},
	} {
		t.Run(protocol.name, func(t *testing.T) {
			for name, field := range map[string]string{
				"null": `"cloudAccess":null`, "empty": `"cloudAccess":""`, "number": `"cloudAccess":1`,
				"object": `"cloudAccess":{}`, "array": `"cloudAccess":[]`,
				"short":           `"cloudAccess":"` + cloudTokenFixture[:68] + `"`,
				"long":            `"cloudAccess":"` + cloudTokenFixture + `0"`,
				"oversize":        `"cloudAccess":"` + strings.Repeat("a", maxFrame) + `"`,
				"case alias":      `"CloudAccess":"` + cloudTokenFixture + `"`,
				"duplicate":       `"cloudAccess":"` + cloudTokenFixture + `","cloudAccess":"` + cloudTokenFixture + `"`,
				"duplicate alias": `"cloudAccess":"` + cloudTokenFixture + `","CloudAccess":"` + cloudTokenFixture + `"`,
				"escape alias":    `"cloudAccess":"\u0073` + cloudTokenFixture[1:] + `"`,
				"upper hex":       `"cloudAccess":"skg1_` + strings.Repeat("A", 64) + `"`,
				"extra authority": `"cloudAccess":"` + cloudTokenFixture + `","cloudApproved":true`,
			} {
				t.Run(name, func(t *testing.T) {
					if err := protocol.decode(addCloudField(protocol.raw, field)); !errors.Is(err, ErrRefused) {
						t.Fatal("malformed or expanded credential wire accepted")
					}
				})
			}
		})
	}
}

func TestCloudAccessCannotEnterOrdinaryStartupOrOtherModes(t *testing.T) {
	for _, operation := range []string{"start-current", "start-selected"} {
		for _, mode := range []StartupMode{StartupXNav, StartupLegacy, StartupSafe} {
			t.Run(operation+"/"+string(mode), func(t *testing.T) {
				q := executionTestRequest()
				q.Operation, q.StartupMode = operation, mode
				old, err := encodeExecutionRequest(q)
				if err != nil {
					t.Fatal("existing offline mode refused")
				}
				if got, err := decodeExecutionRequest(old); err != nil || got != q {
					t.Fatal("existing offline mode did not round-trip")
				}
				q.CloudAccess = cloudAccessFixture(t)
				if _, err := encodeExecutionRequest(q); !errors.Is(err, ErrRefused) {
					t.Fatal("credential entered ordinary startup")
				}
				if _, err := decodeExecutionRequest(addCloudField(old, `"cloudAccess":"`+cloudTokenFixture+`"`)); !errors.Is(err, ErrRefused) {
					t.Fatal("received credential entered ordinary startup")
				}
			})
		}
	}
	for _, operation := range []string{"offer-current", "execute-session"} {
		for _, mode := range []StartupMode{StartupLegacy, StartupSafe} {
			q := executionTestRequest()
			q.Operation, q.CloudAccess = operation, cloudAccessFixture(t)
			raw, _ := encodeExecutionRequest(q)
			q.StartupMode = mode
			if _, err := encodeExecutionRequest(q); err == nil {
				t.Fatal("cloud credential permitted non-XNav update")
			}
			if _, err := decodeExecutionRequest(addCloudField(raw, `"startupMode":"`+string(mode)+`"`)); err == nil {
				t.Fatal("wire credential permitted non-XNav update")
			}
		}
	}
	q := requestFixture()
	q.CloudAccess = CloudAccess{value: "invalid internal value"}
	if _, err := encodeRequest(q); err == nil {
		t.Fatal("unchecked internal staging credential encoded")
	}
	x := executionTestRequest()
	x.CloudAccess = q.CloudAccess
	if _, err := encodeExecutionRequest(x); err == nil {
		t.Fatal("unchecked internal execution credential encoded")
	}
}

func TestCloudAccessDoesNotEnterResponsesOrChangeReplayIdentity(t *testing.T) {
	q := requestFixture()
	q.CloudAccess = cloudAccessFixture(t)
	response, _ := json.Marshal(resultFixture(q))
	ack := encodeAck(q)
	for _, raw := range [][]byte{response, ack} {
		if bytes.Contains(raw, []byte(cloudTokenFixture)) || bytes.Contains(raw, []byte("cloudAccess")) {
			t.Fatal("credential entered staging reply")
		}
	}
	if _, err := decodeResult(addCloudField(response, `"cloudAccess":"`+cloudTokenFixture+`"`), q); err == nil {
		t.Fatal("staging result accepted a credential")
	}
	if validAck(addCloudField(ack, `"cloudAccess":"`+cloudTokenFixture+`"`), q) {
		t.Fatal("staging acknowledgement accepted a credential")
	}
	var reserved requests
	calls := 0
	handler := func(_ context.Context, _ Peer, got Request) (Result, error) {
		calls++
		if got.Consent != q.Consent || got.CloudAccess.Token() != cloudTokenFixture {
			t.Fatal("credential changed consent or did not reach the handler")
		}
		return resultFixture(got), nil
	}
	peer := Peer{SID: "S-1-5-21-1-2-3-1001", SessionID: 1, ProcessID: 42}
	if _, err := reserved.invoke(context.Background(), handler, peer, q); err != nil {
		t.Fatal("credential-bearing request refused")
	}
	q.CloudAccess, _ = ParseCloudAccess("skg1_" + strings.Repeat("f", 64))
	if _, err := reserved.invoke(context.Background(), handler, peer, q); err == nil || calls != 1 {
		t.Fatal("changed credential allowed request replay")
	}
}

func TestCloudAccessExecutionSessionDoesNotEchoCredential(t *testing.T) {
	q := executionTestRequest()
	q.CloudAccess = cloudAccessFixture(t)
	ctx, cancel := context.WithCancel(context.Background())
	a, c := &fakeExecutionAuthority{}, &fakeExecutionConnection{}
	s, err := newExecutionSession(ctx, cancel, q, a, c)
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { s.Close() })
	bindExecutionTestSession(t, s)
	if _, err := s.Exchange(ctx, FinishExecution); err != nil {
		t.Fatal(err)
	}
	if len(c.commands) != 1 {
		t.Fatal("unexpected session transcript")
	}
	command := c.commands[0]
	raw, _ := json.Marshal(command)
	closing, _ := executionClosingFrame(command, "closed")
	for _, data := range [][]byte{raw, c.response, closing} {
		if bytes.Contains(data, []byte(cloudTokenFixture)) || bytes.Contains(data, []byte("cloudAccess")) {
			t.Fatal("execution transcript disclosed credential")
		}
	}
	if _, err := decodeExecutionCommand(addCloudField(raw, `"cloudAccess":"`+cloudTokenFixture+`"`), "command"); err == nil {
		t.Fatal("command accepted a credential")
	}
	if _, err := decodeExecutionReport(addCloudField(c.response, `"cloudAccess":"`+cloudTokenFixture+`"`), command); err == nil {
		t.Fatal("report accepted a credential")
	}
	if validExecutionClosingFrame(addCloudField(closing, `"cloudAccess":"`+cloudTokenFixture+`"`), command, "closed") {
		t.Fatal("closing frame accepted a credential")
	}
}

type partialCloudFrame struct {
	header   bool
	retained []byte
}

func (r *partialCloudFrame) Read(out []byte) (int, error) {
	if !r.header {
		r.header = true
		binary.LittleEndian.PutUint32(out, 100)
		return 4, nil
	}
	r.retained = out
	return copy(out, cloudTokenFixture), io.ErrUnexpectedEOF
}

func TestCloudAccessPartialFrameIsCleared(t *testing.T) {
	r := &partialCloudFrame{}
	if raw, err := readFrame(r); err == nil || raw != nil || len(r.retained) != 100 {
		t.Fatal("partial request frame accepted")
	}
	if !bytes.Equal(r.retained, make([]byte, len(r.retained))) {
		t.Fatal("partial credential frame retained bytes after refusal")
	}
}
