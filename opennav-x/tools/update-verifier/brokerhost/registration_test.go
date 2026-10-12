package brokerhost

import (
	"bytes"
	"encoding/json"
	"strings"
	"testing"
	"time"

	"example.com/opennav-update-verifier/brokerregistration"
)

func TestProtectedRegistrationRejectsAmbiguousAndUnapprovedInput(t *testing.T) {
	valid := testRegistration()
	if _, err := parseRegistration(valid, testSID); err != nil {
		t.Fatal(err)
	}
	cases := map[string][]byte{
		"duplicate":    bytes.Replace(valid, []byte(`"schema":2`), []byte(`"schema":2,"schema":2`), 1),
		"case folding": bytes.Replace(valid, []byte(`"clientSid"`), []byte(`"ClientSid"`), 1),
		"unknown":      bytes.Replace(valid, []byte(`"schema":2`), []byte(`"schema":2,"execute":true`), 1),
		"nested duplicate": mutateProtectedClientRequest(valid, func(raw []byte) []byte {
			return bytes.Replace(raw, []byte(`"version":"0.2.0-beta.1"`), []byte(`"version":"0.2.0-beta.1","Version":"0.2.0-beta.1"`), 1)
		}),
		"trailing":     append(bytes.Clone(valid), []byte(`{}`)...),
		"invalid UTF8": append([]byte{0xff}, valid...),
		"oversize":     bytes.Repeat([]byte{' '}, 1<<20+1),
		"null root": mutateProtectedClientRequest(valid, func(raw []byte) []byte {
			return bytes.Replace(raw, []byte(`"bootstrapRoot":`), []byte(`"bootstrapRoot":null,"ignored":`), 1)
		}),
		"wrong stock hash": mutateProtectedClientRequest(valid, func(raw []byte) []byte {
			return bytes.ReplaceAll(raw, []byte(stockBaseline.ExecutableSHA256), []byte(strings.Repeat("0", 64)))
		}),
		"wrong arch":         mutateProtectedClientRequest(valid, func(raw []byte) []byte { return bytes.ReplaceAll(raw, []byte(`"arch":"x86"`), []byte(`"arch":"x64"`)) }),
		"wrong peer":         bytes.Replace(valid, []byte(testSID), []byte(testSID+"1"), 1),
		"wrong request peer": mutateProtectedClientRequest(valid, func(raw []byte) []byte { return bytes.Replace(raw, []byte(testSID), []byte(testSID+"1"), 1) }),
		"untrusted URL": mutateProtectedClientRequest(valid, func(raw []byte) []byte {
			return bytes.ReplaceAll(raw, []byte("https://updates.invalid/metadata/"), []byte("http://updates.invalid/metadata/"))
		}),
		"origin with path": mutateProtectedClientRequest(valid, func(raw []byte) []byte {
			return bytes.ReplaceAll(raw, []byte(`"artifactOrigin":"https://updates.invalid"`), []byte(`"artifactOrigin":"https://updates.invalid/artifacts"`))
		}),
		"schema1 cannot migrate": []byte(`{"schema":1,"clientSid":"` + testSID + `","installed":{"version":"0.2.0-beta.1","commit":"` + strings.Repeat("a", 40) + `"},"stock":{},"updateTrust":{}}`),
	}
	for name, data := range cases {
		t.Run(name, func(t *testing.T) {
			if _, err := parseRegistration(data, testSID); err == nil {
				t.Fatal("invalid registration accepted")
			}
		})
	}
}

func TestProtectedRegistrationUsesFixedLocalStockContract(t *testing.T) {
	for _, path := range []string{`\\server\share\opencpn.exe`, `C:\OpenCPN\..\opencpn.exe`, `C:\OpenCPN\opencpn.exe:stream`, `C:/OpenCPN/opencpn.exe`, `C:\OpenCPN.\opencpn.exe`, `C:\OpenCPN\other.exe`, `C:\\OpenCPN\opencpn.exe`, `C:\CON\opencpn.exe`, "C:\\Open\nCPN\\opencpn.exe"} {
		t.Run(path, func(t *testing.T) {
			raw := mutateProtectedClientRequest(testRegistration(), func(raw []byte) []byte {
				var request map[string]json.RawMessage
				_ = json.Unmarshal(raw, &request)
				var stock map[string]json.RawMessage
				_ = json.Unmarshal(request["stock"], &stock)
				stock["path"], _ = json.Marshal(path)
				request["stock"], _ = json.Marshal(stock)
				raw, _ = json.Marshal(request)
				return raw
			})
			if _, err := parseRegistration(raw, testSID); err == nil {
				t.Fatal("unsafe stock path accepted")
			}
		})
	}
}

func TestProtectedRegistrationPreservesExactOriginalEvidence(t *testing.T) {
	valid := testRegistration()
	raw := mutateProtectedClientRequest(valid, func(raw []byte) []byte { return append(append([]byte("\n "), raw...), '\n') })
	parsed, err := parseRegistrationAt(raw, testSID, clientRegistrationTestNow)
	if err != nil {
		t.Fatal(err)
	}
	if parsed.Schema != 2 || parsed.Installed.Version != "0.2.0-beta.1" || parsed.Installed.Commit != strings.Repeat("a", 40) || parsed.observedDeclaration() != stockBaseline {
		t.Fatal("protected evidence derivation differs")
	}
	var envelope protectedRegistration
	_ = json.Unmarshal(raw, &envelope)
	if envelope.ApprovedRequest[0] != '\n' || clientRequestDigest(envelope.ApprovedRequest) != envelope.ApprovedRequestSHA256 {
		t.Fatal("original approved bytes were compacted or identity lost")
	}
	if _, err = parseRegistrationAt(raw, testSID, time.Date(2037, 1, 1, 0, 0, 0, 0, time.UTC)); err == nil {
		t.Fatal("expired approved public root accepted at read")
	}
}

func TestProtectedRegistrationRejectsFormerSchema1Record(t *testing.T) {
	raw := testApprovedClientRequest()
	approved, err := brokerregistration.ParseApprovedRequest(raw, clientRequestDigest(raw), clientRegistrationTestNow)
	if err != nil {
		t.Fatal(err)
	}
	legacy := registrationFromApproved(approved)
	legacy.Schema = 1
	if _, err = parseRegistrationAt(clientFixtureJSON(legacy), testSID, clientRegistrationTestNow); err == nil {
		t.Fatal("formerly accepted schema1 declaration bypassed original approved evidence")
	}
}

func TestProtectedRegistrationRequiresCanonicalBase64Evidence(t *testing.T) {
	valid := testRegistration()
	var fields map[string]json.RawMessage
	if err := json.Unmarshal(valid, &fields); err != nil {
		t.Fatal(err)
	}
	var envelope protectedRegistration
	if err := json.Unmarshal(valid, &envelope); err != nil {
		t.Fatal(err)
	}
	values := make([]int, len(envelope.ApprovedRequest))
	for i, value := range envelope.ApprovedRequest {
		values[i] = int(value)
	}
	for name, encoded := range map[string][]byte{
		"numeric array": clientFixtureJSON(values),
		"null":          []byte("null"),
		"line break":    append(append(bytes.Clone(fields["approvedRequest"][:10]), []byte(`\n`)...), fields["approvedRequest"][10:]...),
	} {
		t.Run(name, func(t *testing.T) {
			fields["approvedRequest"] = encoded
			if _, err := parseRegistrationAt(clientFixtureJSON(fields), testSID, clientRegistrationTestNow); err == nil {
				t.Fatal("noncanonical protected evidence encoding accepted")
			}
		})
	}
}
