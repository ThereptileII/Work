package brokerhost

import (
	"bytes"
	"encoding/json"
	"strings"
	"testing"
)

func TestProtectedRegistrationRejectsAmbiguousAndUnapprovedInput(t *testing.T) {
	valid := testRegistration()
	if _, err := parseRegistration(valid, testSID); err != nil {
		t.Fatal(err)
	}
	cases := map[string][]byte{
		"duplicate":        bytes.Replace(valid, []byte(`"schema":1`), []byte(`"schema":1,"schema":1`), 1),
		"case folding":     bytes.Replace(valid, []byte(`"clientSid"`), []byte(`"ClientSid"`), 1),
		"unknown":          bytes.Replace(valid, []byte(`"schema":1`), []byte(`"schema":1,"execute":true`), 1),
		"nested duplicate": bytes.Replace(valid, []byte(`"signed":{}`), []byte(`"signed":{"x":1,"x":2}`), 1),
		"trailing":         append(append([]byte(nil), valid...), []byte(`{}`)...),
		"invalid UTF8":     append([]byte{0xff}, valid...),
		"oversize":         bytes.Repeat([]byte{' '}, 1<<20+1),
		"null root":        bytes.Replace(valid, []byte(`{"signed":{},"signatures":[]}`), []byte(`null`), 1),
		"wrong stock hash": bytes.Replace(valid, []byte(stockBaseline.ExecutableSHA256), []byte(strings.Repeat("0", 64)), 1),
		"wrong arch":       bytes.Replace(valid, []byte(`"arch":"x86"`), []byte(`"arch":"x64"`), 1),
		"wrong peer":       bytes.Replace(valid, []byte(testSID), []byte(testSID+"1"), 1),
		"untrusted URL":    bytes.Replace(valid, []byte("https://updates.invalid/metadata/"), []byte("http://updates.invalid/metadata/"), 1),
		"origin with path": bytes.Replace(valid, []byte(`"artifactOrigin":"https://updates.invalid"`), []byte(`"artifactOrigin":"https://updates.invalid/artifacts"`), 1),
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
			var r registration
			_ = json.Unmarshal(testRegistration(), &r)
			r.Stock.Path = path
			b, _ := json.Marshal(r)
			if _, err := parseRegistration(b, testSID); err == nil {
				t.Fatal("unsafe stock path accepted")
			}
		})
	}
}
