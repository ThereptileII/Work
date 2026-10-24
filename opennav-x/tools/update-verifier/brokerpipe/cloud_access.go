package brokerpipe

import (
	"bytes"
	"encoding/json"
	"fmt"
)

// CloudAccess is optional download permission, never update, execution or trust
// authority. Its zero value is absent. Generic formatting and JSON cannot expose
// its value; only the private authenticated wire encoders and explicit Token use
// for protected-origin HTTPS requests may disclose it. Do not persist this type.
type CloudAccess struct{ value string }

// ParseCloudAccess accepts exactly one pairing-issued ASCII device token. A
// missing credential uses CloudAccess{}; an explicitly empty value is invalid.
func ParseCloudAccess(token string) (CloudAccess, error) {
	if len(token) != 69 || token[:5] != "skg1_" {
		return CloudAccess{}, ErrRefused
	}
	for i := 5; i < len(token); i++ {
		if !(token[i] >= '0' && token[i] <= '9' || token[i] >= 'a' && token[i] <= 'f') {
			return CloudAccess{}, ErrRefused
		}
	}
	return CloudAccess{value: token}, nil
}

func (a CloudAccess) IsPresent() bool { return a.value != "" }

// Token deliberately reveals the credential for an authenticated pipe request
// or a request to the independently protected update origin, never diagnostics.
func (a CloudAccess) Token() string  { return a.value }
func (CloudAccess) String() string   { return "[redacted cloud access]" }
func (CloudAccess) GoString() string { return "[redacted cloud access]" }
func (CloudAccess) Format(state fmt.State, _ rune) {
	// Unsupported formatting verbs must not fall back to reflecting private fields.
	_, _ = state.Write([]byte("[redacted cloud access]"))
}
func (CloudAccess) MarshalJSON() ([]byte, error) { return []byte("null"), nil }
func (*CloudAccess) UnmarshalJSON([]byte) error  { return ErrRefused }

func (a CloudAccess) valid() bool {
	if !a.IsPresent() {
		return true
	}
	_, err := ParseCloudAccess(a.value)
	return err == nil
}

// The deliberately optional wire member has a single unescaped ASCII spelling.
// Null, explicit empty strings, Unicode aliases and arbitrary JSON are refused.
func decodeCloudAccess(raw json.RawMessage) (CloudAccess, error) {
	raw = bytes.TrimSpace(raw)
	if len(raw) != 71 || raw[0] != '"' || raw[len(raw)-1] != '"' {
		return CloudAccess{}, ErrRefused
	}
	return ParseCloudAccess(string(raw[1 : len(raw)-1]))
}
