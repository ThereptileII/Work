package releasepolicy

import (
	"encoding/json"
	"strings"
	"testing"
)

func adversarialPolicyJSON(t *testing.T) []byte {
	t.Helper()
	artifact := func(path string) map[string]any {
		return map[string]any{
			"path": path, "url": "https://updates.example/" + path,
			"sha256": strings.Repeat("a", 64), "bytes": int64(1),
		}
	}
	p := map[string]any{
		"schema": 1, "product": "skager", "channel": "beta",
		"version": "1.2.3", "commit": strings.Repeat("b", 40),
		"releaseNotes":        artifact("releases/notes.txt"),
		"installer":           artifact("releases/setup.exe"),
		"recovery":            artifact("releases/recovery.zip"),
		"correspondingSource": artifact("source/source.zip"),
		"notices":             artifact("source/notices.txt"),
		"supportedOpenCpn": []any{map[string]any{
			"version": "5.12.4", "arch": "x86",
			"executableSha256": strings.Repeat("c", 64),
			"upstreamCommit":   strings.Repeat("d", 40),
		}},
	}
	b, err := json.Marshal(p)
	if err != nil {
		t.Fatal(err)
	}
	return b
}

func requireReviewParseReject(t *testing.T, data []byte) {
	t.Helper()
	if _, err := Parse(data); err == nil {
		t.Fatal("Parse accepted adversarial policy JSON")
	}
}

func TestReviewRejectsCaseFoldedFieldAliases(t *testing.T) {
	tests := []struct {
		name   string
		mutate func(map[string]any)
	}{
		{
			name:   "schema alias",
			mutate: func(p map[string]any) { p["Schema"] = 9 },
		},
		{
			name: "nested artifact digest alias",
			mutate: func(p map[string]any) {
				a := p["installer"].(map[string]any)
				a["SHA256"] = strings.Repeat("e", 64)
			},
		},
		{
			name: "unknown-case channel key",
			mutate: func(p map[string]any) {
				p["CHANNEL"] = p["channel"]
				delete(p, "channel")
			},
		},
	}
	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			var p map[string]any
			if err := json.Unmarshal(adversarialPolicyJSON(t), &p); err != nil {
				t.Fatal(err)
			}
			tc.mutate(p)
			b, err := json.Marshal(p)
			if err != nil {
				t.Fatal(err)
			}
			requireReviewParseReject(t, b)
		})
	}
}

func TestReviewRejectsUnsafeWindowsDeviceArtifactPath(t *testing.T) {
	var p map[string]any
	if err := json.Unmarshal(adversarialPolicyJSON(t), &p); err != nil {
		t.Fatal(err)
	}
	installer := p["installer"].(map[string]any)
	installer["path"] = "releases/CON.txt"
	installer["url"] = "https://updates.example/releases/CON.txt"
	b, err := json.Marshal(p)
	if err != nil {
		t.Fatal(err)
	}
	requireReviewParseReject(t, b)
}

func TestReviewRejectsDuplicateNestedKey(t *testing.T) {
	b := string(adversarialPolicyJSON(t))
	needle := `"sha256":"` + strings.Repeat("a", 64) + `"`
	if !strings.Contains(b, needle) {
		t.Fatal("fixture is missing expected nested digest")
	}
	b = strings.Replace(b, needle, needle+`,"sha256":"`+strings.Repeat("e", 64)+`"`, 1)
	requireReviewParseReject(t, []byte(b))
}

func TestReviewRejectsExcessiveJSONNesting(t *testing.T) {
	// Keep the input below the policy byte cap while exceeding encoding/json's
	// maximum nesting depth. Parsing must fail without panic or runaway recursion.
	const depth = 10001
	b := []byte(strings.Repeat("[", depth) + "0" + strings.Repeat("]", depth))
	if len(b) >= maxPolicyBytes {
		t.Fatal("deep-nesting fixture unexpectedly exceeds policy byte cap")
	}
	requireReviewParseReject(t, b)
}

func TestReviewRejectsMalformedAndTrailingJSON(t *testing.T) {
	valid := adversarialPolicyJSON(t)
	for name, data := range map[string][]byte{
		"malformed":       []byte(`{"schema":`),
		"trailing object": append(append([]byte(nil), valid...), []byte(` {}`)...),
		"trailing scalar": append(append([]byte(nil), valid...), []byte(` true`)...),
	} {
		t.Run(name, func(t *testing.T) { requireReviewParseReject(t, data) })
	}
}
