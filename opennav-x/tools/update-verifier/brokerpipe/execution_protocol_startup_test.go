package brokerpipe

import (
	"bytes"
	"encoding/json"
	"strings"
	"testing"
)

func TestExecutionStartupAdmissionExactRequest(t *testing.T) {
	q := executionTestRequest()
	q.Operation = "start-selected"
	raw, err := encodeExecutionRequest(q)
	if err != nil {
		t.Fatal(err)
	}
	if got, err := decodeExecutionRequest(raw); err != nil || got != q {
		t.Fatal(got, err)
	}
	for _, fault := range []string{"case-operation", "trailing-operation", "unknown-operation", "null", "duplicate", "key-alias", "extra", "stage-case", "stage-path", "request-missing"} {
		t.Run(fault, func(t *testing.T) {
			bad := bytes.Clone(raw)
			switch fault {
			case "case-operation":
				bad = bytes.Replace(bad, []byte("start-selected"), []byte("Start-selected"), 1)
			case "trailing-operation":
				bad = bytes.Replace(bad, []byte("start-selected"), []byte("start-selected "), 1)
			case "unknown-operation":
				bad = bytes.Replace(bad, []byte("start-selected"), []byte("start-or-recover"), 1)
			case "null":
				bad = bytes.Replace(bad, []byte(`"operation":"start-selected"`), []byte(`"operation":null`), 1)
			case "duplicate":
				bad = bytes.Replace(bad, []byte(`"schema":1`), []byte(`"schema":1,"operation":"start-selected"`), 1)
			case "key-alias":
				bad = bytes.Replace(bad, []byte(`"operation"`), []byte(`"Operation"`), 1)
			case "extra":
				bad = bytes.Replace(bad, []byte(`"schema":1`), []byte(`"schema":1,"selected":true`), 1)
			case "stage-case":
				copy := q
				copy.StageID = strings.Repeat("A", 32)
				bad, _ = json.Marshal(copy)
			case "stage-path":
				copy := q
				copy.StageID = "../app"
				bad, _ = json.Marshal(copy)
			case "request-missing":
				copy := q
				copy.RequestID = ""
				bad, _ = json.Marshal(copy)
			}
			if _, err := decodeExecutionRequest(bad); err == nil {
				t.Fatal("ambiguous startup admission accepted", string(bad))
			}
		})
	}
}
