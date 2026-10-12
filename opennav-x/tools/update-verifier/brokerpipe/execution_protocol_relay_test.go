package brokerpipe

import (
	"encoding/json"
	"strings"
	"testing"
)

func TestExecutionStartCurrentRequestStrictIntent(t *testing.T) {
	q := executionTestRequest()
	q.Operation = "start-current"
	raw, err := encodeExecutionRequest(q)
	if err != nil {
		t.Fatal(err)
	}
	if got, err := decodeExecutionRequest(raw); err != nil || got != q {
		t.Fatal(got, err)
	}
	for _, operation := range []string{"Start-current", "start-current ", "relay-selected-executor", "", "start_current"} {
		q.Operation = operation
		b, _ := json.Marshal(q)
		if _, err := decodeExecutionRequest(b); err == nil {
			t.Fatal("unknown operation accepted", operation)
		}
	}
	for _, b := range []string{strings.Replace(string(raw), `"operation":"start-current"`, `"operation":null`, 1), strings.Replace(string(raw), `"operation"`, `"Operation"`, 1), strings.Replace(string(raw), `"schema":1`, `"schema":1,"path":"C:/x"`, 1), strings.Replace(string(raw), `"schema":1`, `"schema":1,"operation":"start-current"`, 1)} {
		if _, err := decodeExecutionRequest([]byte(b)); err == nil {
			t.Fatal("open current schema")
		}
	}
}
func TestExecutionRelayDirectiveBindsOnlyCanonicalLookup(t *testing.T) {
	c := executionTestCommand()
	c.Phase = RelaySelectedExecutor
	c.ApplicationGeneration = strings.Repeat("e", 32)
	raw, _ := json.Marshal(c)
	if _, err := decodeExecutionCommand(raw, "command"); err != nil {
		t.Fatal(err)
	}
	for _, mode := range []string{"missing", "alias", "path", "pipe", "challenge", "command"} {
		t.Run(mode, func(t *testing.T) {
			bad := c
			switch mode {
			case "missing":
				bad.ApplicationGeneration = ""
			case "alias":
				bad.ApplicationGeneration = strings.Repeat("E", 32)
			case "path":
				bad.ApplicationGeneration = "../app"
			case "pipe":
				bad.HealthPipe = "Skager.Update." + strings.Repeat("f", 32)
			case "challenge":
				bad.HealthChallenge = strings.Repeat("f", 64)
			case "command":
				bad.Phase = "relay-command"
			}
			b, _ := json.Marshal(bad)
			if _, err := decodeExecutionCommand(b, "command"); err == nil {
				t.Fatal("unsafe relay payload")
			}
		})
	}
	// Neither equality nor inequality with caller stage is selection authority.
	c.ApplicationGeneration = c.StageID
	raw, _ = json.Marshal(c)
	if _, err := decodeExecutionCommand(raw, "command"); err != nil {
		t.Fatal(err)
	}
}
