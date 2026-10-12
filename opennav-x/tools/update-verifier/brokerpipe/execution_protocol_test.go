package brokerpipe

import (
	"encoding/json"
	"fmt"
	"strings"
	"testing"
)

func executionTestRequest() ExecutionRequest {
	return ExecutionRequest{1, "execute-session", strings.Repeat("a", 32), strings.Repeat("b", 32)}
}
func executionTestCommand() ExecutionCommand {
	q := executionTestRequest()
	return ExecutionCommand{1, "command", q.RequestID, q.StageID, strings.Repeat("c", 32), 1, strings.Repeat("d", 32), InspectPrevious}
}
func executionTestObservation() ExecutionObservation {
	return ExecutionObservation{Status: "reported-complete", EvidenceSHA256: strings.Repeat("e", 64)}
}

func TestExecutionRequestStrictBoundary(t *testing.T) {
	raw, err := encodeExecutionRequest(executionTestRequest())
	if err != nil {
		t.Fatal(err)
	}
	if got, err := decodeExecutionRequest(raw); err != nil || got != executionTestRequest() {
		t.Fatal(got, err)
	}
	for name, mutate := range map[string]func(string) string{
		"duplicate":       func(s string) string { return strings.Replace(s, `"schema":1`, `"schema":1,"schema":1`, 1) },
		"alias":           func(s string) string { return strings.Replace(s, `"schema"`, `"Schema"`, 1) },
		"unknown-command": func(s string) string { return strings.Replace(s, `"schema":1`, `"schema":1,"command":"run"`, 1) },
		"null":            func(s string) string { return strings.Replace(s, `"schema":1`, `"schema":null`, 1) },
		"fraction":        func(s string) string { return strings.Replace(s, `"schema":1`, `"schema":1.0`, 1) },
		"other-operation": func(s string) string { return strings.Replace(s, "execute-session", "prepare-and-stage", 1) },
		"uppercase-id":    func(s string) string { return strings.Replace(s, strings.Repeat("a", 32), strings.Repeat("A", 32), 1) },
		"trailing-object": func(s string) string { return s + "{}" },
		"oversized":       func(s string) string { return strings.Repeat(" ", maxFrame) + s },
		"invalid-utf8":    func(s string) string { return s + string([]byte{255}) },
	} {
		t.Run(name, func(t *testing.T) {
			if _, err := decodeExecutionRequest([]byte(mutate(string(raw)))); err == nil {
				t.Fatal("untrusted request accepted")
			}
		})
	}
}
func TestExecutionReportExactOutstandingChallenge(t *testing.T) {
	command := executionTestCommand()
	observed := executionTestObservation()
	raw, err := encodeExecutionReport(command, observed)
	if err != nil {
		t.Fatal(err)
	}
	if got, err := decodeExecutionReport(raw, command); err != nil || got != observed {
		t.Fatal(got, err)
	}
	for name, mutate := range map[string]func(*executionReport){
		"wrong-request":     func(r *executionReport) { r.RequestID = strings.Repeat("f", 32) },
		"wrong-stage":       func(r *executionReport) { r.StageID = strings.Repeat("f", 32) },
		"wrong-session":     func(r *executionReport) { r.SessionID = strings.Repeat("f", 32) },
		"future-sequence":   func(r *executionReport) { r.Sequence++ },
		"old-sequence":      func(r *executionReport) { r.Sequence = 0 },
		"guessed-challenge": func(r *executionReport) { r.Challenge = strings.Repeat("f", 32) },
		"wrong-phase":       func(r *executionReport) { r.Phase = SpawnCandidate },
		"unknown-phase":     func(r *executionReport) { r.Phase = "run-command" },
		"command-as-report": func(r *executionReport) { r.Kind = "command" },
		"health-status":     func(r *executionReport) { r.Status = "healthy" },
		"missing-evidence":  func(r *executionReport) { r.EvidenceSHA256 = "" },
		"partial-process":   func(r *executionReport) { r.ProcessID = 99 },
		"partial-creation":  func(r *executionReport) { r.ProcessCreation = 999 },
	} {
		t.Run(name, func(t *testing.T) {
			r := executionReport{command, observed}
			r.Kind = "report"
			mutate(&r)
			b, _ := json.Marshal(r)
			if _, err := decodeExecutionReport(b, command); err == nil {
				t.Fatal("unrequested report accepted")
			}
		})
	}
	for name, mutate := range map[string]func(string) string{
		"duplicate": func(s string) string { return strings.Replace(s, `"sequence":1`, `"sequence":1,"sequence":1`, 1) },
		"unknown":   func(s string) string { return strings.Replace(s, `"schema":1`, `"schema":1,"path":"C:/x"`, 1) },
		"null-pid":  func(s string) string { return strings.Replace(s, `"processId":0`, `"processId":null`, 1) },
		"alias":     func(s string) string { return strings.Replace(s, `"phase"`, `"Phase"`, 1) },
	} {
		t.Run(name, func(t *testing.T) {
			if _, err := decodeExecutionReport([]byte(mutate(string(raw))), command); err == nil {
				t.Fatal("malformed report accepted")
			}
		})
	}
}
func TestExecutionClientSequenceAndClose(t *testing.T) {
	initial := executionTestCommand()
	for _, name := range []string{"duplicate", "out-of-order", "session-switch", "challenge-reuse", "wrong-request", "after-finish"} {
		t.Run(name, func(t *testing.T) {
			s := executionClientSequence{request: executionTestRequest()}
			if s.accept(initial) != nil {
				t.Fatal("first command refused")
			}
			c := initial
			c.Sequence = 2
			c.Challenge = strings.Repeat("f", 32)
			switch name {
			case "duplicate":
				c = initial
			case "out-of-order":
				c.Sequence = 3
			case "session-switch":
				c.SessionID = strings.Repeat("f", 32)
			case "challenge-reuse":
				c.Challenge = initial.Challenge
			case "wrong-request":
				c.RequestID = strings.Repeat("f", 32)
			case "after-finish":
				c.Phase = FinishExecution
				if s.accept(c) != nil {
					t.Fatal("finish refused")
				}
				c.Sequence++
				c.Challenge = strings.Repeat("1", 32)
			}
			if s.accept(c) == nil {
				t.Fatal("invalid command sequence accepted")
			}
		})
	}
	s := executionClientSequence{request: executionTestRequest()}
	for i := 1; i <= maxExecutionPhases; i++ {
		c := initial
		c.Sequence = uint32(i)
		c.Challenge = fmt.Sprintf("%032x", i)
		if i == maxExecutionPhases {
			c.Phase = FinishExecution
		}
		if s.accept(c) != nil {
			t.Fatal("explicit finite command refused", i)
		}
	}
	c := initial
	c.Sequence = maxExecutionPhases + 1
	c.Challenge = strings.Repeat("2", 32)
	if s.accept(c) == nil {
		t.Fatal("unbounded conversation accepted")
	}
	finish := initial
	finish.Phase = FinishExecution
	closeRaw, err := executionClosingFrame(finish, "closed")
	if err != nil || !validExecutionClosingFrame(closeRaw, finish, "closed") {
		t.Fatal(err)
	}
	if validExecutionClosingFrame(closeRaw, finish, "closed-ack") {
		t.Fatal("close accepted as ack")
	}
	finish.Challenge = strings.Repeat("f", 32)
	if validExecutionClosingFrame(closeRaw, finish, "closed") {
		t.Fatal("other finish accepted")
	}
}
