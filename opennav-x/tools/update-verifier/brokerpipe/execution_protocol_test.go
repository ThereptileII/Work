package brokerpipe

import (
	"encoding/json"
	"fmt"
	"strings"
	"testing"
)

func executionTestRequest() ExecutionRequest {
	return ExecutionRequest{Schema: 1, Operation: "execute-session", RequestID: strings.Repeat("a", 32), StageID: strings.Repeat("b", 32)}
}
func executionTestCommand() ExecutionCommand {
	q := executionTestRequest()
	return ExecutionCommand{Schema: 1, Kind: "command", RequestID: q.RequestID, StageID: q.StageID, SessionID: strings.Repeat("c", 32), Sequence: 1, Challenge: strings.Repeat("d", 32), Phase: InspectPrevious}
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

func executionTestDirective(phase ExecutionPhase) ExecutionDirective {
	d := ExecutionDirective{Phase: phase}
	switch phase {
	case SpawnCandidate, ObserveCandidate:
		d.ApplicationGeneration = executionTestRequest().StageID
		d.HealthPipe = "Skager.Update." + strings.Repeat("f", 32)
		d.HealthChallenge = strings.Repeat("1", 64)
	case SpawnFallback, ObserveFallback:
		d.ApplicationGeneration = strings.Repeat("f", 32)
		d.HealthPipe = "Skager.Update." + strings.Repeat("2", 32)
		d.HealthChallenge = strings.Repeat("3", 64)
	}
	return d
}
func TestExecutionCommandClosedGenerationAndHealthPayload(t *testing.T) {
	for _, phase := range []ExecutionPhase{SpawnCandidate, ObserveCandidate, SpawnFallback, ObserveFallback} {
		t.Run(string(phase), func(t *testing.T) {
			command := executionTestCommand()
			directive := executionTestDirective(phase)
			command.Phase = phase
			command.ApplicationGeneration = directive.ApplicationGeneration
			command.HealthPipe = directive.HealthPipe
			command.HealthChallenge = directive.HealthChallenge
			raw, _ := json.Marshal(command)
			got, err := decodeExecutionCommand(raw, "command")
			if err != nil || got != command {
				t.Fatal("closed directive refused", err)
			}
			report, err := encodeExecutionReport(command, executionTestObservation())
			if err != nil {
				t.Fatal(err)
			}
			if _, err := decodeExecutionReport(report, command); err != nil {
				t.Fatal("exact payload echo refused", err)
			}
			for _, name := range []string{"no-generation", "uppercase-generation", "wrong-lineage", "no-pipe", "raw-pipe-path", "pipe-alias", "pipe-suffix", "no-challenge", "short-challenge", "payload-replaced"} {
				t.Run(name, func(t *testing.T) {
					c := command
					switch name {
					case "no-generation":
						c.ApplicationGeneration = ""
					case "uppercase-generation":
						c.ApplicationGeneration = strings.Repeat("B", 32)
					case "wrong-lineage":
						if phase == SpawnCandidate || phase == ObserveCandidate {
							c.ApplicationGeneration = strings.Repeat("f", 32)
						} else {
							c.ApplicationGeneration = c.StageID
						}
					case "no-pipe":
						c.HealthPipe = ""
					case "raw-pipe-path":
						c.HealthPipe = `\\.\pipe\` + c.HealthPipe
					case "pipe-alias":
						c.HealthPipe = strings.ToLower(c.HealthPipe)
					case "pipe-suffix":
						c.HealthPipe += ".extra"
					case "no-challenge":
						c.HealthChallenge = ""
					case "short-challenge":
						c.HealthChallenge = strings.Repeat("1", 63)
					case "payload-replaced":
						c.HealthChallenge = strings.Repeat("4", 64)
					}
					c.Kind = "report"
					b, _ := json.Marshal(executionReport{c, executionTestObservation()})
					if _, err := decodeExecutionReport(b, command); err == nil {
						t.Fatal("altered payload report accepted")
					}
				})
			}
		})
	}
	for _, phase := range []ExecutionPhase{InspectPrevious, RestorePrevious, CommissionCandidate, CloseCandidate, CommissionFallback, CloseFallback, RestoreProfile, FinishExecution} {
		t.Run("no-health-"+string(phase), func(t *testing.T) {
			d := ExecutionDirective{Phase: phase}
			if !validExecutionDirective(d, executionTestRequest().StageID) {
				t.Fatal("no-payload phase refused")
			}
			d.HealthPipe = "Skager.Update." + strings.Repeat("a", 32)
			if validExecutionDirective(d, executionTestRequest().StageID) {
				t.Fatal("health accepted outside spawn/observe")
			}
			d.HealthPipe = ""
			d.HealthChallenge = strings.Repeat("b", 64)
			if validExecutionDirective(d, executionTestRequest().StageID) {
				t.Fatal("challenge accepted outside spawn/observe")
			}
		})
	}
	finish := ExecutionDirective{Phase: FinishExecution, ApplicationGeneration: executionTestRequest().StageID}
	if validExecutionDirective(finish, executionTestRequest().StageID) {
		t.Fatal("finish accepted business payload")
	}
	for _, phase := range []ExecutionPhase{CommissionCandidate, CloseCandidate, CommissionFallback, CloseFallback} {
		d := ExecutionDirective{Phase: phase, ApplicationGeneration: executionTestRequest().StageID}
		want := phase == CommissionCandidate || phase == CloseCandidate
		if validExecutionDirective(d, executionTestRequest().StageID) != want {
			t.Fatal("optional generation lineage ignored", phase)
		}
	}
}

func TestExecutionCurrentNormalDirectiveClosedGenerationWithoutHealth(t *testing.T) {
	for _, phase := range []ExecutionPhase{SpawnCurrent, ObserveCurrent, CloseCurrent} {
		t.Run(string(phase), func(t *testing.T) {
			for _, generation := range []string{executionTestRequest().StageID, strings.Repeat("9", 32)} {
				command := executionTestCommand()
				command.Phase, command.ApplicationGeneration = phase, generation
				raw, _ := json.Marshal(command)
				if got, err := decodeExecutionCommand(raw, "command"); err != nil || got != command {
					t.Fatal("normal generation refused", err)
				}
				encoded, err := encodeExecutionReport(command, executionTestObservation())
				if err != nil {
					t.Fatal(err)
				}
				if _, err = decodeExecutionReport(encoded, command); err != nil {
					t.Fatal(err)
				}
				altered := command
				altered.Kind = "report"
				altered.ApplicationGeneration = strings.Repeat("8", 32)
				wrong, _ := json.Marshal(executionReport{altered, executionTestObservation()})
				if _, err = decodeExecutionReport(wrong, command); err == nil {
					t.Fatal("normal report switched original generation")
				}
			}
			for _, mode := range []string{"no-generation", "uppercase", "path", "health-pipe", "health-challenge", "both-health", "unknown-selected-field"} {
				t.Run(mode, func(t *testing.T) {
					command := executionTestCommand()
					command.Phase, command.ApplicationGeneration = phase, strings.Repeat("9", 32)
					switch mode {
					case "no-generation":
						command.ApplicationGeneration = ""
					case "uppercase":
						command.ApplicationGeneration = strings.Repeat("A", 32)
					case "path":
						command.ApplicationGeneration = "../current"
					case "health-pipe":
						command.HealthPipe = "Skager.Update." + strings.Repeat("6", 32)
					case "health-challenge":
						command.HealthChallenge = strings.Repeat("7", 64)
					case "both-health":
						command.HealthPipe = "Skager.Update." + strings.Repeat("6", 32)
						command.HealthChallenge = strings.Repeat("7", 64)
					}
					raw, _ := json.Marshal(command)
					if mode == "unknown-selected-field" {
						raw = []byte(strings.Replace(string(raw), `"schema":1`, `"schema":1,"selected":true`, 1))
					}
					if _, err := decodeExecutionCommand(raw, "command"); err == nil {
						t.Fatal("unsafe normal command accepted")
					}
				})
			}
		})
	}
}
