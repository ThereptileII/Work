package brokerpipe

import (
	"bytes"
	"encoding/json"
	"strings"
	"testing"
)

func TestExecutionStartupModeCanonicalStrictWire(t *testing.T) {
	for _, operation := range []string{"start-selected", "start-current"} {
		for _, mode := range []StartupMode{StartupXNav, StartupLegacy, StartupSafe} {
			q := executionTestRequest()
			q.Operation = operation
			q.StartupMode = mode
			raw, err := encodeExecutionRequest(q)
			if err != nil {
				t.Fatal(err)
			}
			if bytes.Contains(raw, []byte("startupMode")) != (mode != StartupXNav) {
				t.Fatalf("noncanonical mode encoding: %s", raw)
			}
			if got, e := decodeExecutionRequest(raw); e != nil || got != q {
				t.Fatalf("got=%+v err=%v", got, e)
			}
		}
	}
	q := executionTestRequest()
	q.Operation = "start-current"
	raw, _ := encodeExecutionRequest(q)
	for _, field := range []string{`"startupMode":""`, `"startupMode":"xnav"`, `"startupMode":null`, `"startupMode":1`, `"startupMode":"Legacy"`, `"startupMode":"safe-mode"`, `"startupMode":"--legacy"`, `"startupMode":"legacy "`, `"StartupMode":"legacy"`, `"startupMode":"legacy","startupMode":"safe"`, `"startupMode":"legacy","args":"--safe-mode"`, `"startupMode":"\u006cegacy"`} {
		bad := append(bytes.TrimSuffix(raw, []byte("}")), []byte(","+field+"}")...)
		if _, e := decodeExecutionRequest(bad); e == nil {
			t.Fatalf("ambiguous mode accepted: %s", bad)
		}
	}
	for _, operation := range []string{"execute-session", "offer-current"} {
		q.Operation = operation
		q.StartupMode = StartupLegacy
		if _, e := encodeExecutionRequest(q); e == nil {
			t.Fatal("qualification/offer mode accepted")
		}
	}
}
func TestExecutionStartupModeNormalPhaseAndReportBinding(t *testing.T) {
	for _, mode := range []StartupMode{StartupXNav, StartupLegacy, StartupSafe} {
		for _, phase := range []ExecutionPhase{RelaySelectedExecutor, SpawnCurrent, ObserveCurrent, CloseCurrent} {
			c := executionTestCommand()
			c.Phase = phase
			c.ApplicationGeneration = c.StageID
			c.StartupMode = mode
			raw, _ := json.Marshal(c)
			if got, e := decodeExecutionCommand(raw, "command"); e != nil || got != c {
				t.Fatalf("normal mode refused: %v", e)
			}
			report, e := encodeExecutionReport(c, executionTestObservation())
			if e != nil {
				t.Fatal(e)
			}
			if _, e := decodeExecutionReport(report, c); e != nil {
				t.Fatal(e)
			}
			changed := c
			changed.StartupMode = StartupSafe
			if mode == StartupSafe {
				changed.StartupMode = StartupLegacy
			}
			changed.Kind = "report"
			wrong, _ := json.Marshal(executionReport{changed, executionTestObservation()})
			if _, e := decodeExecutionReport(wrong, c); e == nil {
				t.Fatal("report changed original mode")
			}
		}
	}
	for _, phase := range []ExecutionPhase{InspectPrevious, RestorePrevious, CommissionCandidate, SpawnCandidate, ObserveCandidate, CloseCandidate, CommissionFallback, SpawnFallback, ObserveFallback, CloseFallback, RestoreProfile, OfferUpdate, FinishExecution, RelayStartupExecutor} {
		c := executionTestCommand()
		c.Phase = phase
		c.StartupMode = StartupLegacy
		if validExecutionDirective(c.directive(), c.StageID) {
			t.Fatalf("mode entered nonnormal phase %s", phase)
		}
	}
}
func TestExecutionStartupModeClientSequenceRejectsSubstitution(t *testing.T) {
	for _, mode := range []StartupMode{StartupXNav, StartupLegacy, StartupSafe} {
		q := executionTestRequest()
		q.Operation = "start-current"
		q.StartupMode = mode
		c := executionTestCommand()
		c.Phase = SpawnCurrent
		c.ApplicationGeneration = q.StageID
		c.StartupMode = mode
		state := &executionClientSequence{request: q}
		if err := state.accept(c); err != nil {
			t.Fatal(err)
		}
		c.Sequence++
		c.Challenge = strings.Repeat("f", 32)
		c.Phase = ObserveCurrent
		c.StartupMode = StartupLegacy
		if mode == StartupLegacy {
			c.StartupMode = StartupSafe
		}
		if err := state.accept(c); err == nil {
			t.Fatal("sequence accepted mode substitution")
		}
	}
}
