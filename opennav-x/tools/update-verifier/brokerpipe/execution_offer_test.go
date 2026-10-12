package brokerpipe

import (
	"context"
	"encoding/json"
	"strings"
	"testing"
)

func executionOfferDirective() ExecutionDirective {
	return ExecutionDirective{Phase: OfferUpdate, ApplicationGeneration: strings.Repeat("b", 32), OfferVersion: "2.0.0", OfferCommit: strings.Repeat("a", 40), OfferPolicySHA256: strings.Repeat("c", 64), PromptBytes: 123, PromptSHA256: strings.Repeat("d", 64)}
}
func TestExecutionOfferClosedPayloadAndEcho(t *testing.T) {
	s, _, connection := executionTestSession(t)
	bindExecutionTestSession(t, s)
	d := executionOfferDirective()
	if _, err := s.ExchangeCommand(context.Background(), d); err != nil {
		t.Fatal(err)
	}
	c := connection.commands[0]
	if c.directive() != d {
		t.Fatal("producer dropped offer")
	}
	raw, err := encodeExecutionReport(c, executionTestObservation())
	if err != nil {
		t.Fatal(err)
	}
	for _, key := range []string{"offerVersion", "offerCommit", "offerPolicySha256", "promptBytes", "promptSha256"} {
		t.Run(key, func(t *testing.T) {
			var data map[string]any
			json.Unmarshal(raw, &data)
			if key == "promptBytes" {
				data[key] = 124
			} else {
				data[key] = "changed"
			}
			changed, _ := json.Marshal(data)
			if _, err := decodeExecutionReport(changed, c); err == nil {
				t.Fatal("changed echo accepted")
			}
		})
	}
	for name, change := range map[string]func(*ExecutionDirective){
		"wrong-generation": func(d *ExecutionDirective) { d.ApplicationGeneration = strings.Repeat("e", 32) }, "zero-size": func(d *ExecutionDirective) { d.PromptBytes = 0 }, "large": func(d *ExecutionDirective) { d.PromptBytes = 512<<20 + 1 }, "bad-version": func(d *ExecutionDirective) { d.OfferVersion = "offer\n" }, "bad-commit": func(d *ExecutionDirective) { d.OfferCommit = strings.Repeat("A", 40) }, "bad-policy": func(d *ExecutionDirective) { d.OfferPolicySHA256 = "" }, "bad-helper": func(d *ExecutionDirective) { d.PromptSHA256 = "" }, "health": func(d *ExecutionDirective) { d.HealthPipe = "Skager.Update." + strings.Repeat("1", 32) }, "payload-on-finish": func(d *ExecutionDirective) { d.Phase = FinishExecution }, "payload-on-spawn": func(d *ExecutionDirective) { d.Phase = SpawnCurrent },
	} {
		t.Run(name, func(t *testing.T) {
			v := d
			change(&v)
			if validExecutionDirective(v, strings.Repeat("b", 32)) {
				t.Fatal("unsafe payload accepted")
			}
		})
	}
}
func TestExecutionOfferIntentAndStartupRelay(t *testing.T) {
	q := executionTestRequest()
	q.Operation = "offer-current"
	raw, err := encodeExecutionRequest(q)
	if err != nil {
		t.Fatal(err)
	}
	if got, err := decodeExecutionRequest(raw); err != nil || got != q {
		t.Fatal(got, err)
	}
	d := ExecutionDirective{Phase: RelayStartupExecutor, ApplicationGeneration: q.StageID}
	if !validExecutionDirective(d, q.StageID) {
		t.Fatal("startup relay rejected")
	}
	d.PromptSHA256 = strings.Repeat("d", 64)
	if validExecutionDirective(d, q.StageID) {
		t.Fatal("relay acquired offer payload")
	}
}
