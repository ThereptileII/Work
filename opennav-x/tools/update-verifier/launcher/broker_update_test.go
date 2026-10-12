package launcher

import (
	"context"
	"errors"
	verifier "example.com/opennav-update-verifier"
	"example.com/opennav-update-verifier/brokerpipe"
	"reflect"
	"strings"
	"testing"
)

func updateCommand(q brokerpipe.ExecutionRequest, p brokerpipe.ExecutionPhase) brokerpipe.ExecutionCommand {
	c := brokerpipe.ExecutionCommand{Schema: 1, Kind: "command", RequestID: q.RequestID, StageID: q.StageID, Phase: p}
	if p == brokerpipe.OfferUpdate {
		c.ApplicationGeneration = q.StageID
		c.OfferVersion = "2.0.0"
		c.OfferCommit = strings.Repeat("c", 40)
		c.OfferPolicySHA256 = strings.Repeat("d", 64)
		c.PromptBytes = 123
		c.PromptSHA256 = strings.Repeat("e", 64)
	}
	return c
}

func TestBrokerUpdatePromptFailureCannotBecomeLater(t *testing.T) {
	for _, code := range []uint32{0, 10, 1, 2, 0xc0000005, ^uint32(0)} {
		now, err := brokerUpdatePromptResult(code)
		if code == 0 || code == 10 {
			if err != nil || now != (code == 10) {
				t.Fatalf("explicit prompt choice %d: now=%v err=%v", code, now, err)
			}
		} else if now || !errors.Is(err, errBrokerExecutionUncertain) {
			t.Fatalf("failed/crashed prompt became Later/Now: %d now=%v err=%v", code, now, err)
		}
	}
}
func TestBrokerUpdateNowStagesAfterFinishAndDrain(t *testing.T) {
	var order []string
	var offerID string
	consent := verifier.ConsentIdentity{Version: "2.0.0", Commit: strings.Repeat("c", 40), PolicySHA256: strings.Repeat("d", 64)}
	s := brokerUpdateServices{conversation: func(ctx context.Context, q brokerpipe.ExecutionRequest, h brokerpipe.ExecutionClientHandler) error {
		offerID = q.RequestID
		if q.Operation != "offer-current" {
			t.Fatal(q)
		}
		for _, p := range []brokerpipe.ExecutionPhase{brokerpipe.OfferUpdate, brokerpipe.FinishExecution} {
			if _, err := h(ctx, updateCommand(q, p)); err != nil {
				return err
			}
			order = append(order, string(p))
		}
		return nil
	}, prompt: func(context.Context, brokerpipe.ExecutionCommand) (bool, error) {
		order = append(order, "prompt")
		return true, nil
	}, close: func() error { order = append(order, "drain"); return nil }, stage: func(_ context.Context, q brokerpipe.Request) (brokerpipe.Result, error) {
		order = append(order, "stage")
		if q.Consent != consent || q.RequestID == offerID || !generationPattern.MatchString(q.RequestID) || q.Operation != "prepare-and-stage" {
			t.Fatal(q)
		}
		return brokerpipe.Result{Schema: 1, RequestID: q.RequestID, StageID: q.RequestID, State: brokerpipe.Staged, Version: q.Consent.Version, Commit: q.Consent.Commit}, nil
	}, execute: func(_ context.Context, gen string) error {
		order = append(order, "execute")
		if gen == strings.Repeat("a", 32) {
			t.Fatal("old gen")
		}
		return nil
	}, current: func(context.Context, string) error { t.Fatal("Now started Current"); return nil }}
	if err := runBrokerUpdate(context.Background(), strings.Repeat("a", 32), s); err != nil {
		t.Fatal(err)
	}
	if !reflect.DeepEqual(order, []string{"prompt", "offer-update", "finish", "drain", "stage", "execute"}) {
		t.Fatal(order)
	}
}
func TestBrokerUpdateLaterAndNoOfferUseProtectedCurrent(t *testing.T) {
	for _, offer := range []bool{true, false} {
		t.Run(map[bool]string{true: "later", false: "offline-or-no-upgrade"}[offer], func(t *testing.T) {
			closes, prompts, currents := 0, 0, 0
			s := brokerUpdateServices{conversation: func(ctx context.Context, q brokerpipe.ExecutionRequest, h brokerpipe.ExecutionClientHandler) error {
				if offer {
					if _, err := h(ctx, updateCommand(q, brokerpipe.OfferUpdate)); err != nil {
						return err
					}
				}
				_, err := h(ctx, updateCommand(q, brokerpipe.FinishExecution))
				return err
			}, prompt: func(context.Context, brokerpipe.ExecutionCommand) (bool, error) { prompts++; return false, nil }, close: func() error { closes++; return nil }, stage: func(context.Context, brokerpipe.Request) (brokerpipe.Result, error) {
				t.Fatal("staged without Now")
				return brokerpipe.Result{}, nil
			}, execute: func(context.Context, string) error { t.Fatal("qualified without Now"); return nil }, current: func(context.Context, string) error {
				if closes != 1 {
					t.Fatal("current before drain")
				}
				currents++
				return nil
			}}
			if err := runBrokerUpdate(context.Background(), strings.Repeat("a", 32), s); err != nil {
				t.Fatal(err)
			}
			expected := 0
			if offer {
				expected = 1
			}
			if prompts != expected || closes != 1 || currents != 1 {
				t.Fatal(prompts, closes, currents)
			}
		})
	}
}
func TestBrokerUpdateFailuresNeverRetryOrStartCurrent(t *testing.T) {
	fault := errors.New("injected uncertainty")
	for _, mode := range []string{"transport-before", "transport-after-now", "missing-finish", "prompt", "drain", "stage", "stage-mismatch", "execute"} {
		t.Run(mode, func(t *testing.T) {
			closes, stages, executes := 0, 0, 0
			s := brokerUpdateServices{conversation: func(ctx context.Context, q brokerpipe.ExecutionRequest, h brokerpipe.ExecutionClientHandler) error {
				if mode == "transport-before" {
					return fault
				}
				if _, err := h(ctx, updateCommand(q, brokerpipe.OfferUpdate)); err != nil {
					return err
				}
				if mode == "transport-after-now" {
					return fault
				}
				if mode == "missing-finish" {
					return nil
				}
				_, err := h(ctx, updateCommand(q, brokerpipe.FinishExecution))
				return err
			}, prompt: func(context.Context, brokerpipe.ExecutionCommand) (bool, error) {
				if mode == "prompt" {
					return false, fault
				}
				return true, nil
			}, close: func() error {
				closes++
				if mode == "drain" {
					return fault
				}
				return nil
			}, stage: func(_ context.Context, q brokerpipe.Request) (brokerpipe.Result, error) {
				stages++
				if mode == "stage" {
					return brokerpipe.Result{}, fault
				}
				r := brokerpipe.Result{Schema: 1, RequestID: q.RequestID, StageID: q.RequestID, State: brokerpipe.Staged, Version: q.Consent.Version, Commit: q.Consent.Commit}
				if mode == "stage-mismatch" {
					r.StageID = strings.Repeat("f", 32)
				}
				return r, nil
			}, execute: func(context.Context, string) error { executes++; return fault }, current: func(context.Context, string) error { t.Fatal("uncertainty started Current"); return nil }}
			if err := runBrokerUpdate(context.Background(), strings.Repeat("a", 32), s); err == nil {
				t.Fatal("failure succeeded")
			}
			wantStages, wantExecutes := 0, 0
			if mode == "stage" || mode == "stage-mismatch" || mode == "execute" {
				wantStages = 1
			}
			if mode == "execute" {
				wantExecutes = 1
			}
			if closes != 1 || stages != wantStages || executes != wantExecutes {
				t.Fatal("failed boundary was skipped or replayed", closes, stages, executes)
			}
		})
	}
}
func TestBrokerOfferRejectsRepeatedMixedOrMalformedCommands(t *testing.T) {
	for _, mode := range []string{"repeat", "current", "candidate", "relay", "wrong-generation", "payload-on-finish", "bad-hash"} {
		t.Run(mode, func(t *testing.T) {
			q := brokerpipe.ExecutionRequest{Schema: 1, Operation: "offer-current", RequestID: strings.Repeat("1", 32), StageID: strings.Repeat("a", 32)}
			calls := 0
			a := &brokerOfferActions{request: q, prompt: func(context.Context, brokerpipe.ExecutionCommand) (bool, error) { calls++; return true, nil }}
			c := updateCommand(q, brokerpipe.OfferUpdate)
			switch mode {
			case "repeat":
				if _, err := a.handle(context.Background(), c); err != nil {
					t.Fatal(err)
				}
			case "current":
				c.Phase = brokerpipe.SpawnCurrent
			case "candidate":
				c.Phase = brokerpipe.SpawnCandidate
			case "relay":
				c.Phase = brokerpipe.RelaySelectedExecutor
			case "wrong-generation":
				c.ApplicationGeneration = strings.Repeat("b", 32)
			case "payload-on-finish":
				c.Phase = brokerpipe.FinishExecution
			case "bad-hash":
				c.PromptSHA256 = ""
			}
			if _, err := a.handle(context.Background(), c); err == nil {
				t.Fatal("invalid command accepted")
			}
			if calls > 1 || (mode != "repeat" && calls != 0) {
				t.Fatal(calls)
			}
		})
	}
}
func TestBrokerStartupRelayIsAdmissionOnlyAndTerminal(t *testing.T) {
	for _, op := range []string{"execute-session", "start-current", "start-selected"} {
		t.Run(op, func(t *testing.T) {
			a, b := brokerExecutionFixture()
			defer a.close()
			a.request.Operation = op
			c := brokerExecutionCommand(a, brokerpipe.RelaySelectedExecutor)
			c.Phase = brokerpipe.RelayStartupExecutor
			_, err := a.handle(context.Background(), c)
			if op != "start-selected" {
				if err == nil || len(b.commands) != 0 {
					t.Fatal("startup relay outside admission")
				}
				return
			}
			if err != nil {
				t.Fatal(err)
			}
			if _, err = a.handle(context.Background(), c); err == nil {
				t.Fatal("repeated relay")
			}
			if len(b.commands) != 1 {
				t.Fatal("duplicate creator")
			}
		})
	}
}

func TestBrokerOfferArgumentIsSoleCanonicalIntent(t *testing.T) {
	valid := "--broker-offer=" + strings.Repeat("a", 32)
	for _, args := range [][]string{{valid, valid}, {valid, "--xnav"}, {valid, "--broker-current=" + strings.Repeat("b", 32)}, {"--broker-offer"}, {"--broker-offer=" + strings.Repeat("A", 32)}} {
		if _, matched, err := brokerIntentArgument(args, "--broker-offer"); !matched || err == nil {
			t.Fatal(args, matched, err)
		}
	}
	if gen, matched, err := brokerIntentArgument([]string{valid}, "--broker-offer"); err != nil || !matched || gen != strings.Repeat("a", 32) {
		t.Fatal(gen, matched, err)
	}
}
func TestBrokerUpdateCancellationAfterNowDoesNotStage(t *testing.T) {
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	closes := 0
	s := brokerUpdateServices{conversation: func(ctx context.Context, q brokerpipe.ExecutionRequest, h brokerpipe.ExecutionClientHandler) error {
		_, err := h(ctx, updateCommand(q, brokerpipe.OfferUpdate))
		return err
	}, prompt: func(context.Context, brokerpipe.ExecutionCommand) (bool, error) { cancel(); return true, nil }, close: func() error { closes++; return nil }, stage: func(context.Context, brokerpipe.Request) (brokerpipe.Result, error) {
		t.Fatal("cancelled Now staged")
		return brokerpipe.Result{}, nil
	}, execute: func(context.Context, string) error { t.Fatal("cancelled Now executed"); return nil }, current: func(context.Context, string) error { t.Fatal("cancelled Now fell through"); return nil }}
	if err := runBrokerUpdate(ctx, strings.Repeat("a", 32), s); !errors.Is(err, context.Canceled) {
		t.Fatal(err)
	}
	if closes != 1 {
		t.Fatal(closes)
	}
}
