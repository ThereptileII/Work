package brokerhost

import (
	"context"
	"encoding/json"
	"errors"
	"reflect"
	"strings"
	"testing"

	"example.com/opennav-update-verifier/brokerpipe"
	"example.com/opennav-update-verifier/brokerstore"
)

var hostTestFailure = errors.New("injected native failure")

type hostTestFixture struct {
	run      *executionHostRun
	facts    []executionSelectionFact
	events   []string
	fail     string
	report   brokerpipe.ExecutionObservation
	children [2]*hostTestChild
	resolved string
}
type hostTestJournal struct{ f *hostTestFixture }

func (j hostTestJournal) AppendFact(ctx context.Context, kind brokerstore.ExecutionFactKind, raw []byte) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	step := "begin"
	if kind != brokerstore.ExecutionPreparation {
		var s executionSelectionStep
		if err := json.Unmarshal(raw, &s); err != nil {
			return err
		}
		step = s.Step
	}
	j.f.events = append(j.f.events, "append:"+step)
	if j.f.fail == "append:"+step {
		return hostTestFailure
	}
	j.f.facts = append(j.f.facts, executionSelectionFact{kind, append([]byte(nil), raw...)})
	return nil
}
func (j hostTestJournal) Close() error {
	if j.f.fail == "journal-close" {
		return hostTestFailure
	}
	return nil
}

type hostTestProfile struct{ f *hostTestFixture }

func (p hostTestProfile) Validate(ctx context.Context) error {
	if p.f.fail == "profile" {
		return hostTestFailure
	}
	return ctx.Err()
}
func (p hostTestProfile) Evidence(context.Context) (string, error) {
	return strings.Repeat("d", 64), nil
}
func (p hostTestProfile) ReleaseForQualification(context.Context) error {
	p.f.events = append(p.f.events, "release")
	if p.f.fail == "release" {
		return hostTestFailure
	}
	return nil
}
func (p hostTestProfile) RestoreOriginal(context.Context) (string, error) {
	p.f.events = append(p.f.events, "restore-native")
	if p.f.fail == "restore" || p.f.fail == "health+restore" {
		return "", hostTestFailure
	}
	return strings.Repeat("e", 64), nil
}
func (p hostTestProfile) Close() error {
	if p.f.fail == "profile-close" {
		return hostTestFailure
	}
	return nil
}

type hostTestConversation struct{ f *hostTestFixture }

func (c hostTestConversation) ExchangeCommand(ctx context.Context, d brokerpipe.ExecutionDirective) (brokerpipe.ExecutionObservation, error) {
	c.f.events = append(c.f.events, "command:"+string(d.Phase))
	if d.Phase == brokerpipe.SpawnCandidate || d.Phase == brokerpipe.SpawnFallback || d.Phase == brokerpipe.ObserveCandidate || d.Phase == brokerpipe.ObserveFallback {
		if !strings.HasPrefix(d.HealthPipe, "Skager.Update.") || !hashPattern.MatchString(d.HealthChallenge) {
			return brokerpipe.ExecutionObservation{}, errors.New("missing native health payload")
		}
	}
	if c.f.fail == "command:"+string(d.Phase) {
		return brokerpipe.ExecutionObservation{}, hostTestFailure
	}
	if d.Phase == brokerpipe.SpawnCandidate || d.Phase == brokerpipe.SpawnFallback {
		step := "spawn-consumed"
		if d.Phase == brokerpipe.SpawnFallback {
			step = "fallback-spawn-consumed"
		}
		found := false
		for _, fact := range c.f.facts {
			var s executionSelectionStep
			_ = json.Unmarshal(fact.original, &s)
			found = found || s.Step == step
		}
		if !found {
			return brokerpipe.ExecutionObservation{}, errors.New("creator reached before durable consumption")
		}
	}
	return c.f.report, ctx.Err()
}

type hostTestChild struct {
	f        *hostTestFixture
	index    int
	observed bool
	closed   bool
}

func (c *hostTestChild) Environment(context.Context) (map[string]string, error) {
	g := c.f.run.begin.Generation
	if c.index == 1 {
		g = c.f.run.begin.PreviousGeneration
	}
	return map[string]string{"SKAGER_UPDATE_GENERATION": g, "SKAGER_UPDATE_QUALIFY_PROFILE": "1", "SKAGER_UPDATE_PIPE": "Skager.Update." + strings.Repeat("a", 32), "SKAGER_UPDATE_CHALLENGE": strings.Repeat("b", 64)}, nil
}
func (c *hostTestChild) Observe(context.Context) (string, error) {
	c.f.events = append(c.f.events, "observe-native")
	if (c.index == 0 && strings.HasPrefix(c.f.fail, "health")) || (c.index == 1 && c.f.fail == "health+fallback-health") {
		return "", hostTestFailure
	}
	c.observed = true
	return strings.Repeat("c", 64), nil
}
func (c *hostTestChild) ValidateHealthy(context.Context) error {
	if !c.observed || c.f.fail == "healthy-recheck" {
		return hostTestFailure
	}
	return nil
}
func (c *hostTestChild) WaitClosed(context.Context) (string, error) {
	c.f.events = append(c.f.events, "closed-native")
	if c.f.fail == "teardown" {
		return "", hostTestFailure
	}
	return strings.Repeat("f", 64), nil
}
func (c *hostTestChild) Close() error {
	c.closed = true
	if c.f.fail == "child-close" {
		return hostTestFailure
	}
	return nil
}
func newHostTestFixture(root bool) *hostTestFixture {
	f := &hostTestFixture{report: brokerpipe.ExecutionObservation{Status: "reported-complete", ProcessID: 999, ProcessCreation: 999, EvidenceSHA256: strings.Repeat("0", 64)}}
	b := executionSelectionBegin{1, "broker-selected-transaction-v1", strings.Repeat("1", 32), "S-1-5-21-1-2-3-500", strings.Repeat("2", 32), "2.0.0", strings.Repeat("3", 40), strings.Repeat("4", 64), "", ""}
	if !root {
		b.PreviousGeneration = strings.Repeat("5", 32)
		b.PreviousHeadSHA256 = strings.Repeat("6", 64)
	}
	f.run = &executionHostRun{begin: b, journal: hostTestJournal{f}, conversation: hostTestConversation{f}, profile: hostTestProfile{f}}
	f.run.validate = func(ctx context.Context) error {
		if f.fail == "custody" {
			return hostTestFailure
		}
		return ctx.Err()
	}
	f.run.commission = func(ctx context.Context, fallback bool) (string, error) {
		f.events = append(f.events, "commission-native")
		if f.fail == "commission" {
			return "", hostTestFailure
		}
		return strings.Repeat("7", 64), ctx.Err()
	}
	f.run.prepare = func(ctx context.Context, fallback bool) (executionHostChild, error) {
		index := 0
		if fallback {
			index = 1
		}
		c := &hostTestChild{f: f, index: index}
		f.children[index] = c
		if f.fail == "prepare" {
			return c, hostTestFailure
		}
		return c, ctx.Err()
	}
	f.run.resolve = func(ctx context.Context, terminal string) error {
		f.events = append(f.events, "resolve-native")
		if f.fail == "resolve" {
			return hostTestFailure
		}
		if err := f.run.journal.Close(); err != nil {
			return err
		}
		f.resolved = terminal
		return ctx.Err()
	}
	return f
}
func (f *hostTestFixture) parsed(t *testing.T, root bool) executionSelectionAttempt {
	t.Helper()
	purpose := string(brokerstore.GenerationPurposeSignedCandidate)
	if root {
		purpose = string(brokerstore.GenerationPurposeApprovedBaseline)
	}
	a, err := parseExecutionSelectionAttempt(f.run.begin.ClientSID, f.run.begin.ExecutionID, f.run.begin.Generation, f.run.begin.CompletionSHA256, purpose, strings.Repeat("8", 64), f.facts)
	if err != nil {
		t.Fatal("actual selection reducer rejected coordinator facts", err)
	}
	return a
}
func TestExecutionHostNativeEffectsPrecedeDurableSelection(t *testing.T) {
	for _, root := range []bool{true, false} {
		t.Run(map[bool]string{true: "initial-baseline", false: "signed-candidate"}[root], func(t *testing.T) {
			f := newHostTestFixture(root)
			if err := f.run.execute(context.Background()); err != nil {
				t.Fatal(err)
			}
			if f.parsed(t, root).terminal != "selected" || f.resolved != "selected" || !f.children[0].closed {
				t.Fatal("missing actual completion")
			}
			want := []string{"append:begin", "release", "commission-native", "append:commissioned", "append:spawn-consumed", "command:spawn-candidate", "command:observe-candidate", "observe-native", "append:healthy", "command:close-candidate", "closed-native", "append:selected", "resolve-native", "command:finish"}
			if !reflect.DeepEqual(f.events, want) {
				t.Fatalf("effect order %v", f.events)
			}
			for _, event := range f.events {
				if strings.Contains(event, "current") {
					t.Fatal("qualification converted into ordinary launch")
				}
			}
			if !errors.Is(f.run.execute(context.Background()), errExecutionHostRefused) {
				t.Fatal("one-use run replayed")
			}
		})
	}
}
func TestExecutionHostClientReportsCannotSupplyNativeHealthOrClosure(t *testing.T) {
	for _, failure := range []string{"health", "healthy-recheck", "teardown", "child-close"} {
		t.Run(failure, func(t *testing.T) {
			f := newHostTestFixture(true)
			f.fail = failure
			if !errors.Is(f.run.execute(context.Background()), errExecutionHostUncertain) {
				t.Fatal("native failure accepted")
			}
			if f.resolved != "" || f.parsed(t, true).terminal != "" {
				t.Fatal("client report selected failed native observation")
			}
		})
	}
	t.Run("client-failure-is-not-native-failure", func(t *testing.T) {
		f := newHostTestFixture(true)
		f.report.Status = "reported-failed"
		if err := f.run.execute(context.Background()); err != nil {
			t.Fatal(err)
		}
	})
}
func TestExecutionHostFallbackRequiresNativeCloseRestoreAndHealth(t *testing.T) {
	f := newHostTestFixture(false)
	f.fail = "health"
	if err := f.run.execute(context.Background()); err != nil {
		t.Fatal(err)
	}
	if f.parsed(t, false).terminal != "rolled-back" || f.resolved != "rolled-back" || !f.children[1].closed {
		t.Fatal("fallback failed")
	}
	seen := strings.Join(f.events, ",")
	for _, needed := range []string{"closed-native,append:candidate-closed,restore-native,append:profile-restored,commission-native,append:fallback-commissioned,append:fallback-spawn-consumed", "observe-native,append:fallback-healthy,command:close-fallback,closed-native,append:rolled-back,resolve-native"} {
		if !strings.Contains(seen, needed) {
			t.Fatal("fallback omitted native boundary", seen)
		}
	}
}
func TestExecutionHostFailureBoundariesRemainPending(t *testing.T) {
	for _, failure := range []string{"custody", "append:begin", "profile", "release", "commission", "append:commissioned", "prepare", "append:spawn-consumed", "command:spawn-candidate", "command:observe-candidate", "append:healthy", "command:close-candidate", "append:selected", "resolve", "command:finish", "journal-close", "profile-close"} {
		t.Run(failure, func(t *testing.T) {
			f := newHostTestFixture(true)
			f.fail = failure
			if !errors.Is(f.run.execute(context.Background()), errExecutionHostUncertain) {
				t.Fatal("failed boundary reported successful")
			}
			if failure != "resolve" && failure != "command:finish" && failure != "journal-close" && failure != "profile-close" {
				for _, fact := range f.facts {
					var step executionSelectionStep
					_ = json.Unmarshal(fact.original, &step)
					if step.Step == "selected" {
						t.Fatal("premature selected terminal")
					}
				}
			}
		})
	}
	t.Run("cancel-before-work", func(t *testing.T) {
		f := newHostTestFixture(false)
		ctx, cancel := context.WithCancel(context.Background())
		cancel()
		if !errors.Is(f.run.execute(ctx), context.Canceled) || len(f.facts) != 0 {
			t.Fatal("cancelled work mutated")
		}
	})
	t.Run("prepare-failure-no-fallback", func(t *testing.T) {
		f := newHostTestFixture(false)
		f.fail = "prepare"
		_ = f.run.execute(context.Background())
		if f.children[1] != nil {
			t.Fatal("pre-spawn failure invented candidate teardown/fallback")
		}
	})
}

func TestExecutionHostFallbackCannotBypassUnavailableRestoration(t *testing.T) {
	for _, failure := range []string{"health+restore", "health+fallback-health"} {
		t.Run(failure, func(t *testing.T) {
			f := newHostTestFixture(false)
			f.fail = failure
			if !errors.Is(f.run.execute(context.Background()), errExecutionHostUncertain) {
				t.Fatal("failed restoration/fallback reported complete")
			}
			if f.parsed(t, false).terminal != "" || f.resolved != "" {
				t.Fatal("pending failure discarded")
			}
			if failure == "health+restore" && f.children[1] != nil {
				t.Fatal("fallback spawned without original profile")
			}
		})
	}
}

func TestExecutionHostPreparationPrecedesNativeProfileCapture(t *testing.T) {
	for _, failure := range []bool{false, true} {
		t.Run(map[bool]string{false: "capture", true: "capture-failure"}[failure], func(t *testing.T) {
			f := newHostTestFixture(true)
			profile := f.run.profile
			f.run.profile = nil
			f.run.capture = func(ctx context.Context) (executionHostProfile, error) {
				if len(f.facts) != 1 || f.facts[0].kind != brokerstore.ExecutionPreparation {
					t.Fatal("profile reserved before original preparation")
				}
				if failure {
					return nil, hostTestFailure
				}
				return profile, nil
			}
			err := f.run.execute(context.Background())
			if failure {
				if !errors.Is(err, errExecutionHostUncertain) || len(f.facts) != 1 || f.parsed(t, true).state != "begin" {
					t.Fatal("failed capture lost recoverable preparation", err)
				}
			} else if err != nil {
				t.Fatal(err)
			}
		})
	}
}
