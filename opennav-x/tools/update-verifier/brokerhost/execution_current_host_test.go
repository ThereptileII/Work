package brokerhost

import (
	"context"
	"errors"
	"reflect"
	"strings"
	"testing"

	"example.com/opennav-update-verifier/brokerpipe"
)

type currentHostFixture struct {
	run    *executionCurrentHostRun
	events []string
	fail   string
	child  *currentHostTestChild
	report brokerpipe.ExecutionObservation
}
type currentHostConversation struct{ f *currentHostFixture }

func (c currentHostConversation) ExchangeCommand(ctx context.Context, d brokerpipe.ExecutionDirective) (brokerpipe.ExecutionObservation, error) {
	c.f.events = append(c.f.events, "command:"+string(d.Phase))
	if d.HealthPipe != "" || d.HealthChallenge != "" {
		return brokerpipe.ExecutionObservation{}, errors.New("normal creator received qualification environment")
	}
	if d.Phase != brokerpipe.FinishExecution && d.ApplicationGeneration != c.f.run.generation {
		return brokerpipe.ExecutionObservation{}, errors.New("client stage substituted for selection")
	}
	if d.Phase == brokerpipe.SpawnCurrent && !c.f.run.consumed {
		return brokerpipe.ExecutionObservation{}, errors.New("unconsumed creator")
	}
	if c.f.fail == string(d.Phase) {
		return brokerpipe.ExecutionObservation{}, hostTestFailure
	}
	return c.f.report, ctx.Err()
}

type currentHostTestChild struct {
	f      *currentHostFixture
	closed bool
}

func (c *currentHostTestChild) Validate(ctx context.Context, report brokerpipe.ExecutionObservation) error {
	c.f.events = append(c.f.events, "native-recheck")
	if c.f.fail == "child" {
		return hostTestFailure
	}
	return ctx.Err()
}
func (c *currentHostTestChild) Close() error {
	c.closed = true
	if c.f.fail == "cleanup" {
		return hostTestFailure
	}
	return nil
}
func newCurrentHostFixture() *currentHostFixture {
	f := &currentHostFixture{report: brokerpipe.ExecutionObservation{Status: "reported-complete", ProcessID: 10, ProcessCreation: 20, EvidenceSHA256: strings.Repeat("0", 64)}}
	f.run = &executionCurrentHostRun{generation: strings.Repeat("2", 32), conversation: currentHostConversation{f}}
	f.run.validate = func(ctx context.Context) error {
		if f.fail == "selection" || f.fail == "pending" || f.fail == "baseline-unqualified" || f.fail == "armed" {
			return hostTestFailure
		}
		return ctx.Err()
	}
	f.run.beforeSpawn = func(ctx context.Context) error {
		f.events = append(f.events, "prepare-native")
		if f.fail == "prepare" {
			return hostTestFailure
		}
		return ctx.Err()
	}
	f.run.corroborate = func(ctx context.Context, report brokerpipe.ExecutionObservation) (executionCurrentHostChild, error) {
		f.events = append(f.events, "native-corroborate")
		f.child = &currentHostTestChild{f: f}
		if f.fail == "process" || f.fail == "image" || f.fail == "token" || f.fail == "parent" || f.fail == "birth" {
			return f.child, hostTestFailure
		}
		return f.child, ctx.Err()
	}
	return f
}
func TestExecutionCurrentHostNormalSelectedStartup(t *testing.T) {
	for _, reason := range []string{"ordinary", "later", "metadata-offline"} {
		t.Run(reason, func(t *testing.T) {
			f := newCurrentHostFixture()
			if err := f.run.execute(context.Background()); err != nil {
				t.Fatal(err)
			}
			want := []string{"prepare-native", "command:spawn-current", "native-corroborate", "command:observe-current", "native-recheck", "command:finish"}
			if !reflect.DeepEqual(f.events, want) || f.child == nil || !f.child.closed {
				t.Fatal("normal effect order or handle lifetime", f.events)
			}
			if !errors.Is(f.run.execute(context.Background()), errExecutionCurrentRefused) {
				t.Fatal("creator role reused")
			}
		})
	}
}
func TestExecutionCurrentHostNativeIdentityRequired(t *testing.T) {
	for _, failure := range []string{"process", "image", "token", "parent", "birth", "child"} {
		t.Run(failure, func(t *testing.T) {
			f := newCurrentHostFixture()
			f.fail = failure
			if !errors.Is(f.run.execute(context.Background()), errExecutionCurrentUncertain) {
				t.Fatal("client report replaced native observation")
			}
			for _, event := range f.events {
				if event == "command:finish" {
					t.Fatal("failed native evidence reported complete")
				}
			}
			if f.child != nil && !f.child.closed {
				t.Fatal("original query handle leaked")
			}
		})
	}
}
func TestExecutionCurrentHostPreselectionUncertaintyRefuses(t *testing.T) {
	for _, failure := range []string{"selection", "pending", "baseline-unqualified", "armed"} {
		t.Run(failure, func(t *testing.T) {
			f := newCurrentHostFixture()
			f.fail = failure
			if !errors.Is(f.run.execute(context.Background()), errExecutionCurrentRefused) || f.run.consumed || len(f.events) != 0 {
				t.Fatal("unqualified state reached creator")
			}
		})
	}
}
func TestExecutionCurrentHostNoRetryOrOrdinaryFallback(t *testing.T) {
	for _, failure := range []string{"prepare", "spawn-current", "observe-current", "finish", "cleanup"} {
		t.Run(failure, func(t *testing.T) {
			f := newCurrentHostFixture()
			f.fail = failure
			if !errors.Is(f.run.execute(context.Background()), errExecutionCurrentUncertain) {
				t.Fatal("lost conversation accepted")
			}
			if !errors.Is(f.run.execute(context.Background()), errExecutionCurrentRefused) {
				t.Fatal("uncertain creator retried")
			}
			for _, event := range f.events {
				if strings.Contains(event, "candidate") || strings.Contains(event, "fallback") || strings.Contains(event, "close-current") {
					t.Fatal("normal failure entered qualification or signalled app")
				}
			}
		})
	}
}
func TestExecutionCurrentHostCancellationAndMissingGates(t *testing.T) {
	t.Run("cancelled", func(t *testing.T) {
		f := newCurrentHostFixture()
		ctx, cancel := context.WithCancel(context.Background())
		cancel()
		if !errors.Is(f.run.execute(ctx), context.Canceled) || f.run.consumed {
			t.Fatal("cancelled startup consumed creator")
		}
	})
	for _, missing := range []string{"conversation", "selection", "prepare", "corroboration", "generation"} {
		t.Run(missing, func(t *testing.T) {
			f := newCurrentHostFixture()
			switch missing {
			case "conversation":
				f.run.conversation = nil
			case "selection":
				f.run.validate = nil
			case "prepare":
				f.run.beforeSpawn = nil
			case "corroboration":
				f.run.corroborate = nil
			case "generation":
				f.run.generation = "untrusted"
			}
			if !errors.Is(f.run.execute(context.Background()), errExecutionCurrentRefused) || len(f.events) != 0 {
				t.Fatal("missing actual native boundary accepted")
			}
		})
	}
}
