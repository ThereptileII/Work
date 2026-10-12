package brokerenroll

import (
	"context"
	"errors"
	"reflect"
	"strings"
	"testing"
)

func TestStartupShortcutClosedForms(t *testing.T) {
	generation := strings.Repeat("b", 32)
	for _, tc := range []struct {
		mode                startupShortcutMode
		name, pending, args string
	}{
		{startupNormal, "Skager.lnk", "pending.lnk", "--broker-start=" + generation},
		{startupLegacy, "Skager Legacy.lnk", "legacy-pending.lnk", "--broker-start=" + generation + ":legacy"},
		{startupSafe, "Skager Safe Mode.lnk", "safe-pending.lnk", "--broker-start=" + generation + ":safe"},
	} {
		spec, err := tc.mode.spec()
		if err != nil || spec.name != tc.name || spec.pending != tc.pending {
			t.Fatalf("fixed destination: %+v %v", spec, err)
		}
		args, err := tc.mode.arguments(generation)
		if err != nil || args != tc.args {
			t.Fatalf("fixed argument: %q %v", args, err)
		}
	}
	for _, generation := range []string{"", strings.Repeat("b", 31), strings.Repeat("B", 32), strings.Repeat("b", 32) + ":legacy", strings.Repeat("b", 31) + "/"} {
		if _, err := startupNormal.arguments(generation); err == nil {
			t.Fatalf("accepted generation %q", generation)
		}
	}
	if _, err := startupShortcutMode(3).spec(); err == nil {
		t.Fatal("accepted caller-selected mode")
	}
	if _, err := startupShortcutMode(3).arguments(generation); err == nil {
		t.Fatal("accepted caller-selected command")
	}
}
func TestStartupAllDestinationsPrecedePublication(t *testing.T) {
	for _, failure := range []int{-1, 0, 1, 2} {
		var events []int
		fault := errors.New("unknown destination")
		err := startupPublishSequence(context.Background(), func(mode startupShortcutMode) error {
			events = append(events, int(mode))
			if int(mode) == failure {
				return fault
			}
			return nil
		}, func(mode startupShortcutMode) error { events = append(events, 10+int(mode)); return nil })
		want := []int{0, 1, 2, 10, 11, 12}
		if failure >= 0 {
			want = want[:failure+1]
			if !errors.Is(err, fault) {
				t.Fatal(err)
			}
		} else if err != nil {
			t.Fatal(err)
		}
		if !reflect.DeepEqual(events, want) {
			t.Fatalf("failure %d publication order %v want %v", failure, events, want)
		}
	}
}
func TestStartupPartialPublicationNeverRetriesOrContinues(t *testing.T) {
	for _, failure := range []int{0, 1, 2} {
		var calls []int
		fault := errors.New("native move uncertain")
		err := startupPublishSequence(context.Background(), func(startupShortcutMode) error { return nil }, func(mode startupShortcutMode) error {
			calls = append(calls, int(mode))
			if int(mode) == failure {
				return fault
			}
			return nil
		})
		if !errors.Is(err, ErrStartupUncertain) || !errors.Is(err, fault) || len(calls) != failure+1 {
			t.Fatalf("failure %d: %v calls %v", failure, err, calls)
		}
	}
	ctx, cancel := context.WithCancel(context.Background())
	var published int
	err := startupPublishSequence(ctx, func(startupShortcutMode) error { return nil }, func(startupShortcutMode) error { published++; cancel(); return nil })
	if !errors.Is(err, ErrStartupUncertain) || !errors.Is(err, context.Canceled) || published != 1 {
		t.Fatalf("canceled partial publication %d %v", published, err)
	}
}
