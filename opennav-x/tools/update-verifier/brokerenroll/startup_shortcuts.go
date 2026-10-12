package brokerenroll

import (
	"context"
	"errors"
)

// These closed forms are installation policy, never caller-selected paths or
// commands. Zero remains normal for the original single-link native fixtures.
type startupShortcutMode uint8

const (
	startupNormal startupShortcutMode = iota
	startupLegacy
	startupSafe
)

type startupShortcutSpec struct{ name, pending, suffix string }

func (mode startupShortcutMode) spec() (startupShortcutSpec, error) {
	switch mode {
	case startupNormal:
		return startupShortcutSpec{"Skager.lnk", "pending.lnk", ""}, nil
	case startupLegacy:
		return startupShortcutSpec{"Skager Legacy.lnk", "legacy-pending.lnk", ":legacy"}, nil
	case startupSafe:
		return startupShortcutSpec{"Skager Safe Mode.lnk", "safe-pending.lnk", ":safe"}, nil
	default:
		return startupShortcutSpec{}, errors.New("fixed startup shortcut mode required")
	}
}
func (mode startupShortcutMode) arguments(generation string) (string, error) {
	spec, err := mode.spec()
	if err != nil {
		return "", err
	}
	if len(generation) != 32 {
		return "", errors.New("approved startup generation required")
	}
	for _, c := range generation {
		if !(c >= '0' && c <= '9' || c >= 'a' && c <= 'f') {
			return "", errors.New("approved startup generation required")
		}
	}
	return "--broker-start=" + generation + spec.suffix, nil
}

// Preflight every fixed destination before the first publication. Rechecks at
// each native rename and non-replacing moves still refuse later interference;
// this sequence cannot make three filesystem renames atomic.
func startupPublishSequence(ctx context.Context, preflight func(startupShortcutMode) error, publish func(startupShortcutMode) error) error {
	for mode := startupNormal; mode <= startupSafe; mode++ {
		if err := ctx.Err(); err != nil {
			return err
		}
		if err := preflight(mode); err != nil {
			return err
		}
	}
	for mode := startupNormal; mode <= startupSafe; mode++ {
		if err := ctx.Err(); err != nil {
			return errors.Join(ErrStartupUncertain, err)
		}
		if err := publish(mode); err != nil {
			return errors.Join(ErrStartupUncertain, err)
		}
	}
	return nil
}
