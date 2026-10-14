//go:build windows

package brokerenroll

import (
	"context"
	"errors"
)

const installedOriginalStartupName = "retired-original-Skager.lnk"

func installedOriginalStartupRetirementName(mode startupShortcutMode) string {
	spec, _ := mode.spec()
	return "retired-original-" + spec.name
}

// Classification happens before the terminal fence, from the already retained
// original LNK bytes. The installed owner supplies only a closed, authenticated
// target comparison; it cannot provide a new shortcut path or replacement file.
func (n *nativeStartupRestoration) prepareInstalled(ctx context.Context, owns func(string, string, string) bool) (resultErr error) {
	if n == nil || n.installedPrepared || n.attempted || n.anchor == nil || n.anchor.bound || owns == nil {
		return errors.New("fresh original installed startup preparation required")
	}
	n.installedPrepared = true
	defer func() {
		if resultErr != nil {
			n.installedFailed = true
		}
	}()
	if err := n.check(ctx); err != nil {
		return err
	}
	for i, original := range n.originals {
		if original == nil {
			continue
		}
		raw, err := startupReadOriginal(ctx, original.pin)
		if err != nil {
			return err
		}
		target, arguments, working, err := startupLinkFields(raw)
		if err != nil {
			return errors.Join(errors.New("original shortcut cannot be classified safely"), err)
		}
		n.installedEligible[i] = owns(target, arguments, working)
		if n.installedEligible[i] {
			if err := startupRestorationAbsent(n.recovery, installedOriginalStartupRetirementName(startupShortcutMode(i))); err != nil {
				return err
			}
		}
	}
	return n.check(ctx)
}

// Retire the restored integration link before SCM stop, while the same sealed
// final witness is still live. Preserve its exact bytes in recovery and keep the
// original handle through subsequent checks; stock/unknown links are untouched.
func (n *nativeStartupRestoration) retireInstalled(ctx context.Context, guard func(context.Context) error) (resultErr error) {
	if n == nil || !n.installedPrepared || n.installedAttempted || n.installedFailed || n.anchor == nil || !n.anchor.bound || guard == nil {
		return errors.New("bound original installed startup retirement required")
	}
	n.installedAttempted = true
	defer func() {
		if resultErr != nil {
			n.installedFailed = true
		}
	}()
	if err := errors.Join(guard(ctx), n.check(ctx)); err != nil {
		return err
	}
	for i, eligible := range n.installedEligible {
		if !eligible {
			continue
		}
		original := n.originals[i]
		if original == nil || !n.restored[i] || n.originalRetired[i] {
			return errors.New("same restored original integration shortcut required")
		}
		if err := errors.Join(guard(ctx), n.check(ctx)); err != nil {
			return err
		}
		if err := startupRenameRestorationFile(ctx, original, n.recovery, installedOriginalStartupRetirementName(startupShortcutMode(i)), n.record.Links[i].Original.SHA256); err != nil {
			return err
		}
		n.originalRetired[i] = true
		n.record.Links[i].Original.Creation = uint64(original.pin.info.CreationTime.HighDateTime)<<32 | uint64(original.pin.info.CreationTime.LowDateTime)
		if err := errors.Join(n.check(ctx), guard(ctx)); err != nil {
			return err
		}
	}
	return nil
}
