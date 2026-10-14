//go:build windows

package brokerenroll

import (
	"context"
	"errors"
)

const installedOriginalStartupName = "retired-original-Skager.lnk"

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
	if n.original == nil {
		return nil
	}
	raw, err := startupReadOriginal(ctx, n.original.pin)
	if err != nil {
		return err
	}
	target, arguments, working, err := startupLinkFields(raw)
	if err != nil {
		return errors.Join(errors.New("original normal shortcut cannot be classified safely"), err)
	}
	n.installedEligible = owns(target, arguments, working)
	if n.installedEligible {
		if err := startupRestorationAbsent(n.recovery, installedOriginalStartupName); err != nil {
			return err
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
	if !n.installedEligible {
		return nil
	}
	if n.original == nil || !n.restored || n.originalRetired {
		return errors.New("same restored original integration shortcut required")
	}
	if err := startupRenameRestorationFile(ctx, n.original, n.recovery, installedOriginalStartupName, n.record.Links[0].Original.SHA256); err != nil {
		return err
	}
	n.originalRetired = true
	n.record.Links[0].Original.Creation = uint64(n.original.pin.info.CreationTime.HighDateTime)<<32 | uint64(n.original.pin.info.CreationTime.LowDateTime)
	return errors.Join(n.check(ctx), guard(ctx))
}
