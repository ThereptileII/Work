package brokerenroll

import (
	"context"
	"errors"
)

// Comparisons come from the retained authenticated native launch parent. They
// supply no path, SID or replacement authority on their own.
type InstalledCodeUpgradeComparisons struct {
	PolicySHA256, ProvisionerSHA256, TrustConfigSHA256, OriginalCompletionSHA256 string
}

type installedCodeSuccessorPreparation interface {
	check(context.Context) error
	request() CodeUpgradeRequest
	releaseCompletion(context.Context) error
	close() error
}

// RunInstalledCodeUpgradeSuccessor is the fixed new-image child entry. Its cmd
// caller must authenticate and retain the actual same-account launch parent.
// It independently reopens original trust/completion/cache, verifies the exact
// signed target and running successor, then reuses the existing upgrade engine.
// It is fresh-only; recovery remains the separate authenticated operator route.
func RunInstalledCodeUpgradeSuccessor(ctx context.Context, c InstalledCodeUpgradeComparisons) error {
	return runInstalledCodeUpgradeSuccessorWith(ctx, c, prepareInstalledCodeSuccessorNative, UpgradeEnrolledCode)
}
func runInstalledCodeUpgradeSuccessorWith(ctx context.Context, c InstalledCodeUpgradeComparisons, prepare func(context.Context, InstalledCodeUpgradeComparisons) (installedCodeSuccessorPreparation, error), upgrade func(context.Context, CodeUpgradeRequest) error) (resultErr error) {
	if ctx == nil || ctx.Err() != nil || prepare == nil || upgrade == nil {
		return errors.New("live original installed successor required")
	}
	for _, pin := range []string{c.PolicySHA256, c.ProvisionerSHA256, c.TrustConfigSHA256, c.OriginalCompletionSHA256} {
		if !validPin(pin) {
			return errors.New("complete authenticated launch comparisons required")
		}
	}
	p, err := prepare(ctx, c)
	if p != nil {
		defer func() { resultErr = errors.Join(resultErr, p.close()) }()
	}
	if err != nil || p == nil {
		return errors.Join(errors.New("independent native successor preparation required"), err)
	}
	if err = p.check(ctx); err != nil {
		return err
	}
	req := p.request()
	if req.Resume || req.PolicySHA256 != c.PolicySHA256 || req.ProvisionerSHA256 != c.ProvisionerSHA256 || req.TrustConfigSHA256 != c.TrustConfigSHA256 || req.OriginalCompletionSHA256 != c.OriginalCompletionSHA256 {
		return errors.New("prepared successor differs from original launch comparisons")
	}
	if err = p.releaseCompletion(ctx); err != nil {
		return err
	}
	if err = p.check(ctx); err != nil {
		return err
	}
	return upgrade(ctx, req)
}
