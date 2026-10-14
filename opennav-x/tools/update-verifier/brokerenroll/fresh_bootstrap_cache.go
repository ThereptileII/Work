package brokerenroll

import (
	"bytes"
	"context"
	"errors"
	"time"

	"example.com/opennav-update-verifier/repository"
)

// FreshBootstrapCacheRequest is an explicit operator preparation, not an
// installed-customer initializer. Authenticate the final provisioner before its
// first elevated instruction and supply original approvals independently.
type FreshBootstrapCacheRequest struct {
	TrustConfiguration                                      []byte
	TrustConfigSHA256, ProvisionerSHA256, BootstrapCacheDir string
}

var ErrFreshBootstrapCacheUncertain = errors.New("fresh bootstrap cache preparation effect uncertain; preserve partial cache and do not reset or retry")

type freshBootstrapCacheOwner interface {
	check(context.Context) error
	seed(context.Context, []byte) error
	close() error
}
type freshBootstrapCacheOperations struct {
	source func(context.Context, string) (maintenanceImageSource, error)
	open   func(context.Context, string) (freshBootstrapCacheOwner, error)
}

// PrepareFreshBootstrapCache exclusively creates a previously absent cache and
// its metadata/root.json from exact independently approved original public trust.
// Existing, damaged and partial namespaces refuse; no floor/cache repair exists.
func PrepareFreshBootstrapCache(ctx context.Context, r FreshBootstrapCacheRequest) error {
	return prepareFreshBootstrapCacheWith(ctx, r, freshBootstrapCacheNativeOperations())
}
func prepareFreshBootstrapCacheWith(ctx context.Context, r FreshBootstrapCacheRequest, op freshBootstrapCacheOperations) (resultErr error) {
	if ctx == nil || ctx.Err() != nil || !validPin(r.TrustConfigSHA256) || !validPin(r.ProvisionerSHA256) || r.BootstrapCacheDir == "" || len(r.TrustConfiguration) == 0 || len(r.TrustConfiguration) > maxTrustConfigurationBytes || op.source == nil || op.open == nil {
		return errors.New("complete independently approved fresh cache inputs required")
	}
	ctx, cancel := context.WithTimeout(ctx, maxEnrollmentTime)
	defer cancel()
	raw := bytes.Clone(r.TrustConfiguration)
	if digest(raw) != r.TrustConfigSHA256 {
		return errors.New("fresh cache trust differs from original approval")
	}
	trust, err := repository.ValidateTrust(raw, time.Now().UTC())
	if err != nil {
		return err
	}
	effect := false
	cleanup := func(err error) {
		if err != nil {
			resultErr = errors.Join(resultErr, enrollmentCleanupResult(err))
			if effect {
				resultErr = errors.Join(ErrFreshBootstrapCacheUncertain, resultErr)
			}
		}
	}
	source, err := op.source(ctx, r.ProvisionerSHA256)
	if source != nil {
		defer func() { cleanup(source.Close()) }()
	}
	if err != nil || source == nil {
		return errors.Join(errors.New("authenticated original provisioner required before cache preparation"), err)
	}
	if err = source.Check(ctx); err != nil {
		return err
	}
	owner, err := op.open(ctx, r.BootstrapCacheDir)
	if owner != nil {
		defer func() { cleanup(owner.close()) }()
	}
	if err != nil || owner == nil {
		return errors.Join(errors.New("absent fresh cache under original protected parent required"), err)
	}
	if err = errors.Join(ctx.Err(), source.Check(ctx), owner.check(ctx)); err != nil {
		return err
	}
	effect = true
	if err = owner.seed(ctx, bytes.Clone(trust.BootstrapRoot)); err != nil {
		return errors.Join(ErrFreshBootstrapCacheUncertain, err)
	}
	if err = errors.Join(ctx.Err(), source.Check(ctx), owner.check(ctx)); err != nil {
		return errors.Join(ErrFreshBootstrapCacheUncertain, err)
	}
	return nil
}
