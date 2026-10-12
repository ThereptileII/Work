package brokerhost

import (
	"context"
	"errors"

	verifier "example.com/opennav-update-verifier"
	"example.com/opennav-update-verifier/brokerstore"
)

// One handler owns one original stage-seal view. Revalidation never adopts a
// replacement receipt or reinterprets a previous failed transaction as current.
type nativeExecutionAuthorization struct {
	store  *brokerstore.Store
	target *brokerstore.ReopenedGeneration
	seal   *brokerstore.GenerationStageSeal
}

func nativeExecutionProviders(store *brokerstore.Store) (executionHostProviders, func() error) {
	authorization := &nativeExecutionAuthorization{store: store}
	return executionHostProviders{authorize: authorization.authorize,
		capture: captureExecutionProfileProvider, commission: commissionExecutionProvider}, func() error {
		if authorization.seal != nil {
			return authorization.seal.Close()
		}
		return nil
	}
}

func (a *nativeExecutionAuthorization) authorize(ctx context.Context, target *brokerstore.ReopenedGeneration, selected *nativeSelectedExecution) (resultErr error) {
	if a == nil || a.store == nil || ctx == nil || ctx.Err() != nil || target == nil ||
		(a.target != nil && a.target != target) {
		return errExecutionAuthorization
	}
	if err := errors.Join(a.store.Validate(), target.Validate()); err != nil {
		return err
	}
	a.target = target
	if target.Purpose() == brokerstore.GenerationPurposeApprovedBaseline {
		// Independent approved originals establish only this initial qualifier's
		// identity. Empty-history and fresh journal checks remain in the host.
		if selected != nil || a.seal != nil {
			return errExecutionAuthorization
		}
		baseline, err := target.ApprovedBaseline()
		if err != nil {
			return err
		}
		c, err := baseline.Completion()
		if err != nil {
			return err
		}
		raw, err := a.store.ReadRegistration(c.ClientSID)
		if err != nil {
			return err
		}
		r, err := parseRegistration(raw, c.ClientSID)
		if err != nil || c.Generation != r.Installed.Generation || c.ApprovedRequestSHA256 != r.ApprovedRequestSHA256 ||
			c.Identity.Version != r.Installed.Version || c.Identity.Commit != r.Installed.Commit ||
			c.Identity.ExecutableSHA256 != r.Installed.ExecutableSHA256 || c.Identity.ProductBuildSHA256 != r.Installed.ProductBuildSHA256 ||
			c.Identity.InventorySHA256 != r.Installed.InventorySHA256 || c.Identity.LifecycleManifestSHA256 != r.Installed.PackageSHA256 {
			return errors.Join(errExecutionAuthorization, err)
		}
		return target.Validate()
	}
	if target.Purpose() != brokerstore.GenerationPurposeSignedCandidate || selected == nil || selected.store != a.store || selected.closed {
		return errExecutionAuthorization
	}
	candidate, err := target.Candidate()
	if err != nil {
		return err
	}
	c, err := candidate.Completion()
	if err != nil {
		return err
	}
	raw, err := a.store.ReadRegistration(c.ClientSID)
	if err != nil {
		return err
	}
	r, err := parseRegistration(raw, c.ClientSID)
	if err != nil || r.ClientSID != selected.registration.ClientSID || executionOriginalDigest(raw) != selected.registrationDigest {
		return errors.Join(errExecutionAuthorization, err)
	}
	if a.seal == nil {
		a.seal, err = a.store.OpenGenerationStageSeal(ctx, target)
		if err != nil {
			return err
		}
	}
	snapshot, err := a.seal.Snapshot(ctx)
	if err != nil || snapshot.Completion != c {
		return errors.Join(errExecutionAuthorization, err)
	}
	receipt, err := parseExecutionStagingReceipt(snapshot.Receipt, r, c)
	if err != nil {
		return err
	}
	state, artifacts, err := a.store.VerifierDirectories(r.ClientSID, r.Trust.Channel)
	if err != nil {
		return err
	}
	observed, stock, err := observeStock(ctx, r)
	if stock != nil {
		defer func() { resultErr = errors.Join(resultErr, stock.Close()) }()
	}
	if err != nil || stock == nil || observed != stockBaseline {
		return errors.Join(errExecutionAuthorization, err)
	}
	config := r.configuration(state, artifacts)
	config.Installed = selected.installed()
	current, err := verifier.CheckUpdate(ctx, config, observed)
	if err != nil {
		return err
	}
	if err = matchExecutionSignedOffer(receipt, current, selected.installed(), r.Trust.Channel); err != nil {
		return err
	}
	// The host separately admits only its own newly reserved journal names. Here
	// retain/recheck every ORIGINAL selected reader instead of rejecting that
	// deliberate extension with selected.validate() or inventing a new head.
	for _, reader := range selected.readers {
		if err = reader.Validate(); err != nil {
			return err
		}
	}
	after, err := a.store.ReadRegistration(c.ClientSID)
	if err != nil || executionOriginalDigest(after) != selected.registrationDigest {
		return errors.Join(errExecutionAuthorization, err)
	}
	return errors.Join(stock.validate(), selected.generation.Validate(), target.Validate(), a.seal.Validate(ctx), ctx.Err())
}

// Restoration always requires the host's independently retained original closed
// child authority. Capture/backup evidence alone cannot permit an overwrite.
type nativeExecutionProfileProvider struct {
	*ExecutionProfileCapture
	restoration *executionHostClosedChild
}

func (p *nativeExecutionProfileProvider) RestoreOriginal(ctx context.Context) (string, error) {
	if p == nil || p.ExecutionProfileCapture == nil || p.restoration == nil {
		return "", errExecutionAuthorization
	}
	return p.ExecutionProfileCapture.restoreOriginalNative(ctx, p.restoration)
}

func (p *nativeExecutionProfileProvider) Close() error {
	if p == nil {
		return nil
	}
	var err error
	if p.restoration != nil {
		err = p.restoration.Close()
	}
	if p.ExecutionProfileCapture != nil {
		err = errors.Join(err, p.ExecutionProfileCapture.Close())
	}
	return err
}

func captureExecutionProfileProvider(ctx context.Context, store *brokerstore.Store, journal *brokerstore.ExecutionJournal) (executionHostProfile, error) {
	profile, err := CaptureExecutionProfileBackupNative(ctx, store, journal)
	// Preserve a partially returned owner so the host can drain it on failure.
	if profile == nil {
		return nil, err
	}
	return &nativeExecutionProfileProvider{ExecutionProfileCapture: profile}, err
}

// Qualification is process-lifetime nonmutating with respect to controls and
// third-party plugin loading. The producer capability is bound to exact original
// executable bytes; normal connection settings/COM ports are never rewritten.
func commissionExecutionProvider(ctx context.Context, image *brokerstore.GenerationApplicationImage, cold *ExecutionColdProfile, profile executionHostProfile) (string, error) {
	provider, ok := profile.(*nativeExecutionProfileProvider)
	if !ok || provider == nil || provider.ExecutionProfileCapture == nil || cold == nil {
		return "", errExecutionAuthorization
	}
	captured := provider.ExecutionProfileCapture
	identity, err := validateExecutionCapability(ctx, image)
	if err != nil {
		return "", err
	}
	if err = cold.ValidateBeforeLaunch(ctx); err != nil {
		return "", err
	}
	snapshot, err := captured.Snapshot(ctx)
	if err != nil {
		return "", err
	}
	cold.state.mu.Lock()
	original := cold.state.original
	cold.state.mu.Unlock()
	digest, err := executionCommissionEvidence(identity, original, snapshot)
	if err != nil {
		return "", err
	}
	after, err := validateExecutionCapability(ctx, image)
	if err != nil || after != identity {
		return "", errors.Join(errExecutionAuthorization, err)
	}
	if err = errors.Join(cold.ValidateBeforeLaunch(ctx), captured.Validate(ctx)); err != nil {
		return "", err
	}
	return digest, ctx.Err()
}
