//go:build windows

package brokerenroll

import (
	"context"
	"errors"

	verifier "example.com/opennav-update-verifier"
)

type nativeInstalledCodeSuccessorPreparation struct {
	authority installedCodeAdmissionAuthority
	source    maintenanceImageSource
	req       CodeUpgradeRequest
	released  bool
}

func (p *nativeInstalledCodeSuccessorPreparation) check(ctx context.Context) error {
	if p == nil || p.authority == nil || p.source == nil || ctx == nil || ctx.Err() != nil {
		return errors.New("retained native running successor required")
	}
	var err error
	if p.released {
		err = p.authority.checkLaunch(ctx)
	} else {
		err = p.authority.check(ctx)
	}
	return errors.Join(err, p.source.Check(ctx), ctx.Err())
}
func (p *nativeInstalledCodeSuccessorPreparation) request() CodeUpgradeRequest {
	r := p.req
	r.TrustConfiguration = append([]byte(nil), r.TrustConfiguration...)
	return r
}
func (p *nativeInstalledCodeSuccessorPreparation) releaseCompletion(ctx context.Context) error {
	if p.released {
		return errors.New("successor original completion already released")
	}
	if err := p.check(ctx); err != nil {
		return err
	}
	p.released = true
	if err := p.authority.releaseOriginal(ctx); err != nil {
		return err
	}
	// UpgradeEnrolledCode acquires the same common operator lock before its own
	// fresh exact verification. Release only this preliminary refresh coordinator.
	return p.authority.releaseRefresh(ctx)
}
func (p *nativeInstalledCodeSuccessorPreparation) close() error {
	var err error
	if p.source != nil {
		err = p.source.Close()
	}
	if p.authority != nil {
		err = errors.Join(err, p.authority.close())
	}
	return err
}
func prepareInstalledCodeSuccessorNative(ctx context.Context, c InstalledCodeUpgradeComparisons) (result installedCodeSuccessorPreparation, resultErr error) {
	p := &nativeInstalledCodeSuccessorPreparation{}
	defer func() {
		if resultErr != nil {
			resultErr = errors.Join(resultErr, p.close())
		}
	}()
	var err error
	p.authority, err = openInstalledCodeAdmissionAuthorityModeNative(ctx, false)
	if err != nil {
		return nil, err
	}
	inputs, err := p.authority.inputs(ctx)
	if err != nil {
		return nil, err
	}
	if inputs.original != c.OriginalCompletionSHA256 || inputs.discovery.TrustConfigSHA256 != c.TrustConfigSHA256 || inputs.currentBroker != inputs.broker || inputs.currentHelper != inputs.helper {
		return nil, errors.New("successor launch differs from original completion/trust/code")
	}
	policy, raw, err := verifier.VerifyBrokerCodeUpgrade(verifier.BrokerBootstrapRequest{MetadataURL: inputs.discovery.MetadataURL, TrustedRoot: inputs.discovery.TrustedRoot, CacheDir: inputs.discovery.CacheDir, Channel: inputs.discovery.Channel, ArtifactOrigin: inputs.discovery.ArtifactOrigin, ApprovedPolicySHA256: c.PolicySHA256, TrustedProvisionerSHA256: c.ProvisionerSHA256, TrustConfigSHA256: c.TrustConfigSHA256})
	if err != nil {
		return nil, err
	}
	if digest(raw) != c.PolicySHA256 || policy.PreviousBrokerSHA256 != inputs.broker || policy.PreviousHelperSHA256 != inputs.helper {
		return nil, errors.New("successor signed predecessor differs from actual original installation")
	}
	p.source, err = openMaintenanceImageNative(ctx, policy.Bootstrap.Provisioner)
	if err != nil {
		return nil, err
	}
	p.req = CodeUpgradeRequest{TrustConfiguration: append([]byte(nil), inputs.trust...), BootstrapCacheDir: inputs.directory, ArtifactDirectory: inputs.directory, PolicySHA256: c.PolicySHA256, ProvisionerSHA256: c.ProvisionerSHA256, TrustConfigSHA256: c.TrustConfigSHA256, OriginalCompletionSHA256: c.OriginalCompletionSHA256}
	return p, p.check(ctx)
}
