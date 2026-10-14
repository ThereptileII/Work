package brokerenroll

import (
	"bytes"
	"context"
	"errors"
	"io"

	verifier "example.com/opennav-update-verifier"
	"example.com/opennav-update-verifier/releasepolicy"
	"example.com/opennav-update-verifier/repository"
)

// CodeUpgradeRequest is an explicit, independently approved operator action.
// Authenticate the NEW running provisioner and its loader chain before elevation.
// A UAC prompt, self hash, mutable adjacent file or enrollment target cannot
// establish that initial execution authority. The original completion pin and
// signed upgrade target bind one existing enrollment to one successor code pair.
type CodeUpgradeRequest struct {
	TrustConfiguration       []byte
	BootstrapCacheDir        string
	ArtifactDirectory        string
	PolicySHA256             string
	ProvisionerSHA256        string
	TrustConfigSHA256        string
	Resume                   bool
	OriginalCompletionSHA256 string
}

var ErrCodeUpgradeUncertain = errors.New("broker code upgrade reserved or effect uncertain; preserve service state and owned recovery files")

type codeUpgradeOwner interface {
	check(context.Context) error
	stage(context.Context, io.Reader, maintenanceImageSource) error
	reserve(context.Context) error
	quiesce(context.Context) error
	replace(context.Context) error
	validateStore(context.Context) error
	activate(context.Context) error
	ready(context.Context) error
	complete(context.Context) error
	close() error
}
type codeUpgradeOperations struct {
	openResume func(context.Context, releasepolicy.BrokerCodeUpgradePolicy, []byte, []byte, string) (codeUpgradeResumeOwner, error)
	trust      func(context.Context, []byte, string) (repository.Trust, error)
	hold       func(string, string) (io.Closer, error)
	verify     func(verifier.BrokerBootstrapRequest) (releasepolicy.BrokerCodeUpgradePolicy, []byte, error)
	source     func(context.Context, releasepolicy.Artifact) (maintenanceImageSource, error)
	acquire    func(context.Context, releasepolicy.Artifact, string, string) (acquiredBroker, error)
	open       func(context.Context, releasepolicy.BrokerCodeUpgradePolicy, []byte, []byte, string) (codeUpgradeOwner, error)
}

// UpgradeEnrolledCode changes only the fixed broker/helper pair. It never calls
// fresh enrollment, initializes/registers/resets a store, changes B0 publication
// or opens application/profile data. This bounded implementation permits one
// immutable successor; any prior reservation (including partial) refuses a new
// transaction. Ordinary maintenance has no upgrade capability.
func UpgradeEnrolledCode(ctx context.Context, r CodeUpgradeRequest) error {
	return upgradeEnrolledCodeWith(ctx, r, codeUpgradeOperations{
		openResume: openCodeResumeNative,
		trust: func(ctx context.Context, raw []byte, pin string) (repository.Trust, error) {
			return codeUpgradeTrustNativeMode(ctx, raw, pin, r.Resume)
		}, hold: holdOperatorDirectories, verify: verifier.VerifyBrokerCodeUpgrade, source: openMaintenanceImageNative,
		acquire: func(ctx context.Context, a releasepolicy.Artifact, o, d string) (acquiredBroker, error) {
			return verifier.DownloadArtifact(ctx, a, o, d)
		}, open: func(ctx context.Context, p releasepolicy.BrokerCodeUpgradePolicy, raw, trust []byte, pin string) (codeUpgradeOwner, error) {
			return openCodeUpgradeNativeMode(ctx, p, raw, trust, pin, false)
		},
	})
}
func upgradeEnrolledCodeWith(ctx context.Context, r CodeUpgradeRequest, op codeUpgradeOperations) (result error) {
	if ctx == nil || ctx.Err() != nil || !validPin(r.PolicySHA256) || !validPin(r.ProvisionerSHA256) || !validPin(r.TrustConfigSHA256) || !validPin(r.OriginalCompletionSHA256) || r.BootstrapCacheDir == "" || r.ArtifactDirectory == "" || len(r.TrustConfiguration) < 1 || len(r.TrustConfiguration) > maxTrustConfigurationBytes || op.trust == nil || op.hold == nil || op.verify == nil || op.source == nil || (!r.Resume && (op.acquire == nil || op.open == nil)) || (r.Resume && op.openResume == nil) {
		return errors.New("live complete independently authenticated code upgrade inputs required")
	}
	ctx, cancel := context.WithTimeout(ctx, maxEnrollmentTime)
	defer cancel()
	trustBytes := bytes.Clone(r.TrustConfiguration)
	if digest(trustBytes) != r.TrustConfigSHA256 {
		return errors.New("upgrade trust differs from approval")
	}
	trust, err := op.trust(ctx, trustBytes, r.OriginalCompletionSHA256)
	if err != nil {
		return err
	}
	inputs, err := op.hold(r.BootstrapCacheDir, r.ArtifactDirectory)
	if inputs != nil {
		defer func() {
			if e := inputs.Close(); e != nil {
				result = errors.Join(result, ErrCleanup, e)
			}
		}()
	}
	if err != nil || inputs == nil {
		return errors.Join(errors.New("protected upgrade input custody required"), err)
	}
	_, raw, err := op.verify(verifier.BrokerBootstrapRequest{MetadataURL: trust.MetadataURL, TrustedRoot: bytes.Clone(trust.BootstrapRoot), CacheDir: r.BootstrapCacheDir, Channel: trust.Channel, ArtifactOrigin: trust.ArtifactOrigin, ApprovedPolicySHA256: r.PolicySHA256, TrustedProvisionerSHA256: r.ProvisionerSHA256, TrustConfigSHA256: r.TrustConfigSHA256})
	if err != nil {
		return err
	}
	p, err := releasepolicy.ParseBrokerCodeUpgrade(raw)
	if err != nil || digest(raw) != r.PolicySHA256 || p.Bootstrap.Channel != trust.Channel || p.Bootstrap.TrustConfigSHA256 != r.TrustConfigSHA256 || p.Bootstrap.Provisioner.SHA256 != r.ProvisionerSHA256 || p.Bootstrap.Broker.Bytes > maxBrokerBytes || p.Bootstrap.Provisioner.Bytes > maxBrokerBytes {
		return errors.New("exact purpose-bound signed upgrade required")
	}
	source, err := op.source(ctx, p.Bootstrap.Provisioner)
	if source != nil {
		defer func() {
			if e := source.Close(); e != nil {
				result = errors.Join(result, ErrCleanup, e)
			}
		}()
	}
	if err != nil || source == nil {
		return errors.Join(errors.New("authenticated running new provisioner required"), err)
	}
	if err = source.Check(ctx); err != nil {
		return err
	}
	if r.Resume {
		owner, err := op.openResume(ctx, p, bytes.Clone(raw), trustBytes, r.OriginalCompletionSHA256)
		if owner != nil {
			defer func() {
				if e := owner.close(); e != nil {
					result = errors.Join(result, ErrCleanup, e)
				}
			}()
		}
		if err != nil || owner == nil {
			return errors.Join(errors.New("exact same-transaction native resume owner required"), err)
		}
		if err := errors.Join(ctx.Err(), source.Check(ctx), owner.check(ctx)); err != nil {
			return err
		}
		if err := owner.resume(ctx); err != nil {
			if errors.Is(err, ErrCodeUpgradeAborted) {
				if checkErr := errors.Join(ctx.Err(), source.Check(ctx), owner.check(ctx)); checkErr != nil {
					return errors.Join(ErrCodeUpgradeUncertain, checkErr)
				}
				return err
			}
			return errors.Join(ErrCodeUpgradeUncertain, err)
		}
		return errors.Join(ctx.Err(), source.Check(ctx), owner.check(ctx))
	}
	broker, err := op.acquire(ctx, p.Bootstrap.Broker, trust.ArtifactOrigin, r.ArtifactDirectory)
	if broker != nil {
		defer func() {
			if e := broker.Close(); e != nil {
				result = errors.Join(result, ErrCleanup, e)
			}
		}()
	}
	if err != nil || broker == nil {
		return errors.Join(errors.New("exact successor broker custody required"), err)
	}
	owner, err := op.open(ctx, p, bytes.Clone(raw), trustBytes, r.OriginalCompletionSHA256)
	if owner != nil {
		defer func() {
			if e := owner.close(); e != nil {
				result = errors.Join(result, ErrCleanup, e)
			}
		}()
	}
	if err != nil || owner == nil {
		return errors.Join(errors.New("exact original installed upgrade authority required"), err)
	}
	check := func() error { return errors.Join(ctx.Err(), source.Check(ctx), owner.check(ctx)) }
	if err = check(); err != nil {
		return err
	}
	// Reserve before the first SCM/code effect. Any subsequent failure must never
	// be reported as an untouched machine or retried as a new enrollment.
	reserved := true
	defer func() {
		if reserved && result != nil {
			result = errors.Join(ErrCodeUpgradeUncertain, result)
		}
	}()
	if err = owner.stage(ctx, broker, source); err != nil {
		return err
	}
	if err = check(); err != nil {
		return err
	}
	if err = owner.reserve(ctx); err != nil {
		return err
	}
	for _, step := range []func(context.Context) error{owner.quiesce, owner.replace, owner.validateStore, owner.activate, owner.ready, owner.complete} {
		if err = check(); err != nil {
			return err
		}
		if err = step(ctx); err != nil {
			return err
		}
	}
	return check()
}
