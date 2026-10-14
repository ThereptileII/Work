package brokerenroll

import (
	"bytes"
	"context"
	"errors"
	"io"

	verifier "example.com/opennav-update-verifier"
	"example.com/opennav-update-verifier/releasepolicy"
)

// InstalledCodeAdmissionSnapshot contains comparisons only. A copied snapshot
// cannot authorize execution: native launch must retain and recheck its owner.
type InstalledCodeAdmissionSnapshot struct {
	Path                                                                         string
	CacheDirectory                                                               string
	Policy                                                                       releasepolicy.BrokerCodeUpgradePolicy
	PolicyBytes, TrustConfiguration                                              []byte
	PolicySHA256, ProvisionerSHA256, TrustConfigSHA256, OriginalCompletionSHA256 string
}

var ErrInstalledCodeUpgradeUnavailable = errors.New("authenticated repository has no published broker code upgrade")
var ErrInstalledCodeUpgradeCurrent = errors.New("verified installed broker code successor is already current")

type installedCodeAdmissionInputs struct {
	discovery                           verifier.BrokerCodeUpgradeDiscoveryRequest
	trust                               []byte
	original, broker, helper, directory string
	currentBroker, currentHelper        string
	successorReady                      bool
}
type installedCodeAdmissionAuthority interface {
	check(context.Context) error
	inputs(context.Context) (installedCodeAdmissionInputs, error)
	close() error
	releaseOriginal(context.Context) error
	releaseRefresh(context.Context) error
	checkLaunch(context.Context) error
}
type installedCodeAdmissionImage interface {
	io.ReaderAt
	Path() string
	Check(context.Context) error
	Close() error
}
type installedCodeAdmissionOperations struct {
	open     func(context.Context) (installedCodeAdmissionAuthority, error)
	discover func(verifier.BrokerCodeUpgradeDiscoveryRequest) (releasepolicy.BrokerCodeUpgradePolicy, []byte, error)
	acquire  func(context.Context, releasepolicy.Artifact, string, string) (installedCodeAdmissionImage, error)
	loader   func(io.ReaderAt, int64) error
}

// InstalledCodeAdmission retains genuine original completion, its original
// protected cache/ancestry and exact downloaded successor through native launch.
// It grants no service, configuration, enrollment or replacement capability.
type InstalledCodeAdmission struct {
	self      *InstalledCodeAdmission
	authority installedCodeAdmissionAuthority
	image     installedCodeAdmissionImage
	snapshot  InstalledCodeAdmissionSnapshot
	closed    bool
	closeErr  error
}

// AcquireInstalledCodeAdmission is called by the authenticated elevated fixed
// installed helper. No path, SID, approval pin or replacement source is accepted.
func AcquireInstalledCodeAdmission(ctx context.Context) (*InstalledCodeAdmission, error) {
	return acquireInstalledCodeAdmissionWith(ctx, installedCodeAdmissionNativeOperations())
}

func acquireInstalledCodeAdmissionWith(ctx context.Context, op installedCodeAdmissionOperations) (result *InstalledCodeAdmission, resultErr error) {
	if ctx == nil || ctx.Err() != nil || op.open == nil || op.discover == nil || op.acquire == nil || op.loader == nil {
		return nil, errors.New("live native installed successor admission required")
	}
	ctx, cancel := context.WithTimeout(ctx, maxEnrollmentTime)
	defer cancel()
	n := &InstalledCodeAdmission{}
	n.self = n
	defer func() {
		if resultErr != nil {
			if closeErr := n.Close(); closeErr != nil {
				if errors.Is(resultErr, ErrInstalledCodeUpgradeUnavailable) || errors.Is(resultErr, ErrInstalledCodeUpgradeCurrent) {
					resultErr = closeErr
				} else {
					resultErr = errors.Join(resultErr, closeErr)
				}
			}
		}
	}()
	var err error
	n.authority, err = op.open(ctx)
	if err != nil || n.authority == nil {
		return nil, errors.Join(errors.New("genuine original installed authority required"), err)
	}
	if err = n.authority.check(ctx); err != nil {
		return nil, err
	}
	inputs, err := n.authority.inputs(ctx)
	if err != nil {
		return nil, err
	}
	if !validPin(inputs.original) || !validPin(inputs.broker) || !validPin(inputs.helper) || inputs.directory == "" || len(inputs.trust) == 0 || digest(inputs.trust) != inputs.discovery.TrustConfigSHA256 {
		return nil, errors.New("original completion trust and code custody required")
	}
	p, raw, err := op.discover(inputs.discovery)
	if err != nil {
		if errors.Is(err, verifier.ErrAuthenticatedTargetAbsent) {
			if checkErr := errors.Join(ctx.Err(), n.authority.check(ctx)); checkErr != nil {
				return nil, checkErr
			}
			return nil, ErrInstalledCodeUpgradeUnavailable
		}
		return nil, err
	}
	parsed, err := releasepolicy.ParseBrokerCodeUpgrade(raw)
	if err != nil || parsed != p || p.PreviousBrokerSHA256 != inputs.broker || p.PreviousHelperSHA256 != inputs.helper || p.Bootstrap.TrustConfigSHA256 != inputs.discovery.TrustConfigSHA256 || p.Bootstrap.Channel != inputs.discovery.Channel || validateMaintenanceArtifact(p.Bootstrap.Provisioner) != nil {
		return nil, errors.Join(errors.New("signed successor differs from original installed predecessor or trust"), err)
	}
	if inputs.successorReady && inputs.currentBroker == p.Bootstrap.Broker.SHA256 && inputs.currentHelper == p.Bootstrap.Provisioner.SHA256 {
		if checkErr := errors.Join(ctx.Err(), n.authority.check(ctx)); checkErr != nil {
			return nil, checkErr
		}
		return nil, ErrInstalledCodeUpgradeCurrent
	}
	if inputs.currentBroker != inputs.broker || inputs.currentHelper != inputs.helper {
		return nil, errors.New("installed code is neither original nor exact ready signed successor")
	}
	if err = errors.Join(ctx.Err(), n.authority.check(ctx)); err != nil {
		return nil, err
	}
	n.image, err = op.acquire(ctx, p.Bootstrap.Provisioner, inputs.discovery.ArtifactOrigin, inputs.directory)
	if err != nil || n.image == nil {
		return nil, errors.Join(errors.New("retained exact successor acquisition required"), err)
	}
	if err = errors.Join(ctx.Err(), n.authority.check(ctx), n.image.Check(ctx)); err != nil {
		return nil, err
	}
	if err = op.loader(n.image, p.Bootstrap.Provisioner.Bytes); err != nil {
		return nil, err
	}
	n.snapshot = InstalledCodeAdmissionSnapshot{Path: n.image.Path(), CacheDirectory: inputs.directory, Policy: p, PolicyBytes: bytes.Clone(raw), TrustConfiguration: bytes.Clone(inputs.trust), PolicySHA256: digest(raw), ProvisionerSHA256: p.Bootstrap.Provisioner.SHA256, TrustConfigSHA256: inputs.discovery.TrustConfigSHA256, OriginalCompletionSHA256: inputs.original}
	if n.snapshot.Path == "" {
		return nil, errors.New("retained successor path required")
	}
	if err = n.Check(ctx); err != nil {
		return nil, err
	}
	return n, nil
}

func (n *InstalledCodeAdmission) Check(ctx context.Context) error {
	if n == nil || n.self != n || n.closed || ctx == nil || ctx.Err() != nil || n.authority == nil || n.image == nil {
		return errors.New("live original installed admission owner required")
	}
	return errors.Join(n.authority.check(ctx), n.image.Check(ctx), ctx.Err())
}
func (n *InstalledCodeAdmission) Snapshot(ctx context.Context) (InstalledCodeAdmissionSnapshot, error) {
	if err := n.Check(ctx); err != nil {
		return InstalledCodeAdmissionSnapshot{}, err
	}
	s := n.snapshot
	s.PolicyBytes = bytes.Clone(s.PolicyBytes)
	s.TrustConfiguration = bytes.Clone(s.TrustConfiguration)
	return s, nil
}
func (n *InstalledCodeAdmission) Close() error {
	if n == nil {
		return nil
	}
	if n.self != n {
		return errors.New("installed admission owner cannot be copied")
	}
	if n.closed {
		return n.closeErr
	}
	n.closed = true
	// Release native image pins before VerifiedArtifact removes its temporary file;
	// acquisition ancestry and original completion remain retained until afterwards.
	if n.image != nil {
		n.closeErr = n.image.Close()
	}
	if n.authority != nil {
		n.closeErr = errors.Join(n.closeErr, n.authority.close())
	}
	if n.closeErr != nil {
		n.closeErr = errors.Join(ErrCleanup, n.closeErr)
	}
	return n.closeErr
}

// InstalledCodeLaunchAdmission is explicitly past original-completion read
// custody. Its retained cache and exact acquired image authenticate launch;
// the new provisioner must independently reopen completion before replacement.
type InstalledCodeLaunchAdmission struct {
	self  *InstalledCodeLaunchAdmission
	owner *InstalledCodeAdmission
}

func (n *InstalledCodeAdmission) ConsumeForLaunch(ctx context.Context) (*InstalledCodeLaunchAdmission, error) {
	if err := n.Check(ctx); err != nil {
		return nil, errors.Join(err, n.Close())
	}
	// No future check on this initial owner can claim retained original authority.
	n.closed = true
	if err := n.authority.releaseOriginal(ctx); err != nil {
		n.closed = false
		return nil, errors.Join(err, n.Close())
	}
	if err := n.authority.releaseRefresh(ctx); err != nil {
		n.closed = false
		return nil, errors.Join(err, n.Close())
	}
	next := &InstalledCodeAdmission{authority: n.authority, image: n.image, snapshot: n.snapshot}
	next.self = next
	n.authority, n.image = nil, nil
	launch := &InstalledCodeLaunchAdmission{owner: next}
	launch.self = launch
	if err := launch.Check(ctx); err != nil {
		return nil, errors.Join(err, launch.Close())
	}
	return launch, nil
}
func (n *InstalledCodeLaunchAdmission) Check(ctx context.Context) error {
	if n == nil || n.self != n || n.owner == nil || n.owner.closed || ctx == nil || ctx.Err() != nil {
		return errors.New("live consumed installed launch admission required")
	}
	return errors.Join(n.owner.authority.checkLaunch(ctx), n.owner.image.Check(ctx), ctx.Err())
}
func (n *InstalledCodeLaunchAdmission) Snapshot(ctx context.Context) (InstalledCodeAdmissionSnapshot, error) {
	if err := n.Check(ctx); err != nil {
		return InstalledCodeAdmissionSnapshot{}, err
	}
	s := n.owner.snapshot
	s.PolicyBytes = bytes.Clone(s.PolicyBytes)
	s.TrustConfiguration = bytes.Clone(s.TrustConfiguration)
	return s, nil
}
func (n *InstalledCodeLaunchAdmission) Close() error {
	if n == nil {
		return nil
	}
	if n.self != n {
		return errors.New("installed launch admission owner cannot be copied")
	}
	return n.owner.Close()
}
