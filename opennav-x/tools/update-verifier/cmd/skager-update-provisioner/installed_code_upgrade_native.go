package main

import (
	"context"
	"io"

	"example.com/opennav-update-verifier/brokerenroll"
)

type installedCodeNativeAdmission struct {
	owner *brokerenroll.InstalledCodeAdmission
}
type installedCodeNativeLaunch struct {
	owner *brokerenroll.InstalledCodeLaunchAdmission
}

func installedCodeSnapshotFrom(s brokerenroll.InstalledCodeAdmissionSnapshot) installedCodeSnapshot {
	return installedCodeSnapshot{path: s.Path, directory: s.CacheDirectory, pins: installedCodeComparisons{s.PolicySHA256, s.ProvisionerSHA256, s.TrustConfigSHA256, s.OriginalCompletionSHA256}}
}
func (n *installedCodeNativeAdmission) check(ctx context.Context) error { return n.owner.Check(ctx) }
func (n *installedCodeNativeAdmission) close() error                    { return n.owner.Close() }
func (n *installedCodeNativeAdmission) snapshot(ctx context.Context) (installedCodeSnapshot, error) {
	s, err := n.owner.Snapshot(ctx)
	return installedCodeSnapshotFrom(s), err
}
func (n *installedCodeNativeAdmission) consume(ctx context.Context) (installedCodeLaunchOwner, error) {
	owner, err := n.owner.ConsumeForLaunch(ctx)
	if owner == nil {
		return nil, err
	}
	return &installedCodeNativeLaunch{owner}, err
}
func (n *installedCodeNativeLaunch) check(ctx context.Context) error { return n.owner.Check(ctx) }
func (n *installedCodeNativeLaunch) close() error                    { return n.owner.Close() }
func (n *installedCodeNativeLaunch) snapshot(ctx context.Context) (installedCodeSnapshot, error) {
	s, err := n.owner.Snapshot(ctx)
	return installedCodeSnapshotFrom(s), err
}
func runCustomerCodeUpgrade(ctx context.Context, output io.Writer) error {
	return runCustomerCodeUpgradeWith(ctx, output, installedCodeCustomerOperations{
		acquire: func(ctx context.Context) (installedCodeAdmissionOwner, error) {
			owner, err := brokerenroll.AcquireInstalledCodeAdmission(ctx)
			if owner == nil {
				return nil, err
			}
			return &installedCodeNativeAdmission{owner}, err
		},
		launch: launchInstalledCodeSuccessor, retainUntilExit: retainInstalledCodeUntilExit,
		current: brokerenroll.ErrInstalledCodeUpgradeCurrent, unavailable: brokerenroll.ErrInstalledCodeUpgradeUnavailable,
	})
}
func runInstalledCodeSuccessor(ctx context.Context, p installedCodeComparisons) error {
	return brokerenroll.RunInstalledCodeUpgradeSuccessor(ctx, brokerenroll.InstalledCodeUpgradeComparisons{PolicySHA256: p.policy, ProvisionerSHA256: p.provisioner, TrustConfigSHA256: p.trust, OriginalCompletionSHA256: p.completion})
}
