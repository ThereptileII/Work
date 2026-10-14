//go:build windows

package brokerenroll

import (
	"bytes"
	"context"
	"errors"
	"example.com/opennav-update-verifier/brokerregistration"
	"golang.org/x/sys/windows"
	"runtime"

	verifier "example.com/opennav-update-verifier"
	"example.com/opennav-update-verifier/releasepolicy"
	"example.com/opennav-update-verifier/repository"
)

type nativeInstalledCodeAdmissionAuthority struct {
	original                     *nativeInstalledMaintenanceAuthority
	cache                        *nativeEnrollmentFiles
	refresh                      *nativeInstalledCodeRefresh
	currentBroker, currentHelper startupFileBinding
}

func (a *nativeInstalledCodeAdmissionAuthority) check(ctx context.Context) error {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if a == nil || a.original == nil || a.cache == nil || a.refresh == nil || ctx == nil || ctx.Err() != nil {
		return errors.New("retained original installed cache authority required")
	}
	c := a.original
	return errors.Join(c.check(ctx), a.cache.Check(), a.refresh.check(ctx), installedMatchFile(ctx, c.brokerImage, a.currentBroker), installedMatchFile(ctx, c.helperImage, a.currentHelper), ctx.Err())
}
func (a *nativeInstalledCodeAdmissionAuthority) inputs(ctx context.Context) (installedCodeAdmissionInputs, error) {
	if err := a.check(ctx); err != nil {
		return installedCodeAdmissionInputs{}, err
	}
	c := a.original
	client, err := brokerregistration.ParseRetainedApprovedRequest(c.record.ClientRequest, c.record.ClientRequestSHA256)
	if err != nil {
		return installedCodeAdmissionInputs{}, err
	}
	trust, err := repository.ValidateTrustIdentity(client.TrustConfiguration)
	if err != nil {
		return installedCodeAdmissionInputs{}, err
	}
	return installedCodeAdmissionInputs{discovery: verifier.BrokerCodeUpgradeDiscoveryRequest{MetadataURL: trust.MetadataURL, TrustedRoot: bytes.Clone(trust.BootstrapRoot), CacheDir: a.cache.bootstrapCache.dos, Channel: trust.Channel, ArtifactOrigin: trust.ArtifactOrigin, TrustConfigSHA256: client.TrustConfigSHA256}, trust: bytes.Clone(client.TrustConfiguration), original: digest(c.originalRaw), broker: c.record.Broker.SHA256, helper: c.record.MaintenanceHelper.SHA256, directory: a.cache.bootstrapCache.dos, currentBroker: a.currentBroker.SHA256, currentHelper: a.currentHelper.SHA256, successorReady: a.currentBroker != c.record.Broker && a.currentHelper != c.record.MaintenanceHelper}, nil

}
func (a *nativeInstalledCodeAdmissionAuthority) close() error {
	var err error
	if a.refresh != nil {
		err = a.refresh.close()
	}
	if a.cache != nil {
		err = errors.Join(err, a.cache.Close())
	}
	if a.original != nil {
		err = errors.Join(err, a.original.close())
	}
	return err
}

type nativeInstalledCodeAdmissionImage struct {
	downloaded *verifier.VerifiedArtifact
	files      *nativeEnrollmentFiles
	pin        *enrollmentPin
}

func (n *nativeInstalledCodeAdmissionImage) Path() string { return n.pin.dos }
func (n *nativeInstalledCodeAdmissionImage) ReadAt(p []byte, off int64) (int, error) {
	return clientBaselinePinReader{n.pin}.ReadAt(p, off)
}
func (n *nativeInstalledCodeAdmissionImage) Check(ctx context.Context) error {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if ctx == nil || ctx.Err() != nil || n == nil || n.files == nil || n.pin == nil || n.downloaded == nil {
		return errors.New("retained native successor image required")
	}
	return errors.Join(n.files.Check(), n.pin.check(), ctx.Err())
}
func (n *nativeInstalledCodeAdmissionImage) Close() error {
	var err error
	if n.files != nil {
		err = n.files.Close()
	}
	if n.downloaded != nil {
		err = errors.Join(err, n.downloaded.Close())
	}
	return err
}
func acquireInstalledCodeImageNative(ctx context.Context, artifact releasepolicy.Artifact, origin, directory string) (result installedCodeAdmissionImage, resultErr error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	downloaded, err := verifier.DownloadArtifact(ctx, artifact, origin, directory)
	if err != nil {
		return nil, err
	}
	n := &nativeInstalledCodeAdmissionImage{downloaded: downloaded, files: &nativeEnrollmentFiles{}}
	defer func() {
		if resultErr != nil {
			resultErr = errors.Join(resultErr, n.Close())
		}
	}()
	// The retained DownloadArtifact handle denies write/delete sharing before this
	// native pin is opened, so no independently reopened path can replace its bytes.
	user, err := installedAdmissionUserNative()
	if err != nil {
		return nil, err
	}
	n.pin, err = maintenanceHoldOriginalImage(n.files, downloaded.Path(), user)
	if err != nil {
		return nil, err
	}
	if n.pin.length != artifact.Bytes {
		return nil, errors.New("successor size differs from signed artifact")
	}
	if err = startupHashPin(ctx, n.pin, artifact.SHA256); err != nil {
		return nil, err
	}
	return n, n.Check(ctx)
}

func (a *nativeInstalledCodeAdmissionAuthority) releaseOriginal(ctx context.Context) error {
	if err := a.check(ctx); err != nil {
		return err
	}
	err := a.original.close()
	a.original = nil
	return err
}
func (a *nativeInstalledCodeAdmissionAuthority) checkLaunch(ctx context.Context) error {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if a == nil || a.original != nil || a.cache == nil || ctx == nil || ctx.Err() != nil {
		return errors.New("explicit consumed admission phase required")
	}
	var refreshErr error
	if a.refresh != nil {
		refreshErr = a.refresh.check(ctx)
	}
	return errors.Join(a.cache.Check(), refreshErr, ctx.Err())
}
func installedAdmissionUserNative() (string, error) {
	if err := enrollmentOperator(); err != nil {
		return "", err
	}
	user, err := windows.GetCurrentProcessToken().GetTokenUser()
	if err != nil || user == nil || user.User.Sid == nil {
		return "", errors.Join(errors.New("actual installed admission account required"), err)
	}
	return user.User.Sid.String(), nil
}
func openInstalledCodeAdmissionAuthorityNative(ctx context.Context) (result installedCodeAdmissionAuthority, resultErr error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	return openInstalledCodeAdmissionAuthorityModeNative(ctx, true)
}
func openInstalledCodeAdmissionAuthorityModeNative(ctx context.Context, fixed bool) (result installedCodeAdmissionAuthority, resultErr error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	inputs, err := readInstalledCompletionModeNative(ctx, fixed)
	if err != nil {
		return nil, err
	}
	c, ok := inputs.completion.(*nativeInstalledMaintenanceAuthority)
	if !ok {
		return nil, errors.Join(errors.New("native installed completion required"), inputs.completion.close())
	}
	a := &nativeInstalledCodeAdmissionAuthority{original: c}
	defer func() {
		if resultErr != nil {
			resultErr = errors.Join(resultErr, a.close())
		}
	}()
	// The genuine installed reader above authenticated either original full IDs or
	// complete successor-ready closure. Capture its already validated actual pins.
	brokerSHA, err := installedCompletionHash(ctx, c.brokerImage)
	if err != nil {
		return nil, err
	}
	helperSHA, err := installedCompletionHash(ctx, c.helperImage)
	if err != nil {
		return nil, err
	}
	a.currentBroker, err = startupFileIdentity(ctx, c.brokerImage, brokerSHA)
	if err != nil {
		return nil, err
	}
	a.currentHelper, err = startupFileIdentity(ctx, c.helperImage, helperSHA)
	if err != nil {
		return nil, err
	}
	a.cache, err = openInstalledBootstrapCacheNative(ctx, c)
	if err != nil {
		return nil, err
	}
	a.refresh, err = openInstalledCodeRefreshNative(ctx, a.cache)
	if err != nil {
		return nil, err
	}
	return a, a.check(ctx)
}
func installedCodeAdmissionNativeOperations() installedCodeAdmissionOperations {
	return installedCodeAdmissionOperations{open: openInstalledCodeAdmissionAuthorityNative, discover: verifier.DiscoverBrokerCodeUpgrade, acquire: acquireInstalledCodeImageNative, loader: validateMaintenanceLoader}
}

func (a *nativeInstalledCodeAdmissionAuthority) releaseRefresh(ctx context.Context) error {
	if err := a.checkLaunch(ctx); err != nil {
		return err
	}
	if a.refresh == nil {
		return errors.New("original refresh lock already released")
	}
	err := a.refresh.close()
	a.refresh = nil
	return err
}
