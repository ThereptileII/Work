package verifier

import (
	"context"
	"errors"
	"io"
	"net/http"
	"os"
	"time"

	"example.com/opennav-update-verifier/brokerstore"
	"example.com/opennav-update-verifier/releasepolicy"
)

// These private seams exercise the real authenticated protocol with inert local
// files in portable tests. The exported entry point accepts only the concrete
// distinct-principal native Candidate, never a caller-provided storage adapter.
type brokerAcquisition interface {
	Bind(brokerstore.AcquisitionPlan) error
	create(brokerstore.ArtifactSlot) (brokerArtifact, error)
	Close() error
}

type brokerArtifact interface {
	io.Writer
	Sync() error
	Path() string
	SealReadOnly() (*os.File, error)
	Close() error
}

type nativeBrokerAcquisition struct{ *brokerstore.Acquisition }

func (a nativeBrokerAcquisition) create(slot brokerstore.ArtifactSlot) (brokerArtifact, error) {
	return a.Create(slot)
}

// PrepareBrokerUpdateSet acquires one freshly authenticated eligible policy's
// Recovery and Setup into a reserved broker Candidate. A durable acquisition
// lease is required before metadata networking; its exact plan is bound only
// after matching explicit consent and both installed-origin checks. The lease
// creates protected files before any artifact bytes and owns their cleanup.
//
// Keep the returned set alive throughout separately authorized use and close it
// on every path. This grants no installer, commissioning or launch authority;
// it does not provision the broker, adopt paths or enable a portable fallback.
func PrepareBrokerUpdateSet(ctx context.Context, config PrepareConfig, observed releasepolicy.OpenCpn, consent ConsentIdentity, candidate *brokerstore.Candidate) (*PreparedUpdateSet, error) {
	if candidate == nil {
		return nil, errors.New("reserved protected broker candidate required")
	}
	return prepareBrokerUpdateSet(ctx, config, observed, consent, func() (brokerAcquisition, error) {
		lease, err := candidate.BeginAcquisition()
		if err != nil {
			return nil, err
		}
		return nativeBrokerAcquisition{lease}, nil
	}, nil)
}

// Client and lease injection remain private to signed local TLS fixtures.
func prepareBrokerUpdateSet(ctx context.Context, config PrepareConfig, observed releasepolicy.OpenCpn, consent ConsentIdentity, begin func() (brokerAcquisition, error), client *http.Client) (_ *PreparedUpdateSet, resultErr error) {
	if ctx == nil || begin == nil {
		return nil, errors.New("broker acquisition context and reservation required")
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if !validConsentIdentity(consent) {
		return nil, errors.New("exact confirmed update identity required")
	}
	lease, err := begin()
	if err != nil {
		return nil, err
	}
	if lease == nil {
		return nil, errors.New("protected broker acquisition lease required")
	}
	set := &PreparedUpdateSet{acquisition: lease}
	defer func() {
		if resultErr != nil {
			resultErr = errors.Join(resultErr, set.Close())
		}
	}()
	candidate, err := checkUpdate(ctx, config, observed, client)
	if err != nil {
		return nil, err
	}
	if candidate.Decision != releasepolicy.Upgrade || candidate.Consent != consent {
		return nil, errors.New("broker update set requires the exact confirmed eligible upgrade")
	}
	if err := validateDownload(candidate.Policy.Recovery, config.ArtifactOrigin); err != nil {
		return nil, errors.New("signed recovery differs from installed acquisition policy")
	}
	if err := lease.Bind(brokerstore.AcquisitionPlan{
		PolicySHA256: consent.PolicySHA256, Version: consent.Version, Commit: consent.Commit,
		Recovery:  brokerstore.ArtifactSpec{Size: candidate.Policy.Recovery.Bytes, SHA256: candidate.Policy.Recovery.SHA256},
		Installer: brokerstore.ArtifactSpec{Size: candidate.Policy.Installer.Bytes, SHA256: candidate.Policy.Installer.SHA256},
	}); err != nil {
		return nil, err
	}
	set.Policy, set.Consent = candidate.Policy, candidate.Consent
	set.Recovery, err = downloadBrokerArtifact(ctx, candidate.Policy.Recovery, config.ArtifactOrigin, lease, brokerstore.RecoverySlot, client)
	if err != nil {
		return nil, err
	}
	set.RecoveryIdentity, err = readRecoveryIdentity(ctx, set.Recovery, candidate.Policy)
	if err != nil {
		return nil, err
	}
	if !set.RecoveryIdentity.LifecycleManifestAvailable {
		return nil, errors.New("authenticated recovery lacks the exact Lifecycle manifest sidecar")
	}
	set.Installer, err = downloadBrokerArtifact(ctx, candidate.Policy.Installer, config.ArtifactOrigin, lease, brokerstore.InstallerSlot, client)
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	sealRecoveryStage(set)
	return set, nil
}

func downloadBrokerArtifact(ctx context.Context, artifact releasepolicy.Artifact, origin string, lease brokerAcquisition, slot brokerstore.ArtifactSlot, injected *http.Client) (_ *VerifiedArtifact, resultErr error) {
	if ctx == nil || lease == nil {
		return nil, errors.New("protected broker acquisition context required")
	}
	ctx, cancel := context.WithTimeout(ctx, maxDownloadTime)
	defer cancel()
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if err := validateDownload(artifact, origin); err != nil {
		return nil, err
	}
	owner, err := lease.create(slot)
	if err != nil {
		return nil, err
	}
	if owner == nil {
		return nil, errors.New("protected broker artifact owner required")
	}
	defer func() {
		if resultErr != nil {
			if err := owner.Close(); err != nil {
				resultErr = errors.Join(resultErr, ErrArtifactCleanup, err)
			}
		}
	}()
	client := &http.Client{}
	if injected != nil {
		*client = *injected
	} else {
		transport := &http.Transport{
			Proxy: http.ProxyFromEnvironment, TLSHandshakeTimeout: 10 * time.Second,
			ResponseHeaderTimeout: 30 * time.Second, MaxResponseHeaderBytes: 64 << 10,
			DisableCompression: true, DisableKeepAlives: true,
		}
		defer transport.CloseIdleConnections()
		client.Transport = transport
	}
	client.Jar = nil
	client.Timeout = maxDownloadTime
	client.CheckRedirect = func(*http.Request, []*http.Request) error {
		return errors.New("artifact redirects prohibited")
	}
	request, err := http.NewRequestWithContext(ctx, http.MethodGet, artifact.URL, nil)
	if err != nil {
		return nil, errors.New("invalid artifact request")
	}
	request.Header.Set("Accept-Encoding", "identity")
	response, err := client.Do(request)
	if err != nil {
		return nil, downloadError(ctx, "artifact HTTPS request failed")
	}
	defer response.Body.Close()
	if response.StatusCode != http.StatusOK {
		return nil, errors.New("artifact response must have status 200")
	}
	if response.Header.Get("Content-Encoding") != "" && response.Header.Get("Content-Encoding") != "identity" {
		return nil, errors.New("artifact response encoding prohibited")
	}
	if response.ContentLength >= 0 && response.ContentLength != artifact.Bytes {
		return nil, errors.New("artifact response length differs from signed size")
	}
	if err := copyVerifiedDownload(ctx, response.Body, owner, artifact); err != nil {
		return nil, err
	}
	if err := owner.Sync(); err != nil {
		return nil, errors.New("artifact sync failed")
	}
	reader, err := owner.SealReadOnly()
	if err != nil {
		return nil, err
	}
	if reader == nil {
		return nil, errors.New("retained read-only broker artifact required")
	}
	info, err := reader.Stat()
	if err != nil || !info.Mode().IsRegular() || info.Size() != artifact.Bytes {
		return nil, errors.New("retained broker artifact size or type changed")
	}
	if err := copyVerifiedDownload(ctx, io.NewSectionReader(reader, 0, info.Size()), io.Discard, artifact); err != nil {
		return nil, err
	}
	if _, err := reader.Seek(0, io.SeekStart); err != nil {
		return nil, errors.New("artifact rewind failed")
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	return &VerifiedArtifact{file: reader, path: owner.Path(), managedCloser: owner}, nil
}
