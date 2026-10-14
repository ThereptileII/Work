package brokerenroll

import (
	"bytes"
	"context"
	"errors"
	"io"
	"strings"
	"testing"

	verifier "example.com/opennav-update-verifier"
	"example.com/opennav-update-verifier/releasepolicy"
)

type admissionAuthorityFixture struct {
	in                                   installedCodeAdmissionInputs
	err, closeErr, releaseErr, launchErr error
	released                             bool
	closes, releases                     int
	events                               *[]string
}

func (a *admissionAuthorityFixture) check(ctx context.Context) error {
	if a.released {
		return errors.New("original completion released")
	}
	return errors.Join(ctx.Err(), a.err)
}
func (a *admissionAuthorityFixture) inputs(ctx context.Context) (installedCodeAdmissionInputs, error) {
	return a.in, a.check(ctx)
}
func (a *admissionAuthorityFixture) close() error {
	a.closes++
	*a.events = append(*a.events, "authority.close")
	return a.closeErr
}
func (a *admissionAuthorityFixture) releaseOriginal(ctx context.Context) error {
	a.releases++
	*a.events = append(*a.events, "original.release")
	a.released = true
	return errors.Join(ctx.Err(), a.releaseErr)
}
func (a *admissionAuthorityFixture) releaseRefresh(ctx context.Context) error { return ctx.Err() }
func (a *admissionAuthorityFixture) checkLaunch(ctx context.Context) error {
	if !a.released {
		return errors.New("original owner not consumed")
	}
	return errors.Join(ctx.Err(), a.launchErr)
}

type admissionImageFixture struct {
	*bytes.Reader
	err, closeErr error
	closes        int
	events        *[]string
}

func (i *admissionImageFixture) Path() string { return `C:\ProtectedCache\admitted.exe` }
func (i *admissionImageFixture) Check(ctx context.Context) error {
	return errors.Join(ctx.Err(), i.err)
}
func (i *admissionImageFixture) Close() error {
	i.closes++
	*i.events = append(*i.events, "image.close")
	return i.closeErr
}

type admissionFixture struct {
	authority             *admissionAuthorityFixture
	image                 *admissionImageFixture
	policy                releasepolicy.BrokerCodeUpgradePolicy
	raw                   []byte
	events                []string
	op                    installedCodeAdmissionOperations
	acquisitions, loaders int
}

func newAdmissionFixture(t *testing.T) *admissionFixture {
	t.Helper()
	e := newEnrollmentFixture(t)
	f := &admissionFixture{policy: releasepolicy.BrokerCodeUpgradePolicy{Schema: 1, Purpose: releasepolicy.BrokerCodeUpgradePurpose, PreviousBrokerSHA256: strings.Repeat("e", 64), PreviousHelperSHA256: strings.Repeat("f", 64), Bootstrap: e.policy}}
	var err error
	f.raw, err = releasepolicy.MarshalBrokerCodeUpgrade(f.policy)
	if err != nil {
		t.Fatal(err)
	}
	f.authority = &admissionAuthorityFixture{events: &f.events, in: installedCodeAdmissionInputs{discovery: verifier.BrokerCodeUpgradeDiscoveryRequest{MetadataURL: "https://private.invalid/metadata", TrustedRoot: []byte("retained root"), CacheDir: `C:\ProtectedCache`, Channel: "beta", ArtifactOrigin: "https://private.invalid", TrustConfigSHA256: e.req.TrustConfigSHA256}, trust: e.req.TrustConfiguration, original: strings.Repeat("d", 64), broker: f.policy.PreviousBrokerSHA256, helper: f.policy.PreviousHelperSHA256, currentBroker: f.policy.PreviousBrokerSHA256, currentHelper: f.policy.PreviousHelperSHA256, directory: `C:\ProtectedCache`}}
	f.image = &admissionImageFixture{Reader: bytes.NewReader([]byte("inert signed PE stand-in")), events: &f.events}
	f.op = installedCodeAdmissionOperations{
		open: func(context.Context) (installedCodeAdmissionAuthority, error) {
			f.events = append(f.events, "original.open")
			return f.authority, nil
		},
		discover: func(req verifier.BrokerCodeUpgradeDiscoveryRequest) (releasepolicy.BrokerCodeUpgradePolicy, []byte, error) {
			f.events = append(f.events, "signed.discover")
			if req.CacheDir != f.authority.in.directory || req.TrustConfigSHA256 != digest(f.authority.in.trust) {
				t.Fatal("discovery substituted original cache/trust")
			}
			return f.policy, bytes.Clone(f.raw), nil
		},
		acquire: func(ctx context.Context, a releasepolicy.Artifact, origin, directory string) (installedCodeAdmissionImage, error) {
			f.events = append(f.events, "image.acquire")
			f.acquisitions++
			if a != f.policy.Bootstrap.Provisioner || origin != f.authority.in.discovery.ArtifactOrigin || directory != f.authority.in.directory || f.authority.released {
				t.Fatal("acquisition substituted signed artifact or original custody")
			}
			return f.image, nil
		},
		loader: func(r io.ReaderAt, length int64) error {
			f.events = append(f.events, "loader.validate")
			f.loaders++
			if r != f.image || length != f.policy.Bootstrap.Provisioner.Bytes {
				t.Fatal("loader checked other image")
			}
			return nil
		},
	}
	return f
}
func TestInstalledCodeAdmissionRetainsExactTupleThroughExplicitConsume(t *testing.T) {
	f := newAdmissionFixture(t)
	n, err := acquireInstalledCodeAdmissionWith(context.Background(), f.op)
	if err != nil {
		t.Fatal(err)
	}
	s, err := n.Snapshot(context.Background())
	if err != nil || s.Path != f.image.Path() || s.CacheDirectory != f.authority.in.directory || !bytes.Equal(s.PolicyBytes, f.raw) || s.PolicySHA256 != digest(f.raw) || s.ProvisionerSHA256 != f.policy.Bootstrap.Provisioner.SHA256 || s.OriginalCompletionSHA256 != f.authority.in.original {
		t.Fatal("authenticated snapshot lost", err)
	}
	s.PolicyBytes[0] = 'x'
	s.TrustConfiguration[0] = 'x'
	again, err := n.Snapshot(context.Background())
	if err != nil || !bytes.Equal(again.PolicyBytes, f.raw) || digest(again.TrustConfiguration) != again.TrustConfigSHA256 {
		t.Fatal("snapshot bytes alias authority", err)
	}
	launch, err := n.ConsumeForLaunch(context.Background())
	if err != nil {
		t.Fatal(err)
	}
	if n.Check(context.Background()) == nil || f.authority.releases != 1 || f.image.closes != 0 || f.authority.closes != 0 {
		t.Fatal("consume retained original pins or released executable")
	}
	if _, err := n.ConsumeForLaunch(context.Background()); err == nil {
		t.Fatal("original admission consumed twice")
	}
	if err := n.Close(); err != nil || f.image.closes != 0 {
		t.Fatal("initial owner closed transferred executable", err)
	}
	if _, err := launch.Snapshot(context.Background()); err != nil {
		t.Fatal(err)
	}
	if err := launch.Close(); err != nil {
		t.Fatal(err)
	}
	if err := launch.Close(); err != nil || f.image.closes != 1 || f.authority.closes != 1 {
		t.Fatal("ownership not closed once", err)
	}
	if f.events[len(f.events)-2] != "image.close" || f.events[len(f.events)-1] != "authority.close" {
		t.Fatal("cache closed before image cleanup", f.events)
	}
}
func TestInstalledCodeAdmissionRefusesBeforeExecutionAndDrainsPartialOwners(t *testing.T) {
	for _, fault := range []string{"original", "trust", "predecessor", "raw-policy", "network", "download", "image", "loader", "cancel-after-download"} {
		t.Run(fault, func(t *testing.T) {
			f := newAdmissionFixture(t)
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			switch fault {
			case "original":
				f.authority.err = errors.New("original native ID differs")
			case "trust":
				f.authority.in.trust = []byte("foreign trust")
			case "predecessor":
				f.authority.in.broker = strings.Repeat("1", 64)
			case "raw-policy":
				f.raw = []byte("{}")
			case "network":
				f.op.discover = func(verifier.BrokerCodeUpgradeDiscoveryRequest) (releasepolicy.BrokerCodeUpgradePolicy, []byte, error) {
					return releasepolicy.BrokerCodeUpgradePolicy{}, nil, errors.New("network unavailable")
				}
			case "download":
				f.op.acquire = func(context.Context, releasepolicy.Artifact, string, string) (installedCodeAdmissionImage, error) {
					return f.image, errors.New("partial acquisition")
				}
			case "image":
				f.image.err = errors.New("native image identity changed")
			case "loader":
				f.op.loader = func(io.ReaderAt, int64) error { return errors.New("foreign DLL imports") }
			case "cancel-after-download":
				original := f.op.acquire
				f.op.acquire = func(ctx context.Context, a releasepolicy.Artifact, o, d string) (installedCodeAdmissionImage, error) {
					i, e := original(ctx, a, o, d)
					cancel()
					return i, e
				}
			}
			n, err := acquireInstalledCodeAdmissionWith(ctx, f.op)
			if err == nil || n != nil || errors.Is(err, ErrInstalledCodeUpgradeUnavailable) || errors.Is(err, ErrInstalledCodeUpgradeCurrent) {
				t.Fatal("unverified failure became admission/no-change", err)
			}
			if f.authority.closes != 1 || f.image.closes > 1 {
				t.Fatal("partial owner leak/double close")
			}
			if fault == "download" || fault == "image" || fault == "loader" || fault == "cancel-after-download" {
				if f.image.closes != 1 {
					t.Fatal("acquired image leaked")
				}
			}
			if f.authority.releases != 0 {
				t.Fatal("failed admission reached launch phase")
			}
		})
	}
}
func TestInstalledCodeAdmissionNoChangeRequiresAuthenticatedAbsenceOrReadyPair(t *testing.T) {
	for _, kind := range []string{"absent", "ready", "unready", "foreign-pair", "absent-custody-loss", "absent-cleanup-failure"} {
		t.Run(kind, func(t *testing.T) {
			f := newAdmissionFixture(t)
			if strings.HasPrefix(kind, "absent") {
				f.op.discover = func(verifier.BrokerCodeUpgradeDiscoveryRequest) (releasepolicy.BrokerCodeUpgradePolicy, []byte, error) {
					if kind == "absent-custody-loss" {
						f.authority.err = errors.New("completion lost")
					}
					return releasepolicy.BrokerCodeUpgradePolicy{}, nil, verifier.ErrAuthenticatedTargetAbsent
				}
				if kind == "absent-cleanup-failure" {
					f.authority.closeErr = errors.New("cleanup refused")
				}
			} else {
				f.authority.in.currentBroker = f.policy.Bootstrap.Broker.SHA256
				f.authority.in.currentHelper = f.policy.Bootstrap.Provisioner.SHA256
				f.authority.in.successorReady = kind != "unready"
				if kind == "foreign-pair" {
					f.authority.in.currentHelper = strings.Repeat("1", 64)
				}
			}
			n, err := acquireInstalledCodeAdmissionWith(context.Background(), f.op)
			if err == nil || n != nil || f.acquisitions != 0 || f.authority.closes != 1 {
				t.Fatal("no-change attempted executable or leaked authority", err)
			}
			if errors.Is(err, ErrInstalledCodeUpgradeUnavailable) != (kind == "absent") || errors.Is(err, ErrInstalledCodeUpgradeCurrent) != (kind == "ready") {
				t.Fatal("unverified no-change classification", kind, err)
			}
		})
	}
}
func TestInstalledCodeAdmissionConsumeFailureInvalidatesAndPreservesCleanupCause(t *testing.T) {
	for _, kind := range []string{"canceled", "release", "launch-check"} {
		t.Run(kind, func(t *testing.T) {
			f := newAdmissionFixture(t)
			n, err := acquireInstalledCodeAdmissionWith(context.Background(), f.op)
			if err != nil {
				t.Fatal(err)
			}
			cleanup := errors.New("image cleanup refused")
			f.image.closeErr = cleanup
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			switch kind {
			case "canceled":
				cancel()
			case "release":
				f.authority.releaseErr = errors.New("original pin close refused")
			case "launch-check":
				f.authority.launchErr = errors.New("cache custody lost")
			}
			launch, err := n.ConsumeForLaunch(ctx)
			if launch != nil || err == nil || !errors.Is(err, cleanup) || n.Check(context.Background()) == nil || f.image.closes != 1 || f.authority.closes != 1 {
				t.Fatal("failed consume retained usable owner or lost cleanup cause", err)
			}
		})
	}
}
