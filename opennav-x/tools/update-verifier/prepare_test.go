package verifier

import (
	"context"
	"errors"
	"io"
	"net/http"
	"os"
	"path/filepath"
	"strings"
	"sync/atomic"
	"testing"
	"time"

	"example.com/opennav-update-verifier/releasepolicy"
)

type prepareFixture struct {
	repository       *fixture
	config           PrepareConfig
	policy           releasepolicy.Policy
	observed         releasepolicy.OpenCpn
	client           *http.Client
	artifactRequests atomic.Int32
	metadataRequests atomic.Int32
}

type prepareCountingTransport struct {
	fixture *prepareFixture
	base    http.RoundTripper
}

func (t prepareCountingTransport) RoundTrip(r *http.Request) (*http.Response, error) {
	if r.URL.Path == "/"+t.fixture.policy.Installer.Path {
		t.fixture.artifactRequests.Add(1)
	} else {
		t.fixture.metadataRequests.Add(1)
	}
	return t.base.RoundTrip(r)
}

func newPrepareFixture(t *testing.T) *prepareFixture {
	t.Helper()
	f := &prepareFixture{repository: newFixture(t), policy: signedPolicyFixture()}
	f.policy.Installer = downloadTestArtifact(f.repository.server.URL, "verified installer bytes")
	f.repository.files["/"+f.policy.Installer.Path] = []byte("verified installer bytes")
	f.repository.publish(t, 1, "beta", policyBytes(t, f.policy), false)
	f.observed = f.policy.SupportedOpenCpn[0]
	f.config = PrepareConfig{
		Trust:             TrustConfig{BootstrapRoot: f.repository.root, MetadataURL: f.repository.server.URL, Channel: "beta"},
		StateDirectory:    filepath.Join(t.TempDir(), "state"),
		ArtifactOrigin:    f.repository.server.URL,
		ArtifactDirectory: filepath.Join(t.TempDir(), "artifacts"),
		Installed:         releasepolicy.Installed{Version: "1.9.0", Commit: strings.Repeat("f", 40)},
		OpenCpnAllowlist:  append([]releasepolicy.OpenCpn(nil), f.policy.SupportedOpenCpn...),
	}
	if err := makePrivateStateDirectory(f.config.ArtifactDirectory); err != nil {
		t.Fatal(err)
	}
	if err := InitializeState(f.config.StateDirectory, f.config.Trust); err != nil {
		t.Fatal(err)
	}
	f.client = f.repository.server.Client()
	f.client.Transport = prepareCountingTransport{fixture: f, base: f.client.Transport}
	return f
}

func (f *prepareFixture) check(t *testing.T) UpdateCandidate {
	t.Helper()
	candidate, err := checkUpdate(context.Background(), f.config, f.observed, f.client)
	if err != nil {
		t.Fatal(err)
	}
	if f.artifactRequests.Load() != 0 {
		t.Fatal("check downloaded an installer before consent")
	}
	return candidate
}

func TestPrepareSignedUpgradeBindsConsentAndOwnsArtifact(t *testing.T) {
	f := newPrepareFixture(t)
	candidate := f.check(t)
	if candidate.Decision != releasepolicy.Upgrade || candidate.Consent.Version != f.policy.Version || candidate.Consent.Commit != f.policy.Commit || candidate.Consent.PolicySHA256 != stateDigest(policyBytes(t, f.policy)) {
		t.Fatal("incorrect authenticated offer")
	}
	prepared, err := prepareUpdate(context.Background(), f.config, f.observed, candidate.Consent, f.client)
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { prepared.Close() })
	if prepared.Policy.Installer != f.policy.Installer || prepared.Consent != candidate.Consent || f.artifactRequests.Load() != 1 {
		t.Fatal("prepared a different installer")
	}
	bytes, err := io.ReadAll(prepared.Artifact)
	if err != nil || string(bytes) != "verified installer bytes" {
		t.Fatalf("wrong owned installer bytes: %v", err)
	}
	if err := prepared.Close(); err != nil {
		t.Fatal(err)
	}
	assertDownloadDirectoryEmpty(t, f.config.ArtifactDirectory)
}

func TestPrepareNeverDownloadsNonUpgradeOrIncompatiblePolicy(t *testing.T) {
	for _, scenario := range []string{"current", "downgrade", "same version conflict", "observed hash", "observed upstream", "observed architecture", "independent allowlist"} {
		t.Run(scenario, func(t *testing.T) {
			f := newPrepareFixture(t)
			consent := f.check(t).Consent
			switch scenario {
			case "current":
				f.config.Installed = releasepolicy.Installed{Version: f.policy.Version, Commit: f.policy.Commit}
			case "downgrade":
				f.config.Installed.Version = "3.0.0"
			case "same version conflict":
				f.config.Installed.Version = f.policy.Version
			case "observed hash":
				f.observed.ExecutableSHA256 = strings.Repeat("b", 64)
			case "observed upstream":
				f.observed.UpstreamCommit = strings.Repeat("b", 40)
			case "observed architecture":
				f.observed.Arch = "x64"
			case "independent allowlist":
				f.config.OpenCpnAllowlist[0].ExecutableSHA256 = strings.Repeat("b", 64)
			}
			prepared, err := prepareUpdate(context.Background(), f.config, f.observed, consent, f.client)
			if err == nil || prepared != nil || f.artifactRequests.Load() != 0 {
				t.Fatal("noneligible update downloaded an installer")
			}
			assertDownloadDirectoryEmpty(t, f.config.ArtifactDirectory)
		})
	}
}

func TestPrepareRejectsChangedConsentAndFullPolicy(t *testing.T) {
	for _, scenario := range []string{"version", "commit", "digest", "empty", "changed installer", "changed notes"} {
		t.Run(scenario, func(t *testing.T) {
			f := newPrepareFixture(t)
			consent := f.check(t).Consent
			switch scenario {
			case "version":
				consent.Version = "2.0.1"
			case "commit":
				consent.Commit = strings.Repeat("b", 40)
			case "digest":
				consent.PolicySHA256 = strings.Repeat("b", 64)
			case "empty":
				consent = ConsentIdentity{}
			case "changed installer":
				f.policy.Installer.SHA256 = strings.Repeat("b", 64)
			case "changed notes":
				f.policy.ReleaseNotes.SHA256 = strings.Repeat("b", 64)
			}
			if strings.HasPrefix(scenario, "changed") {
				f.repository.publish(t, 2, "beta", policyBytes(t, f.policy), false)
			}
			prepared, err := prepareUpdate(context.Background(), f.config, f.observed, consent, f.client)
			if err == nil || prepared != nil || f.artifactRequests.Load() != 0 {
				t.Fatal("mismatched offer acquired an installer")
			}
			assertDownloadDirectoryEmpty(t, f.config.ArtifactDirectory)
		})
	}
}

func TestPrepareRejectsUnconfiguredBeforeNetwork(t *testing.T) {
	for _, scenario := range []string{"origin", "credential origin", "root", "state", "artifact directory", "allowlist", "installed identity"} {
		t.Run(scenario, func(t *testing.T) {
			f := newPrepareFixture(t)
			consent, err := policyConsent(f.policy)
			if err != nil {
				t.Fatal(err)
			}
			switch scenario {
			case "origin":
				f.config.ArtifactOrigin = ""
			case "credential origin":
				f.config.ArtifactOrigin = "https://user:top-secret@example.invalid"
			case "root":
				f.config.Trust.BootstrapRoot = nil
			case "state":
				f.config.StateDirectory += "-missing"
			case "artifact directory":
				f.config.ArtifactDirectory += "-missing"
			case "allowlist":
				f.config.OpenCpnAllowlist = nil
			case "installed identity":
				f.config.Installed.Commit = "unknown"
			}
			prepared, err := prepareUpdate(context.Background(), f.config, f.observed, consent, f.client)
			if err == nil || prepared != nil || f.artifactRequests.Load() != 0 || f.metadataRequests.Load() != 0 {
				t.Fatal("unconfigured preparation reached the network")
			}
			if strings.Contains(err.Error(), "top-secret") {
				t.Fatal("configuration error exposed a credential")
			}
		})
	}
}

func TestPrepareRejectsInvalidSignedPolicyAndUntrustedChanges(t *testing.T) {
	for _, scenario := range []string{"schema", "tampered policy", "corrupt state", "artifact origin", "artifact bytes"} {
		t.Run(scenario, func(t *testing.T) {
			f := newPrepareFixture(t)
			consent := f.check(t).Consent
			switch scenario {
			case "schema":
				f.policy.Schema = 2
				f.repository.publish(t, 2, "beta", policyBytes(t, f.policy), false)
			case "tampered policy":
				f.repository.files["/targets/releases/beta.json"] = []byte("untrusted")
			case "corrupt state":
				if err := os.WriteFile(filepath.Join(f.config.StateDirectory, "state.json"), []byte("untrusted"), 0600); err != nil {
					t.Fatal(err)
				}
			case "artifact origin":
				f.config.ArtifactOrigin = "https://unconfigured.invalid"
			case "artifact bytes":
				f.repository.files["/"+f.policy.Installer.Path] = []byte("corrupted installer data")
			}
			prepared, err := prepareUpdate(context.Background(), f.config, f.observed, consent, f.client)
			if err == nil || prepared != nil {
				t.Fatal("invalid update prepared")
			}
			if scenario != "artifact bytes" && f.artifactRequests.Load() != 0 {
				t.Fatal("invalid update reached artifact acquisition")
			}
			assertDownloadDirectoryEmpty(t, f.config.ArtifactDirectory)
		})
	}
}

func TestPrepareCancellationBeforeAcquisition(t *testing.T) {
	f := newPrepareFixture(t)
	consent, err := policyConsent(f.policy)
	if err != nil {
		t.Fatal(err)
	}
	ctx, cancel := context.WithCancel(context.Background())
	cancel()
	if _, err := prepareUpdate(ctx, f.config, f.observed, consent, f.client); !errors.Is(err, context.Canceled) {
		t.Fatalf("canceled preparation: %v", err)
	}
	if f.metadataRequests.Load() != 0 || f.artifactRequests.Load() != 0 {
		t.Fatal("canceled preparation reached network")
	}
}

type prepareBlockedTransport struct {
	base    http.RoundTripper
	started chan struct{}
}

func (t prepareBlockedTransport) RoundTrip(r *http.Request) (*http.Response, error) {
	if r.URL.Path == "/timestamp.json" {
		close(t.started)
		<-r.Context().Done()
		return nil, r.Context().Err()
	}
	return t.base.RoundTrip(r)
}

func TestCheckCancellationReachesSignedMetadataTransport(t *testing.T) {
	f := newPrepareFixture(t)
	started := make(chan struct{})
	f.client.Transport = prepareBlockedTransport{base: f.client.Transport, started: started}
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	done := make(chan error, 1)
	go func() { _, err := checkUpdate(ctx, f.config, f.observed, f.client); done <- err }()
	select {
	case <-started:
		cancel()
	case <-time.After(5 * time.Second):
		t.Fatal("metadata request did not start")
	}
	select {
	case err := <-done:
		if !errors.Is(err, context.Canceled) {
			t.Fatalf("metadata cancellation: %v", err)
		}
	case <-time.After(5 * time.Second):
		t.Fatal("metadata cancellation did not finish")
	}
	if f.artifactRequests.Load() != 0 {
		t.Fatal("canceled metadata check downloaded installer")
	}
}
