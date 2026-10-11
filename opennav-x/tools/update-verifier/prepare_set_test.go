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
)

type updateSetTransport struct {
	base   http.RoundTripper
	before func(*http.Request)
}

func (s updateSetTransport) RoundTrip(request *http.Request) (*http.Response, error) {
	s.before(request)
	return s.base.RoundTrip(request)
}

func TestPrepareUpdateSetOwnsBothArtifactsFromOneAuthenticatedPolicy(t *testing.T) {
	f := newRecoveryFixture(t, true)
	consent := f.check(t).Consent
	expectedPolicy := f.policy
	var checks atomic.Int32
	// Publish a different offer AFTER the set's one fresh authenticated check.
	// Both acquisitions must still use the exact confirmed snapshot, never a mix.
	f.client.Transport = updateSetTransport{base: f.client.Transport, before: func(request *http.Request) {
		if request.URL.Path == "/timestamp.json" {
			checks.Add(1)
		}
		if request.URL.Path == "/"+expectedPolicy.Recovery.Path {
			f.policy.Installer.SHA256 = strings.Repeat("f", 64)
			f.repository.publish(t, 3, "beta", policyBytes(t, f.policy), false)
		}
	}}
	p, err := prepareUpdateSet(context.Background(), f.config, f.observed, consent, f.client)
	if err != nil {
		t.Fatal(err)
	}
	defer p.Close()
	if checks.Load() != 1 || p.Policy.Installer != expectedPolicy.Installer || p.Policy.Recovery != expectedPolicy.Recovery || p.Consent != consent ||
		f.requests.Load() != 1 || f.artifactRequests.Load() != 1 || !p.RecoveryIdentity.LifecycleManifestAvailable ||
		p.RecoveryIdentity.LifecycleManifestSHA256 != recoveryDigest(f.entries["installer/package.json"]) {
		t.Fatal("acquisition set lost its confirmed policy or inner identity")
	}
	installer, err := io.ReadAll(p.Installer)
	if err != nil || string(installer) != "verified installer bytes" {
		t.Fatal("wrong retained Setup")
	}
	var zipSignature [4]byte
	if _, err := p.Recovery.ReadAt(zipSignature[:], 0); err != nil || string(zipSignature[:]) != "PK\x03\x04" {
		t.Fatal("Recovery owner unavailable")
	}
	for _, artifact := range []*VerifiedArtifact{p.Installer, p.Recovery} {
		if _, err := artifact.file.Write([]byte("tamper")); err == nil {
			t.Fatal("writable acquisition owner")
		}
	}
	if err := p.Close(); err != nil {
		t.Fatal(err)
	}
	for _, artifact := range []*VerifiedArtifact{p.Installer, p.Recovery} {
		if _, err := artifact.Read(make([]byte, 1)); !errors.Is(err, os.ErrClosed) {
			t.Fatal("closed set retained a readable owner")
		}
	}
	if err := p.Close(); err != nil {
		t.Fatal(err)
	}
	assertDownloadDirectoryEmpty(t, f.config.ArtifactDirectory)
}

func TestPrepareUpdateSetFailureCleansBothAcquisitions(t *testing.T) {
	for _, scenario := range []string{"old-recovery", "recovery-download", "recovery-layout", "setup-download", "cancel-before-setup", "changed-consent", "unapproved-recovery"} {
		t.Run(scenario, func(t *testing.T) {
			f := newRecoveryFixture(t, scenario != "old-recovery")
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			consent := f.check(t).Consent
			wantRecovery, wantInstaller := int32(1), int32(0)
			switch scenario {
			case "recovery-download":
				f.repository.files["/"+f.policy.Recovery.Path] = []byte("changed signed bytes")
			case "recovery-layout":
				f.publish(t, []byte("authenticated but unsupported ZIP"))
				f.repository.publish(t, 3, "beta", policyBytes(t, f.policy), false)
				consent = f.check(t).Consent
			case "setup-download":
				f.repository.files["/"+f.policy.Installer.Path] = []byte("changed Setup bytes")
				wantInstaller = 1
			case "cancel-before-setup":
				wantInstaller = 1 // cancellation at its transport boundary
				f.client.Transport = updateSetTransport{base: f.client.Transport, before: func(request *http.Request) {
					if request.URL.Path == "/"+f.policy.Installer.Path {
						cancel()
					}
				}}
			case "changed-consent":
				consent.PolicySHA256 = strings.Repeat("f", 64)
				wantRecovery = 0
			case "unapproved-recovery":
				f.policy.Recovery.URL = "https://unapproved.invalid/" + f.policy.Recovery.Path
				f.repository.publish(t, 3, "beta", policyBytes(t, f.policy), false)
				consent, _ = policyConsent(f.policy)
				wantRecovery = 0
			}
			p, err := prepareUpdateSet(ctx, f.config, f.observed, consent, f.client)
			if err == nil || p != nil {
				if p != nil {
					p.Close()
				}
				t.Fatal("unsafe acquisition set returned")
			}
			if f.requests.Load() != wantRecovery || f.artifactRequests.Load() != wantInstaller {
				t.Fatalf("unexpected acquisition requests: recovery=%d installer=%d", f.requests.Load(), f.artifactRequests.Load())
			}
			assertDownloadDirectoryEmpty(t, f.config.ArtifactDirectory)
		})
	}
}

func TestPreparedUpdateSetCloseAttemptsBothOwnersAfterCleanupFailures(t *testing.T) {
	f := newRecoveryFixture(t, true)
	consent := f.check(t).Consent
	p, err := prepareUpdateSet(context.Background(), f.config, f.observed, consent, f.client)
	if err != nil {
		t.Fatal(err)
	}
	installerPath, recoveryPath := p.Installer.path, p.Recovery.path
	defer func() { p.Installer.path = installerPath; p.Recovery.path = recoveryPath; p.Close() }()
	for n, artifact := range []*VerifiedArtifact{p.Installer, p.Recovery} {
		blocked := filepath.Join(t.TempDir(), "nonempty")
		if err := os.Mkdir(blocked, 0700); err != nil {
			t.Fatal(err)
		}
		if err := os.WriteFile(filepath.Join(blocked, "inert"), []byte{byte(n)}, 0600); err != nil {
			t.Fatal(err)
		}
		// A failed file removal exercises each real owner's cleanup error without
		// invalidating Windows' retained read-only handle or weakening sharing.
		artifact.path = blocked
	}
	if err := p.Close(); err == nil || strings.Count(err.Error(), "artifact cleanup failed") != 2 {
		t.Fatalf("both cleanup errors not retained: %v", err)
	}
	if p.Installer.file != nil || p.Recovery.file != nil {
		t.Fatal("first cleanup failure skipped another owner")
	}
}
