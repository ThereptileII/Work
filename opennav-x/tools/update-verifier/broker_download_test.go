package verifier

import (
	"context"
	"errors"
	"io"
	"net/http"
	"net/http/httptest"
	"os"
	"path/filepath"
	"slices"
	"strings"
	"sync/atomic"
	"testing"

	"example.com/opennav-update-verifier/brokerstore"
	"example.com/opennav-update-verifier/releasepolicy"
)

// This portable fixture tests verifier/owner ordering and exact signed TLS
// acquisition, not Windows DACLs, sharing or distinct-principal qualification.
// Production cannot supply this adapter through the exported API.
type brokerLeaseFixture struct {
	t           *testing.T
	directory   string
	events      []string
	plan        brokerstore.AcquisitionPlan
	bound       bool
	closed      bool
	beginErr    error
	bindErr     error
	closeErr    error
	artifactErr error
	syncErr     error
	sealErr     error
	createErr   error
	createSlot  brokerstore.ArtifactSlot
	corruptSeal bool
	owners      []*brokerArtifactFixture
	closeCalls  int
}

func newBrokerLeaseFixture(t *testing.T) *brokerLeaseFixture {
	t.Helper()
	return &brokerLeaseFixture{t: t, directory: t.TempDir()}
}

func (a *brokerLeaseFixture) begin() (brokerAcquisition, error) {
	a.events = append(a.events, "begin")
	if a.beginErr != nil {
		return nil, a.beginErr
	}
	return a, nil
}

func (a *brokerLeaseFixture) Bind(plan brokerstore.AcquisitionPlan) error {
	a.events = append(a.events, "bind")
	if a.bindErr != nil {
		return a.bindErr
	}
	if a.bound {
		a.t.Fatal("lease rebound")
	}
	a.bound, a.plan = true, plan
	return nil
}

func (a *brokerLeaseFixture) create(slot brokerstore.ArtifactSlot) (brokerArtifact, error) {
	a.t.Helper()
	if !a.bound || a.closed {
		a.t.Fatal("file creation without live bound lease")
	}
	a.events = append(a.events, "create:"+string(slot))
	if a.createErr != nil && a.createSlot == slot {
		return nil, a.createErr
	}
	name := filepath.Join(a.directory, string(slot))
	writer, err := os.OpenFile(name, os.O_WRONLY|os.O_CREATE|os.O_EXCL, 0600)
	if err != nil {
		return nil, err
	}
	owner := &brokerArtifactFixture{lease: a, slot: slot, name: name, writer: writer}
	a.owners = append(a.owners, owner)
	return owner, nil
}

func (a *brokerLeaseFixture) Close() error {
	if !a.closed {
		a.closed = true
		a.closeCalls++
		a.events = append(a.events, "close:lease")
		for _, owner := range a.owners {
			if err := owner.Close(); err != nil {
				a.closeErr = errors.Join(a.closeErr, err)
			}
		}
	}
	return a.closeErr
}

type brokerArtifactFixture struct {
	lease    *brokerLeaseFixture
	slot     brokerstore.ArtifactSlot
	name     string
	writer   *os.File
	reader   *os.File
	closed   bool
	closeErr error
}

func (a *brokerArtifactFixture) Write(p []byte) (int, error) { return a.writer.Write(p) }
func (a *brokerArtifactFixture) Path() string                { return a.name }
func (a *brokerArtifactFixture) Sync() error {
	if a.lease.syncErr != nil {
		return a.lease.syncErr
	}
	return a.writer.Sync()
}
func (a *brokerArtifactFixture) SealReadOnly() (*os.File, error) {
	a.lease.events = append(a.lease.events, "seal:"+string(a.slot))
	if a.lease.sealErr != nil {
		return nil, a.lease.sealErr
	}
	if err := a.writer.Close(); err != nil {
		return nil, err
	}
	a.writer = nil
	if a.lease.corruptSeal {
		// Same-size alteration after the streamed hash must be caught by the
		// verifier's independent read from the returned readonly file.
		info, err := os.Stat(a.name)
		if err != nil {
			return nil, err
		}
		if err := os.WriteFile(a.name, []byte(strings.Repeat("x", int(info.Size()))), 0600); err != nil {
			return nil, err
		}
	}
	var err error
	a.reader, err = os.Open(a.name)
	return a.reader, err
}
func (a *brokerArtifactFixture) Close() error {
	if a.closed {
		return a.closeErr
	}
	a.closed = true
	a.lease.events = append(a.lease.events, "close:"+string(a.slot))
	if a.reader != nil {
		if _, err := a.reader.Stat(); err != nil {
			a.lease.t.Fatal("borrowed readonly file closed before native owner")
		}
		a.closeErr = errors.Join(a.closeErr, a.reader.Close())
	}
	if a.writer != nil {
		a.closeErr = errors.Join(a.closeErr, a.writer.Close())
	}
	a.closeErr = errors.Join(a.closeErr, os.Remove(a.name), a.lease.artifactErr)
	return a.closeErr
}

func watchBrokerRequests(f *recoveryFixture, lease *brokerLeaseFixture) {
	f.client.Transport = updateSetTransport{base: f.client.Transport, before: func(request *http.Request) {
		switch request.URL.Path {
		case "/" + f.policy.Recovery.Path:
			lease.events = append(lease.events, "request:recovery")
		case "/" + f.policy.Installer.Path:
			lease.events = append(lease.events, "request:installer")
		default:
			lease.events = append(lease.events, "request:metadata")
		}
	}}
}

func TestPrepareBrokerUpdateSetAuthenticatesAndRetainsManagedOwners(t *testing.T) {
	f := newRecoveryFixture(t, true)
	consent := f.check(t).Consent
	lease := newBrokerLeaseFixture(t)
	watchBrokerRequests(f, lease)
	p, err := prepareBrokerUpdateSet(context.Background(), f.config, f.observed, consent, lease.begin, f.client)
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { p.Close() })
	expected := brokerstore.AcquisitionPlan{PolicySHA256: consent.PolicySHA256, Version: consent.Version, Commit: consent.Commit,
		Recovery:  brokerstore.ArtifactSpec{Size: f.policy.Recovery.Bytes, SHA256: f.policy.Recovery.SHA256},
		Installer: brokerstore.ArtifactSpec{Size: f.policy.Installer.Bytes, SHA256: f.policy.Installer.SHA256}}
	if lease.plan != expected || !p.recoveryStageValid() || !p.RecoveryIdentity.LifecycleManifestAvailable ||
		p.RecoveryIdentity.LifecycleManifestSHA256 != recoveryDigest(f.entries["installer/package.json"]) {
		t.Fatal("lost exact authenticated plan or recovery contents")
	}
	ordered := []string{"begin", "request:metadata", "bind", "create:recovery", "request:recovery", "seal:recovery",
		"create:installer", "request:installer", "seal:installer"}
	last := -1
	for _, event := range ordered {
		index := slices.Index(lease.events, event)
		if index <= last {
			t.Fatalf("incorrect ownership/network ordering: %v", lease.events)
		}
		last = index
	}
	if f.requests.Load() != 1 || f.artifactRequests.Load() != 1 {
		t.Fatal("artifact acquisition repeated")
	}
	if data, err := io.ReadAll(p.Installer); err != nil || string(data) != "verified installer bytes" {
		t.Fatal("managed installer bytes unavailable")
	}
	for _, artifact := range []*VerifiedArtifact{p.Installer, p.Recovery} {
		if _, err := artifact.file.Write([]byte("tamper")); err == nil {
			t.Fatal("broker file not readonly")
		}
	}
	// A public informational Path is not cleanup authority. Close must use the
	// retained native owner, leaving an unrelated path completely untouched.
	unrelated := filepath.Join(t.TempDir(), "unrelated")
	if err := os.WriteFile(unrelated, []byte("preserved"), 0600); err != nil {
		t.Fatal(err)
	}
	p.Installer.path, p.Recovery.path = unrelated, unrelated
	if err := p.Close(); err != nil {
		t.Fatal(err)
	}
	if err := p.Close(); err != nil || lease.closeCalls != 1 {
		t.Fatal("managed acquisition close retried or failed")
	}
	if tail := lease.events[len(lease.events)-3:]; !slices.Equal(tail, []string{"close:installer", "close:recovery", "close:lease"}) {
		t.Fatalf("lease released before artifact owners: %v", tail)
	}
	if data, err := os.ReadFile(unrelated); err != nil || string(data) != "preserved" {
		t.Fatal("managed cleanup used informational artifact path")
	}
	assertDownloadDirectoryEmpty(t, lease.directory)
	assertDownloadDirectoryEmpty(t, f.config.ArtifactDirectory)
}

func TestPrepareBrokerUpdateSetRefusesBeforeArtifactCreation(t *testing.T) {
	for _, scenario := range []string{"begin", "bind", "consent", "current", "origin", "installer-origin", "old-recovery"} {
		t.Run(scenario, func(t *testing.T) {
			f := newRecoveryFixture(t, scenario != "old-recovery")
			consent := f.check(t).Consent
			lease := newBrokerLeaseFixture(t)
			wantRecovery := int32(0)
			switch scenario {
			case "begin":
				lease.beginErr = errors.New("reservation refused")
			case "bind":
				lease.bindErr = errors.New("plan binding refused")
			case "consent":
				consent.PolicySHA256 = strings.Repeat("f", 64)
			case "current":
				f.config.Installed = releasepolicy.Installed{Version: f.policy.Version, Commit: f.policy.Commit}
			case "origin":
				f.policy.Recovery.URL = "https://unapproved.invalid/" + f.policy.Recovery.Path
				f.repository.publish(t, 3, "beta", policyBytes(t, f.policy), false)
				consent, _ = policyConsent(f.policy)
			case "installer-origin":
				f.policy.Installer.URL = "https://unapproved.invalid/" + f.policy.Installer.Path
				f.repository.publish(t, 3, "beta", policyBytes(t, f.policy), false)
				consent, _ = policyConsent(f.policy)
			case "old-recovery":
				wantRecovery = 1
			}
			watchBrokerRequests(f, lease)
			p, err := prepareBrokerUpdateSet(context.Background(), f.config, f.observed, consent, lease.begin, f.client)
			if err == nil || p != nil || errors.Is(err, ErrArtifactCleanup) {
				t.Fatalf("unsafe result or spurious cleanup uncertainty: %v", err)
			}
			if f.requests.Load() != wantRecovery || f.artifactRequests.Load() != 0 {
				t.Fatal("rejected preparation acquired unexpected artifacts")
			}
			if scenario == "begin" {
				if !slices.Equal(lease.events, []string{"begin"}) {
					t.Fatalf("network before durable lease: %v", lease.events)
				}
			} else if !lease.closed {
				t.Fatal("rejected preparation retained lease")
			}
			if scenario != "old-recovery" && len(lease.owners) != 0 {
				t.Fatal("rejected policy created files")
			}
			assertDownloadDirectoryEmpty(t, lease.directory)
			assertDownloadDirectoryEmpty(t, f.config.ArtifactDirectory)
		})
	}
}

func TestPrepareBrokerUpdateSetFailuresDrainOwnedFilesAndLease(t *testing.T) {
	for _, scenario := range []string{"recovery-create", "installer-create", "recovery-hash", "installer-hash", "recovery-layout", "sync", "seal", "sealed-hash", "cancel"} {
		t.Run(scenario, func(t *testing.T) {
			f := newRecoveryFixture(t, true)
			consent := f.check(t).Consent
			lease := newBrokerLeaseFixture(t)
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			switch scenario {
			case "recovery-create":
				lease.createErr, lease.createSlot = errors.New("injected protected creation refusal"), brokerstore.RecoverySlot
			case "installer-create":
				lease.createErr, lease.createSlot = errors.New("injected protected creation refusal"), brokerstore.InstallerSlot
			case "recovery-hash":
				f.repository.files["/"+f.policy.Recovery.Path][0] ^= 1
			case "installer-hash":
				f.repository.files["/"+f.policy.Installer.Path][0] ^= 1
			case "recovery-layout":
				f.publish(t, []byte("signed bytes are not a Recovery ZIP"))
				f.repository.publish(t, 3, "beta", policyBytes(t, f.policy), false)
				consent = f.check(t).Consent
			case "sync":
				lease.syncErr = errors.New("injected sync failure")
			case "seal":
				lease.sealErr = errors.New("injected seal failure")
			case "sealed-hash":
				lease.corruptSeal = true
			case "cancel":
				f.client.Transport = updateSetTransport{base: f.client.Transport, before: func(request *http.Request) {
					if request.URL.Path == "/"+f.policy.Recovery.Path {
						cancel()
					}
				}}
			}
			p, err := prepareBrokerUpdateSet(ctx, f.config, f.observed, consent, lease.begin, f.client)
			if p != nil || err == nil || errors.Is(err, ErrArtifactCleanup) || !lease.closed || lease.closeCalls != 1 {
				t.Fatalf("acquisition failure lost cleanup or classification: %v", err)
			}
			if scenario == "cancel" && !errors.Is(err, context.Canceled) {
				t.Fatalf("cancellation lost: %v", err)
			}
			if scenario != "installer-hash" && f.artifactRequests.Load() != 0 {
				t.Fatal("installer requested after recovery refusal")
			}
			if scenario == "recovery-create" && f.requests.Load() != 0 {
				t.Fatal("recovery requested before protected file creation")
			}
			assertDownloadDirectoryEmpty(t, lease.directory)
		})
	}
}

func TestBrokerAcquisitionCleanupUncertaintySurvivesPrimaryFailure(t *testing.T) {
	for _, scenario := range []string{"pre-download", "partial-download", "completed"} {
		t.Run(scenario, func(t *testing.T) {
			f := newRecoveryFixture(t, true)
			consent := f.check(t).Consent
			lease := newBrokerLeaseFixture(t)
			cleanup := errors.New("native lease cleanup uncertain")
			ownerCleanup := errors.New("native file cleanup uncertain")
			primary := errors.New("plan rejected")
			lease.closeErr = cleanup
			switch scenario {
			case "pre-download":
				lease.bindErr = primary
			case "partial-download":
				lease.artifactErr = ownerCleanup
				f.repository.files["/"+f.policy.Recovery.Path][0] ^= 1
			}
			p, err := prepareBrokerUpdateSet(context.Background(), f.config, f.observed, consent, lease.begin, f.client)
			if scenario == "completed" {
				if err != nil {
					t.Fatal(err)
				}
				lease.artifactErr = ownerCleanup
				err = p.Close()
				if again := p.Close(); again.Error() != err.Error() || lease.closeCalls != 1 {
					t.Fatal("cleanup result not retained")
				}
				if !p.Installer.closed || !p.Recovery.closed {
					t.Fatal("first owner failure skipped other owner")
				}
			} else if p != nil {
				t.Fatal("failed operation returned set")
			}
			if !errors.Is(err, ErrArtifactCleanup) || !errors.Is(err, cleanup) || lease.closeCalls != 1 {
				t.Fatalf("cleanup uncertainty lost: %v", err)
			}
			if scenario == "pre-download" && !errors.Is(err, primary) {
				t.Fatal("primary preparation failure lost")
			}
			if scenario != "pre-download" && !errors.Is(err, ownerCleanup) {
				t.Fatal("native file cleanup error lost")
			}
			assertDownloadDirectoryEmpty(t, lease.directory)
		})
	}
}

func TestBrokerPreparationInvalidInputsNeverReserveOrNetwork(t *testing.T) {
	f := newRecoveryFixture(t, true)
	consent := f.check(t).Consent
	for _, scenario := range []string{"nil-context", "canceled", "invalid-consent"} {
		t.Run(scenario, func(t *testing.T) {
			lease := newBrokerLeaseFixture(t)
			ctx := context.Background()
			selected := consent
			switch scenario {
			case "nil-context":
				ctx = nil
			case "canceled":
				var cancel context.CancelFunc
				ctx, cancel = context.WithCancel(ctx)
				cancel()
			case "invalid-consent":
				selected = ConsentIdentity{}
			}
			p, err := prepareBrokerUpdateSet(ctx, f.config, f.observed, selected, lease.begin, f.client)
			if err == nil || p != nil || len(lease.events) != 0 {
				t.Fatal("invalid input reserved acquisition")
			}
		})
	}
	if p, err := PrepareBrokerUpdateSet(context.Background(), f.config, f.observed, consent, nil); err == nil || p != nil {
		t.Fatal("nil concrete candidate accepted")
	}
}

func TestBrokerDownloadURLAndRedirectRefusalBeforeFurtherRequests(t *testing.T) {
	var requests atomic.Int32
	server := httptest.NewTLSServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		requests.Add(1)
		http.Redirect(w, r, "/payload", http.StatusFound)
	}))
	t.Cleanup(server.Close)
	for _, scenario := range []string{"query", "redirect"} {
		t.Run(scenario, func(t *testing.T) {
			requests.Store(0)
			lease := newBrokerLeaseFixture(t)
			lease.bound = true // Isolate the downloader; full Bind ordering is tested above.
			artifact := downloadTestArtifact(server.URL, "inert bytes")
			artifact.Path, artifact.URL = "redirect", server.URL+"/redirect"
			if scenario == "query" {
				artifact.URL += "?refused=1"
			}
			p, err := downloadBrokerArtifact(context.Background(), artifact, server.URL, lease, brokerstore.RecoverySlot, server.Client())
			if err == nil || p != nil || errors.Is(err, ErrArtifactCleanup) {
				t.Fatalf("unsafe URL response accepted: %v", err)
			}
			wantRequests := int32(0)
			if scenario == "redirect" {
				wantRequests = 1
			}
			if requests.Load() != wantRequests || (scenario == "query" && len(lease.owners) != 0) {
				t.Fatal("URL validation or redirect refusal occurred too late")
			}
			if err := lease.Close(); err != nil {
				t.Fatal(err)
			}
			assertDownloadDirectoryEmpty(t, lease.directory)
		})
	}
}
