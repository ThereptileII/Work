package verifier

import (
	"context"
	"crypto/sha256"
	"encoding/hex"
	"errors"
	"io"
	"os"
	"path/filepath"
	"reflect"
	"runtime"
	"strings"
	"testing"

	"example.com/opennav-update-verifier/baselineevidence"
)

type recoverySinkFunc func(context.Context, string, io.Reader, int64, string) error

func (f recoverySinkFunc) WriteFile(ctx context.Context, name string, r io.Reader, size int64, digest string) error {
	return f(ctx, name, r, size, digest)
}

func (f recoverySinkFunc) WriteEvidence(context.Context, string, []byte, string) error { return nil }
func (f recoverySinkFunc) CompleteRecoveryEvidence(context.Context, baselineevidence.Identity) error {
	return nil
}

// Disposable file IO only: this fixture makes no protected-root custody claim.
type recoveryFileSink struct {
	root          string
	names         []string
	evidenceNames []string
	identity      RecoveryIdentity
}

func (s *recoveryFileSink) WriteFile(ctx context.Context, name string, r io.Reader, size int64, digest string) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	destination := filepath.Join(s.root, filepath.FromSlash(name))
	if err := os.MkdirAll(filepath.Dir(destination), 0700); err != nil {
		return err
	}
	f, err := os.OpenFile(destination, os.O_CREATE|os.O_EXCL|os.O_WRONLY, 0600)
	if err != nil {
		return err
	}
	h := sha256.New()
	n, copyErr := io.CopyBuffer(io.MultiWriter(f, h), r, make([]byte, 32<<10))
	closeErr := f.Close()
	if copyErr != nil || closeErr != nil || n != size || hex.EncodeToString(h.Sum(nil)) != digest {
		return errors.New("inert sink write differs")
	}
	s.names = append(s.names, name)
	return nil
}

func (s *recoveryFileSink) WriteEvidence(ctx context.Context, name string, raw []byte, digest string) error {
	if name != "FILE_SHA256.json" && name != "installer/package.json" && !baselineevidence.IsEngineHelper(name) && !baselineevidence.IsCommissioningHelper(name) {
		return errors.New("unexpected evidence")
	}
	if err := s.WriteFile(ctx, name, strings.NewReader(string(raw)), int64(len(raw)), digest); err != nil {
		return err
	}
	s.names = s.names[:len(s.names)-1]
	s.evidenceNames = append(s.evidenceNames, name)
	return nil
}
func (s *recoveryFileSink) CompleteRecoveryEvidence(ctx context.Context, identity baselineevidence.Identity) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	s.identity = identity
	return nil
}

func prepareStageFixture(t *testing.T, engine ...bool) (*recoveryFixture, *PreparedUpdateSet) {
	t.Helper()
	f := newRecoveryFixture(t, true)
	// Preserve noncanonical valid whitespace to detect any reserialization.
	f.entries["installer/package.json"] = append([]byte(" \n"), f.entries["installer/package.json"]...)
	if len(engine) > 0 && engine[0] {
		for name, raw := range recoveryEngineFixture() {
			f.entries[name] = raw
		}
	}
	if len(engine) > 1 && engine[1] {
		for name, raw := range recoveryCommissioningFixture() {
			f.entries[name] = raw
		}
	}
	inventory := map[string]string{}
	for name, raw := range f.entries {
		if name != "FILE_SHA256.json" {
			inventory[name] = recoveryDigest(raw)
		}
	}
	f.entries["FILE_SHA256.json"] = append(recoveryJSON(t, inventory), []byte("\n ")...)
	f.publish(t, recoveryArchive(t, f.entries, nil))
	offer := f.check(t)
	p, err := prepareUpdateSet(context.Background(), f.config, f.observed, offer.Consent, f.client)
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { p.Close() })
	return f, p
}

// Distinct non-executable fixture bytes preserve BOM/CRLF and whitespace. These
// are never run and cannot be mistaken for an installer helper implementation.
func recoveryEngineFixture() map[string][]byte {
	return map[string][]byte{
		"installer/engine/Lifecycle.ps1":         []byte("\xef\xbb\xbf# INERT lifecycle fixture\r\n"),
		"installer/engine/UpdateSupervisor.ps1":  []byte("# INERT supervisor fixture\r\n\r\n"),
		"installer/engine/UpdateTransaction.ps1": []byte("# INERT transaction fixture\n "),
	}
}

func TestStageRecoveryRetainsOriginalEngineBeforeCompletion(t *testing.T) {
	f, p := prepareStageFixture(t, true)
	sink := &recoveryFileSink{root: t.TempDir()}
	identity, err := p.StageRecovery(context.Background(), sink)
	if err != nil || identity != p.RecoveryIdentity || sink.identity != identity || len(sink.evidenceNames) != 5 {
		t.Fatalf("original engine completion missing: %v", err)
	}
	for name, raw := range recoveryEngineFixture() {
		got, err := os.ReadFile(filepath.Join(sink.root, filepath.FromSlash(name)))
		if err != nil || string(got) != string(raw) || string(got) != string(f.entries[name]) {
			t.Fatalf("original helper bytes changed: %s: %v", name, err)
		}
	}
	for _, name := range sink.names {
		if baselineevidence.IsEngineHelper(name) {
			t.Fatal("engine incorrectly installed as app/docs payload")
		}
	}
}

func TestStageRecoveryEngineSinkFailureNeverCompletes(t *testing.T) {
	_, p := prepareStageFixture(t, true)
	for _, scenario := range []string{"refused", "canceled"} {
		t.Run(scenario, func(t *testing.T) {
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			complete := false
			sink := recoveryEvidenceFaultSink{
				recoverySinkFunc: func(_ context.Context, _ string, r io.Reader, _ int64, _ string) error {
					_, err := io.Copy(io.Discard, r)
					return err
				},
				writeEvidence: func(_ context.Context, name string, _ []byte, _ string) error {
					if baselineevidence.IsEngineHelper(name) {
						if scenario == "canceled" {
							cancel()
						}
						return errors.New("private engine destination")
					}
					return nil
				},
				complete: func(context.Context, baselineevidence.Identity) error { complete = true; return nil },
			}
			identity, err := p.StageRecovery(ctx, sink)
			if err == nil || identity != (RecoveryIdentity{}) || complete || strings.Contains(err.Error(), "private") {
				t.Fatalf("failed helper supplied completion or detail: %v", err)
			}
			if scenario == "canceled" && !errors.Is(err, context.Canceled) {
				t.Fatal("engine cancellation lost")
			}
		})
	}
}

func TestStageRecoveryWritesExactAuthenticatedInstalledPayload(t *testing.T) {
	f, p := prepareStageFixture(t)
	sink := &recoveryFileSink{root: t.TempDir()}
	identity, err := p.StageRecovery(context.Background(), sink)
	if err != nil || identity != p.RecoveryIdentity || !identity.LifecycleManifestAvailable ||
		identity.LifecycleManifestSHA256 != recoveryDigest(f.entries["installer/package.json"]) {
		t.Fatalf("lost authenticated inner identity: %v", err)
	}
	var expected []string
	for _, name := range recoveryNames(f.entries) {
		if name == "FILE_SHA256.json" || name == "installer/package.json" {
			got, err := os.ReadFile(filepath.Join(sink.root, filepath.FromSlash(name)))
			if err != nil || string(got) != string(f.entries[name]) {
				t.Fatalf("original evidence bytes differ: %s: %v", name, err)
			}
			continue
		}
		if !(strings.HasPrefix(name, "app/") || strings.HasPrefix(name, "docs/")) || name == "app/OPENNAV_PORTABLE_PREVIEW" {
			if _, err := os.Stat(filepath.Join(sink.root, filepath.FromSlash(name))); !errors.Is(err, os.ErrNotExist) {
				t.Fatalf("non-payload member staged: %s", name)
			}
			continue
		}
		expected = append(expected, name)
		got, err := os.ReadFile(filepath.Join(sink.root, filepath.FromSlash(name)))
		if err != nil || string(got) != string(f.entries[name]) {
			t.Fatalf("actual staged bytes differ: %s: %v", name, err)
		}
	}
	if sink.identity != identity || !reflect.DeepEqual(sink.evidenceNames, []string{"FILE_SHA256.json", "installer/package.json"}) {
		t.Fatal("original evidence or final identity binding absent")
	}
	if !reflect.DeepEqual(sink.names, expected) || p.Installer.file == nil || p.Recovery.file == nil {
		t.Fatal("wrong deterministic payload set or released acquisition owner")
	}
}

func TestStageRecoveryRefusesUnsealedMutatedOrClosedSets(t *testing.T) {
	for _, scenario := range []string{"unsealed", "copied", "policy", "nested-policy", "consent", "identity", "substituted-recovery", "substituted-installer", "closed-recovery", "closed-installer", "closed-set"} {
		t.Run(scenario, func(t *testing.T) {
			_, p := prepareStageFixture(t)
			originalInstaller, originalRecovery := p.Installer, p.Recovery
			defer func() { p.Installer, p.Recovery = originalInstaller, originalRecovery }()
			target := p
			switch scenario {
			case "unsealed":
				target = &PreparedUpdateSet{Policy: p.Policy, Consent: p.Consent, Installer: p.Installer, Recovery: p.Recovery, RecoveryIdentity: p.RecoveryIdentity}
			case "copied":
				copy := *p
				target = &copy
			case "policy":
				p.Policy.Recovery.SHA256 = strings.Repeat("f", 64)
			case "nested-policy":
				p.Policy.SupportedOpenCpn[0].Version = "9.9.9"
			case "consent":
				p.Consent.PolicySHA256 = strings.Repeat("f", 64)
			case "identity":
				p.RecoveryIdentity.ExecutableSHA256 = strings.Repeat("f", 64)
			case "substituted-recovery":
				p.Recovery = p.Installer
			case "substituted-installer":
				p.Installer = p.Recovery
			case "closed-recovery":
				p.Recovery.Close()
			case "closed-installer":
				p.Installer.Close()
			case "closed-set":
				p.Close()
			}
			calls := 0
			identity, err := target.StageRecovery(context.Background(), recoverySinkFunc(func(context.Context, string, io.Reader, int64, string) error {
				calls++
				return nil
			}))
			if err == nil || identity != (RecoveryIdentity{}) || calls != 0 {
				t.Fatal("unsealed, changed or closed set supplied staging provenance")
			}
		})
	}
}

func TestStageRecoveryRefusesCanceledAndFaultySinksWithoutIdentity(t *testing.T) {
	for _, scenario := range []string{"canceled-before", "canceled-during", "sink-failure", "unread-success", "partial-success", "set-closed-by-sink", "nil-context", "nil-sink"} {
		t.Run(scenario, func(t *testing.T) {
			_, p := prepareStageFixture(t)
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			calls := 0
			var sink RecoverySink = recoverySinkFunc(func(ctx context.Context, name string, reader io.Reader, size int64, digest string) error {
				calls++
				switch scenario {
				case "canceled-during":
					cancel()
					_, err := io.Copy(io.Discard, reader)
					return err
				case "sink-failure":
					return errors.New("private-destination-path")
				case "partial-success":
					_, _ = io.CopyN(io.Discard, reader, size-1)
				case "set-closed-by-sink":
					_, _ = io.Copy(io.Discard, reader)
					p.Close()
				}
				return nil
			})
			var stageCtx context.Context = ctx
			switch scenario {
			case "canceled-before":
				cancel()
			case "nil-context":
				stageCtx = nil
			case "nil-sink":
				sink = nil
			}
			identity, err := p.StageRecovery(stageCtx, sink)
			if err == nil || identity != (RecoveryIdentity{}) || strings.Contains(err.Error(), "private-destination-path") {
				t.Fatalf("staging failure supplied identity or leaked destination: %v", err)
			}
			if strings.HasPrefix(scenario, "canceled-") && !errors.Is(err, context.Canceled) {
				t.Fatal("staging lost cancellation")
			}
			if (scenario == "canceled-before" || strings.HasPrefix(scenario, "nil-")) && calls != 0 {
				t.Fatal("invalid staging request reached sink")
			}
		})
	}
}

func TestStageRecoveryUnavailableForOldOrMalformedAuthenticatedRecovery(t *testing.T) {
	for _, scenario := range []string{"old-layout", "malformed-zip", "inventory-mismatch", "unsafe-path"} {
		t.Run(scenario, func(t *testing.T) {
			f := newRecoveryFixture(t, scenario != "old-layout")
			if scenario != "old-layout" {
				archive := []byte("authenticated unsupported ZIP")
				if scenario == "inventory-mismatch" {
					f.entries["FILE_SHA256.json"] = []byte("{}")
					archive = recoveryArchive(t, f.entries, nil)
				}
				if scenario == "unsafe-path" {
					f.entries["app/../escape"] = []byte("inert")
					archive = recoveryArchive(t, f.entries, nil)
				}
				f.publish(t, archive)
			}
			offer := f.check(t)
			p, err := prepareUpdateSet(context.Background(), f.config, f.observed, offer.Consent, f.client)
			if err == nil || p != nil || f.requests.Load() != 1 || f.artifactRequests.Load() != 0 {
				if p != nil {
					p.Close()
				}
				t.Fatal("invalid signed Recovery produced staging-capable acquisition")
			}
			assertDownloadDirectoryEmpty(t, f.config.ArtifactDirectory)
		})
	}
}

func TestStageRecoveryRechecksActualRetainedBytes(t *testing.T) {
	if runtime.GOOS == "windows" {
		t.Skip("portable writer violates custody precondition; Windows retained-sharing qualification remains separate")
	}
	_, p := prepareStageFixture(t)
	path := p.Recovery.Path()
	archive, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}
	archive[len(archive)/2] ^= 1
	if err := os.WriteFile(path, archive, 0600); err != nil {
		t.Fatal(err)
	}
	calls := 0
	identity, err := p.StageRecovery(context.Background(), recoverySinkFunc(func(context.Context, string, io.Reader, int64, string) error {
		calls++
		return nil
	}))
	if err == nil || identity != (RecoveryIdentity{}) || calls != 0 {
		t.Fatal("staging trusted cached hashes after actual retained bytes changed")
	}
}

// A payload-only sink cannot silently discard original sidecar evidence.
type payloadOnlyRecoverySink struct{ calls int }

func (s *payloadOnlyRecoverySink) WriteFile(context.Context, string, io.Reader, int64, string) error {
	s.calls++
	return nil
}

type recoveryEvidenceFaultSink struct {
	recoverySinkFunc
	writeEvidence func(context.Context, string, []byte, string) error
	complete      func(context.Context, baselineevidence.Identity) error
}

func (s recoveryEvidenceFaultSink) WriteEvidence(ctx context.Context, name string, raw []byte, hash string) error {
	return s.writeEvidence(ctx, name, raw, hash)
}
func (s recoveryEvidenceFaultSink) CompleteRecoveryEvidence(ctx context.Context, identity baselineevidence.Identity) error {
	return s.complete(ctx, identity)
}

func TestStageRecoveryRequiresOriginalEvidenceCompletion(t *testing.T) {
	f, p := prepareStageFixture(t)
	payloadOnly := &payloadOnlyRecoverySink{}
	if identity, err := p.StageRecovery(context.Background(), payloadOnly); err == nil || identity != (RecoveryIdentity{}) || payloadOnly.calls != 0 {
		t.Fatal("payload-only sink accepted")
	}
	for _, scenario := range []string{"inventory-failure", "manifest-failure", "completion-failure", "cancel-inventory", "cancel-manifest", "cancel-completion", "sink-mutates-owned-copy"} {
		t.Run(scenario, func(t *testing.T) {
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			var names []string
			completed := false
			sink := recoveryEvidenceFaultSink{
				recoverySinkFunc: func(ctx context.Context, name string, r io.Reader, size int64, hash string) error {
					_, err := io.Copy(io.Discard, r)
					return err
				},
				writeEvidence: func(ctx context.Context, name string, raw []byte, hash string) error {
					names = append(names, name)
					if string(raw) != string(f.entries[name]) || recoveryDigest(raw) != hash {
						t.Fatal("lost exact original")
					}
					if scenario == "inventory-failure" && name == "FILE_SHA256.json" || scenario == "manifest-failure" && name == "installer/package.json" {
						return errors.New("private evidence path")
					}
					if scenario == "cancel-inventory" && name == "FILE_SHA256.json" || scenario == "cancel-manifest" && name == "installer/package.json" {
						cancel()
					}
					if scenario == "sink-mutates-owned-copy" {
						raw[0] ^= 1
					}
					return nil
				},
				complete: func(ctx context.Context, identity baselineevidence.Identity) error {
					completed = true
					if !reflect.DeepEqual(names, []string{"FILE_SHA256.json", "installer/package.json"}) || identity != p.RecoveryIdentity {
						t.Fatal("completion before originals or changed identity")
					}
					if scenario == "completion-failure" {
						return errors.New("private completion path")
					}
					if scenario == "cancel-completion" {
						cancel()
					}
					return nil
				},
			}
			identity, err := p.StageRecovery(ctx, sink)
			if scenario == "sink-mutates-owned-copy" {
				if err != nil || identity != p.RecoveryIdentity {
					t.Fatal("sink copy mutation changed original identity")
				}
				return
			}
			if err == nil || identity != (RecoveryIdentity{}) || strings.Contains(err.Error(), "private") {
				t.Fatalf("evidence failure leaked identity/detail: %v", err)
			}
			if strings.HasPrefix(scenario, "cancel-") && !errors.Is(err, context.Canceled) {
				t.Fatal("lost evidence cancellation")
			}
			if (scenario == "inventory-failure" || scenario == "manifest-failure" || scenario == "cancel-inventory" || scenario == "cancel-manifest") && completed {
				t.Fatal("completed after evidence failure")
			}
		})
	}
}

func recoveryCommissioningFixture() map[string][]byte {
	return map[string][]byte{
		"installer/commissioning-v1/commission-read-only.ps1":        []byte("\xef\xbb\xbf# inert commission-read-only.ps1\r\n "),
		"installer/commissioning-v1/verify-commissioning-launch.ps1": []byte("\xef\xbb\xbf# inert verify-commissioning-launch.ps1\r\n "),
		"installer/commissioning-v1/Commissioning.ps1":               []byte("\xef\xbb\xbf# inert Commissioning.ps1\r\n "),
		"installer/commissioning-v1/Preparation.ps1":                 []byte("\xef\xbb\xbf# inert Preparation.ps1\r\n "),
		"installer/commissioning-v1/Common.ps1":                      []byte("\xef\xbb\xbf# inert Common.ps1\r\n "),
		"installer/commissioning-v1/CommissioningProfileLock.cs":     []byte("\xef\xbb\xbf# inert CommissioningProfileLock.cs\r\n "),
		"installer/commissioning-v1/InstalledResourceReview.ps1":     []byte("\xef\xbb\xbf# inert InstalledResourceReview.ps1\r\n "),
		"installer/commissioning-v1/CommissioningBaseline.ps1":       []byte("\xef\xbb\xbf# inert CommissioningBaseline.ps1\r\n "),
		"installer/commissioning-v1/ColdBaseline.ps1":                []byte("\xef\xbb\xbf# inert ColdBaseline.ps1\r\n "),
		"installer/commissioning-v1/RestartCommissioningPolicy.ps1":  []byte("\xef\xbb\xbf# inert RestartCommissioningPolicy.ps1\r\n "),
		"installer/commissioning-v1/RestartAuiPersistence.ps1":       []byte("\xef\xbb\xbf# inert RestartAuiPersistence.ps1\r\n "),
		"installer/commissioning-v1/RestartDashboardPersistence.ps1": []byte("\xef\xbb\xbf# inert RestartDashboardPersistence.ps1\r\n "),
	}
}

func TestStageRecoveryRetainsOriginalCommissioningBeforeCompletion(t *testing.T) {
	f, p := prepareStageFixture(t, true, true)
	sink := &recoveryFileSink{root: t.TempDir()}
	identity, err := p.StageRecovery(context.Background(), sink)
	if err != nil || identity != p.RecoveryIdentity || sink.identity != identity || len(sink.evidenceNames) != 17 {
		t.Fatal("commissioning originals missing before completion", err)
	}
	for name, raw := range recoveryCommissioningFixture() {
		stored, err := os.ReadFile(filepath.Join(sink.root, filepath.FromSlash(name)))
		if err != nil || string(stored) != string(raw) || string(stored) != string(f.entries[name]) {
			t.Fatal("commissioning original changed", name, err)
		}
	}
	for _, name := range sink.names {
		if baselineevidence.IsCommissioningHelper(name) {
			t.Fatal("commissioning helper installed as app/docs")
		}
	}
}

func TestStageRecoveryCommissioningSinkFailureNeverCompletes(t *testing.T) {
	for _, scenario := range []string{"refuse", "cancel"} {
		t.Run(scenario, func(t *testing.T) {
			_, p := prepareStageFixture(t, true, true)
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			complete := false
			sink := recoveryEvidenceFaultSink{
				recoverySinkFunc: func(_ context.Context, _ string, r io.Reader, _ int64, _ string) error {
					_, err := io.Copy(io.Discard, r)
					return err
				},
				writeEvidence: func(_ context.Context, name string, _ []byte, _ string) error {
					if baselineevidence.IsCommissioningHelper(name) {
						if scenario == "cancel" {
							cancel()
						}
						return errors.New("commissioning destination refused")
					}
					return nil
				},
				complete: func(context.Context, baselineevidence.Identity) error { complete = true; return nil },
			}
			identity, err := p.StageRecovery(ctx, sink)
			if err == nil || identity != (RecoveryIdentity{}) || complete {
				t.Fatal("failed commissioning write completed")
			}
			if scenario == "cancel" && !errors.Is(err, context.Canceled) {
				t.Fatal("cancellation lost", err)
			}
		})
	}
}
