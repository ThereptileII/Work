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
)

type recoverySinkFunc func(context.Context, string, io.Reader, int64, string) error

func (f recoverySinkFunc) WriteFile(ctx context.Context, name string, r io.Reader, size int64, digest string) error {
	return f(ctx, name, r, size, digest)
}

// Disposable file IO only: this fixture makes no protected-root custody claim.
type recoveryFileSink struct {
	root  string
	names []string
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

func prepareStageFixture(t *testing.T) (*recoveryFixture, *PreparedUpdateSet) {
	t.Helper()
	f := newRecoveryFixture(t, true)
	offer := f.check(t)
	p, err := prepareUpdateSet(context.Background(), f.config, f.observed, offer.Consent, f.client)
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { p.Close() })
	return f, p
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
