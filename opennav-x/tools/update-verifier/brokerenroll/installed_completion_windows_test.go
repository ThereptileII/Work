//go:build windows

package brokerenroll

import (
	"bytes"
	"context"
	"errors"
	"os"
	"path/filepath"
	"testing"

	"example.com/opennav-update-verifier/brokerregistration"
)

func installedCompletionFixtureGuard(ctx context.Context) error { return ctx.Err() }

// Inert disposable files exercise original native handles, never a real
// enrollment, service, executable, menu, approved ZIP or user-data mutation.
func installedCompletionNativeFixture(t *testing.T) (*nativeStartupPublication, *nativeEnrollmentFiles, *brokerregistration.Request, *nativeClientBaseline) {
	t.Helper()
	s, menu, recovery, request := startupCompletedBindingFixture(t)
	ctx := context.Background()
	if err := completeStartupPublicationBinding(ctx, s, menu, recovery, request, recovery.descriptor); err != nil {
		t.Fatal(err)
	}
	f := &nativeEnrollmentFiles{}
	t.Cleanup(func() {
		if err := f.Close(); err != nil {
			t.Error(err)
		}
	})
	root := t.TempDir()
	parent, err := f.pinDirectory(root, "", root, nil, false, s.clientSID)
	if err != nil {
		t.Fatal(err)
	}
	broker, err := enrollmentBrokerSID()
	if err != nil {
		t.Fatal(err)
	}
	dirSD, err := clientRequestDescriptor(broker.String(), true)
	if err != nil {
		t.Fatal(err)
	}
	fileSD, err := clientRequestDescriptor(broker.String(), false)
	if err != nil {
		t.Fatal(err)
	}
	f.clientParent, err = f.newDirectory(parent, clientRequestsDirectory, dirSD, broker.String())
	if err != nil {
		t.Fatal(err)
	}
	f.clientAttempted = true
	f.clientBaselineReady = true
	f.clientSID = request.ClientSID
	f.clientSHA = request.RequestSHA256
	if _, err = startupWriteFile(ctx, f, f.clientParent, request.ClientSID+".json", request.OriginalBytes(), fileSD, false); err != nil {
		t.Fatal(err)
	}
	body := []byte("INERT original already-authenticated baseline source fixture")
	if _, err = startupWriteFile(ctx, f, f.clientParent, request.ClientSID+".baseline.zip", body, fileSD, false); err != nil {
		t.Fatal(err)
	}
	image, err := startupWriteFile(ctx, f, parent, enrollmentImageName, []byte("INERT-BROKER"), fileSD, false)
	if err != nil {
		t.Fatal(err)
	}
	f.imagePath = image.dos
	f.maintenanceHelper, err = startupWriteFile(ctx, f, parent, maintenanceImageName, []byte("INERT-HELPER"), fileSD, false)
	if err != nil {
		t.Fatal(err)
	}
	f.maintenanceHelperSHA = digest([]byte("INERT-HELPER"))
	input := &nativeEnrollmentFiles{}
	inputRoot := t.TempDir()
	inputParent, err := input.pinDirectory(inputRoot, "", inputRoot, nil, false, s.clientSID)
	if err != nil {
		t.Fatal(err)
	}
	input.bootstrapCache = inputParent
	f.bootstrapCache, f.bootstrapCacheOwner = inputParent, input
	inputPin, err := startupWriteFile(ctx, input, inputParent, clientBaselineInputName, body, fileSD, false)
	if err != nil {
		t.Fatal(err)
	}
	source := &nativeClientBaseline{ctx: ctx, files: input, pin: inputPin, size: int64(len(body))}
	t.Cleanup(func() {
		if err := source.Close(); err != nil {
			t.Error(err)
		}
	})
	return s, f, request, source
}

func prepareInstalledCompletionFixture(t *testing.T, s *nativeStartupPublication, f *nativeEnrollmentFiles, request *brokerregistration.Request, source *nativeClientBaseline) *nativeInstalledCompletion {
	t.Helper()
	owner, err := s.prepareInstalledCompletion(context.Background(), f, request, source)
	if owner != nil {
		t.Cleanup(func() {
			if err := owner.Close(); err != nil {
				t.Error(err)
			}
		})
	}
	if err != nil {
		t.Fatal(err)
	}
	return owner.(*nativeInstalledCompletion)
}

func TestNativeInstalledCompletionStartupCommitPreservesOriginalCustody(t *testing.T) {
	s, f, request, source := installedCompletionNativeFixture(t)
	n := prepareInstalledCompletionFixture(t, s, f, request, source)
	markerPath := filepath.Join(f.clientParent.dos, request.ClientSID+".maintenance.once")
	raw, err := os.ReadFile(markerPath)
	if err != nil {
		t.Fatal(err)
	}
	if validateInstalledCompletionMarker(raw, digest(n.raw)) == nil {
		t.Fatal("reservation alone became maintenance authority")
	}
	if err = n.Complete(context.Background(), installedCompletionFixtureGuard); err != nil {
		t.Fatal(err)
	}
	raw, err = os.ReadFile(markerPath)
	if err != nil || validateInstalledCompletionMarker(raw, digest(n.raw)) != nil {
		t.Fatalf("completed marker missing: %v", err)
	}
	recordRaw, err := os.ReadFile(filepath.Join(f.clientParent.dos, request.ClientSID+".maintenance.json"))
	if err != nil {
		t.Fatal(err)
	}
	r, err := parseInstalledCompletion(recordRaw)
	if err != nil || r.Schema != 2 || r.BootstrapCache == nil || r.BootstrapCache.Path != f.bootstrapCache.dos || !bytes.Equal(r.ClientRequest, request.OriginalBytes()) || r.MaintenanceHelper.SHA256 != f.maintenanceHelperSHA || r.Request.FileID == r.Baseline.FileID {
		t.Fatalf("original completion lost identity: %v", err)
	}
	if err = n.Complete(context.Background(), installedCompletionFixtureGuard); err == nil {
		t.Fatal("completion repeated")
	}
	if err = n.Close(); err != nil {
		t.Fatal(err)
	}
	// After producer close, the fixed protected original is durable and readable.
	raw, err = os.ReadFile(markerPath)
	if err != nil || validateInstalledCompletionMarker(raw, digest(recordRaw)) != nil {
		t.Fatalf("completion did not survive owner drain: %v", err)
	}
}

func TestNativeInstalledCompletionStartupInterruptionRemainsSticky(t *testing.T) {
	s, f, request, source := installedCompletionNativeFixture(t)
	n := prepareInstalledCompletionFixture(t, s, f, request, source)
	ctx, cancel := context.WithCancel(context.Background())
	cancel()
	if err := n.Complete(ctx, installedCompletionFixtureGuard); err == nil {
		t.Fatal("cancelled completion committed")
	}
	if err := n.Complete(context.Background(), installedCompletionFixtureGuard); err == nil {
		t.Fatal("cancelled original owner retried")
	}
	if _, err := os.Stat(filepath.Join(f.clientParent.dos, request.ClientSID+".maintenance.json")); !errors.Is(err, os.ErrNotExist) {
		t.Fatalf("cancelled attempt created completion: %v", err)
	}
	if err := n.Close(); err != nil {
		t.Fatal(err)
	}
	other, err := s.prepareInstalledCompletion(context.Background(), f, request, source)
	if other != nil {
		_ = other.Close()
	}
	if err == nil {
		t.Fatal("permanent original reservation adopted after interruption")
	}
}

func TestNativeInstalledCompletionStartupFinalSCMGuardRefusalPreservesPartial(t *testing.T) {
	s, f, request, source := installedCompletionNativeFixture(t)
	n := prepareInstalledCompletionFixture(t, s, f, request, source)
	guards := 0
	guard := func(ctx context.Context) error {
		guards++
		// Exercise the actual startup guard's reentry into the original file
		// owner. The producer must not retain files.mu across this callback.
		if err := f.Check(); err != nil {
			return err
		}
		if guards == 2 {
			return errors.New("original service exited after completion JSON synced")
		}
		return ctx.Err()
	}
	if err := n.Complete(context.Background(), guard); err == nil || guards != 2 {
		t.Fatalf("final machine refusal bypassed err=%v guards=%d", err, guards)
	}
	recordRaw, err := os.ReadFile(filepath.Join(f.clientParent.dos, request.ClientSID+".maintenance.json"))
	if err != nil {
		t.Fatal(err)
	}
	if _, err = parseInstalledCompletion(recordRaw); err != nil {
		t.Fatal("original partial record was not preserved", err)
	}
	marker, err := os.ReadFile(filepath.Join(f.clientParent.dos, request.ClientSID+".maintenance.once"))
	if err != nil || validateInstalledCompletionMarker(marker, digest(recordRaw)) == nil {
		t.Fatalf("partial record became completion authority: %v", err)
	}
	if err = n.Complete(context.Background(), installedCompletionFixtureGuard); err == nil {
		t.Fatal("failed final guard retried original completion")
	}
}

func TestNativeInstalledCompletionStartupExistingCompletionNeverAdopted(t *testing.T) {
	for _, beforeReserve := range []bool{true, false} {
		t.Run(map[bool]string{true: "before-reservation", false: "after-reservation"}[beforeReserve], func(t *testing.T) {
			s, f, request, source := installedCompletionNativeFixture(t)
			var n *nativeInstalledCompletion
			if !beforeReserve {
				n = prepareInstalledCompletionFixture(t, s, f, request, source)
			}
			path := filepath.Join(f.clientParent.dos, request.ClientSID+".maintenance.json")
			body := []byte("UNKNOWN historical completion must survive")
			if err := os.WriteFile(path, body, 0600); err != nil {
				t.Fatal(err)
			}
			if beforeReserve {
				other, err := s.prepareInstalledCompletion(context.Background(), f, request, source)
				if other != nil {
					_ = other.Close()
				}
				if err == nil {
					t.Fatal("historical completion adopted")
				}
			} else if err := n.Complete(context.Background(), installedCompletionFixtureGuard); err == nil {
				t.Fatal("post-reservation collision overwritten")
			}
			got, err := os.ReadFile(path)
			if err != nil || !bytes.Equal(got, body) {
				t.Fatalf("historical bytes changed %v", err)
			}
		})
	}
}

func TestNativeInstalledCompletionStartupOriginalCapabilityMismatchRefuses(t *testing.T) {
	for _, failure := range []string{"request", "helper", "publication", "baseline", "foreign-files", "copied-owner"} {
		t.Run(failure, func(t *testing.T) {
			s, f, request, source := installedCompletionNativeFixture(t)
			switch failure {
			case "request":
				f.clientSHA = digest([]byte("foreign request"))
			case "helper":
				f.maintenanceHelperSHA = digest([]byte("foreign helper"))
			case "publication":
				s.bindingSHA = digest([]byte("foreign publication"))
			case "baseline":
				source.size--
			case "foreign-files":
				s.files = f
			case "copied-owner":
				forged := &nativeInstalledCompletion{self: &nativeInstalledCompletion{}}
				if err := forged.Complete(context.Background(), installedCompletionFixtureGuard); err == nil {
					t.Fatal("copied owner completed")
				}
				return
			}
			owner, err := s.prepareInstalledCompletion(context.Background(), f, request, source)
			if owner != nil {
				_ = owner.Close()
			}
			if err == nil {
				t.Fatal("foreign original capability accepted")
			}
			if _, err := os.Stat(filepath.Join(f.clientParent.dos, request.ClientSID+".maintenance.once")); !errors.Is(err, os.ErrNotExist) {
				t.Fatalf("foreign capability consumed reservation %v", err)
			}
		})
	}
}
