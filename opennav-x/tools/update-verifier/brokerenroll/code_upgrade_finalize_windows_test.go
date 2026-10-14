//go:build windows

package brokerenroll

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"os"
	"path/filepath"
	"strings"
	"testing"

	"example.com/opennav-update-verifier/brokerpipe"
	"example.com/opennav-update-verifier/brokerregistration"
	"example.com/opennav-update-verifier/releasepolicy"
	"golang.org/x/sys/windows"
)

// These fixtures use disposable NTFS files and read-only observations of this
// test process. Service state and the authenticated IPC result are inert seams;
// no SCM service, installed namespace, application or real profile is changed.
type nativeFinalizeProcessFixture struct {
	original *nativeInstalledMaintenanceAuthority
	image    *enrollmentPin
	identity maintenanceTokenIdentity
}

func (f *nativeFinalizeProcessFixture) check(ctx context.Context) error { return f.original.check(ctx) }
func (f *nativeFinalizeProcessFixture) status() (serviceState, error) {
	return serviceState{state: 4, processID: windows.GetCurrentProcessId(), controls: 5}, nil
}
func (f *nativeFinalizeProcessFixture) bindProcess(pid uint32) (maintenanceProcess, error) {
	return openMaintenanceProcess(pid, f.image, f.identity.sid, f.identity.session)
}
func (f *nativeFinalizeProcessFixture) close() error { return nil }

type nativeFinalizeFixture struct {
	publication *nativeCodeFinalizePublication
	backend     *nativeFinalizeProcessFixture
	originals   []*enrollmentPin
	bindings    []startupFileBinding
}

func nativeFinalizationFixture(t *testing.T) *nativeFinalizeFixture {
	t.Helper()
	owner, _, parent, sd := startupNativeFixture(t)
	ctx := context.Background()
	c := &nativeInstalledMaintenanceAuthority{files: owner.files, clientSID: owner.clientSID, requestParent: parent}
	p := &nativeCodeFinalizePublication{original: c, files: &nativeEnrollmentFiles{}, parent: parent, name: owner.clientSID + codeSuccessorMarkerSuffix, descriptor: sd, raw: codeSuccessorMarker(strings.Repeat("a", 64))}
	t.Cleanup(func() {
		if err := p.close(); err != nil {
			t.Error(err)
		}
	})
	f := &nativeFinalizeFixture{publication: p}
	for _, name := range []string{"fixture-original-completion.json", "fixture-original-startup.json", "fixture-profile.dat"} {
		body := []byte("INERT-UNCHANGED-" + name)
		pin, err := startupWriteFile(ctx, c.files, parent, name, body, sd, false)
		if err != nil {
			t.Fatal(err)
		}
		binding, err := startupFileIdentity(ctx, pin, digest(body))
		if err != nil {
			t.Fatal(err)
		}
		f.originals = append(f.originals, pin)
		f.bindings = append(f.bindings, binding)
	}
	identity, err := maintenanceToken(windows.GetCurrentProcessToken())
	if err != nil {
		t.Fatal(err)
	}
	path, err := os.Executable()
	if err != nil {
		t.Fatal(err)
	}
	pointer, err := windows.UTF16PtrFromString(path)
	if err != nil {
		t.Fatal(err)
	}
	h, err := windows.CreateFile(pointer, windows.GENERIC_READ|windows.READ_CONTROL, windows.FILE_SHARE_READ, nil, windows.OPEN_EXISTING, windows.FILE_FLAG_OPEN_REPARSE_POINT, 0)
	if err != nil {
		t.Fatal(err)
	}
	pin := &enrollmentPin{handle: h, file: true, broker: identity.sid, dos: path}
	c.files.pins = append(c.files.pins, pin)
	pin.info, pin.id, err = enrollmentInfo(h)
	if err != nil {
		t.Fatal(err)
	}
	pin.length = int64(uint64(pin.info.FileSizeHigh)<<32 | uint64(pin.info.FileSizeLow))
	pin.physical, err = enrollmentPhysicalPath(h)
	if err != nil {
		t.Fatal(err)
	}
	pin.descriptor, err = windows.GetSecurityInfo(h, windows.SE_FILE_OBJECT, windows.OWNER_SECURITY_INFORMATION|windows.DACL_SECURITY_INFORMATION)
	if err != nil {
		t.Fatal(err)
	}
	f.backend = &nativeFinalizeProcessFixture{original: c, image: pin, identity: identity}
	return f
}
func (f *nativeFinalizeFixture) checkOriginals(t *testing.T) {
	t.Helper()
	for i, pin := range f.originals {
		if err := installedMatchFile(context.Background(), pin, f.bindings[i]); err != nil {
			t.Fatal("original completion/startup/profile fixture changed", err)
		}
	}
}
func TestNativeCodeUpgradeFinalizeReadyPublicationRetainsProcessAndOriginals(t *testing.T) {
	f := nativeFinalizationFixture(t)
	queries := 0
	owner, err := openCodeFinalizerWith(context.Background(), f.backend, f.publication, func(_ context.Context, q brokerpipe.MaintenanceRequest) (brokerpipe.MaintenanceResult, error) {
		queries++
		return finalizeResponse(q), nil
	})
	if err != nil {
		t.Fatal(err)
	}
	if err = owner.resume(context.Background()); err != nil {
		t.Fatal(err)
	}
	raw, err := installedReadPin(context.Background(), f.publication.pin, maxInstalledCompletionMarkerBytes)
	if err != nil || !bytes.Equal(raw, f.publication.raw) {
		t.Fatal("exact ready marker was not flushed/read back", err)
	}
	before, err := startupFileIdentity(context.Background(), f.publication.pin, digest(raw))
	if err != nil {
		t.Fatal(err)
	}
	if err = owner.resume(context.Background()); err == nil || queries != 1 {
		t.Fatal("ready finalization replayed")
	}
	if err = installedMatchFile(context.Background(), f.publication.pin, before); err != nil {
		t.Fatal(err)
	}
	f.checkOriginals(t)
	if err = owner.close(); err != nil {
		t.Fatal(err)
	}
}
func TestNativeCodeUpgradeFinalizeRefusesEveryExistingReadyNamespace(t *testing.T) {
	for _, kind := range []string{"empty", "malformed", "complete", "directory"} {
		t.Run(kind, func(t *testing.T) {
			f := nativeFinalizationFixture(t)
			path := filepath.Join(f.publication.parent.dos, f.publication.name)
			body := []byte("MALFORMED")
			if kind == "empty" {
				body = nil
			}
			if kind == "complete" {
				body = bytes.Clone(f.publication.raw)
			}
			var err error
			if kind == "directory" {
				err = os.Mkdir(path, 0700)
			} else {
				err = os.WriteFile(path, body, 0600)
			}
			if err != nil {
				t.Fatal(err)
			}
			before, err := os.Lstat(path)
			if err != nil {
				t.Fatal(err)
			}
			if n, err := openCodeFinalizerWith(context.Background(), f.backend, f.publication, func(context.Context, brokerpipe.MaintenanceRequest) (brokerpipe.MaintenanceResult, error) {
				t.Fatal("existing ready namespace reached readiness query")
				return brokerpipe.MaintenanceResult{}, nil
			}); err == nil || n != nil {
				t.Fatal("existing ready namespace adopted")
			}
			after, err := os.Lstat(path)
			if err != nil || !os.SameFile(before, after) || before.Mode() != after.Mode() || before.Size() != after.Size() {
				t.Fatal("existing ready identity changed", err)
			}
			if kind != "directory" {
				actual, err := os.ReadFile(path)
				if err != nil || !bytes.Equal(actual, body) {
					t.Fatal("existing ready bytes changed", err)
				}
			}
		})
	}
}

type nativeFinalizeGuardFixture struct {
	*nativeCodeFinalizePublication
	beforeGuard func(int)
	calls       int
}

func (p *nativeFinalizeGuardFixture) publish(ctx context.Context, guard func(context.Context) error) error {
	return p.nativeCodeFinalizePublication.publish(ctx, func(ctx context.Context) error {
		p.calls++
		if p.beforeGuard != nil {
			p.beforeGuard(p.calls)
		}
		return guard(ctx)
	})
}
func TestNativeCodeUpgradeFinalizeRechecksProcessBeforeAndAfterCreate(t *testing.T) {
	for _, phase := range []string{"before-create", "after-readback"} {
		t.Run(phase, func(t *testing.T) {
			f := nativeFinalizationFixture(t)
			wrapped := &nativeFinalizeGuardFixture{nativeCodeFinalizePublication: f.publication}
			owner, err := openCodeFinalizerWith(context.Background(), f.backend, wrapped, func(_ context.Context, q brokerpipe.MaintenanceRequest) (brokerpipe.MaintenanceResult, error) {
				return finalizeResponse(q), nil
			})
			if err != nil {
				t.Fatal(err)
			}
			wrapped.beforeGuard = func(call int) {
				if phase == "before-create" && call == 2 || phase == "after-readback" && call == 3 {
					owner.process.(*nativeMaintenanceProcess).created.LowDateTime ^= 1
				}
			}
			err = owner.resume(context.Background())
			if err == nil {
				t.Fatal("revoked original native process authorized completion")
			}
			path := filepath.Join(f.publication.parent.dos, f.publication.name)
			if phase == "before-create" {
				if _, e := os.Lstat(path); !errors.Is(e, os.ErrNotExist) {
					t.Fatal("revoked process created ready marker", e)
				}
			} else {
				if !errors.Is(err, ErrCodeUpgradeUncertain) {
					t.Fatal("post-publication uncertainty lost", err)
				}
				raw, e := os.ReadFile(path)
				if e != nil || !bytes.Equal(raw, f.publication.raw) {
					t.Fatal("owned post-effect evidence not retained", e)
				}
			}
			if e := owner.resume(context.Background()); e == nil {
				t.Fatal("uncertain publication retried")
			}
			f.checkOriginals(t)
			if e := owner.close(); e != nil {
				t.Fatal(e)
			}
		})
	}
}

// A full original/successor record fixture for the same native closure used by
// the authenticated opener. It does not install a service or grant authority.
func nativeFinalizePairFixture(t *testing.T, fault string) (*nativeInstalledMaintenanceAuthority, releasepolicy.BrokerCodeUpgradePolicy, []byte, *windows.SECURITY_DESCRIPTOR) {
	t.Helper()
	n := codePairNativeFixture(t)
	original := installedCompletionRecordFixture(t)
	original.Broker, original.MaintenanceHelper = n.oldBrokerID, n.oldHelperID
	originalRaw, err := marshalInstalledCompletion(original)
	if err != nil {
		t.Fatal(err)
	}
	client, err := brokerregistration.ParseRetainedApprovedRequest(original.ClientRequest, original.ClientRequestSHA256)
	if err != nil {
		t.Fatal(err)
	}
	n.policy.Bootstrap.TrustConfigSHA256 = client.TrustConfigSHA256
	policyRaw, err := releasepolicy.MarshalBrokerCodeUpgrade(n.policy)
	if err != nil {
		t.Fatal(err)
	}
	parent, err := startupDirectoryIdentity(n.parent)
	if err != nil {
		t.Fatal(err)
	}
	r := codeUpgradeRecord{Schema: 1, Purpose: codeUpgradeRecordPurpose, OriginalCompletionSHA256: digest(originalRaw), ClientRequestSHA256: original.ClientRequestSHA256, Policy: policyRaw, PolicySHA256: digest(policyRaw), CodeParent: parent, PreviousBroker: n.oldBrokerID, PreviousHelper: n.oldHelperID, Broker: n.nextBrokerID, Helper: n.nextHelperID}
	reservedRaw, err := marshalCodeUpgradeRecord(r)
	if err != nil {
		t.Fatal(err)
	}
	intent := r
	intent.Broker = startupFileBinding{}
	intent.Helper = startupFileBinding{}
	intentRaw, _ := json.Marshal(intent)
	drainRaw, _ := json.Marshal(codeDrainRecord{Schema: 1, ReservationSHA256: digest(reservedRaw), PID: 1, Birth: 1})
	n.movePrefix(t, 4)
	r.ReservationSHA256 = digest(reservedRaw)
	for _, item := range []struct {
		pin *enrollmentPin
		sha string
		out *startupFileBinding
	}{{n.oldBroker.pin, n.policy.PreviousBrokerSHA256, &r.PreviousBroker}, {n.oldHelper.pin, n.policy.PreviousHelperSHA256, &r.PreviousHelper}, {n.nextBroker.pin, n.policy.Bootstrap.Broker.SHA256, &r.Broker}, {n.nextHelper.pin, n.policy.Bootstrap.Provisioner.SHA256, &r.Helper}} {
		*item.out, err = startupFileIdentity(context.Background(), item.pin, item.sha)
		if err != nil {
			t.Fatal(err)
		}
	}
	successorRaw, err := marshalCodeUpgradeRecord(r)
	if err != nil {
		t.Fatal(err)
	}
	if err = n.files.Close(); err != nil {
		t.Fatal(err)
	}
	n.files = &nativeEnrollmentFiles{}
	for _, item := range []struct {
		suffix string
		raw    []byte
	}{{codeIntentSuffix, intentRaw}, {codeReservationSuffix, reservedRaw}, {codeDrainSuffix, drainRaw}, {codeSuccessorSuffix, successorRaw}, {codeSuccessorPairSuffix, codeSuccessorMarker(digest(successorRaw))}} {
		if fault == "missing-drain" && item.suffix == codeDrainSuffix || fault == "missing-pair" && item.suffix == codeSuccessorPairSuffix {
			continue
		}
		if fault == "partial-pair" && item.suffix == codeSuccessorPairSuffix {
			item.raw = []byte("PARTIAL")
		}
		if _, err = startupWriteFile(context.Background(), n.files, n.parent, original.ClientSID+item.suffix, item.raw, n.sd, false); err != nil {
			t.Fatal(err)
		}
	}
	if fault == "existing-ready" {
		if _, err = startupWriteFile(context.Background(), n.files, n.parent, original.ClientSID+codeSuccessorMarkerSuffix, []byte("PARTIAL"), n.sd, false); err != nil {
			t.Fatal(err)
		}
	}
	if fault == "foreign-fixed-helper" {
		if err = os.Remove(filepath.Join(n.parent.dos, maintenanceImageName)); err != nil {
			t.Fatal(err)
		}
		if err = os.WriteFile(filepath.Join(n.parent.dos, maintenanceImageName), []byte("FOREIGN"), 0600); err != nil {
			t.Fatal(err)
		}
	}
	c := &nativeInstalledMaintenanceAuthority{files: &nativeEnrollmentFiles{}, clientSID: original.ClientSID, record: original, originalRaw: originalRaw, codeParent: n.parent, requestParent: n.parent}
	t.Cleanup(func() {
		if err := c.files.Close(); err != nil {
			t.Error(err)
		}
	})
	return c, n.policy, policyRaw, n.sd
}
func TestNativeCodeUpgradeFinalizeRequiresExactPendingPairClosure(t *testing.T) {
	for _, fault := range []string{"valid", "missing-drain", "missing-pair", "partial-pair", "existing-ready", "foreign-fixed-helper", "wrong-approved-policy"} {
		t.Run(fault, func(t *testing.T) {
			c, p, raw, sd := nativeFinalizePairFixture(t, fault)
			if fault == "wrong-approved-policy" {
				raw = append(bytes.Clone(raw), ' ')
			}
			originalRaw := bytes.Clone(c.originalRaw)
			originalRecordRaw, err := marshalInstalledCompletion(c.record)
			if err != nil {
				t.Fatal(err)
			}
			image, ready, err := openCodeFinalizePairNative(context.Background(), c, p, raw, sd, sd)
			if fault != "valid" {
				if err == nil {
					t.Fatal("incomplete/foreign tuple accepted for finalization")
				}
				return
			}
			if err != nil || image == nil || len(ready) == 0 {
				t.Fatal("exact pending pair refused", err)
			}
			if _, err = readCodeSuccessorNative(context.Background(), c, c.record, c.originalRaw, sd, sd); err == nil {
				t.Fatal("public installed reader accepted missing-ready state")
			}
			receipt, err := maintenanceHoldFile(c.files, c.requestParent, c.clientSID+codeSuccessorSuffix, sd, maxCodeUpgradeRecordBytes)
			if err != nil {
				t.Fatal(err)
			}
			receiptRaw, err := installedReadPin(context.Background(), receipt, maxCodeUpgradeRecordBytes)
			if err != nil {
				t.Fatal(err)
			}
			successor, err := parseCodeUpgradeRecord(receiptRaw)
			if err != nil {
				t.Fatal(err)
			}
			if err = errors.Join(installedMatchFile(context.Background(), image, successor.Broker), installedMatchFile(context.Background(), c.helperImage, successor.Helper)); err != nil {
				t.Fatal(err)
			}
			if !bytes.Equal(ready, codeSuccessorMarker(digest(receiptRaw))) {
				t.Fatal("ready marker lost exact successor receipt binding")
			}
			afterRecordRaw, err := marshalInstalledCompletion(c.record)
			if err != nil || !bytes.Equal(originalRecordRaw, afterRecordRaw) || !bytes.Equal(originalRaw, c.originalRaw) {
				t.Fatal("original completion/request/publication/baseline changed", err)
			}
		})
	}
}
