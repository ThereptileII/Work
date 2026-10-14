//go:build windows

package brokerenroll

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"example.com/opennav-update-verifier/brokerregistration"
	"os"
	"path/filepath"
	"testing"

	"example.com/opennav-update-verifier/releasepolicy"
	"golang.org/x/sys/windows"
)

// All fixtures are inert disposable NTFS files. No SCM, installed namespace,
// executable process, real profile, trust store or selected history is mutated.
type nativeCodePairFixture struct {
	parent                                               *enrollmentPin
	files                                                *nativeEnrollmentFiles
	oldBroker, oldHelper, nextBroker, nextHelper         startupRestorationFile
	oldBrokerID, oldHelperID, nextBrokerID, nextHelperID startupFileBinding
	sd                                                   *windows.SECURITY_DESCRIPTOR
	policy                                               releasepolicy.BrokerCodeUpgradePolicy
}

func codePairNativeFixture(t *testing.T) *nativeCodePairFixture {
	t.Helper()
	owner, _, parent, sd := startupNativeFixture(t)
	ctx := context.Background()
	n := &nativeCodePairFixture{parent: parent, files: &nativeEnrollmentFiles{}, sd: sd}
	t.Cleanup(func() {
		if e := n.files.Close(); e != nil {
			t.Error(e)
		}
	})
	f := newEnrollmentFixture(t)
	n.policy = releasepolicy.BrokerCodeUpgradePolicy{Schema: 1, Purpose: releasepolicy.BrokerCodeUpgradePurpose, PreviousBrokerSHA256: digest([]byte("OLD-BROKER")), PreviousHelperSHA256: digest([]byte("OLD-HELPER")), Bootstrap: f.policy}
	n.policy.Bootstrap.Broker.SHA256 = digest([]byte("NEW-BROKER"))
	n.policy.Bootstrap.Broker.Bytes = 10
	n.policy.Bootstrap.Provisioner.SHA256 = digest([]byte("NEW-HELPER"))
	n.policy.Bootstrap.Provisioner.Bytes = 10
	for _, item := range []struct {
		name     string
		body     []byte
		target   *startupRestorationFile
		identity *startupFileBinding
	}{{enrollmentImageName, []byte("OLD-BROKER"), &n.oldBroker, &n.oldBrokerID}, {maintenanceImageName, []byte("OLD-HELPER"), &n.oldHelper, &n.oldHelperID}, {stagedBrokerName, []byte("NEW-BROKER"), &n.nextBroker, &n.nextBrokerID}, {stagedHelperName, []byte("NEW-HELPER"), &n.nextHelper, &n.nextHelperID}} {
		pin, e := startupWriteFile(ctx, owner.files, parent, item.name, item.body, sd, false)
		if e != nil {
			t.Fatal(e)
		}
		*item.identity, e = startupFileIdentity(ctx, pin, digest(item.body))
		if e != nil {
			t.Fatal(e)
		}
		if e = codeReleasePin(owner.files, pin); e != nil {
			t.Fatal(e)
		}
		*item.target, e = openCodeMoveFile(n.files, parent, item.name, sd)
		if e != nil {
			t.Fatal(e)
		}
	}
	return n
}
func (n *nativeCodePairFixture) movePrefix(t *testing.T, count int) {
	t.Helper()
	ctx := context.Background()
	moves := []struct {
		f         *startupRestorationFile
		name, sha string
	}{{&n.oldBroker, previousBrokerName, n.policy.PreviousBrokerSHA256}, {&n.oldHelper, previousHelperName, n.policy.PreviousHelperSHA256}, {&n.nextBroker, enrollmentImageName, n.policy.Bootstrap.Broker.SHA256}, {&n.nextHelper, maintenanceImageName, n.policy.Bootstrap.Provisioner.SHA256}}
	for _, m := range moves[:count] {
		if e := startupRenameRestorationFile(ctx, m.f, n.parent, m.name, m.sha); e != nil {
			t.Fatal(e)
		}
	}
}
func TestNativeCodeUpgradeResumeEveryRenameBoundary(t *testing.T) {
	for count := 0; count <= 4; count++ {
		t.Run(string(rune('0'+count)), func(t *testing.T) {
			n := codePairNativeFixture(t)
			n.movePrefix(t, count)
			// Lose original mutation owners exactly as an interrupted provisioner would.
			// The real entry additionally requires protected exact reservation/drain,
			// same independently approved policy and original disabled/stopped SCM.
			if e := n.files.Close(); e != nil {
				t.Fatal(e)
			}
			n.files = &nativeEnrollmentFiles{}
			var e error
			n.oldBroker, n.nextBroker, e = readReservedCodePositions(context.Background(), n.files, n.parent, enrollmentImageName, previousBrokerName, stagedBrokerName, n.oldBrokerID, n.nextBrokerID, n.sd)
			if e != nil {
				t.Fatal(e)
			}
			n.oldHelper, n.nextHelper, e = readReservedCodePositions(context.Background(), n.files, n.parent, maintenanceImageName, previousHelperName, stagedHelperName, n.oldHelperID, n.nextHelperID, n.sd)
			if e != nil {
				t.Fatal(e)
			}
			if e = resumeOwnedCodePair(context.Background(), func(ctx context.Context) error { return errors.Join(ctx.Err(), n.parent.check(), n.files.Check()) }, n.parent, &n.oldBroker, &n.oldHelper, &n.nextBroker, &n.nextHelper, n.policy); e != nil {
				t.Fatal(e)
			}
			for _, check := range []struct {
				f         startupRestorationFile
				name, sha string
				id        startupFileBinding
			}{{n.oldBroker, previousBrokerName, n.policy.PreviousBrokerSHA256, n.oldBrokerID}, {n.oldHelper, previousHelperName, n.policy.PreviousHelperSHA256, n.oldHelperID}, {n.nextBroker, enrollmentImageName, n.policy.Bootstrap.Broker.SHA256, n.nextBrokerID}, {n.nextHelper, maintenanceImageName, n.policy.Bootstrap.Provisioner.SHA256, n.nextHelperID}} {
				got, e := startupFileIdentity(context.Background(), check.f.pin, check.sha)
				if e != nil || !sameCodeFile(got, check.id) || check.f.dos != filepath.Join(n.parent.dos, check.name) {
					t.Fatal("recovery lost full native code identity", e)
				}
			}
		})
	}
}
func TestNativeCodeUpgradeRestoresSameOwnedOriginalPair(t *testing.T) {
	for count := 0; count <= 4; count++ {
		t.Run(string(rune('0'+count)), func(t *testing.T) {
			n := codePairNativeFixture(t)
			n.movePrefix(t, count)
			if e := restoreOwnedCodePair(context.Background(), n.parent, &n.oldBroker, &n.oldHelper, &n.nextBroker, &n.nextHelper, n.policy); e != nil {
				t.Fatal(e)
			}
			for _, item := range []struct {
				f    startupRestorationFile
				name string
				id   startupFileBinding
			}{{n.oldBroker, enrollmentImageName, n.oldBrokerID}, {n.oldHelper, maintenanceImageName, n.oldHelperID}, {n.nextBroker, stagedBrokerName, n.nextBrokerID}, {n.nextHelper, stagedHelperName, n.nextHelperID}} {
				if item.f.dos != filepath.Join(n.parent.dos, item.name) || item.f.pin.id.ID != item.id.FileID {
					t.Fatal("original pair restoration lost retained identity")
				}
			}
		})
	}
}
func TestNativeCodeUpgradeRefusesForeignRecoveryDestination(t *testing.T) {
	n := codePairNativeFixture(t)
	n.movePrefix(t, 2)
	if e := os.WriteFile(filepath.Join(n.parent.dos, enrollmentImageName), []byte("FOREIGN"), 0600); e != nil {
		t.Fatal(e)
	}
	probe := &nativeEnrollmentFiles{}
	defer probe.Close()
	if _, _, e := readReservedCodePositions(context.Background(), probe, n.parent, enrollmentImageName, previousBrokerName, stagedBrokerName, n.oldBrokerID, n.nextBrokerID, n.sd); e == nil {
		t.Fatal("foreign fixed file adopted")
	}
	if e := resumeOwnedCodePair(context.Background(), func(ctx context.Context) error { return ctx.Err() }, n.parent, &n.oldBroker, &n.oldHelper, &n.nextBroker, &n.nextHelper, n.policy); e == nil {
		t.Fatal("foreign fixed file overwritten")
	}
	raw, e := os.ReadFile(filepath.Join(n.parent.dos, enrollmentImageName))
	if e != nil || !bytes.Equal(raw, []byte("FOREIGN")) {
		t.Fatal("foreign bytes changed", e)
	}
}
func TestNativeCodeUpgradeStageUsesExactProtectedCopy(t *testing.T) {
	owner, _, parent, sd := startupNativeFixture(t)
	n := &nativeCodeUpgrade{original: &nativeInstalledMaintenanceAuthority{codeParent: parent}, files: &nativeEnrollmentFiles{}}
	defer n.files.Close()
	body := []byte("INERT-EXACT-STAGED-BROKER")
	f := newEnrollmentFixture(t)
	a := f.policy.Broker
	a.SHA256 = digest(body)
	a.Bytes = int64(len(body))
	staged, e := n.stageImage(context.Background(), stagedBrokerName, bytes.NewReader(body), a, sd, false)
	if e != nil {
		t.Fatal(e)
	}
	if e = startupHashPin(context.Background(), staged.pin, digest(body)); e != nil {
		t.Fatal(e)
	}
	if e = os.WriteFile(staged.dos, []byte("TAMPER"), 0600); e == nil {
		t.Fatal("staged retained bytes allowed writer")
	}
	if _, e = n.stageImage(context.Background(), stagedBrokerName, bytes.NewReader(body), a, sd, false); e == nil {
		t.Fatal("existing staged code overwritten")
	}
	if e = owner.files.Check(); e != nil {
		t.Fatal(e)
	}
}

func TestNativeCodeUpgradeSuccessorReaderRequiresCompleteTransaction(t *testing.T) {
	for _, fault := range []string{"valid", "missing-drain", "missing-pair", "missing-ready", "wrong-reservation-original"} {
		t.Run(fault, func(t *testing.T) {
			n := codePairNativeFixture(t)
			original := installedCompletionRecordFixture(t)
			original.Broker, original.MaintenanceHelper = n.oldBrokerID, n.oldHelperID
			originalRaw, e := marshalInstalledCompletion(original)
			if e != nil {
				t.Fatal(e)
			}
			client, e := brokerregistration.ParseRetainedApprovedRequest(original.ClientRequest, original.ClientRequestSHA256)
			if e != nil {
				t.Fatal(e)
			}
			n.policy.Bootstrap.TrustConfigSHA256 = client.TrustConfigSHA256
			policyRaw, e := releasepolicy.MarshalBrokerCodeUpgrade(n.policy)
			if e != nil {
				t.Fatal(e)
			}
			parent, e := startupDirectoryIdentity(n.parent)
			if e != nil {
				t.Fatal(e)
			}
			reservation := codeUpgradeRecord{Schema: 1, Purpose: codeUpgradeRecordPurpose, OriginalCompletionSHA256: digest(originalRaw), ClientRequestSHA256: original.ClientRequestSHA256, Policy: policyRaw, PolicySHA256: digest(policyRaw), CodeParent: parent, PreviousBroker: n.oldBrokerID, PreviousHelper: n.oldHelperID, Broker: n.nextBrokerID, Helper: n.nextHelperID}
			if fault == "wrong-reservation-original" {
				reservation.OriginalCompletionSHA256 = digest([]byte("foreign"))
			}
			reservationRaw, e := marshalCodeUpgradeRecord(reservation)
			if e != nil {
				t.Fatal(e)
			}
			intent := reservation
			intent.Broker = startupFileBinding{}
			intent.Helper = startupFileBinding{}
			intentRaw, _ := json.Marshal(intent)
			drainRaw, _ := json.Marshal(codeDrainRecord{Schema: 1, ReservationSHA256: digest(reservationRaw), PID: 1, Birth: 1})
			n.movePrefix(t, 4)
			successor := reservation
			successor.OriginalCompletionSHA256 = digest(originalRaw)
			successor.ReservationSHA256 = digest(reservationRaw)
			successor.PreviousBroker, e = startupFileIdentity(context.Background(), n.oldBroker.pin, n.policy.PreviousBrokerSHA256)
			if e != nil {
				t.Fatal(e)
			}
			successor.PreviousHelper, e = startupFileIdentity(context.Background(), n.oldHelper.pin, n.policy.PreviousHelperSHA256)
			if e != nil {
				t.Fatal(e)
			}
			successor.Broker, e = startupFileIdentity(context.Background(), n.nextBroker.pin, n.policy.Bootstrap.Broker.SHA256)
			if e != nil {
				t.Fatal(e)
			}
			successor.Helper, e = startupFileIdentity(context.Background(), n.nextHelper.pin, n.policy.Bootstrap.Provisioner.SHA256)
			if e != nil {
				t.Fatal(e)
			}
			successorRaw, e := marshalCodeUpgradeRecord(successor)
			if e != nil {
				t.Fatal(e)
			}
			if e = n.files.Close(); e != nil {
				t.Fatal(e)
			}
			n.files = &nativeEnrollmentFiles{}
			for _, item := range []struct {
				suffix string
				raw    []byte
				omit   bool
			}{{codeIntentSuffix, intentRaw, false}, {codeReservationSuffix, reservationRaw, false}, {codeDrainSuffix, drainRaw, fault == "missing-drain"}, {codeSuccessorSuffix, successorRaw, false}, {codeSuccessorPairSuffix, codeSuccessorMarker(digest(successorRaw)), fault == "missing-pair"}, {codeSuccessorMarkerSuffix, codeSuccessorMarker(digest(successorRaw)), fault == "missing-ready"}} {
				if item.omit {
					continue
				}
				if _, e = startupWriteFile(context.Background(), n.files, n.parent, original.ClientSID+item.suffix, item.raw, n.sd, false); e != nil {
					t.Fatal(e)
				}
			}
			c := &nativeInstalledMaintenanceAuthority{files: &nativeEnrollmentFiles{}, clientSID: original.ClientSID, record: original, originalRaw: originalRaw, codeParent: n.parent, requestParent: n.parent}
			defer c.files.Close()
			selected, e := readCodeSuccessorNative(context.Background(), c, original, originalRaw, n.sd, n.sd)
			if fault != "valid" {
				if e == nil {
					t.Fatal("incomplete/foreign successor became installed authority")
				}
				return
			}
			if e != nil || selected.Broker != successor.Broker || selected.MaintenanceHelper != successor.Helper || !bytes.Equal(selected.ClientRequest, original.ClientRequest) || !bytes.Equal(selected.StartupPublication, original.StartupPublication) || selected.Baseline != original.Baseline {
				t.Fatal("successor changed original installed identity", e)
			}
			for _, item := range []struct {
				name string
				want startupFileBinding
			}{{enrollmentImageName, selected.Broker}, {maintenanceImageName, selected.MaintenanceHelper}} {
				pin, e := maintenanceHoldFile(c.files, n.parent, item.name, n.sd, maxBrokerBytes)
				if e != nil {
					t.Fatal(e)
				}
				if e = installedMatchFile(context.Background(), pin, item.want); e != nil {
					t.Fatal(e)
				}
			}
		})
	}
}
