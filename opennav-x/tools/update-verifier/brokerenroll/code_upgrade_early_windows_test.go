//go:build windows

package brokerenroll

import (
	"bytes"
	"context"
	"encoding/json"
	"os"
	"path/filepath"
	"testing"

	"example.com/opennav-update-verifier/releasepolicy"
)

// Disposable NTFS closure only; no installed service, application, user profile,
// process or code transition is created. Actual SCM/IPC remains a separate gate.
func nativeEarlyAbortFixture(t *testing.T, stages int, reservation bool, fault string) (*nativeInstalledMaintenanceAuthority, *nativeCodePairFixture) {
	t.Helper()
	n := codePairNativeFixture(t)
	if err := n.files.Close(); err != nil {
		t.Fatal(err)
	}
	n.files = &nativeEnrollmentFiles{}
	for i, name := range []string{stagedBrokerName, stagedHelperName} {
		if i >= stages {
			if err := os.Remove(filepath.Join(n.parent.dos, name)); err != nil {
				t.Fatal(err)
			}
		}
	}
	original := installedCompletionRecordFixture(t)
	original.Broker, original.MaintenanceHelper = n.oldBrokerID, n.oldHelperID
	originalRaw, err := marshalInstalledCompletion(original)
	if err != nil {
		t.Fatal(err)
	}
	c := &nativeInstalledMaintenanceAuthority{files: &nativeEnrollmentFiles{}, clientSID: original.ClientSID,
		record: original, originalRaw: originalRaw, requestParent: n.parent, codeParent: n.parent}
	t.Cleanup(func() {
		if err := c.files.Close(); err != nil {
			t.Error(err)
		}
	})
	n.policy.Bootstrap.TrustConfigSHA256 = c.recordTrustSHA()
	policyRaw, err := releasepolicy.MarshalBrokerCodeUpgrade(n.policy)
	if err != nil {
		t.Fatal(err)
	}
	parent, err := startupDirectoryIdentity(n.parent)
	if err != nil {
		t.Fatal(err)
	}
	intent := codeUpgradeRecord{Schema: 1, Purpose: codeUpgradeRecordPurpose, OriginalCompletionSHA256: digest(originalRaw),
		ClientRequestSHA256: original.ClientRequestSHA256, Policy: policyRaw, PolicySHA256: digest(policyRaw), CodeParent: parent,
		PreviousBroker: original.Broker, PreviousHelper: original.MaintenanceHelper}
	intentRaw, _ := json.Marshal(intent)
	if fault == "torn-intent" {
		intentRaw = []byte("{")
	}
	write := func(suffix string, raw []byte) {
		t.Helper()
		if _, err := startupWriteFile(context.Background(), n.files, n.parent, c.clientSID+suffix, raw, n.sd, false); err != nil {
			t.Fatal(err)
		}
	}
	write(codeIntentSuffix, intentRaw)
	for i, stage := range []struct {
		name, suffix string
		file         startupFileBinding
	}{{stagedBrokerName, codeBrokerStageSuffix, n.nextBrokerID}, {stagedHelperName, codeHelperStageSuffix, n.nextHelperID}} {
		if i >= stages || fault == "missing-receipt" && i == 0 {
			continue
		}
		receipt := codeStageReceipt{1, digest(intentRaw), stage.name, stage.file}
		if fault == "wrong-intent-receipt" && i == 0 {
			receipt.IntentSHA256 = digest([]byte("FOREIGN"))
		}
		raw, _ := json.Marshal(receipt)
		if fault == "torn-receipt" && i == 0 {
			raw = []byte("{")
		}
		write(stage.suffix, raw)
	}
	if reservation {
		intent.Broker, intent.Helper = n.nextBrokerID, n.nextHelperID
		raw, err := marshalCodeUpgradeRecord(intent)
		if err != nil {
			t.Fatal(err)
		}
		if fault == "torn-reservation" {
			raw = []byte("{")
		}
		write(codeReservationSuffix, raw)
	}
	if fault == "drain-present" {
		write(codeDrainSuffix, []byte("PARTIAL"))
	}
	if fault == "start-present" {
		write(codeStartAttemptSuffix, []byte("PARTIAL"))
	}
	if fault == "demand-present" {
		write(codeDemandAttemptSuffix, []byte("PARTIAL"))
	}
	if fault == "backup-present" {
		if err := os.WriteFile(filepath.Join(n.parent.dos, previousBrokerName), []byte("FOREIGN"), 0600); err != nil {
			t.Fatal(err)
		}
	}
	if fault == "replacement-stage" || fault == "partial-stage" {
		path := filepath.Join(n.parent.dos, stagedBrokerName)
		if err := os.Remove(path); err != nil {
			t.Fatal(err)
		}
		body := []byte("NEW-BROKER")
		if fault == "partial-stage" {
			body = []byte("NEW")
		}
		pin, err := startupWriteFile(context.Background(), n.files, n.parent, stagedBrokerName, body, n.sd, false)
		if err != nil {
			t.Fatal(err)
		}
		identity, err := startupFileIdentity(context.Background(), pin, digest(body))
		if err != nil || identity == n.nextBrokerID {
			t.Fatal("foreign-stage fixture did not change original native identity", err)
		}
	}
	return c, n
}

func TestNativeCodeUpgradeEarlyAbortReaderRequiresExactCompleteClosure(t *testing.T) {
	for _, scenario := range []struct {
		name        string
		stages      int
		reservation bool
	}{{"intent-only", 0, false}, {"broker-complete", 1, false}, {"pair-complete", 2, false}, {"reserved-running-original", 2, true}} {
		t.Run(scenario.name, func(t *testing.T) {
			c, n := nativeEarlyAbortFixture(t, scenario.stages, scenario.reservation, "")
			ctx := context.Background()
			if _, err := readCodeSuccessorNative(ctx, c, c.record, c.originalRaw, n.sd, n.sd); err == nil {
				t.Fatal("early state selected original without abort closure")
			}
			raw, err := readCodeEarlyStateNative(ctx, c, c.record, c.originalRaw, n.sd, n.sd)
			if err != nil {
				t.Fatal(err)
			}
			if _, err := startupWriteFile(ctx, n.files, n.parent, c.clientSID+codeEarlyAbortSuffix, raw, n.sd, false); err != nil {
				t.Fatal(err)
			}
			selected, err := readCodeSuccessorNative(ctx, c, c.record, c.originalRaw, n.sd, n.sd)
			if err != nil || selected.Broker != c.record.Broker || selected.MaintenanceHelper != c.record.MaintenanceHelper ||
				!bytes.Equal(selected.ClientRequest, c.record.ClientRequest) || selected.Baseline != c.record.Baseline {
				t.Fatal("verified abort lost original installation", err)
			}
			for _, suffix := range []string{codeSuccessorSuffix, codeSuccessorMarkerSuffix, codeStartAttemptSuffix, codeDemandAttemptSuffix} {
				if err := startupRestorationAbsent(c.requestParent, c.clientSID+suffix); err != nil {
					t.Fatal("early abort fabricated successor/start evidence", err)
				}
			}
		})
	}
}

func TestNativeCodeUpgradeEarlyAbortRefusesUnprovablePartialAndForeignStages(t *testing.T) {
	for _, fault := range []string{"torn-intent", "missing-receipt", "torn-receipt", "wrong-intent-receipt", "torn-reservation", "replacement-stage", "partial-stage", "drain-present", "start-present", "demand-present", "backup-present"} {
		t.Run(fault, func(t *testing.T) {
			c, n := nativeEarlyAbortFixture(t, 2, true, fault)
			if _, err := readCodeEarlyStateNative(context.Background(), c, c.record, c.originalRaw, n.sd, n.sd); err == nil {
				t.Fatal("unknown/partial physical state granted abort authority")
			}
			if err := startupRestorationAbsent(c.requestParent, c.clientSID+codeEarlyAbortSuffix); err != nil {
				t.Fatal("refusal published abort evidence", err)
			}
		})
	}
}

func TestNativeCodeUpgradeEarlyAbortRejectsPartialOrForeignTerminalRecord(t *testing.T) {
	for _, body := range [][]byte{[]byte("{"), []byte("FOREIGN"), nil} {
		c, n := nativeEarlyAbortFixture(t, 0, false, "")
		path := filepath.Join(c.requestParent.dos, c.clientSID+codeEarlyAbortSuffix)
		if err := os.WriteFile(path, body, 0600); err != nil {
			t.Fatal(err)
		}
		if _, err := readCodeSuccessorNative(context.Background(), c, c.record, c.originalRaw, n.sd, n.sd); err == nil {
			t.Fatal("partial/foreign abort enabled original fallback")
		}
	}
}
