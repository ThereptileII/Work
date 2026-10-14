package brokerenroll

import (
	"bytes"
	"encoding/json"
	"example.com/opennav-update-verifier/releasepolicy"
	"strings"
	"testing"
)

func codeUpgradeRecordFixture(t *testing.T) codeUpgradeRecord {
	t.Helper()
	f := newEnrollmentFixture(t)
	original := installedCompletionRecordFixture(t)
	originalRaw, e := marshalInstalledCompletion(original)
	if e != nil {
		t.Fatal(e)
	}
	p := releasepolicy.BrokerCodeUpgradePolicy{Schema: 1, Purpose: releasepolicy.BrokerCodeUpgradePurpose, PreviousBrokerSHA256: original.Broker.SHA256, PreviousHelperSHA256: original.MaintenanceHelper.SHA256, Bootstrap: f.policy}
	policyRaw, e := releasepolicy.MarshalBrokerCodeUpgrade(p)
	if e != nil {
		t.Fatal(e)
	}
	file := func(id byte, a releasepolicy.Artifact) startupFileBinding {
		return startupFileBinding{startupDirectoryBinding: startupDirectoryBinding{Volume: 1, FileID: [16]byte{id}, Creation: uint64(id), DescriptorSHA256: strings.Repeat("a", 64)}, Bytes: a.Bytes, SHA256: a.SHA256}
	}
	return codeUpgradeRecord{Schema: 1, Purpose: codeUpgradeRecordPurpose, OriginalCompletionSHA256: digest(originalRaw), ClientRequestSHA256: original.ClientRequestSHA256, Policy: policyRaw, PolicySHA256: digest(policyRaw), CodeParent: startupDirectoryBinding{Volume: 1, FileID: [16]byte{18}, Creation: 18, DescriptorSHA256: strings.Repeat("a", 64)}, PreviousBroker: original.Broker, PreviousHelper: original.MaintenanceHelper, Broker: file(16, p.Bootstrap.Broker), Helper: file(17, p.Bootstrap.Provisioner)}
}
func TestCodeUpgradeRecordPreservesExactPairAndRefusesSubstitution(t *testing.T) {
	r := codeUpgradeRecordFixture(t)
	raw, e := marshalCodeUpgradeRecord(r)
	if e != nil {
		t.Fatal(e)
	}
	if _, e = parseCodeUpgradeRecord(raw); e != nil {
		t.Fatal(e)
	}
	for _, change := range []func(*codeUpgradeRecord){func(r *codeUpgradeRecord) { r.Purpose = installedCompletionPurpose }, func(r *codeUpgradeRecord) { r.Policy = append(r.Policy, ' ') }, func(r *codeUpgradeRecord) { r.Broker.FileID = r.PreviousBroker.FileID }, func(r *codeUpgradeRecord) { r.Helper.Volume++ }, func(r *codeUpgradeRecord) { r.PreviousHelper.SHA256 = r.Helper.SHA256 }, func(r *codeUpgradeRecord) { r.Helper.Bytes++ }, func(r *codeUpgradeRecord) { r.OriginalCompletionSHA256 = "" }, func(r *codeUpgradeRecord) { r.ReservationSHA256 = "foreign" }} {
		bad := r
		change(&bad)
		badRaw, _ := json.Marshal(bad)
		if _, e = parseCodeUpgradeRecord(badRaw); e == nil {
			t.Fatal("foreign/partial code record accepted")
		}
	}
	for _, bad := range [][]byte{nil, append([]byte(" "), raw...), append(bytes.Clone(raw), raw...), append([]byte(`{"schema":1,`), raw[1:]...)} {
		if _, e = parseCodeUpgradeRecord(bad); e == nil {
			t.Fatal("noncanonical code record accepted")
		}
	}
	changed := r.Broker
	changed.DescriptorSHA256 = strings.Repeat("b", 64)
	if sameCodeFile(changed, r.Broker) {
		t.Fatal("changed descriptor accepted during recovery")
	}
}
func TestCodeUpgradeDrainProofRequiresExactCleanOriginalProcess(t *testing.T) {
	sha := strings.Repeat("a", 64)
	r := codeDrainRecord{Schema: 1, ReservationSHA256: sha, PID: 42, Birth: 123, ExitCode: 0}
	raw, _ := json.Marshal(r)
	if _, e := parseCodeDrainRecord(raw, sha); e != nil {
		t.Fatal(e)
	}
	for _, change := range []func(*codeDrainRecord){func(r *codeDrainRecord) { r.Schema = 2 }, func(r *codeDrainRecord) { r.ReservationSHA256 = strings.Repeat("b", 64) }, func(r *codeDrainRecord) { r.PID = 0 }, func(r *codeDrainRecord) { r.Birth = 0 }, func(r *codeDrainRecord) { r.ExitCode = 1 }} {
		bad := r
		change(&bad)
		badRaw, _ := json.Marshal(bad)
		if _, e := parseCodeDrainRecord(badRaw, sha); e == nil {
			t.Fatal("unproven original process exit accepted")
		}
	}
	for _, bad := range [][]byte{nil, []byte("null"), append(bytes.Clone(raw), ' '), append([]byte(`{"schema":1,`), raw[1:]...)} {
		if _, e := parseCodeDrainRecord(bad, sha); e == nil {
			t.Fatal("noncanonical drain proof accepted")
		}
	}
}
