package brokerenroll

import (
	"bytes"
	"encoding/json"
	"strings"
	"testing"
)

func installedCompletionRecordFixture(t *testing.T) installedCompletionRecord {
	t.Helper()
	publication, request := startupBindingFixture(t)
	raw, err := json.Marshal(publication)
	if err != nil {
		t.Fatal(err)
	}
	file := func(id byte, body []byte) startupFileBinding {
		return startupFileBinding{startupDirectoryBinding: startupDirectoryBinding{Volume: 1, FileID: [16]byte{id}, Creation: uint64(id), DescriptorSHA256: strings.Repeat("a", 64)}, Bytes: int64(len(body)), SHA256: digest(body)}
	}
	return installedCompletionRecord{Schema: 1, Purpose: installedCompletionPurpose, ClientSID: publication.ClientSID, ClientRequest: request, ClientRequestSHA256: digest(request), StartupPublication: raw, StartupPublicationSHA256: digest(raw), Parent: file(10, []byte("parent")).startupDirectoryBinding, Request: file(11, request), Publication: file(12, raw), Baseline: file(13, []byte("baseline")), Broker: file(14, []byte("broker")), MaintenanceHelper: file(15, []byte("helper"))}
}

func TestInstalledCompletionStartupRecordPreservesExactOriginals(t *testing.T) {
	r := installedCompletionRecordFixture(t)
	raw, err := marshalInstalledCompletion(r)
	if err != nil {
		t.Fatal(err)
	}
	parsed, err := parseInstalledCompletion(raw)
	if err != nil {
		t.Fatal(err)
	}
	if !bytes.Equal(parsed.ClientRequest, r.ClientRequest) || !bytes.Equal(parsed.StartupPublication, r.StartupPublication) || parsed.Baseline != r.Baseline || parsed.MaintenanceHelper != r.MaintenanceHelper {
		t.Fatal("completion lost exact originals/native identities")
	}
	raw[0] = '!'
	if parsed.ClientRequest[0] != '{' {
		t.Fatal("parsed original aliases caller input")
	}
}

func TestInstalledCompletionStartupRecordRefusesSubstitution(t *testing.T) {
	for _, tc := range []struct {
		name   string
		change func(*installedCompletionRecord)
	}{
		{"wrong-purpose", func(r *installedCompletionRecord) { r.Purpose = "uninstall" }},
		{"wrong-client", func(r *installedCompletionRecord) { r.ClientSID = "S-1-5-21-1-2-3-1001" }},
		{"wrong-request", func(r *installedCompletionRecord) { r.ClientRequest = []byte("{}") }},
		{"wrong-publication", func(r *installedCompletionRecord) { r.StartupPublication = []byte("{}") }},
		{"request-size", func(r *installedCompletionRecord) { r.Request.Bytes++ }},
		{"publication-hash", func(r *installedCompletionRecord) { r.Publication.SHA256 = strings.Repeat("f", 64) }},
		{"missing-parent", func(r *installedCompletionRecord) { r.Parent.FileID = [16]byte{} }},
		{"missing-baseline", func(r *installedCompletionRecord) { r.Baseline.Bytes = 0 }},
		{"oversized-baseline", func(r *installedCompletionRecord) { r.Baseline.Bytes = maxClientBaselineBytes + 1 }},
		{"samebyte-object-replacement", func(r *installedCompletionRecord) { r.Baseline.FileID = r.Request.FileID }},
		{"crossvolume-request", func(r *installedCompletionRecord) { r.Request.Volume++ }},
		{"crossvolume-helper", func(r *installedCompletionRecord) { r.MaintenanceHelper.Volume++ }},
		{"missing-helper-hash", func(r *installedCompletionRecord) { r.MaintenanceHelper.SHA256 = "" }},
	} {
		t.Run(tc.name, func(t *testing.T) {
			r := installedCompletionRecordFixture(t)
			tc.change(&r)
			raw, _ := json.Marshal(r)
			if _, err := parseInstalledCompletion(raw); err == nil {
				t.Fatal("substituted original accepted")
			}
		})
	}
	raw, err := marshalInstalledCompletion(installedCompletionRecordFixture(t))
	if err != nil {
		t.Fatal(err)
	}
	for _, bad := range [][]byte{append([]byte(" "), raw...), append(bytes.Clone(raw), raw...), append([]byte(`{"schema":1,`), raw[1:]...), append(bytes.Clone(raw[:len(raw)-1]), []byte(`,"unknown":true}`)...), bytes.Repeat([]byte("x"), maxInstalledCompletionBytes+1)} {
		if _, err := parseInstalledCompletion(bad); err == nil {
			t.Fatal("noncanonical completion accepted")
		}
	}
}

func TestInstalledCompletionStartupMarkerRequiresDurableCompletedRecord(t *testing.T) {
	sha := strings.Repeat("a", 64)
	completed := installedCompletionMarker(sha, true)
	if len(completed) > maxInstalledCompletionMarkerBytes || len(completed) != len(installedCompletionMarker(sha, false)) || validateInstalledCompletionMarker(completed, sha) != nil {
		t.Fatal("completed reservation does not validate")
	}
	for _, bad := range [][]byte{nil, installedCompletionMarker(sha, false), installedCompletionMarker(strings.Repeat("b", 64), true), append(bytes.Clone(completed), ' '), append(bytes.Clone(completed), completed...)} {
		if validateInstalledCompletionMarker(bad, sha) == nil {
			t.Fatal("partial/foreign reservation accepted")
		}
	}
	if installedCompletionMarker("", true) != nil || validateInstalledCompletionMarker(completed, "") == nil {
		t.Fatal("unbound reservation accepted")
	}
}
