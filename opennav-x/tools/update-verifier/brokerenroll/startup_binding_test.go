package brokerenroll

import (
	"bytes"
	"encoding/json"
	"strings"
	"testing"
)

func startupBindingFixture(t *testing.T) (startupBindingRecord, []byte) {
	t.Helper()
	_, _, request, _ := startupFlowFixture(t)
	file := func(id byte) startupFileBinding {
		return startupFileBinding{startupDirectoryBinding: startupDirectoryBinding{
			Volume: 1, FileID: [16]byte{id}, Creation: uint64(id), DescriptorSHA256: strings.Repeat("a", 64),
		}, Bytes: 12, SHA256: strings.Repeat("b", 64)}
	}
	record := startupBindingRecord{Schema: 1, Purpose: startupBindingPurpose, ClientSID: request.ClientSID,
		ClientRequestSHA256: request.RequestSHA256, Generation: request.Installed.Generation,
		Commit: request.Installed.Commit, PackageSHA256: request.Installed.PackageSHA256,
		PrepublicationSHA256: strings.Repeat("c", 64), Menu: file(1).startupDirectoryBinding,
		Recovery: file(2).startupDirectoryBinding, Launcher: file(3)}
	for i := range record.Links {
		spec, _ := startupShortcutMode(i).spec()
		record.Links[i] = startupLinkBinding{Name: spec.name, New: file(byte(i + 4))}
	}
	original := file(7)
	record.Links[0].Original = &original
	return record, request.OriginalBytes()
}

func TestApprovedStartupPublicationRequiresIndependentExactBinding(t *testing.T) {
	record, request := startupBindingFixture(t)
	raw, err := json.Marshal(record)
	if err != nil {
		t.Fatal(err)
	}
	pin := digest(raw)
	parsed, err := ParseApprovedStartupPublication(raw, pin, request, digest(request))
	if err != nil || parsed.SHA256() != pin || !bytes.Equal(parsed.OriginalBytes(), raw) {
		t.Fatalf("exact approved original binding refused: %v", err)
	}
	raw[0] = 'x'
	copy := parsed.OriginalBytes()
	copy[0] = 'x'
	if parsed.OriginalBytes()[0] != '{' {
		t.Fatal("approved original bytes are mutable by caller")
	}
	valid, _ := json.Marshal(record)
	unknown := append(bytes.Clone(valid[:len(valid)-1]), []byte(`,"unexpected":true}`)...)
	duplicate := append([]byte(`{"schema":1,`), valid[1:]...)
	for _, tc := range []struct {
		name   string
		raw    []byte
		pin    string
		req    []byte
		reqPin string
	}{
		{"unapproved publication", valid, strings.Repeat("0", 64), request, digest(request)},
		{"unapproved request", valid, digest(valid), request, strings.Repeat("0", 64)},
		{"adjacent self declared pin", append(bytes.TrimSuffix(valid, []byte("}")), []byte(`,"pin":"`+digest(valid)+`"}`)...), digest(valid), request, digest(request)},
		{"unknown field", unknown, digest(unknown), request, digest(request)},
		{"duplicate key", duplicate, digest(duplicate), request, digest(request)},
		{"trailing document", append(bytes.Clone(valid), valid...), digest(append(bytes.Clone(valid), valid...)), request, digest(request)},
		{"noncanonical whitespace", append([]byte(" "), valid...), digest(append([]byte(" "), valid...)), request, digest(request)},
	} {
		t.Run(tc.name, func(t *testing.T) {
			if _, err := ParseApprovedStartupPublication(tc.raw, tc.pin, tc.req, tc.reqPin); err == nil {
				t.Fatal("unapproved or noncanonical binding accepted")
			}
		})
	}
}

func TestApprovedStartupPublicationRejectsSubstitutedOrPartialObjects(t *testing.T) {
	original, request := startupBindingFixture(t)
	for _, tc := range []struct {
		name   string
		change func(*startupBindingRecord)
	}{
		{"wrong generation", func(r *startupBindingRecord) { r.Generation = strings.Repeat("f", 32) }},
		{"wrong package", func(r *startupBindingRecord) { r.PackageSHA256 = strings.Repeat("f", 64) }},
		{"missing completion evidence", func(r *startupBindingRecord) { r.PrepublicationSHA256 = "" }},
		{"missing safe link", func(r *startupBindingRecord) { r.Links[2] = startupLinkBinding{} }},
		{"foreign normal identity", func(r *startupBindingRecord) { r.Links[0].New.FileID = r.Links[1].New.FileID }},
		{"missing backup hash", func(r *startupBindingRecord) { r.Links[0].Original.SHA256 = "" }},
		{"duplicate backup identity", func(r *startupBindingRecord) { r.Links[1].Original = r.Links[0].Original }},
		{"wrong menu volume", func(r *startupBindingRecord) { r.Menu.Volume = 9 }},
	} {
		t.Run(tc.name, func(t *testing.T) {
			record := original
			backup := *original.Links[0].Original
			record.Links[0].Original = &backup
			tc.change(&record)
			raw, err := json.Marshal(record)
			if err != nil {
				t.Fatal(err)
			}
			if _, err := ParseApprovedStartupPublication(raw, digest(raw), request, digest(request)); err == nil {
				t.Fatal("substituted or incomplete native identity accepted")
			}
		})
	}
}

func TestApprovedStartupBindingRetainsEveryOriginalRoleAndOldRecords(t *testing.T) {
	for mask := 0; mask < 8; mask++ {
		record, request := startupBindingFixture(t)
		original := *record.Links[0].Original
		for i := range record.Links {
			record.Links[i].Original = nil
			if mask&(1<<i) != 0 {
				backup := original
				backup.FileID = [16]byte{byte(7 + i)}
				record.Links[i].Original = &backup
			}
		}
		raw, err := json.Marshal(record)
		if err != nil {
			t.Fatal(err)
		}
		approved, err := ParseApprovedStartupPublication(raw, digest(raw), request, digest(request))
		if err != nil || !bytes.Equal(approved.OriginalBytes(), raw) {
			t.Fatalf("original role mask %d did not retain its exact binding: %v", mask, err)
		}
	}
}
