//go:build windows

package brokerenroll

import (
	"testing"

	"golang.org/x/sys/windows"
)

// These tests inspect native SID/descriptor policy only. They create no machine
// directories, files or services and do not qualify real enrollment execution.
func TestEnrollmentSIDBeforeServiceCreation(t *testing.T) {
	sid, err := enrollmentBrokerSID()
	if err != nil || sid == nil || !sid.IsValid() {
		t.Fatalf("native fixed service SID: %v", err)
	}
}

func TestEnrollmentPhysicalPaths(t *testing.T) {
	for _, path := range []string{`\Device\HarddiskVolume3`, `\Device\HarddiskVolume31\Program Files (x86)\SKAGER`} {
		if _, err := enrollmentPhysicalParts(path); err != nil {
			t.Fatalf("physical path: %v", err)
		}
	}
	for _, path := range []string{`C:\SKAGER`, `\Device\HarddiskVolumeX\SKAGER`, `\Device\HarddiskVolume3\..\SKAGER`, `\Device\HarddiskVolume3\SKAGER.`, `\Device\HarddiskVolume3\SKAGER:stream`} {
		if _, err := enrollmentPhysicalParts(path); err == nil {
			t.Fatal("unsafe physical path accepted")
		}
	}
}

func TestEnrollmentCodeDescriptorDoesNotGrantUserMutation(t *testing.T) {
	sid, err := enrollmentBrokerSID()
	if err != nil {
		t.Fatal(err)
	}
	sd, err := enrollmentCodeDescriptor(sid, true)
	if err != nil {
		t.Fatal(err)
	}
	control, _, err := sd.Control()
	if err != nil || control&windows.SE_DACL_PROTECTED == 0 {
		t.Fatal("code inheritance is not protected")
	}
	acl, _, err := sd.DACL()
	if err != nil || acl == nil || acl.AceCount != 4 {
		t.Fatal("code descriptor entries differ")
	}
	if err := enrollmentSecurityDescriptor(sd, sd, false, ""); err != nil {
		t.Fatalf("exact code descriptor refused: %v", err)
	}
}

func TestEnrollmentAncestorMutationRefusals(t *testing.T) {
	for _, sddl := range []string{
		"O:BUD:P(A;;FA;;;SY)",
		"O:BAD:P(A;;FA;;;SY)(A;;GW;;;BU)",
		"O:BAD:P(A;;FA;;;SY)(A;;0x2;;;BU)",
		"O:BAD:P(A;;FA;;;SY)(A;;0x40;;;BU)",
		"O:BAD:P(A;;FA;;;SY)(A;;0x40000;;;BU)",
	} {
		sd, err := windows.SecurityDescriptorFromString(sddl)
		if err != nil {
			t.Fatal(err)
		}
		if err := enrollmentSecurityDescriptor(sd, nil, false, ""); err == nil {
			t.Fatalf("untrusted code namespace mutation accepted: %s", sddl)
		}
	}
	sd, err := windows.SecurityDescriptorFromString("O:BAD:P(A;;FA;;;SY)(A;;0x2;;;BU)")
	if err != nil {
		t.Fatal(err)
	}
	if err := enrollmentSecurityDescriptor(sd, nil, true, ""); err != nil {
		t.Fatalf("unrelated ancestor child creation refused: %v", err)
	}
	want, err := windows.SecurityDescriptorFromString("O:BAD:P(A;;FA;;;SY)(A;;FA;;;BA)")
	if err != nil {
		t.Fatal(err)
	}
	if err := enrollmentSecurityDescriptor(sd, want, false, ""); err == nil {
		t.Fatal("broadened fresh descriptor accepted")
	}
}
