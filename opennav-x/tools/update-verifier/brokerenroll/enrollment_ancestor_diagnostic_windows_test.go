//go:build windows

package brokerenroll

import (
	"errors"
	"testing"

	"golang.org/x/sys/windows"
)

// Descriptor-only observations create no files, services or privileged state.
// The example ACE is a fixture, never a claim about a real host's current ACL.
func TestEnrollmentAncestorMutationDiagnosticPreservesObservedACE(t *testing.T) {
	sd, err := windows.SecurityDescriptorFromString("O:BAD:P(A;;FA;;;SY)(A;CI;0x116;;;BU)")
	if err != nil {
		t.Fatal(err)
	}
	refusal := enrollmentSecurityDescriptor(sd, nil, true, "")
	var observed *enrollmentAncestorMutationError
	if !errors.As(errors.Join(refusal, errors.New("fixture cleanup")), &observed) {
		t.Fatal("original observed ancestor rejection lost", refusal)
	}
	if observed.ownerSID != "S-1-5-32-544" || observed.aceSID != "S-1-5-32-545" ||
		!observed.aceSIDValid || observed.mask != 0x116 || observed.flags != windows.CONTAINER_INHERIT_ACE || !observed.allowAdds {
		t.Fatal("observed descriptor facts differ")
	}
	if refusal.Error() != "untrusted enrollment ancestor mutation" {
		t.Fatal("public refusal text changed")
	}
}

func TestEnrollmentAncestorDiagnosticBindsClosedCategoryAndOriginalPolicy(t *testing.T) {
	sd, err := windows.SecurityDescriptorFromString("O:SYD:P(A;;FA;;;SY)(A;CI;0x116;;;BU)")
	if err != nil {
		t.Fatal(err)
	}
	for _, location := range []struct {
		role      enrollmentAncestorRole
		component uint32
		name      string
	}{
		{enrollmentAncestorDriveRoot, 0, "drive-root"},
		{enrollmentAncestorInstallRoot, 1, "install-root"},
		{enrollmentAncestorDataRoot, 2, "data-root"},
		{enrollmentAncestorRole(255), 0, "unspecified"},
	} {
		for _, allowAdds := range []bool{false, true} {
			pin := &enrollmentPin{ancestor: enrollmentAncestorLocation{role: location.role, component: location.component}}
			refusal := pin.ancestorDiagnostic(enrollmentSecurityDescriptor(sd, nil, allowAdds, ""))
			var observed *enrollmentAncestorMutationError
			if !errors.As(refusal, &observed) || observed.ancestor != pin.ancestor || observed.allowAdds != allowAdds ||
				observed.ancestor.role.String() != location.name || refusal.Error() != "untrusted enrollment ancestor mutation" {
				t.Fatal("closed category or original policy observation differs")
			}
			if observed.ownerSID != "S-1-5-18" || observed.aceSID != "S-1-5-32-545" || observed.mask != 0x116 ||
				observed.flags != windows.CONTAINER_INHERIT_ACE || !observed.aceSIDValid {
				t.Fatal("category enrichment changed original descriptor observations")
			}
		}
	}
}

func TestEnrollmentAncestorDiagnosticLeavesExistingPolicyUnchanged(t *testing.T) {
	for _, sddl := range []string{
		"O:BAD:P(A;;FA;;;SY)(A;CIIO;FA;;;BU)",
		"O:BAD:P(A;;FA;;;SY)(A;;0x6;;;BU)",
		"O:BAD:P(A;;FA;;;SY)(A;;FA;;;BA)",
	} {
		sd, err := windows.SecurityDescriptorFromString(sddl)
		if err != nil {
			t.Fatal(err)
		}
		if err := enrollmentSecurityDescriptor(sd, nil, true, ""); err != nil {
			t.Fatal("accepted existing ancestor policy changed", err)
		}
	}
	sd, err := windows.SecurityDescriptorFromString("O:BUD:P(A;;FA;;;SY)")
	if err != nil {
		t.Fatal(err)
	}
	refusal := enrollmentSecurityDescriptor(sd, nil, true, "")
	var observed *enrollmentAncestorMutationError
	if refusal == nil || errors.As(refusal, &observed) || refusal.Error() != "untrusted enrollment ancestor owner" {
		t.Fatal("owner refusal relabeled as observed ACE rejection", refusal)
	}
}
