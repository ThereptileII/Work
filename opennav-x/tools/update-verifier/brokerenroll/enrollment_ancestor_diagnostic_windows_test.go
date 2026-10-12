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
	sd, err := windows.SecurityDescriptorFromString("O:BAD:P(A;;FA;;;SY)(A;CI;0x100116;;;BU)")
	if err != nil {
		t.Fatal(err)
	}
	refusal := enrollmentSecurityDescriptor(sd, nil, true, "")
	var observed *enrollmentAncestorMutationError
	if !errors.As(errors.Join(refusal, errors.New("fixture cleanup")), &observed) {
		t.Fatal("original observed ancestor rejection lost", refusal)
	}
	if observed.ownerSID != "S-1-5-32-544" || observed.aceSID != "S-1-5-32-545" ||
		!observed.aceSIDValid || observed.mask != 0x100116 || observed.flags != windows.CONTAINER_INHERIT_ACE {
		t.Fatal("observed descriptor facts differ")
	}
	if refusal.Error() != "untrusted enrollment ancestor mutation" {
		t.Fatal("public refusal text changed")
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
