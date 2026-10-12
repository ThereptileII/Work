//go:build windows

package launcher

import (
	"context"
	"testing"
	"unsafe"

	"golang.org/x/sys/windows"
)

// These tests only manipulate allocated descriptors/capability state. They do
// not grant access to a running process/token, look up a service or launch code.
func TestBrokerQueryAccessNativeUnprovenOneUse(t *testing.T) {
	descriptor, err := windows.SecurityDescriptorFromString("O:S-1-5-21-1-2-3-4G:S-1-5-21-1-2-3-4D:P(A;;0x001fffff;;;S-1-5-21-1-2-3-4)(A;;0x001fffff;;;SY)(A;;0x00101400;;;S-1-5-80-1-2-3-4-5)")
	if err != nil {
		t.Fatal(err)
	}
	owner := &brokerQueryAccess{childSD: descriptor, attributes: windows.SecurityAttributes{Length: uint32(unsafe.Sizeof(windows.SecurityAttributes{})), SecurityDescriptor: descriptor}}
	attrs, err := owner.childProcessAttributes()
	if err == nil || attrs != nil || !owner.attributesUsed {
		t.Fatal("unproven executor issued birth descriptor", err)
	}
	if _, err = owner.childProcessAttributes(); err == nil {
		t.Fatal("birth descriptor reused")
	}
	if err = owner.bindSuspendedChild(context.Background(), &nativeOrdinaryChild{}); err == nil {
		t.Fatal("unowned external child accepted")
	}
	if err = owner.bindSuspendedChild(context.Background(), &nativeOrdinaryChild{}); err == nil {
		t.Fatal("child bind retried")
	}
	first := owner.Close()
	if first == nil || owner.Close() != first {
		t.Fatal("poisoned close failure not cached")
	}
	if err = owner.Validate(context.Background()); err == nil {
		t.Fatal("closed proof accepted")
	}
	if _, err = owner.childProcessAttributes(); err == nil {
		t.Fatal("closed birth accepted")
	}
}
func TestBrokerQueryAccessNativeExactDescriptor(t *testing.T) {
	original, err := windows.SecurityDescriptorFromString("O:S-1-5-21-1-2-3-4G:S-1-5-21-1-2-3-4D:P(A;;0x001fffff;;;S-1-5-21-1-2-3-4)(A;;0x001fffff;;;SY)")
	if err != nil {
		t.Fatal(err)
	}
	copy, err := original.ToAbsolute()
	if err != nil {
		t.Fatal(err)
	}
	copy, err = copy.ToSelfRelative()
	if err != nil || !brokerQuerySameSD(original, copy) {
		t.Fatal("descriptor roundtrip", err)
	}
	for _, sddl := range []string{"O:SYG:SYD:P(A;;0x001fffff;;;SY)", "O:S-1-5-21-1-2-3-4G:S-1-5-21-1-2-3-4D:P(A;;0x001fffff;;;S-1-5-21-1-2-3-4)(A;;0x001fffff;;;SY)(A;;8;;;WD)", "O:S-1-5-21-1-2-3-4G:S-1-5-21-1-2-3-4D:NO_ACCESS_CONTROL"} {
		changed, e := windows.SecurityDescriptorFromString(sddl)
		if e != nil {
			t.Fatal(e)
		}
		if brokerQuerySameSD(original, changed) {
			t.Fatal("changed descriptor accepted")
		}
	}
}
