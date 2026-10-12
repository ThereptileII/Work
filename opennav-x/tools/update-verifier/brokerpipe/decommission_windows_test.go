//go:build windows

package brokerpipe

import (
	"golang.org/x/sys/windows"
	"testing"
)

func TestNativeDecommissionOriginalPrimaryTokenAndCleanup(t *testing.T) {
	p, err := openProcess(windows.GetCurrentProcessId())
	if err != nil {
		t.Fatal(err)
	}
	n := &nativeDecommissionWitness{caller: p}
	defer n.close()
	proof, err := decommissionPrimaryIdentity(p.retainedToken)
	if err != nil {
		t.Fatal(err)
	}
	if err = checkDecommissionPrimary(p, proof); err != nil {
		t.Fatal(err)
	}
	wrong := proof
	wrong.tokenID.LowPart++
	if checkDecommissionPrimary(p, wrong) == nil {
		t.Fatal("wrong original TokenID accepted")
	}
	if err = n.close(); err != nil {
		t.Fatal(err)
	}
	if p.handle != 0 || p.retainedToken != 0 || n.caller != nil {
		t.Fatal("original handles not drained")
	}
	if err = n.close(); err != nil {
		t.Fatal("repeat cleanup failed")
	}
}
func TestNativeDecommissionClosePreservesNativeFailure(t *testing.T) {
	p, err := openProcess(windows.GetCurrentProcessId())
	if err != nil {
		t.Fatal(err)
	}
	if err = p.retainedToken.Close(); err != nil {
		p.close()
		t.Fatal(err)
	}
	// Deliberately retain an already-closed token value only in this inert fixture.
	// Cleanup must report the actual invalid handle and still drain the process.
	n := &nativeDecommissionWitness{caller: p}
	if n.close() == nil {
		t.Fatal("native closure uncertainty discarded")
	}
	if p.handle != 0 || p.retainedToken != 0 || n.caller != nil {
		t.Fatal("later handles not drained after failure")
	}
}
