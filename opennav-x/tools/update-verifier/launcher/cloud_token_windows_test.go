//go:build windows

package launcher

import (
	"crypto/rand"
	"encoding/hex"
	"syscall"
	"testing"
	"unsafe"
)

// This test owns a random test-only entry. It never reads or writes the user's
// SKAGER/Cloud/v1 credential, and uses an inert token with no hosted authority.
func TestNativeCloudCredentialRoundTripAndMissing(t *testing.T) {
	var nonce [16]byte
	if _, err := rand.Read(nonce[:]); err != nil {
		t.Fatal(err)
	}
	name := "SKAGER/Tests/Cloud/" + hex.EncodeToString(nonce[:])
	target, err := syscall.UTF16PtrFromString(name)
	if err != nil {
		t.Fatal(err)
	}
	remove := advapi32.NewProc("CredDeleteW")
	write := advapi32.NewProc("CredWriteW")
	t.Cleanup(func() { remove.Call(uintptr(unsafe.Pointer(target)), credTypeGeneric, 0) })
	if readCloudTokenTarget(name) != "" {
		t.Fatal("missing credential returned a token")
	}
	for _, value := range []string{testBrokerCloudToken, "not-a-cloud-token"} {
		blob := []byte(value)
		entry := credentialW{Type: credTypeGeneric, TargetName: target, Persist: 2, CredentialBlobSize: uint32(len(blob)), CredentialBlob: &blob[0]}
		ok, _, err := write.Call(uintptr(unsafe.Pointer(&entry)), 0)
		if ok == 0 {
			t.Fatal("test-only CredWrite failed", err)
		}
		actual := readCloudTokenTarget(name)
		expected := ""
		if value == testBrokerCloudToken {
			expected = value
		}
		if actual != expected {
			t.Fatal("credential parsing/roundtrip mismatch")
		}
	}
	ok, _, err := remove.Call(uintptr(unsafe.Pointer(target)), credTypeGeneric, 0)
	if ok == 0 {
		t.Fatal("test-only CredDelete failed", err)
	}
	if readCloudTokenTarget(name) != "" {
		t.Fatal("deleted credential remained linked")
	}
}
