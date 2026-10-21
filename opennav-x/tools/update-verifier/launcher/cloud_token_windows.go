//go:build windows

package launcher

import (
	"syscall"
	"unsafe"

	verifier "example.com/opennav-update-verifier"
)

// Generic credential written by SKAGER after app pairing (src/platform/windows/
// AisCredentials.cpp, CreateCloudCredentials). Per user, so it is read in the
// launcher's own user context and never by the SYSTEM broker.
const cloudCredentialTarget = "SKAGER/Cloud/v1"

var (
	advapi32     = syscall.NewLazyDLL("advapi32.dll")
	procCredRead = advapi32.NewProc("CredReadW")
	procCredFree = advapi32.NewProc("CredFree")
)

type credentialW struct {
	Flags              uint32
	Type               uint32
	TargetName         *uint16
	Comment            *uint16
	LastWritten        syscall.Filetime
	CredentialBlobSize uint32
	CredentialBlob     *byte
	Persist            uint32
	AttributeCount     uint32
	Attributes         uintptr
	TargetAlias        *uint16
	UserName           *uint16
}

const credTypeGeneric = 1

// readCloudToken returns the device token, or "" when SKAGER is not linked or the
// stored value is not a well-formed token. Errors are deliberately not reported:
// without a token the launcher simply makes no update check.
func readCloudToken() string {
	target, err := syscall.UTF16PtrFromString(cloudCredentialTarget)
	if err != nil {
		return ""
	}
	var credential *credentialW
	ok, _, _ := procCredRead.Call(uintptr(unsafe.Pointer(target)), credTypeGeneric, 0, uintptr(unsafe.Pointer(&credential)))
	if ok == 0 || credential == nil {
		return ""
	}
	defer procCredFree.Call(uintptr(unsafe.Pointer(credential)))
	if credential.Type != credTypeGeneric || credential.CredentialBlob == nil || credential.CredentialBlobSize == 0 ||
		credential.CredentialBlobSize > 128 {
		return ""
	}
	token := string(unsafe.Slice(credential.CredentialBlob, credential.CredentialBlobSize))
	if !verifier.AccessTokenPattern.MatchString(token) {
		return ""
	}
	return token
}
