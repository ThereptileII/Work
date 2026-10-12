//go:build windows

package brokerenroll

import (
	"errors"

	"golang.org/x/sys/windows"
)

// These descriptors are final at creation. The fixed SKAGER parent and broker
// code stay read/execute-only to the broker, while the separate Applications
// subtree permits broker deployment. Neither descriptor establishes historical
// custody of an existing namespace; fresh exclusive creation remains required.
func enrollmentCodeDescriptor(broker *windows.SID, directory bool) (*windows.SECURITY_DESCRIPTOR, error) {
	if broker == nil || !broker.IsValid() {
		return nil, errors.New("fixed broker SID required")
	}
	flags := ""
	if directory {
		flags = "OICI"
	}
	return windows.SecurityDescriptorFromString("O:BAD:P(A;" + flags + ";FA;;;SY)(A;" + flags + ";FA;;;BA)(A;" + flags + ";FRFX;;;AU)(A;" + flags + ";FRFX;;;" + broker.String() + ")")
}

func enrollmentApplicationsDescriptor(broker *windows.SID) (*windows.SECURITY_DESCRIPTOR, error) {
	if broker == nil || !broker.IsValid() {
		return nil, errors.New("fixed broker SID required")
	}
	return windows.SecurityDescriptorFromString("O:BAD:P(A;OICI;FA;;;SY)(A;OICI;FA;;;BA)(A;OICI;FRFX;;;AU)(A;OICI;FA;;;" + broker.String() + ")")
}
