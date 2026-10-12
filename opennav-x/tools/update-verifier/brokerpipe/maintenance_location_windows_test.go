package brokerpipe

import (
	"errors"
	"runtime"
	"strings"
	"testing"

	"golang.org/x/sys/windows"
)

func TestMaintenanceOperatorLocationActualOriginalTokenAndCallbackLifetime(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	f := actualNativePeerFixture(t)
	identity := nativePeerEvidence(f.process)
	used := false
	want := currentProcessHasEnabledAdmin(t) && windows.GetCurrentProcessToken().IsElevated()
	expected, locationErr := f.process.retainedToken.KnownFolderPath(windows.FOLDERID_LocalAppData, 0)
	// An elevated native acceptance run must prove this retained token actually
	// has usable folder-query rights. Do not mask rights failure with a service-
	// user lookup, token rights workaround, or a skipped assertion.
	if want && (locationErr != nil || expected == "") {
		t.Fatal("original operator token cannot resolve LocalAppData", locationErr)
	}
	callbackFailure := errors.New("inert location callback fault")
	err := f.cap.WithMaintenanceOperatorLocation(f.ctx, identity.SID, func(actual PeerIdentity, location string) error {
		used = true
		if actual != identity || !strings.EqualFold(location, expected) || location == "" {
			t.Fatal("location did not originate from same original operator token")
		}
		if err := maintenanceNoThreadImpersonation(); err != nil {
			t.Fatal("location callback inherited thread impersonation", err)
		}
		return callbackFailure
	})
	if want && (!used || !errors.Is(err, callbackFailure)) {
		t.Fatal("native operator location did not preserve callback outcome", err)
	}
	if !want && (used || err == nil) {
		t.Fatal("filtered/non-admin peer received operator location", err)
	}
	f.cap.Close()
	used = false
	if err = f.cap.WithMaintenanceOperatorLocation(f.ctx, identity.SID, func(PeerIdentity, string) error { used = true; return nil }); !errors.Is(err, ErrPeerRevoked) || used {
		t.Fatal("closed capability supplied operator location", err)
	}
}
