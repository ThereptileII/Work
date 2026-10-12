package brokerpipe

import (
	"runtime"
	"testing"

	"golang.org/x/sys/windows"
)

func TestNativeMaintenanceFolderTokenUsesExactOriginalPrimaryToken(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	f := actualNativePeerFixture(t)
	identity := nativePeerEvidence(f.process)
	want := currentProcessHasEnabledAdmin(t) && windows.GetCurrentProcessToken().IsElevated()
	token, err := openMaintenanceFolderToken(f.backend, identity, identity.SID)
	if !want {
		if token != 0 || err == nil {
			if token != 0 {
				token.Close()
			}
			t.Fatal("filtered/non-admin primary token gained maintenance folder access")
		}
		return
	}
	if err != nil || token == 0 {
		t.Fatal("documented operator folder rights refused", err)
	}
	defer token.Close()
	original, err := maintenancePrimaryToken(f.process.retainedToken, identity.SID, identity)
	if err != nil {
		t.Fatal(err)
	}
	actual, err := maintenancePrimaryToken(token, identity.SID, identity)
	if err != nil || actual != original {
		t.Fatal("folder token is not exact original TokenID/logon", err)
	}
	// Opening the owned token requested named QUERY|IMPERSONATE rights from the
	// retained original process. Actually query the known folder through that
	// handle, with no service-user fallback or token duplication/impersonation.
	location, err := token.KnownFolderPath(windows.FOLDERID_LocalAppData, 0)
	if err != nil || location == "" {
		t.Fatal("owned documented-rights token cannot resolve LocalAppData", err)
	}
	if err = maintenanceNoThreadImpersonation(); err != nil {
		t.Fatal("folder token helper changed thread identity", err)
	}
	if f.process.retainedToken == 0 || f.process.handle == 0 {
		t.Fatal("original operator custody consumed")
	}
}

func TestNativeMaintenanceFolderTokenRefusesLostOrDifferentOriginalCustody(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	for _, kind := range []string{"wrong-sid", "wrong-creation", "closed-process", "closed-token"} {
		t.Run(kind, func(t *testing.T) {
			f := actualNativePeerFixture(t)
			identity := nativePeerEvidence(f.process)
			expected := identity.SID
			switch kind {
			case "wrong-sid":
				expected += "-1"
			case "wrong-creation":
				identity.ProcessCreation++
			case "closed-process":
				if err := windows.CloseHandle(f.process.handle); err != nil {
					t.Fatal(err)
				}
				f.process.handle = 0
			case "closed-token":
				if err := f.process.retainedToken.Close(); err != nil {
					t.Fatal(err)
				}
				f.process.retainedToken = 0
			}
			token, err := openMaintenanceFolderToken(f.backend, identity, expected)
			if token != 0 || err == nil {
				if token != 0 {
					token.Close()
				}
				t.Fatal("lost/different original custody reopened or trusted", err)
			}
		})
	}
}
