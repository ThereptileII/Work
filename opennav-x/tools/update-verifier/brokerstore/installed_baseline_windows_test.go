package brokerstore

import (
	"context"
	"errors"
	"os"
	"path/filepath"
	"runtime"
	"strings"
	"testing"
	"unsafe"

	"golang.org/x/sys/windows"
)

// These disposable fixtures exercise the actual native lower file/directory,
// publication and retained reader mechanics under the current user. They never
// provision/impersonate a broker or qualify actual fixed ProgramData creation,
// operator approval, a running installation or execution of any payload.
type nativeBaselineFixtureStore struct {
	*nativeStore
	sid *windows.SID
}

func (s *nativeBaselineFixtureStore) prepareInstalledBaseline(client, generation, approved string) (installedBaselineBackend, error) {
	return prepareNativeInstalledBaseline(s.nativeStore, s.sid, client, generation, approved)
}
func (s *nativeBaselineFixtureStore) openInstalledBaseline(ctx context.Context, client, generation, approved string) (installedBaselineBackend, []byte, error) {
	b, err := prepareNativeInstalledBaseline(s.nativeStore, s.sid, client, generation, approved)
	if err != nil {
		return nil, nil, err
	}
	return openNativeInstalledBaseline(ctx, b)
}
func nativeBaselineFixture(t *testing.T) (*Store, *InstalledBaseline, *nativeInstalledBaseline, baselineFixtureData) {
	t.Helper()
	n, registration := nativeRegistrationFixture(t)
	if err := beginRegistration(context.Background(), registration); err != nil {
		t.Fatal(err)
	}
	s := &Store{backend: &nativeBaselineFixtureStore{nativeStore: n, sid: n.broker}}
	r, err := s.BeginInstalledBaselineFresh(context.Background(), registration.client, stageA, registrationTestApproval)
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { s.Close() })
	return s, r, r.backend.(*nativeInstalledBaseline), baselineData(t)
}
func nativeAttemptFile(t *testing.T, path string, access uint32) error {
	t.Helper()
	pointer, err := windows.UTF16PtrFromString(path)
	if err != nil {
		t.Fatal(err)
	}
	h, err := windows.CreateFile(pointer, access, windows.FILE_SHARE_READ|windows.FILE_SHARE_WRITE|windows.FILE_SHARE_DELETE, nil, windows.OPEN_EXISTING, windows.FILE_FLAG_OPEN_REPARSE_POINT, 0)
	if err == nil {
		err = windows.CloseHandle(h)
	}
	return err
}
func TestNativeInstalledBaselineFreshCompletionRetainsExactBytesAndReadCustody(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	s, r, b, f := nativeBaselineFixture(t)
	f = baselineEngineData(t)
	if _, err := os.Stat(filepath.Join(b.dos, baselineRecordFilename)); !errors.Is(err, os.ErrNotExist) {
		t.Fatal("premature completion evidence:", err)
	}
	writeBaseline(t, r, f)
	if err := r.Complete(context.Background(), f.identity); err != nil {
		t.Fatal(err)
	}
	if err := r.Validate(); err != nil {
		t.Fatal(err)
	}
	if _, err := s.OpenInstalledBaseline(context.Background(), b.client, stageA, registrationTestApproval, f.expected); !errors.Is(err, windows.ERROR_SHARING_VIOLATION) {
		t.Fatal("live writer unexpectedly reopenable:", err)
	}
	for _, name := range []string{"app/opencpn.exe", "FILE_SHA256.json", "installer/package.json", "installer/engine/Lifecycle.ps1", "installer/engine/UpdateSupervisor.ps1", "installer/engine/UpdateTransaction.ps1"} {
		for _, access := range []uint32{windows.GENERIC_WRITE, windows.DELETE} {
			if err := nativeAttemptFile(t, filepath.Join(b.dos, filepath.FromSlash(name)), access); !errors.Is(err, windows.ERROR_SHARING_VIOLATION) {
				t.Fatalf("writer custody %s access %d: %v", name, access, err)
			}
		}
	}
	if err := r.Close(); err != nil {
		t.Fatal(err)
	}
	opened, err := s.OpenInstalledBaseline(context.Background(), b.client, stageA, registrationTestApproval, f.expected)
	if err != nil {
		t.Fatal(err)
	}
	if err := opened.Validate(); err != nil {
		t.Fatal(err)
	}
	for _, name := range []string{"app/opencpn.exe", "FILE_SHA256.json", "installer/package.json", "installer/engine/Lifecycle.ps1", "installer/engine/UpdateSupervisor.ps1", "installer/engine/UpdateTransaction.ps1"} {
		path := filepath.Join(b.dos, filepath.FromSlash(name))
		if err := nativeAttemptFile(t, path, windows.GENERIC_READ); err != nil {
			t.Fatal("readonly sibling read denied:", err)
		}
		for _, access := range []uint32{windows.GENERIC_WRITE, windows.DELETE} {
			if err := nativeAttemptFile(t, path, access); !errors.Is(err, windows.ERROR_SHARING_VIOLATION) {
				t.Fatalf("reader custody %s access %d: %v", name, access, err)
			}
		}
	}
	if err := opened.Close(); err != nil {
		t.Fatal(err)
	}
	if err := nativeAttemptFile(t, filepath.Join(b.dos, "app", "opencpn.exe"), windows.GENERIC_WRITE); err != nil {
		t.Fatal("reader leaked write exclusion:", err)
	}
}

func TestNativeInstalledBaselineEngineHelpersExactNamespace(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	for _, scenario := range []string{"missing", "tampered", "extra", "alias", "hardlink"} {
		t.Run(scenario, func(t *testing.T) {
			s, r, b, _ := nativeBaselineFixture(t)
			f := baselineEngineData(t)
			if scenario == "missing" {
				delete(f.payload, "installer/engine/UpdateTransaction.ps1")
			}
			writeBaseline(t, r, f)
			err := r.Complete(context.Background(), f.identity)
			if scenario == "missing" {
				if err == nil {
					t.Fatal("missing native helper completed")
				}
				return
			}
			if err != nil {
				t.Fatal(err)
			}
			if err := r.Close(); err != nil {
				t.Fatal(err)
			}
			file := filepath.Join(b.dos, "installer", "engine", "Lifecycle.ps1")
			switch scenario {
			case "tampered":
				if err := os.WriteFile(file, []byte("changed"), 0600); err != nil {
					t.Fatal(err)
				}
			case "extra":
				if err := os.WriteFile(filepath.Join(filepath.Dir(file), "extra.ps1"), []byte("extra"), 0600); err != nil {
					t.Fatal(err)
				}
			case "alias":
				if err := os.Rename(file, filepath.Join(filepath.Dir(file), "lifecycle.ps1")); err != nil {
					t.Fatal(err)
				}
			case "hardlink":
				if err := os.Link(file, filepath.Join(filepath.Dir(b.dos), "outside-helper-link")); err != nil {
					t.Fatal(err)
				}
			}
			if opened, err := s.OpenInstalledBaseline(context.Background(), b.client, stageA, registrationTestApproval, f.expected); err == nil {
				opened.Close()
				t.Fatal("changed native helper namespace accepted")
			}
		})
	}
}
func TestNativeInstalledBaselineReservationRejectsPartialAndRetry(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	s, r, b, f := nativeBaselineFixture(t)
	if err := r.WriteFile(context.Background(), "app/partial", strings.NewReader("partial"), 7, digest("wrong")); err == nil {
		t.Fatal("invalid payload accepted")
	}
	if err := r.Close(); err == nil {
		t.Fatal("lost write failure")
	}
	if _, err := s.OpenInstalledBaseline(context.Background(), b.client, stageA, registrationTestApproval, f.expected); err == nil {
		t.Fatal("partial baseline adopted")
	}
	if _, err := s.BeginInstalledBaselineFresh(context.Background(), b.client, stageA, registrationTestApproval); !errors.Is(err, windows.ERROR_FILE_EXISTS) && !errors.Is(err, windows.ERROR_ALREADY_EXISTS) {
		t.Fatal("permanent reservation retry:", err)
	}
	if raw, err := os.ReadFile(filepath.Join(b.dos, "app", "partial")); err != nil || string(raw) != "partial" {
		t.Fatal("partial evidence removed:", string(raw), err)
	}
}
func TestNativeInstalledBaselineReaderRejectsNamespaceAdditionReplacementAndHardlink(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	for _, kind := range []string{"extra-file", "extra-directory", "replaced-file", "replaced-directory", "hardlink", "descriptor"} {
		t.Run(kind, func(t *testing.T) {
			s, r, b, f := nativeBaselineFixture(t)
			writeBaseline(t, r, f)
			if err := r.Complete(context.Background(), f.identity); err != nil {
				t.Fatal(err)
			}
			if err := r.Close(); err != nil {
				t.Fatal(err)
			}
			file := filepath.Join(b.dos, "app", "opencpn.exe")
			switch kind {
			case "extra-file":
				if err := os.WriteFile(filepath.Join(b.dos, "unexpected"), []byte("extra"), 0600); err != nil {
					t.Fatal(err)
				}
			case "extra-directory":
				if err := os.Mkdir(filepath.Join(b.dos, "unexpected"), 0700); err != nil {
					t.Fatal(err)
				}
			case "replaced-file":
				if err := os.Rename(file, file+".old"); err != nil {
					t.Fatal(err)
				}
				pointer, _ := windows.UTF16PtrFromString(file)
				sd := b.tree.securityFile
				attr := windows.SecurityAttributes{Length: uint32(unsafe.Sizeof(windows.SecurityAttributes{})), SecurityDescriptor: sd}
				h, err := windows.CreateFile(pointer, windows.GENERIC_WRITE, 0, &attr, windows.CREATE_NEW, 0, 0)
				if err != nil {
					t.Fatal(err)
				}
				replacement := os.NewFile(uintptr(h), file)
				if _, err := replacement.Write(f.payload["app/opencpn.exe"]); err != nil {
					t.Fatal(err)
				}
				if err := replacement.Close(); err != nil {
					t.Fatal(err)
				}
				if err := os.Remove(file + ".old"); err != nil {
					t.Fatal(err)
				}
			case "replaced-directory":
				docs := filepath.Join(b.dos, "docs")
				if err := os.Rename(docs, docs+".old"); err != nil {
					t.Fatal(err)
				}
				pointer, _ := windows.UTF16PtrFromString(docs)
				attr := windows.SecurityAttributes{Length: uint32(unsafe.Sizeof(windows.SecurityAttributes{})), SecurityDescriptor: b.tree.securityDir}
				if err := windows.CreateDirectory(pointer, &attr); err != nil {
					t.Fatal(err)
				}
				if err := os.Rename(filepath.Join(docs+".old", "PRODUCT_BUILD.json"), filepath.Join(docs, "PRODUCT_BUILD.json")); err != nil {
					t.Fatal(err)
				}
				if err := os.Remove(docs + ".old"); err != nil {
					t.Fatal(err)
				}
			case "hardlink":
				if err := os.Link(file, filepath.Join(filepath.Dir(b.dos), "outside-hardlink")); err != nil {
					t.Fatal(err)
				}
			case "descriptor":
				pointer, _ := windows.UTF16PtrFromString(file)
				sd, err := windows.SecurityDescriptorFromString("O:" + b.store.broker.String() + "D:P(A;;FA;;;" + b.store.broker.String() + ")(A;;FA;;;SY)(A;;FW;;;WD)")
				if err != nil {
					t.Fatal(err)
				}
				dacl, _, err := sd.DACL()
				if err != nil {
					t.Fatal(err)
				}
				if err := windows.SetNamedSecurityInfo(windows.UTF16PtrToString(pointer), windows.SE_FILE_OBJECT, windows.DACL_SECURITY_INFORMATION|windows.PROTECTED_DACL_SECURITY_INFORMATION, nil, nil, dacl, nil); err != nil {
					t.Fatal(err)
				}
			}
			if opened, err := s.OpenInstalledBaseline(context.Background(), b.client, stageA, registrationTestApproval, f.expected); err == nil {
				opened.Close()
				t.Fatal("mutated completed baseline accepted:", kind)
			}
		})
	}
}
func TestNativeInstalledBaselineDescriptorClientReadExecuteOnly(t *testing.T) {
	user, err := windows.GetCurrentProcessToken().GetTokenUser()
	if err != nil {
		t.Fatal(err)
	}
	client, err := windows.StringToSid(registrationTestSID)
	if err != nil {
		t.Fatal(err)
	}
	for _, directory := range []bool{false, true} {
		sd, err := descriptor(user.User.Sid, client, directory)
		if err != nil {
			t.Fatal(err)
		}
		acl, _, err := sd.DACL()
		if err != nil || acl.AceCount != 3 {
			t.Fatal(err)
		}
		var ace *windows.ACCESS_ALLOWED_ACE
		if err := windows.GetAce(acl, 2, &ace); err != nil {
			t.Fatal(err)
		}
		const mutation uint32 = windows.GENERIC_WRITE | windows.GENERIC_ALL | windows.DELETE | windows.WRITE_DAC | windows.WRITE_OWNER | windows.FILE_WRITE_DATA | windows.FILE_APPEND_DATA | windows.FILE_WRITE_EA | windows.FILE_WRITE_ATTRIBUTES | 0x40
		if uint32(ace.Mask)&mutation != 0 || ace.Mask&windows.FILE_GENERIC_READ != windows.FILE_GENERIC_READ || ace.Mask&windows.FILE_GENERIC_EXECUTE != windows.FILE_GENERIC_EXECUTE {
			t.Fatalf("client rights %x", ace.Mask)
		}
	}
}
func TestNativeInstalledBaselinePublicOpenRefusesOperatorConsole(t *testing.T) {
	if s, err := Open(filepath.Join(t.TempDir(), "caller-selected-root")); !errors.Is(err, ErrPrincipal) {
		if s != nil {
			s.Close()
		}
		t.Fatal("operator console accepted:", err)
	}
}
