package brokerstore

import (
	"errors"
	"io"
	"os"
	"testing"
	"unsafe"

	"golang.org/x/sys/windows"
)

// These native temp fixtures use the actual low-level production lease helpers
// with the current account as the protected owner. They do not impersonate or
// provision the service, and do not qualify Open's service-principal/ProgramData
// policy or the broker's authenticated network acquisition path.
func nativeAcquisitionFixture(t *testing.T) (*nativeCandidate, *nativeAcquisition) {
	t.Helper()
	root, fileSD := storeLockFixture(t)
	user, err := windows.GetCurrentProcessToken().GetTokenUser()
	if err != nil {
		t.Fatal(err)
	}
	lock, err := acquireStoreLock(root, fileSD)
	if err != nil {
		t.Fatal(err)
	}
	n := &nativeStore{broker: user.User.Sid, root: root, pins: []*nativePin{root}, lock: lock}
	c := &nativeCandidate{store: n, directories: map[string]*nativePin{}, securityDir: root.security, securityFile: fileSD, stageID: stageA, client: user.User.Sid.String()}
	stage, err := c.newDirectory(root, stageA)
	if err != nil {
		t.Fatal(err)
	}
	c.directories[""] = stage
	b, err := c.beginAcquisition()
	if err != nil {
		t.Fatal(err)
	}
	a := b.(*nativeAcquisition)
	t.Cleanup(func() {
		// Fault/crash fixtures intentionally do not adopt or erase retained residue.
		for _, f := range a.artifacts {
			if f.file != nil {
				if err := f.file.Close(); err != nil {
					t.Error(err)
				}
				f.file = nil
			}
		}
		if err := a.closeHandles(); err != nil {
			t.Error(err)
		}
		if err := c.close(); err != nil {
			t.Error(err)
		}
		if err := windows.CloseHandle(lock.handle); err != nil {
			t.Error(err)
		}
	})
	return c, a
}
func nativeSlot(t *testing.T, a *nativeAcquisition, slot ArtifactSlot, data string) *nativeArtifact {
	t.Helper()
	spec, err := a.index.plan.spec(slot)
	if err != nil {
		t.Fatal(err)
	}
	b, err := a.create(slot, spec)
	if err != nil {
		t.Fatal(err)
	}
	f := b.(*nativeArtifact)
	if data != "" {
		if n, err := f.Write([]byte(data)); err != nil || n != len(data) {
			t.Fatalf("partial bytes: %d %v", n, err)
		}
	}
	return f
}
func abandonNativeAcquisition(t *testing.T, a *nativeAcquisition) {
	t.Helper()
	for _, f := range a.artifacts {
		if f.file != nil {
			if err := f.file.Close(); err != nil {
				t.Fatal(err)
			}
			f.file = nil
		}
	}
	if err := a.closeHandles(); err != nil {
		t.Fatal(err)
	}
}
func TestNativeAcquisitionReadOnlyHandoffAndHandleDeletion(t *testing.T) {
	_, a := nativeAcquisitionFixture(t)
	if err := a.bind(acquisitionPlan()); err != nil {
		t.Fatal(err)
	}
	f := nativeSlot(t, a, RecoverySlot, "zip")
	borrowed, err := f.seal(acquisitionPlan().Recovery)
	if err != nil {
		t.Fatal(err)
	}
	data, err := io.ReadAll(borrowed)
	if err != nil || string(data) != "zip" {
		t.Fatalf("borrowed read: %q %v", data, err)
	}
	// The actual returned handle must have no data-write access.
	if _, err = borrowed.WriteAt([]byte("bad"), 0); !errors.Is(err, windows.ERROR_ACCESS_DENIED) {
		t.Fatalf("borrowed handle has write access: %v", err)
	}
	ptr, err := windows.UTF16PtrFromString(f.Path())
	if err != nil {
		t.Fatal(err)
	}
	h, err := windows.CreateFile(ptr, windows.GENERIC_WRITE, windows.FILE_SHARE_READ|windows.FILE_SHARE_WRITE|windows.FILE_SHARE_DELETE, nil, windows.OPEN_EXISTING, windows.FILE_FLAG_OPEN_REPARSE_POINT, 0)
	if err == nil {
		windows.CloseHandle(h)
		t.Fatal("readonly custody allowed a new writer")
	}
	if !errors.Is(err, windows.ERROR_SHARING_VIOLATION) {
		t.Fatalf("expected write-sharing refusal: %v", err)
	}
	if err = f.close(); err != nil {
		t.Fatal(err)
	}
	if err = f.close(); err != nil {
		t.Fatal("idempotent cleanup", err)
	}
	if err = artifactAbsent(a.directory, slotFilename(RecoverySlot)); err != nil {
		t.Fatal(err)
	}
	if err = a.close(); err != nil {
		t.Fatal(err)
	}
}
func TestNativeAcquisitionStartupReconcilesIndexedCrashBoundaries(t *testing.T) {
	for _, boundary := range []string{"owned-partial", "delete-intent", "deleted-before-completion"} {
		t.Run(boundary, func(t *testing.T) {
			c, a := nativeAcquisitionFixture(t)
			if err := a.bind(acquisitionPlan()); err != nil {
				t.Fatal(err)
			}
			f := nativeSlot(t, a, RecoverySlot, "z")
			if boundary != "owned-partial" {
				if err := a.append("deleting", RecoverySlot, &f.identity, nil); err != nil {
					t.Fatal(err)
				}
			}
			if boundary == "deleted-before-completion" {
				value := byte(1)
				if err := windows.SetFileInformationByHandle(windows.Handle(f.file.Fd()), windows.FileDispositionInfo, &value, 1); err != nil {
					t.Fatal(err)
				}
			}
			abandonNativeAcquisition(t, a)
			recovered, err := openRecoveryAcquisition(c)
			if err != nil {
				t.Fatal(err)
			}
			t.Cleanup(func() {
				if err := recovered.closeHandles(); err != nil {
					t.Error(err)
				}
			})
			if err = reconcileAcquisition(recovered); err != nil {
				t.Fatal(err)
			}
			if !recovered.index.complete || recovered.index.states[RecoverySlot] != "deleted" {
				t.Fatal("actual cleanup not durably completed")
			}
			if err = artifactAbsent(recovered.directory, slotFilename(RecoverySlot)); err != nil {
				t.Fatal(err)
			}
			if err = reconcileAcquisition(recovered); err != nil {
				t.Fatal("completed tombstone should be inert", err)
			}
		})
	}
}
func TestNativeAcquisitionStartupRefusesUnindexedAndMismatchedResidue(t *testing.T) {
	for _, kind := range []string{"unindexed", "creation-gap", "second-slot-identity", "wrong-acl", "hardlink", "unknown-extra"} {
		t.Run(kind, func(t *testing.T) {
			c, a := nativeAcquisitionFixture(t)
			if err := a.bind(acquisitionPlan()); err != nil {
				t.Fatal(err)
			}
			var recovery *nativeArtifact
			if kind == "unindexed" || kind == "creation-gap" {
				if kind == "creation-gap" {
					if err := a.append("creating", RecoverySlot, nil, nil); err != nil {
						t.Fatal(err)
					}
				}
				file, err := openAcquisitionFile(a.directory, slotFilename(RecoverySlot), a.fileSD, windows.GENERIC_READ|windows.GENERIC_WRITE, windows.FILE_SHARE_READ, windows.CREATE_NEW)
				if err != nil {
					t.Fatal(err)
				}
				if _, err = file.Write([]byte("z")); err != nil {
					t.Fatal(err)
				}
				if err = file.Close(); err != nil {
					t.Fatal(err)
				}
			} else {
				recovery = nativeSlot(t, a, RecoverySlot, "z")
			}
			var second *nativeArtifact
			if kind == "second-slot-identity" {
				second = nativeSlot(t, a, InstallerSlot, "e")
			}
			name := a.directory.name
			abandonNativeAcquisition(t, a)
			switch kind {
			case "second-slot-identity":
				if err := os.Rename(second.Path(), physical(c.directories[""].name+`\retained-original.exe`)); err != nil {
					t.Fatal(err)
				}
				dir, err := openPin(physical(name), name, a.directorySecurityForTest(t))
				if err != nil {
					t.Fatal(err)
				}
				replacement, err := openAcquisitionFile(dir, slotFilename(InstallerSlot), a.fileSD, windows.GENERIC_READ|windows.GENERIC_WRITE, windows.FILE_SHARE_READ, windows.CREATE_NEW)
				if err != nil {
					t.Fatal(err)
				}
				if _, err = replacement.Write([]byte("e")); err != nil {
					t.Fatal(err)
				}
				if err = replacement.Close(); err != nil {
					t.Fatal(err)
				}
				windows.CloseHandle(dir.handle)
			case "wrong-acl":
				user, _ := windows.GetCurrentProcessToken().GetTokenUser()
				altered, err := windows.SecurityDescriptorFromString("O:" + user.User.Sid.String() + "D:P(A;;FA;;;" + user.User.Sid.String() + ")")
				if err != nil {
					t.Fatal(err)
				}
				dacl, _, err := altered.DACL()
				if err != nil {
					t.Fatal(err)
				}
				if err = windows.SetNamedSecurityInfo(recovery.Path(), windows.SE_FILE_OBJECT, windows.DACL_SECURITY_INFORMATION|windows.PROTECTED_DACL_SECURITY_INFORMATION, nil, nil, dacl, nil); err != nil {
					t.Fatal(err)
				}
			case "hardlink":
				if err := os.Link(recovery.Path(), physical(c.directories[""].name+`\retained-link`)); err != nil {
					t.Fatal(err)
				}
			case "unknown-extra":
				dir, err := openPin(physical(name), name, a.directorySecurityForTest(t))
				if err != nil {
					t.Fatal(err)
				}
				extra, err := openAcquisitionFile(dir, "unindexed.bin", a.fileSD, windows.GENERIC_READ|windows.GENERIC_WRITE, windows.FILE_SHARE_READ, windows.CREATE_NEW)
				if err != nil {
					t.Fatal(err)
				}
				extra.Close()
				windows.CloseHandle(dir.handle)
			}
			recovered, err := openRecoveryAcquisition(c)
			if err != nil {
				t.Fatal(err)
			}
			t.Cleanup(func() {
				if err := recovered.closeHandles(); err != nil {
					t.Error(err)
				}
			})
			if err = reconcileAcquisition(recovered); err == nil {
				t.Fatal("unsafe residue accepted")
			}
			if _, err = os.Stat(physical(name + `\` + slotFilename(RecoverySlot))); err != nil {
				t.Fatal("refusal deleted recovery artifact", err)
			}
			if len(recovered.index.events) != len(a.index.events) {
				t.Fatal("refusal changed durable acquisition journal")
			}
		})
	}
}
func (a *nativeAcquisition) directorySecurityForTest(t *testing.T) *windows.SECURITY_DESCRIPTOR {
	t.Helper()
	sd, err := descriptor(a.candidate.store.broker, nil, true)
	if err != nil {
		t.Fatal(err)
	}
	return sd
}
func TestNativeAcquisitionWrongBytesAndCleanupFailureRetained(t *testing.T) {
	_, a := nativeAcquisitionFixture(t)
	if err := a.bind(acquisitionPlan()); err != nil {
		t.Fatal(err)
	}
	f := nativeSlot(t, a, RecoverySlot, "bad")
	if _, err := f.seal(acquisitionPlan().Recovery); err == nil {
		t.Fatal("hash mismatch accepted")
	}
	// Inject premature closure of the owned writer without retaining a stale
	// raw handle. The resulting uncertainty must survive repeated owner drains.
	if err := f.file.Close(); err != nil {
		t.Fatal(err)
	}
	first := f.close()
	if first == nil {
		t.Fatal("closed native custody was treated as successful cleanup")
	}
	if again := f.close(); again != first {
		t.Fatal("original cleanup uncertainty not retained")
	}
	if err := a.close(); err == nil {
		t.Fatal("lease dropped failed artifact cleanup")
	}
	if _, err := os.Stat(f.Path()); err != nil {
		t.Fatal("uncertain artifact was pathname-deleted", err)
	}
}

// Keep compiler assertions local to the real native FILE_ID_INFO ABI: Windows
// x86/x64 both use an eight-byte volume serial followed by a 128-bit file ID.
func TestNativeAcquisitionFileIDShape(t *testing.T) {
	var native struct {
		Volume uint64
		ID     [16]byte
	}
	if unsafe.Sizeof(native) != 24 {
		t.Fatal("FILE_ID_INFO ABI changed")
	}
}
