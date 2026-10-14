//go:build windows

package brokerenroll

import (
	"bytes"
	"context"
	"crypto/sha256"
	"encoding/hex"
	"errors"
	"io"
	"os"
	"path/filepath"
	"strings"
	"testing"

	"example.com/opennav-update-verifier/releasepolicy"
	"golang.org/x/sys/windows"
)

// This inert fixture supplies the test binary as a pretend signed-policy
// artifact to private seams only. It grants no enrollment/installation approval
// and writes only beneath t.TempDir, never fixed machine code, SCM or profiles.
func maintenanceNativeArtifact(t *testing.T) releasepolicy.Artifact {
	t.Helper()
	path, err := os.Executable()
	if err != nil {
		t.Fatal(err)
	}
	file, err := os.Open(path)
	if err != nil {
		t.Fatal(err)
	}
	defer file.Close()
	hash := sha256.New()
	size, err := io.Copy(hash, file)
	if err != nil {
		t.Fatal(err)
	}
	return releasepolicy.Artifact{Path: "inert-test-policy/" + maintenanceImageName, SHA256: hex.EncodeToString(hash.Sum(nil)), Bytes: size}
}

func TestNativeStartupMaintenanceImageSourceBindsActualRunningOriginal(t *testing.T) {
	artifact := maintenanceNativeArtifact(t)
	source, err := openMaintenanceImageNative(context.Background(), artifact)
	if err != nil {
		t.Fatal("original running image custody", err)
	}
	n := source.(*nativeMaintenanceImage)
	if n.pin.length != artifact.Bytes || n.pin.id.ID == [16]byte{} || n.sha != artifact.SHA256 {
		t.Fatal("full original process/source identity missing")
	}
	if err := source.Check(context.Background()); err != nil {
		t.Fatal(err)
	}
	if err := source.Close(); err != nil {
		t.Fatal(err)
	}
	if source.Check(context.Background()) == nil {
		t.Fatal("closed source retained authority")
	}
	for _, fault := range []string{"hash", "length", "artifact"} {
		t.Run(fault, func(t *testing.T) {
			bad := artifact
			switch fault {
			case "hash":
				bad.SHA256 = strings.Repeat("0", 64)
			case "length":
				bad.Bytes++
			case "artifact":
				bad.Path = "inert-test-policy/" + enrollmentImageName
			}
			opened, err := openMaintenanceImageNative(context.Background(), bad)
			if opened != nil {
				defer opened.Close()
			}
			if err == nil || opened != nil {
				t.Fatal("source substituted for approved process image", err)
			}
		})
	}
}

func TestNativeStartupMaintenanceImageOriginalAliasKeepsStrictNativeIdentity(t *testing.T) {
	path, err := maintenanceCurrentImagePath(0)
	if err != nil {
		t.Fatal(err)
	}
	pointer, err := windows.UTF16PtrFromString(path)
	if err != nil {
		t.Fatal(err)
	}
	short := make([]uint16, 32768)
	count, err := windows.GetShortPathName(pointer, &short[0], uint32(len(short)))
	if err != nil || count == 0 || count >= uint32(len(short)) {
		t.Fatal("independent process short path unavailable", err)
	}
	alias := windows.UTF16ToString(short[:count])
	// Volumes may not generate 8.3 names. An explicit case alias still exercises
	// normalization from the original handle without requiring machine settings.
	alias = strings.ToUpper(alias)
	expected, err := windows.CreateFile(pointer, windows.FILE_READ_ATTRIBUTES|windows.READ_CONTROL, windows.FILE_SHARE_READ, nil, windows.OPEN_EXISTING, windows.FILE_FLAG_OPEN_REPARSE_POINT, 0)
	if err != nil {
		t.Fatal(err)
	}
	defer windows.CloseHandle(expected)
	info, id, err := enrollmentInfo(expected)
	if err != nil {
		t.Fatal(err)
	}
	physical, err := enrollmentPhysicalPath(expected)
	if err != nil {
		t.Fatal(err)
	}
	dos, err := maintenanceOriginalDOSPath(expected)
	if err != nil {
		t.Fatal(err)
	}
	if alias == dos {
		t.Fatal("alias fixture did not change original path spelling")
	}
	user, err := windows.GetCurrentProcessToken().GetTokenUser()
	if err != nil {
		t.Fatal(err)
	}
	files := &nativeEnrollmentFiles{}
	defer files.Close()
	pin, err := maintenanceHoldOriginalImage(files, alias, user.User.Sid.String())
	if err != nil {
		t.Fatal("original image alias failed canonical retention", err)
	}
	if pin.id != id || pin.info.CreationTime != info.CreationTime || pin.physical != physical || pin.dos != dos {
		t.Fatal("canonical spelling lost original native file identity")
	}
	foreign := filepath.Join(t.TempDir(), "foreign-original.exe")
	if err := os.WriteFile(foreign, []byte("inert different file"), 0600); err != nil {
		t.Fatal(err)
	}
	original := *pin
	for _, field := range []string{"full ID", "creation", "physical name", "length", "DOS replacement"} {
		t.Run(field, func(t *testing.T) {
			*pin = original
			switch field {
			case "full ID":
				pin.id.ID[0] ^= 1
			case "creation":
				pin.info.CreationTime.LowDateTime ^= 1
			case "physical name":
				pin.physical += "-foreign"
			case "length":
				pin.length++
			case "DOS replacement":
				pin.dos = foreign
			}
			if err := pin.check(); err == nil {
				t.Fatal("changed original identity accepted")
			}
			*pin = original
			if err := pin.check(); err != nil {
				t.Fatal("refused identity changed the original", err)
			}
		})
	}
}

func TestNativeStartupMaintenanceImagePublicationRetainsOriginalAndRefusesExistingOrRetry(t *testing.T) {
	for _, existing := range []bool{false, true} {
		name := "fresh"
		if existing {
			name = "unknown existing"
		}
		t.Run(name, func(t *testing.T) {
			artifact := maintenanceNativeArtifact(t)
			source, err := openMaintenanceImageNative(context.Background(), artifact)
			if err != nil {
				t.Fatal(err)
			}
			defer source.Close()
			files := &nativeEnrollmentFiles{}
			defer files.Close()
			user, err := windows.GetCurrentProcessToken().GetTokenUser()
			if err != nil {
				t.Fatal(err)
			}
			// Retain the fixture directory first; TempDir may contain RUNNER~1.
			// Obtain its normalized spelling from that original directory handle.
			ancestor, err := files.pinDirectory(t.TempDir(), "", "", nil, false, user.User.Sid.String())
			if err != nil {
				t.Fatal(err)
			}
			ancestor.dos, err = maintenanceOriginalDOSPath(ancestor.handle)
			if err != nil {
				t.Fatal(err)
			}
			if err := ancestor.check(); err != nil {
				t.Fatal(err)
			}
			broker, err := enrollmentBrokerSID()
			if err != nil {
				t.Fatal(err)
			}
			codeSD, err := enrollmentCodeDescriptor(broker, true)
			if err != nil {
				t.Fatal(err)
			}
			parent, err := files.newDirectory(ancestor, "inert-maintenance-code", codeSD, "")
			if err != nil {
				t.Fatal(err)
			}
			files.maintenanceCodeParent = parent
			files.imagePath = filepath.Join(parent.dos, enrollmentImageName)
			path := filepath.Join(parent.dos, maintenanceImageName)
			unknown := []byte("unknown original must survive refusal")
			if existing {
				if err := os.WriteFile(path, unknown, 0600); err != nil {
					t.Fatal(err)
				}
			}
			err = publishMaintenanceImageNative(context.Background(), files, source, artifact)
			if existing {
				if err == nil || files.maintenanceHelper != nil {
					t.Fatal("unknown existing code was adopted")
				}
				raw, readErr := os.ReadFile(path)
				if readErr != nil || !bytes.Equal(raw, unknown) {
					t.Fatal("unknown existing code changed", readErr)
				}
			} else {
				if err != nil || files.maintenanceHelper == nil || files.maintenanceHelperSHA != artifact.SHA256 {
					t.Fatal("original helper publication failed", err)
				}
				if err := errors.Join(files.Check(), startupHashPin(context.Background(), files.maintenanceHelper, artifact.SHA256)); err != nil {
					t.Fatal(err)
				}
				pointer, _ := windows.UTF16PtrFromString(path)
				writer, writeErr := windows.CreateFile(pointer, windows.GENERIC_WRITE, windows.FILE_SHARE_READ|windows.FILE_SHARE_WRITE|windows.FILE_SHARE_DELETE, nil, windows.OPEN_EXISTING, windows.FILE_FLAG_OPEN_REPARSE_POINT, 0)
				if writeErr == nil {
					windows.CloseHandle(writer)
					t.Fatal("original helper did not exclude write sharing")
				}
			}
			if err := publishMaintenanceImageNative(context.Background(), files, source, artifact); err == nil {
				t.Fatal("one-shot original publication retried")
			}
		})
	}
}
