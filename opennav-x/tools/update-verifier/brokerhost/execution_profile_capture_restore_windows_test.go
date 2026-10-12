package brokerhost

import (
	"bytes"
	"context"
	"errors"
	"example.com/opennav-update-verifier/brokerstore"
	"golang.org/x/sys/windows"
	"os"
	"path/filepath"
	"runtime"
	"strings"
	"testing"
)

// Inert lower-level restore fixtures use ONLY temporary current-user trees and
// private fake child-proof/journal boundaries. Actual native RW/ID/sharing,
// atomic replacement, original archive reads, writes/sync and metadata readback
// are exercised. No service/profile/application/process/output is touched.
func nativeProfileRestoreFixture(t *testing.T) (string, *ExecutionProfileCapture, *memoryProfileCaptureArchive, *memoryProfileRestoreAuthority) {
	t.Helper()
	root := t.TempDir()
	for name, raw := range map[string][]byte{"opencpn.ini": []byte(restoreINIOriginal), "opencpn.log": []byte("original log\n"), "navobj.xml": []byte("precious route and voyage\x00\xff")} {
		if err := os.WriteFile(filepath.Join(root, name), raw, 0600); err != nil {
			t.Fatal(err)
		}
	}
	source, err := captureProfileTreePath(context.Background(), root, false)
	if err != nil {
		t.Fatal(err)
	}
	archive := &memoryProfileCaptureArchive{}
	p, err := completeProfileCapture(context.Background(), source, archive, brokerstore.ExecutionJournalSnapshot{ClientSID: "S-1-5-21-1-2-3-1001", ExecutionID: strings.Repeat("a", 32), Generation: strings.Repeat("b", 32), CompletionSHA256: strings.Repeat("c", 64)})
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { _ = p.Close() })
	if err = p.ReleaseForQualification(context.Background()); err != nil {
		t.Fatal(err)
	}
	stage := filepath.Join(root, "wx-flush-temporary")
	if err = os.WriteFile(stage, []byte(restoreINICurrent), 0600); err != nil {
		t.Fatal(err)
	}
	from, _ := windows.UTF16PtrFromString(stage)
	to, _ := windows.UTF16PtrFromString(filepath.Join(root, "opencpn.ini"))
	if err = windows.MoveFileEx(from, to, windows.MOVEFILE_REPLACE_EXISTING|windows.MOVEFILE_WRITE_THROUGH); err != nil {
		t.Fatal(err)
	}
	f, err := os.OpenFile(filepath.Join(root, "opencpn.log"), os.O_WRONLY|os.O_APPEND, 0600)
	if err != nil {
		t.Fatal(err)
	}
	_, writeErr := f.WriteString("qualification startup/close\n")
	if err = errors.Join(writeErr, f.Close()); err != nil {
		t.Fatal(err)
	}
	events := []string{}
	return root, p, archive, &memoryProfileRestoreAuthority{events: &events}
}
func TestNativeExecutionProfileRestoreOriginalBytesAndMetadata(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	root, p, archive, a := nativeProfileRestoreFixture(t)
	originalArchive := bytes.Clone(archive.raw)
	a.onIntent = func() {
		if err := os.WriteFile(filepath.Join(root, "opencpn.ini"), []byte("competing writer"), 0600); !errors.Is(err, windows.ERROR_SHARING_VIOLATION) {
			t.Error("restore current RW custody admitted writer", err)
		}
		if err := os.Remove(filepath.Join(root, "opencpn.log")); !errors.Is(err, windows.ERROR_SHARING_VIOLATION) {
			t.Error("restore custody admitted delete", err)
		}
	}
	digest, err := p.restoreOriginalNative(context.Background(), a)
	if err != nil {
		t.Fatal(err)
	}
	if digest != profileTreeDigest(p.state.manifest.Entries) {
		t.Fatal("restored full-tree digest differs")
	}
	for name, expected := range map[string][]byte{"opencpn.ini": []byte(restoreINIOriginal), "opencpn.log": []byte("original log\n"), "navobj.xml": []byte("precious route and voyage\x00\xff")} {
		raw, err := os.ReadFile(filepath.Join(root, name))
		if err != nil || !bytes.Equal(raw, expected) {
			t.Fatal("original native bytes lost", name, err)
		}
	}
	if !bytes.Equal(archive.raw, originalArchive) {
		t.Fatal("backup overwritten")
	}
	if err = p.VerifyRestoredOriginal(context.Background()); err != nil {
		t.Fatal("native metadata/original inspection after apply", err)
	}
	if err = p.Close(); err != nil {
		t.Fatal(err)
	}
}
func TestNativeExecutionProfileRestoreUnreviewedChangesNeverOverwritten(t *testing.T) {
	for _, kind := range []string{"navigation", "connection", "pilot", "extra", "log-rewrite", "log-rotation"} {
		t.Run(kind, func(t *testing.T) {
			runtime.LockOSThread()
			defer runtime.UnlockOSThread()
			root, p, archive, a := nativeProfileRestoreFixture(t)
			switch kind {
			case "navigation":
				if err := os.WriteFile(filepath.Join(root, "navobj.xml"), []byte("new voyage"), 0600); err != nil {
					t.Fatal(err)
				}
			case "connection", "pilot":
				raw := []byte(restoreINICurrent)
				if kind == "connection" {
					raw = bytes.ReplaceAll(raw, []byte("COM8=OUTPUT"), []byte("COM8=INPUT"))
				} else {
					raw = bytes.ReplaceAll(raw, []byte("Pilot=enabled"), []byte("Pilot=disabled"))
				}
				if err := os.WriteFile(filepath.Join(root, "opencpn.ini"), raw, 0600); err != nil {
					t.Fatal(err)
				}
			case "extra":
				if err := os.WriteFile(filepath.Join(root, "new-voyage.gpx"), []byte("new voyage"), 0600); err != nil {
					t.Fatal(err)
				}
			case "log-rewrite":
				if err := os.WriteFile(filepath.Join(root, "opencpn.log"), []byte("rewritten operational log"), 0600); err != nil {
					t.Fatal(err)
				}
			case "log-rotation":
				if err := os.Rename(filepath.Join(root, "opencpn.log"), filepath.Join(root, "opencpn.log.log")); err != nil {
					t.Fatal(err)
				}
			}
			before, err := os.ReadFile(filepath.Join(root, "opencpn.ini"))
			if err != nil {
				t.Fatal(err)
			}
			originalArchive := bytes.Clone(archive.raw)
			if _, err = p.restoreOriginalNative(context.Background(), a); !errors.Is(err, ErrExecutionProfileRestoreReviewRequired) {
				t.Fatal("unreviewed native change accepted", err)
			}
			after, err := os.ReadFile(filepath.Join(root, "opencpn.ini"))
			if err != nil || !bytes.Equal(before, after) || !bytes.Equal(originalArchive, archive.raw) || a.intent.Schema != 0 {
				t.Fatal("refusal erased original or current bytes", err)
			}
		})
	}
}
func TestNativeExecutionProfileRestorePartialEffectRetainsOriginal(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	root, p, archive, a := nativeProfileRestoreFixture(t)
	originalArchive := bytes.Clone(archive.raw)
	armed := false
	a.onIntent = func() { armed = true }
	a.onValidate = func() error {
		if !armed {
			return nil
		}
		raw, err := os.ReadFile(filepath.Join(root, "opencpn.ini"))
		if err != nil {
			return err
		}
		if bytes.Equal(raw, []byte(restoreINIOriginal)) {
			return profileCaptureTestFault
		}
		return nil
	}
	if _, err := p.restoreOriginalNative(context.Background(), a); !errors.Is(err, ErrExecutionProfileRestoreUncertain) || !errors.Is(err, profileCaptureTestFault) {
		t.Fatal("postwrite uncertainty lost", err)
	}
	if !bytes.Equal(archive.raw, originalArchive) {
		t.Fatal("original recovery archive changed")
	}
	if _, err := p.restoreOriginalNative(context.Background(), a); err == nil {
		t.Fatal("partial native restore retried")
	}
	raw, err := os.ReadFile(filepath.Join(root, "opencpn.log"))
	if err != nil || bytes.Equal(raw, []byte("original log\n")) {
		t.Fatal("fault did not stop partial effects", err)
	}
	first := p.Close()
	if !errors.Is(first, ErrExecutionProfileRestoreUncertain) || p.Close() != first {
		t.Fatal("cleanup did not retain exact uncertainty", first)
	}
}
