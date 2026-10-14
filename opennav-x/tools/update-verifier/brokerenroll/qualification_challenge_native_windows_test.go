//go:build windows

package brokerenroll

import (
	"bytes"
	"context"
	"os"
	"path/filepath"
	"testing"

	"golang.org/x/sys/windows"
)

func TestNativeQualificationChallengeOriginalParentAndExclusiveDurableRecord(t *testing.T) {
	fixture, parent := freshCacheNativeParent(t)
	// The fixture creator must not supply the deny-delete observation being
	// tested: release its child pin before the real ready owner opens it.
	if err := codeReleasePin(fixture, parent); err != nil {
		t.Fatal(err)
	}
	ctx := context.Background()
	owner, err := openQualificationChallengeParentNative(ctx, parent.dos, false)
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { owner.close() })
	readyID, err := owner.identity(ctx)
	if err != nil {
		t.Fatal(err)
	}
	source, _ := windows.UTF16PtrFromString(parent.dos)
	target, _ := windows.UTF16PtrFromString(parent.dos + "-swapped")
	if err := windows.MoveFileEx(source, target, 0); err == nil {
		t.Fatal("original qualification parent could be swapped while ready")
	}
	raw := qualificationChallengeFixture()
	record, err := parseQualificationChallenge(raw)
	if err != nil {
		t.Fatal(err)
	}
	receipt, err := owner.consume(ctx, raw, record)
	if err != nil || receipt.Parent != readyID || !validQualificationIdentity(receipt.Record) || receipt.RecordSHA256 != digest(raw) {
		t.Fatal("durable consumption lost original identity", err)
	}
	path := filepath.Join(parent.dos, ".installed-b0-consumed-"+record.Challenge+".json")
	actual, err := os.ReadFile(path)
	if err != nil || !bytes.Equal(actual, raw) {
		t.Fatal("exact consumed bytes changed", err)
	}
	if _, err := owner.consume(ctx, raw, record); err == nil {
		t.Fatal("original owner consumed twice")
	}
	if err := owner.close(); err != nil {
		t.Fatal(err)
	}
	next, err := openQualificationChallengeParentNative(ctx, parent.dos, false)
	if err != nil {
		t.Fatal(err)
	}
	defer next.close()
	if _, err := next.consume(ctx, raw, record); err == nil {
		t.Fatal("restarted owner adopted a consumed challenge")
	}
	actual, err = os.ReadFile(path)
	if err != nil || !bytes.Equal(actual, raw) {
		t.Fatal("consumed fence overwritten", err)
	}
}

func TestNativeQualificationChallengeReadyEOFHasNoDurableEffect(t *testing.T) {
	fixture, parent := freshCacheNativeParent(t)
	if err := codeReleasePin(fixture, parent); err != nil {
		t.Fatal(err)
	}
	var output bytes.Buffer
	handled, err := qualificationChallengeBridge(context.Background(), []string{qualificationChallengeFlag + "=" + parent.dos}, bytes.NewReader(nil), &output, func(ctx context.Context, path string) (qualificationChallengeOwner, error) {
		return openQualificationChallengeParentNative(ctx, path, false)
	})
	if !handled || err == nil || bytes.Count(output.Bytes(), []byte("\n")) != 1 || bytes.Contains(output.Bytes(), []byte(`"consumed"`)) {
		t.Fatal("EOF became consumption", handled, err)
	}
	entries, err := os.ReadDir(parent.dos)
	if err != nil || len(entries) != 0 {
		t.Fatal("ready/EOF created a record", err)
	}
}

func TestNativeQualificationChallengeExistingEmptyOrPartialPermanentlyRefuses(t *testing.T) {
	for _, name := range []string{"empty", "partial"} {
		t.Run(name, func(t *testing.T) {
			_, parent := freshCacheNativeParent(t)
			raw := qualificationChallengeFixture()
			record, err := parseQualificationChallenge(raw)
			if err != nil {
				t.Fatal(err)
			}
			path := filepath.Join(parent.dos, ".installed-b0-consumed-"+record.Challenge+".json")
			original := []byte(nil)
			if name == "partial" {
				original = []byte("partial prior consumption; retain")
			}
			// os.WriteFile is only a deliberately damaged pre-existing fixture;
			// native CREATE_NEW must refuse it regardless of its length/security.
			if err := os.WriteFile(path, original, 0600); err != nil {
				t.Fatal(err)
			}
			info, err := os.Lstat(path)
			if err != nil || info.Size() != int64(len(original)) {
				t.Fatal("pre-existing fixture absent or wrong length", err)
			}
			owner, err := openQualificationChallengeParentNative(context.Background(), parent.dos, false)
			if err != nil {
				t.Fatal(err)
			}
			defer owner.close()
			if _, err := owner.consume(context.Background(), raw, record); err == nil {
				t.Fatal("partial/empty challenge was adopted")
			}
			actual, err := os.ReadFile(path)
			if err != nil || !bytes.Equal(actual, original) {
				t.Fatal("prior partial record was changed", err)
			}
		})
	}
}

func TestNativeQualificationChallengeMutableOrRedirectedParentNeverReady(t *testing.T) {
	for _, kind := range []string{"mutable", "redirected"} {
		t.Run(kind, func(t *testing.T) {
			fixture, parent := freshCacheNativeParent(t)
			path := filepath.Join(parent.dos, "untrusted")
			if kind == "mutable" {
				sd, err := windows.SecurityDescriptorFromString("O:BAD:P(A;OICI;FA;;;SY)(A;OICI;FA;;;BA)(A;OICI;FW;;;WD)")
				if err != nil {
					t.Fatal(err)
				}
				pin, err := fixture.newDirectory(parent, "untrusted", sd, "")
				if err != nil {
					t.Fatal(err)
				}
				if err := codeReleasePin(fixture, pin); err != nil {
					t.Fatal(err)
				}
			} else {
				alias, _ := windows.UTF16PtrFromString(path)
				target, _ := windows.UTF16PtrFromString(parent.dos)
				if err := windows.CreateSymbolicLink(alias, target, windows.SYMBOLIC_LINK_FLAG_DIRECTORY); err != nil {
					t.Fatal("elevated inert parent redirect unavailable", err)
				}
			}
			var output bytes.Buffer
			_, err := qualificationChallengeBridge(context.Background(), []string{qualificationChallengeFlag + "=" + path}, bytes.NewReader(qualificationChallengeFixture()), &output, func(ctx context.Context, path string) (qualificationChallengeOwner, error) {
				return openQualificationChallengeParentNative(ctx, path, false)
			})
			if err == nil || output.Len() != 0 {
				t.Fatal("untrusted parent emitted readiness/consumption", err)
			}
		})
	}
}
