//go:build windows

package brokerenroll

import (
	"bytes"
	"context"
	"os"
	"path/filepath"
	"strings"
	"testing"

	"golang.org/x/sys/windows"
)

// Disposable NTFS publication only: no service is opened, configured or started.
// The same complete-pair authority and optional attempt reader are used by the
// runtime. A native pass here remains component evidence, never installed SCM.
func TestNativeCodeUpgradeDemandAttemptExclusiveAndExactPairBound(t *testing.T) {
	for _, kind := range []string{"absent", "empty", "torn", "foreign-policy", "exact"} {
		t.Run(kind, func(t *testing.T) {
			c, p, raw, sd := nativeFinalizePairFixture(t, "valid")
			ctx := context.Background()
			_, pairRaw, err := openCodeFinalizePairNative(ctx, c, p, raw, sd, sd)
			if err != nil {
				t.Fatal(err)
			}
			receipt, err := maintenanceHoldFile(c.files, c.requestParent, c.clientSID+codeSuccessorSuffix, sd, maxCodeUpgradeRecordBytes)
			if err != nil {
				t.Fatal(err)
			}
			receiptRaw, err := installedReadPin(ctx, receipt, maxCodeUpgradeRecordBytes)
			if err != nil {
				t.Fatal(err)
			}
			original, policy, successor, pair := digest(c.originalRaw), digest(raw), digest(receiptRaw), digest(pairRaw)
			attempt := codeDemandAttemptBytes(original, policy, successor, pair)
			files := &nativeEnrollmentFiles{}
			defer files.Close()
			if kind == "empty" {
				// Simulate interruption before the first write. Production's normal
				// nonempty-file publisher must not be weakened to make this fixture.
				path := filepath.Join(c.requestParent.dos, c.clientSID+codeDemandAttemptSuffix)
				file, e := os.OpenFile(path, os.O_WRONLY|os.O_CREATE|os.O_EXCL, 0600)
				if e != nil {
					t.Fatal(e)
				}
				if e = file.Close(); e != nil {
					t.Fatal(e)
				}
				owner, _, e := sd.Owner()
				if e != nil {
					t.Fatal(e)
				}
				dacl, _, e := sd.DACL()
				if e != nil {
					t.Fatal(e)
				}
				if e = windows.SetNamedSecurityInfo(path, windows.SE_FILE_OBJECT,
					windows.OWNER_SECURITY_INFORMATION|windows.DACL_SECURITY_INFORMATION|windows.PROTECTED_DACL_SECURITY_INFORMATION,
					owner, nil, dacl, nil); e != nil {
					t.Fatal(e)
				}
				actualSD, e := windows.GetNamedSecurityInfo(path, windows.SE_FILE_OBJECT, windows.OWNER_SECURITY_INFORMATION|windows.DACL_SECURITY_INFORMATION)
				if e != nil {
					t.Fatal(e)
				}
				if e = enrollmentSecurityDescriptor(actualSD, sd, false, c.clientSID); e != nil {
					t.Fatal("empty fixture descriptor differs", e)
				}
				before, e := os.Lstat(path)
				if e != nil || !before.Mode().IsRegular() || before.Size() != 0 {
					t.Fatal("empty attempt fixture not created", e)
				}
				defer func() {
					after, e := os.Lstat(path)
					if e != nil || !os.SameFile(before, after) || after.Size() != 0 {
						t.Fatal("refusal changed empty attempt identity", e)
					}
				}()
			} else if kind != "absent" {
				body := attempt
				switch kind {
				case "torn":
					body = attempt[:len(attempt)-1]
				case "foreign-policy":
					body = codeDemandAttemptBytes(original, strings.Repeat("f", 64), successor, pair)
				}
				if _, err = startupWriteFile(ctx, files, c.requestParent, c.clientSID+codeDemandAttemptSuffix, body, sd, false); err != nil {
					t.Fatal(err)
				}
			}
			present, err := readCodeDemandAttemptNative(ctx, c, sd, original, policy, successor, pair)
			if present != (kind != "absent") || (err == nil) != (kind == "absent" || kind == "exact") {
				t.Fatal("native attempt presence/authority", present, err)
			}
			// Exercise the integrated pending-pair reader as well as its leaf reader:
			// malformed attempts must not reach AUTO start or finalization admission.
			if _, _, e := openCodeFinalizePairNative(ctx, c, p, raw, sd, sd); (e == nil) != (kind == "absent" || kind == "exact") {
				t.Fatal("complete-pair reader lost demand-attempt boundary", e)
			}
			pin, writeErr := writeCodeUpgradeFileNative(ctx, files, c.requestParent, c.clientSID+codeDemandAttemptSuffix, sd, attempt)
			if kind == "absent" {
				if writeErr != nil {
					t.Fatal(writeErr)
				}
				actual, e := installedReadPin(ctx, pin, maxCodeUpgradeRecordBytes)
				if e != nil || !bytes.Equal(actual, attempt) {
					t.Fatal("flushed native attempt changed", e)
				}
			} else if writeErr == nil {
				t.Fatal("existing attempt namespace overwritten")
			}
			if _, err = writeCodeUpgradeFileNative(ctx, files, c.requestParent, c.clientSID+codeDemandAttemptSuffix, sd, attempt); err == nil {
				t.Fatal("configuration fence reused")
			}
			if kind == "absent" || kind == "exact" {
				if present, err = readCodeDemandAttemptNative(ctx, c, sd, original, policy, successor, pair); !present || err != nil {
					t.Fatal("exact durable attempt not reconcilable", present, err)
				}
			}
		})
	}
}

func TestNativeCodeUpgradeDemandAttemptCannotAdoptAliasNamespace(t *testing.T) {
	c, p, raw, sd := nativeFinalizePairFixture(t, "valid")
	ctx := context.Background()
	_, pairRaw, err := openCodeFinalizePairNative(ctx, c, p, raw, sd, sd)
	if err != nil {
		t.Fatal(err)
	}
	// A directory with the reserved name is a present namespace, never an absent
	// attempt. It must refuse before any service configuration capability exists.
	path := filepath.Join(c.requestParent.dos, c.clientSID+codeDemandAttemptSuffix)
	pointer, err := windows.UTF16PtrFromString(path)
	if err != nil {
		t.Fatal(err)
	}
	if err = windows.CreateDirectory(pointer, nil); err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { windows.RemoveDirectory(pointer) })
	if present, e := readCodeDemandAttemptNative(ctx, c, sd, digest(c.originalRaw), digest(raw), strings.Repeat("a", 64), digest(pairRaw)); !present || e == nil {
		t.Fatal("nonfile attempt namespace accepted", present, e)
	}
}
