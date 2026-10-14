//go:build windows

package brokerenroll

import (
	"context"
	"testing"
)

func TestNativeCodeUpgradeStartFenceExclusiveDurableAndPairBound(t *testing.T) {
	for _, kind := range []string{"absent", "empty", "foreign", "exact"} {
		t.Run(kind, func(t *testing.T) {
			c, p, raw, sd := nativeFinalizePairFixture(t, "valid")
			ctx := context.Background()
			publication := &nativeCodeFinalizePublication{original: c, files: &nativeEnrollmentFiles{}, parent: c.requestParent, descriptor: sd}
			defer publication.files.Close()
			backend := &nativeCodeStartBackend{nativeCodeFinalizeBackend: &nativeCodeFinalizeBackend{original: c}, publication: publication}
			if kind != "absent" {
				body := []byte("foreign")
				if kind == "empty" {
					body = []byte{}
				}
				if kind == "exact" {
					pin, e := maintenanceHoldFile(c.files, c.requestParent, c.clientSID+codeSuccessorSuffix, sd, maxCodeUpgradeRecordBytes)
					if e != nil {
						t.Fatal(e)
					}
					receipt, e := installedReadPin(ctx, pin, maxCodeUpgradeRecordBytes)
					if e != nil {
						t.Fatal(e)
					}
					body = codeStartAttemptBytes(digest(c.originalRaw), digest(raw), digest(receipt), digest(codeSuccessorMarker(digest(receipt))))
				}
				if _, e := startupWriteFile(ctx, c.files, c.requestParent, c.clientSID+codeStartAttemptSuffix, body, sd, false); e != nil {
					t.Fatal(e)
				}
			}
			err := backend.fence(ctx)
			if (kind == "absent") != (err == nil) {
				t.Fatal("exclusive fence result", err)
			}
			if backend.fence(ctx) == nil {
				t.Fatal("fence reused")
			}
			_, _, err = openCodeFinalizePairNative(ctx, c, p, raw, sd, sd)
			if (kind == "absent" || kind == "exact") != (err == nil) {
				t.Fatal("attempt closure", err)
			}
			if kind == "absent" || kind == "exact" {
				receipt, e := maintenanceHoldFile(c.files, c.requestParent, c.clientSID+codeSuccessorSuffix, sd, maxCodeUpgradeRecordBytes)
				if e != nil {
					t.Fatal(e)
				}
				receiptRaw, e := installedReadPin(ctx, receipt, maxCodeUpgradeRecordBytes)
				if e != nil {
					t.Fatal(e)
				}
				if _, e = startupWriteFile(ctx, c.files, c.requestParent, c.clientSID+codeSuccessorMarkerSuffix, codeSuccessorMarker(digest(receiptRaw)), sd, false); e != nil {
					t.Fatal(e)
				}
				if _, e = readCodeSuccessorNative(ctx, c, c.record, c.originalRaw, sd, sd); e != nil {
					t.Fatal("public reader rejected exact attempt", e)
				}
			}

		})
	}
}
