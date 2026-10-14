//go:build windows

package brokerenroll

import (
	"bytes"
	"context"
	"os"
	"testing"
)

// Inert native originals only; never an installed service or machine namespace.
func TestNativeInstalledMaintenanceBindingReadIdentityAndReplacement(t *testing.T) {
	owner, _, recovery, sd := startupNativeFixture(t)
	ctx := context.Background()
	body := []byte("original completed installation bytes")
	pin, err := startupWriteFile(ctx, owner.files, recovery, "inert-completion.json", body, sd, false)
	if err != nil {
		t.Fatal(err)
	}
	want, err := startupFileIdentity(ctx, pin, digest(body))
	if err != nil {
		t.Fatal(err)
	}
	got, err := installedReadPin(ctx, pin, 1024)
	if err != nil || !bytes.Equal(got, body) || installedMatchFile(ctx, pin, want) != nil {
		t.Fatal("original identity or bytes differed", err)
	}
	for _, field := range []string{"file-id", "creation", "length", "hash"} {
		t.Run(field, func(t *testing.T) {
			changed := want
			switch field {
			case "file-id":
				changed.FileID[0] ^= 1
			case "creation":
				changed.Creation++
			case "length":
				changed.Bytes++
			case "hash":
				changed.SHA256 = digest([]byte("different"))
			}
			if err := installedMatchFile(ctx, pin, changed); err == nil {
				t.Fatal("substituted native identity accepted")
			}
		})
	}
	if err := os.WriteFile(pin.dos, []byte("replacement"), 0600); err == nil {
		t.Fatal("original custody allowed foreign write")
	}
	if err := os.Remove(pin.dos); err == nil {
		t.Fatal("original custody allowed foreign deletion")
	}
	if raw, err := installedReadPin(ctx, pin, int64(len(body)-1)); err == nil || raw != nil {
		t.Fatal("oversized original record accepted")
	}
	canceled, cancel := context.WithCancel(ctx)
	cancel()
	if raw, err := installedReadPin(canceled, pin, 1024); err == nil || raw != nil {
		t.Fatal("canceled original record read accepted")
	}
	if err := installedMatchFile(ctx, pin, want); err != nil {
		t.Fatal("rejected replacement changed original", err)
	}
}
