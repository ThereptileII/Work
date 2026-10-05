package verifier

import (
	"bytes"
	"strings"
	"testing"

	"github.com/theupdateframework/go-tuf/v2/metadata"
)

// Signed length declarations are integrity information, not permission to
// exceed the client's resource policy. go-tuf's configured maxima are defaults
// when a declaration omits Length, so the wrapper must impose independent caps.
func TestReviewSignedMetadataLengthCannotRaiseResourceCeiling(t *testing.T) {
	for _, role := range []string{"snapshot", "targets"} {
		t.Run(role, func(t *testing.T) {
			f := newFixture(t)
			limit := 2 << 20
			if role == "targets" {
				limit = 5 << 20
			}
			path := "/" + role + ".json"
			original := f.files[path]
			// Trailing whitespace preserves the parsed signed metadata exactly.
			f.files[path] = append(original, bytes.Repeat([]byte(" "), limit+1-len(original))...)
			if role == "snapshot" {
				stamp, err := metadata.Timestamp().FromBytes(f.files["/timestamp.json"])
				if err != nil {
					t.Fatal(err)
				}
				stamp.Signed.Meta["snapshot.json"].Length = int64(len(f.files[path]))
				stamp.Signatures = nil
				sign(t, stamp, f.keys["timestamp"])
				f.files["/timestamp.json"] = bytesOf(t, stamp)
			} else {
				snapshot, err := metadata.Snapshot().FromBytes(f.files["/snapshot.json"])
				if err != nil {
					t.Fatal(err)
				}
				snapshot.Signed.Meta["targets.json"].Length = int64(len(f.files[path]))
				snapshot.Signatures = nil
				sign(t, snapshot, f.keys["snapshot"])
				f.files["/snapshot.json"] = bytesOf(t, snapshot)
			}
			if _, _, err := f.verify("beta"); err == nil || !strings.Contains(err.Error(), "hard limit") {
				t.Fatalf("signed %s Length must not raise resource ceiling above %d bytes: %v", role, limit, err)
			}
		})
	}
}
