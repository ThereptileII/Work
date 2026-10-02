package verifier

import (
	"bytes"
	"crypto/sha256"
	"encoding/hex"
	"errors"
	"io"
	"math"
	"strings"
	"testing"

	"example.com/opennav-update-verifier/releasepolicy"
)

func artifactFor(data []byte) releasepolicy.Artifact {
	sum := sha256.Sum256(data)
	return releasepolicy.Artifact{Bytes: int64(len(data)), SHA256: hex.EncodeToString(sum[:])}
}

type artifactReadFunc func([]byte) (int, error)

func (f artifactReadFunc) Read(p []byte) (int, error) { return f(p) }

func TestVerifyArtifactIntegrity(t *testing.T) {
	payload := []byte("retained authenticated installer bytes")
	expected := artifactFor(payload)
	for _, tc := range []struct {
		name string
		data []byte
		want bool
	}{
		{"valid", payload, true},
		{"same-length-tamper", bytes.Repeat([]byte("x"), len(payload)), false},
		{"truncated", payload[:len(payload)-1], false},
		{"empty", nil, false},
		{"extra-byte", append(append([]byte{}, payload...), 0), false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			if err := VerifyArtifact(bytes.NewReader(tc.data), expected); (err == nil) != tc.want {
				t.Fatalf("VerifyArtifact success=%v, want %v: %v", err == nil, tc.want, err)
			}
		})
	}
}

func TestVerifyArtifactRejectsMetadataBeforeReading(t *testing.T) {
	valid := artifactFor([]byte("payload"))
	cases := []releasepolicy.Artifact{}
	for _, length := range []int64{0, -1, math.MinInt64, (int64(4) << 30) + 1, math.MaxInt64} {
		value := valid
		value.Bytes = length
		cases = append(cases, value)
	}
	for _, digest := range []string{"", strings.Repeat("a", 63), strings.Repeat("a", 65), strings.Repeat("g", 64), strings.Repeat("A", 64), valid.SHA256 + "\n"} {
		value := valid
		value.SHA256 = digest
		cases = append(cases, value)
	}
	for _, value := range cases {
		reader := artifactReadFunc(func([]byte) (int, error) { t.Fatal("read before validating metadata"); return 0, io.EOF })
		if err := VerifyArtifact(reader, value); err == nil {
			t.Fatalf("accepted invalid artifact metadata: %+v", value)
		}
	}
	if err := VerifyArtifact(nil, valid); err == nil {
		t.Fatal("accepted nil reader")
	}
	// Exactly 4 GiB is permitted metadata; do not allocate or read that much.
	value := valid
	value.Bytes = int64(4) << 30
	sentinel := errors.New("reader reached at maximum accepted length")
	if err := VerifyArtifact(artifactReadFunc(func([]byte) (int, error) { return 0, sentinel }), value); !errors.Is(err, sentinel) {
		t.Fatalf("maximum metadata rejected before reader: %v", err)
	}
}

func TestVerifyArtifactReaderErrors(t *testing.T) {
	payload := []byte("payload")
	expected := artifactFor(payload)
	sentinel := errors.New("storage failed")
	for _, count := range []int{0, 1, len(payload)} {
		reader := artifactReadFunc(func(p []byte) (int, error) { return copy(p, payload[:count]), sentinel })
		if err := VerifyArtifact(reader, expected); !errors.Is(err, sentinel) {
			t.Fatalf("lost error with %d returned bytes: %v", count, err)
		}
	}
	// Reader is allowed to return its final bytes and EOF together.
	reader := artifactReadFunc(func(p []byte) (int, error) { return copy(p, payload), io.EOF })
	if err := VerifyArtifact(reader, expected); err != nil {
		t.Fatalf("valid final bytes with EOF: %v", err)
	}
	reader = artifactReadFunc(func(p []byte) (int, error) { return copy(p, payload[:1]), io.EOF })
	if err := VerifyArtifact(reader, expected); !errors.Is(err, io.ErrUnexpectedEOF) {
		t.Fatalf("truncation not reported: %v", err)
	}
	for _, count := range []int{-1, 1000} {
		reader = artifactReadFunc(func([]byte) (int, error) { return count, nil })
		if err := VerifyArtifact(reader, expected); err == nil {
			t.Fatalf("accepted invalid reader count %d", count)
		}
	}
}

func TestVerifyArtifactReadBound(t *testing.T) {
	payload := bytes.Repeat([]byte("p"), 70000)
	expected := artifactFor(payload)
	consumed := 0
	maxWindow := 0
	reader := artifactReadFunc(func(p []byte) (int, error) {
		if len(p) > maxWindow {
			maxWindow = len(p)
		}
		for i := range p {
			p[i] = 'p'
		}
		consumed += len(p)
		return len(p), nil // An endless stream must stop at the one-byte probe.
	})
	if err := VerifyArtifact(reader, expected); err == nil {
		t.Fatal("accepted endless oversized stream")
	}
	if consumed != len(payload)+1 || maxWindow > 32<<10 {
		t.Fatalf("unbounded read: consumed=%d, window=%d", consumed, maxWindow)
	}
}

func TestVerifyArtifactNoProgress(t *testing.T) {
	expected := artifactFor([]byte("xy"))
	calls := 0
	reader := artifactReadFunc(func([]byte) (int, error) {
		calls++
		if calls > 100 {
			t.Fatal("no-progress reader did not stop")
		}
		return 0, nil
	})
	if err := VerifyArtifact(reader, expected); !errors.Is(err, io.ErrNoProgress) {
		t.Fatalf("no-progress result: %v", err)
	}
	// Each real byte resets the consecutive-empty-read counter.
	calls = 0
	reader = artifactReadFunc(func(p []byte) (int, error) {
		calls++
		switch calls {
		case 100:
			p[0] = 'x'
			return 1, nil
		case 200:
			p[0] = 'y'
			return 1, io.EOF
		default:
			return 0, nil
		}
	})
	if err := VerifyArtifact(reader, expected); err != nil {
		t.Fatalf("intermittent progress refused: %v", err)
	}
}
