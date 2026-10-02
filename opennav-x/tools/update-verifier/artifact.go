package verifier

import (
	"crypto/sha256"
	"encoding/hex"
	"errors"
	"fmt"
	"io"

	"example.com/opennav-update-verifier/releasepolicy"
)

const maxArtifactPayloadBytes = int64(4) << 30

// VerifyArtifact checks a stream against the length and SHA-256 from an already
// authenticated, parsed release policy. It consumes at most artifact.Bytes+1
// bytes, uses fixed-size buffering, and performs no download or installation.
// The caller must retain and execute these same verified bytes; this function
// does not establish file custody or prevent a later time-of-check/use race.
// The caller is also responsible for deadlines on any blocking Reader.
func VerifyArtifact(reader io.Reader, artifact releasepolicy.Artifact) error {
	if artifact.Bytes <= 0 || artifact.Bytes > maxArtifactPayloadBytes {
		return errors.New("artifact length must be positive and at most 4 GiB")
	}
	if len(artifact.SHA256) != hex.EncodedLen(sha256.Size) {
		return errors.New("artifact SHA-256 must be exactly 64 lowercase hexadecimal characters")
	}
	digest, err := hex.DecodeString(artifact.SHA256)
	if err != nil || len(digest) != sha256.Size || hex.EncodeToString(digest) != artifact.SHA256 {
		return errors.New("artifact SHA-256 must be exactly 64 lowercase hexadecimal characters")
	}
	if reader == nil {
		return errors.New("artifact reader is nil")
	}

	hash := sha256.New()
	var buffer [32 << 10]byte
	var total int64
	emptyReads := 0
	for {
		// Metadata validation makes the addition and conversion safe on 32-bit
		// hosts too. One extra byte establishes whether the stream is oversized.
		remaining := artifact.Bytes + 1 - total
		window := buffer[:]
		if remaining < int64(len(window)) {
			window = window[:int(remaining)]
		}
		n, readErr := reader.Read(window)
		if n < 0 || n > len(window) {
			return errors.New("artifact reader returned an invalid byte count")
		}
		if n > 0 {
			_, _ = hash.Write(window[:n]) // SHA-256's Write cannot fail.
			total += int64(n)
			emptyReads = 0
		} else if readErr == nil {
			emptyReads++
			if emptyReads >= 100 {
				return fmt.Errorf("read artifact: %w", io.ErrNoProgress)
			}
		}
		// Do not turn a partial read with an I/O failure into successful hash
		// verification, even if it delivered the expected final bytes.
		if readErr != nil && readErr != io.EOF {
			return fmt.Errorf("read artifact: %w", readErr)
		}
		if total > artifact.Bytes {
			return errors.New("artifact exceeds declared length")
		}
		if readErr == io.EOF {
			if total != artifact.Bytes {
				return fmt.Errorf("artifact shorter than declared length: %w", io.ErrUnexpectedEOF)
			}
			if hex.EncodeToString(hash.Sum(nil)) != artifact.SHA256 {
				return errors.New("artifact SHA-256 mismatch")
			}
			return nil
		}
	}
}
