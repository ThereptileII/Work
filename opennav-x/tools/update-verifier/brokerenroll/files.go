package brokerenroll

import (
	"context"
	"crypto/sha256"
	"encoding/hex"
	"errors"
	"io"
	"strings"

	"example.com/opennav-update-verifier/releasepolicy"
)

const enrollmentImageName = "skager-update-broker.exe"

func validateEnrollmentImage(ctx context.Context, source io.Reader, artifact releasepolicy.Artifact) error {
	if ctx == nil || ctx.Err() != nil || source == nil {
		return errors.New("live context and authenticated broker stream required")
	}
	decoded, err := hex.DecodeString(artifact.SHA256)
	if err != nil || len(decoded) != sha256.Size || hex.EncodeToString(decoded) != artifact.SHA256 ||
		artifact.Bytes <= 0 || artifact.Bytes > maxBrokerBytes ||
		!strings.HasSuffix(artifact.Path, "/"+enrollmentImageName) {
		return errors.New("bounded exact authenticated broker artifact required")
	}
	return nil
}

// copyEnrollmentImage consumes exactly the approved bytes and one EOF probe.
// The caller owns source and must bound blocking reads; context cancellation is
// checked between reads and writes, never implemented by abandoning a goroutine.
func copyEnrollmentImage(ctx context.Context, destination io.Writer, source io.Reader, artifact releasepolicy.Artifact) error {
	if err := validateEnrollmentImage(ctx, source, artifact); err != nil {
		return err
	}
	hash := sha256.New()
	buffer := make([]byte, 32<<10)
	remaining := artifact.Bytes
	empty := 0
	for {
		if err := ctx.Err(); err != nil {
			return err
		}
		limit := int64(len(buffer))
		if remaining < limit {
			limit = remaining + 1
		}
		n, err := source.Read(buffer[:int(limit)])
		if n < 0 || n > int(limit) {
			return errors.New("invalid broker stream read")
		}
		if int64(n) > remaining {
			return errors.New("broker stream exceeds approved length")
		}
		if n > 0 {
			empty = 0
			if ctx.Err() != nil {
				return ctx.Err()
			}
			written, writeErr := destination.Write(buffer[:n])
			if writeErr != nil {
				return writeErr
			}
			if written != n {
				return io.ErrShortWrite
			}
			_, _ = hash.Write(buffer[:n])
			remaining -= int64(n)
		} else {
			empty++
			if empty >= 100 && err == nil {
				return io.ErrNoProgress
			}
		}
		if err != nil {
			if err != io.EOF {
				return err
			}
			if remaining != 0 || hex.EncodeToString(hash.Sum(nil)) != artifact.SHA256 {
				return errors.New("broker stream differs from approved length or SHA-256")
			}
			return ctx.Err()
		}
	}
}
