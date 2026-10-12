package brokerenroll

import (
	"context"
	"crypto/sha256"
	"encoding/hex"
	"errors"
	"io"

	verifier "example.com/opennav-update-verifier"
)

const clientBaselineInputName = "approved-installed-baseline.zip"
const maxClientBaselineBytes int64 = 512 << 20

// ApprovedClientBaseline retains a bounded read-only native file and its entire
// protected physical ancestry. Custody is not approval: the exact original
// independently approved request must also authenticate the archive's contents.
// Callers retain this owner through import/publication and check Close errors.
type ApprovedClientBaseline interface {
	io.ReaderAt
	Size() int64
	Validate() error
	Close() error
}

func validateApprovedClientBaseline(ctx context.Context, raw []byte, approvedSHA string, source ApprovedClientBaseline) error {
	_, err := verifier.ImportApprovedBaseline(ctx, raw, approvedSHA, source, source.Size(), discardBaselineSink{})
	return err
}

// Validation before mutation consumes every authenticated member without
// publishing an installed generation or manufacturing authority from an ACL.
type discardBaselineSink struct{}

func (discardBaselineSink) WriteFile(ctx context.Context, _ string, source io.Reader, size int64, wantSHA string) error {
	if ctx == nil || ctx.Err() != nil || source == nil || size < 0 || size > maxClientBaselineBytes {
		return errors.New("bounded baseline validation member required")
	}
	hash := sha256.New()
	n, err := io.CopyBuffer(hash, io.LimitReader(baselineContextReader{ctx, source}, size+1), make([]byte, 64<<10))
	if err != nil || n != size || hex.EncodeToString(hash.Sum(nil)) != wantSHA {
		return errors.Join(errors.New("baseline validation member differs"), err)
	}
	return ctx.Err()
}

func (discardBaselineSink) WriteEvidence(ctx context.Context, path string, raw []byte, wantSHA string) error {
	if ctx == nil || path != "FILE_SHA256.json" && path != "installer/package.json" || digest(raw) != wantSHA {
		return errors.New("baseline validation evidence differs or interrupted")
	}
	return ctx.Err()
}

type baselineContextReader struct {
	ctx context.Context
	io.Reader
}

func (r baselineContextReader) Read(p []byte) (int, error) {
	if err := r.ctx.Err(); err != nil {
		return 0, err
	}
	return r.Reader.Read(p)
}

func copyClientBaseline(ctx context.Context, destination io.Writer, source ApprovedClientBaseline) (string, error) {
	if source == nil {
		return "", errors.New("retained approved baseline source required")
	}
	if err := source.Validate(); err != nil {
		return "", err
	}
	hash, err := copyBaselineBytes(ctx, destination, source, source.Size())
	if err != nil {
		return "", err
	}
	if err := source.Validate(); err != nil {
		return "", err
	}
	return hash, nil
}

// Copy exactly the bounded retained file and probe EOF. The returned SHA binds
// original and durable readback bytes; it is never an independent approval pin.
func copyBaselineBytes(ctx context.Context, destination io.Writer, source io.ReaderAt, size int64) (string, error) {
	if ctx == nil || ctx.Err() != nil || destination == nil || source == nil || size < 1 || size > maxClientBaselineBytes {
		return "", errors.New("live bounded baseline file copy required")
	}
	hash := sha256.New()
	buffer := make([]byte, 64<<10)
	for offset := int64(0); offset < size; {
		if err := ctx.Err(); err != nil {
			return "", err
		}
		limit := int64(len(buffer))
		if size-offset < limit {
			limit = size - offset
		}
		n, err := source.ReadAt(buffer[:int(limit)], offset)
		if n != int(limit) || err != nil && !(errors.Is(err, io.EOF) && offset+limit == size) {
			return "", errors.Join(errors.New("baseline file length or read differs"), err)
		}
		if err := ctx.Err(); err != nil {
			return "", err
		}
		written, err := destination.Write(buffer[:n])
		if err != nil {
			return "", err
		}
		if written != n {
			return "", io.ErrShortWrite
		}
		_, _ = hash.Write(buffer[:n])
		offset += int64(n)
	}
	var extra [1]byte
	if n, err := source.ReadAt(extra[:], size); n != 0 || !errors.Is(err, io.EOF) {
		return "", errors.Join(errors.New("baseline file exceeds retained length"), err)
	}
	if err := ctx.Err(); err != nil {
		return "", err
	}
	return hex.EncodeToString(hash.Sum(nil)), nil
}
