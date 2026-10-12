// Package baselinepackage prepares an operator-reviewable initial client handoff.
// It does not approve its own output, enroll a service, or install an application.
package baselinepackage

import (
	"archive/zip"
	"bytes"
	"context"
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"os"
	"path/filepath"
	"runtime"
	"strings"
	"time"

	verifier "example.com/opennav-update-verifier"
	"example.com/opennav-update-verifier/brokerregistration"
)

const ArchiveName = "approved-installed-baseline.zip"
const RequestName = "client-registration-request.json"
const recoveryPrefix = "SKAGER-Beta2-Portable-Recovery/"
const maxArchiveBytes int64 = 2 << 30
const maxTreeFiles = 12000

// Input pins must arrive from independent operator approval, not from the
// supplied archive, extracted tree, mutable installation, or this package.
type Input struct {
	SourceArchive, BaselineTree, TrustConfig, StockPath, OutputDirectory   string
	ClientSID, Generation, Version, Commit                                 string
	ApprovedSourceSHA256, ApprovedPackageSHA256, ApprovedBuildSHA256       string
	ApprovedExecutableSHA256, ApprovedInventorySHA256, ApprovedTrustSHA256 string
	ApprovedStockSHA256                                                    string
}

type Result struct {
	ArchivePath, RequestPath, RequestSHA256 string
}

// Produce is the operator-side producer. The request digest it reports is for
// subsequent separate approval; passing it back as an approval without that
// step would violate the enrollment contract.
func Produce(ctx context.Context, in Input) (Result, error) {
	if runtime.GOOS != "windows" {
		return Result{}, errors.New("native Windows stock observation required")
	}
	return produce(ctx, in, observeStock)
}

func produce(ctx context.Context, in Input, stockCheck func(string) error) (result Result, resultErr error) {
	var empty Result
	if ctx == nil || ctx.Err() != nil || stockCheck == nil ||
		in.SourceArchive == "" || in.BaselineTree == "" || in.TrustConfig == "" || in.StockPath == "" || in.OutputDirectory == "" ||
		in.ClientSID == "" || in.Generation == "" || in.Version == "" || in.Commit == "" {
		return empty, errors.New("complete live baseline inputs required")
	}
	for _, pin := range []string{in.ApprovedSourceSHA256, in.ApprovedPackageSHA256, in.ApprovedBuildSHA256,
		in.ApprovedExecutableSHA256, in.ApprovedInventorySHA256, in.ApprovedTrustSHA256, in.ApprovedStockSHA256} {
		if !pinOK(pin) {
			return empty, errors.New("independent lowercase SHA-256 pins required")
		}
	}
	stock := brokerregistration.StockBaseline()
	if in.ApprovedStockSHA256 != stock.ExecutableSHA256 || !brokerregistration.IsCanonicalUserSID(in.ClientSID) {
		return empty, errors.New("approved fixed stock identity and canonical client required")
	}
	for _, path := range []string{in.SourceArchive, in.BaselineTree, in.TrustConfig, in.OutputDirectory} {
		if err := plainExisting(path); err != nil {
			return empty, err
		}
	}
	if err := stockCheck(in.StockPath); err != nil {
		return empty, fmt.Errorf("observed stock refused: %w", err)
	}
	trust, err := readBoundedRegular(ctx, in.TrustConfig, brokerregistration.MaxTrustConfigurationBytes)
	if err != nil ||
		!bytes.Equal(bytes.TrimSpace(trust), trust) || digest(trust) != in.ApprovedTrustSHA256 {
		return empty, errors.Join(errors.New("trust bytes differ from independent approval"), err)
	}
	source, err := os.Open(in.SourceArchive)
	if err != nil {
		return empty, err
	}
	defer func() {
		if err := source.Close(); err != nil {
			result = Result{}
			resultErr = errors.Join(resultErr, err)
		}
	}()
	info, err := source.Stat()
	if err != nil || !info.Mode().IsRegular() || info.Size() < 22 || info.Size() > maxArchiveBytes {
		return empty, errors.New("bounded regular source archive required")
	}
	if err := hashReader(ctx, source, info.Size(), in.ApprovedSourceSHA256); err != nil {
		return empty, fmt.Errorf("source differs from independent approval: %w", err)
	}
	if err := compareTree(ctx, source, info.Size(), in.BaselineTree); err != nil {
		return empty, err
	}
	request := brokerregistration.Request{Schema: 1, Purpose: brokerregistration.Purpose, ClientSID: in.ClientSID,
		Installed: brokerregistration.Installed{Version: in.Version, Commit: in.Commit, Generation: in.Generation,
			PackageSHA256: in.ApprovedPackageSHA256, ProductBuildSHA256: in.ApprovedBuildSHA256,
			ExecutableSHA256: in.ApprovedExecutableSHA256, InventorySHA256: in.ApprovedInventorySHA256},
		Stock: brokerregistration.Stock{Path: in.StockPath, Version: stock.Version, Arch: stock.Arch,
			ExecutableSHA256: stock.ExecutableSHA256, UpstreamCommit: stock.UpstreamCommit},
		TrustConfiguration: append([]byte(nil), trust...), TrustConfigSHA256: in.ApprovedTrustSHA256}
	// encoding/json compacts RawMessage values. Assemble the last field without
	// changing any independently pinned bytes inside trustConfiguration.
	raw, err := json.Marshal(struct {
		Schema            int                          `json:"schema"`
		Purpose           string                       `json:"purpose"`
		ClientSID         string                       `json:"clientSid"`
		Installed         brokerregistration.Installed `json:"installed"`
		Stock             brokerregistration.Stock     `json:"stock"`
		TrustConfigSHA256 string                       `json:"trustConfigSha256"`
	}{request.Schema, request.Purpose, request.ClientSID, request.Installed, request.Stock, request.TrustConfigSHA256})
	if err != nil {
		return empty, err
	}
	raw = append(raw[:len(raw)-1], []byte(`,"trustConfiguration":`)...)
	raw = append(raw, trust...)
	raw = append(raw, '}', '\n')
	requestSHA := digest(raw)
	if _, err := brokerregistration.ParseApprovedRequest(raw, requestSHA, time.Now().UTC()); err != nil {
		return empty, fmt.Errorf("strict client request refused: %w", err)
	}
	if _, err := verifier.ImportApprovedBaseline(ctx, raw, requestSHA, source, info.Size(), validationSink{}); err != nil {
		return empty, fmt.Errorf("approved baseline importer refused source: %w", err)
	}
	if err := checkOutput(in); err != nil {
		return empty, err
	}
	archivePath := filepath.Join(in.OutputDirectory, ArchiveName)
	requestPath := filepath.Join(in.OutputDirectory, RequestName)
	if err := copyExclusive(ctx, source, info.Size(), archivePath, in.ApprovedSourceSHA256); err != nil {
		return empty, err
	}
	// Recheck the actual final archive, not merely the source or copied hash.
	if err := validateFinalArchive(ctx, archivePath, info.Size(), in.ApprovedSourceSHA256, raw, requestSHA); err != nil {
		return empty, err
	}
	if err := compareTree(ctx, source, info.Size(), in.BaselineTree); err != nil {
		return empty, err
	}
	if err := stockCheck(in.StockPath); err != nil {
		return empty, fmt.Errorf("stock changed during handoff: %w", err)
	}
	if err := writeExclusive(ctx, requestPath, raw); err != nil {
		return empty, err
	}
	// A successful handoff requires the retained original source to remain the
	// exact approved bytes after both outputs have been durably read back.
	if err := hashReader(ctx, source, info.Size(), in.ApprovedSourceSHA256); err != nil {
		return empty, fmt.Errorf("source changed after output: %w", err)
	}
	if last, err := source.Stat(); err != nil || !last.Mode().IsRegular() || last.Size() != info.Size() {
		return empty, errors.Join(errors.New("source size changed after output"), err)
	}
	return Result{ArchivePath: archivePath, RequestPath: requestPath, RequestSHA256: requestSHA}, nil
}

func readBoundedRegular(ctx context.Context, path string, max int64) (data []byte, resultErr error) {
	if ctx == nil || ctx.Err() != nil || max < 1 {
		return nil, errors.New("live bounded input required")
	}
	file, err := os.Open(path)
	if err != nil {
		return nil, err
	}
	defer func() { resultErr = errors.Join(resultErr, file.Close()) }()
	info, err := file.Stat()
	if err != nil || !info.Mode().IsRegular() || info.Size() < 1 || info.Size() > max {
		return nil, errors.Join(errors.New("bounded regular input required"), err)
	}
	data, err = io.ReadAll(io.LimitReader(contextReader{ctx, file}, max+1))
	if err != nil || int64(len(data)) != info.Size() || len(data) == 0 || int64(len(data)) > max {
		return nil, errors.Join(errors.New("bounded input changed during read"), err)
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	return data, nil
}

func validateFinalArchive(ctx context.Context, path string, size int64, sourceSHA string, request []byte, requestSHA string) (resultErr error) {
	final, err := os.Open(path)
	if err != nil {
		return err
	}
	defer func() { resultErr = errors.Join(resultErr, final.Close()) }()
	info, err := final.Stat()
	if err != nil || !info.Mode().IsRegular() || info.Size() != size {
		return errors.Join(errors.New("final archive length differs"), err)
	}
	if err := hashReader(ctx, final, size, sourceSHA); err != nil {
		return fmt.Errorf("final source bytes differ: %w", err)
	}
	if _, err := verifier.ImportApprovedBaseline(ctx, request, requestSHA, final, size, validationSink{}); err != nil {
		return fmt.Errorf("final baseline importer refused: %w", err)
	}
	return nil
}

func pinOK(s string) bool {
	if len(s) != 64 {
		return false
	}
	for _, c := range s {
		if c < '0' || c > '9' && c < 'a' || c > 'f' {
			return false
		}
	}
	return true
}
func digest(b []byte) string { s := sha256.Sum256(b); return hex.EncodeToString(s[:]) }

func observeStock(path string) (resultErr error) {
	if err := plainExisting(path); err != nil {
		return err
	}
	f, err := os.Open(path)
	if err != nil {
		return err
	}
	defer func() { resultErr = errors.Join(resultErr, f.Close()) }()
	info, err := f.Stat()
	if err != nil || !info.Mode().IsRegular() || info.Size() < 1 || info.Size() > maxArchiveBytes {
		return errors.New("bounded regular stock executable required")
	}
	if err := hashReader(context.Background(), f, info.Size(), brokerregistration.StockBaseline().ExecutableSHA256); err != nil {
		return err
	}
	if last, err := f.Stat(); err != nil || !last.Mode().IsRegular() || last.Size() != info.Size() {
		return errors.Join(errors.New("stock executable size changed"), err)
	}
	return nil
}

func plainExisting(path string) error {
	abs, err := filepath.Abs(path)
	if err != nil {
		return err
	}
	for p := abs; ; p = filepath.Dir(p) {
		info, err := os.Lstat(p)
		if err != nil {
			return err
		}
		if info.Mode()&os.ModeSymlink != 0 || info.Mode()&os.ModeType != 0 && !info.IsDir() {
			if p != abs || !info.Mode().IsRegular() {
				return errors.New("linked or special input ancestry refused")
			}
		}
		if parent := filepath.Dir(p); parent == p {
			break
		}
	}
	return nil
}

func hashReader(ctx context.Context, r io.ReaderAt, size int64, want string) error {
	h := sha256.New()
	buf := make([]byte, 64<<10)
	for off := int64(0); off < size; {
		if err := ctx.Err(); err != nil {
			return err
		}
		n := int64(len(buf))
		if size-off < n {
			n = size - off
		}
		read, err := r.ReadAt(buf[:n], off)
		if int64(read) != n || err != nil && !errors.Is(err, io.EOF) {
			return errors.New("source read changed")
		}
		h.Write(buf[:read])
		off += int64(read)
	}
	if hex.EncodeToString(h.Sum(nil)) != want {
		return errors.New("SHA-256 differs")
	}
	return nil
}

func compareTree(ctx context.Context, source io.ReaderAt, size int64, root string) error {
	z, err := zip.NewReader(source, size)
	if err != nil {
		return err
	}
	if len(z.File) == 0 || len(z.File) >= maxTreeFiles {
		return errors.New("baseline archive member bound")
	}
	files := map[string]*zip.File{}
	var expanded uint64
	for _, f := range z.File {
		if !strings.HasPrefix(f.Name, recoveryPrefix) || !f.Mode().IsRegular() ||
			f.UncompressedSize64 > 512<<20 || f.UncompressedSize64 > uint64(maxArchiveBytes)-expanded {
			return errors.New("unexpected baseline archive member")
		}
		expanded += f.UncompressedSize64
		name := strings.TrimPrefix(f.Name, recoveryPrefix)
		if _, ok := files[name]; ok {
			return errors.New("duplicate baseline archive member")
		}
		files[name] = f
	}
	actual := map[string]bool{}
	err = filepath.WalkDir(root, func(path string, d os.DirEntry, walkErr error) error {
		if walkErr != nil {
			return walkErr
		}
		if err := ctx.Err(); err != nil {
			return err
		}
		if err := plainExisting(path); err != nil {
			return err
		}
		if path == root || d.IsDir() {
			return nil
		}
		if !d.Type().IsRegular() {
			return errors.New("non-regular baseline tree member")
		}
		rel, err := filepath.Rel(root, path)
		if err != nil {
			return err
		}
		name := filepath.ToSlash(rel)
		member := files[name]
		if member == nil || actual[name] {
			return errors.New("unlisted or duplicate baseline tree member")
		}
		actual[name] = true
		info, err := os.Lstat(path)
		if err != nil || !info.Mode().IsRegular() || uint64(info.Size()) != member.UncompressedSize64 {
			return errors.New("baseline tree member length differs")
		}
		input, err := os.Open(path)
		if err != nil {
			return err
		}
		reader, err := member.Open()
		if err != nil {
			return errors.Join(err, input.Close())
		}
		left, na, ea := boundedDigest(ctx, input, int64(member.UncompressedSize64))
		right, nb, eb := boundedDigest(ctx, reader, int64(member.UncompressedSize64))
		closeErr := errors.Join(input.Close(), reader.Close())
		if ea != nil || eb != nil || closeErr != nil || na != nb || uint64(na) != member.UncompressedSize64 || !bytes.Equal(left, right) {
			return errors.Join(errors.New("baseline tree differs from source archive"), ea, eb, closeErr)
		}
		return nil
	})
	if err != nil {
		return err
	}
	if len(actual) != len(files) {
		return errors.New("baseline tree has missing archive members")
	}
	return nil
}

type contextReader struct {
	ctx context.Context
	io.Reader
}

func (r contextReader) Read(p []byte) (int, error) {
	if err := r.ctx.Err(); err != nil {
		return 0, err
	}
	return r.Reader.Read(p)
}

func boundedDigest(ctx context.Context, source io.Reader, size int64) ([]byte, int64, error) {
	if ctx == nil || source == nil || size < 0 || size > maxArchiveBytes {
		return nil, 0, errors.New("bounded source required")
	}
	h := sha256.New()
	n, err := io.CopyBuffer(h, io.LimitReader(contextReader{ctx, source}, size+1), make([]byte, 64<<10))
	if err != nil || n != size {
		return nil, n, errors.Join(errors.New("bounded source length or read differs"), err)
	}
	if err := ctx.Err(); err != nil {
		return nil, n, err
	}
	return h.Sum(nil), n, nil
}

func checkOutput(in Input) error {
	root, err := filepath.Abs(in.OutputDirectory)
	if err != nil {
		return err
	}
	info, err := os.Stat(root)
	if err != nil || !info.IsDir() {
		return errors.New("existing output directory required")
	}
	for _, source := range []string{in.BaselineTree, in.SourceArchive, in.TrustConfig} {
		p, err := filepath.Abs(source)
		if err != nil {
			return err
		}
		if p == root || strings.HasPrefix(root+string(os.PathSeparator), p+string(os.PathSeparator)) || strings.HasPrefix(p+string(os.PathSeparator), root+string(os.PathSeparator)) {
			return errors.New("output and source overlap")
		}
	}
	for _, name := range []string{ArchiveName, RequestName} {
		if _, err := os.Lstat(filepath.Join(root, name)); err == nil {
			return errors.New("existing operator output refused")
		} else if !errors.Is(err, os.ErrNotExist) {
			return err
		}
	}
	return nil
}

func copyExclusive(ctx context.Context, source io.ReaderAt, size int64, path, want string) (resultErr error) {
	out, err := os.OpenFile(path, os.O_WRONLY|os.O_CREATE|os.O_EXCL, 0600)
	if err != nil {
		return err
	}
	defer func() { resultErr = errors.Join(resultErr, out.Close()) }()
	h := sha256.New()
	buf := make([]byte, 64<<10)
	for off := int64(0); off < size; {
		if err := ctx.Err(); err != nil {
			return err
		}
		n := int64(len(buf))
		if size-off < n {
			n = size - off
		}
		read, err := source.ReadAt(buf[:n], off)
		if int64(read) != n || err != nil && !errors.Is(err, io.EOF) {
			return errors.New("source changed during copy")
		}
		written, err := out.Write(buf[:read])
		if err != nil || written != read {
			return errors.Join(io.ErrShortWrite, err)
		}
		h.Write(buf[:read])
		off += int64(read)
	}
	if hex.EncodeToString(h.Sum(nil)) != want {
		return errors.New("copied source differs from approved SHA-256")
	}
	return out.Sync()
}

func writeExclusive(ctx context.Context, path string, raw []byte) (resultErr error) {
	out, err := os.OpenFile(path, os.O_RDWR|os.O_CREATE|os.O_EXCL, 0600)
	if err != nil {
		return err
	}
	defer func() { resultErr = errors.Join(resultErr, out.Close()) }()
	if n, err := out.Write(raw); err != nil || n != len(raw) {
		return errors.Join(io.ErrShortWrite, err)
	}
	if err := out.Sync(); err != nil {
		return err
	}
	info, err := out.Stat()
	if err != nil || !info.Mode().IsRegular() || info.Size() != int64(len(raw)) {
		return errors.Join(errors.New("final request length differs"), err)
	}
	if err := hashReader(ctx, out, int64(len(raw)), digest(raw)); err != nil {
		return fmt.Errorf("final request bytes differ: %w", err)
	}
	return nil
}

type validationSink struct{}

func (validationSink) WriteFile(ctx context.Context, _ string, r io.Reader, size int64, want string) error {
	if size < 0 || size > maxArchiveBytes {
		return errors.New("bounded validated member required")
	}
	h := sha256.New()
	n, err := io.Copy(h, io.LimitReader(r, size+1))
	if err != nil || n != size || hex.EncodeToString(h.Sum(nil)) != want {
		return errors.New("validated member differs")
	}
	return ctx.Err()
}
func (validationSink) WriteEvidence(ctx context.Context, _ string, raw []byte, want string) error {
	if digest(raw) != want {
		return errors.New("validated evidence differs")
	}
	return ctx.Err()
}
