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

func produce(ctx context.Context, in Input, stockCheck func(string) error) (Result, error) {
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
	trust, err := os.ReadFile(in.TrustConfig)
	if err != nil || len(trust) == 0 || len(trust) > brokerregistration.MaxTrustConfigurationBytes ||
		!bytes.Equal(bytes.TrimSpace(trust), trust) || digest(trust) != in.ApprovedTrustSHA256 {
		return empty, errors.New("trust bytes differ from independent approval")
	}
	source, err := os.Open(in.SourceArchive)
	if err != nil {
		return empty, err
	}
	defer source.Close()
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
	final, err := os.Open(archivePath)
	if err != nil {
		return empty, err
	}
	if err := hashReader(ctx, final, info.Size(), in.ApprovedSourceSHA256); err != nil {
		final.Close()
		return empty, fmt.Errorf("final source bytes differ: %w", err)
	}
	if _, err = verifier.ImportApprovedBaseline(ctx, raw, requestSHA, final, info.Size(), validationSink{}); err != nil {
		final.Close()
		return empty, fmt.Errorf("final baseline importer refused: %w", err)
	}
	if err := final.Close(); err != nil {
		return empty, err
	}
	if err := compareTree(ctx, source, info.Size(), in.BaselineTree); err != nil {
		return empty, err
	}
	if err := stockCheck(in.StockPath); err != nil {
		return empty, fmt.Errorf("stock changed during handoff: %w", err)
	}
	if err := writeExclusive(requestPath, raw); err != nil {
		return empty, err
	}
	return Result{ArchivePath: archivePath, RequestPath: requestPath, RequestSHA256: requestSHA}, nil
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

func observeStock(path string) error {
	if err := plainExisting(path); err != nil {
		return err
	}
	f, err := os.Open(path)
	if err != nil {
		return err
	}
	defer f.Close()
	info, err := f.Stat()
	if err != nil || !info.Mode().IsRegular() || info.Size() < 1 || info.Size() > maxArchiveBytes {
		return errors.New("bounded regular stock executable required")
	}
	return hashReader(context.Background(), f, info.Size(), brokerregistration.StockBaseline().ExecutableSHA256)
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
			input.Close()
			return err
		}
		a, b := sha256.New(), sha256.New()
		na, ea := io.Copy(a, input)
		nb, eb := io.Copy(b, reader)
		closeErr := errors.Join(input.Close(), reader.Close())
		if ea != nil || eb != nil || closeErr != nil || na != nb || uint64(na) != member.UncompressedSize64 || !equalHash(a.Sum(nil), b.Sum(nil)) {
			return errors.New("baseline tree differs from source archive")
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

func equalHash(a, b []byte) bool { return hex.EncodeToString(a) == hex.EncodeToString(b) }

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

func writeExclusive(path string, raw []byte) (resultErr error) {
	out, err := os.OpenFile(path, os.O_WRONLY|os.O_CREATE|os.O_EXCL, 0600)
	if err != nil {
		return err
	}
	defer func() { resultErr = errors.Join(resultErr, out.Close()) }()
	if n, err := out.Write(raw); err != nil || n != len(raw) {
		return errors.Join(io.ErrShortWrite, err)
	}
	return out.Sync()
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
