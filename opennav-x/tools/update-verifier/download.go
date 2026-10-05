package verifier

import (
	"context"
	"crypto/sha256"
	"encoding/hex"
	"errors"
	"io"
	"net/http"
	"net/url"
	"os"
	"path/filepath"
	"regexp"
	"strconv"
	"strings"
	"time"

	"example.com/opennav-update-verifier/releasepolicy"
)

const maxDownloadBytes = int64(4) << 30
const maxDownloadTime = 30 * time.Minute

var downloadSegment = regexp.MustCompile(`^[A-Za-z0-9][A-Za-z0-9._-]*$`)

// VerifiedArtifact owns a temporary file and a retained read-only handle. Close
// releases the handle and removes the file. Do not copy this value or call Close
// concurrently with its other methods. No method authorizes installation.
//
// The caller must protect the directory and all its ancestors against untrusted
// writes, including rename/reparse attacks, throughout acquisition and use.
// Windows retains a handle denying subsequent write/delete sharing. Portable
// Unix handles do not prevent other writers or pathname substitution. Neither
// mechanism protects against a privileged attacker. Path alone is not proof of
// custody: keep this object alive through any separately authorized launch, and
// qualify Windows sharing and execution behavior on native Windows.
type VerifiedArtifact struct {
	file    *os.File
	path    string
	removed bool
}

func (a *VerifiedArtifact) Path() string { return a.path }
func (a *VerifiedArtifact) Read(p []byte) (int, error) {
	if a.file == nil {
		return 0, os.ErrClosed
	}
	return a.file.Read(p)
}
func (a *VerifiedArtifact) ReadAt(p []byte, off int64) (int, error) {
	if a.file == nil {
		return 0, os.ErrClosed
	}
	return a.file.ReadAt(p, off)
}
func (a *VerifiedArtifact) Seek(offset int64, whence int) (int64, error) {
	if a.file == nil {
		return 0, os.ErrClosed
	}
	return a.file.Seek(offset, whence)
}

func (a *VerifiedArtifact) Close() error {
	if a.removed {
		return nil
	}
	var closeErr error
	if a.file != nil {
		err := a.file.Close()
		a.file = nil
		if err != nil {
			closeErr = errors.New("artifact handle close failed")
		}
	}
	if err := os.Remove(a.path); err != nil && !errors.Is(err, os.ErrNotExist) {
		return errors.Join(closeErr, errors.New("artifact cleanup failed"))
	}
	a.removed = true
	return closeErr
}

// DownloadArtifact acquires an artifact from an already authenticated, parsed
// release policy. This function does not authenticate policy metadata or select
// release eligibility; the caller must do both first. approvedOrigin must be a
// separately approved HTTPS origin (scheme and authority only). Redirects are
// rejected, including same-origin redirects. Policy URLs cannot carry credentials,
// queries or fragments. Errors never include response bodies, URLs or paths.
//
// The protected directory must already exist. Acquisition uses a fixed 32 KiB
// buffer, the signed exact length (at most 4 GiB), and SHA-256, with a 30-minute
// operation cap and the caller's cancellation/deadline. Failed acquisitions remove
// partial files. The closed writer is reopened read-only, its identity and bytes
// verified again, and that handle is returned positioned at the beginning.
func DownloadArtifact(ctx context.Context, artifact releasepolicy.Artifact, approvedOrigin, protectedDir string) (*VerifiedArtifact, error) {
	transport := &http.Transport{
		Proxy:                  http.ProxyFromEnvironment,
		TLSHandshakeTimeout:    10 * time.Second,
		ResponseHeaderTimeout:  30 * time.Second,
		MaxResponseHeaderBytes: 64 << 10,
		DisableCompression:     true,
		DisableKeepAlives:      true,
	}
	defer transport.CloseIdleConnections()
	return downloadArtifact(ctx, artifact, approvedOrigin, protectedDir, &http.Client{Transport: transport})
}

// Only package tests inject a client for the local HTTPS test certificate.
func downloadArtifact(ctx context.Context, artifact releasepolicy.Artifact, approvedOrigin, protectedDir string, injected *http.Client) (_ *VerifiedArtifact, resultErr error) {
	if ctx == nil {
		return nil, errors.New("artifact context required")
	}
	ctx, cancel := context.WithTimeout(ctx, maxDownloadTime)
	defer cancel()
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if err := validateDownload(artifact, approvedOrigin); err != nil {
		return nil, err
	}
	if protectedDir == "" {
		return nil, errors.New("protected artifact directory required")
	}
	dir, err := filepath.Abs(protectedDir)
	if err != nil {
		return nil, errors.New("invalid artifact directory")
	}
	info, err := os.Lstat(dir)
	if err != nil || !info.IsDir() || info.Mode()&os.ModeSymlink != 0 {
		return nil, errors.New("artifact directory must exist and not be a symlink")
	}
	writer, err := os.CreateTemp(dir, "skager-artifact-*"+filepath.Ext(artifact.Path))
	if err != nil {
		return nil, errors.New("artifact temporary file creation failed")
	}
	name := writer.Name()
	defer func() {
		_ = writer.Close()
		if resultErr != nil {
			if err := os.Remove(name); err != nil && !errors.Is(err, os.ErrNotExist) {
				resultErr = errors.Join(resultErr, errors.New("partial artifact cleanup failed"))
			}
		}
	}()
	original, err := writer.Stat()
	if err != nil {
		return nil, errors.New("artifact temporary file inspection failed")
	}
	client := *injected
	client.Jar = nil
	client.Timeout = maxDownloadTime
	client.CheckRedirect = func(*http.Request, []*http.Request) error {
		return errors.New("artifact redirects prohibited")
	}
	request, err := http.NewRequestWithContext(ctx, http.MethodGet, artifact.URL, nil)
	if err != nil {
		return nil, errors.New("invalid artifact request")
	}
	request.Header.Set("Accept-Encoding", "identity")
	response, err := client.Do(request)
	if err != nil {
		return nil, downloadError(ctx, "artifact HTTPS request failed")
	}
	defer response.Body.Close()
	if response.StatusCode != http.StatusOK {
		return nil, errors.New("artifact response must have status 200")
	}
	if response.Header.Get("Content-Encoding") != "" && response.Header.Get("Content-Encoding") != "identity" {
		return nil, errors.New("artifact response encoding prohibited")
	}
	if response.ContentLength >= 0 && response.ContentLength != artifact.Bytes {
		return nil, errors.New("artifact response length differs from signed size")
	}
	if err := copyVerifiedDownload(ctx, response.Body, writer, artifact); err != nil {
		return nil, err
	}
	if err := writer.Sync(); err != nil {
		return nil, errors.New("artifact sync failed")
	}
	if err := writer.Close(); err != nil {
		return nil, errors.New("artifact writer close failed")
	}
	reader, err := openDownloadedArtifact(name)
	if err != nil {
		return nil, errors.New("artifact read-only custody failed")
	}
	defer func() {
		if resultErr != nil {
			_ = reader.Close()
		}
	}()
	sealed, err := reader.Stat()
	if err != nil || !sealed.Mode().IsRegular() || !os.SameFile(original, sealed) || sealed.Size() != artifact.Bytes {
		return nil, errors.New("artifact file identity or size changed")
	}
	if err := copyVerifiedDownload(ctx, reader, io.Discard, artifact); err != nil {
		return nil, err
	}
	if _, err := reader.Seek(0, io.SeekStart); err != nil {
		return nil, errors.New("artifact rewind failed")
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	return &VerifiedArtifact{file: reader, path: name}, nil
}

func downloadError(ctx context.Context, message string) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	return errors.New(message)
}

type downloadContextReader struct {
	ctx    context.Context
	reader io.Reader
}

func (r downloadContextReader) Read(p []byte) (int, error) {
	if err := r.ctx.Err(); err != nil {
		return 0, err
	}
	return r.reader.Read(p)
}

func copyVerifiedDownload(ctx context.Context, source io.Reader, destination io.Writer, artifact releasepolicy.Artifact) error {
	hash := sha256.New()
	limited := io.LimitReader(downloadContextReader{ctx: ctx, reader: source}, artifact.Bytes+1)
	n, err := io.CopyBuffer(io.MultiWriter(destination, hash), limited, make([]byte, 32<<10))
	if err != nil {
		return downloadError(ctx, "artifact transfer failed")
	}
	if n != artifact.Bytes {
		return errors.New("artifact length differs from signed size")
	}
	if hex.EncodeToString(hash.Sum(nil)) != artifact.SHA256 {
		return errors.New("artifact SHA-256 mismatch")
	}
	return ctx.Err()
}

func validateDownload(a releasepolicy.Artifact, origin string) error {
	if len(origin) > 2048 || strings.Contains(origin, "#") || len(a.URL) > 2304 || strings.Contains(a.URL, "#") {
		return errors.New("artifact URL or origin has invalid length or fragment")
	}
	u, err := url.Parse(origin)
	if err != nil || u == nil || u.Scheme != "https" || u.Hostname() == "" || u.User != nil || u.Path != "" || u.RawQuery != "" || u.ForceQuery || u.Fragment != "" || u.Opaque != "" {
		return errors.New("approved artifact origin must be HTTPS scheme and authority only")
	}
	if strings.ContainsAny(u.Host, "\\ \t\n\r") || strings.HasSuffix(u.Host, ":") {
		return errors.New("invalid approved artifact origin authority")
	}
	if u.Port() != "" {
		port, err := strconv.Atoi(u.Port())
		if err != nil || port < 1 || port > 65535 {
			return errors.New("invalid approved artifact origin port")
		}
	}
	if a.Bytes <= 0 || a.Bytes > maxDownloadBytes || len(a.SHA256) != 64 || strings.ToLower(a.SHA256) != a.SHA256 {
		return errors.New("invalid artifact size or SHA-256")
	}
	if _, err := hex.DecodeString(a.SHA256); err != nil {
		return errors.New("invalid artifact SHA-256")
	}
	if len(a.Path) == 0 || len(a.Path) > 240 {
		return errors.New("invalid artifact path")
	}
	for _, segment := range strings.Split(a.Path, "/") {
		base := strings.ToUpper(strings.SplitN(segment, ".", 2)[0])
		reserved := base == "CON" || base == "PRN" || base == "AUX" || base == "NUL" || len(base) == 4 && (strings.HasPrefix(base, "COM") || strings.HasPrefix(base, "LPT")) && base[3] >= '1' && base[3] <= '9'
		if !downloadSegment.MatchString(segment) || strings.HasSuffix(segment, ".") || reserved {
			return errors.New("unsafe artifact path")
		}
	}
	target, err := url.Parse(a.URL)
	if err != nil || target == nil || target.Scheme != "https" || target.Host != u.Host || target.User != nil || target.RawQuery != "" || target.ForceQuery || target.Fragment != "" || target.RawPath != "" || target.Path != "/"+a.Path || target.Opaque != "" {
		return errors.New("artifact URL must match approved HTTPS origin and signed path")
	}
	return nil
}
