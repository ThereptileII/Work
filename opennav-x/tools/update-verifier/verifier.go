// Package verifier is an isolated, verify-only TUF integration probe.
package verifier

import (
	"context"
	"errors"
	"fmt"
	"net/http"
	"net/url"
	"path/filepath"
	"regexp"
	"strings"
	"time"

	"github.com/theupdateframework/go-tuf/v2/metadata/config"
	"github.com/theupdateframework/go-tuf/v2/metadata/updater"
)

const maxTargetBytes = 64 << 20
const maxOperationTime = 30 * time.Second

type deadlineTransport struct {
	base     http.RoundTripper
	deadline time.Time
}
type cancelBody struct {
	ioReadCloser
	cancel context.CancelFunc
}
type ioReadCloser interface {
	Read([]byte) (int, error)
	Close() error
}

func (b *cancelBody) Close() error { b.cancel(); return b.ioReadCloser.Close() }
func (t deadlineTransport) RoundTrip(req *http.Request) (*http.Response, error) {
	ctx, cancel := context.WithDeadline(req.Context(), t.deadline)
	res, err := t.base.RoundTrip(req.WithContext(ctx))
	if err != nil {
		cancel()
		return nil, err
	}
	res.Body = &cancelBody{ioReadCloser: res.Body, cancel: cancel}
	return res, nil
}

var commitPattern = regexp.MustCompile(`^[0-9a-f]{40}$`)

// Release identity is signed as target custom metadata and bound to target bytes.
type Release struct {
	Channel string `json:"channel"`
	Version string `json:"version"`
	Commit  string `json:"commit"`
}

type Request struct {
	MetadataURL string
	TrustedRoot []byte // pinned out of band; never fetched by this package
	CacheDir    string // durable, protected local metadata cache for rollback checks
	Channel     string
	TargetPath  string
}

// Verify refreshes TUF metadata and returns a verified target and its signed identity.
// No returned bytes are executed or installed. The caller must retain a trusted cache.
func Verify(req Request) (Release, []byte, error) {
	return verifyWithClient(req, time.Now().Add(maxOperationTime), nil)
}

// Only tests inject a client, to trust their local TLS certificate.
func verifyWithClient(req Request, deadline time.Time, injected *http.Client) (Release, []byte, error) {
	var empty Release
	u, err := url.Parse(req.MetadataURL)
	if err != nil || u == nil || u.Scheme != "https" || u.Host == "" || u.User != nil || u.RawQuery != "" || u.Fragment != "" {
		return empty, nil, errors.New("metadata URL must be absolute HTTPS without credentials or query")
	}
	if req.Channel != "beta" && req.Channel != "stable" {
		return empty, nil, errors.New("unknown release channel")
	}
	if req.CacheDir == "" || len(req.TrustedRoot) == 0 {
		return empty, nil, errors.New("trusted root and durable cache required")
	}
	if req.TargetPath != "releases/"+req.Channel+".json" {
		return empty, nil, errors.New("target path must match selected channel")
	}
	cfg, err := config.New(strings.TrimRight(req.MetadataURL, "/"), req.TrustedRoot)
	if err != nil {
		return empty, nil, err
	}
	cfg.LocalMetadataDir = filepath.Join(req.CacheDir, "metadata")
	cfg.LocalTargetsDir = filepath.Join(req.CacheDir, "targets")
	cfg.RootMaxLength = 512 << 10
	cfg.TimestampMaxLength = 16 << 10
	cfg.SnapshotMaxLength = 2 << 20
	cfg.TargetsMaxLength = 5 << 20
	cfg.MaxRootRotations = 32
	cfg.MaxDelegations = 16
	cfg.PrefixTargetsWithHash = true
	client := &http.Client{Timeout: 15 * time.Second}
	if injected != nil {
		*client = *injected
		client.Timeout = 15 * time.Second
	}
	base := client.Transport
	if base == nil {
		base = http.DefaultTransport
	}
	client.Transport = deadlineTransport{base: base, deadline: deadline}
	client.CheckRedirect = func(next *http.Request, via []*http.Request) error {
		if next.URL.Scheme != "https" || next.URL.Host != u.Host || len(via) >= 5 {
			return errors.New("metadata redirect outside HTTPS origin")
		}
		return nil
	}
	if err := cfg.SetDefaultFetcherHTTPClient(client); err != nil {
		return empty, nil, err
	}
	if err := cfg.SetDefaultFetcherRetry(time.Millisecond*100, 1); err != nil {
		return empty, nil, err
	}
	up, err := updater.New(cfg)
	if err != nil {
		return empty, nil, fmt.Errorf("trusted root: %w", err)
	}
	if err := up.Refresh(); err != nil {
		return empty, nil, fmt.Errorf("TUF refresh: %w", err)
	}
	target, err := up.GetTargetInfo(req.TargetPath)
	if err != nil || target == nil {
		return empty, nil, fmt.Errorf("target metadata: %w", err)
	}
	if target.Length < 0 || target.Length > maxTargetBytes {
		return empty, nil, errors.New("target exceeds size limit")
	}
	if target.Custom == nil {
		return empty, nil, errors.New("target has no signed release identity")
	}
	release, err := parseReleaseIdentity(*target.Custom)
	if err != nil {
		return empty, nil, fmt.Errorf("signed release identity: %w", err)
	}
	if release.Channel != req.Channel || release.Version == "" || len(release.Version) > 128 || !commitPattern.MatchString(release.Commit) {
		return empty, nil, errors.New("invalid signed release identity")
	}
	_, data, err := up.DownloadTarget(target, "", "")
	if err != nil {
		return empty, nil, fmt.Errorf("target verification: %w", err)
	}
	return release, data, nil
}
