// Package verifier is an isolated, verify-only TUF integration probe.
package verifier

import (
	"bytes"
	"context"
	"errors"
	"fmt"
	"net/http"
	"net/url"
	"os"
	"path"
	"path/filepath"
	"regexp"
	"strings"
	"sync/atomic"
	"time"

	"github.com/theupdateframework/go-tuf/v2/metadata"
	"github.com/theupdateframework/go-tuf/v2/metadata/config"
	"github.com/theupdateframework/go-tuf/v2/metadata/trustedmetadata"
	"github.com/theupdateframework/go-tuf/v2/metadata/updater"
)

const maxTargetBytes = 32 << 10
const maxOperationTime = 30 * time.Second
const maxTrustedRootBytes = 512 << 10

// ErrAuthenticatedTargetAbsent is returned only after a successful live TUF
// refresh proves the target absent from signed top-level metadata with no
// delegations. Network, expiry, rollback, delegation and target-download errors
// never mean that an optional update is absent.
var ErrAuthenticatedTargetAbsent = errors.New("target absent from authenticated nondelegated metadata")

type metadataLimitKey struct{}

type deadlineTransport struct {
	base      http.RoundTripper
	deadline  time.Time
	remaining *atomic.Int64
}

type boundedMetadataBody struct {
	ioReadCloser
	left  int64
	total *atomic.Int64
}

func (b *boundedMetadataBody) Read(p []byte) (int, error) {
	limit := b.left
	if b.total != nil && b.total.Load() < limit {
		limit = b.total.Load()
	}
	if limit < 0 {
		return 0, errors.New("update response length exceeds hard limit")
	}
	if int64(len(p)) > limit+1 {
		p = p[:int(limit+1)]
	}
	n, err := b.ioReadCloser.Read(p)
	b.left -= int64(n)
	remaining := int64(1)
	if b.total != nil {
		remaining = b.total.Add(-int64(n))
	}
	if b.left < 0 || remaining < 0 {
		return n, errors.New("update response length exceeds hard limit")
	}
	return n, err
}

func metadataResponseLimit(u *url.URL) int64 {
	name := path.Base(u.Path)
	switch {
	case strings.HasSuffix(name, "root.json"):
		return 512 << 10
	case name == "timestamp.json":
		return 16 << 10
	case strings.HasSuffix(name, "snapshot.json"):
		return 2 << 20
	case strings.Contains(u.Path, "/targets/releases/"), strings.Contains(u.Path, "/targets/bootstrap/"):
		return maxTargetBytes
	default:
		return 5 << 20
	}
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
	limit := metadataResponseLimit(req.URL)
	if initial, ok := req.Context().Value(metadataLimitKey{}).(int64); ok && initial < limit {
		limit = initial
	}
	if res.ContentLength > limit {
		res.Body.Close()
		cancel()
		return nil, errors.New("update response length exceeds hard limit")
	}
	res.Body = &cancelBody{ioReadCloser: &boundedMetadataBody{ioReadCloser: res.Body, left: limit, total: t.remaining}, cancel: cancel}
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
	return verifyWithClientAt(req, deadline, injected, time.Time{})
}

// referenceTime is a per-call test clock only. Production wrappers leave it zero
// so go-tuf selects its real clock; transport deadlines are always independent.
func verifyWithClientAt(req Request, deadline time.Time, injected *http.Client, referenceTime time.Time) (Release, []byte, error) {
	var release Release
	if _, err := validateTUFRequest(req); err != nil {
		return Release{}, nil, err
	}
	if req.TargetPath != "releases/"+req.Channel+".json" {
		return Release{}, nil, errors.New("target path must match selected channel")
	}
	data, err := verifyTargetWithClientAt(req, deadline, injected, false, func(custom []byte) error {
		var err error
		release, err = parseReleaseIdentity(custom)
		if err != nil {
			return fmt.Errorf("signed release identity: %w", err)
		}
		if release.Channel != req.Channel || release.Version == "" || len(release.Version) > 128 || !commitPattern.MatchString(release.Commit) {
			return errors.New("invalid signed release identity")
		}
		return nil
	}, referenceTime)
	if err != nil {
		return Release{}, nil, err
	}
	return release, data, nil
}

func validateTUFRequest(req Request) (*url.URL, error) {
	u, err := url.Parse(req.MetadataURL)
	if err != nil || u == nil || u.Scheme != "https" || u.Host == "" || u.User != nil || u.RawQuery != "" || u.Fragment != "" {
		return nil, errors.New("metadata URL must be absolute HTTPS without credentials or query")
	}
	if req.Channel != "beta" && req.Channel != "stable" {
		return nil, errors.New("unknown release channel")
	}
	if req.CacheDir == "" || len(req.TrustedRoot) == 0 {
		return nil, errors.New("trusted root and durable cache required")
	}
	return u, nil
}

// verifyTargetWithClient is shared only by the two purpose-specific entry points.
// They select the exact path and validate their own signed custom identity before
// go-tuf downloads and verifies any target bytes. It grants no artifact authority.
func verifyTargetWithClient(req Request, deadline time.Time, injected *http.Client, rejectRedirects bool, validateIdentity func([]byte) error) ([]byte, error) {
	return verifyTargetWithClientAt(req, deadline, injected, rejectRedirects, validateIdentity, time.Time{})
}

// The protected cache's retained root is the current authority, while the
// configured bootstrap remains its independently approved original identity.
// go-tuf New persists the supplied root immediately: supplying the bootstrap on
// every call would overwrite a previously verified rotation before Refresh.
// The caller must authenticate/protect this SAME cache and its history; a valid
// self-signature or current ACL alone cannot establish cache provenance.
func retainedTUFRoot(req Request) ([]byte, error) {
	if len(req.TrustedRoot) > maxTrustedRootBytes {
		return nil, errors.New("configured trusted root exceeds hard limit")
	}
	anchor, err := trustedmetadata.New(req.TrustedRoot)
	if err != nil {
		return nil, errors.New("configured trusted root is invalid")
	}
	_, err = os.Lstat(req.CacheDir)
	if os.IsNotExist(err) {
		// Low-level probes may establish a genuinely new absent cache. Native
		// installed/operator callers already hold an existing protected cache,
		// which must have been explicitly seeded before invoking this verifier.
		return bytes.Clone(req.TrustedRoot), nil
	}
	if err != nil || plainStatePath(req.CacheDir) != nil || plainStatePath(filepath.Join(req.CacheDir, "metadata")) != nil {
		return nil, errors.New("retained TUF cache unavailable; explicit recovery required")
	}
	raw, err := boundedStateRead(filepath.Join(req.CacheDir, "metadata", "root.json"), maxTrustedRootBytes)
	if err != nil {
		return nil, errors.New("retained TUF root unavailable; explicit recovery required")
	}
	retained, err := trustedmetadata.New(raw)
	if err != nil || retained.Root.Signed.Version < anchor.Root.Signed.Version {
		return nil, errors.New("retained TUF root invalid or below configured anchor")
	}
	if retained.Root.Signed.Version == anchor.Root.Signed.Version && !bytes.Equal(raw, req.TrustedRoot) {
		return nil, errors.New("retained TUF root differs at configured anchor version")
	}
	// Do not reject an expired intermediate here. go-tuf must retain its normal
	// forward rotation and final-current-root expiry checks at the actual clock.
	return raw, nil
}

func verifyTargetWithClientAt(req Request, deadline time.Time, injected *http.Client, rejectRedirects bool, validateIdentity func([]byte) error, referenceTime time.Time) ([]byte, error) {
	u, err := validateTUFRequest(req)
	if err != nil {
		return nil, err
	}
	root, err := retainedTUFRoot(req)
	if err != nil {
		return nil, err
	}
	cfg, err := config.New(strings.TrimRight(req.MetadataURL, "/"), root)
	if err != nil {
		return nil, err
	}
	cfg.LocalMetadataDir = filepath.Join(req.CacheDir, "metadata")
	cfg.LocalTargetsDir = filepath.Join(req.CacheDir, "targets")
	cfg.RootMaxLength = maxTrustedRootBytes
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
	budget := &atomic.Int64{}
	budget.Store(32 << 20)
	client.Transport = deadlineTransport{base: base, deadline: deadline, remaining: budget}
	client.CheckRedirect = func(next *http.Request, via []*http.Request) error {
		if rejectRedirects {
			return errors.New("bootstrap metadata and target redirects prohibited")
		}
		if next.URL.Scheme != "https" || next.URL.Host != u.Host || len(via) >= 5 {
			return errors.New("metadata redirect outside HTTPS origin")
		}
		initial := metadataResponseLimit(via[0].URL)
		*next = *next.WithContext(context.WithValue(next.Context(), metadataLimitKey{}, initial))
		return nil
	}
	if err := cfg.SetDefaultFetcherHTTPClient(client); err != nil {
		return nil, err
	}
	if err := cfg.SetDefaultFetcherRetry(time.Millisecond*100, 1); err != nil {
		return nil, err
	}
	up, err := updater.New(cfg)
	if err != nil {
		return nil, fmt.Errorf("trusted root: %w", err)
	}
	if !referenceTime.IsZero() {
		// go-tuf documents this hook for tests. No exported API or production
		// caller supplies a clock override, and all expiry checks still run.
		up.UnsafeSetRefTime(referenceTime)
	}
	if err := up.Refresh(); err != nil {
		return nil, fmt.Errorf("TUF refresh: %w", err)
	}
	trusted := up.GetTrustedMetadataSet()
	if top := trusted.Targets[metadata.TARGETS]; top != nil && top.Signed.Delegations == nil {
		if _, present := top.Signed.Targets[req.TargetPath]; !present {
			return nil, ErrAuthenticatedTargetAbsent
		}
	}
	target, err := up.GetTargetInfo(req.TargetPath)
	if err != nil || target == nil {
		return nil, fmt.Errorf("target metadata: %w", err)
	}
	if target.Length < 0 || target.Length > maxTargetBytes {
		return nil, errors.New("target exceeds size limit")
	}
	if target.Custom == nil {
		return nil, errors.New("target has no signed release identity")
	}
	if err := validateIdentity(*target.Custom); err != nil {
		return nil, err
	}
	_, data, err := up.DownloadTarget(target, "", "")
	if err != nil {
		return nil, fmt.Errorf("target verification: %w", err)
	}
	return data, nil
}
