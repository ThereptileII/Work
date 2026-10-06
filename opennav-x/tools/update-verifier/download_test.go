package verifier

import (
	"context"
	"crypto/sha256"
	"encoding/hex"
	"errors"
	"fmt"
	"io"
	"net/http"
	"net/http/httptest"
	"os"
	"path/filepath"
	"strings"
	"sync/atomic"
	"testing"
	"time"

	"example.com/opennav-update-verifier/releasepolicy"
)

func downloadTestArtifact(origin, body string) releasepolicy.Artifact {
	hash := sha256.Sum256([]byte(body))
	return releasepolicy.Artifact{Path: "releases/installer.exe", URL: origin + "/releases/installer.exe", SHA256: hex.EncodeToString(hash[:]), Bytes: int64(len(body))}
}

func assertDownloadDirectoryEmpty(t *testing.T, dir string) {
	t.Helper()
	entries, err := os.ReadDir(dir)
	if err != nil || len(entries) != 0 {
		t.Fatalf("partial artifact retained: entries=%v error=%v", entries, err)
	}
}

func TestDownloadArtifactRetainsVerifiedReadOnlyFile(t *testing.T) {
	body := strings.Repeat("installer bytes\x00", 8000)
	server := httptest.NewTLSServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		if r.Header.Get("Accept-Encoding") != "identity" || r.URL.Path != "/releases/installer.exe" {
			t.Error("unexpected artifact request")
		}
		w.Header().Set("Content-Length", fmt.Sprint(len(body)))
		io.WriteString(w, body)
	}))
	defer server.Close()
	dir := t.TempDir()
	result, err := downloadArtifact(context.Background(), downloadTestArtifact(server.URL, body), server.URL, dir, server.Client())
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { result.Close() })
	if filepath.Dir(result.Path()) != dir {
		t.Fatal("artifact escaped protected directory")
	}
	got, err := io.ReadAll(result)
	if err != nil || string(got) != body {
		t.Fatalf("verified bytes mismatch: %v", err)
	}
	if _, err := result.file.Write([]byte("tamper")); err == nil {
		t.Fatal("retained handle is write-capable")
	}
	if err := result.Close(); err != nil {
		t.Fatal(err)
	}
	if _, err := result.Read(make([]byte, 1)); !errors.Is(err, os.ErrClosed) {
		t.Fatalf("closed handle remains readable: %v", err)
	}
	if err := os.WriteFile(result.Path(), []byte("later unrelated file"), 0600); err != nil {
		t.Fatal(err)
	}
	if err := result.Close(); err != nil {
		t.Fatalf("repeated close: %v", err)
	}
	if _, err := os.Stat(result.Path()); err != nil {
		t.Fatal("repeated close deleted a later file")
	}
	os.Remove(result.Path())
	assertDownloadDirectoryEmpty(t, dir)
}

func TestDownloadRejectsMalformedDeclarationsBeforeNetwork(t *testing.T) {
	var calls atomic.Int32
	server := httptest.NewTLSServer(http.HandlerFunc(func(w http.ResponseWriter, _ *http.Request) { calls.Add(1) }))
	defer server.Close()
	cases := []struct {
		name   string
		change func(*releasepolicy.Artifact, *string)
	}{
		{"http", func(a *releasepolicy.Artifact, o *string) { a.URL = strings.Replace(a.URL, "https:", "http:", 1) }},
		{"credential", func(a *releasepolicy.Artifact, o *string) {
			a.URL = strings.Replace(a.URL, "https://", "https://user:top-secret@", 1)
		}},
		{"query", func(a *releasepolicy.Artifact, o *string) { a.URL += "?token=top-secret" }},
		{"empty query", func(a *releasepolicy.Artifact, o *string) { a.URL += "?" }},
		{"fragment", func(a *releasepolicy.Artifact, o *string) { a.URL += "#top-secret" }},
		{"empty fragment", func(a *releasepolicy.Artifact, o *string) { a.URL += "#" }},
		{"escaped path", func(a *releasepolicy.Artifact, o *string) {
			a.URL = strings.Replace(a.URL, "installer", "%69nstaller", 1)
		}},
		{"path mismatch", func(a *releasepolicy.Artifact, o *string) { a.Path = "different.exe" }},
		{"traversal", func(a *releasepolicy.Artifact, o *string) { a.Path = "../installer.exe"; a.URL = *o + "/" + a.Path }},
		{"windows device", func(a *releasepolicy.Artifact, o *string) { a.Path = "NUL.exe"; a.URL = *o + "/" + a.Path }},
		{"negative size", func(a *releasepolicy.Artifact, o *string) { a.Bytes = -1 }},
		{"zero size", func(a *releasepolicy.Artifact, o *string) { a.Bytes = 0 }},
		{"oversize declaration", func(a *releasepolicy.Artifact, o *string) { a.Bytes = maxDownloadBytes + 1 }},
		{"malformed hash", func(a *releasepolicy.Artifact, o *string) { a.SHA256 = strings.Repeat("x", 64) }},
		{"uppercase hash", func(a *releasepolicy.Artifact, o *string) { a.SHA256 = strings.ToUpper(a.SHA256) }},
		{"different origin", func(a *releasepolicy.Artifact, o *string) { *o = "https://other.invalid" }},
		{"origin credentials", func(a *releasepolicy.Artifact, o *string) { *o = "https://user:top-secret@example.test" }},
		{"origin path", func(a *releasepolicy.Artifact, o *string) { *o += "/" }},
		{"origin fragment", func(a *releasepolicy.Artifact, o *string) { *o += "#" }},
		{"origin port", func(a *releasepolicy.Artifact, o *string) {
			*o = "https://example.test:65536"
			a.URL = *o + "/" + a.Path
		}},
		{"origin query", func(a *releasepolicy.Artifact, o *string) { *o += "?token=top-secret" }},
		{"origin http", func(a *releasepolicy.Artifact, o *string) { *o = "http://example.test" }},
		{"invalid URL", func(a *releasepolicy.Artifact, o *string) { a.URL = "%top-secret" }},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			dir := t.TempDir()
			a := downloadTestArtifact(server.URL, "expected bytes")
			origin := server.URL
			tc.change(&a, &origin)
			result, err := downloadArtifact(context.Background(), a, origin, dir, server.Client())
			if err == nil || result != nil {
				t.Fatal("malformed declaration accepted")
			}
			if strings.Contains(err.Error(), "top-secret") {
				t.Fatal("error leaked URL credential")
			}
			assertDownloadDirectoryEmpty(t, dir)
		})
	}
	if calls.Load() != 0 {
		t.Fatal("invalid declaration reached network")
	}
}

func TestDownloadAcceptsExactChunkedPayload(t *testing.T) {
	body := "signed chunked artifact"
	server := httptest.NewTLSServer(http.HandlerFunc(func(w http.ResponseWriter, _ *http.Request) {
		w.(http.Flusher).Flush()
		io.WriteString(w, body)
	}))
	defer server.Close()
	dir := t.TempDir()
	result, err := downloadArtifact(context.Background(), downloadTestArtifact(server.URL, body), server.URL, dir, server.Client())
	if err != nil {
		t.Fatal(err)
	}
	if err := result.Close(); err != nil {
		t.Fatal(err)
	}
	assertDownloadDirectoryEmpty(t, dir)
}

func TestDownloadRejectsBadResponseAndCleansPartialData(t *testing.T) {
	for _, mode := range []string{"hash", "oversize chunked", "truncated chunked", "truncated content length", "wrong content length", "encoding", "status", "same origin redirect", "credential redirect", "redirect loop"} {
		t.Run(mode, func(t *testing.T) {
			body := "expected bytes"
			var calls atomic.Int32
			server := httptest.NewTLSServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
				calls.Add(1)
				switch mode {
				case "hash":
					io.WriteString(w, "corrupted data")
				case "oversize chunked":
					w.(http.Flusher).Flush()
					io.WriteString(w, body+"extra")
				case "truncated chunked":
					w.(http.Flusher).Flush()
					io.WriteString(w, "short")
				case "truncated content length":
					w.Header().Set("Content-Length", fmt.Sprint(len(body)))
					io.WriteString(w, "short")
				case "wrong content length":
					w.Header().Set("Content-Length", "1")
					io.WriteString(w, "x")
				case "encoding":
					w.Header().Set("Content-Encoding", "gzip")
					io.WriteString(w, body)
				case "status":
					w.WriteHeader(http.StatusUnauthorized)
					io.WriteString(w, "top-secret response")
				case "same origin redirect":
					http.Redirect(w, r, "/elsewhere", http.StatusFound)
				case "credential redirect":
					http.Redirect(w, r, "https://user:top-secret@outside.invalid/file?token=secret", http.StatusFound)
				case "redirect loop":
					http.Redirect(w, r, r.URL.Path, http.StatusFound)
				}
			}))
			defer server.Close()
			dir := t.TempDir()
			result, err := downloadArtifact(context.Background(), downloadTestArtifact(server.URL, body), server.URL, dir, server.Client())
			if err == nil || result != nil {
				t.Fatal("bad response accepted")
			}
			if strings.Contains(err.Error(), "secret") || strings.Contains(err.Error(), server.URL) {
				t.Fatalf("unsanitized error: %v", err)
			}
			if calls.Load() != 1 {
				t.Fatal("followed rejected redirect")
			}
			assertDownloadDirectoryEmpty(t, dir)
		})
	}
}

func TestDownloadNeverFollowsCrossOriginRedirect(t *testing.T) {
	var escaped atomic.Bool
	other := httptest.NewTLSServer(http.HandlerFunc(func(http.ResponseWriter, *http.Request) { escaped.Store(true) }))
	defer other.Close()
	server := httptest.NewTLSServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) { http.Redirect(w, r, other.URL+"/file", http.StatusFound) }))
	defer server.Close()
	dir := t.TempDir()
	_, err := downloadArtifact(context.Background(), downloadTestArtifact(server.URL, "bytes"), server.URL, dir, server.Client())
	if err == nil || escaped.Load() {
		t.Fatal("cross-origin redirect accepted")
	}
	assertDownloadDirectoryEmpty(t, dir)
}

func TestDownloadCancellationAndDeadlineCleanPartialData(t *testing.T) {
	for _, mode := range []string{"canceled", "deadline"} {
		t.Run(mode, func(t *testing.T) {
			started := make(chan struct{})
			server := httptest.NewTLSServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
				w.(http.Flusher).Flush()
				io.WriteString(w, "part")
				w.(http.Flusher).Flush()
				close(started)
				<-r.Context().Done()
			}))
			defer server.Close()
			var ctx context.Context
			var cancel context.CancelFunc
			expected := context.Canceled
			if mode == "deadline" {
				ctx, cancel = context.WithTimeout(context.Background(), time.Second)
				expected = context.DeadlineExceeded
			} else {
				ctx, cancel = context.WithCancel(context.Background())
			}
			defer cancel()
			dir := t.TempDir()
			done := make(chan error, 1)
			go func() {
				_, err := downloadArtifact(ctx, downloadTestArtifact(server.URL, "partial payload"), server.URL, dir, server.Client())
				done <- err
			}()
			select {
			case <-started:
			case <-time.After(5 * time.Second):
				t.Fatal("request did not start")
			}
			if mode == "canceled" {
				cancel()
			}
			select {
			case err := <-done:
				if !errors.Is(err, expected) {
					t.Fatalf("cancellation error: got %v want %v", err, expected)
				}
			case <-time.After(5 * time.Second):
				t.Fatal("download did not stop")
			}
			assertDownloadDirectoryEmpty(t, dir)
		})
	}
}

func TestDownloadMissingDirectoryAndCanceledContext(t *testing.T) {
	dir := t.TempDir()
	a := downloadTestArtifact("https://example.invalid", "bytes")
	ctx, cancel := context.WithCancel(context.Background())
	cancel()
	if _, err := DownloadArtifact(ctx, a, "https://example.invalid", dir); !errors.Is(err, context.Canceled) {
		t.Fatalf("canceled context: %v", err)
	}
	if _, err := DownloadArtifact(context.Background(), a, "https://example.invalid", filepath.Join(dir, "missing")); err == nil {
		t.Fatal("created missing protected directory")
	}
	assertDownloadDirectoryEmpty(t, dir)
}

type cancelAtEOFReader struct{ cancel context.CancelFunc }

func (r cancelAtEOFReader) Read(p []byte) (int, error) {
	r.cancel()
	return copy(p, "part"), io.EOF
}

func TestDownloadCancellationAtTransportEOF(t *testing.T) {
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	err := copyVerifiedDownload(ctx, cancelAtEOFReader{cancel}, io.Discard,
		downloadTestArtifact("https://example.invalid", "partial payload"))
	if !errors.Is(err, context.Canceled) {
		t.Fatalf("EOF racing cancellation: got %v want context canceled", err)
	}
}
