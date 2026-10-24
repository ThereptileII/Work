package verifier

import (
	"context"
	"net/http"
	"net/http/httptest"
	"testing"
)

const testAccessToken = "skg1_0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"

type recordingTransport struct{ seen []*http.Request }

func (r *recordingTransport) RoundTrip(request *http.Request) (*http.Response, error) {
	r.seen = append(r.seen, request)
	return httptest.NewRecorder().Result(), nil
}

func TestAccessTokenOnlyReachesTheConfiguredHTTPSOrigin(t *testing.T) {
	base := &recordingTransport{}
	transport := withAccess(base, "https://beta.skager.app", testAccessToken)
	for _, target := range []string{
		"https://beta.skager.app/updates/beta/metadata/timestamp.json",
		"https://beta.skager.app:8443/updates/x",
		"http://beta.skager.app/updates/x",
		"https://evil.example/updates/x",
		"https://beta.skager.app.evil.example/x",
	} {
		request, _ := http.NewRequest(http.MethodGet, target, nil)
		if _, err := transport.RoundTrip(request); err != nil {
			t.Fatal(err)
		}
		if request.Header.Get("Authorization") != "" {
			t.Fatal("caller request was mutated")
		}
	}
	if got := base.seen[0].Header.Get("Authorization"); got != "Bearer "+testAccessToken {
		t.Fatalf("origin did not receive token: %q", got)
	}
	for _, request := range base.seen[1:] {
		if request.Header.Get("Authorization") != "" {
			t.Fatalf("token leaked to %s", request.URL)
		}
	}
}

func TestAccessTransportIsInertWithoutAWellFormedTokenOrHTTPSOrigin(t *testing.T) {
	base := &recordingTransport{}
	for _, c := range []struct{ origin, token string }{
		{"https://beta.skager.app", ""},
		{"https://beta.skager.app", "skg1_short"},
		{"https://beta.skager.app", "Bearer " + testAccessToken},
		{"http://beta.skager.app", testAccessToken},
		{"https://user@beta.skager.app", testAccessToken},
		{"not a url", testAccessToken},
		{"https://beta.skager.app/path", testAccessToken},
		{"https://beta.skager.app?query", testAccessToken},
		{"https://beta.skager.app#fragment", testAccessToken},
	} {
		if transport := withAccess(base, c.origin, c.token); transport != http.RoundTripper(base) {
			t.Fatalf("token attached for %q/%q", c.origin, c.token)
		}
	}
	if withAccessClient(nil, PrepareConfig{ArtifactOrigin: "https://beta.skager.app"}) != nil {
		t.Fatal("client created without a token")
	}
	if metadataOrigin("https://beta.skager.app/updates/beta/metadata") != "https://beta.skager.app" {
		t.Fatal("metadata origin")
	}
}

// cloudGate stands in for the Worker: it refuses every request without the token.
type cloudGate struct {
	base     http.RoundTripper
	token    string
	refused  int
	accepted int
}

func (g *cloudGate) RoundTrip(request *http.Request) (*http.Response, error) {
	if request.Header.Get("Authorization") != "Bearer "+g.token {
		g.refused++
		response := httptest.NewRecorder()
		response.WriteHeader(http.StatusPaymentRequired)
		return response.Result(), nil
	}
	g.accepted++
	return g.base.RoundTrip(request)
}

func TestSignedCheckAndDownloadPresentTheCloudTokenToAGatedFeed(t *testing.T) {
	f := newPrepareFixture(t)
	gate := &cloudGate{base: f.client.Transport, token: testAccessToken}
	f.client.Transport = gate
	if _, err := checkUpdate(context.Background(), f.config, f.observed, f.client); err == nil || gate.accepted != 0 {
		t.Fatal("gated feed answered without a SKAGER Cloud token")
	}
	f.config.AccessToken = testAccessToken
	refused := gate.refused
	candidate := f.check(t)
	prepared, err := prepareUpdate(context.Background(), f.config, f.observed, candidate.Consent, f.client)
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { prepared.Close() })
	if gate.refused != refused || f.artifactRequests.Load() != 1 || gate.accepted < 2 {
		t.Fatalf("token not presented on every metadata and artifact request: refused %d accepted %d", gate.refused-refused, gate.accepted)
	}
}
