package verifier

import (
	"context"
	"net/http"
	"net/url"
	"regexp"

	"example.com/opennav-update-verifier/releasepolicy"
)

// AccessTokenPattern is the SKAGER Cloud device token issued by app pairing.
var AccessTokenPattern = regexp.MustCompile(`^skg1_[a-f0-9]{64}$`)

// accessTransport presents the SKAGER Cloud device token to exactly one HTTPS
// origin. The token is download permission only: TUF metadata signatures and the
// signed artifact length and SHA-256 still decide what is trusted, and every other
// origin (including any redirect target) receives no Authorization header.
type accessTransport struct {
	base   http.RoundTripper
	origin string
	token  string
}

func (t accessTransport) RoundTrip(request *http.Request) (*http.Response, error) {
	if request.URL == nil || request.URL.Scheme != "https" || request.URL.User != nil ||
		request.URL.Scheme+"://"+request.URL.Host != t.origin {
		return t.base.RoundTrip(request)
	}
	clone := request.Clone(request.Context())
	clone.Header.Set("Authorization", "Bearer "+t.token)
	return t.base.RoundTrip(clone)
}

// withAccess returns base unchanged unless token is a well-formed device token
// and origin is a bare HTTPS scheme and authority.
func withAccess(base http.RoundTripper, origin, token string) http.RoundTripper {
	if base == nil {
		base = http.DefaultTransport
	}
	if !AccessTokenPattern.MatchString(token) {
		return base
	}
	parsed, err := url.Parse(origin)
	if err != nil || parsed.Scheme != "https" || parsed.Host == "" || parsed.User != nil {
		return base
	}
	return accessTransport{base: base, origin: parsed.Scheme + "://" + parsed.Host, token: token}
}

// acquireArtifact downloads one artifact of an authenticated candidate, presenting
// the configured access token to the installed artifact origin only.
func acquireArtifact(ctx context.Context, artifact releasepolicy.Artifact, config PrepareConfig, injected *http.Client) (*VerifiedArtifact, error) {
	var client http.Client
	if injected == nil {
		transport := newArtifactTransport()
		defer transport.CloseIdleConnections()
		client.Transport = transport
	} else {
		client = *injected
	}
	client.Transport = withAccess(client.Transport, config.ArtifactOrigin, config.AccessToken)
	return downloadArtifact(ctx, artifact, config.ArtifactOrigin, config.ArtifactDirectory, &client)
}

func metadataOrigin(metadataURL string) string {
	parsed, err := url.Parse(metadataURL)
	if err != nil {
		return ""
	}
	return parsed.Scheme + "://" + parsed.Host
}

// withAccessClient returns injected unchanged when no token is configured, so the
// existing default transport is used. Otherwise it returns a client whose transport
// presents the token to the artifact origin.
func withAccessClient(injected *http.Client, config PrepareConfig) *http.Client {
	if !AccessTokenPattern.MatchString(config.AccessToken) {
		return injected
	}
	var client http.Client
	if injected != nil {
		client = *injected
	} else {
		client.Transport = newArtifactTransport()
	}
	client.Transport = withAccess(client.Transport, config.ArtifactOrigin, config.AccessToken)
	return &client
}
