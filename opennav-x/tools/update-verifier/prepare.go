package verifier

import (
	"context"
	"encoding/hex"
	"encoding/json"
	"errors"
	"net/http"
	"strings"

	"example.com/opennav-update-verifier/releasepolicy"
)

// PrepareConfig comes from protected installer-provisioned configuration and
// installed release records, never a UI message or server response. Config and
// its slices must not be changed concurrently with a check or preparation.
// Channel changes require separately provisioned trust and state.
type PrepareConfig struct {
	Trust             TrustConfig
	StateDirectory    string
	ArtifactOrigin    string
	ArtifactDirectory string
	Installed         releasepolicy.Installed
	OpenCpnAllowlist  []releasepolicy.OpenCpn
}

// ConsentIdentity binds a user's selected offer to the entire parsed policy.
// PolicySHA256 is SHA-256 of encoding/json.Marshal of releasepolicy.Policy,
// not a digest of the original JSON serialization. Whitespace/key order have no
// meaning; every parsed field, including artifact identity, remains bound.
// This value identifies consent; it does not establish that a user consented.
// The installed launcher must independently obtain explicit user confirmation.
type ConsentIdentity struct {
	Version      string
	Commit       string
	PolicySHA256 string
}

// UpdateCandidate is authenticated check output suitable for presenting an
// offer. Its fields are informational, not authority to execute or install.
// PrepareUpdate refreshes and reevaluates policy before accepting its Consent.
type UpdateCandidate struct {
	Policy   releasepolicy.Policy
	Decision releasepolicy.Decision
	Consent  ConsentIdentity
}

// PreparedUpdate owns a verified installer handle. Call Close on every path.
// Keep the handle alive through any separately authorized launch. This API
// performs no execution, Authenticode check, installation, recovery or restart.
// Those independent gates and native Windows custody qualification remain the
// launcher's responsibility. Artifact.Path() alone never authorizes execution.
type PreparedUpdate struct {
	Policy   releasepolicy.Policy
	Consent  ConsentIdentity
	Artifact *VerifiedArtifact
}

func (p *PreparedUpdate) Close() error {
	if p == nil || p.Artifact == nil {
		return nil
	}
	return p.Artifact.Close()
}

// CheckUpdate authenticates policy using protected retained state and evaluates
// eligibility against the installed release and independent local allowlist.
// observed must be freshly measured by the trusted installed launcher from the
// actual installed OpenCPN executable and its supported upstream identity. Never
// accept observed from a UI, release server, or executable version string alone.
// No artifact is downloaded by this operation, including for eligible offers.
func CheckUpdate(ctx context.Context, config PrepareConfig, observed releasepolicy.OpenCpn) (UpdateCandidate, error) {
	return checkUpdate(ctx, config, observed, nil)
}

// PrepareUpdate repeats the authenticated check and allows only an Upgrade whose
// complete policy matches the exact explicitly confirmed offer. Current,
// downgrade, incompatible, conflict, invalid or unconfigured results cannot
// download an installer. Directory protection and observed identity requirements
// are the same as CheckUpdate and DownloadArtifact.
func PrepareUpdate(ctx context.Context, config PrepareConfig, observed releasepolicy.OpenCpn, consent ConsentIdentity) (*PreparedUpdate, error) {
	return prepareUpdate(ctx, config, observed, consent, nil)
}

// Client injection is private and exists only for signed local TLS fixtures.
func checkUpdate(ctx context.Context, config PrepareConfig, observed releasepolicy.OpenCpn, injected *http.Client) (UpdateCandidate, error) {
	var empty UpdateCandidate
	if ctx == nil {
		return empty, errors.New("update context required")
	}
	if err := ctx.Err(); err != nil {
		return empty, err
	}
	if err := validatePrepareConfig(config); err != nil {
		return empty, err
	}
	ctx, cancel := context.WithTimeout(ctx, maxOperationTime)
	defer cancel()
	client := &http.Client{}
	if injected != nil {
		*client = *injected
	}
	base := client.Transport
	if base == nil {
		base = http.DefaultTransport
	}
	client.Transport = prepareContextTransport{parent: ctx, base: base}
	policy, err := verifyReleaseInState(config.StateDirectory, config.Trust, client)
	if err != nil {
		return empty, downloadError(ctx, "protected signed update check rejected")
	}
	if err := ctx.Err(); err != nil {
		return empty, err
	}
	if err := validateDownload(policy.Installer, config.ArtifactOrigin); err != nil {
		return empty, errors.New("signed installer does not match installed acquisition policy")
	}
	decision, err := releasepolicy.Evaluate(policy, config.Installed, config.Trust.Channel, observed, config.OpenCpnAllowlist)
	if err != nil {
		return empty, errors.New("update eligibility rejected")
	}
	consent, err := policyConsent(policy)
	if err != nil {
		return empty, err
	}
	return UpdateCandidate{Policy: policy, Decision: decision, Consent: consent}, nil
}

func prepareUpdate(ctx context.Context, config PrepareConfig, observed releasepolicy.OpenCpn, consent ConsentIdentity, injected *http.Client) (*PreparedUpdate, error) {
	if !validConsentIdentity(consent) {
		return nil, errors.New("exact confirmed update identity required")
	}
	candidate, err := checkUpdate(ctx, config, observed, injected)
	if err != nil {
		return nil, err
	}
	if candidate.Decision != releasepolicy.Upgrade {
		return nil, errors.New("update is not an eligible upgrade")
	}
	if candidate.Consent != consent {
		return nil, errors.New("confirmed update policy changed; new confirmation required")
	}
	var artifact *VerifiedArtifact
	if injected == nil {
		artifact, err = DownloadArtifact(ctx, candidate.Policy.Installer, config.ArtifactOrigin, config.ArtifactDirectory)
	} else {
		artifact, err = downloadArtifact(ctx, candidate.Policy.Installer, config.ArtifactOrigin, config.ArtifactDirectory, injected)
	}
	if err != nil {
		return nil, err
	}
	return &PreparedUpdate{Policy: candidate.Policy, Consent: candidate.Consent, Artifact: artifact}, nil
}

func validatePrepareConfig(config PrepareConfig) error {
	if config.StateDirectory == "" || config.ArtifactDirectory == "" || len(config.OpenCpnAllowlist) == 0 || len(config.OpenCpnAllowlist) > 16 {
		return errors.New("installed update configuration incomplete")
	}
	if _, err := releasepolicy.ParseSemVer(config.Installed.Version); err != nil || !commitPattern.MatchString(config.Installed.Commit) {
		return errors.New("installed release identity unavailable")
	}
	// Reuse acquisition's strict origin parser before any metadata networking.
	probe := releasepolicy.Artifact{Path: "origin-validation", URL: config.ArtifactOrigin + "/origin-validation", SHA256: strings.Repeat("0", 64), Bytes: 1}
	if err := validateDownload(probe, config.ArtifactOrigin); err != nil {
		return errors.New("installed artifact origin unavailable")
	}
	if err := plainStatePath(config.ArtifactDirectory); err != nil {
		return errors.New("protected artifact directory unavailable")
	}
	if err := assertPrivateStateDirectory(config.ArtifactDirectory); err != nil {
		return errors.New("protected artifact directory rejected")
	}
	return nil
}

func policyConsent(policy releasepolicy.Policy) (ConsentIdentity, error) {
	data, err := json.Marshal(policy)
	if err != nil {
		return ConsentIdentity{}, errors.New("update policy identity unavailable")
	}
	return ConsentIdentity{Version: policy.Version, Commit: policy.Commit, PolicySHA256: stateDigest(data)}, nil
}

func validConsentIdentity(consent ConsentIdentity) bool {
	if _, err := releasepolicy.ParseSemVer(consent.Version); err != nil || !commitPattern.MatchString(consent.Commit) || len(consent.PolicySHA256) != 64 || strings.ToLower(consent.PolicySHA256) != consent.PolicySHA256 {
		return false
	}
	_, err := hex.DecodeString(consent.PolicySHA256)
	return err == nil
}

// Preserve the verifier's per-request deadline while also honoring coordinator
// cancellation. The callback lives through response-body consumption.
type prepareContextTransport struct {
	parent context.Context
	base   http.RoundTripper
}

func (t prepareContextTransport) RoundTrip(request *http.Request) (*http.Response, error) {
	ctx, cancel := context.WithCancel(request.Context())
	stop := context.AfterFunc(t.parent, cancel)
	finish := func() { stop(); cancel() }
	if err := t.parent.Err(); err != nil {
		finish()
		return nil, err
	}
	response, err := t.base.RoundTrip(request.WithContext(ctx))
	if err != nil {
		finish()
		return nil, err
	}
	response.Body = &cancelBody{ioReadCloser: response.Body, cancel: finish}
	return response, nil
}
