package verifier

import (
	"errors"
	"net/http"
	"time"

	"example.com/opennav-update-verifier/releasepolicy"
)

// VerifyRelease verifies the selected signed target, parses its bounded release
// policy and binds both identities. It does not authorize or execute an update.
// The caller must separately evaluate compatibility and installed-version state.
func VerifyRelease(req Request) (releasepolicy.Policy, error) {
	return verifyReleaseWithClient(req, time.Now().Add(maxOperationTime), nil)
}

// A transport override exists only for the package's local signed TLS fixtures.
func verifyReleaseWithClient(req Request, deadline time.Time, client *http.Client) (releasepolicy.Policy, error) {
	identity, data, err := verifyWithClient(req, deadline, client)
	if err != nil {
		return releasepolicy.Policy{}, err
	}
	policy, err := releasepolicy.Parse(data)
	if err != nil {
		return releasepolicy.Policy{}, err
	}
	if policy.Channel != identity.Channel || policy.Version != identity.Version || policy.Commit != identity.Commit {
		return releasepolicy.Policy{}, errors.New("signed target identity does not match release policy")
	}
	return policy, nil
}
