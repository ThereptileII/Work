package brokerhost

import (
	"bytes"
	"encoding/json"
	"errors"
	"unicode/utf8"

	verifier "example.com/opennav-update-verifier"
	"example.com/opennav-update-verifier/brokerstore"
	"example.com/opennav-update-verifier/releasepolicy"
)

var errExecutionAuthorization = errors.New("original staged update and current signed policy differ")

// This is a consistency check, not a receipt import or launch capability. The
// native caller must obtain raw from its same-Store retained original stage-seal
// owner, completion from its live generation and registration from the protected
// approved request. Current permissions or a caller-supplied JSON cannot do so.
func parseExecutionStagingReceipt(raw []byte, r registration, c brokerstore.GenerationCompletion) (stagingReceipt, error) {
	var receipt stagingReceipt
	if len(raw) == 0 || len(raw) > 1<<20 || !utf8.Valid(raw) {
		return receipt, errExecutionAuthorization
	}
	decoder := json.NewDecoder(bytes.NewReader(raw))
	decoder.DisallowUnknownFields()
	if err := decoder.Decode(&receipt); err != nil {
		return stagingReceipt{}, errExecutionAuthorization
	}
	canonical, err := json.Marshal(receipt)
	// The producer writes exactly this serialization. Exact bytes also reject
	// duplicate/case-aliased keys, omitted fields, trailing data and JSON nulls.
	if err != nil || !bytes.Equal(raw, canonical) || receipt.Schema != 3 || receipt.State != stagedState ||
		receipt.ClientSID != r.ClientSID || receipt.RequestID != c.Generation ||
		receipt.Installed != r.Installed || receipt.ApprovedRequestSHA256 != r.ApprovedRequestSHA256 ||
		!hashPattern.MatchString(r.ApprovedRequestSHA256) || receipt.Materialized != c ||
		c.Schema != 1 || c.ClientSID != r.ClientSID || !requestPattern.MatchString(c.Generation) ||
		c.Generation == r.Installed.Generation || c.PreviousGeneration != r.Installed.Generation ||
		!hashPattern.MatchString(c.CompletionSHA256) || !hashPattern.MatchString(c.PreviousCompletionSHA256) ||
		!hashPattern.MatchString(c.PolicySHA256) || !hashPattern.MatchString(c.RecoverySHA256) ||
		!hashPattern.MatchString(c.InstallerSHA256) || !validConsent(receipt.Consent) ||
		receipt.Consent.PolicySHA256 != c.PolicySHA256 || !validRecovery(receipt.Recovery, receipt.Consent) ||
		receipt.Recovery != c.Identity {
		return stagingReceipt{}, errExecutionAuthorization
	}
	return receipt, nil
}

// A native execution rechecks the real protected verifier; these copied values
// cannot authenticate a policy. This comparison binds ALL policy fields through
// the original consent digest, both outer artifacts, and the actual selected
// predecessor (which need not be the registered initial baseline).
func matchExecutionSignedOffer(receipt stagingReceipt, current verifier.UpdateCandidate, installed releasepolicy.Installed, channel string) error {
	raw, err := json.Marshal(current.Policy)
	if err != nil || current.Decision != releasepolicy.Upgrade || current.Consent != receipt.Consent ||
		current.Policy.Schema != 1 || current.Policy.Product != "skager" || current.Policy.Channel != channel ||
		current.Policy.Version != receipt.Consent.Version || current.Policy.Commit != receipt.Consent.Commit ||
		executionOriginalDigest(raw) != receipt.Consent.PolicySHA256 ||
		current.Policy.Recovery.SHA256 != receipt.Materialized.RecoverySHA256 ||
		current.Policy.Installer.SHA256 != receipt.Materialized.InstallerSHA256 {
		return errExecutionAuthorization
	}
	next, nextErr := releasepolicy.ParseSemVer(current.Policy.Version)
	previous, previousErr := releasepolicy.ParseSemVer(installed.Version)
	if nextErr != nil || previousErr != nil || next.Compare(previous) <= 0 ||
		!commitPattern.MatchString(installed.Commit) {
		return errExecutionAuthorization
	}
	return nil
}
