package brokerhost

import (
	"encoding/json"
	"strings"
	"testing"

	verifier "example.com/opennav-update-verifier"
	"example.com/opennav-update-verifier/releasepolicy"
)

func executionAuthorizationFixture(t *testing.T) (registration, stagingReceipt, []byte, verifier.UpdateCandidate) {
	t.Helper()
	r, err := parseRegistration(testRegistration(), testSID)
	if err != nil {
		t.Fatal(err)
	}
	c := testGenerationCompletion()
	p := releasepolicy.Policy{Schema: 1, Product: "skager", Channel: "beta", Version: c.Identity.Version, Commit: c.Identity.Commit,
		Recovery: releasepolicy.Artifact{SHA256: c.RecoverySHA256}, Installer: releasepolicy.Artifact{SHA256: c.InstallerSHA256}}
	policyRaw, err := json.Marshal(p)
	if err != nil {
		t.Fatal(err)
	}
	c.PolicySHA256 = executionOriginalDigest(policyRaw)
	consent := verifier.ConsentIdentity{Version: c.Identity.Version, Commit: c.Identity.Commit, PolicySHA256: c.PolicySHA256}
	receipt := stagingReceipt{3, stagedState, c.Generation, testSID, consent, c.Identity, r.Installed, r.ApprovedRequestSHA256, c}
	raw, err := json.Marshal(receipt)
	if err != nil {
		t.Fatal(err)
	}
	return r, receipt, raw, verifier.UpdateCandidate{Policy: p, Consent: consent, Decision: releasepolicy.Upgrade}
}

func TestExecutionAuthorizationOriginalReceiptMatchesExactProtectedGeneration(t *testing.T) {
	r, expected, raw, _ := executionAuthorizationFixture(t)
	got, err := parseExecutionStagingReceipt(raw, r, expected.Materialized)
	if err != nil || got != expected {
		t.Fatal("original receipt differs", got, err)
	}
	// These are copied consistency values only; native origin custody is a
	// separate prerequisite tested by the protected generation/stage owners.
}

func TestExecutionAuthorizationRejectsAmbiguousOrSubstitutedReceipt(t *testing.T) {
	for _, fault := range []string{"duplicate", "case-alias", "unknown", "trailing", "invalid-utf8", "oversized", "wrong-approval", "wrong-registration", "wrong-client", "wrong-stage", "wrong-parent", "wrong-recovery", "wrong-policy", "missing-lifecycle", "wrong-state", "wrong-completion"} {
		t.Run(fault, func(t *testing.T) {
			r, receipt, raw, _ := executionAuthorizationFixture(t)
			c := receipt.Materialized
			switch fault {
			case "duplicate":
				raw = append([]byte(`{"schema":3,`), raw[1:]...)
			case "case-alias":
				raw = []byte(strings.Replace(string(raw), `"schema"`, `"Schema"`, 1))
			case "unknown":
				raw = append([]byte(`{"extra":true,`), raw[1:]...)
			case "trailing":
				raw = append(raw, []byte("{}")...)
			case "invalid-utf8":
				raw = append(raw, 0xff)
			case "oversized":
				raw = make([]byte, 1<<20+1)
			default:
				switch fault {
				case "wrong-approval":
					receipt.ApprovedRequestSHA256 = strings.Repeat("0", 64)
				case "wrong-registration":
					receipt.Installed.Commit = strings.Repeat("0", 40)
				case "wrong-client":
					receipt.ClientSID = "S-1-5-21-100-200-300-1002"
				case "wrong-stage":
					receipt.RequestID = r.Installed.Generation
				case "wrong-parent":
					c.PreviousGeneration = strings.Repeat("0", 32)
					receipt.Materialized = c
				case "wrong-recovery":
					receipt.Recovery.ExecutableSHA256 = strings.Repeat("0", 64)
				case "wrong-policy":
					receipt.Consent.PolicySHA256 = strings.Repeat("0", 64)
				case "missing-lifecycle":
					receipt.Recovery.LifecycleManifestAvailable = false
				case "wrong-state":
					receipt.State = "selected"
				case "wrong-completion":
					receipt.Materialized.CompletionSHA256 = strings.Repeat("0", 64)
				}
				raw, _ = json.Marshal(receipt)
			}
			got, err := parseExecutionStagingReceipt(raw, r, c)
			if err == nil || got != (stagingReceipt{}) {
				t.Fatal("ambiguous/substituted original accepted", got, err)
			}
		})
	}
}

func TestExecutionAuthorizationBindsFreshPolicyAndActualSelectedPredecessor(t *testing.T) {
	r, receipt, _, current := executionAuthorizationFixture(t)
	installed := releasepolicy.Installed{Version: r.Installed.Version, Commit: r.Installed.Commit}
	if err := matchExecutionSignedOffer(receipt, current, installed, "beta"); err != nil {
		t.Fatal(err)
	}
	// Original registration remains the materialization parent. A later selected
	// release must separately prevent executing a stale but valid staged offer.
	installed.Version = current.Policy.Version
	if matchExecutionSignedOffer(receipt, current, installed, "beta") == nil {
		t.Fatal("original baseline was substituted for actual selected release")
	}
	installed.Version = "9.0.0"
	if matchExecutionSignedOffer(receipt, current, installed, "beta") == nil {
		t.Fatal("stale signed offer executed as downgrade")
	}
}

func TestExecutionAuthorizationRejectsSupersededSignedFieldsAndArtifacts(t *testing.T) {
	for _, fault := range []string{"no-update", "different-channel", "changed-consent", "changed-static-field", "changed-recovery", "changed-installer", "changed-commit", "invalid-selected"} {
		t.Run(fault, func(t *testing.T) {
			r, receipt, _, current := executionAuthorizationFixture(t)
			installed := releasepolicy.Installed{Version: r.Installed.Version, Commit: r.Installed.Commit}
			switch fault {
			case "no-update":
				current.Decision = releasepolicy.Current
			case "different-channel":
				current.Policy.Channel = "stable"
			case "changed-consent":
				current.Consent.PolicySHA256 = strings.Repeat("0", 64)
			case "changed-static-field":
				current.Policy.Notices.URL = "https://changed.invalid/notices"
			case "changed-recovery":
				current.Policy.Recovery.SHA256 = strings.Repeat("0", 64)
			case "changed-installer":
				current.Policy.Installer.SHA256 = strings.Repeat("0", 64)
			case "changed-commit":
				current.Policy.Commit = strings.Repeat("0", 40)
			case "invalid-selected":
				installed = releasepolicy.Installed{}
			}
			if matchExecutionSignedOffer(receipt, current, installed, "beta") == nil {
				t.Fatal("superseded signed offer accepted")
			}
		})
	}
}
