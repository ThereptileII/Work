package verifier

import (
	"encoding/json"
	"strings"
	"testing"
	"time"

	"example.com/opennav-update-verifier/releasepolicy"
	"github.com/theupdateframework/go-tuf/v2/metadata"
)

func signedPolicyFixture() releasepolicy.Policy {
	artifact := func(path string) releasepolicy.Artifact {
		return releasepolicy.Artifact{Path: path, URL: "https://updates.example/" + path,
			SHA256: strings.Repeat("c", 64), Bytes: 1024}
	}
	return releasepolicy.Policy{
		Schema: 1, Product: "skager", Channel: "beta", Version: "2.0.0", Commit: strings.Repeat("a", 40),
		ReleaseNotes: artifact("releases/notes.md"), Installer: artifact("releases/setup.exe"),
		Recovery: artifact("releases/recovery.zip"), CorrespondingSource: artifact("releases/source.zip"),
		Notices: artifact("releases/notices.zip"),
		SupportedOpenCpn: []releasepolicy.OpenCpn{{Version: "5.12.4", Arch: "x86",
			ExecutableSHA256: strings.Repeat("d", 64), UpstreamCommit: strings.Repeat("e", 40)}},
	}
}

func policyBytes(t *testing.T, p releasepolicy.Policy) []byte {
	t.Helper()
	b, err := json.Marshal(p)
	if err != nil {
		t.Fatal(err)
	}
	return b
}

func TestVerifiedReleasePolicyAndEligibility(t *testing.T) {
	f := newFixture(t)
	p := signedPolicyFixture()
	f.publish(t, 1, "beta", policyBytes(t, p), false)
	got, err := verifyReleaseWithClient(f.request("beta"), time.Now().Add(maxOperationTime), f.server.Client())
	if err != nil {
		t.Fatal(err)
	}
	if got.Commit != p.Commit || got.Version != p.Version || got.Installer.SHA256 != p.Installer.SHA256 {
		t.Fatal("verified release identity/content changed")
	}
	decision, err := releasepolicy.Evaluate(got, releasepolicy.Installed{Version: "1.9.0", Commit: strings.Repeat("f", 40)}, "beta", p.SupportedOpenCpn[0], p.SupportedOpenCpn)
	if err != nil || decision != releasepolicy.Upgrade {
		t.Fatalf("eligible signed upgrade: %s, %v", decision, err)
	}
	decision, err = releasepolicy.Evaluate(got, releasepolicy.Installed{Version: "2.1.0", Commit: strings.Repeat("f", 40)}, "beta", p.SupportedOpenCpn[0], p.SupportedOpenCpn)
	if err != nil || decision != releasepolicy.Downgrade {
		t.Fatalf("signed lower application version must remain a downgrade: %s, %v", decision, err)
	}
	decision, err = releasepolicy.Evaluate(got, releasepolicy.Installed{Version: "1.9.0", Commit: strings.Repeat("f", 40)}, "beta", p.SupportedOpenCpn[0], nil)
	if err != nil || decision != releasepolicy.Incompatible {
		t.Fatalf("signed compatibility must not expand local allowlist: %s, %v", decision, err)
	}
}

func TestVerifiedReleaseRejectsSignedIdentityMismatch(t *testing.T) {
	for _, field := range []string{"channel", "version", "commit"} {
		t.Run(field, func(t *testing.T) {
			f := newFixture(t)
			p := signedPolicyFixture()
			switch field {
			case "channel":
				p.Channel = "stable"
			case "version":
				p.Version = "2.0.1"
			case "commit":
				p.Commit = strings.Repeat("b", 40)
			}
			// These bytes are correctly signed; their body and target custom
			// identity disagree. Cryptographic verification alone cannot accept them.
			f.publish(t, 1, "beta", policyBytes(t, p), false)
			got, err := verifyReleaseWithClient(f.request("beta"), time.Now().Add(maxOperationTime), f.server.Client())
			if err == nil || !strings.Contains(err.Error(), "identity does not match") || got.Commit != "" {
				t.Fatalf("mismatched signed identity returned usable policy: %+v %v", got, err)
			}
		})
	}
}

func TestVerifiedReleaseRejectsInvalidAndTamperedPolicy(t *testing.T) {
	for _, scenario := range []string{"signed-unknown-field", "signed-duplicate-field", "unsigned-byte-change"} {
		t.Run(scenario, func(t *testing.T) {
			f := newFixture(t)
			payload := policyBytes(t, signedPolicyFixture())
			switch scenario {
			case "signed-unknown-field":
				payload = append([]byte(`{"execute":"untrusted",`), payload[1:]...)
			case "signed-duplicate-field":
				payload = append([]byte(`{"schema":1,`), payload[1:]...)
			}
			f.publish(t, 1, "beta", payload, false)
			if scenario == "unsigned-byte-change" {
				f.files["/targets/releases/beta.json"] = []byte(strings.Replace(string(payload), "setup.exe", "other.exe", 1))
			}
			got, err := verifyReleaseWithClient(f.request("beta"), time.Now().Add(maxOperationTime), f.server.Client())
			if err == nil || got.Commit != "" {
				t.Fatalf("invalid policy returned: %+v %v", got, err)
			}
		})
	}
}

func TestVerifiedReleaseRetainsLargeSemVerIdentity(t *testing.T) {
	f := newFixture(t)
	p := signedPolicyFixture()
	p.Version = strings.Repeat("9", 80) + ".0.0"
	f.publish(t, 1, "beta", policyBytes(t, p), false, func(target *metadata.TargetFiles) {
		b, err := json.Marshal(Release{Channel: p.Channel, Version: p.Version, Commit: p.Commit})
		if err != nil {
			t.Fatal(err)
		}
		custom := json.RawMessage(b)
		target.Custom = &custom
	})
	got, err := verifyReleaseWithClient(f.request("beta"), time.Now().Add(maxOperationTime), f.server.Client())
	if err != nil || got.Version != p.Version {
		t.Fatalf("bounded large version lost at signed policy boundary: %q, %v", got.Version, err)
	}
}

func TestVerifiedReleaseRejectsAmbiguousSignedCustomIdentity(t *testing.T) {
	for name, custom := range invalidSignedIdentityCases() {
		t.Run(name, func(t *testing.T) {
			f := newFixture(t)
			f.publish(t, 1, "beta", policyBytes(t, signedPolicyFixture()), false, func(target *metadata.TargetFiles) {
				raw := json.RawMessage(custom)
				target.Custom = &raw
			})
			got, err := verifyReleaseWithClient(f.request("beta"), time.Now().Add(maxOperationTime), f.server.Client())
			if err == nil || !strings.Contains(err.Error(), "signed release identity:") || got.Commit != "" || got.Installer.URL != "" {
				t.Fatalf("valid policy body masked rejected signed custom metadata: %+v %v", got, err)
			}
		})
	}
}
