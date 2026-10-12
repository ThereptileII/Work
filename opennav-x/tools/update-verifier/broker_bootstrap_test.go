package verifier

import (
	"bytes"
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"errors"
	"net/http"
	"strings"
	"testing"
	"time"

	"example.com/opennav-update-verifier/releasepolicy"
	"github.com/theupdateframework/go-tuf/v2/metadata"
)

func bootstrapDigest(data []byte) string {
	sum := sha256.Sum256(data)
	return hex.EncodeToString(sum[:])
}

func bootstrapPolicyFixture(origin string) releasepolicy.BrokerBootstrapPolicy {
	artifact := func(name string) releasepolicy.Artifact {
		path := "artifacts/" + strings.Repeat("a", 40) + "/" + name
		return releasepolicy.Artifact{Path: path, URL: origin + "/" + path, SHA256: strings.Repeat("c", 64), Bytes: 1024}
	}
	return releasepolicy.BrokerBootstrapPolicy{
		Schema: 1, Product: "skager", Purpose: releasepolicy.BrokerBootstrapPurpose,
		Channel: "beta", Version: "2.0.0", Commit: strings.Repeat("a", 40),
		Architecture: "x86", ServiceName: "SkagerUpdateBroker", ServiceAccount: `NT SERVICE\SkagerUpdateBroker`,
		CodeLocation: "ProgramFilesX86/SKAGER/UpdateBroker", DataLocation: "ProgramData/SKAGER/UpdateBroker",
		TrustConfigSHA256: strings.Repeat("d", 64),
		Broker:            artifact("skager-update-broker.exe"), Provisioner: artifact("skager-update-provisioner.exe"),
		CorrespondingSource: artifact("source.zip"), Notices: artifact("notices.zip"),
	}
}

func bootstrapBytes(t *testing.T, value any) []byte {
	t.Helper()
	data, err := json.Marshal(value)
	if err != nil {
		t.Fatal(err)
	}
	return data
}

func bootstrapIdentity(p releasepolicy.BrokerBootstrapPolicy) releasepolicy.BrokerBootstrapIdentity {
	return releasepolicy.BrokerBootstrapIdentity{Purpose: p.Purpose, Channel: p.Channel,
		Version: p.Version, Commit: p.Commit, TrustConfigSHA256: p.TrustConfigSHA256}
}

// All keys and TLS certificates come from the existing ephemeral local fixture.
// This publishes real signed go-tuf metadata, not a substitute verifier.
func publishBootstrap(t *testing.T, f *fixture, version int64, payload []byte, custom []byte, targetPath string, expired bool) {
	t.Helper()
	// Retain an ordinary release in the same signed repository for regression.
	f.publish(t, version, "beta", []byte(`{"package":"signed"}`), expired)
	targets, err := metadata.Targets(time.Now().Add(time.Hour)).FromBytes(f.files["/targets.json"])
	if err != nil {
		t.Fatal(err)
	}
	targets.Signatures = nil
	target, err := metadata.TargetFile().FromBytes(targetPath, payload, "sha256")
	if err != nil {
		t.Fatal(err)
	}
	raw := json.RawMessage(custom)
	target.Custom = &raw
	targets.Signed.Targets[targetPath] = target
	sign(t, targets, f.keys["targets"])
	f.files["/targets.json"] = bytesOf(t, targets)
	f.files["/targets/"+targetPath] = bytes.Clone(payload)
}

func bootstrapFixture(t *testing.T) (*fixture, releasepolicy.BrokerBootstrapPolicy, BrokerBootstrapRequest) {
	t.Helper()
	f := newFixture(t)
	p := bootstrapPolicyFixture(f.server.URL)
	payload := bootstrapBytes(t, p)
	path, err := releasepolicy.BrokerBootstrapTargetPath(p.Channel)
	if err != nil {
		t.Fatal(err)
	}
	publishBootstrap(t, f, 1, payload, bootstrapBytes(t, bootstrapIdentity(p)), path, false)
	return f, p, BrokerBootstrapRequest{MetadataURL: f.server.URL, TrustedRoot: bytes.Clone(f.root),
		CacheDir: f.cache, Channel: p.Channel, ArtifactOrigin: f.server.URL,
		ApprovedPolicySHA256: bootstrapDigest(payload), TrustedProvisionerSHA256: p.Provisioner.SHA256,
		TrustConfigSHA256: p.TrustConfigSHA256}
}

func verifyBootstrapFixture(f *fixture, req BrokerBootstrapRequest) (releasepolicy.BrokerBootstrapPolicy, []byte, error) {
	return verifyBrokerBootstrapWithClient(req, time.Now().Add(maxOperationTime), f.server.Client())
}

func TestBrokerBootstrapVerifiedDeclarationAndReleaseIsolation(t *testing.T) {
	f, p, req := bootstrapFixture(t)
	got, data, err := verifyBootstrapFixture(f, req)
	if err != nil {
		t.Fatal(err)
	}
	want := bootstrapBytes(t, p)
	if got != p || !bytes.Equal(data, want) {
		t.Fatal("verified declaration or exact approved bytes changed")
	}
	// Returning bytes creates no artifact acquisition or execution operation.
	data[0] = 'x'
	if !bytes.Equal(f.files["/targets/bootstrap/broker-beta.json"], want) {
		t.Fatal("returned bytes alias fixture source")
	}
	if _, releaseData, err := f.verify("beta"); err != nil || string(releaseData) != `{"package":"signed"}` {
		t.Fatalf("ordinary release regression: %v", err)
	}
	ordinary := f.request("beta")
	ordinary.TargetPath = "bootstrap/broker-beta.json"
	if _, data, err := verifyWithClient(ordinary, time.Now().Add(maxOperationTime), f.server.Client()); err == nil || data != nil {
		t.Fatal("ordinary release API admitted bootstrap target")
	}
	// Even signed bootstrap custom metadata moved under an ordinary release
	// path cannot cross the original API's strict release identity boundary.
	f.publish(t, 2, "beta", want, false, func(target *metadata.TargetFiles) {
		custom := json.RawMessage(bootstrapBytes(t, bootstrapIdentity(p)))
		target.Custom = &custom
	})
	if _, data, err := f.verify("beta"); err == nil || data != nil {
		t.Fatal("ordinary release API admitted bootstrap signed custom identity")
	}
}

type bootstrapRoundTripFunc func(*http.Request) (*http.Response, error)

func (fn bootstrapRoundTripFunc) RoundTrip(req *http.Request) (*http.Response, error) { return fn(req) }

func TestBrokerBootstrapRejectsMissingInvalidPinsBeforeNetwork(t *testing.T) {
	for _, name := range []string{"policy-empty", "policy-invalid", "provisioner-empty", "provisioner-invalid", "trust-empty", "trust-invalid", "origin-empty", "origin-http", "origin-path", "origin-query", "origin-fragment", "origin-port", "root-empty", "cache-empty", "channel-invalid", "metadata-http", "metadata-query"} {
		t.Run(name, func(t *testing.T) {
			_, _, req := bootstrapFixture(t)
			switch name {
			case "policy-empty":
				req.ApprovedPolicySHA256 = ""
			case "policy-invalid":
				req.ApprovedPolicySHA256 = strings.Repeat("A", 64)
			case "provisioner-empty":
				req.TrustedProvisionerSHA256 = ""
			case "provisioner-invalid":
				req.TrustedProvisionerSHA256 = strings.Repeat("z", 64)
			case "trust-empty":
				req.TrustConfigSHA256 = ""
			case "trust-invalid":
				req.TrustConfigSHA256 = strings.Repeat("1", 63)
			case "origin-empty":
				req.ArtifactOrigin = ""
			case "origin-http":
				req.ArtifactOrigin = "http://example.invalid"
			case "origin-path":
				req.ArtifactOrigin += "/"
			case "origin-query":
				req.ArtifactOrigin += "?"
			case "origin-fragment":
				req.ArtifactOrigin += "#"
			case "origin-port":
				req.ArtifactOrigin = "https://example.invalid:65536"
			case "root-empty":
				req.TrustedRoot = nil
			case "cache-empty":
				req.CacheDir = ""
			case "channel-invalid":
				req.Channel = "production"
			case "metadata-http":
				req.MetadataURL = "http://example.invalid"
			case "metadata-query":
				req.MetadataURL += "?"
			}
			requests := 0
			client := &http.Client{Transport: bootstrapRoundTripFunc(func(*http.Request) (*http.Response, error) {
				requests++
				return nil, errors.New("network must not be reached")
			})}
			got, data, err := verifyBrokerBootstrapWithClient(req, time.Now().Add(maxOperationTime), client)
			if err == nil || got != (releasepolicy.BrokerBootstrapPolicy{}) || data != nil || requests != 0 {
				t.Fatalf("invalid request reached network or returned usable data: requests=%d, %v", requests, err)
			}
		})
	}
}

func TestBrokerBootstrapRejectsPinsAndArtifactOrigin(t *testing.T) {
	for _, name := range []string{"policy", "provisioner", "trust", "approved-origin", "broker-origin", "provisioner-origin", "source-origin", "notices-origin"} {
		t.Run(name, func(t *testing.T) {
			f, p, req := bootstrapFixture(t)
			switch name {
			case "policy":
				req.ApprovedPolicySHA256 = strings.Repeat("e", 64)
			case "provisioner":
				req.TrustedProvisionerSHA256 = strings.Repeat("e", 64)
			case "trust":
				req.TrustConfigSHA256 = strings.Repeat("e", 64)
			case "approved-origin":
				req.ArtifactOrigin = "https://other.invalid"
			default:
				artifact := map[string]*releasepolicy.Artifact{"broker-origin": &p.Broker, "provisioner-origin": &p.Provisioner, "source-origin": &p.CorrespondingSource, "notices-origin": &p.Notices}[name]
				artifact.URL = "https://unapproved.invalid/" + artifact.Path
				payload := bootstrapBytes(t, p)
				req.ApprovedPolicySHA256 = bootstrapDigest(payload)
				publishBootstrap(t, f, 1, payload, bootstrapBytes(t, bootstrapIdentity(p)), "bootstrap/broker-beta.json", false)
			}
			got, data, err := verifyBootstrapFixture(f, req)
			if err == nil || got != (releasepolicy.BrokerBootstrapPolicy{}) || data != nil {
				t.Fatalf("independent pin or origin mismatch returned usable data: %v", err)
			}
		})
	}
}

func TestBrokerBootstrapRejectsSignedIdentityMismatch(t *testing.T) {
	for _, field := range []string{"purpose", "channel", "version", "commit", "trust", "missing-custom", "ordinary-custom"} {
		t.Run(field, func(t *testing.T) {
			f, p, req := bootstrapFixture(t)
			identity := bootstrapIdentity(p)
			switch field {
			case "purpose":
				identity.Purpose = "application-update"
			case "channel":
				identity.Channel = "stable"
			case "version":
				identity.Version = "2.0.1"
			case "commit":
				identity.Commit = strings.Repeat("b", 40)
			case "trust":
				identity.TrustConfigSHA256 = strings.Repeat("e", 64)
			}
			custom := bootstrapBytes(t, identity)
			if field == "missing-custom" {
				custom = []byte(`{}`)
			}
			if field == "ordinary-custom" {
				custom = bootstrapBytes(t, Release{Channel: p.Channel, Version: p.Version, Commit: p.Commit})
			}
			publishBootstrap(t, f, 1, bootstrapBytes(t, p), custom, "bootstrap/broker-beta.json", false)
			got, data, err := verifyBootstrapFixture(f, req)
			if err == nil || got != (releasepolicy.BrokerBootstrapPolicy{}) || data != nil {
				t.Fatalf("mismatched signed custom %s returned usable data: %v", field, err)
			}
		})
	}
}

func TestBrokerBootstrapRejectsTargetSubstitutionAndTamper(t *testing.T) {
	for _, name := range []string{"ordinary-target", "other-channel-target", "payload-purpose", "payload-tamper", "payload-length", "oversize-target"} {
		t.Run(name, func(t *testing.T) {
			f, p, req := bootstrapFixture(t)
			payload := bootstrapBytes(t, p)
			switch name {
			case "ordinary-target", "other-channel-target":
				// Re-sign metadata without the selected bootstrap target.
				f.publish(t, 1, "beta", payload, false)
				if name == "other-channel-target" {
					publishBootstrap(t, f, 1, payload, bootstrapBytes(t, bootstrapIdentity(p)), "bootstrap/broker-stable.json", false)
				}
			case "payload-purpose":
				p.Purpose = "application-update"
				payload = bootstrapBytes(t, p)
				req.ApprovedPolicySHA256 = bootstrapDigest(payload)
				publishBootstrap(t, f, 1, payload, bootstrapBytes(t, bootstrapIdentity(p)), "bootstrap/broker-beta.json", false)
			case "payload-tamper":
				f.files["/targets/bootstrap/broker-beta.json"] = bytes.Replace(payload, []byte("2.0.0"), []byte("2.0.1"), 1)
			case "payload-length":
				f.files["/targets/bootstrap/broker-beta.json"] = []byte(`{}`)
			case "oversize-target":
				targets, err := metadata.Targets(time.Now().Add(time.Hour)).FromBytes(f.files["/targets.json"])
				if err != nil {
					t.Fatal(err)
				}
				targets.Signatures = nil
				targets.Signed.Targets["bootstrap/broker-beta.json"].Length = maxTargetBytes + 1
				sign(t, targets, f.keys["targets"])
				f.files["/targets.json"] = bytesOf(t, targets)
			}
			got, data, err := verifyBootstrapFixture(f, req)
			if err == nil || got != (releasepolicy.BrokerBootstrapPolicy{}) || data != nil {
				t.Fatalf("target substitution or tamper returned usable data: %v", err)
			}
		})
	}
}

func TestBrokerBootstrapRejectsSignatureExpiryAndRollback(t *testing.T) {
	for _, name := range []string{"signature", "expiry", "rollback"} {
		t.Run(name, func(t *testing.T) {
			f, p, req := bootstrapFixture(t)
			payload, custom := bootstrapBytes(t, p), bootstrapBytes(t, bootstrapIdentity(p))
			switch name {
			case "signature":
				f.files["/timestamp.json"] = bytes.Replace(f.files["/timestamp.json"], []byte(`"version":1`), []byte(`"version":9`), 1)
			case "expiry":
				publishBootstrap(t, f, 1, payload, custom, "bootstrap/broker-beta.json", true)
			case "rollback":
				publishBootstrap(t, f, 2, payload, custom, "bootstrap/broker-beta.json", false)
				if _, _, err := verifyBootstrapFixture(f, req); err != nil {
					t.Fatal(err)
				}
				publishBootstrap(t, f, 1, payload, custom, "bootstrap/broker-beta.json", false)
			}
			got, data, err := verifyBootstrapFixture(f, req)
			want := map[string]string{"signature": "not enough signatures", "expiry": "expired", "rollback": "new timestamp version 1 must be >= 2"}[name]
			if err == nil || !strings.Contains(err.Error(), want) || got != (releasepolicy.BrokerBootstrapPolicy{}) || data != nil {
				t.Fatalf("expected real TUF %s rejection, got %v", name, err)
			}
		})
	}
}

func TestBrokerBootstrapRefusesEveryMetadataAndTargetRedirect(t *testing.T) {
	for _, resource := range []string{"2.root.json", "timestamp.json", "snapshot.json", "targets.json", "targets/bootstrap/broker-beta.json"} {
		for _, destination := range []string{"same-origin", "other-origin"} {
			t.Run(resource+"/"+destination, func(t *testing.T) {
				f, _, req := bootstrapFixture(t)
				original := f.server.Config.Handler
				location := f.server.URL + "/redirected"
				if destination == "other-origin" {
					location = "https://other.invalid/redirected"
				}
				f.server.Config.Handler = http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
					if r.URL.Path == "/"+resource {
						http.Redirect(w, r, location, http.StatusFound)
						return
					}
					original.ServeHTTP(w, r)
				})
				got, data, err := verifyBootstrapFixture(f, req)
				if err == nil || !strings.Contains(err.Error(), "redirects prohibited") || got != (releasepolicy.BrokerBootstrapPolicy{}) || data != nil {
					t.Fatalf("bootstrap redirect returned usable data: %v", err)
				}
			})
		}
	}
}

func TestBrokerBootstrapIsolationPreservesOrdinarySameOriginRedirect(t *testing.T) {
	f, _, req := bootstrapFixture(t)
	f.files["/redirected-timestamp.json"] = bytes.Clone(f.files["/timestamp.json"])
	f.redirect = f.server.URL + "/redirected-timestamp.json"
	if _, data, err := f.verify("beta"); err != nil || string(data) != `{"package":"signed"}` {
		t.Fatalf("ordinary same-origin redirect compatibility changed: %v", err)
	}
	if policy, data, err := verifyBootstrapFixture(f, req); err == nil || policy != (releasepolicy.BrokerBootstrapPolicy{}) || data != nil {
		t.Fatal("bootstrap accepted an ordinary-compatible metadata redirect")
	}
}
