package verifier

import (
	"bytes"
	"crypto/ed25519"
	"errors"
	"net/http"
	"os"
	"path/filepath"
	"strings"
	"testing"
	"time"

	"example.com/opennav-update-verifier/releasepolicy"
	"github.com/theupdateframework/go-tuf/v2/metadata"
)

// Rotate actual signed root authority, revoking the original root key. Keeping
// the other roles unchanged isolates root-floor continuity from target policy.
func rotateContinuityRoot(t *testing.T, f *fixture) []byte {
	t.Helper()
	root, err := metadata.Root().FromBytes(f.root)
	if err != nil {
		t.Fatal(err)
	}
	old := root.Signed.Roles["root"].KeyIDs[0]
	_, next, err := ed25519.GenerateKey(nil)
	if err != nil {
		t.Fatal(err)
	}
	public, err := metadata.KeyFromPublicKey(next.Public())
	if err != nil {
		t.Fatal(err)
	}
	if err = root.Signed.AddKey(public, "root"); err != nil {
		t.Fatal(err)
	}
	if err = root.Signed.RevokeKey(old, "root"); err != nil {
		t.Fatal(err)
	}
	root.Signed.Version = 2
	root.Signed.Expires = time.Now().Add(48 * time.Hour)
	root.Signatures = nil
	sign(t, root, f.keys["root"])
	sign(t, root, next)
	raw := bytesOf(t, root)
	f.files["/2.root.json"] = raw
	return raw
}

func continuityFixture(t *testing.T, entry string) (*fixture, BrokerBootstrapRequest, func(BrokerBootstrapRequest, *http.Client) error) {
	t.Helper()
	f, bootstrap, req := bootstrapFixture(t)
	if entry == "bootstrap" {
		return f, req, func(r BrokerBootstrapRequest, c *http.Client) error {
			_, _, err := verifyBrokerBootstrapWithClient(r, time.Now().Add(maxOperationTime), c)
			return err
		}
	}
	policy := releasepolicy.BrokerCodeUpgradePolicy{Schema: 1, Purpose: releasepolicy.BrokerCodeUpgradePurpose,
		PreviousBrokerSHA256: strings.Repeat("e", 64), PreviousHelperSHA256: strings.Repeat("f", 64), Bootstrap: bootstrap}
	raw := bootstrapBytes(t, policy)
	identity := bootstrapIdentity(bootstrap)
	identity.Purpose = releasepolicy.BrokerCodeUpgradePurpose
	path, _ := releasepolicy.BrokerCodeUpgradeTargetPath("beta")
	publishBootstrap(t, f, 2, raw, bootstrapBytes(t, identity), path, false)
	req.ApprovedPolicySHA256 = bootstrapDigest(raw)
	return f, req, func(r BrokerBootstrapRequest, c *http.Client) error {
		if entry == "code-upgrade" {
			_, _, err := verifyBrokerCodeUpgradeWithClient(r, time.Now().Add(maxOperationTime), c)
			return err
		}
		_, _, err := discoverBrokerCodeUpgradeWithClient(BrokerCodeUpgradeDiscoveryRequest{
			MetadataURL: r.MetadataURL, TrustedRoot: r.TrustedRoot, CacheDir: r.CacheDir, Channel: r.Channel,
			ArtifactOrigin: r.ArtifactOrigin, TrustConfigSHA256: r.TrustConfigSHA256,
		}, time.Now().Add(maxOperationTime), c)
		return err
	}
}

func requireContinuityRoot(t *testing.T, f *fixture, want []byte) {
	t.Helper()
	actual, err := os.ReadFile(filepath.Join(f.cache, "metadata", "root.json"))
	if err != nil || !bytes.Equal(actual, want) {
		t.Fatal("previously verified root authority changed", err)
	}
}

func TestTUFProtectedCacheRetainsRevokedRootKeyAcrossRepeatedPurposeChecks(t *testing.T) {
	for _, entry := range []string{"bootstrap", "code-upgrade", "discovery"} {
		t.Run(entry, func(t *testing.T) {
			f, req, verify := continuityFixture(t, entry)
			rotated := rotateContinuityRoot(t, f)
			if err := verify(req, f.server.Client()); err != nil {
				t.Fatal(err)
			}
			requireContinuityRoot(t, f, rotated)
			// Same original anchor, same durable cache, but no remote rotation
			// chain. The old root key must never become trusted again.
			delete(f.files, "/2.root.json")
			if err := verify(req, f.server.Client()); err != nil {
				t.Fatal(err)
			}
			requireContinuityRoot(t, f, rotated)
		})
	}
}

func TestTUFProtectedCacheRetainsRotationObservedDuringLaterRefreshFailure(t *testing.T) {
	f, req, verify := continuityFixture(t, "discovery")
	if err := verify(req, f.server.Client()); err != nil {
		t.Fatal(err)
	}
	rotated := rotateContinuityRoot(t, f)
	timestamp := f.files["/timestamp.json"]
	delete(f.files, "/timestamp.json")
	if err := verify(req, f.server.Client()); err == nil {
		t.Fatal("missing timestamp accepted")
	}
	requireContinuityRoot(t, f, rotated)
	delete(f.files, "/2.root.json")
	f.files["/timestamp.json"] = timestamp
	if err := verify(req, f.server.Client()); err != nil {
		t.Fatal(err)
	}
	requireContinuityRoot(t, f, rotated)
}

func TestTUFProtectedCacheUsesValidCurrentRootAfterOriginalAnchorExpires(t *testing.T) {
	f, req, verify := continuityFixture(t, "discovery")
	original, err := metadata.Root().FromBytes(f.root)
	if err != nil {
		t.Fatal(err)
	}
	original.Signed.Expires = time.Now().Add(-time.Hour)
	original.Signatures = nil
	sign(t, original, f.keys["root"])
	f.root = bytesOf(t, original)
	req.TrustedRoot = f.root
	rotated := rotateContinuityRoot(t, f)
	// An expired intermediate may rotate forward; final root expiry still
	// belongs to live go-tuf verification and is never bypassed here.
	if err = verify(req, f.server.Client()); err != nil {
		t.Fatal(err)
	}
	delete(f.files, "/2.root.json")
	if err = verify(req, f.server.Client()); err != nil {
		t.Fatal("expired original anchor replaced current retained root", err)
	}
	requireContinuityRoot(t, f, rotated)
	// Advance only the existing per-call test clock, preserving all production
	// signatures and expiry predicates. Current-root expiry must still refuse.
	request := Request{MetadataURL: req.MetadataURL, TrustedRoot: req.TrustedRoot, CacheDir: req.CacheDir,
		Channel: req.Channel, TargetPath: "bootstrap/broker-code-upgrade-beta.json"}
	if _, err = verifyTargetWithClientAt(request, time.Now().Add(maxOperationTime), f.server.Client(), true,
		func([]byte) error { return nil }, time.Now().Add(72*time.Hour)); err == nil {
		t.Fatal("expired current retained root accepted")
	}
	requireContinuityRoot(t, f, rotated)
}

func TestTUFProtectedCacheDamageRefusesBeforeNetworkWithoutReset(t *testing.T) {
	for _, fault := range []string{"missing-root", "empty-root", "truncated-root", "oversized-root", "invalid-signature", "foreign-same-version", "below-anchor", "missing-metadata", "root-directory", "empty-existing-cache"} {
		t.Run(fault, func(t *testing.T) {
			f, req, verify := continuityFixture(t, "discovery")
			if err := verify(req, f.server.Client()); err != nil {
				t.Fatal(err)
			}
			rootPath := filepath.Join(f.cache, "metadata", "root.json")
			write := func(raw []byte) {
				t.Helper()
				if err := os.WriteFile(rootPath, raw, 0600); err != nil {
					t.Fatal(err)
				}
			}
			switch fault {
			case "missing-root":
				if err := os.Remove(rootPath); err != nil {
					t.Fatal(err)
				}
			case "empty-root":
				write(nil)
			case "truncated-root":
				write(f.root[:len(f.root)-1])
			case "oversized-root":
				write(bytes.Repeat([]byte(" "), maxTrustedRootBytes+1))
			case "invalid-signature":
				root, err := metadata.Root().FromBytes(f.root)
				if err != nil {
					t.Fatal(err)
				}
				root.Signatures = nil
				write(bytesOf(t, root))
			case "foreign-same-version":
				write(newFixture(t).root)
			case "below-anchor":
				req.TrustedRoot = rotateContinuityRoot(t, f)
			case "missing-metadata":
				if err := os.RemoveAll(filepath.Join(f.cache, "metadata")); err != nil {
					t.Fatal(err)
				}
			case "root-directory":
				if err := os.Remove(rootPath); err != nil {
					t.Fatal(err)
				}
				if err := os.Mkdir(rootPath, 0700); err != nil {
					t.Fatal(err)
				}
			case "empty-existing-cache":
				entries, err := os.ReadDir(f.cache)
				if err != nil {
					t.Fatal(err)
				}
				for _, entry := range entries {
					if err = os.RemoveAll(filepath.Join(f.cache, entry.Name())); err != nil {
						t.Fatal(err)
					}
				}
			}
			before, readErr := os.ReadFile(rootPath)
			calls := 0
			client := &http.Client{Transport: bootstrapRoundTripFunc(func(*http.Request) (*http.Response, error) {
				calls++
				return nil, errors.New("network must not be reached for damaged trust")
			})}
			if err := verify(req, client); err == nil || calls != 0 {
				t.Fatal("damaged cache reached network or reset trust", err, calls)
			}
			after, afterErr := os.ReadFile(rootPath)
			if (readErr == nil) != (afterErr == nil) || !bytes.Equal(before, after) {
				t.Fatal("refusal rewrote or recreated retained root")
			}
		})
	}
}
