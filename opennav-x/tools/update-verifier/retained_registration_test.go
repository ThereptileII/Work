package verifier

import (
	"bytes"
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"path/filepath"
	"strconv"
	"strings"
	"testing"
	"time"

	"example.com/opennav-update-verifier/brokerregistration"
	"example.com/opennav-update-verifier/releasepolicy"
	"example.com/opennav-update-verifier/repository"
	"github.com/theupdateframework/go-tuf/v2/metadata"
)

// Reuse the actual verifier's local TLS/signature fixture and durable state.
// Root expiry uses the real clock throughout: no validator receives a backdated
// clock, and no production expiry, rollback or network verification is bypassed.
func TestRetainedRegistrationUsesRotatedRootAfterOriginalAnchorExpires(t *testing.T) {
	f := newFixture(t)
	original, err := metadata.Root().FromBytes(f.root)
	if err != nil {
		t.Fatal(err)
	}
	original.Signed.ConsistentSnapshot = true
	original.Signed.Expires = time.Now().UTC().Add(3 * time.Second)
	original.Signatures = nil
	sign(t, original, f.keys["root"])
	f.root = bytesOf(t, original)
	f.files["/1.root.json"] = f.root
	rotated, err := metadata.Root().FromBytes(f.root)
	if err != nil {
		t.Fatal(err)
	}
	rotated.Signed.Version = 2
	rotated.Signed.Expires = original.Signed.Expires.Add(3 * time.Second)
	rotated.Signatures = nil
	sign(t, rotated, f.keys["root"])
	rotatedBytes := bytesOf(t, rotated)
	f.files["/2.root.json"] = rotatedBytes
	policyRaw := policyBytes(t, signedPolicyFixture())
	digest := func(raw []byte) string { sum := sha256.Sum256(raw); return hex.EncodeToString(sum[:]) }
	// Match the existing fixture's target/role bytes to consistent-snapshot URLs.
	// Their actual signatures, target hashes and all verifier checks remain active.
	publish := func(version int64) {
		f.publish(t, version, "beta", policyRaw, false)
		snapshot, err := metadata.Snapshot().FromBytes(f.files["/snapshot.json"])
		if err != nil {
			t.Fatal(err)
		}
		targets, err := metadata.Targets().FromBytes(f.files["/targets.json"])
		if err != nil {
			t.Fatal(err)
		}
		f.files["/"+strconv.FormatInt(snapshot.Signed.Version, 10)+".snapshot.json"] = f.files["/snapshot.json"]
		f.files["/"+strconv.FormatInt(targets.Signed.Version, 10)+".targets.json"] = f.files["/targets.json"]
		f.files["/targets/releases/"+digest(policyRaw)+".beta.json"] = policyRaw
	}
	publish(1)
	trustRaw, err := json.Marshal(repository.Trust{Schema: 1, BootstrapRoot: f.root, MetadataURL: f.server.URL, ArtifactOrigin: f.server.URL, Channel: "beta", LocalCompatibilityAllowlist: []releasepolicy.OpenCpn{brokerregistration.StockBaseline()}})
	if err != nil {
		t.Fatal(err)
	}
	baseline := brokerregistration.StockBaseline()
	requestRaw, err := json.Marshal(brokerregistration.Request{Schema: 1, Purpose: brokerregistration.Purpose, ClientSID: "S-1-5-21-100-200-300-1001", Installed: brokerregistration.Installed{Version: "0.4.1-beta.1", Commit: strings.Repeat("a", 40), Generation: strings.Repeat("b", 32), PackageSHA256: strings.Repeat("c", 64), ProductBuildSHA256: strings.Repeat("d", 64), ExecutableSHA256: strings.Repeat("e", 64), InventorySHA256: strings.Repeat("f", 64)}, Stock: brokerregistration.Stock{Path: `C:\OpenCPN\opencpn.exe`, Version: baseline.Version, Arch: baseline.Arch, ExecutableSHA256: baseline.ExecutableSHA256, UpstreamCommit: baseline.UpstreamCommit}, TrustConfiguration: trustRaw, TrustConfigSHA256: digest(trustRaw)})
	if err != nil {
		t.Fatal(err)
	}
	approved, err := brokerregistration.ParseApprovedRequest(requestRaw, digest(requestRaw), time.Now().UTC())
	if err != nil {
		t.Fatal(err)
	}
	trust := approved.Trust()
	config := TrustConfig{BootstrapRoot: trust.BootstrapRoot, MetadataURL: trust.MetadataURL, Channel: trust.Channel}
	state := filepath.Join(t.TempDir(), "protected")
	if err = InitializeState(state, config); err != nil {
		t.Fatal(err)
	}
	refresh := func() error { _, err := verifyReleaseInState(state, config, f.server.Client()); return err }
	if err = refresh(); err != nil {
		t.Fatal(err)
	}
	// The original approved anchor expires while the already-verified current
	// root remains valid. Removing the remote chain must not reset retained trust.
	time.Sleep(time.Until(original.Signed.Expires) + 20*time.Millisecond)
	if _, err = brokerregistration.ParseApprovedRequest(requestRaw, digest(requestRaw), time.Now().UTC()); err == nil {
		t.Fatal("expired original anchor authorized fresh enrollment")
	}
	retained, err := brokerregistration.ParseRetainedApprovedRequest(requestRaw, digest(requestRaw))
	if err != nil {
		t.Fatal(err)
	}
	if !bytes.Equal(retained.OriginalBytes(), requestRaw) {
		t.Fatal("retained approval identity changed")
	}
	delete(f.files, "/2.root.json")
	if err = refresh(); err != nil {
		t.Fatalf("valid persisted rotated root was blocked by expired original anchor: %v", err)
	}
	publish(2)
	if err = refresh(); err != nil {
		t.Fatal(err)
	}
	publish(1)
	if err = refresh(); err == nil {
		t.Fatal("retained identity bypassed durable timestamp rollback floor")
	}
	publish(2)
	// Current retained root expiry still fails under the actual verifier clock.
	time.Sleep(time.Until(rotated.Signed.Expires) + 20*time.Millisecond)
	if err = refresh(); err == nil {
		t.Fatal("expired CURRENT retained root accepted")
	}
	want, err := trustIdentity(config)
	if err != nil {
		t.Fatal(err)
	}
	current, err := readState(state, want)
	if err != nil {
		t.Fatal(err)
	}
	files, err := readStateMetadata(filepath.Join(state, current.Generation))
	if err != nil {
		t.Fatal(err)
	}
	if !bytes.Equal(files["root.json"], rotatedBytes) {
		t.Fatal("failure reset retained root to original anchor")
	}
	timestamp, err := metadata.Timestamp().FromBytes(files["timestamp.json"])
	if err != nil || timestamp.Signed.Version != 2 {
		t.Fatal("failure reset durable metadata floor")
	}
	if err = InitializeState(state, config); err == nil {
		t.Fatal("expired state allowed trust reinitialization")
	}
}
