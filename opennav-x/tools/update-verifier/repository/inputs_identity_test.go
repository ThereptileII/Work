//go:build linux

package repository

import (
	"bytes"
	"encoding/json"
	"testing"
	"time"

	"github.com/theupdateframework/go-tuf/v2/metadata"
)

func TestOriginalTrustIdentityOutlivesAnchorExpiry(t *testing.T) {
	f := newFixture(t)
	raw, err := Export(f.dir, "config", f.now)
	if err != nil {
		t.Fatal(err)
	}
	root, err := metadata.Root().FromBytes(f.root)
	if err != nil {
		t.Fatal(err)
	}
	if _, err = ValidateTrust(raw, root.Signed.Expires.Add(time.Second)); err == nil {
		t.Fatal("expired anchor authorized fresh operator enrollment")
	}
	identity, err := ValidateTrustIdentity(raw)
	if err != nil {
		t.Fatal(err)
	}
	if !bytes.Equal(identity.BootstrapRoot, f.root) || identity.MetadataURL != f.trust.MetadataURL || identity.ArtifactOrigin != f.trust.ArtifactOrigin {
		t.Fatal("retained public trust identity changed")
	}
}

func TestRetainedTrustIdentityKeepsSchemaSignatureAndKeyChecks(t *testing.T) {
	f := newFixture(t)
	raw, err := Export(f.dir, "config", f.now)
	if err != nil {
		t.Fatal(err)
	}
	for _, scenario := range []string{"unknown", "duplicate", "case alias", "origin", "stock", "signature", "version", "missing expiry", "role keys"} {
		t.Run(scenario, func(t *testing.T) {
			var trust Trust
			if err := json.Unmarshal(raw, &trust); err != nil {
				t.Fatal(err)
			}
			root, err := metadata.Root().FromBytes(trust.BootstrapRoot)
			if err != nil {
				t.Fatal(err)
			}
			switch scenario {
			case "origin":
				trust.ArtifactOrigin = "https://updates.invalid/path"
			case "stock":
				trust.LocalCompatibilityAllowlist[0].Arch = "x64"
			case "signature":
				root.Signatures[0].Signature = []byte{0}
			case "version":
				root.Signed.Version = 2
			case "missing expiry":
				root.Signed.Expires = time.Time{}
			case "role keys":
				root.Signed.Roles["timestamp"].KeyIDs = root.Signed.Roles["snapshot"].KeyIDs
			}
			if scenario == "version" || scenario == "missing expiry" || scenario == "role keys" {
				root.Signatures = nil
				trust.BootstrapRoot, err = sign(root, f.keys["root"])
			} else {
				trust.BootstrapRoot, err = root.ToBytes(false)
			}
			if err != nil {
				t.Fatal(err)
			}
			changed := jsonBytes(trust)
			switch scenario {
			case "unknown":
				changed = bytes.Replace(changed, []byte(`"schema": 1`), []byte(`"schema": 1,"extra":true`), 1)
			case "duplicate":
				changed = bytes.Replace(changed, []byte(`"schema": 1`), []byte(`"schema": 1,"schema":1`), 1)
			case "case alias":
				changed = bytes.Replace(changed, []byte(`"channel"`), []byte(`"Channel"`), 1)
			}
			if _, err := ValidateTrustIdentity(changed); err == nil {
				t.Fatal("retained identity bypassed strict public trust contract")
			}
			if _, err := ValidateTrust(changed, f.now); err == nil {
				t.Fatal("fresh trust contract weakened")
			}
		})
	}
}
