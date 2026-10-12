package brokerhost

import (
	"context"
	"encoding/json"
	"strings"
	"testing"

	"example.com/opennav-update-verifier/brokerstore"
)

// Only copied consistency is tested here. Actual native provider requires sealed
// image, retained full archive and fixed-profile owners; these values cannot mint
// any of those capabilities.
func executionCommissionFixture(t *testing.T) (brokerstore.GenerationApplicationIdentity, executionProfileOriginal, ExecutionProfileCaptureSnapshot) {
	t.Helper()
	p, source, _ := profileCaptureFixture(t)
	t.Cleanup(func() { _ = p.Close() })
	if err := p.ReleaseForQualification(context.Background()); err != nil {
		t.Fatal(err)
	}
	snapshot, err := p.Snapshot(context.Background())
	if err != nil {
		t.Fatal(err)
	}
	manifest := p.state.manifest
	snapshot.Archive.ClientSID, snapshot.Archive.ExecutionID = manifest.ClientSID, manifest.ExecutionID
	snapshot.Archive.Generation, snapshot.Archive.CompletionSHA256 = manifest.Generation, manifest.CompletionSHA256
	snapshot.Archive.JournalFenceSHA256 = strings.Repeat("f", 64)
	id := executionProfileID{volume: 1, file: [16]byte{1}, created: 1}
	cold := executionProfileOriginal{directory: id, file: id, raw: append([]byte(nil), source.files["opencpn.ini"]...)}
	cold.digest = captureDigest(cold.raw)
	image := brokerstore.GenerationApplicationIdentity{ClientSID: manifest.ClientSID, Generation: manifest.Generation, Commit: strings.Repeat("c", 40), ExecutableSHA256: strings.Repeat("d", 64)}
	return image, cold, snapshot
}

func TestExecutionCommissionBindsOriginalImageINIAndWholeProfile(t *testing.T) {
	image, cold, snapshot := executionCommissionFixture(t)
	digest, err := executionCommissionEvidence(image, cold, snapshot)
	if err != nil || !hashPattern.MatchString(digest) {
		t.Fatal("coherent original qualification refused", digest, err)
	}
	// Rollback's executable is a different generation, but it must consume the
	// same restored original profile and a separately native-qualified image.
	image.Generation = strings.Repeat("e", 32)
	fallback, err := executionCommissionEvidence(image, cold, snapshot)
	if err != nil || fallback == digest {
		t.Fatal("fallback image was not bound separately", fallback, err)
	}
}

func TestExecutionCommissionRejectsProfileSubstitutionBeforeQualification(t *testing.T) {
	for _, fault := range []string{"wrong-client", "unreleased", "changed-ini", "changed-ini-digest", "missing-ini", "duplicated-ini", "wrong-manifest", "wrong-execution", "wrong-completion", "missing-marker", "missing-fence", "zero-file", "invalid-image"} {
		t.Run(fault, func(t *testing.T) {
			image, cold, snapshot := executionCommissionFixture(t)
			var manifest profileCaptureManifest
			if json.Unmarshal(snapshot.OriginalManifest, &manifest) != nil {
				t.Fatal("fixture")
			}
			switch fault {
			case "wrong-client":
				image.ClientSID = testSID
			case "unreleased":
				snapshot.ReleasedForQualification = false
			case "changed-ini":
				cold.raw = []byte("[Connections]\nCOM8=modified\n")
				cold.digest = captureDigest(cold.raw)
			case "changed-ini-digest":
				cold.digest = strings.Repeat("0", 64)
			case "missing-ini", "duplicated-ini":
				var entries []profileCaptureEntry
				for _, entry := range manifest.Entries {
					if entry.Path == "opencpn.ini" && fault == "missing-ini" {
						continue
					}
					entries = append(entries, entry)
					if entry.Path == "opencpn.ini" && fault == "duplicated-ini" {
						entries = append(entries, entry)
					}
				}
				manifest.Entries = entries
				snapshot.OriginalManifest, _ = json.Marshal(manifest)
				snapshot.ManifestSHA256 = captureDigest(snapshot.OriginalManifest)
			case "wrong-manifest":
				snapshot.ManifestSHA256 = strings.Repeat("0", 64)
			case "wrong-execution":
				snapshot.Archive.ExecutionID = strings.Repeat("0", 32)
			case "wrong-completion":
				snapshot.Archive.CompletionSHA256 = strings.Repeat("0", 64)
			case "missing-marker":
				snapshot.Archive.OriginalMarker = nil
			case "missing-fence":
				snapshot.Archive.JournalFenceSHA256 = ""
			case "zero-file":
				cold.file = executionProfileID{}
			case "invalid-image":
				image.ExecutableSHA256 = ""
			}
			if digest, err := executionCommissionEvidence(image, cold, snapshot); err == nil || digest != "" {
				t.Fatal("incoherent qualification accepted", digest, err)
			}
		})
	}
}
