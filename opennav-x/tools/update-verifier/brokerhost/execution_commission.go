package brokerhost

import (
	"bytes"
	"encoding/json"

	"example.com/opennav-update-verifier/brokerstore"
)

// Copied evidence cannot commission an application. The native provider obtains
// these only after corroborating the live original image/capability, full-profile
// archive and actual fixed INI. It changes no sensor connection or user setting.
func executionCommissionEvidence(image brokerstore.GenerationApplicationIdentity, cold executionProfileOriginal, captured ExecutionProfileCaptureSnapshot) (string, error) {
	var manifest profileCaptureManifest
	if len(captured.OriginalManifest) == 0 || len(captured.OriginalManifest) > maxProfileManifestBytes ||
		json.Unmarshal(captured.OriginalManifest, &manifest) != nil {
		return "", errExecutionAuthorization
	}
	canonical, err := json.Marshal(manifest)
	if err != nil || !bytes.Equal(canonical, captured.OriginalManifest) ||
		manifest.Schema != 1 || manifest.Purpose != "protected-opencpn-profile-capture-v1" ||
		captureDigest(canonical) != captured.ManifestSHA256 || !captured.ReleasedForQualification ||
		manifest.ClientSID != image.ClientSID || manifest.ClientSID != captured.Archive.ClientSID ||
		manifest.ExecutionID != captured.Archive.ExecutionID || manifest.Generation != captured.Archive.Generation ||
		manifest.CompletionSHA256 != captured.Archive.CompletionSHA256 ||
		!requestPattern.MatchString(manifest.ExecutionID) || !requestPattern.MatchString(manifest.Generation) ||
		!hashPattern.MatchString(captured.Archive.SHA256) || !hashPattern.MatchString(manifest.CompletionSHA256) ||
		!hashPattern.MatchString(captured.Archive.JournalFenceSHA256) || len(captured.Archive.OriginalMarker) == 0 ||
		captured.Archive.Bytes < int64(len(canonical))+8 || captured.Archive.Bytes > brokerstore.MaxPayloadBytes ||
		!requestPattern.MatchString(image.Generation) || !commitPattern.MatchString(image.Commit) ||
		!hashPattern.MatchString(image.ExecutableSHA256) || len(cold.raw) == 0 || len(cold.raw) > executionProfileLimit ||
		captureDigest(cold.raw) != cold.digest || cold.directory == (executionProfileID{}) || cold.file == (executionProfileID{}) {
		return "", errExecutionAuthorization
	}
	ini := 0
	for _, entry := range manifest.Entries {
		if entry.Path == "opencpn.ini" {
			ini++
			if entry.Directory || entry.Bytes != int64(len(cold.raw)) || entry.SHA256 != cold.digest {
				return "", errExecutionAuthorization
			}
		}
	}
	if ini != 1 {
		return "", errExecutionAuthorization
	}
	raw, err := json.Marshal(struct {
		Schema                                                                                int
		Purpose                                                                               string
		Image                                                                                 brokerstore.GenerationApplicationIdentity
		Configuration, ManifestSHA256, ArchiveSHA256, ArchiveMarkerSHA256, JournalFenceSHA256 string
	}{1, "native-nonmutating-qualification-v1", image, cold.wire(), captured.ManifestSHA256,
		captured.Archive.SHA256, captureDigest(captured.Archive.OriginalMarker), captured.Archive.JournalFenceSHA256})
	if err != nil {
		return "", err
	}
	return captureDigest(raw), nil
}
