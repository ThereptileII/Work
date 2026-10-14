package brokerenroll

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"

	"example.com/opennav-update-verifier/brokerregistration"
)

const installedCompletionPurpose = "original-installed-maintenance"
const maxInstalledCompletionBytes = 2 << 20
const maxInstalledCompletionMarkerBytes = 128

// This private producer record preserves authority established by the original
// approved install. Parsing a record establishes no native custody or authority.
// Admin/SYSTEM remain trusted machine principals, as in createFreshFiles.
type installedCompletionRecord struct {
	Schema                   int                             `json:"schema"`
	Purpose                  string                          `json:"purpose"`
	ClientSID                string                          `json:"clientSid"`
	ClientRequest            []byte                          `json:"clientRequest"`
	ClientRequestSHA256      string                          `json:"clientRequestSha256"`
	StartupPublication       []byte                          `json:"startupPublication"`
	StartupPublicationSHA256 string                          `json:"startupPublicationSha256"`
	Parent                   startupDirectoryBinding         `json:"parent"`
	Request                  startupFileBinding              `json:"request"`
	Publication              startupFileBinding              `json:"publication"`
	Baseline                 startupFileBinding              `json:"baseline"`
	Broker                   startupFileBinding              `json:"broker"`
	MaintenanceHelper        startupFileBinding              `json:"maintenanceHelper"`
	BootstrapCache           *installedBootstrapCacheBinding `json:"bootstrapCache,omitempty"`
}

type installedCompletion interface {
	Complete(context.Context, func(context.Context) error) error
	Close() error
}

// Only the original native publication implements this in production. It takes
// the actual enrollment and baseline owners, never caller declarations/pins.
type installedCompletionPublisher interface {
	prepareInstalledCompletion(context.Context, enrollmentFiles, *brokerregistration.Request, ApprovedClientBaseline) (installedCompletion, error)
}

func marshalInstalledCompletion(r installedCompletionRecord) ([]byte, error) {
	raw, err := json.Marshal(r)
	if err != nil {
		return nil, err
	}
	if _, err = parseInstalledCompletion(raw); err != nil {
		return nil, err
	}
	return raw, nil
}

func parseInstalledCompletion(raw []byte) (r installedCompletionRecord, err error) {
	refuse := func() (installedCompletionRecord, error) {
		return installedCompletionRecord{}, errors.New("exact original installed maintenance completion required")
	}
	if len(raw) == 0 || len(raw) > maxInstalledCompletionBytes {
		return refuse()
	}
	d := json.NewDecoder(bytes.NewReader(raw))
	d.DisallowUnknownFields()
	if d.Decode(&r) != nil {
		return refuse()
	}
	canonical, e := json.Marshal(r)
	if e != nil || !bytes.Equal(canonical, raw) || (r.Schema != 1 && r.Schema != 2) || r.Purpose != installedCompletionPurpose || !validStartupDirectory(r.Parent) {
		return refuse()
	}
	if (r.Schema == 1 && r.BootstrapCache != nil) || (r.Schema == 2 && !validInstalledBootstrapCache(r.BootstrapCache)) {
		return refuse()
	}
	request, e := brokerregistration.ParseRetainedApprovedRequest(r.ClientRequest, r.ClientRequestSHA256)
	if e != nil || request.ClientSID != r.ClientSID {
		return refuse()
	}
	if _, e = ParseApprovedStartupPublication(r.StartupPublication, r.StartupPublicationSHA256, r.ClientRequest, r.ClientRequestSHA256); e != nil {
		return refuse()
	}
	if !validStartupFile(r.Request, brokerregistration.MaxRequestBytes) || r.Request.Bytes != int64(len(r.ClientRequest)) || r.Request.SHA256 != r.ClientRequestSHA256 ||
		!validStartupFile(r.Publication, maxStartupBindingBytes) || r.Publication.Bytes != int64(len(r.StartupPublication)) || r.Publication.SHA256 != r.StartupPublicationSHA256 ||
		!validStartupFile(r.Baseline, maxClientBaselineBytes) || !validStartupFile(r.Broker, maxBrokerBytes) || !validStartupFile(r.MaintenanceHelper, maxBrokerBytes) {
		return refuse()
	}
	seen := map[struct {
		volume uint64
		id     [16]byte
	}]bool{}
	for _, id := range []startupDirectoryBinding{r.Parent, r.Request.startupDirectoryBinding, r.Publication.startupDirectoryBinding, r.Baseline.startupDirectoryBinding, r.Broker.startupDirectoryBinding, r.MaintenanceHelper.startupDirectoryBinding} {
		key := struct {
			volume uint64
			id     [16]byte
		}{id.Volume, id.FileID}
		if seen[key] {
			return refuse()
		}
		seen[key] = true
	}
	if r.Request.Volume != r.Parent.Volume || r.Baseline.Volume != r.Parent.Volume || r.MaintenanceHelper.Volume != r.Broker.Volume {
		return refuse()
	}
	return r, nil
}

func installedCompletionMarker(recordSHA string, completed bool) []byte {
	if !validPin(recordSHA) {
		return nil
	}
	state := "0"
	if completed {
		state = "1"
	}
	return []byte("SKAGER-installed-maintenance-v1:" + state + "\n" + recordSHA + "\n")
}

func validateInstalledCompletionMarker(raw []byte, recordSHA string) error {
	want := installedCompletionMarker(recordSHA, true)
	if len(want) == 0 || len(raw) > maxInstalledCompletionMarkerBytes || !bytes.Equal(raw, want) {
		return errors.New("completed original installed maintenance reservation required")
	}
	return nil
}
