package brokerenroll

import (
	"bytes"
	"encoding/json"
	"errors"
	"example.com/opennav-update-verifier/releasepolicy"
)

const (
	codeDrainSuffix           = ".code-upgrade.drained.json"
	codeIntentSuffix          = ".code-upgrade.intent.json"
	codeReservationSuffix     = ".code-upgrade.json"
	codeSuccessorSuffix       = ".code-successor.json"
	codeSuccessorPairSuffix   = ".code-successor.pair"
	codeSuccessorMarkerSuffix = ".code-successor.once"
	codeUpgradeRecordPurpose  = "installed-broker-code-successor"
	maxCodeUpgradeRecordBytes = 2 << 20
	stagedBrokerName          = "broker.successor.exe"
	stagedHelperName          = "helper.successor.exe"
	previousBrokerName        = "broker.previous.exe"
	previousHelperName        = "helper.previous.exe"
)

type codeUpgradeRecord struct {
	Schema                   int                     `json:"schema"`
	Purpose                  string                  `json:"purpose"`
	OriginalCompletionSHA256 string                  `json:"originalCompletionSha256"`
	ClientRequestSHA256      string                  `json:"clientRequestSha256"`
	Policy                   []byte                  `json:"policy"`
	PolicySHA256             string                  `json:"policySha256"`
	ReservationSHA256        string                  `json:"reservationSha256"`
	CodeParent               startupDirectoryBinding `json:"codeParent"`
	PreviousBroker           startupFileBinding      `json:"previousBroker"`
	PreviousHelper           startupFileBinding      `json:"previousHelper"`
	Broker                   startupFileBinding      `json:"broker"`
	Helper                   startupFileBinding      `json:"helper"`
}

func parseCodeUpgradeRecord(raw []byte) (r codeUpgradeRecord, err error) {
	refuse := func() (codeUpgradeRecord, error) {
		return codeUpgradeRecord{}, errors.New("exact protected code upgrade record required")
	}
	if len(raw) < 1 || len(raw) > maxCodeUpgradeRecordBytes {
		return refuse()
	}
	d := json.NewDecoder(bytes.NewReader(raw))
	d.DisallowUnknownFields()
	if d.Decode(&r) != nil {
		return refuse()
	}
	canonical, e := json.Marshal(r)
	if e != nil || !bytes.Equal(canonical, raw) || r.Schema != 1 || r.Purpose != codeUpgradeRecordPurpose || !validPin(r.OriginalCompletionSHA256) || !validPin(r.ClientRequestSHA256) || digest(r.Policy) != r.PolicySHA256 || (r.ReservationSHA256 != "" && !validPin(r.ReservationSHA256)) || !validStartupDirectory(r.CodeParent) {
		return refuse()
	}
	p, e := releasepolicy.ParseBrokerCodeUpgrade(r.Policy)
	if e != nil {
		return refuse()
	}
	for _, item := range []struct {
		file startupFileBinding
		sha  string
		size int64
	}{{r.PreviousBroker, p.PreviousBrokerSHA256, 0}, {r.PreviousHelper, p.PreviousHelperSHA256, 0}, {r.Broker, p.Bootstrap.Broker.SHA256, p.Bootstrap.Broker.Bytes}, {r.Helper, p.Bootstrap.Provisioner.SHA256, p.Bootstrap.Provisioner.Bytes}} {
		if !validStartupFile(item.file, maxBrokerBytes) || item.file.SHA256 != item.sha || (item.size != 0 && item.file.Bytes != item.size) || item.file.Volume != r.CodeParent.Volume {
			return refuse()
		}
	}
	ids := map[[16]byte]bool{}
	for _, id := range [][16]byte{r.CodeParent.FileID, r.PreviousBroker.FileID, r.PreviousHelper.FileID, r.Broker.FileID, r.Helper.FileID} {
		if ids[id] {
			return refuse()
		}
		ids[id] = true
	}
	return r, nil
}
func marshalCodeUpgradeRecord(r codeUpgradeRecord) ([]byte, error) {
	raw, e := json.Marshal(r)
	if e != nil {
		return nil, e
	}
	_, e = parseCodeUpgradeRecord(raw)
	return raw, e
}
func codeSuccessorMarker(sha string) []byte {
	if !validPin(sha) {
		return nil
	}
	return []byte("SKAGER-code-upgrade-v1:1\n" + sha + "\n")
}
func sameCodeFile(a, b startupFileBinding) bool {
	return a.Volume == b.Volume && a.FileID == b.FileID && a.Bytes == b.Bytes && a.SHA256 == b.SHA256 && a.DescriptorSHA256 == b.DescriptorSHA256
}

type codeDrainRecord struct {
	Schema            int    `json:"schema"`
	ReservationSHA256 string `json:"reservationSha256"`
	PID               uint32 `json:"pid"`
	Birth             uint64 `json:"birth"`
	ExitCode          uint32 `json:"exitCode"`
}

func parseCodeDrainRecord(raw []byte, sha string) (codeDrainRecord, error) {
	var r codeDrainRecord
	d := json.NewDecoder(bytes.NewReader(raw))
	d.DisallowUnknownFields()
	e := d.Decode(&r)
	canonical, _ := json.Marshal(r)
	if e != nil || !bytes.Equal(raw, canonical) || r.Schema != 1 || r.ReservationSHA256 != sha || !validPin(sha) || r.PID == 0 || r.Birth == 0 || r.ExitCode != 0 {
		return codeDrainRecord{}, errors.New("exact completed original process drain required")
	}
	return r, nil
}
