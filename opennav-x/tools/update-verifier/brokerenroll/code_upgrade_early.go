package brokerenroll

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"

	"example.com/opennav-update-verifier/releasepolicy"
)

const (
	codeBrokerStageSuffix = ".code-upgrade.broker-staged.json"
	codeHelperStageSuffix = ".code-upgrade.helper-staged.json"
	codeEarlyAbortSuffix  = ".code-upgrade.aborted.json"
)

// ErrCodeUpgradeAborted is a verified terminal recovery of original-code reader
// eligibility, not a successful upgrade or permission to retry this transaction.
var ErrCodeUpgradeAborted = errors.New("broker code upgrade aborted; verified original code retained; this transition cannot be retried")

type codeStageReceipt struct {
	Schema       int                `json:"schema"`
	IntentSHA256 string             `json:"intentSha256"`
	Name         string             `json:"name"`
	File         startupFileBinding `json:"file"`
}

type codeEarlyAbortRecord struct {
	Schema                   int    `json:"schema"`
	Purpose                  string `json:"purpose"`
	OriginalCompletionSHA256 string `json:"originalCompletionSha256"`
	IntentSHA256             string `json:"intentSha256"`
	BrokerStageSHA256        string `json:"brokerStageSha256"`
	HelperStageSHA256        string `json:"helperStageSha256"`
	ReservationSHA256        string `json:"reservationSha256"`
}

// Intent has no successor IDs yet. Never infer those IDs from staged paths.
func parseCodeIntent(raw []byte) (r codeUpgradeRecord, err error) {
	d := json.NewDecoder(bytes.NewReader(raw))
	d.DisallowUnknownFields()
	if len(raw) < 1 || len(raw) > maxCodeUpgradeRecordBytes || d.Decode(&r) != nil {
		return r, errors.New("canonical original code intent required")
	}
	canonical, _ := json.Marshal(r)
	p, e := releasepolicy.ParseBrokerCodeUpgrade(r.Policy)
	if e != nil || !bytes.Equal(canonical, raw) || r.Schema != 1 || r.Purpose != codeUpgradeRecordPurpose ||
		!validPin(r.OriginalCompletionSHA256) || !validPin(r.ClientRequestSHA256) || digest(r.Policy) != r.PolicySHA256 ||
		r.ReservationSHA256 != "" || r.Broker != (startupFileBinding{}) || r.Helper != (startupFileBinding{}) ||
		!validStartupDirectory(r.CodeParent) {
		return r, errors.New("exact original-only intent required")
	}
	ids := map[[16]byte]bool{r.CodeParent.FileID: true}
	for _, item := range []struct {
		file startupFileBinding
		sha  string
	}{{r.PreviousBroker, p.PreviousBrokerSHA256}, {r.PreviousHelper, p.PreviousHelperSHA256}} {
		if !validStartupFile(item.file, maxBrokerBytes) || item.file.SHA256 != item.sha || item.file.Volume != r.CodeParent.Volume || ids[item.file.FileID] {
			return r, errors.New("exact original code intent identities required")
		}
		ids[item.file.FileID] = true
	}
	return r, nil
}

func parseCodeStageReceipt(raw []byte, intentSHA, name string, artifact releasepolicy.Artifact, parent startupDirectoryBinding) (r codeStageReceipt, err error) {
	d := json.NewDecoder(bytes.NewReader(raw))
	d.DisallowUnknownFields()
	if len(raw) < 1 || len(raw) > maxCodeUpgradeRecordBytes || d.Decode(&r) != nil {
		return r, errors.New("complete canonical staged-code receipt required")
	}
	canonical, _ := json.Marshal(r)
	if !bytes.Equal(canonical, raw) || r.Schema != 1 || r.IntentSHA256 != intentSHA || !validPin(intentSHA) ||
		r.Name != name || (name != stagedBrokerName && name != stagedHelperName) ||
		!validStartupFile(r.File, maxBrokerBytes) || r.File.SHA256 != artifact.SHA256 || r.File.Bytes != artifact.Bytes ||
		r.File.Volume != parent.Volume || r.File.FileID == parent.FileID {
		return r, errors.New("exact intent-bound complete staged code required")
	}
	return r, nil
}

func codeEarlyAbortBytes(r codeEarlyAbortRecord) ([]byte, error) {
	if r.Schema != 1 || r.Purpose != "installed-broker-code-early-abort" || !validPin(r.OriginalCompletionSHA256) || !validPin(r.IntentSHA256) ||
		(r.BrokerStageSHA256 != "" && !validPin(r.BrokerStageSHA256)) || (r.HelperStageSHA256 != "" && !validPin(r.HelperStageSHA256)) ||
		(r.ReservationSHA256 != "" && !validPin(r.ReservationSHA256)) ||
		(r.HelperStageSHA256 != "" && r.BrokerStageSHA256 == "") || (r.ReservationSHA256 != "" && r.HelperStageSHA256 == "") {
		return nil, errors.New("exact terminal early-abort closure required")
	}
	return json.Marshal(r)
}

// Reuse the existing query-only retained-process/readiness/publication owner.
// Its only durable effect is the distinct abort record; no ready marker is made.
type codeUpgradeEarlyAborter struct{ finalizer *codeUpgradeFinalizer }

func (n *codeUpgradeEarlyAborter) check(ctx context.Context) error { return n.finalizer.check(ctx) }
func (n *codeUpgradeEarlyAborter) close() error                    { return n.finalizer.close() }
func (n *codeUpgradeEarlyAborter) resume(ctx context.Context) error {
	if err := n.finalizer.resume(ctx); err != nil {
		return err
	}
	return ErrCodeUpgradeAborted
}
