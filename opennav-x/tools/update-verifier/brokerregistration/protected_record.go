package brokerregistration

import (
	"encoding/json"
	"errors"
)

const MaxProtectedRecordBytes = 1 << 20

// ProtectedRecord is the existing Schema2 protected-store envelope. []byte's
// canonical base64 encoding preserves every original request/trust byte. This
// value, its encoding and its hash are comparison data, never independent
// approval, native store custody, fresh enrollment or mutation authority.
type ProtectedRecord struct {
	Schema                int    `json:"schema"`
	ClientSID             string `json:"clientSid"`
	ApprovedRequestSHA256 string `json:"approvedRequestSha256"`
	ApprovedRequest       []byte `json:"approvedRequest"`
}

// EncodeProtectedRecord is shared by the original protected writer and later
// expected-identity comparisons. Reparse privately retained original bytes:
// caller-mutated exported request fields must not alter the stored identity.
// Identity-only validation permits comparisons after anchor rotation/expiry;
// a fresh writer MUST still call ParseApprovedRequest with its live clock before
// encoding. Encoding does not authenticate approval or authorize persistence.
func EncodeProtectedRecord(request Request) ([]byte, error) {
	original, err := ParseRetainedApprovedRequest(request.OriginalBytes(), request.RequestSHA256)
	if err != nil {
		return nil, err
	}
	raw, err := json.Marshal(ProtectedRecord{Schema: 2, ClientSID: original.ClientSID,
		ApprovedRequestSHA256: original.RequestSHA256, ApprovedRequest: original.OriginalBytes()})
	if err != nil || len(raw) > MaxProtectedRecordBytes {
		return nil, errors.New("bounded original protected client registration required")
	}
	return raw, nil
}
