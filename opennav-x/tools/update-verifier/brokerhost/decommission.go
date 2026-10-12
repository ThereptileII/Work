package brokerhost

import (
	"context"
	"errors"
	"example.com/opennav-update-verifier/brokerpipe"
	"example.com/opennav-update-verifier/brokerstore"
)

var errDecommissionRefused = errors.New("original decommission preparation refused or uncertain")

func validDecommissionRequest(q brokerpipe.MaintenanceRequest) bool {
	return q.Schema == 1 && q.Operation == brokerpipe.PrepareDecommission && requestPattern.MatchString(q.RequestID) && hashPattern.MatchString(q.ExpectedSelectedHeadSHA256) && q.TargetGeneration == "" && q.TargetCompletionSHA256 == "" && q.Page == nil && q.InventorySHA256 == ""
}

// Complete only a receipt returned by the one-shot original Store operation.
// Its concrete implementation seals provenance; this private seam permits fault
// coverage without manufacturing native service/profile authority.
type decommissionPreparationReceipt interface {
	Validate(context.Context) error
	CloseExecutionJournalReader(context.Context, *brokerstore.ExecutionJournalReader) error
	EvidenceSHA256(context.Context) (string, error)
}

func finishDecommissionPreparation(ctx context.Context, receipt decommissionPreparationReceipt, readers []*brokerstore.ExecutionJournalReader, standalone func(context.Context) error) (string, error) {
	if ctx == nil || receipt == nil || standalone == nil {
		return "", brokerstore.ErrDecommissionUncertain
	}
	for _, reader := range readers {
		if err := receipt.CloseExecutionJournalReader(ctx, reader); err != nil {
			return "", errors.Join(brokerstore.ErrDecommissionUncertain, err)
		}
	}
	if err := errors.Join(receipt.Validate(ctx), standalone(ctx), ctx.Err()); err != nil {
		return "", errors.Join(brokerstore.ErrDecommissionUncertain, err)
	}
	digest, err := receipt.EvidenceSHA256(ctx)
	if err != nil || !hashPattern.MatchString(digest) {
		return "", errors.Join(brokerstore.ErrDecommissionUncertain, err)
	}
	return digest, nil
}
