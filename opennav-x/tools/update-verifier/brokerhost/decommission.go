package brokerhost

import (
	"context"
	"errors"
	"example.com/opennav-update-verifier/brokerpipe"
	"example.com/opennav-update-verifier/brokerstore"
)

var errDecommissionRefused = errors.New("original decommission preparation refused or uncertain")

func validDecommissionRequest(q brokerpipe.MaintenanceRequest) bool {
	return q.Schema == 1 && (q.Operation == brokerpipe.PrepareDecommission || q.Operation == brokerpipe.PrepareUninstall) && requestPattern.MatchString(q.RequestID) && hashPattern.MatchString(q.ExpectedSelectedHeadSHA256) && q.TargetGeneration == "" && q.TargetCompletionSHA256 == "" && q.Page == nil && q.InventorySHA256 == ""
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

// Original retirement custody is provided only by brokerstore's sealed native
// transaction. This seam lets portable faults withhold the authenticated result.
type retirementReceipt interface {
	Validate(context.Context) error
	FenceSHA256(context.Context) (string, error)
	EvidenceSHA256(context.Context) (string, error)
}

func finishRetirement(ctx context.Context, receipt retirementReceipt, standalone func(context.Context) error) (fence, retirement string, err error) {
	if ctx == nil || receipt == nil || standalone == nil {
		return "", "", brokerstore.ErrDecommissionUncertain
	}
	if err = errors.Join(receipt.Validate(ctx), standalone(ctx), ctx.Err()); err != nil {
		return "", "", errors.Join(brokerstore.ErrDecommissionUncertain, err)
	}
	fence, err = receipt.FenceSHA256(ctx)
	if err != nil || !hashPattern.MatchString(fence) {
		return "", "", errors.Join(brokerstore.ErrDecommissionUncertain, err)
	}
	retirement, err = receipt.EvidenceSHA256(ctx)
	if err != nil || !hashPattern.MatchString(retirement) {
		return "", "", errors.Join(brokerstore.ErrDecommissionUncertain, err)
	}
	if err = errors.Join(receipt.Validate(ctx), standalone(ctx), ctx.Err()); err != nil {
		return "", "", errors.Join(brokerstore.ErrDecommissionUncertain, err)
	}
	return fence, retirement, nil
}

func commitPreparedRetirement(ctx context.Context, releaseSelected func() error, commit func(context.Context) (retirementReceipt, error), standalone func(context.Context) error) (string, string, error) {
	if ctx == nil || releaseSelected == nil || commit == nil || standalone == nil || ctx.Err() != nil {
		return "", "", brokerstore.ErrDecommissionUncertain
	}
	if err := releaseSelected(); err != nil {
		return "", "", errors.Join(brokerstore.ErrDecommissionUncertain, err)
	}
	if err := ctx.Err(); err != nil {
		return "", "", errors.Join(brokerstore.ErrDecommissionUncertain, err)
	}
	receipt, err := commit(ctx)
	if err != nil || receipt == nil {
		return "", "", errors.Join(brokerstore.ErrDecommissionUncertain, err)
	}
	return finishRetirement(ctx, receipt, standalone)
}

// The synchronous Store callback publishes one interim fence and waits for the
// same operator to restore startup and close descendant handles. Final retirement
// is accepted only for that exact fence. Neither an omitted callback nor a caught
// callback failure can produce a final witness.
func commitPreparedStartupRetirement(ctx context.Context, releaseSelected func() error, commit func(context.Context, func(context.Context, string) error) (retirementReceipt, error), standalone func(context.Context) error, publishFence func(context.Context, string) error) (string, string, error) {
	if commit == nil || publishFence == nil || standalone == nil {
		return "", "", brokerstore.ErrDecommissionUncertain
	}
	var publishedFence string
	attempted, drained, poisoned := false, false, false
	fence, retirement, err := commitPreparedRetirement(ctx, releaseSelected, func(ctx context.Context) (retirementReceipt, error) {
		receipt, err := commit(ctx, func(callbackCtx context.Context, digest string) error {
			if attempted || callbackCtx == nil || callbackCtx.Err() != nil || ctx.Err() != nil || !hashPattern.MatchString(digest) {
				poisoned = true
				return brokerstore.ErrDecommissionUncertain
			}
			attempted = true
			if err := standalone(callbackCtx); err != nil {
				poisoned = true
				return errors.Join(brokerstore.ErrDecommissionUncertain, err)
			}
			if err := publishFence(callbackCtx, digest); err != nil {
				poisoned = true
				return errors.Join(brokerstore.ErrDecommissionUncertain, err)
			}
			if err := errors.Join(standalone(callbackCtx), callbackCtx.Err(), ctx.Err()); err != nil {
				poisoned = true
				return errors.Join(brokerstore.ErrDecommissionUncertain, err)
			}
			publishedFence, drained = digest, true
			return nil
		})
		if err != nil || !drained || poisoned {
			return nil, errors.Join(brokerstore.ErrDecommissionUncertain, err)
		}
		return receipt, nil
	}, standalone)
	if err != nil || fence != publishedFence || !drained || poisoned {
		return "", "", errors.Join(brokerstore.ErrDecommissionUncertain, err)
	}
	return fence, retirement, nil
}
