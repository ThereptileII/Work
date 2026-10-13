package launcher

import (
	"context"
	"errors"

	"example.com/opennav-update-verifier/brokerpipe"
)

type decommissionClientCall func(context.Context, brokerpipe.MaintenanceRequest) (*brokerpipe.DecommissionWitness, error)

// Restore the temporary folder grant before the ordinary query owner. Any
// validation/restoration uncertainty destroys the returned native witness and
// withholds success; a transmitted action is never automatically repeated.
func callDecommissionWithCustody(ctx context.Context, q brokerpipe.MaintenanceRequest, prepareQuery, prepareFolder maintenanceClientPrepare, call decommissionClientCall) (witness *brokerpipe.DecommissionWitness, retErr error) {
	if ctx == nil || ctx.Err() != nil || prepareQuery == nil || prepareFolder == nil || call == nil || q.Schema != 1 || (q.Operation != brokerpipe.PrepareDecommission && q.Operation != brokerpipe.PrepareUninstall) || !maintenanceClientHex(q.RequestID, 32) || !maintenanceClientHex(q.ExpectedSelectedHeadSHA256, 64) || q.TargetGeneration != "" || q.TargetCompletionSHA256 != "" || q.Page != nil || q.InventorySHA256 != "" {
		return nil, errMaintenanceClient
	}
	defer func() {
		if retErr != nil && witness != nil {
			retErr = errors.Join(retErr, witness.Close())
			witness = nil
		}
	}()
	query, err := prepareQuery(ctx)
	if query != nil {
		defer func() { retErr = errors.Join(retErr, query.Close(), ctx.Err()) }()
	}
	if err != nil || query == nil {
		return nil, errors.Join(errMaintenanceClient, err)
	}
	folder, err := prepareFolder(ctx)
	if folder != nil {
		defer func() { retErr = errors.Join(retErr, folder.Close()) }()
	}
	if err != nil || folder == nil {
		return nil, errors.Join(errMaintenanceClient, err)
	}
	if err = errors.Join(query.Validate(ctx), folder.Validate(ctx), ctx.Err()); err != nil {
		return nil, errors.Join(errMaintenanceClient, err)
	}
	witness, err = call(ctx, q)
	retErr = errors.Join(err, query.Validate(ctx), folder.Validate(ctx), ctx.Err())
	if witness == nil {
		retErr = errors.Join(retErr, errMaintenanceClient)
	}
	return witness, retErr
}
