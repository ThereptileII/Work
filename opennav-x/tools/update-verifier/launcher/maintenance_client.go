package launcher

import (
	"context"
	"errors"

	"example.com/opennav-update-verifier/brokerpipe"
)

var errMaintenanceClient = errors.New("native maintenance client refused or uncertain")

type maintenanceClientCustody interface {
	Validate(context.Context) error
	Close() error
}
type maintenanceClientPrepare func(context.Context) (maintenanceClientCustody, error)
type maintenanceClientCall func(context.Context, brokerpipe.MaintenanceRequest) (brokerpipe.MaintenanceResult, error)

// callMaintenanceWithCustody keeps the actual caller query and temporary folder
// access owners live for one broker call. Closing the folder owner must restore
// the ordinary query descriptor before the query owner releases native handles.
func callMaintenanceWithCustody(ctx context.Context, q brokerpipe.MaintenanceRequest, prepareQuery, prepareFolder maintenanceClientPrepare, call maintenanceClientCall) (result brokerpipe.MaintenanceResult, retErr error) {
	if ctx == nil || ctx.Err() != nil || prepareQuery == nil || prepareFolder == nil || call == nil || !maintenanceClientRequest(q) {
		return result, errMaintenanceClient
	}
	query, err := prepareQuery(ctx)
	if query != nil {
		defer func() {
			retErr = errors.Join(retErr, query.Close(), ctx.Err())
			if retErr != nil {
				result = brokerpipe.MaintenanceResult{}
			}
		}()
	}
	if err != nil || query == nil {
		return result, errors.Join(errMaintenanceClient, err)
	}
	folder, err := prepareFolder(ctx)
	if folder != nil {
		defer func() {
			retErr = errors.Join(retErr, folder.Close())
			if retErr != nil {
				result = brokerpipe.MaintenanceResult{}
			}
		}()
	}
	if err != nil || folder == nil {
		return result, errors.Join(errMaintenanceClient, err)
	}
	if err := errors.Join(query.Validate(ctx), folder.Validate(ctx), ctx.Err()); err != nil {
		return result, errors.Join(errMaintenanceClient, err)
	}
	result, err = call(ctx, q)
	// An error after transmission can mean a committed selection. Preserve the
	// error, drain both owners, clear all results and never repeat the request.
	retErr = errors.Join(err, query.Validate(ctx), folder.Validate(ctx), ctx.Err())
	if retErr != nil || !maintenanceClientResult(result, q) {
		result = brokerpipe.MaintenanceResult{}
		return result, errors.Join(errMaintenanceClient, retErr)
	}
	return result, nil
}

func maintenanceClientHex(value string, size int) bool {
	if len(value) != size {
		return false
	}
	for _, c := range value {
		if !(c >= '0' && c <= '9' || c >= 'a' && c <= 'f') {
			return false
		}
	}
	return true
}
func maintenanceClientRequest(q brokerpipe.MaintenanceRequest) bool {
	return brokerpipe.ValidMaintenanceClientRequest(q)
}
func maintenanceClientResult(r brokerpipe.MaintenanceResult, q brokerpipe.MaintenanceRequest) bool {
	return brokerpipe.ValidMaintenanceClientResult(r, q)
}
