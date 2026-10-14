package brokerenroll

import (
	"context"
	"errors"
)

// This private capability is created only by the native fixed-location reader.
// JSON, caller-selected paths and a present ACL cannot implement this public API.
// Admin/SYSTEM remain trusted under the original enrollment threat model.
type installedMaintenanceCompletion interface {
	check(context.Context) error
	close() error
}

type installedMaintenanceInputs struct {
	completion  installedMaintenanceCompletion
	authority   maintenanceAuthority
	publication *ApprovedStartupPublication
	baseline    ApprovedClientBaseline
}

// OpenInstalledMaintenanceNative prepares the actual current user's completed
// installation for uninstall. It takes no approval hashes, SID, baseline path or
// cleanup target from Maintain. The fixed native executable and original
// machine-protected completion must match, and the normal broker must be live.
// No deletion or SCM change occurs until the same owner receives a sealed live
// terminal witness and DecommissionInstalled is explicitly requested.
func OpenInstalledMaintenanceNative(ctx context.Context) (*MaintenanceOwner, error) {
	return openInstalledMaintenanceWith(ctx, readInstalledCompletionNative, maintenanceOperations{
		authenticate: maintenanceOperatorNative, openDecommission: openDecommissionNative,
		poll: servicePoll, limit: maintenanceStopLimit,
	}, prepareInstalledFromCompletionNative)
}

func openInstalledMaintenanceWith(ctx context.Context, read func(context.Context) (installedMaintenanceInputs, error), op maintenanceOperations,
	prepare func(context.Context, *MaintenanceOwner, ApprovedClientBaseline) error) (result *MaintenanceOwner, resultErr error) {
	if ctx == nil || ctx.Err() != nil || read == nil || prepare == nil {
		return nil, errors.New("live fixed installed maintenance required")
	}
	inputs, err := read(ctx)
	transferred := false
	defer func() {
		if !transferred && inputs.completion != nil {
			resultErr = errors.Join(resultErr, inputs.completion.close())
		}
	}()
	if err != nil || inputs.completion == nil || inputs.publication == nil || inputs.baseline == nil {
		return nil, errors.Join(errors.New("original completed native installation required"), err)
	}
	if err := errors.Join(inputs.completion.check(ctx), inputs.baseline.Validate()); err != nil {
		return nil, err
	}
	// This transfers all original completion/record/baseline handle custody to
	// the same service owner. The identity parser is never a mutation receipt.
	transferred = true
	m, err := openMaintenanceAuthorityWith(ctx, inputs.authority, op, true, inputs.completion)
	if err != nil {
		return nil, err
	}
	defer func() {
		if resultErr != nil {
			resultErr = errors.Join(resultErr, m.Close())
		}
	}()
	if err := m.PrepareStartupRestoration(ctx, inputs.publication); err != nil {
		return nil, err
	}
	if err := prepare(ctx, m, inputs.baseline); err != nil {
		return nil, err
	}
	if err := m.Check(ctx); err != nil {
		return nil, err
	}
	return m, nil
}
