package main

import (
	"context"
	"crypto/rand"
	"encoding/hex"
	"encoding/json"
	"errors"
	"io"
	"time"

	"example.com/opennav-update-verifier/brokerenroll"
	"example.com/opennav-update-verifier/brokerpipe"
	"example.com/opennav-update-verifier/launcher"
)

const uninstallCurrent = "uninstall-current"

type currentInstalledOwner interface {
	Check(context.Context) error
	RestoreStartupAndDrain(context.Context, *brokerpipe.UninstallFenceWitness) error
	BindDecommission(context.Context, *brokerpipe.DecommissionWitness) error
	DecommissionInstalled(context.Context) (brokerenroll.InstalledUninstallResult, error)
	Close() error
}
type currentInstalledOperations struct {
	open         func(context.Context) (currentInstalledOwner, error)
	history      maintenanceCaller
	uninstall    protectedUninstallCaller
	newID        func() (string, error)
	closeWitness func(*brokerpipe.DecommissionWitness) error
}

func currentInstalledRequested(args []string) bool {
	return len(args) > 0 && (args[0] == "--maintenance="+uninstallCurrent || len(args) > 1 && args[0] == "--maintenance" && args[1] == uninstallCurrent)
}
func runCurrentInstalled(ctx context.Context, args []string, output io.Writer) error {
	return runCurrentInstalledWith(ctx, args, output, currentInstalledOperations{
		open: func(ctx context.Context) (currentInstalledOwner, error) {
			return brokerenroll.OpenInstalledMaintenanceNative(ctx)
		},
		history:   launcher.CallMaintenanceNative,
		uninstall: launcher.CallProtectedUninstallNative,
		newID: func() (string, error) {
			var id [16]byte
			_, err := rand.Read(id[:])
			return hex.EncodeToString(id[:]), err
		},
		closeWitness: func(w *brokerpipe.DecommissionWitness) error { return w.Close() },
	})
}

// Customer maintenance derives every privileged input from original protected
// installation owners. The only command argument selects this explicit action.
// No CLI approval hashes, directories, JSON receipts or caller-selected SID.
func runCurrentInstalledWith(ctx context.Context, args []string, output io.Writer, op currentInstalledOperations) (resultErr error) {
	refused := errors.New("SKAGER maintenance could not verify completion; preserve recovery and collect diagnostics")
	defer func() {
		if resultErr != nil {
			resultErr = refused
		}
	}()
	if ctx == nil || ctx.Err() != nil || output == nil || op.open == nil || op.history == nil || op.uninstall == nil || op.newID == nil || op.closeWitness == nil ||
		!(len(args) == 1 && args[0] == "--maintenance="+uninstallCurrent || len(args) == 2 && args[0] == "--maintenance" && args[1] == uninstallCurrent) {
		return refused
	}
	ctx, cancel := context.WithTimeout(ctx, 90*time.Second)
	defer cancel()
	owner, err := op.open(ctx)
	if owner != nil {
		defer func() { resultErr = errors.Join(resultErr, owner.Close()) }()
	}
	if err != nil || owner == nil {
		return refused
	}
	if err := owner.Check(ctx); err != nil {
		return err
	}
	historyID, err := op.newID()
	if err != nil || !manualRollbackID(historyID) {
		return refused
	}
	page := 0
	query := brokerpipe.MaintenanceRequest{Schema: 1, Operation: brokerpipe.SelectedHistory, RequestID: historyID, Page: &page}
	selected, err := op.history(ctx, query)
	if err != nil || ctx.Err() != nil || !brokerpipe.ValidSelectedHistoryResult(selected, query) {
		return refused
	}
	if err := owner.Check(ctx); err != nil {
		return err
	}
	requestID, err := op.newID()
	if err != nil || !manualRollbackID(requestID) || requestID == historyID {
		return refused
	}
	request := brokerpipe.MaintenanceRequest{Schema: 1, Operation: brokerpipe.PrepareUninstall, RequestID: requestID, ExpectedSelectedHeadSHA256: selected.SelectedHeadSHA256}
	witness, err := op.uninstall(ctx, request, owner.RestoreStartupAndDrain)
	if witness != nil {
		defer func() { resultErr = errors.Join(resultErr, op.closeWitness(witness)) }()
	}
	if err != nil || witness == nil || ctx.Err() != nil {
		return refused
	}
	if err := owner.BindDecommission(ctx, witness); err != nil {
		return err
	}
	result, err := owner.DecommissionInstalled(ctx)
	if err != nil || result.Service != brokerenroll.ServiceMarkedForRemoval || result.Integration != "verified-owned-members-removed" || result.Registration != "unregistered" || result.Recovery != "preserved" {
		return refused
	}
	if err := errors.Join(owner.Close(), op.closeWitness(witness), ctx.Err()); err != nil {
		return err
	}
	return json.NewEncoder(output).Encode(struct {
		Schema   int    `json:"schema"`
		State    string `json:"state"`
		Recovery string `json:"recovery"`
	}{1, "verified-installed-integration-uninstalled", result.Recovery})
}
