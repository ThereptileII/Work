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

const repairCurrent = "repair-current"

func runCustomerRepair(ctx context.Context, out io.Writer) error {
	return runCustomerRepairWith(ctx, out, customerRollbackOperations{open: func(ctx context.Context) (customerRollbackOwner, error) {
		return brokerenroll.OpenInstalledPayloadMaintenanceNative(ctx)
	}, call: launcher.CallMaintenanceNative, confirm: confirmCustomerRepairNative, newID: func() (string, error) {
		var id [16]byte
		_, err := rand.Read(id[:])
		return hex.EncodeToString(id[:]), err
	}})
}

// Fixed customer operation: no SID, source/destination, package, pins or target
// can enter via argv. Inspect is repair-specific and authenticates retained
// source even when ordinary selected payload validation correctly refuses.
func runCustomerRepairWith(ctx context.Context, out io.Writer, op customerRollbackOperations) (retErr error) {
	refused := errors.New("SKAGER repair completion could not be verified; preserve recovery and collect diagnostics; do not retry automatically")
	defer func() {
		if retErr != nil {
			retErr = refused
		}
	}()
	if ctx == nil || ctx.Err() != nil || out == nil || op.open == nil || op.call == nil || op.confirm == nil || op.newID == nil {
		return refused
	}
	ctx, cancel := context.WithTimeout(ctx, 90*time.Second)
	defer cancel()
	owner, err := op.open(ctx)
	if owner != nil {
		defer func() { retErr = errors.Join(retErr, owner.Close()) }()
	}
	if err != nil || owner == nil || owner.Check(ctx) != nil {
		return refused
	}
	seen := map[string]bool{}
	request := func(operation string) (brokerpipe.MaintenanceRequest, error) {
		id, err := op.newID()
		if err != nil || !manualRollbackID(id) || seen[id] {
			return brokerpipe.MaintenanceRequest{}, refused
		}
		seen[id] = true
		return brokerpipe.MaintenanceRequest{Schema: 1, Operation: operation, RequestID: id}, nil
	}
	query, err := request(brokerpipe.RepairInspect)
	if err != nil {
		return err
	}
	plan, err := op.call(ctx, query)
	if err != nil || !brokerpipe.ValidMaintenanceClientResult(plan, query) || owner.Check(ctx) != nil {
		return refused
	}
	// Freeze comparison data before user UI. No callback can change selection.
	head, version, commit, oldGeneration := plan.SelectedHeadSHA256, plan.Version, plan.Commit, plan.Generation
	yes, err := op.confirm(ctx, plan)
	if err != nil || owner.Check(ctx) != nil || ctx.Err() != nil {
		return refused
	}
	if !yes {
		if err = owner.Close(); err != nil {
			return err
		}
		return json.NewEncoder(out).Encode(struct {
			Schema int    `json:"schema"`
			State  string `json:"state"`
		}{1, "repair-cancelled"})
	}
	mutation, err := request(brokerpipe.RepairCurrent)
	if err != nil {
		return err
	}
	mutation.ExpectedSelectedHeadSHA256 = head
	selected, err := op.call(ctx, mutation) // exactly once, including ambiguous failure
	if err != nil || !brokerpipe.ValidMaintenanceClientResult(selected, mutation) || selected.Generation == oldGeneration || selected.Version != version || selected.Commit != commit || owner.Check(ctx) != nil {
		return refused
	}
	readback, err := request(brokerpipe.SelectedHistory)
	if err != nil {
		return err
	}
	page := 0
	readback.Page = &page
	observed, err := op.call(ctx, readback)
	if err != nil || !brokerpipe.ValidSelectedHistoryResult(observed, readback) || observed.RepairSourceQualification == nil || observed.Generation != selected.Generation || observed.CompletionSHA256 != selected.CompletionSHA256 || observed.SelectedHeadSHA256 != selected.SelectedHeadSHA256 || observed.Version != version || observed.Commit != commit || owner.Check(ctx) != nil || ctx.Err() != nil {
		return refused
	}
	if err = errors.Join(owner.Close(), ctx.Err()); err != nil {
		return err
	}
	return json.NewEncoder(out).Encode(struct {
		Schema             int    `json:"schema"`
		State              string `json:"state"`
		Version            string `json:"version"`
		Commit             string `json:"commit"`
		SelectedHeadSHA256 string `json:"selectedHeadSha256"`
	}{1, "verified-customer-repair-selected", version, commit, observed.SelectedHeadSHA256})
}
