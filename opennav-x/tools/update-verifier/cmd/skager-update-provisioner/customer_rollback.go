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

const rollbackCurrent = "rollback-current"

type customerRollbackOwner interface {
	Check(context.Context) error
	Close() error
}
type customerRollbackOperations struct {
	open    func(context.Context) (customerRollbackOwner, error)
	call    maintenanceCaller
	confirm func(context.Context, brokerpipe.MaintenanceResult) (bool, error)
	newID   func() (string, error)
}

func runCustomerRollback(ctx context.Context, output io.Writer) error {
	return runCustomerRollbackWith(ctx, output, customerRollbackOperations{
		open: func(ctx context.Context) (customerRollbackOwner, error) {
			return brokerenroll.OpenInstalledMaintenanceNative(ctx)
		},
		call: launcher.CallMaintenanceNative, confirm: confirmCustomerRollbackNative,
		newID: func() (string, error) {
			var id [16]byte
			_, err := rand.Read(id[:])
			return hex.EncodeToString(id[:]), err
		},
	})
}

// The fixed customer action accepts no SID, path, target or pin. Only original
// protected installation and broker observations supply those comparisons.
// Completion never promises a launch or compatibility with today's profile.
func runCustomerRollbackWith(ctx context.Context, output io.Writer, op customerRollbackOperations) (resultErr error) {
	refused := errors.New("SKAGER rollback completion could not be verified; preserve recovery and collect diagnostics; do not retry automatically")
	defer func() {
		if resultErr != nil {
			resultErr = refused
		}
	}()
	if ctx == nil || ctx.Err() != nil || output == nil || op.open == nil || op.call == nil || op.confirm == nil || op.newID == nil {
		return refused
	}
	ctx, cancel := context.WithTimeout(ctx, 90*time.Second)
	defer cancel()
	owner, err := op.open(ctx)
	if owner != nil {
		defer func() { resultErr = errors.Join(resultErr, owner.Close()) }()
	}
	if err != nil || owner == nil || owner.Check(ctx) != nil {
		return refused
	}
	ids := map[string]bool{}
	newRequest := func(operation string) (brokerpipe.MaintenanceRequest, error) {
		id, err := op.newID()
		if err != nil || !manualRollbackID(id) || ids[id] {
			return brokerpipe.MaintenanceRequest{}, refused
		}
		ids[id] = true
		return brokerpipe.MaintenanceRequest{Schema: 1, Operation: operation, RequestID: id}, nil
	}
	query, err := newRequest(brokerpipe.PreviousQualified)
	if err != nil {
		return err
	}
	plan, err := op.call(ctx, query)
	if err != nil || !brokerpipe.ValidPreviousQualifiedResult(plan, query) || owner.Check(ctx) != nil || ctx.Err() != nil {
		return refused
	}
	// Copy the authenticated target before UI code runs. Neither a dialog nor a
	// callback can replace the later broker request with other supplied pins.
	target, head := *plan.Previous, plan.SelectedHeadSHA256
	confirmed, err := op.confirm(ctx, plan)
	if err != nil || owner.Check(ctx) != nil || ctx.Err() != nil {
		return refused
	}
	if !confirmed {
		if err := owner.Close(); err != nil {
			return err
		}
		return json.NewEncoder(output).Encode(struct {
			Schema int    `json:"schema"`
			State  string `json:"state"`
		}{1, "rollback-cancelled"})
	}
	request, err := newRequest(brokerpipe.ManualRollback)
	if err != nil {
		return err
	}
	request.ExpectedSelectedHeadSHA256, request.TargetGeneration, request.TargetCompletionSHA256 = head, target.Generation, target.CompletionSHA256
	selected, err := op.call(ctx, request) // exactly one mutation attempt; never retry
	if err != nil || !brokerpipe.ValidMaintenanceClientResult(selected, request) || owner.Check(ctx) != nil || ctx.Err() != nil {
		return refused
	}
	readback, err := newRequest(brokerpipe.SelectedHistory)
	if err != nil {
		return err
	}
	page := 0
	readback.Page = &page
	observed, err := op.call(ctx, readback)
	if err != nil || !brokerpipe.ValidSelectedHistoryResult(observed, readback) || observed.SelectedHeadSHA256 != selected.SelectedHeadSHA256 ||
		observed.Generation != target.Generation || observed.CompletionSHA256 != target.CompletionSHA256 || observed.Version != target.Version || observed.Commit != target.Commit ||
		owner.Check(ctx) != nil || ctx.Err() != nil {
		return refused
	}
	if err := errors.Join(owner.Close(), ctx.Err()); err != nil {
		return err
	}
	return json.NewEncoder(output).Encode(struct {
		Schema             int    `json:"schema"`
		State              string `json:"state"`
		Version            string `json:"version"`
		Commit             string `json:"commit"`
		SelectedHeadSHA256 string `json:"selectedHeadSha256"`
	}{1, "verified-customer-rollback-selected", target.Version, target.Commit, observed.SelectedHeadSHA256})
}
