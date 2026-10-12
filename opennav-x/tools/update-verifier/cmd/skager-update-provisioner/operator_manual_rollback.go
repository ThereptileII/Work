package main

import (
	"context"
	"encoding/json"
	"errors"
	"flag"
	"io"
	"os"
	"strings"

	"example.com/opennav-update-verifier/brokerpipe"
)

type maintenanceCaller func(context.Context, brokerpipe.MaintenanceRequest) (brokerpipe.MaintenanceResult, error)

// manualRollbackRequested selects only the explicit operator action. Parsing
// below still rejects conflicting, duplicate, unknown and incomplete inputs.
func manualRollbackRequested(args []string) bool {
	for i, arg := range args {
		name, value, inline := strings.Cut(arg, "=")
		if name == "--maintenance" && ((inline && value == brokerpipe.ManualRollback) || (!inline && i+1 < len(args) && args[i+1] == brokerpipe.ManualRollback)) {
			return true
		}
	}
	return false
}

func runMaintenanceDispatch(ctx context.Context, args []string, read func(string) ([]byte, error), maintain maintenanceOpener) error {
	if manualRollbackRequested(args) {
		return runManualRollback(ctx, args, brokerpipe.CallMaintenance, os.Stdout)
	}
	return runMaintenance(ctx, args, read, maintain)
}

// runManualRollback sends one data-only request to the fixed original broker.
// The native endpoint authenticates the actual elevated registered peer; no
// supplied SID, machine path, enrollment or local mutation grants authority.
func runManualRollback(ctx context.Context, args []string, call maintenanceCaller, output io.Writer) error {
	if ctx == nil || ctx.Err() != nil || call == nil || output == nil {
		return errors.New("live explicit manual rollback required")
	}
	flags := flag.NewFlagSet("skager-update-provisioner manual rollback", flag.ContinueOnError)
	flags.SetOutput(io.Discard)
	action := flags.String("maintenance", "", "explicit manual-rollback operation")
	requestID := flags.String("request-id", "", "fresh independently chosen request ID")
	head := flags.String("expected-selected-head-sha256", "", "exact current protected selected head")
	generation := flags.String("target-generation", "", "retained qualified ancestor generation")
	completion := flags.String("target-completion-sha256", "", "exact ancestor completion digest")
	seen := map[string]bool{}
	for i := 0; i < len(args); i++ {
		if !strings.HasPrefix(args[i], "--") {
			return errors.New("explicit named manual rollback inputs required")
		}
		name, _, inline := strings.Cut(strings.TrimPrefix(args[i], "--"), "=")
		if flags.Lookup(name) == nil || seen[name] {
			return errors.New("unknown or duplicate manual rollback input")
		}
		seen[name] = true
		if !inline {
			i++
			if i >= len(args) || strings.HasPrefix(args[i], "--") {
				return errors.New("manual rollback input value required")
			}
		}
	}
	if flags.Parse(args) != nil || flags.NArg() != 0 || len(seen) != 5 || *action != brokerpipe.ManualRollback || !manualRollbackID(*requestID) || !pin(*head) || !manualRollbackID(*generation) || !pin(*completion) {
		return errors.New("complete exact manual rollback pins required")
	}
	q := brokerpipe.MaintenanceRequest{Schema: 1, Operation: brokerpipe.ManualRollback, RequestID: *requestID, ExpectedSelectedHeadSHA256: *head, TargetGeneration: *generation, TargetCompletionSHA256: *completion}
	if ctx.Err() != nil {
		return errors.New("manual rollback interrupted before request")
	}
	result, err := call(ctx, q)
	// Transmission may already have committed. Never retry or emit any supplied
	// result on error, cancellation or mismatched readback, including private text.
	if err != nil || ctx.Err() != nil || !manualRollbackResultMatches(result, q) {
		return errors.New("manual rollback completion unverified; request was not retried")
	}
	if json.NewEncoder(output).Encode(result) != nil {
		return errors.New("manual rollback result output failed; request was not retried")
	}
	return nil
}

func manualRollbackID(value string) bool {
	if len(value) != 32 {
		return false
	}
	for _, c := range value {
		if !(c >= '0' && c <= '9' || c >= 'a' && c <= 'f') {
			return false
		}
	}
	return true
}

func manualRollbackResultMatches(result brokerpipe.MaintenanceResult, q brokerpipe.MaintenanceRequest) bool {
	return result.Schema == 1 && result.RequestID == q.RequestID && result.State == "manual-selected" && result.Generation == q.TargetGeneration && result.CompletionSHA256 == q.TargetCompletionSHA256 && pin(result.SelectedHeadSHA256) && result.SelectedHeadSHA256 != q.ExpectedSelectedHeadSHA256
}
