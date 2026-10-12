package main

import (
	"context"
	"encoding/json"
	"errors"
	"flag"
	"io"
	"strconv"
	"strings"

	"example.com/opennav-update-verifier/brokerpipe"
)

func selectedHistoryRequested(args []string) bool {
	for i, arg := range args {
		name, value, inline := strings.Cut(arg, "=")
		if name == "--maintenance" && ((inline && value == brokerpipe.SelectedHistory) || (!inline && i+1 < len(args) && args[i+1] == brokerpipe.SelectedHistory)) {
			return true
		}
	}
	return false
}

// runSelectedHistory prints one authenticated read-only page. A later page is a
// separate fresh request pinned to the original selected head and inventory.
func runSelectedHistory(ctx context.Context, args []string, call maintenanceCaller, output io.Writer) error {
	if ctx == nil || ctx.Err() != nil || call == nil || output == nil {
		return errors.New("live explicit selected-history diagnostic required")
	}
	flags := flag.NewFlagSet("skager-update-provisioner selected history", flag.ContinueOnError)
	flags.SetOutput(io.Discard)
	action := flags.String("maintenance", "", "read-only selected-history diagnostic")
	requestID := flags.String("request-id", "", "fresh diagnostic request ID")
	pageText := flags.String("page", "", "bounded selected-history page")
	head := flags.String("expected-selected-head-sha256", "", "original first-page selected head")
	inventory := flags.String("inventory-sha256", "", "original first-page journal inventory")
	seen := map[string]bool{}
	for i := 0; i < len(args); i++ {
		if !strings.HasPrefix(args[i], "--") {
			return errors.New("explicit named selected-history inputs required")
		}
		name, _, inline := strings.Cut(strings.TrimPrefix(args[i], "--"), "=")
		if flags.Lookup(name) == nil || seen[name] {
			return errors.New("unknown or duplicate selected-history input")
		}
		seen[name] = true
		if !inline {
			i++
			if i >= len(args) || strings.HasPrefix(args[i], "--") {
				return errors.New("selected-history input value required")
			}
		}
	}
	if flags.Parse(args) != nil || flags.NArg() != 0 || *action != brokerpipe.SelectedHistory || !manualRollbackID(*requestID) || !seen["page"] {
		return errors.New("complete exact selected-history inputs required")
	}
	page, err := strconv.Atoi(*pageText)
	if err != nil || page < 0 || page > 7 || strconv.Itoa(page) != *pageText {
		return errors.New("bounded canonical selected-history page required")
	}
	if page == 0 && (len(seen) != 3 || *head != "" || *inventory != "") || page > 0 && (len(seen) != 5 || !pin(*head) || !pin(*inventory)) {
		return errors.New("selected-history continuation pins required")
	}
	q := brokerpipe.MaintenanceRequest{Schema: 1, Operation: brokerpipe.SelectedHistory, RequestID: *requestID, Page: &page,
		ExpectedSelectedHeadSHA256: *head, InventorySHA256: *inventory}
	if ctx.Err() != nil {
		return errors.New("selected-history interrupted before request")
	}
	result, err := call(ctx, q)
	if err != nil || ctx.Err() != nil || !brokerpipe.ValidSelectedHistoryResult(result, q) {
		return errors.New("selected-history observation unverified; request was not retried")
	}
	if json.NewEncoder(output).Encode(result) != nil {
		return errors.New("selected-history result output failed; request was not retried")
	}
	return nil
}
