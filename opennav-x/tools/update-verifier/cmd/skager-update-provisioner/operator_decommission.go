package main

import (
	"bytes"
	"context"
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"errors"
	"flag"
	"io"
	"strings"

	"example.com/opennav-update-verifier/brokerenroll"
	"example.com/opennav-update-verifier/brokerpipe"
	"example.com/opennav-update-verifier/brokerregistration"
)

const decommissionService = "decommission-service"
const uninstallProtected = "uninstall-protected"

// Matches the existing purpose-bound publication parser. The shared input
// reader remains bounded by MaxRequestBytes; this narrower record is checked
// again before parsing or obtaining any original SCM/process owner.
const maxStartupPublicationBytes = 32 << 10

type decommissionOwner interface {
	Check(context.Context) error
	BindDecommission(context.Context, *brokerpipe.DecommissionWitness) error
	BindStartupRestoration(context.Context, *brokerenroll.ApprovedStartupPublication) error
	PrepareStartupRestoration(context.Context, *brokerenroll.ApprovedStartupPublication) error
	Decommission(context.Context) (brokerenroll.DecommissionResult, error)
	Close() error
}

type decommissionOpener func(context.Context, brokerenroll.MaintenanceRequest) (decommissionOwner, error)
type decommissionCaller func(context.Context, brokerpipe.MaintenanceRequest) (*brokerpipe.DecommissionWitness, error)
type startupPublicationParser func([]byte, string, []byte, string) (*brokerenroll.ApprovedStartupPublication, error)

type decommissionOperations struct {
	parse        startupPublicationParser
	closeWitness func(*brokerpipe.DecommissionWitness) error
}

func openDecommissionNative(ctx context.Context, request brokerenroll.MaintenanceRequest) (decommissionOwner, error) {
	owner, err := brokerenroll.OpenDecommissionMaintenance(ctx, request)
	if owner == nil {
		return nil, err
	}
	return owner, err
}

func decommissionRequested(args []string) bool {
	for i, arg := range args {
		name, value, inline := strings.Cut(arg, "=")
		if name == "--maintenance" && ((inline && (value == decommissionService || value == uninstallProtected)) || (!inline && i+1 < len(args) && (args[i+1] == decommissionService || args[i+1] == uninstallProtected))) {
			return true
		}
	}
	return false
}

// This explicit elevated operator invocation owns the original service BEFORE
// asking that same broker to publish its terminal fence. Explicitly requested
// original shortcut restoration binds to that SAME fenced owner before its
// Decommission operation. Protected uninstall first completes native retirement
// through the same live broker; no file/profile effect follows service shutdown.
// The native witness is never reconstructed from JSON.
func runDecommission(ctx context.Context, args []string, read func(string) ([]byte, error), open decommissionOpener, call decommissionCaller, output io.Writer) (resultErr error) {
	return runDecommissionWith(ctx, args, read, open, call, output, decommissionOperations{
		parse: brokerenroll.ParseApprovedStartupPublication,
		closeWitness: func(witness *brokerpipe.DecommissionWitness) error {
			return witness.Close()
		},
	})
}

// The parser/close seam is private and exists only for portable ordering/fault
// tests. Production always uses the purpose-bound parser and native witness close.
func runDecommissionWith(ctx context.Context, args []string, read func(string) ([]byte, error), open decommissionOpener, call decommissionCaller, output io.Writer, op decommissionOperations) (resultErr error) {
	refused := errors.New("service decommission unverified; preserve all state; no automatic retry")
	defer func() {
		if resultErr != nil {
			resultErr = refused
		}
	}()
	if ctx == nil || ctx.Err() != nil || read == nil || open == nil || call == nil || output == nil || op.parse == nil || op.closeWitness == nil {
		return refused
	}
	flags := flag.NewFlagSet("skager-update-provisioner service decommission", flag.ContinueOnError)
	flags.SetOutput(io.Discard)
	action := flags.String("maintenance", "", "explicit decommission-service or uninstall-protected; optional original startup restoration requires paired approval")
	broker := flags.String("broker-sha256", "", "independently approved installed broker hash")
	clientPath := flags.String("client-registration-request", "", "original independently approved request file")
	clientSHA := flags.String("client-registration-request-sha256", "", "independently approved original request hash")
	requestID := flags.String("request-id", "", "fresh exact request ID")
	head := flags.String("expected-selected-head-sha256", "", "observed original selected head")
	publicationPath := flags.String("original-startup-publication", "", "optional independently approved ORIGINAL completed startup publication file")
	publicationSHA := flags.String("original-startup-publication-sha256", "", "separately approved original completed publication SHA-256")
	seen := map[string]bool{}
	for i := 0; i < len(args); i++ {
		if !strings.HasPrefix(args[i], "--") {
			return refused
		}
		name, _, inline := strings.Cut(strings.TrimPrefix(args[i], "--"), "=")
		if flags.Lookup(name) == nil || seen[name] {
			return refused
		}
		seen[name] = true
		if !inline {
			i++
			if i >= len(args) || strings.HasPrefix(args[i], "--") {
				return refused
			}
		}
	}
	hasPublication := seen["original-startup-publication"] || seen["original-startup-publication-sha256"]
	expected := 6
	if hasPublication {
		expected += 2
	}
	if flags.Parse(args) != nil || flags.NArg() != 0 || len(seen) != expected || (*action != decommissionService && *action != uninstallProtected) || !pin(*broker) || !pin(*clientSHA) || !pin(*head) || !manualRollbackID(*requestID) || *clientPath == "" {
		return refused
	}
	if *action == uninstallProtected && !hasPublication {
		return refused
	}
	if hasPublication && (!seen["original-startup-publication"] || !seen["original-startup-publication-sha256"] || *publicationPath == "" || !pin(*publicationSHA)) {
		return refused
	}
	raw, err := read(*clientPath)
	if err != nil || len(raw) < 1 || len(raw) > brokerregistration.MaxRequestBytes {
		return refused
	}
	raw = bytes.Clone(raw)
	hash := sha256.Sum256(raw)
	if hex.EncodeToString(hash[:]) != *clientSHA || ctx.Err() != nil {
		return refused
	}
	var approved *brokerenroll.ApprovedStartupPublication
	if hasPublication {
		publication, readErr := read(*publicationPath)
		if readErr != nil || len(publication) < 1 || len(publication) > maxStartupPublicationBytes || ctx.Err() != nil {
			return refused
		}
		// Both original records and both independent pins are validated before
		// opening any SCM owner or asking the broker for its terminal fence.
		approved, err = op.parse(bytes.Clone(publication), *publicationSHA, raw, *clientSHA)
		if err != nil || approved == nil || ctx.Err() != nil {
			return refused
		}
	}
	owner, err := open(ctx, brokerenroll.MaintenanceRequest{ApprovedBrokerSHA256: *broker,
		ClientRegistrationRequest: raw, ApprovedClientRequestSHA256: *clientSHA})
	if owner != nil {
		defer func() { resultErr = errors.Join(resultErr, owner.Close()) }()
	}
	if err != nil || owner == nil {
		return refused
	}
	if err = owner.Check(ctx); err != nil {
		return refused
	}
	if *action == uninstallProtected {
		// Retain the original launcher and all publication objects before the one
		// broker call can move its generation. Binding the authenticated retirement
		// witness below finalizes those SAME move-aware handles; no path reopening.
		if err = owner.PrepareStartupRestoration(ctx, approved); err != nil || ctx.Err() != nil {
			return refused
		}
	}
	operation := brokerpipe.PrepareDecommission
	if *action == uninstallProtected {
		operation = brokerpipe.PrepareUninstall
	}
	witness, err := call(ctx, brokerpipe.MaintenanceRequest{Schema: 1, Operation: operation,
		RequestID: *requestID, ExpectedSelectedHeadSHA256: *head})
	if witness != nil {
		defer func() { resultErr = errors.Join(resultErr, op.closeWitness(witness)) }()
	}
	if err != nil || witness == nil || ctx.Err() != nil {
		return refused
	}
	if err = owner.BindDecommission(ctx, witness); err != nil {
		return refused
	}
	if hasPublication && *action == decommissionService {
		if err = owner.BindStartupRestoration(ctx, approved); err != nil || ctx.Err() != nil {
			return refused
		}
	}
	result, err := owner.Decommission(ctx)
	if err != nil || result.Service != brokerenroll.ServiceMarkedForRemoval {
		return refused
	}
	// Close all original native owners before reporting either bounded action.
	// Neither this result nor SCM deletion grants later cleanup authority.
	if err = errors.Join(owner.Close(), op.closeWitness(witness)); err != nil {
		return refused
	}
	startup := ""
	if hasPublication {
		startup = "original-shortcuts-restored"
	}
	state := "service-decommissioned"
	if *action == uninstallProtected {
		state = "protected-retired-service-decommissioned"
	}
	return json.NewEncoder(output).Encode(struct {
		Schema    int    `json:"schema"`
		RequestID string `json:"requestId"`
		State     string `json:"state"`
		Service   string `json:"service"`
		Startup   string `json:"startup,omitempty"`
	}{1, *requestID, state, result.Service, startup})
}
