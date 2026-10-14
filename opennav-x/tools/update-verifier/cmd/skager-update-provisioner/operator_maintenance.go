package main

import (
	"bytes"
	"context"
	"crypto/sha256"
	"encoding/hex"
	"errors"
	"flag"
	"io"
	"strings"

	"example.com/opennav-update-verifier/brokerenroll"
	"example.com/opennav-update-verifier/brokerregistration"
)

// This operator-only path does not run during application startup. The final
// provisioner and independent pins must be authenticated before elevation.
// Quiescence is a prerequisite, never application/profile mutation authority.
type maintenanceOwner interface {
	Check(context.Context) error
	Quiesce(context.Context) error
	Close() error
}

type maintenanceOpener func(context.Context, brokerenroll.MaintenanceRequest) (maintenanceOwner, error)

func openMaintenanceNative(ctx context.Context, req brokerenroll.MaintenanceRequest) (maintenanceOwner, error) {
	owner, err := brokerenroll.OpenMaintenance(ctx, req)
	if owner == nil {
		return nil, err
	}
	return owner, err
}

func runOperator(ctx context.Context, args []string, read func(string) ([]byte, error), enroll, activate func(context.Context, brokerenroll.Request) error, maintain maintenanceOpener) (bool, error) {
	if freshBootstrapCacheRequested(args) {
		return true, runFreshBootstrapCache(ctx, args, read, brokerenroll.PrepareFreshBootstrapCache)
	}
	if codeUpgradeRequested(args) {
		return true, runCodeUpgrade(ctx, args, read, brokerenroll.UpgradeEnrolledCode)
	}
	for _, arg := range args {
		name, _, _ := strings.Cut(arg, "=")
		if name == "--maintenance" {
			return true, runMaintenanceDispatch(ctx, args, read, maintain)
		}
	}
	return false, runWithActivation(ctx, args, read, enroll, activate)
}

func runMaintenance(ctx context.Context, args []string, read func(string) ([]byte, error), open maintenanceOpener) (resultErr error) {
	if ctx == nil || ctx.Err() != nil || read == nil || open == nil {
		return errors.New("live explicit operator maintenance required")
	}
	flags := flag.NewFlagSet("skager-update-provisioner maintenance", flag.ContinueOnError)
	flags.SetOutput(io.Discard)
	action := flags.String("maintenance", "", "explicit quiesce-maintenance: stop only the verified original broker")
	broker := flags.String("broker-sha256", "", "independently approved installed broker SHA-256")
	clientPath := flags.String("client-registration-request", "", "exact independently approved original client request")
	clientSHA := flags.String("client-registration-request-sha256", "", "independently approved original client request SHA-256")
	seen := map[string]bool{}
	for i := 0; i < len(args); i++ {
		if !strings.HasPrefix(args[i], "--") {
			return errors.New("explicit named maintenance inputs required")
		}
		name, _, inline := strings.Cut(strings.TrimPrefix(args[i], "--"), "=")
		if flags.Lookup(name) == nil || seen[name] {
			return errors.New("unknown or duplicate maintenance input")
		}
		seen[name] = true
		if !inline {
			i++
			if i >= len(args) || strings.HasPrefix(args[i], "--") {
				return errors.New("maintenance input value required")
			}
		}
	}
	if flags.Parse(args) != nil || flags.NArg() != 0 || len(seen) != 4 || *action != "quiesce-maintenance" || !pin(*broker) || !pin(*clientSHA) || *clientPath == "" {
		return errors.New("complete independently approved quiescence inputs required")
	}
	raw, err := read(*clientPath)
	if err != nil || len(raw) < 1 || len(raw) > brokerregistration.MaxRequestBytes {
		return errors.New("bounded original approved client request required")
	}
	raw = bytes.Clone(raw)
	hash := sha256.Sum256(raw)
	if hex.EncodeToString(hash[:]) != *clientSHA || ctx.Err() != nil {
		return errors.New("original client request differs or maintenance interrupted")
	}
	owner, err := open(ctx, brokerenroll.MaintenanceRequest{
		ApprovedBrokerSHA256: *broker, ClientRegistrationRequest: raw,
		ApprovedClientRequestSHA256: *clientSHA,
	})
	if owner != nil {
		defer func() { resultErr = errors.Join(resultErr, owner.Close()) }()
	}
	if err != nil || owner == nil {
		return errors.Join(errors.New("original protected maintenance custody unavailable"), err)
	}
	if err := owner.Check(ctx); err != nil {
		return err
	}
	// The native owner consumes this one stop attempt before any effect. No retry,
	// auto-start, enrollment, application launch or destructive cleanup follows.
	if err := owner.Quiesce(ctx); err != nil {
		return err
	}
	return owner.Check(ctx)
}
