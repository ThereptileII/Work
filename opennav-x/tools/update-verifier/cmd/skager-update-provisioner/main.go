// This explicit operator tool is never called by ordinary application startup.
// Authenticate its final signed bytes/publisher and public pins independently
// before elevation. It cannot establish trust in its own initial execution.
package main

import (
	"context"
	"errors"
	"flag"
	"fmt"
	"io"
	"os"
	"os/signal"
	"strings"

	"example.com/opennav-update-verifier/brokerenroll"
	"example.com/opennav-update-verifier/brokerregistration"
)

const maxTrustBytes = 576 << 10

func main() {
	ctx, cancel := signal.NotifyContext(context.Background(), os.Interrupt)
	defer cancel()
	maintenance, err := runOperator(ctx, os.Args[1:], func(path string) ([]byte, error) { return readInput(path, brokerregistration.MaxRequestBytes) }, brokerenroll.Run, brokerenroll.RunAndActivateStartup, openMaintenanceNative)
	if err != nil {
		fmt.Fprintln(os.Stderr, "SKAGER operator operation did not complete. Preserve partial state for operator recovery; do not reset or retry automatically.")
		os.Exit(1)
	}
	if maintenance {
		if reportMaintenanceCompletion(os.Args[1:], os.Stdout) != nil {
			os.Exit(1)
		}
		return
	}
	fmt.Println("SKAGER provisioning completed. Startup activation occurs only when explicitly requested.")
}

func reportMaintenanceCompletion(args []string, output io.Writer) error {
	if manualRollbackRequested(args) || selectedHistoryRequested(args) || decommissionRequested(args) || currentInstalledRequested(args) || len(args) > 0 && args[0] == "--maintenance=uninstall-current-elevated" {
		return nil // The operation already emitted its one bound JSON result.
	}
	_, err := fmt.Fprintln(output, "SKAGER broker stop and original process exit verified. Application/profile recovery, repair and uninstall have not been performed.")
	return err
}

func run(ctx context.Context, args []string, read func(string) ([]byte, error), enroll func(context.Context, brokerenroll.Request) error) error {
	return runWithActivation(ctx, args, read, enroll, nil)
}

func runWithActivation(ctx context.Context, args []string, read func(string) ([]byte, error), enroll, activate func(context.Context, brokerenroll.Request) error) error {
	if ctx == nil || ctx.Err() != nil || read == nil || enroll == nil {
		return errors.New("live operator invocation required")
	}
	flags := flag.NewFlagSet("skager-update-provisioner", flag.ContinueOnError)
	flags.SetOutput(io.Discard)
	trustPath := flags.String("trust-config", "", "independently approved public trust file")
	cache := flags.String("bootstrap-cache", "", "existing protected durable operator TUF cache")
	artifacts := flags.String("artifact-directory", "", "existing protected operator acquisition directory")
	policy := flags.String("policy-sha256", "", "independently approved exact bootstrap policy SHA-256")
	provisioner := flags.String("provisioner-sha256", "", "independently authenticated final provisioner SHA-256")
	trust := flags.String("trust-config-sha256", "", "independently approved exact public trust SHA-256")
	clientPath := flags.String("client-registration-request", "", "optional exact separately approved public client request; requires approved-installed-baseline.zip in artifact-directory")
	clientSHA := flags.String("client-registration-request-sha256", "", "separately authenticated exact client request SHA-256")
	startup := flags.String("activate-startup", "", "explicit true: publish approved client startup and activate the newly enrolled broker")
	// flag accepts duplicate options by default. Refuse ambiguous operator pins,
	// aliases and positional/unknown arguments before opening any input file.
	seen := map[string]bool{}
	for i := 0; i < len(args); i++ {
		arg := args[i]
		if !strings.HasPrefix(arg, "--") {
			return errors.New("explicit named operator inputs required")
		}
		name, _, hasValue := strings.Cut(strings.TrimPrefix(arg, "--"), "=")
		if flags.Lookup(name) == nil || seen[name] {
			return errors.New("unknown or duplicate operator input")
		}
		seen[name] = true
		if !hasValue {
			i++
			if i >= len(args) || strings.HasPrefix(args[i], "--") {
				return errors.New("operator input value required")
			}
		}
	}
	expected := 6
	if seen["client-registration-request"] || seen["client-registration-request-sha256"] {
		expected += 2
	}
	if seen["activate-startup"] {
		expected++
	}
	if flags.Parse(args) != nil || flags.NArg() != 0 || len(seen) != expected ||
		*trustPath == "" || *cache == "" || *artifacts == "" || !pin(*policy) || !pin(*provisioner) || !pin(*trust) {
		return errors.New("complete independently approved operator inputs required")
	}
	hasClient := seen["client-registration-request"] || seen["client-registration-request-sha256"]
	if hasClient && (!seen["client-registration-request"] || !seen["client-registration-request-sha256"] || *clientPath == "" || !pin(*clientSHA)) {
		return errors.New("client request requires paired independent exact approval")
	}
	if seen["activate-startup"] && (*startup != "true" || !hasClient || activate == nil) {
		return errors.New("startup activation requires explicit true and independently approved client registration")
	}
	data, err := read(*trustPath)
	if err != nil || len(data) == 0 || len(data) > maxTrustBytes {
		return errors.New("bounded public trust configuration required")
	}
	var client []byte
	if hasClient {
		client, err = read(*clientPath)
		if err != nil || len(client) < 1 || len(client) > brokerregistration.MaxRequestBytes {
			return errors.New("bounded separately approved public client request required")
		}
	}
	selected := enroll
	if seen["activate-startup"] {
		selected = activate
	}
	return selected(ctx, brokerenroll.Request{TrustConfiguration: data, BootstrapCacheDir: *cache,
		ArtifactDirectory: *artifacts, PolicySHA256: *policy, ProvisionerSHA256: *provisioner, TrustConfigSHA256: *trust,
		ClientRegistrationRequest: client, ApprovedClientRequestSHA256: *clientSHA})
}

func pin(value string) bool {
	if len(value) != 64 {
		return false
	}
	for _, c := range value {
		if !('0' <= c && c <= '9' || 'a' <= c && c <= 'f') {
			return false
		}
	}
	return true
}

func readTrust(path string) (_ []byte, resultErr error) {
	return readInput(path, maxTrustBytes)
}

func readInput(path string, bound int64) (_ []byte, resultErr error) {
	before, err := os.Lstat(path)
	if err != nil || !before.Mode().IsRegular() || before.Size() < 1 || before.Size() > bound {
		return nil, errors.New("bounded regular public trust file required")
	}
	f, err := os.Open(path)
	if err != nil {
		return nil, errors.New("public trust input unavailable")
	}
	defer func() {
		if f.Close() != nil {
			resultErr = errors.New("public trust input close uncertain")
		}
	}()
	held, err := f.Stat()
	if err != nil || !os.SameFile(before, held) {
		return nil, errors.New("public trust input changed")
	}
	data, err := io.ReadAll(io.LimitReader(f, bound+1))
	after, statErr := f.Stat()
	if err != nil || statErr != nil || len(data) < 1 || int64(len(data)) > bound ||
		!os.SameFile(held, after) || held.Size() != after.Size() || !held.ModTime().Equal(after.ModTime()) {
		return nil, errors.New("public trust input changed or exceeded bound")
	}
	// Run hashes this owned copy against an independent pin before any network or
	// machine mutation. The input pathname itself never grants authority.
	return data, nil
}
