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
)

const maxTrustBytes = 576 << 10

func main() {
	ctx, cancel := signal.NotifyContext(context.Background(), os.Interrupt)
	defer cancel()
	if err := run(ctx, os.Args[1:], readTrust, brokerenroll.Run); err != nil {
		fmt.Fprintln(os.Stderr, "SKAGER machine broker provisioning did not complete. Preserve partial state for operator recovery; do not reset or retry automatically.")
		os.Exit(1)
	}
	fmt.Println("SKAGER machine broker initialized and left stopped. Client registration and installed updates are not enabled by this command.")
}

func run(ctx context.Context, args []string, read func(string) ([]byte, error), enroll func(context.Context, brokerenroll.Request) error) error {
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
	if flags.Parse(args) != nil || flags.NArg() != 0 || len(seen) != 6 ||
		*trustPath == "" || *cache == "" || *artifacts == "" || !pin(*policy) || !pin(*provisioner) || !pin(*trust) {
		return errors.New("complete independently approved operator inputs required")
	}
	data, err := read(*trustPath)
	if err != nil || len(data) == 0 || len(data) > maxTrustBytes {
		return errors.New("bounded public trust configuration required")
	}
	return enroll(ctx, brokerenroll.Request{TrustConfiguration: data, BootstrapCacheDir: *cache,
		ArtifactDirectory: *artifacts, PolicySHA256: *policy, ProvisionerSHA256: *provisioner, TrustConfigSHA256: *trust})
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
	before, err := os.Lstat(path)
	if err != nil || !before.Mode().IsRegular() || before.Size() < 1 || before.Size() > maxTrustBytes {
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
	data, err := io.ReadAll(io.LimitReader(f, maxTrustBytes+1))
	after, statErr := f.Stat()
	if err != nil || statErr != nil || len(data) < 1 || len(data) > maxTrustBytes ||
		!os.SameFile(held, after) || held.Size() != after.Size() || !held.ModTime().Equal(after.ModTime()) {
		return nil, errors.New("public trust input changed or exceeded bound")
	}
	// Run hashes this owned copy against an independent pin before any network or
	// machine mutation. The input pathname itself never grants authority.
	return data, nil
}
