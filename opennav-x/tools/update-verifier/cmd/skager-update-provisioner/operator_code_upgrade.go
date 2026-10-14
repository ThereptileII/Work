package main

import (
	"context"
	"errors"
	"example.com/opennav-update-verifier/brokerenroll"
	"flag"
	"io"
	"strings"
)

func codeUpgradeRequested(args []string) bool {
	for _, arg := range args {
		n, _, _ := strings.Cut(arg, "=")
		if n == "--code-upgrade" || n == "--code-upgrade-resume" {
			return true
		}
	}
	return false
}

// This route is intentionally separate from fresh enrollment and STOP-only
// maintenance. It accepts no machine path, SID, replacement source path, trust
// floor or store reset option. Initial new-provisioner authentication is external.
func runCodeUpgrade(ctx context.Context, args []string, read func(string) ([]byte, error), upgrade func(context.Context, brokerenroll.CodeUpgradeRequest) error) error {
	if ctx == nil || ctx.Err() != nil || read == nil || upgrade == nil {
		return errors.New("live explicit code upgrade required")
	}
	flags := flag.NewFlagSet("skager-update-provisioner code-upgrade", flag.ContinueOnError)
	flags.SetOutput(io.Discard)
	resume := flags.String("code-upgrade-resume", "", "explicit true: resume only the same fully staged/drained approved transaction")
	action := flags.String("code-upgrade", "", "explicit true: upgrade one existing enrolled code pair")
	trustPath := flags.String("trust-config", "", "exact unchanged independently approved trust file")
	cache := flags.String("bootstrap-cache", "", "existing protected durable TUF cache")
	artifacts := flags.String("artifact-directory", "", "existing protected acquisition directory")
	policy := flags.String("upgrade-policy-sha256", "", "independently approved purpose-bound signed target SHA-256")
	provisioner := flags.String("provisioner-sha256", "", "independently authenticated running new provisioner SHA-256")
	trust := flags.String("trust-config-sha256", "", "exact original trust SHA-256")
	original := flags.String("original-completion-sha256", "", "independently approved original installed completion SHA-256")
	seen := map[string]bool{}
	for i := 0; i < len(args); i++ {
		if !strings.HasPrefix(args[i], "--") {
			return errors.New("explicit named upgrade inputs required")
		}
		name, _, inline := strings.Cut(strings.TrimPrefix(args[i], "--"), "=")
		if flags.Lookup(name) == nil || seen[name] {
			return errors.New("unknown or duplicate code upgrade input")
		}
		seen[name] = true
		if !inline {
			i++
			if i >= len(args) || strings.HasPrefix(args[i], "--") {
				return errors.New("code upgrade input value required")
			}
		}
	}
	if flags.Parse(args) != nil || flags.NArg() != 0 || len(seen) != 8 || ((*action != "true" || *resume != "") && (*resume != "true" || *action != "")) || *trustPath == "" || *cache == "" || *artifacts == "" || !pin(*policy) || !pin(*provisioner) || !pin(*trust) || !pin(*original) {
		return errors.New("complete independent code upgrade approval required")
	}
	raw, err := read(*trustPath)
	if err != nil || len(raw) < 1 || len(raw) > maxTrustBytes {
		return errors.New("bounded unchanged approved trust required")
	}
	return upgrade(ctx, brokerenroll.CodeUpgradeRequest{TrustConfiguration: raw, BootstrapCacheDir: *cache, ArtifactDirectory: *artifacts, PolicySHA256: *policy, ProvisionerSHA256: *provisioner, TrustConfigSHA256: *trust, OriginalCompletionSHA256: *original, Resume: *resume == "true"})
}
