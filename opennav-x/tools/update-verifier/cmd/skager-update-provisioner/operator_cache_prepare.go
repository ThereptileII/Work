package main

import (
	"context"
	"errors"
	"flag"
	"io"
	"strings"

	"example.com/opennav-update-verifier/brokerenroll"
)

func freshBootstrapCacheRequested(args []string) bool {
	for _, arg := range args {
		name, _, _ := strings.Cut(arg, "=")
		if name == "--prepare-bootstrap-cache" {
			return true
		}
	}
	return false
}

// Explicit operator-only preparation. No installed-customer path calls this,
// and no recovery/reset/registration/service input is accepted.
func runFreshBootstrapCache(ctx context.Context, args []string, read func(string) ([]byte, error), prepare func(context.Context, brokerenroll.FreshBootstrapCacheRequest) error) error {
	if ctx == nil || ctx.Err() != nil || read == nil || prepare == nil {
		return errors.New("live explicit original cache preparation required")
	}
	flags := flag.NewFlagSet("skager-update-provisioner prepare-bootstrap-cache", flag.ContinueOnError)
	flags.SetOutput(io.Discard)
	action := flags.String("prepare-bootstrap-cache", "", "explicit true: create one absent protected original bootstrap cache")
	trustPath := flags.String("trust-config", "", "exact independently approved original public trust")
	trust := flags.String("trust-config-sha256", "", "independent original public trust approval")
	provisioner := flags.String("provisioner-sha256", "", "independent authenticated final running provisioner approval")
	cache := flags.String("bootstrap-cache", "", "independently selected absent cache under an already protected parent")
	seen := map[string]bool{}
	for i := 0; i < len(args); i++ {
		if !strings.HasPrefix(args[i], "--") {
			return errors.New("explicit named original cache inputs required")
		}
		name, _, inline := strings.Cut(strings.TrimPrefix(args[i], "--"), "=")
		if flags.Lookup(name) == nil || seen[name] {
			return errors.New("unknown or duplicate original cache preparation input")
		}
		seen[name] = true
		if !inline {
			i++
			if i >= len(args) || strings.HasPrefix(args[i], "--") {
				return errors.New("original cache input value required")
			}
		}
	}
	if flags.Parse(args) != nil || flags.NArg() != 0 || len(seen) != 5 || *action != "true" || *trustPath == "" || *cache == "" || !pin(*trust) || !pin(*provisioner) {
		return errors.New("complete independently approved original cache preparation required")
	}
	raw, err := read(*trustPath)
	if err != nil || len(raw) == 0 || len(raw) > maxTrustBytes {
		return errors.New("bounded original public trust required")
	}
	return prepare(ctx, brokerenroll.FreshBootstrapCacheRequest{TrustConfiguration: raw, TrustConfigSHA256: *trust, ProvisionerSHA256: *provisioner, BootstrapCacheDir: *cache})
}
