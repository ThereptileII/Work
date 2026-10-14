package main

import (
	"bytes"
	"context"
	"errors"
	"strings"
	"testing"

	"example.com/opennav-update-verifier/brokerenroll"
)

func freshCacheArgs() []string {
	return []string{"--prepare-bootstrap-cache=true", "--trust-config=original.json", "--trust-config-sha256=" + strings.Repeat("a", 64), "--provisioner-sha256=" + strings.Repeat("b", 64), "--bootstrap-cache=C:\\Protected\\AbsentCache"}
}
func TestFreshBootstrapCacheOperatorExactInputsAndNoParallelAction(t *testing.T) {
	for _, kind := range []string{"valid", "duplicate", "maintenance", "upgrade", "reset", "existing-option", "missing", "false", "short-pin"} {
		t.Run(kind, func(t *testing.T) {
			args := freshCacheArgs()
			switch kind {
			case "duplicate":
				args = append(args, args[1])
			case "maintenance":
				args = append(args, "--maintenance=repair-current")
			case "upgrade":
				args = append(args, "--code-upgrade=true")
			case "reset":
				args = append(args, "--reset=true")
			case "existing-option":
				args = append(args, "--artifact-directory=other")
			case "missing":
				args = args[:4]
			case "false":
				args[0] = "--prepare-bootstrap-cache=false"
			case "short-pin":
				args[2] = "--trust-config-sha256=a"
			}
			reads, calls := 0, 0
			err := runFreshBootstrapCache(context.Background(), args, func(path string) ([]byte, error) {
				reads++
				if path != "original.json" {
					t.Fatal("trust source substituted")
				}
				return []byte("original public bytes"), nil
			}, func(ctx context.Context, r brokerenroll.FreshBootstrapCacheRequest) error {
				calls++
				if r.BootstrapCacheDir != `C:\Protected\AbsentCache` || r.TrustConfigSHA256 != strings.Repeat("a", 64) || r.ProvisionerSHA256 != strings.Repeat("b", 64) || string(r.TrustConfiguration) != "original public bytes" {
					t.Fatal("original operator approvals changed")
				}
				return nil
			})
			if (err == nil) != (kind == "valid") || reads != freshCacheTestInt(kind == "valid") || calls != freshCacheTestInt(kind == "valid") {
				t.Fatal("ambiguous inputs opened trust or reached native effect", kind, err)
			}
		})
	}
}
func freshCacheTestInt(v bool) int {
	if v {
		return 1
	}
	return 0
}
func TestFreshBootstrapCacheReportAndNativeRefusalRemainSeparateFromEnrollment(t *testing.T) {
	var out bytes.Buffer
	if err := reportMaintenanceCompletion(freshCacheArgs(), &out); err != nil || !strings.Contains(out.String(), "Enrollment and application installation have not run.") {
		t.Fatal("cache preparation claimed enrollment", err)
	}
	called := false
	err := runFreshBootstrapCache(context.Background(), freshCacheArgs(), func(string) ([]byte, error) { return []byte("original"), nil }, func(context.Context, brokerenroll.FreshBootstrapCacheRequest) error {
		called = true
		return errors.New("existing/damaged native cache refused")
	})
	if err == nil || !called {
		t.Fatal("native refusal became success")
	}
}
