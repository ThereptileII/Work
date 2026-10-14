package main

import (
	"context"
	"example.com/opennav-update-verifier/brokerenroll"
	"strings"
	"testing"
)

func codeUpgradeArgs() []string {
	return []string{"--code-upgrade=true", "--trust-config=public.json", "--bootstrap-cache=cache", "--artifact-directory=artifacts", "--upgrade-policy-sha256=" + strings.Repeat("a", 64), "--provisioner-sha256=" + strings.Repeat("b", 64), "--trust-config-sha256=" + strings.Repeat("c", 64), "--original-completion-sha256=" + strings.Repeat("d", 64)}
}
func TestCodeUpgradeOperatorExplicitUpgradeAndResume(t *testing.T) {
	for _, resume := range []bool{false, true} {
		args := codeUpgradeArgs()
		if resume {
			args[0] = "--code-upgrade-resume=true"
		}
		calls := 0
		e := runCodeUpgrade(context.Background(), args, func(string) ([]byte, error) { return []byte("inert trust"), nil }, func(_ context.Context, r brokerenroll.CodeUpgradeRequest) error {
			calls++
			if r.Resume != resume || r.OriginalCompletionSHA256 != strings.Repeat("d", 64) || r.PolicySHA256 != strings.Repeat("a", 64) {
				t.Fatal("code approval binding lost")
			}
			return nil
		})
		if e != nil || calls != 1 {
			t.Fatal(e, calls)
		}
	}
}
func TestCodeUpgradeOperatorAmbiguityRefusesBeforeInputs(t *testing.T) {
	for _, args := range [][]string{append(codeUpgradeArgs(), "--code-upgrade-resume=true"), append(codeUpgradeArgs(), "--code-upgrade=true"), append(codeUpgradeArgs(), "--activate-startup=true"), append(codeUpgradeArgs(), "--maintenance=quiesce-maintenance"), append(codeUpgradeArgs(), "--replacement-path=C:/other.exe"), append(codeUpgradeArgs(), "--client-registration-request=forged.json"), codeUpgradeArgs()[:7]} {
		if e := runCodeUpgrade(context.Background(), args, func(string) ([]byte, error) { t.Fatal("ambiguous request read input"); return nil, nil }, func(context.Context, brokerenroll.CodeUpgradeRequest) error {
			t.Fatal("ambiguous upgrade executed")
			return nil
		}); e == nil {
			t.Fatal("ambiguous request accepted")
		}
	}
}
