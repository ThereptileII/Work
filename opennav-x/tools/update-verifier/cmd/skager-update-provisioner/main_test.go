package main

import (
	"context"
	"errors"
	"os"
	"path/filepath"
	"strings"
	"testing"

	"example.com/opennav-update-verifier/brokerenroll"
)

func operatorArgs() []string {
	return []string{"--trust-config", "public-trust.json", "--bootstrap-cache", "protected-cache", "--artifact-directory", "protected-artifacts", "--policy-sha256", strings.Repeat("a", 64), "--provisioner-sha256", strings.Repeat("b", 64), "--trust-config-sha256", strings.Repeat("c", 64)}
}

func TestExplicitOperatorInputs(t *testing.T) {
	reads, enrolls := 0, 0
	err := run(context.Background(), operatorArgs(), func(path string) ([]byte, error) {
		reads++
		if path != "public-trust.json" {
			t.Fatal("trust selection changed")
		}
		return []byte("inert hash checked by actual Run"), nil
	}, func(ctx context.Context, r brokerenroll.Request) error {
		enrolls++
		if r.PolicySHA256 != strings.Repeat("a", 64) || r.ProvisionerSHA256 != strings.Repeat("b", 64) || r.TrustConfigSHA256 != strings.Repeat("c", 64) || r.BootstrapCacheDir != "protected-cache" || r.ArtifactDirectory != "protected-artifacts" {
			t.Fatal("independent pins lost")
		}
		return nil
	})
	if err != nil || reads != 1 || enrolls != 1 {
		t.Fatalf("err=%v reads=%d enrolls=%d", err, reads, enrolls)
	}
}

func TestAmbiguousOperatorInputsNeverReadOrEnroll(t *testing.T) {
	for _, args := range [][]string{nil, append(operatorArgs(), "--policy-sha256", strings.Repeat("d", 64)), append(operatorArgs(), "--unknown", "value"), append(operatorArgs(), "extra"), {"--trust-config"}, {"-trust-config", "x"}, append(operatorArgs()[:10], "--trust-config-sha256", strings.Repeat("A", 64))} {
		calls := 0
		err := run(context.Background(), args, func(string) ([]byte, error) { calls++; return nil, nil }, func(context.Context, brokerenroll.Request) error { calls++; return nil })
		if err == nil || calls != 0 {
			t.Fatalf("bad inputs effects=%d err=%v", calls, err)
		}
	}
}

func TestPublicTrustReadRefusalNeverEnrolls(t *testing.T) {
	for _, read := range []func(string) ([]byte, error){func(string) ([]byte, error) { return nil, errors.New("private path") }, func(string) ([]byte, error) { return []byte{}, nil }, func(string) ([]byte, error) { return make([]byte, maxTrustBytes+1), nil }} {
		calls := 0
		err := run(context.Background(), operatorArgs(), read, func(context.Context, brokerenroll.Request) error { calls++; return nil })
		if err == nil || calls != 0 || strings.Contains(err.Error(), "private") {
			t.Fatalf("calls=%d err=%v", calls, err)
		}
	}
}

func TestBoundedPublicTrustFile(t *testing.T) {
	path := filepath.Join(t.TempDir(), "trust.json")
	if err := os.WriteFile(path, []byte("public bytes"), 0600); err != nil {
		t.Fatal(err)
	}
	if b, err := readTrust(path); err != nil || string(b) != "public bytes" {
		t.Fatalf("read=%q err=%v", b, err)
	}
	if err := os.WriteFile(path, make([]byte, maxTrustBytes+1), 0600); err != nil {
		t.Fatal(err)
	}
	if _, err := readTrust(path); err == nil {
		t.Fatal("oversized file accepted")
	}
	if _, err := readTrust(filepath.Dir(path)); err == nil {
		t.Fatal("directory accepted")
	}
}
