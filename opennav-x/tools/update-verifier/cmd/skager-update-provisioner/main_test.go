package main

import (
	"context"
	"errors"
	"os"
	"path/filepath"
	"strings"
	"testing"

	"example.com/opennav-update-verifier/brokerenroll"
	"example.com/opennav-update-verifier/brokerregistration"
)

func operatorArgs() []string {
	return []string{"--trust-config", "public-trust.json", "--bootstrap-cache", "protected-cache", "--artifact-directory", "protected-artifacts", "--policy-sha256", strings.Repeat("a", 64), "--provisioner-sha256", strings.Repeat("b", 64), "--trust-config-sha256", strings.Repeat("c", 64)}
}

func TestOptionalClientRequestPreservesSeparateApproval(t *testing.T) {
	args := append(operatorArgs(), "--client-registration-request", "approved-client.json", "--client-registration-request-sha256", strings.Repeat("d", 64))
	reads := []string{}
	enrolls := 0
	err := run(context.Background(), args, func(path string) ([]byte, error) { reads = append(reads, path); return []byte(path), nil }, func(_ context.Context, r brokerenroll.Request) error {
		enrolls++
		if string(r.ClientRegistrationRequest) != "approved-client.json" || r.ApprovedClientRequestSHA256 != strings.Repeat("d", 64) {
			t.Fatal("separate approval or exact request input lost")
		}
		return nil
	})
	if err != nil || enrolls != 1 || len(reads) != 2 || reads[0] != "public-trust.json" || reads[1] != "approved-client.json" {
		t.Fatalf("err=%v reads=%v enrolls=%d", err, reads, enrolls)
	}
}

func TestIncompleteClientOptionsNeverReadOrEnroll(t *testing.T) {
	for _, extra := range [][]string{
		{"--client-registration-request", "client.json"},
		{"--client-registration-request-sha256", strings.Repeat("d", 64)},
		{"--client-registration-request=", "--client-registration-request-sha256", strings.Repeat("d", 64)},
		{"--client-registration-request", "client.json", "--client-registration-request-sha256", strings.Repeat("D", 64)},
		{"--client-registration-request", "client.json", "--client-registration-request-sha256", strings.Repeat("d", 64), "--client-registration-request", "other.json"},
	} {
		calls := 0
		err := run(context.Background(), append(operatorArgs(), extra...), func(string) ([]byte, error) { calls++; return nil, nil }, func(context.Context, brokerenroll.Request) error { calls++; return nil })
		if err == nil || calls != 0 {
			t.Fatalf("invalid options had effects: %v %d", err, calls)
		}
	}
}

func TestClientRequestReadFailureNeverEnrolls(t *testing.T) {
	for _, fault := range []string{"error", "empty", "oversize"} {
		t.Run(fault, func(t *testing.T) {
			enrolls := 0
			err := run(context.Background(), append(operatorArgs(), "--client-registration-request", "client.json", "--client-registration-request-sha256", strings.Repeat("d", 64)), func(path string) ([]byte, error) {
				if path == "public-trust.json" {
					return []byte("trust"), nil
				}
				switch fault {
				case "error":
					return nil, errors.New("private input")
				case "oversize":
					return make([]byte, brokerregistration.MaxRequestBytes+1), nil
				default:
					return nil, nil
				}
			}, func(context.Context, brokerenroll.Request) error { enrolls++; return nil })
			if err == nil || enrolls != 0 || strings.Contains(err.Error(), "private input") {
				t.Fatalf("err=%v enrolls=%d", err, enrolls)
			}
		})
	}
}

func TestClientInputReaderHasSeparateRequestBound(t *testing.T) {
	path := filepath.Join(t.TempDir(), "client.json")
	if err := os.WriteFile(path, make([]byte, maxTrustBytes+1), 0600); err != nil {
		t.Fatal(err)
	}
	if b, err := readInput(path, brokerregistration.MaxRequestBytes); err != nil || len(b) != maxTrustBytes+1 {
		t.Fatalf("request bound: %d %v", len(b), err)
	}
	if _, err := readTrust(path); err == nil {
		t.Fatal("trust bound broadened")
	}
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

func TestStartupRestorationInputsAreRejectedForEveryOtherOperatorAction(t *testing.T) {
	extra := []string{"--original-startup-publication=original-private-publication", "--original-startup-publication-sha256=" + strings.Repeat("a", 64)}
	for _, action := range []string{"", "quiesce-maintenance", "selected-history", "manual-rollback", "uninstall"} {
		t.Run(action, func(t *testing.T) {
			args := operatorArgs()
			if action != "" {
				args = []string{"--maintenance=" + action}
			}
			args = append(args, extra...)
			calls := 0
			_, err := runOperator(context.Background(), args, func(string) ([]byte, error) { calls++; return nil, nil },
				func(context.Context, brokerenroll.Request) error { calls++; return nil },
				func(context.Context, brokerenroll.Request) error { calls++; return nil },
				func(context.Context, brokerenroll.MaintenanceRequest) (maintenanceOwner, error) {
					calls++
					return nil, nil
				})
			if err == nil || calls != 0 {
				t.Fatalf("restoration inputs reached a different action: calls=%d err=%v", calls, err)
			}
		})
	}
}
