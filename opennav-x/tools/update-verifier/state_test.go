package verifier

import (
	"bytes"
	"os"
	"path/filepath"
	"strings"
	"testing"
	"time"

	"github.com/theupdateframework/go-tuf/v2/metadata"
)

func initializedState(t *testing.T) (*fixture, string, TrustConfig) {
	t.Helper()
	f := newFixture(t)
	f.publish(t, 1, "beta", policyBytes(t, signedPolicyFixture()), false)
	dir := filepath.Join(t.TempDir(), "protected")
	c := TrustConfig{BootstrapRoot: f.root, MetadataURL: f.server.URL, Channel: "beta"}
	if err := InitializeState(dir, c); err != nil {
		t.Fatal(err)
	}
	return f, dir, c
}

func TestStateRefreshAndDurableRollbackFloor(t *testing.T) {
	f, dir, c := initializedState(t)
	refresh := func() error { _, err := verifyReleaseInState(dir, c, f.server.Client()); return err }
	if err := refresh(); err != nil {
		t.Fatal(err)
	}
	if err := refresh(); err != nil {
		t.Fatal(err)
	}
	f.publish(t, 2, "beta", policyBytes(t, signedPolicyFixture()), false)
	delete(f.files, "/snapshot.json")
	if err := refresh(); err == nil {
		t.Fatal("missing new snapshot accepted")
	}
	want, _ := trustIdentity(c)
	s, err := readState(dir, want)
	if err != nil {
		t.Fatal(err)
	}
	files, err := readStateMetadata(filepath.Join(dir, s.Generation))
	if err != nil {
		t.Fatal(err)
	}
	timestamp, err := metadata.Timestamp().FromBytes(files["timestamp.json"])
	if err != nil || timestamp.Signed.Version != 2 {
		t.Fatalf("newer signed timestamp forgotten after failed refresh: %v", err)
	}
	f.publish(t, 1, "beta", policyBytes(t, signedPolicyFixture()), false)
	if err := refresh(); err == nil {
		t.Fatal("older signed timestamp replay accepted after failed request")
	}
	f.publish(t, 2, "beta", policyBytes(t, signedPolicyFixture()), false)
	if err := refresh(); err != nil {
		t.Fatal(err)
	}
}

func TestStateNeverSilentlyInitializesOrAdoptsOrphans(t *testing.T) {
	for _, scenario := range []string{"missing-state", "missing-root", "changed-root", "extra-cache-file", "unknown-state-field", "duplicate-state-field", "pending-refresh", "wrong-origin", "wrong-channel", "wrong-bootstrap", "missing-directory"} {
		t.Run(scenario, func(t *testing.T) {
			f, dir, c := initializedState(t)
			want, _ := trustIdentity(c)
			s, err := readState(dir, want)
			if err != nil {
				t.Fatal(err)
			}
			cache := filepath.Join(dir, s.Generation, "metadata")
			state := filepath.Join(dir, "state.json")
			must := func(err error) {
				t.Helper()
				if err != nil {
					t.Fatal(err)
				}
			}
			switch scenario {
			case "missing-state":
				must(os.Remove(state))
			case "missing-root":
				must(os.Remove(filepath.Join(cache, "root.json")))
			case "changed-root":
				must(os.WriteFile(filepath.Join(cache, "root.json"), append(c.BootstrapRoot, ' '), 0600))
			case "extra-cache-file":
				must(os.WriteFile(filepath.Join(cache, "extra.json"), []byte("{}"), 0600))
			case "unknown-state-field", "duplicate-state-field":
				b, err := os.ReadFile(state)
				must(err)
				prefix := `{"extra":1,`
				if scenario == "duplicate-state-field" {
					prefix = `{"schema":1,`
				}
				must(os.WriteFile(state, append([]byte(prefix), b[1:]...), 0600))
			case "pending-refresh":
				must(os.WriteFile(filepath.Join(dir, "refresh.pending"), []byte(strings.Repeat("a", 32)), 0600))
			case "wrong-origin":
				c.MetadataURL += "/new"
			case "wrong-channel":
				c.Channel = "stable"
			case "wrong-bootstrap":
				c.BootstrapRoot = newFixture(t).root
			case "missing-directory":
				must(os.RemoveAll(dir))
			}
			if _, err := verifyReleaseInState(dir, c, f.server.Client()); err == nil {
				t.Fatal("damaged/mismatched trust state accepted")
			}
			if scenario == "missing-directory" {
				if _, err := os.Stat(dir); !os.IsNotExist(err) {
					t.Fatal("check silently recreated trust")
				}
			}
		})
	}
}

func TestStatePrivateDirectoryAndExclusiveLock(t *testing.T) {
	f, dir, c := initializedState(t)
	if err := assertPrivateStateDirectory(dir); err != nil {
		t.Fatal(err)
	}
	if err := InitializeState(dir, c); err == nil {
		t.Fatal("reinitialization accepted")
	}
	empty := filepath.Join(t.TempDir(), "empty")
	if err := os.Mkdir(empty, 0700); err != nil {
		t.Fatal(err)
	}
	if err := InitializeState(empty, c); err == nil {
		t.Fatal("existing empty directory adopted")
	}
	unlock, err := lockState(dir)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := verifyReleaseInState(dir, c, f.server.Client()); err == nil {
		unlock()
		t.Fatal("concurrent state refresh accepted")
	}
	unlock()
	if _, err := verifyReleaseInState(dir, c, f.server.Client()); err != nil {
		t.Fatal(err)
	}
}

func TestStateRotatedRootSurvivesRestart(t *testing.T) {
	f, dir, c := initializedState(t)
	rotated, err := metadata.Root().FromBytes(f.root)
	if err != nil {
		t.Fatal(err)
	}
	rotated.Signed.Version = 2
	rotated.Signed.Expires = time.Now().Add(48 * time.Hour)
	rotated.Signatures = nil
	sign(t, rotated, f.keys["root"])
	f.files["/2.root.json"] = bytesOf(t, rotated)
	if _, err := verifyReleaseInState(dir, c, f.server.Client()); err != nil {
		t.Fatal(err)
	}
	// A later check must use the previously verified root even if the server
	// stops serving its rotation chain. Bootstrap is identity, not a reset floor.
	delete(f.files, "/2.root.json")
	if _, err := verifyReleaseInState(dir, c, f.server.Client()); err != nil {
		t.Fatal(err)
	}
	want, _ := trustIdentity(c)
	s, err := readState(dir, want)
	if err != nil {
		t.Fatal(err)
	}
	files, err := readStateMetadata(filepath.Join(dir, s.Generation))
	if err != nil {
		t.Fatal(err)
	}
	if !bytes.Equal(files["root.json"], bytesOf(t, rotated)) {
		t.Fatal("rotated root lost")
	}
}

func TestMetadataRedirectPreservesOriginalRoleHardLimit(t *testing.T) {
	f := newFixture(t)
	f.redirect = f.server.URL + "/blob"
	f.files["/blob"] = []byte(strings.Repeat(" ", (16<<10)+1))
	if _, _, err := f.verify("beta"); err == nil || !strings.Contains(err.Error(), "hard limit") {
		t.Fatalf("redirect raised timestamp resource ceiling: %v", err)
	}
}
