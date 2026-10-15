package launcher

import (
	"context"
	"encoding/json"
	"os"
	"path/filepath"
	"strings"
	"testing"
)

func TestInitializeTrustRevalidatesSelectionUnderLock(t *testing.T) {
	for _, configured := range []bool{false, true} {
		for _, scenario := range []string{"current changed", "stock changed", "owner changed", "state missing", "lock busy", "armed"} {
			t.Run(scenario+"/configured="+map[bool]string{false: "false", true: "true"}[configured], func(t *testing.T) {
				f := newLaunchFixture(t)
				if !configured {
					delete(f.p.files, fixtureRoot+"/generations/"+fixtureGeneration+"/app/update-trust.json")
				}
				f.p.lockBusy = scenario == "lock busy"
				f.p.onLock = func() {
					switch scenario {
					case "current changed":
						f.state.Current = nextGeneration
						f.storeState()
					case "stock changed":
						f.state.Stock.SHA256 = strings.Repeat("f", 64)
						f.storeState()
					case "owner changed":
						f.p.files[fixtureRoot+"/owner.json"] = []byte(`{"owner":"foreign"}`)
					case "state missing":
						delete(f.p.files, fixtureRoot+"/state.json")
					case "armed":
						f.p.files[fixtureRoot+"/"+armedUpdateMarker] = nil
					}
				}
				if err := run(context.Background(), []string{"--initialize-trust"}, f.s); err == nil {
					t.Fatal("stale, contended or armed initialization succeeded")
				}
				if _, present := f.p.files[fixtureRoot+"/update-trust-initialized-beta"]; present {
					t.Fatal("rejected selection created the irreversible once marker")
				}
				if len(f.p.private)+len(f.p.waits)+len(f.p.starts)+f.checks+f.prepares != 0 || f.p.lockHeld {
					t.Fatal("refusal changed trust directories, launched work or leaked the lock")
				}
				for _, held := range f.p.held {
					if !held.closed {
						t.Fatal("refusal leaked generation custody")
					}
				}
			})
		}
	}
}

func TestInitializeTrustNoConfigRechecksStateUnderLock(t *testing.T) {
	f := newLaunchFixture(t)
	delete(f.p.files, fixtureRoot+"/generations/"+fixtureGeneration+"/app/update-trust.json")
	if err := run(context.Background(), []string{"--initialize-trust"}, f.s); err != nil {
		t.Fatal(err)
	}
	if f.p.readsWhileLocked != 1 || f.p.lockHeld || len(f.p.private) != 0 {
		t.Fatal("no-config success bypassed current-generation serialization")
	}
}

// The optional output is public, freshly generated inert fixture material for
// test-installer-trust.ps1. The signing key never leaves testRoot's local scope;
// the URLs are reserved test domains and initialization performs no network I/O.
func TestInstallerTrustFixture(t *testing.T) {
	f := newLaunchFixture(t)
	if _, err := parseTrust(jsonBytes(t, f.trust)); err != nil {
		t.Fatal(err)
	}
	name := os.Getenv("SKAGER_INSTALLER_TRUST_FIXTURE")
	if name == "" {
		return
	}
	if !filepath.IsAbs(name) {
		t.Fatal("fixture output must be an explicit absolute new file")
	}
	data, err := json.Marshal(struct {
		Trust trustRecord `json:"trust"`
		Stock []byte      `json:"stock"`
	}{f.trust, testPE()})
	if err != nil {
		t.Fatal(err)
	}
	file, err := os.OpenFile(name, os.O_WRONLY|os.O_CREATE|os.O_EXCL, 0600)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := file.Write(data); err != nil {
		file.Close()
		t.Fatal(err)
	}
	if err := file.Close(); err != nil {
		t.Fatal(err)
	}
}
