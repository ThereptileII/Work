//go:build linux

package repository

import (
	"bytes"
	"errors"
	"io/fs"
	"os"
	"path/filepath"
	"reflect"
	"strings"
	"testing"

	"example.com/opennav-update-verifier/releasepolicy"
	"github.com/theupdateframework/go-tuf/v2/metadata/config"
	"github.com/theupdateframework/go-tuf/v2/metadata/updater"
)

func codeUpgradeFixture(t *testing.T) (*fixture, []byte, []byte) {
	t.Helper()
	f := newFixture(t)
	original := bootstrapFixture(t, f)
	if _, e := publishBootstrap(f, original); e != nil {
		t.Fatal(e)
	}
	old, _ := releasepolicy.ParseBrokerBootstrap(original)
	f.release(t, "0.4.1-beta.2", strings.Repeat("b", 40))
	successor := bootstrapFixture(t, f)
	next, _ := releasepolicy.ParseBrokerBootstrap(successor)
	p := releasepolicy.BrokerCodeUpgradePolicy{Schema: 1, Purpose: releasepolicy.BrokerCodeUpgradePurpose,
		PreviousBrokerSHA256: old.Broker.SHA256, PreviousHelperSHA256: old.Provisioner.SHA256, Bootstrap: next}
	raw := append(append([]byte(" \n"), jsonBytes(p)...), '\n')
	return f, original, raw
}

func publishCodeUpgrade(f *fixture, raw []byte) (int64, error) {
	return PublishWithBrokerCodeUpgrade(f.dir, f.assets, f.selection, f.policy, checksum(f.selection), raw, checksum(raw), f.online(), f.now)
}

func verifyCodeUpgrade(t *testing.T, f *fixture) ([]byte, releasepolicy.BrokerBootstrapIdentity) {
	t.Helper()
	cfg, e := config.New(f.server.URL, f.root)
	if e != nil {
		t.Fatal(e)
	}
	cfg.LocalMetadataDir, cfg.LocalTargetsDir = t.TempDir(), t.TempDir()
	if e = cfg.SetDefaultFetcherHTTPClient(f.server.Client()); e != nil {
		t.Fatal(e)
	}
	u, e := updater.New(cfg)
	if e != nil {
		t.Fatal(e)
	}
	if e = u.Refresh(); e != nil {
		t.Fatal(e)
	}
	target, e := u.GetTargetInfo("bootstrap/broker-code-upgrade-beta.json")
	if e != nil || target.Custom == nil {
		t.Fatalf("upgrade target missing: %v", e)
	}
	identity, e := releasepolicy.ParseBrokerCodeUpgradeIdentity(*target.Custom)
	if e != nil {
		t.Fatal(e)
	}
	if _, e = releasepolicy.ParseBrokerBootstrapIdentity(*target.Custom); e == nil {
		t.Fatal("upgrade identity accepted as enrollment")
	}
	_, raw, e := u.DownloadTarget(target, "", "")
	if e != nil {
		t.Fatal(e)
	}
	return raw, identity
}

func TestBrokerCodeUpgradePublicationAndRetention(t *testing.T) {
	f, original, raw := codeUpgradeFixture(t)
	if v, e := publishCodeUpgrade(f, raw); e != nil || v != 2 {
		t.Fatalf("publish: %d %v", v, e)
	}
	got, identity := verifyCodeUpgrade(t, f)
	p, _ := releasepolicy.ParseBrokerCodeUpgrade(raw)
	want := releasepolicy.BrokerBootstrapIdentity{Purpose: releasepolicy.BrokerCodeUpgradePurpose, Channel: p.Bootstrap.Channel,
		Version: p.Bootstrap.Version, Commit: p.Bootstrap.Commit, TrustConfigSHA256: p.Bootstrap.TrustConfigSHA256}
	if !bytes.Equal(got, raw) || identity != want || p.Bootstrap.Purpose != releasepolicy.BrokerBootstrapPurpose {
		t.Fatal("signed policy/identity/purpose changed")
	}
	if got, _ := verifyBootstrapTarget(t, f); !bytes.Equal(got, original) {
		t.Fatal("upgrade replaced enrollment")
	}
	if got, e := f.verify(t); e != nil || !bytes.Equal(got, f.policy) {
		t.Fatalf("app target regressed: %v", e)
	}
	// Exact renewal is permitted, reformatting is a different authority.
	if v, e := publishCodeUpgrade(f, raw); e != nil || v != 3 {
		t.Fatalf("renew: %d %v", v, e)
	}
	f.release(t, "0.4.1-beta.3", strings.Repeat("c", 40))
	if v, e := f.publish(); e != nil || v != 4 {
		t.Fatalf("retention: %d %v", v, e)
	}
	got, retained := verifyCodeUpgrade(t, f)
	if !bytes.Equal(got, raw) || identity != retained {
		t.Fatal("ordinary publication dropped upgrade authority")
	}
	if got, _ := verifyBootstrapTarget(t, f); !bytes.Equal(got, original) {
		t.Fatal("ordinary publication dropped enrollment")
	}
}

func TestBrokerCodeUpgradeCombinedEnrollmentAndArtifactReuse(t *testing.T) {
	f, _, raw := codeUpgradeFixture(t)
	p, _ := releasepolicy.ParseBrokerCodeUpgrade(raw)
	bootstrap := jsonBytes(p.Bootstrap)
	if _, e := PublishWithBrokerBootstrapAndCodeUpgrade(f.dir, f.assets, f.selection, f.policy, checksum(f.selection), bootstrap, checksum(bootstrap), raw, checksum(raw), f.online(), f.now); e != nil {
		t.Fatal(e)
	}
	// Reuse exactly authenticated already-published artifacts, without requiring
	// duplicate successor files in the staging assets directory.
	for _, a := range bootstrapArtifacts(p.Bootstrap) {
		if a.Path != p.Bootstrap.CorrespondingSource.Path && a.Path != p.Bootstrap.Notices.Path {
			if e := os.Remove(filepath.Join(f.assets, filepath.Base(a.Path))); e != nil {
				t.Fatal(e)
			}
		}
	}
	if _, e := publishCodeUpgrade(f, raw); e != nil {
		t.Fatalf("exact public artifact reuse refused: %v", e)
	}
	got, _ := verifyCodeUpgrade(t, f)
	if !bytes.Equal(got, raw) {
		t.Fatal("reused artifact publication changed target")
	}
}

func TestBrokerCodeUpgradeRefusalDoesNotReserve(t *testing.T) {
	for _, scenario := range []string{"missing pin", "wrong pin", "missing policy", "no enrollment history", "wrong old broker", "wrong old helper", "wrong config", "wrong app version", "wrong app commit", "other origin", "missing artifact", "changed artifact", "case alias", "shared identity conflict", "enrollment purpose", "machine identity", "target case alias", "target parent link", "target different bytes"} {
		t.Run(scenario, func(t *testing.T) {
			f, _, raw := codeUpgradeFixture(t)
			p, _ := releasepolicy.ParseBrokerCodeUpgrade(raw)
			pin := checksum(raw)
			rewrite := true
			switch scenario {
			case "missing pin":
				pin = ""
				rewrite = false
			case "wrong pin":
				pin = strings.Repeat("0", 64)
				rewrite = false
			case "missing policy":
				raw = nil
				rewrite = false
			case "no enrollment history":
				var s state
				b, _ := os.ReadFile(filepath.Join(f.dir, "state.json"))
				if e := decode(b, &s); e != nil {
					t.Fatal(e)
				}
				s.BrokerBootstrap = nil
				put(t, filepath.Join(f.dir, "state.json"), jsonBytes(s))
			case "wrong old broker":
				p.PreviousBrokerSHA256 = strings.Repeat("d", 64)
			case "wrong old helper":
				p.PreviousHelperSHA256 = strings.Repeat("e", 64)
			case "wrong config":
				p.Bootstrap.TrustConfigSHA256 = strings.Repeat("f", 64)
			case "wrong app version":
				p.Bootstrap.Version = "0.4.1-beta.3"
			case "wrong app commit":
				p.Bootstrap.Commit = strings.Repeat("c", 40)
			case "other origin":
				for _, a := range []*releasepolicy.Artifact{&p.Bootstrap.Broker, &p.Bootstrap.Provisioner, &p.Bootstrap.CorrespondingSource, &p.Bootstrap.Notices} {
					a.URL = "https://other.invalid/" + a.Path
				}
			case "missing artifact":
				os.Remove(filepath.Join(f.assets, "skager-update-broker.exe"))
			case "changed artifact":
				put(t, filepath.Join(f.assets, "skager-update-provisioner.exe"), []byte("CORRUPT"))
			case "case alias":
				p.Bootstrap.CorrespondingSource.Path = strings.Replace(p.Bootstrap.CorrespondingSource.Path, "SKAGER-Beta2-source.zip", "skager-beta2-source.zip", 1)
				p.Bootstrap.CorrespondingSource.URL = f.trust.ArtifactOrigin + "/" + p.Bootstrap.CorrespondingSource.Path
			case "shared identity conflict":
				p.Bootstrap.CorrespondingSource.SHA256 = strings.Repeat("d", 64)
			case "enrollment purpose":
				p.Purpose = releasepolicy.BrokerBootstrapPurpose
			case "target parent link":
				parent := filepath.Join(f.dir, "public", "targets", "bootstrap")
				if e := os.Rename(parent, parent+"-saved"); e != nil {
					t.Fatal(e)
				}
				if e := os.Symlink(parent+"-saved", parent); e != nil {
					t.Fatal(e)
				}
			case "target case alias", "target different bytes":
				dest, _ := codeUpgradeDestination(f.dir, p, &brokerCodeUpgradeState{Policy: raw, SHA256: pin})
				if scenario == "target case alias" {
					dest = filepath.Join(filepath.Dir(dest), strings.ToUpper(filepath.Base(dest)))
				}
				put(t, dest, []byte("INERT COLLISION"))
				rewrite = false
			case "machine identity":
				raw = bytes.Replace(raw, []byte(`"schema": 1`), []byte(`"schema": 1,"machineIdentity":"not-global"`), 1)
				pin = checksum(raw)
				rewrite = false
			}
			if rewrite {
				raw = jsonBytes(p)
				pin = checksum(raw)
			}
			assertUpgradeRefusedWithoutReservation(t, f, func() (int64, error) {
				return PublishWithBrokerCodeUpgrade(f.dir, f.assets, f.selection, f.policy, checksum(f.selection), raw, pin, f.online(), f.now)
			})
		})
	}
}

func assertUpgradeRefusedWithoutReservation(t *testing.T, f *fixture, invoke func() (int64, error)) {
	t.Helper()
	statePath := filepath.Join(f.dir, "state.json")
	before, _ := os.ReadFile(statePath)
	stamp, _ := os.ReadFile(filepath.Join(f.dir, "public", "timestamp.json"))
	publicBefore := codeUpgradePublicSnapshot(t, f.dir)
	if _, e := invoke(); e == nil {
		t.Fatal("invalid code upgrade accepted")
	}
	after, _ := os.ReadFile(statePath)
	published, _ := os.ReadFile(filepath.Join(f.dir, "public", "timestamp.json"))
	if !bytes.Equal(before, after) || !bytes.Equal(stamp, published) || !reflect.DeepEqual(publicBefore, codeUpgradePublicSnapshot(t, f.dir)) {
		t.Fatal("invalid upgrade consumed authority or changed visible publication")
	}
}

func codeUpgradePublicSnapshot(t *testing.T, directory string) map[string]string {
	t.Helper()
	result := map[string]string{}
	e := filepath.WalkDir(filepath.Join(directory, "public"), func(name string, d fs.DirEntry, e error) error {
		if e != nil {
			return e
		}
		if d.IsDir() {
			result[name] = "directory"
			return nil
		}
		if d.Type()&os.ModeSymlink != 0 {
			target, e := os.Readlink(name)
			result[name] = "link:" + target
			return e
		}
		b, e := os.ReadFile(name)
		result[name] = checksum(b)
		return e
	})
	if e != nil {
		t.Fatal(e)
	}
	return result
}

func TestBrokerCodeUpgradeOldPairUsesPrepublicationHistory(t *testing.T) {
	f, _, raw := codeUpgradeFixture(t)
	p, _ := releasepolicy.ParseBrokerCodeUpgrade(raw)
	// Simultaneous enrollment selection cannot manufacture original-code authority.
	bootstrap := jsonBytes(p.Bootstrap)
	p.PreviousBrokerSHA256 = strings.Repeat("e", 64)
	p.PreviousHelperSHA256 = strings.Repeat("f", 64)
	raw = jsonBytes(p)
	assertUpgradeRefusedWithoutReservation(t, f, func() (int64, error) {
		return PublishWithBrokerBootstrapAndCodeUpgrade(f.dir, f.assets, f.selection, f.policy, checksum(f.selection), bootstrap, checksum(bootstrap), raw, checksum(raw), f.online(), f.now)
	})
}

func TestBrokerCodeUpgradeVersionChainAndConflict(t *testing.T) {
	f, _, raw := codeUpgradeFixture(t)
	if _, e := publishCodeUpgrade(f, raw); e != nil {
		t.Fatal(e)
	}
	assertUpgradeRefusedWithoutReservation(t, f, func() (int64, error) { return publishCodeUpgrade(f, append(raw, ' ')) })
	previous, _ := releasepolicy.ParseBrokerCodeUpgrade(raw)
	f.release(t, "0.4.1-beta.3", strings.Repeat("c", 40))
	nextRaw := bootstrapFixture(t, f)
	successor, _ := releasepolicy.ParseBrokerBootstrap(nextRaw)
	next := releasepolicy.BrokerCodeUpgradePolicy{Schema: 1, Purpose: releasepolicy.BrokerCodeUpgradePurpose, PreviousBrokerSHA256: previous.PreviousBrokerSHA256, PreviousHelperSHA256: previous.PreviousHelperSHA256, Bootstrap: successor}
	assertUpgradeRefusedWithoutReservation(t, f, func() (int64, error) { return publishCodeUpgrade(f, jsonBytes(next)) })
	next.PreviousBrokerSHA256 = previous.Bootstrap.Broker.SHA256
	next.PreviousHelperSHA256 = previous.Bootstrap.Provisioner.SHA256
	if _, e := publishCodeUpgrade(f, jsonBytes(next)); e != nil {
		t.Fatal(e)
	}
	got, _ := verifyCodeUpgrade(t, f)
	if !bytes.Equal(got, jsonBytes(next)) {
		t.Fatal("monotonic chain target changed")
	}
}

func TestBrokerCodeUpgradeRetainedCorruptionRefusesBeforeReservation(t *testing.T) {
	for _, scenario := range []string{"state hash", "state config", "public policy", "missing policy", "changed artifact", "missing artifact", "case alias", "future target collision"} {
		t.Run(scenario, func(t *testing.T) {
			f, _, raw := codeUpgradeFixture(t)
			if _, e := publishCodeUpgrade(f, raw); e != nil {
				t.Fatal(e)
			}
			p, _ := releasepolicy.ParseBrokerCodeUpgrade(raw)
			var s state
			b, _ := os.ReadFile(filepath.Join(f.dir, "state.json"))
			if e := decode(b, &s); e != nil {
				t.Fatal(e)
			}
			dest, _ := codeUpgradeDestination(f.dir, p, s.BrokerCodeUpgrade)
			switch scenario {
			case "state hash":
				s.BrokerCodeUpgrade.SHA256 = strings.Repeat("0", 64)
				put(t, filepath.Join(f.dir, "state.json"), jsonBytes(s))
			case "state config":
				p.Bootstrap.TrustConfigSHA256 = strings.Repeat("0", 64)
				s.BrokerCodeUpgrade.Policy = jsonBytes(p)
				s.BrokerCodeUpgrade.SHA256 = checksum(s.BrokerCodeUpgrade.Policy)
				put(t, filepath.Join(f.dir, "state.json"), jsonBytes(s))
			case "public policy":
				put(t, dest, []byte("CORRUPT"))
			case "missing policy":
				os.Remove(dest)
			case "changed artifact":
				put(t, filepath.Join(f.dir, "public", filepath.FromSlash(p.Bootstrap.Broker.Path)), []byte("CORRUPT"))
			case "missing artifact":
				os.Remove(filepath.Join(f.dir, "public", filepath.FromSlash(p.Bootstrap.Broker.Path)))
			case "case alias":
				os.Rename(filepath.Join(f.dir, "public", filepath.FromSlash(p.Bootstrap.Broker.Path)), filepath.Join(filepath.Dir(filepath.Join(f.dir, "public", filepath.FromSlash(p.Bootstrap.Broker.Path))), "SKAGER-UPDATE-BROKER.EXE"))
			case "future target collision":
				f.release(t, "0.4.1-beta.3", strings.Repeat("c", 40))
				nextRaw := bootstrapFixture(t, f)
				next, _ := releasepolicy.ParseBrokerBootstrap(nextRaw)
				p.PreviousBrokerSHA256 = p.Bootstrap.Broker.SHA256
				p.PreviousHelperSHA256 = p.Bootstrap.Provisioner.SHA256
				p.Bootstrap = next
				raw = jsonBytes(p)
				dest, _ = codeUpgradeDestination(f.dir, p, &brokerCodeUpgradeState{Policy: raw, SHA256: checksum(raw)})
				put(t, dest, []byte("COLLISION"))
			}
			assertUpgradeRefusedWithoutReservation(t, f, func() (int64, error) {
				if scenario == "future target collision" {
					return publishCodeUpgrade(f, raw)
				}
				return f.publish()
			})
		})
	}
}

func TestBrokerCodeUpgradeInterruptedPublicationRetainsAuthority(t *testing.T) {
	for _, stop := range []string{"reserved", "metadata-staged"} {
		t.Run(stop, func(t *testing.T) {
			f, _, raw := codeUpgradeFixture(t)
			stamp, _ := os.ReadFile(filepath.Join(f.dir, "public", "timestamp.json"))
			_, e := publishBatchWithCodeUpgrade(f.dir, f.assets, f.selection, f.policy, checksum(f.selection), nil, "", raw, checksum(raw), f.online(), f.now, func(step string) error {
				if step == stop {
					return errors.New("inert interruption")
				}
				return nil
			})
			if e == nil {
				t.Fatal("interruption missed")
			}
			after, _ := os.ReadFile(filepath.Join(f.dir, "public", "timestamp.json"))
			if !bytes.Equal(stamp, after) {
				t.Fatal("interruption changed visible timestamp")
			}
			_, s, _, e := load(f.dir, f.now)
			if e != nil || s.Version != 2 || s.BrokerCodeUpgrade == nil || !bytes.Equal(s.BrokerCodeUpgrade.Policy, raw) {
				t.Fatal("reserved declaration was lost")
			}
			if stop == "reserved" {
				assertUpgradeRefusedWithoutReservation(t, f, f.publish)
				if v, e := publishCodeUpgrade(f, raw); e != nil || v != 3 {
					t.Fatalf("explicit recovery: %d %v", v, e)
				}
			} else if v, e := f.publish(); e != nil || v != 3 {
				t.Fatalf("retained recovery: %d %v", v, e)
			}
			got, _ := verifyCodeUpgrade(t, f)
			if !bytes.Equal(got, raw) {
				t.Fatal("recovery changed approved bytes")
			}
		})
	}
}
