//go:build linux

package repository

import (
	"bytes"
	"encoding/json"
	"errors"
	"os"
	"path/filepath"
	"strings"
	"testing"

	"example.com/opennav-update-verifier/releasepolicy"
	"github.com/theupdateframework/go-tuf/v2/metadata/config"
	"github.com/theupdateframework/go-tuf/v2/metadata/updater"
)

func bootstrapFixture(t *testing.T, f *fixture) []byte {
	t.Helper()
	app, e := releasepolicy.Parse(f.policy)
	if e != nil {
		t.Fatal(e)
	}
	config, e := os.ReadFile(filepath.Join(f.dir, "update-trust.json"))
	if e != nil {
		t.Fatal(e)
	}
	artifact := func(name string) releasepolicy.Artifact {
		b := []byte("INERT TEST ARTIFACT " + name + app.Commit)
		put(t, filepath.Join(f.assets, name), b)
		p := "artifacts/" + app.Commit + "/" + name
		return releasepolicy.Artifact{Path: p, URL: f.trust.ArtifactOrigin + "/" + p, SHA256: checksum(b), Bytes: int64(len(b))}
	}
	p := releasepolicy.BrokerBootstrapPolicy{
		Schema: 1, Product: "skager", Purpose: releasepolicy.BrokerBootstrapPurpose,
		Channel: "beta", Version: app.Version, Commit: app.Commit, Architecture: "x86",
		ServiceName: "SkagerUpdateBroker", ServiceAccount: `NT SERVICE\SkagerUpdateBroker`,
		CodeLocation: "ProgramFilesX86/SKAGER/UpdateBroker", DataLocation: "ProgramData/SKAGER/UpdateBroker",
		TrustConfigSHA256: checksum(config), Broker: artifact("skager-update-broker.exe"),
		Provisioner: artifact("skager-update-provisioner.exe"), CorrespondingSource: app.CorrespondingSource, Notices: app.Notices,
	}
	// Distinct formatting must survive signed publication and retained state.
	b, e := json.Marshal(p)
	if e != nil {
		t.Fatal(e)
	}
	return append(append([]byte(" \n"), b...), '\n')
}

func publishBootstrap(f *fixture, b []byte) (int64, error) {
	return PublishWithBrokerBootstrap(f.dir, f.assets, f.selection, f.policy, checksum(f.selection), b, checksum(b), f.online(), f.now)
}

func verifyBootstrapTarget(t *testing.T, f *fixture) ([]byte, releasepolicy.BrokerBootstrapIdentity) {
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
	target, e := u.GetTargetInfo("bootstrap/broker-beta.json")
	if e != nil || target.Custom == nil {
		t.Fatalf("bootstrap target missing: %v", e)
	}
	identity, e := releasepolicy.ParseBrokerBootstrapIdentity(*target.Custom)
	if e != nil {
		t.Fatal(e)
	}
	_, b, e := u.DownloadTarget(target, "", "")
	if e != nil {
		t.Fatal(e)
	}
	return b, identity
}

func TestBrokerBootstrapPublicationAndRetention(t *testing.T) {
	f := newFixture(t)
	b := bootstrapFixture(t, f)
	if v, e := publishBootstrap(f, b); e != nil || v != 1 {
		t.Fatalf("publish: %d %v", v, e)
	}
	got, identity := verifyBootstrapTarget(t, f)
	p, _ := releasepolicy.ParseBrokerBootstrap(b)
	if !bytes.Equal(got, b) || identity.Purpose != p.Purpose || identity.Channel != p.Channel || identity.Version != p.Version || identity.Commit != p.Commit || identity.TrustConfigSHA256 != p.TrustConfigSHA256 {
		t.Fatal("exact policy or signed purpose/config identity changed")
	}
	if got, e := f.verify(t); e != nil || !bytes.Equal(got, f.policy) {
		t.Fatalf("ordinary target regressed: %v", e)
	}
	// Later app selection does not replace or drop a separately approved broker.
	f.release(t, "0.4.1-beta.2", strings.Repeat("b", 40))
	if v, e := f.publish(); e != nil || v != 2 {
		t.Fatalf("retained publish: %d %v", v, e)
	}
	got, retained := verifyBootstrapTarget(t, f)
	if !bytes.Equal(got, b) || retained != identity {
		t.Fatal("ordinary publish changed retained bootstrap")
	}
	if got, e := f.verify(t); e != nil || !bytes.Equal(got, f.policy) {
		t.Fatalf("new app target missing: %v", e)
	}
	// Explicit approved upgrade binds the new app revision and artifact bytes.
	next := bootstrapFixture(t, f)
	if v, e := publishBootstrap(f, next); e != nil || v != 3 {
		t.Fatalf("upgrade: %d %v", v, e)
	}
	got, retained = verifyBootstrapTarget(t, f)
	if !bytes.Equal(got, next) || retained.Commit != strings.Repeat("b", 40) {
		t.Fatal("approved bootstrap upgrade missing")
	}
}

func TestBrokerBootstrapRefusalDoesNotReserve(t *testing.T) {
	for _, scenario := range []string{"missing pin", "wrong pin", "missing policy", "wrong config", "wrong app version", "wrong app commit", "other origin", "missing artifact", "changed artifact", "case alias", "shared path different identity"} {
		t.Run(scenario, func(t *testing.T) {
			f := newFixture(t)
			b := bootstrapFixture(t, f)
			p, _ := releasepolicy.ParseBrokerBootstrap(b)
			pin := checksum(b)
			switch scenario {
			case "missing pin":
				pin = ""
			case "wrong pin":
				pin = strings.Repeat("0", 64)
			case "missing policy":
				b = nil
			case "wrong config":
				p.TrustConfigSHA256 = strings.Repeat("f", 64)
			case "wrong app version":
				p.Version = "0.4.1-beta.2"
			case "wrong app commit":
				p.Commit = strings.Repeat("c", 40)
			case "other origin":
				for _, a := range []*releasepolicy.Artifact{&p.Broker, &p.Provisioner, &p.CorrespondingSource, &p.Notices} {
					a.URL = "https://other.invalid/" + a.Path
				}
			case "missing artifact":
				os.Remove(filepath.Join(f.assets, "skager-update-broker.exe"))
			case "changed artifact":
				put(t, filepath.Join(f.assets, "skager-update-provisioner.exe"), []byte("corrupt"))
			case "case alias":
				p.CorrespondingSource.Path = strings.Replace(p.CorrespondingSource.Path, "SKAGER-Beta2-source.zip", "skager-beta2-source.zip", 1)
				p.CorrespondingSource.URL = f.trust.ArtifactOrigin + "/" + p.CorrespondingSource.Path
			case "shared path different identity":
				p.CorrespondingSource.SHA256 = strings.Repeat("d", 64)
			}
			if scenario != "missing pin" && scenario != "wrong pin" && scenario != "missing policy" && scenario != "missing artifact" && scenario != "changed artifact" {
				b = jsonBytes(p)
				pin = checksum(b)
			}
			before, _ := os.ReadFile(filepath.Join(f.dir, "state.json"))
			if _, e := PublishWithBrokerBootstrap(f.dir, f.assets, f.selection, f.policy, checksum(f.selection), b, pin, f.online(), f.now); e == nil {
				t.Fatal("unapproved/inconsistent bootstrap accepted")
			}
			after, _ := os.ReadFile(filepath.Join(f.dir, "state.json"))
			if !bytes.Equal(before, after) {
				t.Fatal("invalid bootstrap reserved a publication")
			}
			if _, e := os.Lstat(filepath.Join(f.dir, "public", "timestamp.json")); !os.IsNotExist(e) {
				t.Fatal("invalid bootstrap became visible")
			}
		})
	}
}

func TestBrokerBootstrapSameVersionReplacementRefused(t *testing.T) {
	f := newFixture(t)
	b := bootstrapFixture(t, f)
	if _, e := publishBootstrap(f, b); e != nil {
		t.Fatal(e)
	}
	// Even semantically identical reformatting is a different approved identity.
	before, _ := os.ReadFile(filepath.Join(f.dir, "state.json"))
	if _, e := publishBootstrap(f, append(b, ' ')); e == nil {
		t.Fatal("same-version replacement accepted")
	}
	after, _ := os.ReadFile(filepath.Join(f.dir, "state.json"))
	if !bytes.Equal(before, after) {
		t.Fatal("replacement changed floor")
	}
	if v, e := publishBootstrap(f, b); e != nil || v != 2 {
		t.Fatalf("exact renewal refused: %d %v", v, e)
	}
}

func TestBrokerBootstrapInterruptedPublicationRetainsFloor(t *testing.T) {
	for _, stop := range []string{"reserved", "metadata-staged"} {
		t.Run(stop, func(t *testing.T) {
			f := newFixture(t)
			if _, e := f.publish(); e != nil {
				t.Fatal(e)
			}
			original, _ := os.ReadFile(filepath.Join(f.dir, "public", "timestamp.json"))
			b := bootstrapFixture(t, f)
			_, e := publishBatch(f.dir, f.assets, f.selection, f.policy, checksum(f.selection), b, checksum(b), f.online(), f.now, func(step string) error {
				if step == stop {
					return errors.New("injected interruption")
				}
				return nil
			})
			if e == nil {
				t.Fatal("interruption missed")
			}
			after, _ := os.ReadFile(filepath.Join(f.dir, "public", "timestamp.json"))
			if !bytes.Equal(original, after) {
				t.Fatal("interrupted bootstrap changed publication")
			}
			_, s, _, e := load(f.dir, f.now)
			if e != nil || s.Version != 2 || s.BrokerBootstrap == nil || !bytes.Equal(s.BrokerBootstrap.Policy, b) {
				t.Fatalf("approval/floor not retained: %v", e)
			}
			if stop == "reserved" {
				// Missing pending bytes must fail closed, not silently drop the target.
				if _, e = f.publish(); e == nil {
					t.Fatal("ordinary publish dropped incomplete bootstrap")
				}
				if v, e := publishBootstrap(f, b); e != nil || v != 3 {
					t.Fatalf("explicit repair lost floor: %d %v", v, e)
				}
			} else {
				if v, e := f.publish(); e != nil || v != 3 {
					t.Fatalf("retained retry lost floor: %d %v", v, e)
				}
			}
			got, _ := verifyBootstrapTarget(t, f)
			if !bytes.Equal(got, b) {
				t.Fatal("recovered bootstrap differs")
			}
		})
	}
}

func TestBrokerBootstrapRetainedCorruptionRefused(t *testing.T) {
	for _, scenario := range []string{"state hash", "state config", "missing published artifact", "changed published artifact"} {
		t.Run(scenario, func(t *testing.T) {
			f := newFixture(t)
			b := bootstrapFixture(t, f)
			if _, e := publishBootstrap(f, b); e != nil {
				t.Fatal(e)
			}
			p, _ := releasepolicy.ParseBrokerBootstrap(b)
			var s state
			statePath := filepath.Join(f.dir, "state.json")
			raw, _ := os.ReadFile(statePath)
			if e := decode(raw, &s); e != nil {
				t.Fatal(e)
			}
			switch scenario {
			case "state hash":
				s.BrokerBootstrap.SHA256 = strings.Repeat("0", 64)
				put(t, statePath, jsonBytes(s))
			case "state config":
				p.TrustConfigSHA256 = strings.Repeat("0", 64)
				s.BrokerBootstrap.Policy = jsonBytes(p)
				s.BrokerBootstrap.SHA256 = checksum(s.BrokerBootstrap.Policy)
				put(t, statePath, jsonBytes(s))
			case "missing published artifact":
				os.Remove(filepath.Join(f.dir, "public", filepath.FromSlash(p.Broker.Path)))
			case "changed published artifact":
				put(t, filepath.Join(f.dir, "public", filepath.FromSlash(p.Broker.Path)), []byte("corrupt"))
			}
			before, _ := os.ReadFile(statePath)
			if _, e := f.publish(); e == nil {
				t.Fatal("corrupt retained bootstrap accepted")
			}
			after, _ := os.ReadFile(statePath)
			if !bytes.Equal(before, after) {
				t.Fatal("corruption silently reset or reserved state")
			}
		})
	}
}

func TestBrokerBootstrapHistoricalCollisionDoesNotPoisonFloor(t *testing.T) {
	for _, scenario := range []string{"different immutable bytes", "historical case alias"} {
		t.Run(scenario, func(t *testing.T) {
			f := newFixture(t)
			b := bootstrapFixture(t, f)
			p, _ := releasepolicy.ParseBrokerBootstrap(b)
			if scenario == "historical case alias" {
				raw := []byte("INERT BROKER CORRESPONDING SOURCE FIXTURE")
				put(t, filepath.Join(f.assets, "broker-source.zip"), raw)
				p.CorrespondingSource = releasepolicy.Artifact{Path: "artifacts/" + p.Commit + "/broker-source.zip", SHA256: checksum(raw), Bytes: int64(len(raw))}
				p.CorrespondingSource.URL = f.trust.ArtifactOrigin + "/" + p.CorrespondingSource.Path
				b = jsonBytes(p)
			}
			if _, e := publishBootstrap(f, b); e != nil {
				t.Fatal(e)
			}
			f.release(t, "0.4.1-beta.2", p.Commit)
			p.Version = "0.4.1-beta.2"
			if scenario == "different immutable bytes" {
				raw := []byte("CHANGED INERT BROKER")
				put(t, filepath.Join(f.assets, "skager-update-broker.exe"), raw)
				p.Broker.SHA256, p.Broker.Bytes = checksum(raw), int64(len(raw))
			} else {
				p.CorrespondingSource.Path = strings.Replace(p.CorrespondingSource.Path, "broker-source.zip", "BROKER-SOURCE.zip", 1)
				p.CorrespondingSource.URL = f.trust.ArtifactOrigin + "/" + p.CorrespondingSource.Path
				raw, _ := os.ReadFile(filepath.Join(f.assets, "broker-source.zip"))
				put(t, filepath.Join(f.assets, "BROKER-SOURCE.zip"), raw)
			}
			before, _ := os.ReadFile(filepath.Join(f.dir, "state.json"))
			if _, e := publishBootstrap(f, jsonBytes(p)); e == nil {
				t.Fatal("historical immutable collision accepted")
			}
			after, _ := os.ReadFile(filepath.Join(f.dir, "state.json"))
			if !bytes.Equal(before, after) {
				t.Fatal("known invalid input poisoned authority floor")
			}
			if v, e := f.publish(); e != nil || v != 2 {
				t.Fatalf("previous broker could not be retained: %d %v", v, e)
			}
			got, _ := verifyBootstrapTarget(t, f)
			if !bytes.Equal(got, b) {
				t.Fatal("collision damaged original broker declaration")
			}
		})
	}
}
