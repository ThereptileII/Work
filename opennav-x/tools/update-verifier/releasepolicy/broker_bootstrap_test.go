package releasepolicy

import (
	"encoding/json"
	"strings"
	"testing"
)

func validBrokerBootstrap() BrokerBootstrapPolicy {
	artifact := func(name string) Artifact {
		path := "artifacts/" + commit + "/" + name
		return Artifact{Path: path, URL: "https://updates.skager.app/" + path, SHA256: hash, Bytes: 1024}
	}
	return BrokerBootstrapPolicy{
		Schema: 1, Product: "skager", Purpose: BrokerBootstrapPurpose, Channel: "beta", Version: "0.4.0-beta2", Commit: commit,
		Architecture: "x86", ServiceName: "SkagerUpdateBroker", ServiceAccount: `NT SERVICE\SkagerUpdateBroker`,
		CodeLocation: "ProgramFilesX86/SKAGER/UpdateBroker", DataLocation: "ProgramData/SKAGER/UpdateBroker", TrustConfigSHA256: hash,
		Broker: artifact("skager-update-broker.exe"), Provisioner: artifact("skager-update-provisioner.exe"),
		CorrespondingSource: artifact("source.tar.gz"), Notices: artifact("notices.txt"),
	}
}

func bootstrapJSON(t *testing.T, value any) []byte {
	t.Helper()
	data, err := json.Marshal(value)
	if err != nil {
		t.Fatal(err)
	}
	return data
}

func TestBrokerBootstrapValidDeclarations(t *testing.T) {
	for _, tc := range []struct{ channel, version string }{
		{"beta", "0.4.0-beta2"}, {"beta", "0.4.0"}, {"stable", "0.4.0+build.7"},
	} {
		t.Run(tc.channel+"/"+tc.version, func(t *testing.T) {
			p := validBrokerBootstrap()
			p.Channel, p.Version = tc.channel, tc.version
			got, err := ParseBrokerBootstrap(bootstrapJSON(t, p))
			if err != nil || got != p {
				t.Fatalf("bootstrap round trip: got %+v, %v", got, err)
			}
			identity := BrokerBootstrapIdentity{Purpose: p.Purpose, Channel: p.Channel, Version: p.Version, Commit: p.Commit, TrustConfigSHA256: p.TrustConfigSHA256}
			parsed, err := ParseBrokerBootstrapIdentity(bootstrapJSON(t, identity))
			if err != nil || parsed != identity {
				t.Fatalf("identity round trip: got %+v, %v", parsed, err)
			}
			path, err := BrokerBootstrapTargetPath(tc.channel)
			if err != nil || path != "bootstrap/broker-"+tc.channel+".json" {
				t.Fatalf("target: %q, %v", path, err)
			}
		})
	}
	for _, channel := range []string{"", "Beta", "production", "../stable", "stable.json", "stable\x00"} {
		if path, err := BrokerBootstrapTargetPath(channel); err == nil || path != "" {
			t.Fatalf("accepted unsupported target channel %q: %q, %v", channel, path, err)
		}
	}
}

func TestBrokerBootstrapStrictJSON(t *testing.T) {
	data := bootstrapJSON(t, validBrokerBootstrap())
	assertRejected := func(t *testing.T, data []byte) {
		t.Helper()
		if p, err := ParseBrokerBootstrap(data); err == nil || p != (BrokerBootstrapPolicy{}) {
			t.Fatalf("invalid declaration returned nonzero policy or no error: %+v, %v", p, err)
		}
	}
	for name, invalid := range map[string][]byte{
		"empty": nil, "nonobject": []byte(`[]`), "null": []byte(`null`), "malformed": []byte(`{"schema":`),
		"invalid UTF8": append(append([]byte(nil), data...), 0xff), "oversize": []byte(strings.Repeat(" ", maxPolicyBytes+1)),
		"trailing object":       append(append([]byte(nil), data...), []byte(` {}`)...),
		"trailing scalar":       append(append([]byte(nil), data...), []byte(` true`)...),
		"duplicate escaped key": []byte(strings.Replace(string(data), `"schema":1`, `"schema":1,"\u0073chema":1`, 1)),
		"wrong field type":      []byte(strings.Replace(string(data), `"schema":1`, `"schema":"1"`, 1)),
		"excess nesting":        []byte(strings.Repeat("[", 18) + "0" + strings.Repeat("]", 18)),
	} {
		t.Run(name, func(t *testing.T) { assertRejected(t, invalid) })
	}
	var root map[string]json.RawMessage
	if err := json.Unmarshal(data, &root); err != nil {
		t.Fatal(err)
	}
	for key, value := range root {
		for _, mode := range []string{"missing", "null", "casefold", "duplicate", "unknown"} {
			t.Run(key+"/"+mode, func(t *testing.T) {
				copy := make(map[string]json.RawMessage, len(root))
				for k, v := range root {
					copy[k] = v
				}
				switch mode {
				case "missing":
					delete(copy, key)
				case "null":
					copy[key] = json.RawMessage(`null`)
				case "casefold":
					delete(copy, key)
					copy[strings.ToUpper(key)] = value
				case "unknown":
					copy["untrustedCommand"] = json.RawMessage(`"run.exe"`)
				case "duplicate":
					encoded := bootstrapJSON(t, copy)
					assertRejected(t, []byte(`{"`+key+`":`+string(value)+`,`+string(encoded[1:])))
					return
				}
				assertRejected(t, bootstrapJSON(t, copy))
			})
		}
	}
	for _, artifact := range []string{"broker", "provisioner", "correspondingSource", "notices"} {
		for _, key := range []string{"path", "url", "sha256", "bytes"} {
			for _, mode := range []string{"missing", "null", "casefold", "duplicate", "unknown"} {
				t.Run(artifact+"/"+key+"/"+mode, func(t *testing.T) {
					var fields map[string]json.RawMessage
					if err := json.Unmarshal(root[artifact], &fields); err != nil {
						t.Fatal(err)
					}
					value := fields[key]
					switch mode {
					case "missing":
						delete(fields, key)
					case "null":
						fields[key] = json.RawMessage(`null`)
					case "casefold":
						delete(fields, key)
						fields[strings.ToUpper(key)] = value
					case "unknown":
						fields["redirect"] = json.RawMessage(`"https://elsewhere.invalid/"`)
					}
					encoded := bootstrapJSON(t, fields)
					if mode == "duplicate" {
						encoded = []byte(`{"` + key + `":` + string(value) + `,` + string(encoded[1:]))
					}
					copy := make(map[string]json.RawMessage, len(root))
					for k, v := range root {
						copy[k] = v
					}
					copy[artifact] = encoded
					assertRejected(t, bootstrapJSON(t, copy))
				})
			}
		}
	}
}

func TestBrokerBootstrapRejectsAuthorityAndArtifactSubstitution(t *testing.T) {
	setPath := func(a *Artifact, path string) { a.Path, a.URL = path, "https://updates.skager.app/"+path }
	cases := map[string]func(*BrokerBootstrapPolicy){
		"schema":                  func(p *BrokerBootstrapPolicy) { p.Schema = 2 },
		"product":                 func(p *BrokerBootstrapPolicy) { p.Product = "opennav" },
		"application purpose":     func(p *BrokerBootstrapPolicy) { p.Purpose = "application-update" },
		"unknown channel":         func(p *BrokerBootstrapPolicy) { p.Channel = "production" },
		"stable prerelease":       func(p *BrokerBootstrapPolicy) { p.Channel = "stable" },
		"invalid version":         func(p *BrokerBootstrapPolicy) { p.Version = "1.01.0" },
		"uppercase commit":        func(p *BrokerBootstrapPolicy) { p.Commit = strings.ToUpper(commit) },
		"trust hash uppercase":    func(p *BrokerBootstrapPolicy) { p.TrustConfigSHA256 = strings.ToUpper(hash) },
		"trust hash wrong length": func(p *BrokerBootstrapPolicy) { p.TrustConfigSHA256 = hash[:63] },
		"x64":                     func(p *BrokerBootstrapPolicy) { p.Architecture = "x64" },
		"alternate service":       func(p *BrokerBootstrapPolicy) { p.ServiceName = "OtherBroker" },
		"privileged account":      func(p *BrokerBootstrapPolicy) { p.ServiceAccount = "LocalSystem" },
		"raw code path":           func(p *BrokerBootstrapPolicy) { p.CodeLocation = `C:\Program Files (x86)\SKAGER\UpdateBroker` },
		"raw data path":           func(p *BrokerBootstrapPolicy) { p.DataLocation = `C:\ProgramData\SKAGER\UpdateBroker` },
		"expanded location":       func(p *BrokerBootstrapPolicy) { p.CodeLocation = "%ProgramFiles(x86)%/SKAGER/UpdateBroker" },
		"broker name":             func(p *BrokerBootstrapPolicy) { setPath(&p.Broker, "artifacts/"+commit+"/other.exe") },
		"provisioner name":        func(p *BrokerBootstrapPolicy) { setPath(&p.Provisioner, "artifacts/"+commit+"/other.exe") },
		"foreign commit":          func(p *BrokerBootstrapPolicy) { setPath(&p.Notices, "artifacts/"+otherCommit+"/notices.txt") },
		"path traversal":          func(p *BrokerBootstrapPolicy) { setPath(&p.Notices, "artifacts/"+commit+"/../notices.txt") },
		"nested source": func(p *BrokerBootstrapPolicy) {
			setPath(&p.CorrespondingSource, "artifacts/"+commit+"/source/archive.zip")
		},
		"long source name": func(p *BrokerBootstrapPolicy) {
			setPath(&p.CorrespondingSource, "artifacts/"+commit+"/"+strings.Repeat("a", 129))
		},
		"source notices collision": func(p *BrokerBootstrapPolicy) { p.Notices = p.CorrespondingSource },
		"case alias collision":     func(p *BrokerBootstrapPolicy) { setPath(&p.Notices, "artifacts/"+commit+"/SOURCE.TAR.GZ") },
		"executable alias collision": func(p *BrokerBootstrapPolicy) {
			setPath(&p.CorrespondingSource, "artifacts/"+commit+"/SKAGER-UPDATE-BROKER.EXE")
		},
		"reserved name":      func(p *BrokerBootstrapPolicy) { setPath(&p.Notices, "artifacts/"+commit+"/CON.txt") },
		"trailing dot alias": func(p *BrokerBootstrapPolicy) { setPath(&p.Notices, "artifacts/"+commit+"/notices.txt.") },
		"http":               func(p *BrokerBootstrapPolicy) { p.Broker.URL = strings.Replace(p.Broker.URL, "https:", "http:", 1) },
		"foreign origin": func(p *BrokerBootstrapPolicy) {
			p.Provisioner.URL = strings.Replace(p.Provisioner.URL, "updates.skager.app", "elsewhere.invalid", 1)
		},
		"origin case alias": func(p *BrokerBootstrapPolicy) {
			p.Provisioner.URL = strings.Replace(p.Provisioner.URL, "updates.skager.app", "UPDATES.skager.app", 1)
		},
		"origin port alias": func(p *BrokerBootstrapPolicy) {
			p.Provisioner.URL = strings.Replace(p.Provisioner.URL, "updates.skager.app", "updates.skager.app:443", 1)
		},
		"empty port": func(p *BrokerBootstrapPolicy) {
			p.Broker.URL = strings.Replace(p.Broker.URL, "updates.skager.app", "updates.skager.app:", 1)
		},
		"credentials": func(p *BrokerBootstrapPolicy) {
			p.Broker.URL = strings.Replace(p.Broker.URL, "https://", "https://user:pass@", 1)
		},
		"query":          func(p *BrokerBootstrapPolicy) { p.Broker.URL += "?command=run" },
		"empty query":    func(p *BrokerBootstrapPolicy) { p.Broker.URL += "?" },
		"fragment":       func(p *BrokerBootstrapPolicy) { p.Broker.URL += "#run" },
		"empty fragment": func(p *BrokerBootstrapPolicy) { p.Broker.URL += "#" },
		"encoded path alias": func(p *BrokerBootstrapPolicy) {
			p.Broker.URL = strings.Replace(p.Broker.URL, "skager-update", "%73kager-update", 1)
		},
		"URL path mismatch": func(p *BrokerBootstrapPolicy) { p.Broker.URL += ".other" },
		"empty artifact":    func(p *BrokerBootstrapPolicy) { p.Broker.Bytes = 0 },
		"oversize artifact": func(p *BrokerBootstrapPolicy) { p.Provisioner.Bytes = maxArtifactBytes + 1 },
		"bad artifact hash": func(p *BrokerBootstrapPolicy) { p.Notices.SHA256 = "unknown" },
	}
	for name, mutate := range cases {
		t.Run(name, func(t *testing.T) {
			p := validBrokerBootstrap()
			mutate(&p)
			if got, err := ParseBrokerBootstrap(bootstrapJSON(t, p)); err == nil || got != (BrokerBootstrapPolicy{}) {
				t.Fatalf("accepted substituted authority/artifact: %+v, %v", got, err)
			}
		})
	}
}

func TestBrokerBootstrapIdentityStrictAndPurposeBound(t *testing.T) {
	identity := BrokerBootstrapIdentity{Purpose: BrokerBootstrapPurpose, Channel: "beta", Version: "0.4.0-beta2", Commit: commit, TrustConfigSHA256: hash}
	data := bootstrapJSON(t, identity)
	assertRejected := func(t *testing.T, data []byte) {
		t.Helper()
		if got, err := ParseBrokerBootstrapIdentity(data); err == nil || got != (BrokerBootstrapIdentity{}) {
			t.Fatalf("invalid identity returned nonzero identity or no error: %+v, %v", got, err)
		}
	}
	for name, invalid := range map[string][]byte{
		"empty": nil, "null": []byte(`null`), "array": []byte(`[]`), "malformed": []byte(`{`),
		"trailing": append(append([]byte(nil), data...), []byte(` {}`)...),
		"oversize": []byte(strings.Repeat(" ", maxPolicyBytes+1)), "invalid UTF8": {0xff},
		"release policy confusion":   bootstrapJSON(t, validPolicy()),
		"bootstrap policy confusion": bootstrapJSON(t, validBrokerBootstrap()),
	} {
		t.Run(name, func(t *testing.T) { assertRejected(t, invalid) })
	}
	var root map[string]json.RawMessage
	if err := json.Unmarshal(data, &root); err != nil {
		t.Fatal(err)
	}
	for key, value := range root {
		for _, mode := range []string{"missing", "null", "duplicate", "casefold", "unknown", "wrong type"} {
			t.Run(key+"/"+mode, func(t *testing.T) {
				copy := make(map[string]json.RawMessage, len(root))
				for k, v := range root {
					copy[k] = v
				}
				switch mode {
				case "missing":
					delete(copy, key)
				case "null":
					copy[key] = json.RawMessage(`null`)
				case "casefold":
					delete(copy, key)
					copy[strings.ToUpper(key)] = value
				case "unknown":
					copy["authorization"] = json.RawMessage(`true`)
				case "wrong type":
					copy[key] = json.RawMessage(`1`)
				case "duplicate":
					assertRejected(t, []byte(`{"`+key+`":`+string(value)+`,`+string(data[1:])))
					return
				}
				assertRejected(t, bootstrapJSON(t, copy))
			})
		}
	}
	for name, mutate := range map[string]func(*BrokerBootstrapIdentity){
		"purpose":           func(p *BrokerBootstrapIdentity) { p.Purpose = "application-update" },
		"channel":           func(p *BrokerBootstrapIdentity) { p.Channel = "Beta" },
		"version":           func(p *BrokerBootstrapIdentity) { p.Version = "1.0" },
		"stable prerelease": func(p *BrokerBootstrapIdentity) { p.Channel = "stable" },
		"commit":            func(p *BrokerBootstrapIdentity) { p.Commit = strings.ToUpper(commit) },
		"trust hash":        func(p *BrokerBootstrapIdentity) { p.TrustConfigSHA256 = strings.ToUpper(hash) },
	} {
		t.Run(name, func(t *testing.T) { p := identity; mutate(&p); assertRejected(t, bootstrapJSON(t, p)) })
	}
}
