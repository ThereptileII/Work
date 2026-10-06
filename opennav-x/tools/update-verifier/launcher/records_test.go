package launcher

import (
	"bytes"
	"encoding/binary"
	"strings"
	"testing"
)

func TestLauncherPathDerivationRejectsAmbiguity(t *testing.T) {
	valid := fixtureRoot + "/generations/" + fixtureGeneration + "/app/skager-start.exe"
	for _, input := range []string{valid, strings.ReplaceAll(valid, "/", "\\")} {
		got, err := deriveLayout(input)
		if err != nil || got.root != fixtureRoot || got.generation != fixtureGeneration {
			t.Fatalf("valid installed path failed: %+v %v", got, err)
		}
	}
	for _, input := range []string{"skager-start.exe", "//server/share/skager-start.exe", "\\\\?\\C:\\generation\\app\\skager-start.exe", "C:relative/app/skager-start.exe", strings.Replace(valid, "/app/", "/app/../app/", 1), strings.Replace(valid, fixtureGeneration, "not-a-generation", 1), strings.Replace(valid, "/generations/", "/other/", 1), strings.Replace(valid, "skager-start.exe", "arbitrary.exe", 1), valid + ":stream", strings.Replace(valid, "/app/", "/NUL/", 1), strings.Replace(valid, "/app/", "/app. /", 1)} {
		if _, err := deriveLayout(input); err == nil {
			t.Fatalf("ambiguous/remote path accepted: %q", input)
		}
	}
	for _, relative := range []string{"../evil.exe", "app//evil.exe", "/app/evil.exe", "app\\evil.exe", "app/CON.exe", "app/file.exe:stream", "app/file.exe.", "app/evil\n.exe"} {
		if _, err := ownedPath(fixtureRoot, relative); err == nil {
			t.Fatalf("unsafe inventory path accepted: %q", relative)
		}
	}
}

func TestLauncherStrictTrustConfiguration(t *testing.T) {
	f := newLaunchFixture(t)
	original := jsonBytes(t, f.trust)
	if _, err := parseTrust(original); err != nil {
		t.Fatal(err)
	}
	for _, scenario := range []string{"unknown", "duplicate", "alias", "nested alias", "missing", "trailing", "http", "credentials", "query", "origin path", "unknown channel", "unknown baseline", "unknown arch", "duplicate allowlist", "bad root", "oversize"} {
		t.Run(scenario, func(t *testing.T) {
			copy := f.trust
			copy.Allowlist = append(copy.Allowlist[:0:0], copy.Allowlist...)
			data := original
			switch scenario {
			case "unknown":
				data = append([]byte(`{"extra":true,`), original[1:]...)
			case "duplicate":
				data = append([]byte(`{"schema":1,`), original[1:]...)
			case "alias":
				data = bytes.Replace(original, []byte(`"schema"`), []byte(`"Schema"`), 1)
			case "nested alias":
				data = bytes.Replace(original, []byte(`"executableSha256"`), []byte(`"ExecutableSha256"`), 1)
			case "missing":
				data = bytes.Replace(original, []byte(`"schema":1,`), nil, 1)
			case "trailing":
				data = append(append([]byte(nil), original...), []byte(`{}`)...)
			case "http":
				copy.MetadataURL = "http://metadata.example.test"
			case "credentials":
				copy.ArtifactOrigin = "https://user:secret@artifact.example.test"
			case "query":
				copy.MetadataURL += "?"
			case "origin path":
				copy.ArtifactOrigin += "/artifacts"
			case "unknown channel":
				copy.Channel = "nightly"
			case "unknown baseline":
				copy.Allowlist[0].UpstreamCommit = strings.Repeat("f", 40)
			case "unknown arch":
				copy.Allowlist[0].Arch = "x64"
			case "duplicate allowlist":
				copy.Allowlist = append(copy.Allowlist, copy.Allowlist[0])
			case "bad root":
				copy.Root = []byte(`{}`)
			case "oversize":
				data = bytes.Repeat([]byte(" "), maxTrustBytes+1)
			}
			if scenario != "unknown" && scenario != "duplicate" && scenario != "alias" && scenario != "nested alias" && scenario != "missing" && scenario != "trailing" && scenario != "oversize" {
				data = jsonBytes(t, copy)
			}
			if _, err := parseTrust(data); err == nil {
				t.Fatal("invalid installed trust accepted")
			}
		})
	}
}

func TestLauncherStateAndInventoryFailures(t *testing.T) {
	f := newLaunchFixture(t)
	directory := fixtureRoot + "/generations/" + fixtureGeneration
	for _, scenario := range []string{"owner", "generation", "stock relative", "duplicate key", "oversize"} {
		t.Run(scenario, func(t *testing.T) {
			s := f.state
			var data []byte
			switch scenario {
			case "owner":
				s.Owner = "foreign"
			case "generation":
				s.Current = "../bad"
			case "stock relative":
				s.Stock.Path = "opencpn.exe"
			}
			data = jsonBytes(t, s)
			if scenario == "duplicate key" {
				data = append([]byte(`{"current":"`+fixtureGeneration+`",`), data[1:]...)
			}
			if scenario == "oversize" {
				data = bytes.Repeat([]byte(" "), 32769)
			}
			if _, err := parseState(data); err == nil {
				t.Fatal("bad state accepted")
			}
		})
	}
	original := f.p.files[directory+"/ownership.json"]
	o, _, err := parseOwnership(original, directory)
	if err != nil {
		t.Fatal(err)
	}
	for _, scenario := range []string{"duplicate inventory", "unmanaged helper", "conflicting hash", "traversal", "non-status-only"} {
		t.Run(scenario, func(t *testing.T) {
			copy := o
			copy.Files = append(copy.Files[:0:0], copy.Files...)
			copy.Managed = append(copy.Managed[:0:0], copy.Managed...)
			switch scenario {
			case "duplicate inventory":
				copy.Files = append(copy.Files, copy.Files[0])
			case "unmanaged helper":
				copy.Managed[0].Path = "app/unowned.exe"
			case "conflicting hash":
				copy.Managed[0].SHA256 = strings.Repeat("f", 64)
			case "traversal":
				copy.Files[0].Path = "../app/opencpn.exe"
			case "non-status-only":
				copy.HardwarePolicy = "hardware-enabled"
			}
			if _, _, err := parseOwnership(jsonBytes(t, copy), directory); err == nil {
				t.Fatal("bad immutable inventory accepted")
			}
		})
	}
}

func TestLauncherPEArchitectureIsMeasured(t *testing.T) {
	good := testPE()
	if err := validatePE32(bytes.NewReader(good), int64(len(good))); err != nil {
		t.Fatal(err)
	}
	for _, scenario := range []string{"short", "dos", "offset", "signature", "x64", "PE32+"} {
		t.Run(scenario, func(t *testing.T) {
			b := append([]byte(nil), good...)
			switch scenario {
			case "short":
				b = b[:30]
			case "dos":
				b[0] = 0
			case "offset":
				binary.LittleEndian.PutUint32(b[60:], 0xffffffff)
			case "signature":
				b[64] = 0
			case "x64":
				binary.LittleEndian.PutUint16(b[68:], 0x8664)
			case "PE32+":
				binary.LittleEndian.PutUint16(b[88:], 0x20b)
			}
			if err := validatePE32(bytes.NewReader(b), int64(len(b))); err == nil {
				t.Fatal("non-x86 PE accepted")
			}
		})
	}
}

func TestVersionedManualControlOwnership(t *testing.T) {
	f := newLaunchFixture(t)
	directory := fixtureRoot + "/generations/" + fixtureGeneration
	o, _, err := parseOwnership(f.p.files[directory+"/ownership.json"], directory)
	if err != nil {
		t.Fatal(err)
	}
	o.HardwarePolicy = "manual-commissioning"
	for _, v := range []int{0, -1, 1, 2} {
		o.ManualControl = v
		_, _, e := parseOwnership(jsonBytes(t, o), directory)
		if (e == nil) != (v == 1) {
			t.Fatalf("control contract %d admitted=%v", v, e == nil)
		}
	}
	o.HardwarePolicy = "status-only"
	o.ManualControl = 1
	if _, _, e := parseOwnership(jsonBytes(t, o), directory); e == nil {
		t.Fatal("contradictory control capability")
	}
	o.HardwarePolicy = "manual-commissioning"
	original := jsonBytes(t, o)
	for _, value := range []string{`true`, `false`, `"1"`, `1.0`, `null`} {
		bad := bytes.Replace(original, []byte(`"xnavManualControlContract":1`), []byte(`"xnavManualControlContract":`+value), 1)
		if _, _, e := parseOwnership(bad, directory); e == nil {
			t.Fatalf("coerced control contract %s", value)
		}
	}
}
