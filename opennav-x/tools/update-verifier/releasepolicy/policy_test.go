package releasepolicy

import (
	"encoding/json"
	"strings"
	"testing"
)

var hash = strings.Repeat("a", 64)
var commit = strings.Repeat("b", 40)
var otherCommit = strings.Repeat("c", 40)

func validPolicy() Policy {
	artifact := func(name string) Artifact {
		return Artifact{Path: "releases/" + name, URL: "https://updates.skager.app/releases/" + name, SHA256: hash, Bytes: 1024}
	}
	return Policy{Schema: 1, Product: "skager", Channel: "beta", Version: "0.4.0-beta2", Commit: commit,
		ReleaseNotes: artifact("notes.md"), Installer: artifact("setup.exe"), Recovery: artifact("recovery.zip"), CorrespondingSource: artifact("source.tar.gz"), Notices: artifact("notices.txt"),
		SupportedOpenCpn: []OpenCpn{{Version: "5.12.4", Arch: "x86", ExecutableSHA256: hash, UpstreamCommit: otherCommit}}}
}
func parsePolicy(t *testing.T, p Policy) (Policy, error) {
	t.Helper()
	b, err := json.Marshal(p)
	if err != nil {
		t.Fatal(err)
	}
	return Parse(b)
}

func TestParseStrictJSONAndArtifacts(t *testing.T) {
	p := validPolicy()
	if _, err := parsePolicy(t, p); err != nil {
		t.Fatal(err)
	}
	raw, _ := json.Marshal(p)
	for _, b := range [][]byte{append(raw, []byte(" true")...), []byte(`{"schema":1,"schema":1}`), []byte(strings.Replace(string(raw), `"bytes":1024`, `"bytes":1024,"bytes":1024`, 1)), []byte(strings.Replace(string(raw), `"product":"skager"`, `"product":"skager","extra":1`, 1)), append([]byte(nil), 0xff)} {
		if _, err := Parse(b); err == nil {
			t.Fatalf("accepted invalid JSON %q", string(b))
		}
	}
	changes := []func(*Policy){
		func(p *Policy) { p.Schema = 2 }, func(p *Policy) { p.Product = "other" }, func(p *Policy) { p.Channel = "stable" },
		func(p *Policy) { p.Installer.Path = "../setup.exe" }, func(p *Policy) { p.Installer.Path = `C:\setup.exe` },
		func(p *Policy) { p.Installer.URL = "http://updates.skager.app/releases/setup.exe" }, func(p *Policy) { p.Installer.URL = "https://user:pass@updates.skager.app/releases/setup.exe" },
		func(p *Policy) { p.Installer.URL = "https://updates.skager.app/releases/other.exe" }, func(p *Policy) { p.Installer.SHA256 = "bad" },
		func(p *Policy) { p.Installer.Bytes = 0 }, func(p *Policy) { p.Installer.Bytes = maxArtifactBytes + 1 },
		func(p *Policy) { p.SupportedOpenCpn[0].Arch = "x64" }, func(p *Policy) { p.SupportedOpenCpn[0].ExecutableSHA256 = "bad" },
	}
	for i, change := range changes {
		q := validPolicy()
		change(&q)
		if _, err := parsePolicy(t, q); err == nil {
			t.Fatalf("accepted invalid policy case %d", i)
		}
	}
	if _, err := Parse([]byte(strings.Repeat(" ", maxPolicyBytes+1))); err == nil {
		t.Fatal("oversize policy accepted")
	}
}
func TestEvaluate(t *testing.T) {
	p := validPolicy()
	c := p.SupportedOpenCpn[0]
	installed := Installed{Version: "0.3.0", Commit: otherCommit}
	decision, err := Evaluate(p, installed, "beta", c, []OpenCpn{c})
	if err != nil || decision != Upgrade {
		t.Fatalf("upgrade: %s %v", decision, err)
	}
	cases := []struct {
		name   string
		mutate func(*Policy, *Installed, *OpenCpn, *[]OpenCpn, *string)
		want   Decision
	}{
		{"current", func(p *Policy, i *Installed, c *OpenCpn, l *[]OpenCpn, s *string) {
			i.Version = p.Version
			i.Commit = p.Commit
		}, Current},
		{"downgrade", func(p *Policy, i *Installed, c *OpenCpn, l *[]OpenCpn, s *string) { i.Version = "0.5.0" }, Downgrade},
		{"same precedence different commit", func(p *Policy, i *Installed, c *OpenCpn, l *[]OpenCpn, s *string) { i.Version = "0.4.0-beta2+build" }, VersionConflict},
		{"channel mismatch", func(p *Policy, i *Installed, c *OpenCpn, l *[]OpenCpn, s *string) { *s = "stable" }, ChannelMismatch},
		{"wrong exe hash", func(p *Policy, i *Installed, c *OpenCpn, l *[]OpenCpn, s *string) {
			c.ExecutableSHA256 = strings.Repeat("d", 64)
		}, Incompatible},
		{"wrong version", func(p *Policy, i *Installed, c *OpenCpn, l *[]OpenCpn, s *string) { c.Version = "5.12.5" }, Incompatible},
		{"wrong arch", func(p *Policy, i *Installed, c *OpenCpn, l *[]OpenCpn, s *string) { c.Arch = "x64" }, Incompatible},
		{"wrong upstream", func(p *Policy, i *Installed, c *OpenCpn, l *[]OpenCpn, s *string) {
			c.UpstreamCommit = strings.Repeat("d", 40)
		}, Incompatible},
		{"missing local allowlist", func(p *Policy, i *Installed, c *OpenCpn, l *[]OpenCpn, s *string) { *l = nil }, Incompatible},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			q := validPolicy()
			i := installed
			o := c
			local := []OpenCpn{c}
			s := "beta"
			tc.mutate(&q, &i, &o, &local, &s)
			got, e := Evaluate(q, i, s, o, local)
			if e != nil || got != tc.want {
				t.Fatalf("got %s, %v; want %s", got, e, tc.want)
			}
		})
	}
}
