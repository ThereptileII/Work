//go:build linux

package main

import (
	"bytes"
	"crypto/ed25519"
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"os"
	"path/filepath"
	"testing"
	"time"

	"example.com/opennav-update-verifier/repository"
)

func TestPublicValidatorCLI(t *testing.T) {
	d := t.TempDir()
	keys := repository.Keys{}
	for _, role := range []string{"root", "targets", "snapshot", "timestamp"} {
		_, k, e := ed25519.GenerateKey(nil)
		if e != nil {
			t.Fatal(e)
		}
		keys[role] = k
	}
	defer keys.Clear()
	repo := filepath.Join(d, "repository")
	if e := repository.Initialize(repo, "https://fixture.invalid", "https://fixture.invalid", keys, time.Now()); e != nil {
		t.Fatal(e)
	}
	b, e := repository.Export(repo, "config", time.Now())
	if e != nil {
		t.Fatal(e)
	}
	config := filepath.Join(d, "public.json")
	if e = os.WriteFile(config, b, 0600); e != nil {
		t.Fatal(e)
	}
	var out bytes.Buffer
	if e = run([]string{"validate-trust", "--config", config}, &out); e != nil {
		t.Fatal(e)
	}
	hash := sha256.Sum256(b)
	expected := map[string]any{"schema": float64(1), "status": "valid", "configSha256": hex.EncodeToString(hash[:]), "channel": "beta"}
	var actual map[string]any
	if e = json.Unmarshal(out.Bytes(), &actual); e != nil {
		t.Fatal(e)
	}
	if len(actual) != len(expected) {
		t.Fatal("unexpected validation output")
	}
	for key, want := range expected {
		if actual[key] != want {
			t.Fatal("validation identity differs")
		}
	}
	original := append([]byte{}, b...)
	var c map[string]any
	json.Unmarshal(b, &c)
	c["privateKey"] = "DO_NOT_LOG"
	b, _ = json.Marshal(c)
	os.WriteFile(config, b, 0600)
	out.Reset()
	if e = run([]string{"validate-trust", "--config", config}, &out); e == nil || out.Len() != 0 {
		t.Fatal("unknown/private field accepted or leaked")
	}
	os.WriteFile(config, original, 0600)
	link := filepath.Join(d, "link.json")
	os.Symlink(config, link)
	if e = run([]string{"validate-trust", "--config", link}, &out); e == nil {
		t.Fatal("symlink input accepted")
	}
	out.Reset()
	if e = run([]string{"export-public-config", "--repository", repo}, &out); e != nil || !bytes.Equal(original, out.Bytes()) {
		t.Fatal("public export bytes differ")
	}
	if e = run([]string{"publish", "--repository", repo}, &out); e == nil {
		t.Fatal("implicit signing key input accepted")
	}
}
