// skager-repository is an explicit offline operator tool. It never generates
// signing keys, uploads data, changes hosting, or runs the application.
package main

import (
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"errors"
	"flag"
	"fmt"
	"io"
	"os"
	"path/filepath"
	"time"

	"example.com/opennav-update-verifier/repository"
)

func plainRead(name string, limit int64) ([]byte, error) {
	full, e := filepath.Abs(name)
	if e != nil {
		return nil, e
	}
	for p := full; ; p = filepath.Dir(p) {
		st, e := os.Lstat(p)
		if e != nil {
			return nil, e
		}
		if st.Mode()&os.ModeSymlink != 0 {
			return nil, errors.New("link refused")
		}
		if p == filepath.Dir(p) {
			break
		}
	}
	f, e := os.Open(full)
	if e != nil {
		return nil, e
	}
	defer f.Close()
	st, e := f.Stat()
	if e != nil || !st.Mode().IsRegular() || st.Size() < 1 || st.Size() > limit {
		return nil, errors.New("file refused")
	}
	return io.ReadAll(io.LimitReader(f, limit+1))
}
func run(args []string, out io.Writer) error {
	if len(args) < 1 {
		return errors.New("command required")
	}
	command := args[0]
	fs := flag.NewFlagSet(command, flag.ContinueOnError)
	fs.SetOutput(io.Discard)
	var repo, config, metadataURL, origin, assets, policy, selection, pin, keyFile string
	var keyStdin bool
	switch command {
	case "validate-trust":
		fs.StringVar(&config, "config", "", "")
	case "initialize":
		fs.StringVar(&repo, "repository", "", "")
		fs.StringVar(&metadataURL, "metadata-url", "", "")
		fs.StringVar(&origin, "artifact-origin", "", "")
	case "export-public-root", "export-public-config":
		fs.StringVar(&repo, "repository", "", "")
	case "publish":
		fs.StringVar(&repo, "repository", "", "")
		fs.StringVar(&assets, "assets", "", "")
		fs.StringVar(&policy, "policy", "", "")
		fs.StringVar(&selection, "selection", "", "")
		fs.StringVar(&pin, "selection-sha256", "", "")
	default:
		return errors.New("unsupported command")
	}
	mutation := command == "initialize" || command == "publish"
	if mutation {
		fs.BoolVar(&keyStdin, "keys-stdin", false, "")
		fs.StringVar(&keyFile, "keys-file", "", "")
	}
	if e := fs.Parse(args[1:]); e != nil || fs.NArg() != 0 {
		return errors.New("arguments refused")
	}
	now := time.Now().UTC()
	if command == "validate-trust" {
		b, e := plainRead(config, 576<<10)
		if e != nil {
			return e
		}
		if _, e = repository.ValidateTrust(b, now); e != nil {
			return e
		}
		h := sha256.Sum256(b)
		return json.NewEncoder(out).Encode(map[string]any{"schema": 1, "status": "valid", "configSha256": hex.EncodeToString(h[:]), "channel": "beta"})
	}
	if repo == "" {
		return errors.New("repository required")
	}
	if !mutation {
		kind := "root"
		if command == "export-public-config" {
			kind = "config"
		}
		b, e := repository.Export(repo, kind, now)
		if e != nil {
			return e
		}
		_, e = out.Write(b)
		return e
	}
	if keyStdin == (keyFile != "") {
		return errors.New("select exactly one private key input")
	}
	var input io.Reader = os.Stdin
	if keyFile != "" {
		st, e := os.Lstat(keyFile)
		if e != nil || st.Mode().Perm()&0077 != 0 {
			return errors.New("key file must be private")
		}
		b, e := plainRead(keyFile, 16384)
		if e != nil {
			return e
		}
		defer clear(b)
		input = &byteReader{b: b}
	}
	keys, e := repository.ReadKeys(input, command == "initialize")
	if e != nil {
		return e
	}
	defer keys.Clear()
	if command == "initialize" {
		if e = repository.Initialize(repo, metadataURL, origin, keys, now); e != nil {
			return e
		}
		return json.NewEncoder(out).Encode(map[string]any{"schema": 1, "status": "initialized", "channel": "beta"})
	}
	s, e := plainRead(selection, 1<<20)
	if e != nil {
		return e
	}
	p, e := plainRead(policy, 32768)
	if e != nil {
		return e
	}
	v, e := repository.Publish(repo, assets, s, p, pin, keys, now)
	if e != nil {
		return e
	}
	return json.NewEncoder(out).Encode(map[string]any{"schema": 1, "status": "published", "channel": "beta", "metadataVersion": v})
}

type byteReader struct{ b []byte }

func (r *byteReader) Read(p []byte) (int, error) {
	if len(r.b) == 0 {
		return 0, io.EOF
	}
	n := copy(p, r.b)
	r.b = r.b[n:]
	return n, nil
}
func main() {
	if e := run(os.Args[1:], os.Stdout); e != nil {
		fmt.Fprintln(os.Stderr, "Repository operation refused; retain state and inspect the selected inputs. No secret material is logged.")
		os.Exit(1)
	}
}
