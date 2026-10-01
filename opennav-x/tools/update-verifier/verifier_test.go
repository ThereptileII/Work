package verifier

import (
	"crypto"
	"crypto/ed25519"
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"path/filepath"
	"strings"
	"testing"
	"time"

	"github.com/sigstore/sigstore/pkg/signature"
	"github.com/theupdateframework/go-tuf/v2/metadata"
)

type fixture struct {
	files    map[string][]byte
	keys     map[string]ed25519.PrivateKey
	root     []byte
	server   *httptest.Server
	cache    string
	delay    time.Duration
	redirect string
}

func newFixture(t *testing.T) *fixture {
	t.Helper()
	f := &fixture{files: map[string][]byte{}, keys: map[string]ed25519.PrivateKey{}, cache: filepath.Join(t.TempDir(), "cache")}
	root := metadata.Root(time.Now().Add(365 * 24 * time.Hour))
	root.Signed.ConsistentSnapshot = false
	for _, role := range []string{"root", "timestamp", "snapshot", "targets"} {
		_, priv, err := ed25519.GenerateKey(nil)
		if err != nil {
			t.Fatal(err)
		}
		f.keys[role] = priv
		key, err := metadata.KeyFromPublicKey(priv.Public())
		if err != nil {
			t.Fatal(err)
		}
		if err := root.Signed.AddKey(key, role); err != nil {
			t.Fatal(err)
		}
	}
	sign(t, root, f.keys["root"])
	f.root = bytesOf(t, root)
	f.files["/1.root.json"] = f.root
	f.server = httptest.NewTLSServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		if r.URL.Path == "/timestamp.json" && f.redirect != "" {
			http.Redirect(w, r, f.redirect, http.StatusFound)
			return
		}
		if r.URL.Path == "/timestamp.json" && f.delay > 0 {
			time.Sleep(f.delay)
		}
		b, ok := f.files[r.URL.Path]
		if !ok {
			http.NotFound(w, r)
			return
		}
		_, _ = w.Write(b)
	}))
	t.Cleanup(f.server.Close)
	f.publish(t, 1, "beta", []byte(`{"package":"signed"}`), false)
	return f
}

func sign[T metadata.Roles](t *testing.T, m *metadata.Metadata[T], key ed25519.PrivateKey) {
	t.Helper()
	signer, err := signature.LoadSigner(key, crypto.Hash(0))
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.Sign(signer); err != nil {
		t.Fatal(err)
	}
}
func bytesOf[T metadata.Roles](t *testing.T, m *metadata.Metadata[T]) []byte {
	t.Helper()
	b, err := m.ToBytes(false)
	if err != nil {
		t.Fatal(err)
	}
	return b
}

func (f *fixture) publish(t *testing.T, version int64, channel string, payload []byte, expired bool, changes ...func(*metadata.TargetFiles)) {
	t.Helper()
	future := time.Now().Add(24 * time.Hour)
	targets := metadata.Targets(future)
	targets.Signed.Version = version
	path := "releases/" + channel + ".json"
	tf, err := metadata.TargetFile().FromBytes(path, payload, "sha256")
	if err != nil {
		t.Fatal(err)
	}
	custom := json.RawMessage(`{"channel":"` + channel + `","version":"2.0.0","commit":"` + strings.Repeat("a", 40) + `"}`)
	tf.Custom = &custom
	for _, change := range changes {
		change(tf)
	}
	targets.Signed.Targets[path] = tf
	sign(t, targets, f.keys["targets"])
	snapshot := metadata.Snapshot(future)
	snapshot.Signed.Version = version
	snapshot.Signed.Meta["targets.json"] = metadata.MetaFile(version)
	sign(t, snapshot, f.keys["snapshot"])
	timestampExpiry := future
	if expired {
		timestampExpiry = time.Now().Add(-time.Hour)
	}
	timestamp := metadata.Timestamp(timestampExpiry)
	timestamp.Signed.Version = version
	timestamp.Signed.Meta["snapshot.json"] = metadata.MetaFile(version)
	sign(t, timestamp, f.keys["timestamp"])
	f.files["/targets.json"] = bytesOf(t, targets)
	f.files["/snapshot.json"] = bytesOf(t, snapshot)
	f.files["/timestamp.json"] = bytesOf(t, timestamp)
	f.files["/targets/"+path] = payload
}

func (f *fixture) verify(channel string) (Release, []byte, error) {
	return verifyWithClient(f.request(channel), time.Now().Add(maxOperationTime), f.server.Client())
}
func (f *fixture) request(channel string) Request {
	return Request{MetadataURL: f.server.URL, TrustedRoot: f.root, CacheDir: f.cache, Channel: channel, TargetPath: "releases/" + channel + ".json"}
}

func TestSignedRepositoryAndTargetTamper(t *testing.T) {
	f := newFixture(t)
	release, data, err := f.verify("beta")
	if err != nil {
		t.Fatal(err)
	}
	if release.Channel != "beta" || release.Version != "2.0.0" || string(data) != `{"package":"signed"}` {
		t.Fatal("wrong signed result")
	}
	f.files["/targets/releases/beta.json"] = []byte(`{"package":"sinned"}`)
	if _, _, err := f.verify("beta"); err == nil || !strings.Contains(strings.ToLower(err.Error()), "hash") {
		t.Fatalf("same-size target hash failure expected: %v", err)
	}
	f.files["/targets/releases/beta.json"] = []byte(`x`)
	if _, _, err := f.verify("beta"); err == nil || !strings.Contains(strings.ToLower(err.Error()), "length") {
		t.Fatalf("wrong target length failure expected: %v", err)
	}
}

func TestInvalidMetadataAndExpiry(t *testing.T) {
	for _, tc := range []string{"signature", "malformed", "expired"} {
		t.Run(tc, func(t *testing.T) {
			f := newFixture(t)
			want := map[string]string{"signature": "not enough signatures", "malformed": "invalid character", "expired": "timestamp.json is expired"}[tc]
			switch tc {
			case "signature":
				f.files["/timestamp.json"] = []byte(strings.Replace(string(f.files["/timestamp.json"]), `"version":1`, `"version":9`, 1))
			case "malformed":
				f.files["/timestamp.json"] = []byte(`not-json`)
			case "expired":
				f.publish(t, 1, "beta", []byte(`{"package":"signed"}`), true)
			}
			if _, _, err := f.verify("beta"); err == nil || !strings.Contains(err.Error(), want) {
				t.Fatalf("invalid metadata accepted or wrong failure: %v", err)
			}
		})
	}
}

func TestRollbackAndMissingChannel(t *testing.T) {
	f := newFixture(t)
	if _, _, err := f.verify("stable"); err == nil || !strings.Contains(err.Error(), "target metadata") {
		t.Fatalf("missing stable target failure expected: %v", err)
	}
	f.publish(t, 2, "beta", []byte(`{"package":"newer"}`), false)
	if _, _, err := f.verify("beta"); err != nil {
		t.Fatal(err)
	}
	f.publish(t, 1, "beta", []byte(`{"package":"older"}`), false)
	if _, _, err := f.verify("beta"); err == nil || !strings.Contains(err.Error(), "new timestamp version 1 must be >= 2") {
		t.Fatalf("rollback failure expected: %v", err)
	}
}

func TestUnsignedIdentityRejected(t *testing.T) {
	f := newFixture(t)
	// A caller may not supply an unsigned version/commit or redirect beta to stable.
	if _, _, err := Verify(Request{MetadataURL: f.server.URL, TrustedRoot: f.root, CacheDir: f.cache, Channel: "stable", TargetPath: "releases/beta.json"}); err == nil {
		t.Fatal("channel mismatch accepted")
	}
}

func TestSignedIdentityAndSizePolicy(t *testing.T) {
	for _, tc := range []string{"wrong-channel", "invalid-commit", "oversize-target"} {
		t.Run(tc, func(t *testing.T) {
			f := newFixture(t)
			f.publish(t, 1, "beta", []byte(`{"package":"signed"}`), false, func(tf *metadata.TargetFiles) {
				switch tc {
				case "wrong-channel":
					raw := json.RawMessage(`{"channel":"stable","version":"2.0.0","commit":"` + strings.Repeat("a", 40) + `"}`)
					tf.Custom = &raw
				case "invalid-commit":
					raw := json.RawMessage(`{"channel":"beta","version":"2.0.0","commit":"unsigned"}`)
					tf.Custom = &raw
				case "oversize-target":
					tf.Length = maxTargetBytes + 1
				}
			})
			want := "invalid signed release identity"
			if tc == "oversize-target" {
				want = "target exceeds size limit"
			}
			if _, _, err := f.verify("beta"); err == nil || !strings.Contains(err.Error(), want) {
				t.Fatalf("expected %s: %v", want, err)
			}
		})
	}
}

func TestTransportDeadlineRedirectAndMetadataLimit(t *testing.T) {
	t.Run("deadline", func(t *testing.T) {
		f := newFixture(t)
		f.delay = 200 * time.Millisecond
		start := time.Now()
		_, _, err := verifyWithClient(f.request("beta"), time.Now().Add(20*time.Millisecond), f.server.Client())
		if err == nil || !strings.Contains(strings.ToLower(err.Error()), "deadline") || time.Since(start) > time.Second {
			t.Fatalf("deadline not enforced: %v", err)
		}
	})
	t.Run("redirect", func(t *testing.T) {
		f := newFixture(t)
		other := httptest.NewTLSServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) { w.WriteHeader(http.StatusOK) }))
		defer other.Close()
		f.redirect = other.URL + "/timestamp.json"
		if _, _, err := f.verify("beta"); err == nil || !strings.Contains(err.Error(), "outside HTTPS origin") {
			t.Fatalf("cross-origin redirect accepted: %v", err)
		}
	})
	t.Run("metadata-size", func(t *testing.T) {
		f := newFixture(t)
		f.files["/timestamp.json"] = []byte(strings.Repeat("x", (16<<10)+1))
		if _, _, err := f.verify("beta"); err == nil || !strings.Contains(strings.ToLower(err.Error()), "length") {
			t.Fatalf("oversize metadata accepted: %v", err)
		}
	})
}
