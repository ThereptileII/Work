package brokerstore

import (
	"bytes"
	"context"
	"errors"
	"os"
	"path/filepath"
	"runtime"
	"testing"

	"golang.org/x/sys/windows"
)

// These current-user temporary namespaces exercise the actual Win32 store
// custody and terminal file mechanics. They never open the product namespace,
// provision a service, touch profiles, or manufacture client/broker authority.
func nativeDecommissionFixture(t *testing.T) (*nativeStore, DecommissionBinding, []byte) {
	t.Helper()
	n, r := nativeRegistrationFixture(t)
	if err := beginRegistration(context.Background(), r); err != nil {
		t.Fatal(err)
	}
	createNativeTestChannel(t, r)
	raw := []byte(`{"schema":1,"inertDecommissionFixture":true}`)
	if err := r.complete(context.Background(), raw); err != nil {
		t.Fatal(err)
	}
	if err := r.close(); err != nil {
		t.Fatal(err)
	}
	binding := decommissionTestBinding()
	binding.ClientSID = r.client
	binding.RegistrationSHA256 = registrationDigest(raw)
	return n, binding, raw
}
func TestNativeDecommissionExactOriginalReceiptAndPermanentOrdinaryDenial(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	n, binding, raw := nativeDecommissionFixture(t)
	d, err := openDecommissionInventory(context.Background(), n, binding)
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() {
		if err := d.close(); err != nil {
			t.Error(err)
		}
	})
	// The same original completion and registration handles exclude mutation.
	if err = os.WriteFile(filepath.Join(n.rootDOS, "registrations", binding.ClientSID+".json"), []byte("changed"), 0600); !errors.Is(err, windows.ERROR_SHARING_VIOLATION) {
		t.Fatalf("original registration writable: %v", err)
	}
	if err = d.commit(context.Background()); err != nil {
		t.Fatal(err)
	}
	if err = d.check(); err != nil {
		t.Fatal("original terminal custody", err)
	}
	if err = n.check(); !errors.Is(err, ErrDecommissioned) {
		t.Fatalf("ordinary native check: %v", err)
	}
	// Ordinary refusal must not invalidate the retained exact terminal receipt.
	if err = d.check(); err != nil {
		t.Fatal("ordinary denial poisoned receipt", err)
	}
	if err = d.commit(context.Background()); err == nil {
		t.Fatal("terminal native writer retried")
	}
	if err = d.close(); err != nil {
		t.Fatal(err)
	}
	actual, err := os.ReadFile(filepath.Join(n.rootDOS, decommissionFilename))
	if err != nil || !bytes.Equal(actual, decommissionBytes(binding)) {
		t.Fatalf("durable exact binding: %q %v", actual, err)
	}
	if err = os.Remove(filepath.Join(n.rootDOS, decommissionFilename)); err != nil {
		t.Fatal("fixture-only fence removal", err)
	}
	if err = n.check(); !errors.Is(err, ErrDecommissioned) {
		t.Fatal("original store resumed after observed terminal fence", err)
	}
	actual, err = os.ReadFile(filepath.Join(n.rootDOS, "registrations", binding.ClientSID+".json"))
	if err != nil || !bytes.Equal(actual, raw) {
		t.Fatal("original registration changed", err)
	}
}
func TestNativeDecommissionAnyPresenceRefusesWithoutParsing(t *testing.T) {
	for _, kind := range []string{"empty", "malformed", "directory", "held-exclusive"} {
		t.Run(kind, func(t *testing.T) {
			runtime.LockOSThread()
			defer runtime.UnlockOSThread()
			n, _ := nativeRegistrationFixture(t)
			path := filepath.Join(n.rootDOS, decommissionFilename)
			switch kind {
			case "empty":
				if err := os.WriteFile(path, nil, 0600); err != nil {
					t.Fatal(err)
				}
			case "malformed":
				if err := os.WriteFile(path, []byte("partial\x00garbage"), 0600); err != nil {
					t.Fatal(err)
				}
			case "directory":
				if err := os.Mkdir(path, 0700); err != nil {
					t.Fatal(err)
				}
			case "held-exclusive":
				sd, err := descriptor(n.broker, nil, false)
				if err != nil {
					t.Fatal(err)
				}
				held, err := createRegistrationFile(n.root, decommissionFilename, sd)
				if err != nil {
					t.Fatal(err)
				}
				defer held.Close()
			}
			if err := decommissionAbsent(n.root); !errors.Is(err, ErrDecommissioned) {
				t.Fatalf("presence accepted: %v", err)
			}
			if err := n.check(); !errors.Is(err, ErrDecommissioned) {
				t.Fatalf("store resumed: %v", err)
			}
		})
	}
}
func TestNativeDecommissionInventoryRefusesOtherClientPartialAndChangedDigest(t *testing.T) {
	for _, kind := range []string{"extra", "partial", "digest", "missing"} {
		t.Run(kind, func(t *testing.T) {
			runtime.LockOSThread()
			defer runtime.UnlockOSThread()
			n, binding, _ := nativeDecommissionFixture(t)
			registrationRoot := filepath.Join(n.rootDOS, "registrations")
			switch kind {
			case "extra":
				if err := os.WriteFile(filepath.Join(registrationRoot, "unknown.once"), nil, 0600); err != nil {
					t.Fatal(err)
				}
			case "partial":
				if err := os.WriteFile(filepath.Join(registrationRoot, binding.ClientSID+".once"), []byte("started"), 0600); err != nil {
					t.Fatal(err)
				}
			case "digest":
				binding.RegistrationSHA256 = registrationDigest([]byte("wrong"))
			case "missing":
				if err := os.Remove(filepath.Join(registrationRoot, binding.ClientSID+".json")); err != nil {
					t.Fatal(err)
				}
			}
			d, err := openDecommissionInventory(context.Background(), n, binding)
			if d != nil || err == nil {
				if d != nil {
					d.close()
				}
				t.Fatal("unowned inventory accepted", err)
			}
			if err = decommissionAbsent(n.root); err != nil {
				t.Fatal("preflight created terminal evidence", err)
			}
		})
	}
}
