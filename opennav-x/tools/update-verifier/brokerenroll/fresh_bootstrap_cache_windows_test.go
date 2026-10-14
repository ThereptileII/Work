//go:build windows

package brokerenroll

import (
	"bytes"
	"context"
	"crypto/rand"
	"encoding/hex"
	"os"
	"path/filepath"
	"testing"

	"golang.org/x/sys/windows"
)

// Disposable protected children only; no service, installed customer cache,
// root rotation, profile or executable is initialized by these component tests.
func freshCacheNativeParent(t *testing.T) (*nativeEnrollmentFiles, *enrollmentPin) {
	t.Helper()
	if !windows.GetCurrentProcessToken().IsElevated() {
		t.Skip("fresh protected cache fixture requires elevation; not a native pass")
	}
	data, err := windows.KnownFolderPath(windows.FOLDERID_ProgramData, 0)
	if err != nil {
		t.Fatal(err)
	}
	var nonce [16]byte
	if _, err = rand.Read(nonce[:]); err != nil {
		t.Fatal(err)
	}
	name := "SKAGER-cache-seed-fixture-" + hex.EncodeToString(nonce[:])
	path := filepath.Join(data, name)
	t.Cleanup(func() {
		if err := os.RemoveAll(path); err != nil {
			t.Error(err)
		}
	})
	f := &nativeEnrollmentFiles{}
	t.Cleanup(func() {
		if err := f.Close(); err != nil {
			t.Error(err)
		}
	})
	parent, err := f.holdAncestry(data, true)
	if err != nil {
		t.Fatal(err)
	}
	sd, err := windows.SecurityDescriptorFromString("O:BAD:P(A;OICI;FA;;;SY)(A;OICI;FA;;;BA)")
	if err != nil {
		t.Fatal(err)
	}
	parent, err = f.newDirectory(parent, name, sd, "")
	if err != nil {
		t.Fatal(err)
	}
	return f, parent
}
func TestNativeFreshBootstrapCacheExactRootExclusiveCreation(t *testing.T) {
	_, parent := freshCacheNativeParent(t)
	ctx := context.Background()
	path := filepath.Join(parent.dos, "cache")
	owner, err := openFreshBootstrapCacheNative(ctx, path)
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { owner.close() })
	original := []byte("{ \"signed\": \"INERT exact original public root bytes\" }\n")
	if err = owner.seed(ctx, original); err != nil {
		t.Fatal(err)
	}
	if err = owner.check(ctx); err != nil {
		t.Fatal(err)
	}
	actual, err := os.ReadFile(filepath.Join(path, "metadata", "root.json"))
	if err != nil || !bytes.Equal(actual, original) {
		t.Fatal("root bytes were normalized or changed", err)
	}
	if err = owner.seed(ctx, []byte("replacement")); err == nil {
		t.Fatal("fresh owner seeded twice")
	}
	if err = owner.close(); err != nil {
		t.Fatal(err)
	}
	if next, e := openFreshBootstrapCacheNative(ctx, path); e == nil || next != nil {
		if next != nil {
			next.close()
		}
		t.Fatal("valid existing cache adopted as fresh")
	}
	actual, err = os.ReadFile(filepath.Join(path, "metadata", "root.json"))
	if err != nil || !bytes.Equal(actual, original) {
		t.Fatal("existing root overwritten", err)
	}
}
func TestNativeFreshBootstrapCacheRefusesExistingPartialRedirectedOrMutableParent(t *testing.T) {
	for _, kind := range []string{"empty", "partial", "symlink-cache", "symlink-parent", "mutable-parent"} {
		t.Run(kind, func(t *testing.T) {
			f, parent := freshCacheNativeParent(t)
			path := filepath.Join(parent.dos, "cache")
			original := []byte("PARTIAL ORIGINAL ROOT; PRESERVE")
			switch kind {
			case "empty":
				if err := os.Mkdir(path, 0700); err != nil {
					t.Fatal(err)
				}
			case "partial":
				if err := os.MkdirAll(filepath.Join(path, "metadata"), 0700); err != nil {
					t.Fatal(err)
				}
				if err := os.WriteFile(filepath.Join(path, "metadata", "root.json"), original, 0600); err != nil {
					t.Fatal(err)
				}
			case "symlink-cache", "symlink-parent":
				target := filepath.Join(parent.dos, "target")
				if err := os.Mkdir(target, 0700); err != nil {
					t.Fatal(err)
				}
				link := path
				if kind == "symlink-parent" {
					link = filepath.Join(parent.dos, "alias")
					path = filepath.Join(link, "cache")
				}
				p, _ := windows.UTF16PtrFromString(link)
				q, _ := windows.UTF16PtrFromString(target)
				if err := windows.CreateSymbolicLink(p, q, windows.SYMBOLIC_LINK_FLAG_DIRECTORY); err != nil {
					t.Fatal("elevated inert reparse fixture unavailable", err)
				}
			case "mutable-parent":
				sd, err := windows.SecurityDescriptorFromString("O:BAD:P(A;OICI;FA;;;SY)(A;OICI;FA;;;BA)(A;OICI;FW;;;WD)")
				if err != nil {
					t.Fatal(err)
				}
				mutable, err := f.newDirectory(parent, "mutable", sd, "")
				if err != nil {
					t.Fatal(err)
				}
				path = filepath.Join(mutable.dos, "cache")
			}
			owner, err := openFreshBootstrapCacheNative(context.Background(), path)
			if err == nil || owner != nil {
				if owner != nil {
					owner.close()
				}
				t.Fatal("unproven cache/parent adopted", kind)
			}
			if kind == "partial" {
				actual, e := os.ReadFile(filepath.Join(path, "metadata", "root.json"))
				if e != nil || !bytes.Equal(actual, original) {
					t.Fatal("partial cache reset or repaired", e)
				}
			}
			if kind == "mutable-parent" {
				if _, e := os.Lstat(path); !os.IsNotExist(e) {
					t.Fatal("cache created under untrusted parent", e)
				}
			}
		})
	}
}
