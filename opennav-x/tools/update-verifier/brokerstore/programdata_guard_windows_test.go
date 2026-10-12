package brokerstore

import (
	"errors"
	"os"
	"path/filepath"
	"strings"
	"testing"

	"golang.org/x/sys/windows"
)

func TestProgramDataGuardNarrowDescriptorException(t *testing.T) {
	trusted := map[string]bool{"S-1-5-18": true}
	for _, tc := range []struct {
		name, mask string
		accepted   bool
	}{
		{"add plus EA and attributes", "0x116", true},
		{"delete child", "0x156", false},
		{"delete", "0x10116", false},
		{"write DAC", "0x50116", false},
		{"generic write", "0x40000116", false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			sd, err := windows.SecurityDescriptorFromString("O:SYD:(A;;FA;;;SY)(A;;" + tc.mask + ";;;BU)")
			if err != nil {
				t.Fatal(err)
			}
			ordinary := &ancestorPolicy{trusted: trusted}
			if err := checkAncestorDescriptor(sd, ordinary); err == nil {
				t.Fatal("ordinary ancestor accepted untrusted metadata mutation")
			}
			guarded := &ancestorPolicy{trusted: trusted, programData: &programDataGuard{}}
			if got := checkAncestorDescriptor(sd, guarded); (got == nil) != tc.accepted {
				t.Fatalf("guarded accepted=%v, want %v: %v", got == nil, tc.accepted, got)
			}
			guarded.forbidChildAdds = true
			if err := checkAncestorDescriptor(sd, guarded); err == nil {
				t.Fatal("strict protected parent accepted child-add rights")
			}
		})
	}
}

func TestProgramDataGuardRetainsOriginalPlainChild(t *testing.T) {
	root := filepath.Join(t.TempDir(), "Data")
	menu := filepath.Join(root, "Machine", "Windows", "Start Menu")
	if err := os.MkdirAll(menu, 0700); err != nil {
		t.Fatal(err)
	}
	parent, err := openPin(root, "", nil)
	if err != nil {
		t.Fatal(err)
	}
	parent.dos = root
	guard, held, err := retainProgramDataGuard(parent, root, menu)
	if err != nil {
		windows.CloseHandle(parent.handle)
		t.Fatal(err)
	}
	t.Cleanup(func() {
		var closeErr error
		for i := len(held) - 1; i >= 0; i-- {
			closeErr = errors.Join(closeErr, windows.CloseHandle(held[i].handle))
		}
		closeErr = errors.Join(closeErr, windows.CloseHandle(parent.handle))
		if closeErr != nil {
			t.Error(closeErr)
		}
	})
	if len(held) != 3 || guard.check(parent.handle) != nil {
		t.Fatal("original existing descendant was not retained")
	}
	if err := os.Rename(filepath.Join(root, "Machine"), filepath.Join(root, "Renamed")); err == nil {
		t.Fatal("retained immediate child could be renamed")
	}
	old := guard.children[0].id
	guard.children[0].id.FileID = strings.Repeat("f", 32)
	if guard.check(parent.handle) == nil {
		t.Fatal("changed full child identity accepted")
	}
	guard.children[0].id = old
	if err := guard.check(parent.handle); err != nil {
		t.Fatal(err)
	}
}

func TestProgramDataGuardRefusesMissingAndUnrelatedDescendant(t *testing.T) {
	base := t.TempDir()
	root := filepath.Join(base, "Data")
	other := filepath.Join(base, "DataOther")
	if err := os.Mkdir(root, 0700); err != nil {
		t.Fatal(err)
	}
	if err := os.Mkdir(other, 0700); err != nil {
		t.Fatal(err)
	}
	if err := os.Mkdir(filepath.Join(root, "Existing"), 0700); err != nil {
		t.Fatal(err)
	}
	parent, err := openPin(root, "", nil)
	if err != nil {
		t.Fatal(err)
	}
	defer windows.CloseHandle(parent.handle)
	for _, menu := range []string{filepath.Join(root, "Missing", "Start Menu"), filepath.Join(root, "Existing", "Missing"), filepath.Join(other, "Start Menu"), root} {
		guard, owned, err := retainProgramDataGuard(parent, root, menu)
		if err == nil || guard != nil || len(owned) != 0 {
			t.Fatalf("unrelated or missing stock descendant accepted: %q", menu)
		}
	}
	// A partial descendant open must release the first original handle even
	// when a deeper child is missing; otherwise this rename is share-blocked.
	if err := os.Rename(filepath.Join(root, "Existing"), filepath.Join(root, "Renamed")); err != nil {
		t.Fatal("partial guard leaked its original child handle:", err)
	}
}
