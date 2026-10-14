//go:build windows

package brokerstore

import "testing"

// Inert file owners exercise the exact native-source lifetime drain even when
// baseline.openMembers returns only a partial map alongside its error. Native
// Windows descriptor/ID/source acquisition remains a separate acceptance gate.
func TestNativeRepairSourceDrainsPartialMembersExactlyOnce(t *testing.T) {
	for _, count := range []int{0, 1, 3} {
		owner := &nativeRepairSource{}
		files := map[string]candidateFile{}
		originals := []*memoryFile{}
		for i := 0; i < count; i++ {
			f := &memoryFile{}
			files[string(rune('a'+i))] = f
			originals = append(originals, f)
		}
		owner.retainMembers(files)
		if err := owner.close(); err != nil {
			t.Fatal(err)
		}
		if err := owner.close(); err != nil {
			t.Fatal(err)
		}
		for _, f := range originals {
			if f.closes != 1 {
				t.Fatal("partial source file leaked or double closed", f.closes)
			}
		}
	}
}
