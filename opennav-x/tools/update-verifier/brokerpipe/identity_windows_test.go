//go:build windows

package brokerpipe

import (
	"testing"
	"unsafe"

	"golang.org/x/sys/windows"
)

func TestNativeABIAndNoClientInstanceRight(t *testing.T) {
	if unsafe.Sizeof(tokenStatistics{}) != 56 || unsafe.Offsetof(tokenStatistics{}.AuthenticationID) != 8 || unsafe.Offsetof(tokenStatistics{}.ModifiedID) != 48 || unsafe.Sizeof(fileID{}) != 24 {
		t.Fatal("native token/file identity layout mismatch")
	}
	if clientPipeRights&windows.FILE_APPEND_DATA != 0 || clientPipeRights&(windows.GENERIC_ALL|windows.GENERIC_WRITE) != 0 {
		t.Fatal("client may create pipe instances")
	}
}
func TestProtectedDescriptorRefusesUserControlAndUnknownOwner(t *testing.T) {
	mask := uint32(windows.DELETE | windows.WRITE_DAC | windows.WRITE_OWNER | windows.FILE_WRITE_DATA | windows.FILE_APPEND_DATA | fileDeleteChild | windows.GENERIC_ALL | windows.GENERIC_WRITE)
	for _, test := range []struct {
		name, sddl string
		allow      bool
	}{
		{"admin protected", `O:BAD:P(A;;FA;;;BA)(A;;FA;;;SY)(A;;FR;;;IU)`, true},
		{"user owns", `O:S-1-5-21-1-2-3-1001D:P(A;;FR;;;IU)`, false},
		{"user write DAC", `O:BAD:P(A;;WD;;;IU)`, false},
		{"user generic all", `O:BAD:P(A;;GA;;;IU)`, false},
		{"user delete child", `O:BAD:P(A;;0x40;;;IU)`, false},
		{"deny does not excuse allow", `O:BAD:P(D;;WD;;;IU)(A;;WD;;;IU)`, false},
		{"inherit only not effective", `O:BAD:P(A;IO;GA;;;IU)(A;;FR;;;IU)`, true},
		{"null DACL", `O:BA`, false},
	} {
		t.Run(test.name, func(t *testing.T) {
			sd, e := windows.SecurityDescriptorFromString(test.sddl)
			if e != nil {
				t.Fatal(e)
			}
			if got := protectedDescriptor(sd, mask) == nil; got != test.allow {
				t.Fatalf("allow=%v expected %v", got, test.allow)
			}
		})
	}
}

func TestPhysicalAncestorsCannotHideBehindDosAlias(t *testing.T) {
	good := `\Device\HarddiskVolume12\Program Files (x86)\SKAGER\UpdateBroker\skager-update-broker.exe`
	parents, e := physicalParents(good)
	if e != nil || len(parents) != 4 || parents[0] != `\Device\HarddiskVolume12` || parents[3] != `\Device\HarddiskVolume12\Program Files (x86)\SKAGER\UpdateBroker` {
		t.Fatal(parents, e)
	}
	for _, name := range []string{`C:\Program Files (x86)\SKAGER\UpdateBroker\skager-update-broker.exe`, `\Device\Mup\server\share\broker.exe`, `\Device\HarddiskVolumeX\x.exe`, `\Device\HarddiskVolume1\..\x.exe`, `\Device\HarddiskVolume1\dir.\x.exe`, `\Device\HarddiskVolume1\dir \x.exe`, `\Device\HarddiskVolume1\x.exe:stream`} {
		if _, e := physicalParents(name); e == nil {
			t.Fatal("accepted unsupported physical path", name)
		}
	}
}
