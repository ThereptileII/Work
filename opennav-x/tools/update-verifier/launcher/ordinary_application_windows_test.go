package launcher

import (
	"errors"
	"os"
	"strings"
	"testing"
)

func TestNativeOrdinaryDOSApplicationBounds(t *testing.T) {
	physical := `\Device\HarddiskVolume7\parent\app\opencpn.exe`
	for _, name := range []string{`C:\parent\app\opencpn.exe`, `\\?\C:\parent\app\opencpn.exe`, `c:/parent/app/opencpn.exe`} {
		got, err := ordinaryApplicationPath(name, physical)
		if err != nil || !strings.EqualFold(got, `C:\parent\app\opencpn.exe`) {
			t.Fatalf("held DOS application rejected: %q %v", got, err)
		}
	}
	for _, name := range []string{
		`opencpn.exe`, `C:parent\app\opencpn.exe`, `C:\other\app\opencpn.exe`,
		`C:\parent\app\other.exe`, `C:\parent\app\..\opencpn.exe`, `C:\parent\app\opencpn.exe:stream`,
		`\\host\share\parent\app\opencpn.exe`, `\\.\C:\parent\app\opencpn.exe`,
		`\\?\UNC\host\share\parent\app\opencpn.exe`, `\\?\GLOBALROOT` + physical,
		`\\?\Volume{12345678-1234-1234-1234-123456789abc}\parent\app\opencpn.exe`,
		"C:\\parent\\app\\opencpn.exe\x00", "C:/" + strings.Repeat("a", 4096),
	} {
		if _, err := ordinaryApplicationPath(name, physical); err == nil {
			t.Fatal("unsupported DOS application accepted: " + name)
		}
	}
}

func TestNativeOrdinarySelectedApplicationCorruptionRefuses(t *testing.T) {
	spec, marker := ordinaryNativeFixture(t)
	image, err := acquireNativeOrdinaryImage(spec)
	if err != nil {
		t.Fatal(err)
	}
	owner := image.(*nativeOrdinaryImage)
	defer owner.Close()
	selected := owner.application
	owner.application += ".different"
	if startOrdinary(spec, func(processSpec) (ordinaryImage, error) { return owner, nil }) == nil {
		t.Fatal("corrupted selected application reached native creation")
	}
	if owner.application != selected+".different" {
		t.Fatal("selected application was silently rederived")
	}
	if _, err := os.Stat(marker); !errors.Is(err, os.ErrNotExist) {
		t.Fatal("refused selected application executed")
	}
}

func TestNativeOrdinaryWrongSuspendedImageRefuses(t *testing.T) {
	spec, marker := ordinaryNativeFixture(t)
	image, err := acquireNativeOrdinaryImage(spec)
	if err != nil {
		t.Fatal(err)
	}
	defer image.Close()
	otherSpec, otherMarker := ordinaryNativeFixture(t)
	otherImage, err := acquireNativeOrdinaryImage(otherSpec)
	if err != nil {
		t.Fatal(err)
	}
	defer otherImage.Close()
	// Deliberately bypass the pre-create recheck to exercise the independent
	// suspended-child check against a genuinely different copied inert image.
	image.(*nativeOrdinaryImage).application = otherImage.(*nativeOrdinaryImage).application
	created, err := image.create(spec)
	if err != nil {
		t.Fatal(err)
	}
	child := created.(*nativeOrdinaryChild)
	defer child.Close()
	if err := child.corroborate(); err == nil {
		_ = child.abortSuspended()
		t.Fatal("wrong native image name accepted before resume")
	}
	if err := child.abortSuspended(); err != nil {
		t.Fatal(err)
	}
	if child.resumeAttempted {
		t.Fatal("refused native image reached resume")
	}
	for _, name := range []string{marker, otherMarker} {
		if _, err := os.Stat(name); !errors.Is(err, os.ErrNotExist) {
			t.Fatal("refused native image executed")
		}
	}
}
