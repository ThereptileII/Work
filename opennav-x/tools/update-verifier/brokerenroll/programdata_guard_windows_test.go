//go:build windows

package brokerenroll

import (
	"errors"
	"reflect"
	"testing"

	"golang.org/x/sys/windows"
)

func TestEnrollmentProgramDataGuardClosePreservesUncertainty(t *testing.T) {
	// NULL is invalid for CloseHandle; InvalidHandle (-1) is also the current
	// process pseudo-handle, whose close has no effect rather than failing.
	g := &enrollmentProgramDataGuard{children: []enrollmentGuardDirectory{{handle: windows.Handle(0)}}}
	err := g.Close()
	if err == nil || !errors.Is(err, ErrCleanup) || !errors.Is(err, windows.ERROR_INVALID_HANDLE) || g.Close() != err {
		t.Fatal("original guard cleanup uncertainty not retained idempotently")
	}
	if err := g.check(&enrollmentPin{}); err == nil {
		t.Fatal("uncertain closed guard retained policy exception")
	}
}

func TestEnrollmentProgramDataMenuRequiresCanonicalStrictDescendant(t *testing.T) {
	parts, err := enrollmentProgramDataComponents(`C:\SharedData`, `C:\SharedData\Vendor\Menu`)
	if err != nil || !reflect.DeepEqual(parts, []string{"Vendor", "Menu"}) {
		t.Fatal("independent known-folder component derivation differs:", err)
	}
	for _, menu := range []string{
		`C:\SharedData`, `C:\SharedDataOther\Menu`, `D:\SharedData\Menu`,
		`\\server\SharedData\Menu`, `C:\SharedData\..\Menu`, `C:\SharedData\Menu.`,
		`C:\SharedData\Menu `, `C:\SharedData\Menu:stream`, `C:\SharedData\\Menu`,
		`C:\SharedData\Menu\`, `C:/SharedData/Menu`,
	} {
		if _, err := enrollmentProgramDataComponents(`C:\SharedData`, menu); err == nil {
			t.Fatal("outside, aliased or ambiguous common menu accepted")
		}
	}
}

func TestEnrollmentProgramDataMetadataPolicyRemainsNarrow(t *testing.T) {
	sd, err := windows.SecurityDescriptorFromString("O:SYD:P(A;;FA;;;SY)(A;CI;0x116;;;BU)")
	if err != nil {
		t.Fatal(err)
	}
	if err := enrollmentSecurityDescriptor(sd, nil, true, ""); err == nil {
		t.Fatal("unguarded ancestor accepted metadata writes")
	}
	if err := enrollmentSecurityDescriptorPolicy(sd, nil, true, "", true); err != nil {
		t.Fatal("guarded ProgramData concrete metadata/create rights refused:", err)
	}
	if err := enrollmentSecurityDescriptorPolicy(sd, nil, false, "", true); err == nil {
		t.Fatal("strict ancestor policy relaxed by ProgramData flag")
	}
	for _, mask := range []string{"0x10116", "0x40116", "0x80116", "0x156", "GW", "GA"} {
		bad, err := windows.SecurityDescriptorFromString("O:SYD:P(A;;FA;;;SY)(A;CI;" + mask + ";;;BU)")
		if err != nil {
			t.Fatal(err)
		}
		if err := enrollmentSecurityDescriptorPolicy(bad, nil, true, "", true); err == nil {
			t.Fatal("guard excused delete, ACL/owner, delete-child or generic mutation")
		}
	}
	wanted, err := windows.SecurityDescriptorFromString("O:SYD:P(A;;FA;;;SY)")
	if err != nil {
		t.Fatal(err)
	}
	if err := enrollmentSecurityDescriptorPolicy(sd, wanted, true, "", true); err == nil {
		t.Fatal("guard relaxed exact protected child descriptor")
	}
	badOwner, err := windows.SecurityDescriptorFromString("O:BUD:P(A;CI;0x116;;;BU)")
	if err != nil {
		t.Fatal(err)
	}
	if err := enrollmentSecurityDescriptorPolicy(badOwner, nil, true, "", true); err == nil {
		t.Fatal("guard trusted untrusted parent owner")
	}
}

// Read-only actual machine known-folder capture. No machine entries are
// created, renamed, deleted or adopted, and no SCM or enrollment effects occur.
func TestNativeEnrollmentProgramDataGuardRetainsActualKnownFolderChain(t *testing.T) {
	data, err := windows.KnownFolderPath(windows.FOLDERID_ProgramData, windows.KF_FLAG_DEFAULT)
	if err != nil {
		t.Fatal(err)
	}
	files := &nativeEnrollmentFiles{}
	t.Cleanup(func() {
		if err := files.Close(); err != nil {
			t.Error("original read-only known-folder owners close:", err)
		}
	})
	parent, err := files.pinDirectory(data, "", data, nil, true, "")
	if err != nil {
		t.Fatal("actual ProgramData original guard capture:", err)
	}
	g := parent.programData
	if g == nil || len(g.children) == 0 || len(files.programDataGuards) != 1 {
		t.Fatal("actual ProgramData requires exactly one retained original guard owner")
	}
	if err := parent.check(); err != nil {
		t.Fatal(err)
	}
	for _, corrupt := range []func(*enrollmentPin){
		func(p *enrollmentPin) { p.id.ID[0] ^= 1 },
		func(p *enrollmentPin) { p.id.Volume ^= 1 },
		func(p *enrollmentPin) { p.physical += `\other` },
		func(p *enrollmentPin) { p.allowAdds = false },
	} {
		wrong := *parent
		corrupt(&wrong)
		if err := wrong.check(); err == nil {
			t.Fatal("guard transferred to different identity/parent or strict policy")
		}
	}
	original := g.children[0]
	for _, corrupt := range []func(*enrollmentGuardDirectory){
		func(p *enrollmentGuardDirectory) { p.id.ID[0] ^= 1 },
		func(p *enrollmentGuardDirectory) { p.id.Volume ^= 1 },
		func(p *enrollmentGuardDirectory) { p.physical += `\other` },
		func(p *enrollmentGuardDirectory) { p.handle = windows.InvalidHandle },
		func(p *enrollmentGuardDirectory) { p.dos = data },
	} {
		g.children[0] = original
		corrupt(&g.children[0])
		err := parent.check()
		g.children[0] = original
		if err == nil {
			t.Fatal("guard accepted corrupted/closed child or different resolution")
		}
	}
	if err := parent.check(); err != nil {
		t.Fatal("fault injection changed original custody:", err)
	}
	if err := g.Close(); err != nil {
		t.Fatal(err)
	}
	if err := parent.check(); err == nil {
		t.Fatal("closed original nonempty guard retained policy exception")
	}
	if err := files.Close(); err != nil {
		t.Fatal("guard owned or closed more than once:", err)
	}
}
