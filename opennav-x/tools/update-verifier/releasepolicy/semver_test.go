package releasepolicy

import (
	"strings"
	"testing"
)

func TestSemVerOfficialPrecedence(t *testing.T) {
	ordered := []string{"1.0.0-alpha", "1.0.0-alpha.1", "1.0.0-alpha.beta", "1.0.0-beta", "1.0.0-beta.2", "1.0.0-beta.11", "1.0.0-rc.1", "1.0.0"}
	for i := 0; i < len(ordered)-1; i++ {
		a, e := ParseSemVer(ordered[i])
		if e != nil {
			t.Fatal(e)
		}
		b, e := ParseSemVer(ordered[i+1])
		if e != nil {
			t.Fatal(e)
		}
		if a.Compare(b) >= 0 {
			t.Fatalf("%s should precede %s", ordered[i], ordered[i+1])
		}
	}
	a, _ := ParseSemVer("1.0.0+build.1")
	b, _ := ParseSemVer("1.0.0+build.2")
	if a.Compare(b) != 0 {
		t.Fatal("build metadata changed precedence")
	}
	a, _ = ParseSemVer("1.99999999999999999999999999999999999.0")
	b, _ = ParseSemVer("1.99999999999999999999999999999999998.999")
	if a.Compare(b) <= 0 {
		t.Fatal("large numeric compare overflow")
	}
	a, _ = ParseSemVer("0.4.0-beta2")
	if !a.IsPrerelease() {
		t.Fatal("current beta rejected")
	}
}
func TestSemVerStrict(t *testing.T) {
	for _, s := range []string{"", "v1.0.0", "01.0.0", "1.02.0", "1.0.00", "1.0", "1.0.0-01", "1.0.0-alpha..1", "1.0.0+", "1.0.0+bad!", "1.0.0-β", strings.Repeat("1", 129) + ".0.0"} {
		if _, err := ParseSemVer(s); err == nil {
			t.Fatalf("accepted %q", s)
		}
	}
}
