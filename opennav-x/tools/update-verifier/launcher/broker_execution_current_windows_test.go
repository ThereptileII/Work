package launcher

import (
	"strings"
	"testing"
	"unicode/utf16"

	"example.com/opennav-update-verifier/brokerpipe"
)

func TestNativeBrokerCurrentEnvironmentPreservesNormalSemanticsWithoutQualification(t *testing.T) {
	a, _ := brokerExecutionFixture()
	command := brokerExecutionCommand(a, brokerpipe.SpawnCurrent)
	inherited := []string{"PATH=C:\\Windows", "KEEP=ordinary-value", "SKAGER_UPDATE_PIPE=untrusted", "skager_update_generation=untrusted", "SKAGER_UPDATE_CHALLENGE=untrusted", "sKaGeR_UpDaTe_QUALIFY_PROFILE=1", "SKAGER_UPDATE_UNKNOWN=untrusted", "UNICODE=åäö", "=C:=C:\\owned"}
	block, err := brokerCurrentEnvironment(inherited, command)
	if err != nil {
		t.Fatal(err)
	}
	raw := string(utf16.Decode(block))
	if !strings.HasSuffix(raw, "\x00\x00") || strings.Contains(strings.ToUpper(raw), "SKAGER_UPDATE_") || strings.Contains(raw, "untrusted") {
		t.Fatal("normal child retained qualification fields")
	}
	for _, value := range []string{"PATH=C:\\Windows", "KEEP=ordinary-value", "UNICODE=åäö", "=C:=C:\\owned"} {
		if !strings.Contains(raw, value+"\x00") {
			t.Fatal("normal product environment changed", value)
		}
	}
	if !strings.Contains(strings.Join(inherited, "|"), "sKaGeR_UpDaTe_QUALIFY_PROFILE=1") {
		t.Fatal("mutated parent qualification state")
	}
	qualification := brokerExecutionCommand(a, brokerpipe.SpawnCandidate)
	qualified, err := brokerExecutionEnvironment(inherited, qualification)
	if err != nil || !strings.Contains(string(utf16.Decode(qualified)), "SKAGER_UPDATE_QUALIFY_PROFILE=1\x00") {
		t.Fatal("qualification creator lost lifetime output fence", err)
	}
	for _, mode := range []string{"health-pipe", "health-challenge", "wrong-phase", "embedded-nul", "oversize"} {
		t.Run(mode, func(t *testing.T) {
			c, values := command, inherited
			switch mode {
			case "health-pipe":
				c.HealthPipe = "Skager.Update." + strings.Repeat("6", 32)
			case "health-challenge":
				c.HealthChallenge = strings.Repeat("7", 64)
			case "wrong-phase":
				c.Phase = brokerpipe.SpawnCandidate
			case "embedded-nul":
				values = []string{"BAD=embedded\x00value"}
			case "oversize":
				values = []string{"BIG=" + strings.Repeat("x", 32767)}
			}
			if _, err := brokerCurrentEnvironment(values, c); err == nil {
				t.Fatal("unsafe normal creator environment")
			}
		})
	}
}

// Actual creator handles/image are exercised while the inert test PE remains
// suspended. No normal product/health/session/service qualification is claimed.
func TestNativeBrokerCurrentSuspendedCreatorRetainsOriginalImage(t *testing.T) {
	a, _ := brokerExecutionFixture()
	brokerExecutionSuspendedCreator(t, func(image *nativeOrdinaryImage) (*nativeOrdinaryChild, error) {
		return image.createBrokerCurrent(brokerExecutionCommand(a, brokerpipe.SpawnCurrent), nil)
	})
}
