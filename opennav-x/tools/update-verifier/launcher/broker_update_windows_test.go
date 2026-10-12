package launcher

import (
	"strings"
	"testing"
	"unicode/utf16"

	"example.com/opennav-update-verifier/brokerpipe"
)

func TestNativeBrokerStartupRelayEnvironmentStripsQualification(t *testing.T) {
	a, _ := brokerExecutionFixture()
	c := brokerExecutionCommand(a, brokerpipe.RelaySelectedExecutor)
	c.Phase = brokerpipe.RelayStartupExecutor
	env, err := brokerRelayEnvironment([]string{"PATH=C:\\Windows", "SKAGER_UPDATE_QUALIFY_PROFILE=1", "SKAGER_UPDATE_PIPE=bad", "SKAGER_UPDATE_CHALLENGE=bad"}, c)
	if err != nil {
		t.Fatal(err)
	}
	text := strings.ToUpper(string(utf16.Decode(env)))
	if strings.Contains(text, "SKAGER_UPDATE_") {
		t.Fatal("qualification inherited into fresh offer")
	}
	c.PromptSHA256 = strings.Repeat("1", 64)
	if _, err = brokerRelayEnvironment(nil, c); err == nil {
		t.Fatal("relay accepted offer payload")
	}
}
