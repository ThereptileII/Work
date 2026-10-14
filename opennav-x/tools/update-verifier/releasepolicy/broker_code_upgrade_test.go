package releasepolicy

import (
	"bytes"
	"strings"
	"testing"
)

func codeUpgradePolicyFixture() BrokerCodeUpgradePolicy {
	return BrokerCodeUpgradePolicy{Schema: 1, Purpose: BrokerCodeUpgradePurpose, PreviousBrokerSHA256: strings.Repeat("e", 64), PreviousHelperSHA256: strings.Repeat("f", 64), Bootstrap: validBrokerBootstrap()}
}
func TestBrokerCodeUpgradeDistinctPurposeAndExactPair(t *testing.T) {
	p := codeUpgradePolicyFixture()
	raw, e := MarshalBrokerCodeUpgrade(p)
	if e != nil {
		t.Fatal(e)
	}
	got, e := ParseBrokerCodeUpgrade(raw)
	if e != nil || got != p {
		t.Fatal("pair round trip", e)
	}
	for _, change := range []func(*BrokerCodeUpgradePolicy){func(p *BrokerCodeUpgradePolicy) { p.Purpose = BrokerBootstrapPurpose }, func(p *BrokerCodeUpgradePolicy) { p.Schema = 2 }, func(p *BrokerCodeUpgradePolicy) { p.PreviousBrokerSHA256 = "*" }, func(p *BrokerCodeUpgradePolicy) { p.PreviousHelperSHA256 = p.Bootstrap.Provisioner.SHA256 }, func(p *BrokerCodeUpgradePolicy) { p.PreviousBrokerSHA256 = p.Bootstrap.Broker.SHA256 }, func(p *BrokerCodeUpgradePolicy) { p.Bootstrap.Purpose = BrokerCodeUpgradePurpose }, func(p *BrokerCodeUpgradePolicy) {
		p.Bootstrap.Provisioner.URL = "https://other.invalid/" + p.Bootstrap.Provisioner.Path
	}} {
		bad := p
		change(&bad)
		if _, e := ParseBrokerCodeUpgrade(bootstrapJSON(t, bad)); e == nil {
			t.Fatal("accepted foreign/unchanged/unsigned-purpose pair")
		}
	}
	for _, bad := range [][]byte{bootstrapJSON(t, p.Bootstrap), append(bytes.Clone(raw), []byte("{}")...), []byte(strings.Replace(string(raw), `"schema":1`, `"schema":1,"schema":1`, 1)), []byte(strings.Replace(string(raw), `"previousHelperSha256"`, `"PreviousHelperSha256"`, 1)), []byte(strings.Replace(string(raw), `"bootstrap":{`, `"bootstrap":{"unknown":1,`, 1))} {
		if _, e := ParseBrokerCodeUpgrade(bad); e == nil {
			t.Fatal("nonexact target accepted")
		}
	}
	i := BrokerBootstrapIdentity{Purpose: BrokerCodeUpgradePurpose, Channel: p.Bootstrap.Channel, Version: p.Bootstrap.Version, Commit: p.Bootstrap.Commit, TrustConfigSHA256: p.Bootstrap.TrustConfigSHA256}
	if _, e := ParseBrokerCodeUpgradeIdentity(bootstrapJSON(t, i)); e != nil {
		t.Fatal(e)
	}
	i.Purpose = BrokerBootstrapPurpose
	if _, e := ParseBrokerCodeUpgradeIdentity(bootstrapJSON(t, i)); e == nil {
		t.Fatal("enrollment purpose authorizes code replacement")
	}
	if path, e := BrokerCodeUpgradeTargetPath("beta"); e != nil || path != "bootstrap/broker-code-upgrade-beta.json" {
		t.Fatal(path, e)
	}
	if _, e := BrokerCodeUpgradeTargetPath("../beta"); e == nil {
		t.Fatal("path escape")
	}
}
