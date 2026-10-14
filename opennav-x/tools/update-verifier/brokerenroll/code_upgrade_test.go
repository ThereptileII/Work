package brokerenroll

import (
	"bytes"
	"context"
	"errors"
	verifier "example.com/opennav-update-verifier"
	"example.com/opennav-update-verifier/releasepolicy"
	"example.com/opennav-update-verifier/repository"
	"io"
	"reflect"
	"strings"
	"testing"
)

type codeOwnerFixture struct {
	events *[]string
	fault  string
	closed bool
}

func (o *codeOwnerFixture) step(s string) error {
	*o.events = append(*o.events, s)
	if o.fault == s {
		return errors.New("injected " + s)
	}
	return nil
}
func (o *codeOwnerFixture) check(context.Context) error {
	if o.closed {
		return errors.New("closed")
	}
	return nil
}
func (o *codeOwnerFixture) stage(context.Context, io.Reader, maintenanceImageSource) error {
	return o.step("stage-intent-pair")
}
func (o *codeOwnerFixture) reserve(context.Context) error { return o.step("reserve") }
func (o *codeOwnerFixture) quiesce(context.Context) error { return o.step("disable-stop-exit") }
func (o *codeOwnerFixture) replace(context.Context) error { return o.step("replace") }
func (o *codeOwnerFixture) validateStore(context.Context) error {
	return o.step("validate-store-select-pair")
}
func (o *codeOwnerFixture) activate(context.Context) error { return o.step("activate") }
func (o *codeOwnerFixture) ready(context.Context) error    { return o.step("ready") }
func (o *codeOwnerFixture) complete(context.Context) error { return o.step("complete") }
func (o *codeOwnerFixture) close() error                   { o.closed = true; return o.step("owner.close") }
func upgradeFixture(t *testing.T) (CodeUpgradeRequest, codeUpgradeOperations, *codeOwnerFixture, *[]string) {
	f := newEnrollmentFixture(t)
	events := []string{}
	owner := &codeOwnerFixture{events: &events}
	p := releasepolicy.BrokerCodeUpgradePolicy{Schema: 1, Purpose: releasepolicy.BrokerCodeUpgradePurpose, PreviousBrokerSHA256: strings.Repeat("e", 64), PreviousHelperSHA256: strings.Repeat("f", 64), Bootstrap: f.policy}
	raw, e := releasepolicy.MarshalBrokerCodeUpgrade(p)
	if e != nil {
		t.Fatal(e)
	}
	r := CodeUpgradeRequest{TrustConfiguration: f.req.TrustConfiguration, BootstrapCacheDir: f.req.BootstrapCacheDir, ArtifactDirectory: f.req.ArtifactDirectory, PolicySHA256: digest(raw), ProvisionerSHA256: p.Bootstrap.Provisioner.SHA256, TrustConfigSHA256: p.Bootstrap.TrustConfigSHA256, OriginalCompletionSHA256: strings.Repeat("d", 64)}
	source := &maintenanceImageFixture{Reader: bytes.NewReader([]byte("INERT PROVISIONER")), testOwner: &testOwner{onClose: func() { events = append(events, "source.close") }}}
	op := codeUpgradeOperations{trust: func(_ context.Context, raw []byte, pin string) (repository.Trust, error) {
		events = append(events, "native-original-trust")
		if pin != r.OriginalCompletionSHA256 {
			t.Fatal("completion pin lost")
		}
		return repository.ValidateTrustIdentity(raw)
	}, hold: func(string, string) (io.Closer, error) {
		events = append(events, "hold")
		return &testOwner{onClose: func() { events = append(events, "inputs.close") }}, nil
	}, verify: func(q verifier.BrokerBootstrapRequest) (releasepolicy.BrokerCodeUpgradePolicy, []byte, error) {
		events = append(events, "verify-upgrade-purpose")
		if q.ApprovedPolicySHA256 != r.PolicySHA256 {
			t.Fatal("policy pin lost")
		}
		return p, raw, nil
	}, source: func(context.Context, releasepolicy.Artifact) (maintenanceImageSource, error) {
		events = append(events, "source")
		return source, nil
	}, acquire: func(context.Context, releasepolicy.Artifact, string, string) (acquiredBroker, error) {
		events = append(events, "acquire")
		return &testBroker{bytes.NewReader(f.brokerBytes), &testOwner{onClose: func() { events = append(events, "broker.close") }}}, nil
	}, open: func(_ context.Context, _ releasepolicy.BrokerCodeUpgradePolicy, actualRaw, trust []byte, pin string) (codeUpgradeOwner, error) {
		events = append(events, "native-completion-service")
		if !bytes.Equal(raw, actualRaw) || !bytes.Equal(trust, r.TrustConfiguration) || pin != r.OriginalCompletionSHA256 {
			t.Fatal("native input binding lost")
		}
		return owner, nil
	}}
	return r, op, owner, &events
}
func TestCodeUpgradeCustodyPurposeSequenceAndSingleEffects(t *testing.T) {
	r, op, _, events := upgradeFixture(t)
	if e := upgradeEnrolledCodeWith(context.Background(), r, op); e != nil {
		t.Fatal(e)
	}
	want := []string{"native-original-trust", "hold", "verify-upgrade-purpose", "source", "acquire", "native-completion-service", "stage-intent-pair", "reserve", "disable-stop-exit", "replace", "validate-store-select-pair", "activate", "ready", "complete", "owner.close", "broker.close", "source.close", "inputs.close"}
	if !reflect.DeepEqual(*events, want) {
		t.Fatal(*events)
	}
}
func TestCodeUpgradeFailureNeverContinuesOrRetries(t *testing.T) {
	steps := []string{"stage-intent-pair", "reserve", "disable-stop-exit", "replace", "validate-store-select-pair", "activate", "ready", "complete"}
	for i, fault := range steps {
		t.Run(fault, func(t *testing.T) {
			r, op, owner, events := upgradeFixture(t)
			owner.fault = fault
			e := upgradeEnrolledCodeWith(context.Background(), r, op)
			if !errors.Is(e, ErrCodeUpgradeUncertain) || !owner.closed {
				t.Fatal("postintent failure lost uncertainty/owners", e)
			}
			for _, next := range steps[i+1:] {
				for _, event := range *events {
					if event == next {
						t.Fatal("continued past uncertainty", *events)
					}
				}
			}
		})
	}
}
func TestCodeUpgradeExactVerifiedBytesBeforeMachineOpen(t *testing.T) {
	for _, fault := range []string{"policy", "source", "original-trust", "missing-completion-pin", "changed-trust"} {
		t.Run(fault, func(t *testing.T) {
			r, op, _, events := upgradeFixture(t)
			switch fault {
			case "policy":
				v := op.verify
				op.verify = func(q verifier.BrokerBootstrapRequest) (releasepolicy.BrokerCodeUpgradePolicy, []byte, error) {
					p, raw, e := v(q)
					return p, append(raw, ' '), e
				}
			case "source":
				op.source = func(context.Context, releasepolicy.Artifact) (maintenanceImageSource, error) {
					return nil, errors.New("source changed")
				}
			case "original-trust":
				op.trust = func(context.Context, []byte, string) (repository.Trust, error) {
					return repository.Trust{}, errors.New("no genuine completion")
				}
			case "missing-completion-pin":
				r.OriginalCompletionSHA256 = ""
			case "changed-trust":
				r.TrustConfiguration = append(r.TrustConfiguration, ' ')
			}
			if e := upgradeEnrolledCodeWith(context.Background(), r, op); e == nil {
				t.Fatal("substitution accepted")
			}
			for _, event := range *events {
				if event == "native-completion-service" || event == "stage-intent-pair" {
					t.Fatal("reached machine code mutation", *events)
				}
			}
		})
	}
}
