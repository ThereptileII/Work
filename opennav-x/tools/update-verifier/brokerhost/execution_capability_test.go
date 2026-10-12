package brokerhost

import (
	"context"
	"encoding/json"
	"errors"
	"strings"
	"testing"

	"example.com/opennav-update-verifier/brokerpipe"
	"example.com/opennav-update-verifier/brokerstore"
)

func executionCapabilityFixture(t *testing.T) (brokerstore.GenerationApplicationIdentity, []byte) {
	t.Helper()
	identity := brokerstore.GenerationApplicationIdentity{ClientSID: testSID, Generation: strings.Repeat("a", 32), Commit: strings.Repeat("b", 40), ExecutableSHA256: strings.Repeat("c", 64)}
	raw, err := json.Marshal(executionQualificationCapability{1, "skager-qualified-update-v1", identity.Commit, identity.ExecutableSHA256, 1, 1, "deferred-until-ordinary-start"})
	if err != nil {
		t.Fatal(err)
	}
	return identity, raw
}

func TestExecutionCapabilityExactProducerBindingAndClosedSchema(t *testing.T) {
	identity, raw := executionCapabilityFixture(t)
	if err := parseExecutionCapability(raw, identity); err != nil {
		t.Fatal(err)
	}
	for _, tc := range []struct {
		name   string
		mutate func([]byte) []byte
	}{
		{"missing", func([]byte) []byte { return nil }},
		{"old producer", func([]byte) []byte { return []byte("{}") }},
		{"malformed", func([]byte) []byte { return []byte("{") }},
		{"oversized", func([]byte) []byte { return []byte(strings.Repeat("x", 4097)) }},
		{"newline", func(b []byte) []byte { return append(b, '\n') }},
		{"unknown field", func(b []byte) []byte {
			return append(append([]byte(nil), b[:len(b)-1]...), []byte(",\"healthy\":true}")...)
		}},
		{"duplicate field", func(b []byte) []byte {
			return append(append([]byte(nil), b[:len(b)-1]...), []byte(",\"schema\":1}")...)
		}},
		{"wrong commit", func(b []byte) []byte {
			return []byte(strings.ReplaceAll(string(b), identity.Commit, strings.Repeat("d", 40)))
		}},
		{"wrong executable", func(b []byte) []byte {
			return []byte(strings.ReplaceAll(string(b), identity.ExecutableSHA256, strings.Repeat("d", 64)))
		}},
		{"uppercase commit", func(b []byte) []byte {
			return []byte(strings.ReplaceAll(string(b), identity.Commit, strings.ToUpper(identity.Commit)))
		}},
		{"unsupported schema", func(b []byte) []byte { return []byte(strings.ReplaceAll(string(b), "\"schema\":1", "\"schema\":2")) }},
		{"wrong purpose", func(b []byte) []byte {
			return []byte(strings.ReplaceAll(string(b), "skager-qualified-update-v1", "skager-qualified-update-v2"))
		}},
		{"configuration absent", func(b []byte) []byte {
			return []byte(strings.ReplaceAll(string(b), "\"configurationProtocol\":1", "\"configurationProtocol\":0"))
		}},
		{"suppression absent", func(b []byte) []byte {
			return []byte(strings.ReplaceAll(string(b), "\"coreOutputSuppression\":1", "\"coreOutputSuppression\":0"))
		}},
		{"third party code allowed", func(b []byte) []byte {
			return []byte(strings.ReplaceAll(string(b), "deferred-until-ordinary-start", "loaded-before-health"))
		}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			if err := parseExecutionCapability(tc.mutate(append([]byte(nil), raw...)), identity); !errors.Is(err, ErrExecutionChildRefused) {
				t.Fatal("unsafe capability accepted", err)
			}
		})
	}
	if err := parseExecutionCapability(raw, brokerstore.GenerationApplicationIdentity{}); err == nil {
		t.Fatal("data qualified empty image identity")
	}
}

type memoryExecutionCapability struct {
	identity          brokerstore.GenerationApplicationIdentity
	raw               []byte
	err               error
	identities, reads int
	onRead            func()
	changeAfter       bool
}

func (m *memoryExecutionCapability) Identity(ctx context.Context) (brokerstore.GenerationApplicationIdentity, error) {
	m.identities++
	if err := ctx.Err(); err != nil {
		return brokerstore.GenerationApplicationIdentity{}, err
	}
	if m.changeAfter && m.identities > 1 {
		x := m.identity
		x.Generation = strings.Repeat("f", 32)
		return x, nil
	}
	return m.identity, m.err
}
func (m *memoryExecutionCapability) QualificationBytes(ctx context.Context) ([]byte, error) {
	m.reads++
	if m.onRead != nil {
		m.onRead()
	}
	return append([]byte(nil), m.raw...), errors.Join(ctx.Err(), m.err)
}
func TestExecutionCapabilityRequiresLiveOriginalAcrossReadBeforeNativeFactory(t *testing.T) {
	identity, raw := executionCapabilityFixture(t)
	for _, fault := range []string{"valid", "cancel-before", "cancel-during", "owner-close", "mutation", "malformed", "missing"} {
		t.Run(fault, func(t *testing.T) {
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			m := &memoryExecutionCapability{identity: identity, raw: raw}
			switch fault {
			case "cancel-before":
				cancel()
			case "cancel-during":
				m.onRead = cancel
			case "owner-close":
				m.onRead = func() { m.err = brokerstore.ErrClosed }
			case "mutation":
				m.changeAfter = true
			case "malformed":
				m.raw = []byte("{}")
			case "missing":
				m.raw = nil
			}
			got, err := validateExecutionCapability(ctx, m)
			if fault == "valid" {
				if err != nil || got != identity || m.identities != 2 || m.reads != 1 {
					t.Fatal(got, err, m.identities, m.reads)
				}
			} else if !errors.Is(err, ErrExecutionChildRefused) || got != (brokerstore.GenerationApplicationIdentity{}) {
				t.Fatal("invalid custody reached factory identity", got, err)
			}
			if fault == "cancel-before" && (m.identities != 0 || m.reads != 0) {
				t.Fatal("cancelled gate read original")
			}
		})
	}
}
func TestExecutionCapabilityPublicFactoryCannotMintFromDataOrZeroImage(t *testing.T) {
	profile, _ := profileFixture()
	defer profile.Close()
	// Even with a real-shaped profile and nonnil peer wrapper, no sealed
	// image/generation exists. The factory must refuse before native arming.
	observer, err := PrepareExecutionChildNative(context.Background(), &brokerpipe.PeerCapability{}, &brokerstore.GenerationApplicationImage{}, profile)
	if err == nil || observer != nil {
		t.Fatal("zero serialized image armed native observer", observer, err)
	}
	if got, err := validateExecutionCapability(context.Background(), (*brokerstore.GenerationApplicationImage)(nil)); err == nil || got != (brokerstore.GenerationApplicationIdentity{}) {
		t.Fatal("typed nil image qualified", got, err)
	}
}
