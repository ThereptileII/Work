package brokerstore

import (
	"bytes"
	"context"
	"errors"
	"strings"
	"testing"
)

func decommissionTestBinding() DecommissionBinding {
	return DecommissionBinding{RequestID: stageA, ClientSID: "S-1-5-21-111-222-333-1001", RegistrationSHA256: digest("registration"), SelectedHeadSHA256: digest("head"), GenerationID: stageA, CompletionSHA256: digest("completion"), ProfileSHA256: digest("profile"), ProfileIdentitySHA256: digest("identity")}
}

type decommissionTestStore struct {
	memoryStore
	prepared, commits, closes, guards                       int
	inventoryFault, prepareFault, commitFault, receiptFault error
	seen                                                    DecommissionBinding
	terminal                                                bool
}

func (m *decommissionTestStore) check() error {
	if m.terminal {
		return ErrDecommissioned
	}
	return m.memoryStore.check()
}
func (m *decommissionTestStore) singletonRegistration(ctx context.Context, sid, digest string) error {
	return m.inventoryFault
}
func (m *decommissionTestStore) prepareDecommission(ctx context.Context, b DecommissionBinding) (decommissionBackend, error) {
	m.prepared++
	m.seen = b
	if m.prepareFault != nil {
		return nil, m.prepareFault
	}
	if m.inventoryFault != nil {
		return nil, m.inventoryFault
	}
	return &decommissionTestBackend{store: m}, nil
}

type decommissionTestBackend struct{ store *decommissionTestStore }

func (b *decommissionTestBackend) commit(ctx context.Context) error {
	b.store.commits++
	b.store.terminal = true
	return b.store.commitFault
}
func (b *decommissionTestBackend) check() error { return b.store.receiptFault }
func (b *decommissionTestBackend) close() error { b.store.closes++; return nil }
func TestDecommissionOriginalGuardOutsideLockAndTerminalReceipt(t *testing.T) {
	m := &decommissionTestStore{}
	s := &Store{backend: m}
	binding := decommissionTestBinding()
	owner, err := s.CommitDecommission(context.Background(), binding, func(ctx context.Context) error {
		m.guards++
		if err := s.Validate(); err != nil {
			return err
		}
		_, err := s.ReadRegistration(binding.ClientSID)
		return err
	})
	if err != nil {
		t.Fatal(err)
	}
	if m.guards != 1 || m.commits != 1 || m.prepared != 1 || m.seen != binding {
		t.Fatal("original exact operation not retained")
	}
	if err = s.Validate(); !errors.Is(err, ErrDecommissioned) {
		t.Fatalf("ordinary use after fence: %v", err)
	}
	if err = owner.Validate(context.Background()); err != nil {
		t.Fatal(err)
	}
	snapshot := owner.Binding()
	snapshot.ClientSID = "S-1-5-18"
	if owner.Binding() != binding {
		t.Fatal("binding escaped immutable custody")
	}
	if _, err = s.CommitDecommission(context.Background(), binding, func(context.Context) error { t.Fatal("retry guard invoked"); return nil }); err == nil {
		t.Fatal("one-shot retry accepted")
	}
	if err = s.Close(); err != nil {
		t.Fatal(err)
	}
	if m.closes != 1 {
		t.Fatal("original writer not drained exactly once")
	}
	if err = owner.Validate(context.Background()); !errors.Is(err, ErrClosed) {
		t.Fatal(err)
	}
}
func TestDecommissionGuardAndFinalInventoryRefuseBeforePublication(t *testing.T) {
	for _, test := range []string{"guard", "inventory-after-guard", "cancel-after-guard", "store-close-in-guard"} {
		t.Run(test, func(t *testing.T) {
			m := &decommissionTestStore{}
			s := &Store{backend: m}
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			_, err := s.CommitDecommission(ctx, decommissionTestBinding(), func(context.Context) error {
				switch test {
				case "guard":
					return testFault
				case "inventory-after-guard":
					m.inventoryFault = testFault
				case "cancel-after-guard":
					cancel()
				case "store-close-in-guard":
					return s.Close()
				}
				return nil
			})
			if err == nil || m.commits != 0 || m.terminal {
				t.Fatalf("precondition permitted mutation: %v", err)
			}
			if _, err = s.CommitDecommission(context.Background(), decommissionTestBinding(), func(context.Context) error { return nil }); err == nil {
				t.Fatal("failed original attempt retried")
			}
			s.Close()
		})
	}
}
func TestDecommissionUncertainOwnerRetainedForStoreCleanup(t *testing.T) {
	m := &decommissionTestStore{commitFault: ErrDecommissionUncertain}
	s := &Store{backend: m}
	owner, err := s.CommitDecommission(context.Background(), decommissionTestBinding(), func(context.Context) error { return nil })
	if owner != nil || !errors.Is(err, ErrDecommissionUncertain) || !m.terminal {
		t.Fatal("uncertain attempt reported usable receipt", owner, err)
	}
	if len(s.decommissions) != 1 {
		t.Fatal("uncertain original custody discarded")
	}
	s.Close()
	s.Close()
	if m.closes != 1 {
		t.Fatal("cleanup retried or omitted")
	}
}
func TestDecommissionBindingBoundsAndSingletonInventory(t *testing.T) {
	binding := decommissionTestBinding()
	if !binding.valid() {
		t.Fatal("fixture invalid")
	}
	for _, field := range []*string{&binding.RequestID, &binding.ClientSID, &binding.RegistrationSHA256, &binding.SelectedHeadSHA256, &binding.GenerationID, &binding.CompletionSHA256, &binding.ProfileSHA256, &binding.ProfileIdentitySHA256} {
		original := *field
		*field = original + "\n"
		if binding.valid() {
			t.Fatal("unbounded/injected binding accepted")
		}
		*field = original
	}
	sid := binding.ClientSID
	for _, names := range [][]string{nil, {sid + ".json"}, {sid + ".once", sid + ".once"}, {sid + ".once", sid + ".json", "foreign.once"}, {sid + ".once", strings.ToLower(sid) + ".json"}, {sid + ".once", "foreign.json"}} {
		if decommissionInventoryNames(names, sid) == nil {
			t.Fatalf("ambiguous inventory accepted: %q", names)
		}
	}
	if err := decommissionInventoryNames([]string{sid + ".json", sid + ".once"}, sid); err != nil {
		t.Fatal(err)
	}
}
func TestDecommissionPublicationCancellationAndFaultBoundaries(t *testing.T) {
	raw := decommissionBytes(decommissionTestBinding())
	for _, fault := range []string{"create", "sync", "readback", "custody", "cancel-after-create", "cancel-before-create"} {
		t.Run(fault, func(t *testing.T) {
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			file := &memoryFile{}
			creates := 0
			checks := 0
			if fault == "cancel-before-create" {
				cancel()
			}
			if fault == "sync" {
				file.syncFault = testFault
			}
			if fault == "readback" {
				file.onRead = func() { file.data[0] = '!' }
			}
			_, err := publishDecommission(ctx, func() (candidateFile, error) {
				creates++
				if fault == "create" {
					return nil, testFault
				}
				if fault == "cancel-after-create" {
					cancel()
				}
				return file, nil
			}, raw, func() error {
				checks++
				if fault == "custody" && checks > 1 {
					return testFault
				}
				return nil
			})
			switch fault {
			case "cancel-before-create":
				if !errors.Is(err, context.Canceled) || creates != 0 || errors.Is(err, ErrDecommissionUncertain) {
					t.Fatal("cancel crossed creation", err)
				}
			case "cancel-after-create":
				if err != nil || !bytes.Equal(file.data, raw) {
					t.Fatal("cancel relabeled completed fence", err)
				}
			default:
				if !errors.Is(err, ErrDecommissionUncertain) || creates != 1 {
					t.Fatal("publication uncertainty lost", err)
				}
			}
			if file.closes != 0 {
				t.Fatal("publication dropped original custody")
			}
		})
	}
}

func TestDecommissionPublicationEveryIOFaultRetainsOriginalFile(t *testing.T) {
	raw := decommissionBytes(decommissionTestBinding())
	good := &registrationFileFixture{}
	original, err := publishDecommission(context.Background(), func() (candidateFile, error) { return good, nil }, raw, func() error { return good.step("custody") })
	if err != nil || original != good {
		t.Fatal("original file lost", err)
	}
	for failure := 1; failure <= len(good.events); failure++ {
		file := &registrationFileFixture{failAt: failure}
		original, err := publishDecommission(context.Background(), func() (candidateFile, error) { return file, nil }, raw, func() error { return file.step("custody") })
		if original != file || !errors.Is(err, ErrDecommissionUncertain) || !errors.Is(err, testFault) {
			t.Fatalf("boundary %d lost uncertainty/custody: %v", failure, err)
		}
		for _, event := range file.events {
			if event == "close" {
				t.Fatal("uncertain publication released original file")
			}
		}
	}
	for _, file := range []*registrationFileFixture{{short: true}, {corrupt: true}} {
		original, err := publishDecommission(context.Background(), func() (candidateFile, error) { return file, nil }, raw, func() error { return nil })
		if original != file || !errors.Is(err, ErrDecommissionUncertain) {
			t.Fatal("short/mutated publication accepted", err)
		}
	}
}

func TestDecommissionRequiresNativeCapabilityAndPoisonsFailedReceipt(t *testing.T) {
	ordinary, _ := testStore()
	if _, err := ordinary.CommitDecommission(context.Background(), decommissionTestBinding(), func(context.Context) error { t.Fatal("guard invoked without native capability"); return nil }); !errors.Is(err, ErrNativeRequired) {
		t.Fatal(err)
	}
	ordinary.Close()
	m := &decommissionTestStore{}
	s := &Store{backend: m}
	defer s.Close()
	owner, err := s.CommitDecommission(context.Background(), decommissionTestBinding(), func(context.Context) error { return nil })
	if err != nil {
		t.Fatal(err)
	}
	m.receiptFault = testFault
	if err = owner.Validate(context.Background()); !errors.Is(err, testFault) {
		t.Fatal("receipt uncertainty ignored", err)
	}
	m.receiptFault = nil
	if err = owner.Validate(context.Background()); !errors.Is(err, ErrPoisoned) {
		t.Fatal("uncertain original receipt recovered", err)
	}
	if err = s.Validate(); !errors.Is(err, ErrDecommissioned) {
		t.Fatal("ordinary store recovered", err)
	}
}
