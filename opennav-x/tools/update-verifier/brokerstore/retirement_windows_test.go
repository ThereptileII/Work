package brokerstore

import (
	"context"
	"errors"
	"os"
	"path/filepath"
	"runtime"
	"testing"

	"golang.org/x/sys/windows"
)

// These current-user fixtures exercise real same-parent NTFS rename custody.
// They never access the product namespace, service, profile or stock recovery.
func TestNativeRetirementOriginalIdentityCollisionCancellationAndPartialMove(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	n, _ := nativeRegistrationFixture(t)
	parent := n.registrations
	sd, err := descriptor(n.broker, nil, false)
	if err != nil {
		t.Fatal(err)
	}
	create := func(name string) diskIdentity {
		t.Helper()
		file, err := createRegistrationFile(parent, name, sd)
		if err != nil {
			t.Fatal(err)
		}
		id, err := fileIdentity(windows.Handle(file.Fd()))
		if err != nil {
			t.Fatal(err)
		}
		if err = file.Close(); err != nil {
			t.Fatal(err)
		}
		return id
	}
	first, second := "retire-fixture-first.json", "retire-fixture-second.json"
	firstID, secondID := create(first), create(second)
	if _, err = openRetirementObject(parent, first, secondID, false, sd); err == nil {
		t.Fatal("changed physical identity accepted")
	}
	owner1, err := openRetirementObject(parent, first, firstID, false, sd)
	if err != nil {
		t.Fatal(err)
	}
	defer owner1.close()
	owner2, err := openRetirementObject(parent, second, secondID, false, sd)
	if err != nil {
		t.Fatal(err)
	}
	defer owner2.close()
	ctx, cancel := context.WithCancel(context.Background())
	cancel()
	if err = owner1.move(ctx, "retired-first.json"); err == nil {
		t.Fatal("cancelled retirement moved original")
	}
	if _, err = os.Stat(filepath.Join(n.rootDOS, "registrations", first)); err != nil {
		t.Fatal("cancelled original missing", err)
	}
	collision := "retired-second.json"
	create(collision)
	if err = owner2.move(context.Background(), collision); err == nil {
		t.Fatal("colliding destination replaced")
	}
	if err = owner1.move(context.Background(), "retired-first.json"); err != nil {
		t.Fatal("first original move", err)
	}
	if err = owner2.move(context.Background(), collision); err == nil {
		t.Fatal("partial second move replaced collision")
	}
	if err = owner1.check(); err != nil {
		t.Fatal("moved original identity lost", err)
	}
	if err = owner2.check(); err != nil {
		t.Fatal("unmoved original identity lost", err)
	}
	if _, err = os.Stat(filepath.Join(n.rootDOS, "registrations", "retired-first.json")); err != nil {
		t.Fatal("partial first evidence missing", err)
	}
	if _, err = os.Stat(filepath.Join(n.rootDOS, "registrations", second)); err != nil {
		t.Fatal("partial second evidence missing", err)
	}
	if _, err = os.Stat(filepath.Join(n.rootDOS, "registrations", first)); !errors.Is(err, os.ErrNotExist) {
		t.Fatal("old name still present", err)
	}
}

func TestNativeRetirementMovesEntireOriginalDirectoryOnItsHeldVolume(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	n, _ := nativeRegistrationFixture(t)
	parent := n.registrations
	dirSD, err := descriptor(n.broker, nil, true)
	if err != nil {
		t.Fatal(err)
	}
	creator := &nativeCandidate{store: n, securityDir: dirSD}
	pin, err := creator.newDirectory(parent, "retire-fixture-generation")
	if err != nil {
		t.Fatal(err)
	}
	id, err := fileIdentity(pin.handle)
	if err != nil {
		t.Fatal(err)
	}
	if err = creator.close(); err != nil {
		t.Fatal(err)
	}
	unknown := filepath.Join(n.rootDOS, "registrations", "retire-fixture-generation", "unknown-child.txt")
	if err = os.WriteFile(unknown, []byte("keep"), 0600); err != nil {
		t.Fatal(err)
	}
	owner, err := openRetirementObject(parent, "retire-fixture-generation", id, true, dirSD)
	if err != nil {
		t.Fatal(err)
	}
	defer owner.close()
	if err = owner.move(context.Background(), "retired-fixture-generation"); err != nil {
		t.Fatal(err)
	}
	if err = owner.check(); err != nil {
		t.Fatal(err)
	}
	actual, err := os.ReadFile(filepath.Join(n.rootDOS, "registrations", "retired-fixture-generation", "unknown-child.txt"))
	if err != nil || string(actual) != "keep" {
		t.Fatal("unknown child not preserved", err)
	}
}

func TestNativeRetirementCompletedOriginalRegistrationAndSelectedGeneration(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	s, previous, view, _, raw := nativeBaselineViewFixture(t)
	baseline, err := s.MaterializeApprovedBaselineReadOnlyFresh(context.Background(), previous, raw, registrationDigest(raw))
	if err != nil {
		t.Fatal(err)
	}
	if err = baseline.Close(); err != nil {
		t.Fatal(err)
	}
	if err = previous.Close(); err != nil {
		t.Fatal(err)
	}
	opener := &nativeGenerationReopenFixtureStore{view}
	selected, _, err := opener.openReadOnlyGeneration(context.Background(), view.sid.String(), stageA)
	if err != nil {
		t.Fatal(err)
	}
	n := view.nativeStore
	sd, err := descriptor(n.broker, nil, false)
	if err != nil {
		t.Fatal(err)
	}
	registration, err := readCompletedRegistration(n.registrations, view.sid.String(), sd)
	if err != nil {
		t.Fatal(err)
	}
	binding := decommissionTestBinding()
	binding.ClientSID = view.sid.String()
	binding.RegistrationSHA256 = registrationDigest(registration)
	backend, err := n.prepareRetirementWithOpeners(context.Background(), binding, decommissionInventoryOpeners{opener.openReadOnlyGeneration, nil})
	if err != nil {
		t.Fatal(err)
	}
	p := backend.(*nativeRetirementPreparation)
	defer p.close()
	if err = p.check(); err != nil {
		t.Fatal(err)
	}
	if rejected, err := p.commit(context.Background()); err == nil || rejected != nil {
		t.Fatal("production retirement accepted identical fixture broker/client SID", err)
	}
	if _, err := os.Stat(filepath.Join(n.rootDOS, decommissionFilename)); !errors.Is(err, os.ErrNotExist) {
		t.Fatal("principal refusal published terminal marker", err)
	}
	if err = selected.close(); err != nil {
		t.Fatal("selected owner did not drain", err)
	}
	owner, err := p.commitWithSelectedDescriptor(context.Background(), p.generationSecurity)
	// Commit can return an owner together with an error after DELETE acquisition;
	// even a failed fixture must release the retained parent and partial handles.
	var r *nativeRetirement
	if owner != nil {
		var ok bool
		r, ok = owner.(*nativeRetirement)
		if !ok {
			t.Fatal("unexpected native retirement owner")
		}
		defer func() {
			if closeErr := r.close(); closeErr != nil {
				t.Error("retirement owner close", closeErr)
			}
		}()
	}
	if err != nil {
		t.Fatal("actual original retirement", err)
	}
	if r == nil {
		t.Fatal("completed native retirement lacked original owner")
	}
	if err = r.check(); err != nil {
		t.Fatal("original recovery evidence", err)
	}
	if _, err = r.fenceSHA256(); err != nil {
		t.Fatal(err)
	}
	if _, err = r.evidenceSHA256(); err != nil {
		t.Fatal(err)
	}
	if err = n.check(); !errors.Is(err, ErrDecommissioned) {
		t.Fatal("startup permitted after retirement", err)
	}
	if _, err = os.Stat(filepath.Join(n.rootDOS, "registrations", binding.ClientSID+".once")); !errors.Is(err, os.ErrNotExist) {
		t.Fatal("registration source remains", err)
	}
	if _, err = os.Stat(filepath.Join(n.rootDOS, "registrations", retirementRegistrationName(binding, "once"))); err != nil {
		t.Fatal("original registration recovery missing", err)
	}
	if _, err = os.Stat(physical(r.generationParent.name + `\` + retirementGenerationName(binding))); err != nil {
		t.Fatal("original generation recovery missing", err)
	}
}
