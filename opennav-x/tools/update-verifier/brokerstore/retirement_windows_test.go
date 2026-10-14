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

func nativeRetirementDrainFixture(t *testing.T) (*nativeRetirement, []windows.Handle) {
	t.Helper()
	n, _ := nativeRegistrationFixture(t)
	binding := decommissionTestBinding()
	fileSD, err := descriptor(n.broker, nil, false)
	if err != nil {
		t.Fatal(err)
	}
	dirSD, err := descriptor(n.broker, nil, true)
	if err != nil {
		t.Fatal(err)
	}
	parent, err := openPin(physical(n.registrations.name), n.registrations.name, dirSD)
	if err != nil {
		t.Fatal(err)
	}
	r := &nativeRetirement{store: n, binding: binding, generationParent: parent, objects: make([]*retirementObject, 3)}
	t.Cleanup(func() {
		if err := r.close(); err != nil {
			t.Error(err)
		}
	})
	for i, suffix := range []string{"once", "json"} {
		name := binding.ClientSID + "." + suffix
		file, err := createRegistrationFile(n.registrations, name, fileSD)
		if err != nil {
			t.Fatal(err)
		}
		id, idErr := fileIdentity(file.pin.handle)
		if err := errors.Join(idErr, file.Close()); err != nil {
			t.Fatal(err)
		}
		r.objects[i], err = openRetirementObject(n.registrations, name, id, false, fileSD)
		if err != nil {
			t.Fatal(err)
		}
	}
	creator := &nativeCandidate{store: n, securityDir: dirSD}
	pin, err := creator.newDirectory(parent, binding.GenerationID)
	if err != nil {
		t.Fatal(err)
	}
	id, idErr := fileIdentity(pin.handle)
	if err := errors.Join(idErr, creator.close()); err != nil {
		t.Fatal(err)
	}
	r.objects[2], err = openRetirementObject(parent, binding.GenerationID, id, true, dirSD)
	if err != nil {
		t.Fatal(err)
	}
	children := make([]windows.Handle, 0, 2)
	for _, name := range []string{"app.exe", "launcher.exe"} {
		path := physical(parent.name + `\` + binding.GenerationID + `\` + name)
		if err := os.WriteFile(path, []byte(name), 0600); err != nil {
			t.Fatal(err)
		}
		pointer, err := windows.UTF16PtrFromString(path)
		if err != nil {
			t.Fatal(err)
		}
		// Sharing delete does not make directory rename safe while descendants
		// remain open. Keep the actual app/launcher handles until the callback.
		handle, err := windows.CreateFile(pointer, windows.GENERIC_READ, windows.FILE_SHARE_READ|windows.FILE_SHARE_DELETE, nil, windows.OPEN_EXISTING, windows.FILE_FLAG_OPEN_REPARSE_POINT, 0)
		if err != nil {
			t.Fatal(err)
		}
		children = append(children, handle)
	}
	t.Cleanup(func() {
		for _, handle := range children {
			if handle != 0 {
				if err := windows.CloseHandle(handle); err != nil {
					t.Error(err)
				}
			}
		}
	})
	return r, children
}

func TestNativeRetirementStartupDrainKeepsOriginalRootAndRefusesFailedHandoffs(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	for _, stage := range []string{"success", "guard before", "refused", "lost callback", "cancelled", "guard after", "changed terminal"} {
		t.Run(stage, func(t *testing.T) {
			r, children := nativeRetirementDrainFixture(t)
			originalHandles := [3]windows.Handle{r.objects[0].handle, r.objects[1].handle, r.objects[2].handle}
			originalIDs := [3]diskIdentity{r.objects[0].id, r.objects[1].id, r.objects[2].id}
			parent := r.generationParent
			// Preserve the real failure: FILE_SHARE_DELETE on descendants is
			// insufficient. No root handle is closed or reopened for recovery.
			if err := r.objects[2].move(context.Background(), "retired-blocked-fixture"); err == nil {
				t.Fatal("directory renamed while app/launcher descendants remained open")
			}
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			guards, callbacks := 0, 0
			guard := func(context.Context) error {
				guards++
				if err := r.terminalCheck(); err != nil {
					t.Fatal("terminal guard ran without durable marker and start", err)
				}
				if stage == "guard before" && guards == 1 || stage == "guard after" && guards == 2 {
					return testFault
				}
				return nil
			}
			err := r.retire(ctx, guard, func(callbackCtx context.Context, fence string) error {
				callbacks++
				if callbackCtx != ctx || fence != registrationDigest(r.markerRaw) || guards != 1 || r.completed != nil {
					t.Fatal("callback did not bind durable terminal boundary")
				}
				if !r.store.mu.TryLock() {
					t.Fatal("native Store mutex held during external drain")
				}
				r.store.mu.Unlock()
				for i, object := range r.objects {
					if object.handle != originalHandles[i] || object.id != originalIDs[i] {
						t.Fatal("original DELETE owner changed before callback")
					}
				}
				if err := r.terminalCheck(); err != nil {
					t.Fatal("callback preceded validated started record", err)
				}
				switch stage {
				case "refused":
					return testFault
				case "lost callback":
					return context.DeadlineExceeded
				case "cancelled":
					cancel()
				case "changed terminal":
					// Corrupt only the expected comparison bytes; native readback
					// must still refuse the actual held started FILE_OBJECT.
					r.startedRaw[0] ^= 1
				}
				for i, handle := range children {
					if err := windows.CloseHandle(handle); err != nil {
						return err
					}
					children[i] = 0
				}
				return nil
			})
			if r.generationParent != parent {
				t.Fatal("original parent custody replaced")
			}
			for i, object := range r.objects {
				if object.handle != originalHandles[i] || object.id.Volume != originalIDs[i].Volume || object.id.FileID != originalIDs[i].FileID {
					t.Fatal("original DELETE handle or file identity replaced")
				}
			}
			if stage == "success" {
				if err != nil || callbacks != 1 || guards != 2 {
					t.Fatal("original root rename after descendant drain", err)
				}
				if err := r.check(); err != nil {
					t.Fatal("completed retirement evidence", err)
				}
				for _, name := range []string{"app.exe", "launcher.exe"} {
					content, err := os.ReadFile(physical(parent.name + `\` + retirementGenerationName(r.binding) + `\` + name))
					if err != nil || string(content) != name {
						t.Fatal("retired descendant bytes changed", err)
					}
				}
			} else {
				if !errors.Is(err, ErrRetirementUncertain) || r.completed != nil || r.completedMove || r.marker == nil || r.started == nil {
					t.Fatal("failed terminal handoff lost recovery custody", err)
				}
				for i, object := range r.objects {
					if object.id != originalIDs[i] {
						t.Fatal("failed callback moved original object")
					}
					if err := object.check(); err != nil {
						t.Fatal("original name or native custody changed", err)
					}
				}
				if _, err := os.Stat(filepath.Join(r.store.rootDOS, retirementCompletedName(r.binding))); !errors.Is(err, os.ErrNotExist) {
					t.Fatal("failed handoff published completed evidence", err)
				}
			}
			wantCallbacks := 1
			if stage == "guard before" {
				wantCallbacks = 0
			}
			if callbacks != wantCallbacks {
				t.Fatal("callback count differs", callbacks)
			}
		})
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
