//go:build windows

package brokerstore

import (
	"context"
	"encoding/json"
	"errors"
	"os"
	"path/filepath"
	"runtime"
	"strings"
	"testing"

	"golang.org/x/sys/windows"
)

// Fresh inert native B0/B1 completion/seal fixtures. The host's qualified
// selection reducer is tested at its own boundary; these comparisons alone
// cannot establish successful execution, selection or installed acceptance.
func nativeRetirementLineageFixture(t *testing.T, seal bool) (*nativeStageSealFixtureStore, DecommissionBinding, []RetirementGeneration) {
	t.Helper()
	s, previous, view, fixture, approved := nativeBaselineViewFixture(t)
	view.nativeStore.candidates = view.candidates
	wrapper := &nativeStageSealFixtureStore{&nativeGenerationReopenFixtureStore{view}}
	s.backend = wrapper
	baseline, err := s.MaterializeApprovedBaselineReadOnlyFresh(context.Background(), previous, approved, registrationDigest(approved))
	if err != nil {
		t.Fatal(err)
	}
	b0, err := baseline.Completion()
	if err != nil {
		t.Fatal(err)
	}
	if err := baseline.Close(); err != nil {
		t.Fatal(err)
	}
	candidate := nativeHistoryCandidate(t, s, previous, fixture, strings.Repeat("9", 32))
	generation, err := s.MaterializeCandidateReadOnlyFresh(context.Background(), previous, candidate)
	if err != nil {
		t.Fatal(err)
	}
	b1, err := generation.Completion()
	if err != nil {
		t.Fatal(err)
	}
	if err := generation.Close(); err != nil {
		t.Fatal(err)
	}
	if err := previous.Close(); err != nil {
		t.Fatal(err)
	}
	if seal {
		if err := candidate.Seal(context.Background(), []byte("original completed B1 stage receipt\n")); err != nil {
			t.Fatal(err)
		}
	}
	if err := candidate.Close(); err != nil {
		t.Fatal(err)
	}
	sd, err := descriptor(view.nativeStore.broker, nil, false)
	if err != nil {
		t.Fatal(err)
	}
	registration, err := readCompletedRegistration(view.nativeStore.registrations, view.sid.String(), sd)
	if err != nil {
		t.Fatal(err)
	}
	binding := decommissionTestBinding()
	binding.ClientSID, binding.RegistrationSHA256 = view.sid.String(), registrationDigest(registration)
	binding.GenerationID, binding.CompletionSHA256 = b1.Generation, b1.CompletionSHA256
	return wrapper, binding, []RetirementGeneration{{b0.Generation, b0.CompletionSHA256}, {b1.Generation, b1.CompletionSHA256}}
}

func TestNativeRetirementCompletedLineageKeepsOriginalRootsAcrossDrainAndJournal(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	for _, stage := range []string{"selected B1", "selected B0 after rollback", "lost callback", "cancelled callback", "partial final move"} {
		t.Run(stage, func(t *testing.T) {
			wrapper, binding, lineage := nativeRetirementLineageFixture(t, true)
			if stage == "selected B0 after rollback" {
				binding.GenerationID, binding.CompletionSHA256 = lineage[0].GenerationID, lineage[0].CompletionSHA256
			}
			n := wrapper.nativeStore
			root := physical(wrapper.applications.name + `\` + binding.ClientSID + `\generations`)
			for _, generation := range lineage {
				if err := os.WriteFile(filepath.Join(root, generation.GenerationID, "unknown-addition.txt"), []byte("preserved-"+generation.GenerationID), 0600); err != nil {
					t.Fatal(err)
				}
			}
			// The operator retains B0's root and only drains its descendants.
			// Both the broker's eventual DELETE root and this root survive drain.
			operator := make([]windows.Handle, 3)
			for i, suffix := range []string{"", "app", `app\opencpn.exe`} {
				path := filepath.Join(root, lineage[0].GenerationID)
				if suffix != "" {
					path = filepath.Join(path, suffix)
				}
				pointer, err := windows.UTF16PtrFromString(path)
				if err != nil {
					t.Fatal(err)
				}
				access, flags := uint32(windows.FILE_LIST_DIRECTORY|windows.FILE_READ_ATTRIBUTES|windows.READ_CONTROL), uint32(windows.FILE_FLAG_OPEN_REPARSE_POINT|windows.FILE_FLAG_BACKUP_SEMANTICS)
				if i == 2 {
					access, flags = windows.GENERIC_READ|windows.READ_CONTROL, windows.FILE_FLAG_OPEN_REPARSE_POINT
				}
				operator[i], err = windows.CreateFile(pointer, access, windows.FILE_SHARE_READ|windows.FILE_SHARE_DELETE, nil, windows.OPEN_EXISTING, flags, 0)
				if err != nil {
					t.Fatal(err)
				}
			}
			t.Cleanup(func() {
				for _, handle := range operator {
					if handle != 0 {
						if err := windows.CloseHandle(handle); err != nil {
							t.Error(err)
						}
					}
				}
			})
			backend, err := n.prepareRetirementLineageWithOpeners(context.Background(), binding, lineage, decommissionInventoryOpeners{wrapper.openReadOnlyGeneration, wrapper.openStageSeal})
			if err != nil {
				t.Fatal("original completed lineage preparation", err)
			}
			p := backend.(*nativeRetirementPreparation)
			defer p.close()
			original := append([]retirementGenerationRecord(nil), p.lineage...)
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			callbacks := 0
			owner, err := p.commitWithSelectedDescriptorAndStartupDrain(ctx, p.generationSecurity, func(ctx context.Context) error { return ctx.Err() }, func(callbackCtx context.Context, fence string) error {
				callbacks++
				if callbackCtx != ctx || fence != registrationDigest(decommissionBytes(binding)) || !p.transferred {
					return errors.New("original terminal drain binding changed")
				}
				raw, err := readRetirementFixtureRecord(filepath.Join(n.rootDOS, retirementStartedName(binding)))
				if err != nil {
					return errors.Join(errors.New("durable lineage start missing before drain"), err)
				}
				var start retirementLineageStartRecord
				if json.Unmarshal(raw, &start) != nil || start.Schema != 2 || start.BaselineGeneration != lineage[0].GenerationID || len(start.Generations) != len(lineage) || start.Generation != binding.GenerationID {
					return errors.New("durable bounded lineage start differs")
				}
				for i, generation := range original {
					if start.Generations[i] != generation {
						return errors.New("original ordered completion/root identities lost")
					}
					if _, err := os.Stat(filepath.Join(root, generation.Generation)); err != nil {
						return errors.Join(errors.New("generation moved before callback"), err)
					}
				}
				if stage == "lost callback" {
					return context.DeadlineExceeded
				}
				for i := len(operator) - 1; i > 0; i-- {
					if err := windows.CloseHandle(operator[i]); err != nil {
						return err
					}
					operator[i] = 0
				}
				if stage == "cancelled callback" {
					cancel()
				}
				if stage == "partial final move" {
					collision := binding
					collision.GenerationID = lineage[1].GenerationID
					return os.Mkdir(filepath.Join(root, retirementGenerationName(collision)), 0700)
				}
				return nil
			})
			if owner == nil {
				t.Fatal("original lineage custody lost", err)
			}
			r := owner.(*nativeRetirement)
			defer r.close()
			if callbacks != 1 || len(r.objects) != len(lineage)+2 {
				t.Fatal("lineage did not retain one original owner per move")
			}
			failed := stage == "lost callback" || stage == "cancelled callback" || stage == "partial final move"
			if failed {
				if !errors.Is(err, ErrRetirementUncertain) || r.completed != nil || r.completedMove {
					t.Fatal("failed lineage emitted completion or lost uncertainty", err)
				}
			} else {
				if err != nil || r.check() != nil {
					t.Fatal("completed lineage retirement failed", err)
				}
				var completed retirementLineageCompleteRecord
				if json.Unmarshal(r.completedRaw, &completed) != nil || completed.Schema != 2 || completed.BaselineGeneration != lineage[0].GenerationID || completed.StartedSHA256 != registrationDigest(r.startedRaw) || completed.FenceSHA256 != registrationDigest(r.markerRaw) || len(completed.Generations) != len(lineage) {
					t.Fatal("completed lineage journal bindings lost")
				}
				for i, generation := range completed.Generations {
					if generation.Generation != original[i].Generation || generation.CompletionSHA256 != original[i].CompletionSHA256 || generation.Directory != r.objects[i+2].id {
						t.Fatal("completed recovery identities differ")
					}
				}
			}
			for i, generation := range original {
				object := r.objects[i+2]
				if object.id.Volume != generation.Directory.Volume || object.id.FileID != generation.Directory.FileID {
					t.Fatal("original lineage root was replaced")
				}
				moved := !failed || stage == "partial final move" && i == 0
				name := generation.Generation
				if moved {
					destination := binding
					destination.GenerationID = generation.Generation
					name = retirementGenerationName(destination)
				}
				if object.name != r.generationParent.name+`\`+name || object.check() != nil {
					t.Fatal("original partial/completed lineage location or custody differs")
				}
				content, err := os.ReadFile(filepath.Join(root, name, "unknown-addition.txt"))
				if err != nil || string(content) != "preserved-"+generation.Generation {
					t.Fatal("unknown lineage child changed or was removed", err)
				}
			}
			if _, err := p.commitWithSelectedDescriptorAndStartupDrain(context.Background(), p.generationSecurity, func(context.Context) error { return nil }, func(context.Context, string) error { t.Fatal("lineage callback retried"); return nil }); err == nil {
				t.Fatal("original lineage retirement was reusable")
			}
		})
	}
}

func TestNativeRetirementLineageRefusesPendingExtraPartialAndChangedOriginals(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	for _, fault := range []string{"baseline adapter", "missing baseline", "extra sealed candidate", "unsealed candidate", "wrong completion", "partial stage", "unknown root", "replaced baseline root", "replaced selected root"} {
		t.Run(fault, func(t *testing.T) {
			wrapper, binding, lineage := nativeRetirementLineageFixture(t, fault != "unsealed candidate")
			n := wrapper.nativeStore
			switch fault {
			case "baseline adapter":
				lineage = nil
			case "missing baseline":
				lineage = lineage[1:]
			case "extra sealed candidate":
				binding.GenerationID, binding.CompletionSHA256 = lineage[0].GenerationID, lineage[0].CompletionSHA256
				lineage = lineage[:1]
			case "wrong completion":
				lineage[0].CompletionSHA256 = digest("foreign baseline completion")
			case "partial stage":
				if err := os.Remove(filepath.Join(n.rootDOS, "registrations", binding.ClientSID+".stage."+lineage[1].GenerationID+".json")); err != nil {
					t.Fatal(err)
				}
			case "unknown root":
				root := physical(wrapper.applications.name + `\` + binding.ClientSID + `\generations\unknown`)
				if err := os.Mkdir(root, 0700); err != nil {
					t.Fatal(err)
				}
			case "replaced baseline root", "replaced selected root":
				generation := lineage[0].GenerationID
				if fault == "replaced selected root" {
					generation = lineage[1].GenerationID
				}
				root := physical(wrapper.applications.name + `\` + binding.ClientSID + `\generations\` + generation)
				if err := os.Rename(root, filepath.Join(n.rootDOS, "displaced-original-generation")); err != nil {
					t.Fatal(err)
				}
				if err := os.Mkdir(root, 0700); err != nil {
					t.Fatal(err)
				}
			}
			owner, err := n.prepareRetirementLineageWithOpeners(context.Background(), binding, lineage, decommissionInventoryOpeners{wrapper.openReadOnlyGeneration, wrapper.openStageSeal})
			if owner != nil {
				defer owner.close()
			}
			if err == nil || owner != nil {
				t.Fatal("unresolved or foreign retirement scope accepted")
			}
			if _, err := os.Stat(filepath.Join(n.rootDOS, decommissionFilename)); !errors.Is(err, os.ErrNotExist) {
				t.Fatal("refused lineage published terminal marker", err)
			}
		})
	}
}

// Test-only readback of a durable original record while its production writer
// remains held. All sharing must be granted to that original WRITE owner;
// os.ReadFile would refuse before exercising the retirement under test.
func readRetirementFixtureRecord(path string) (result []byte, resultErr error) {
	pointer, err := windows.UTF16PtrFromString(path)
	if err != nil {
		return nil, err
	}
	handle, err := windows.CreateFile(pointer, windows.GENERIC_READ, windows.FILE_SHARE_READ|windows.FILE_SHARE_WRITE|windows.FILE_SHARE_DELETE, nil, windows.OPEN_EXISTING, windows.FILE_FLAG_OPEN_REPARSE_POINT, 0)
	if err != nil {
		return nil, err
	}
	defer func() { resultErr = errors.Join(resultErr, windows.CloseHandle(handle)) }()
	var info windows.ByHandleFileInformation
	if err := windows.GetFileInformationByHandle(handle, &info); err != nil {
		return nil, err
	}
	if info.FileSizeHigh != 0 || info.FileSizeLow == 0 || info.FileSizeLow > 256<<10 || info.FileAttributes&(windows.FILE_ATTRIBUTE_DIRECTORY|windows.FILE_ATTRIBUTE_REPARSE_POINT) != 0 {
		return nil, errors.New("bounded original fixture record required")
	}
	result = make([]byte, info.FileSizeLow)
	var read uint32
	if err := windows.ReadFile(handle, result, &read, nil); err != nil || read != info.FileSizeLow {
		return nil, errors.Join(errors.New("complete original fixture record required"), err)
	}
	return result, nil
}
