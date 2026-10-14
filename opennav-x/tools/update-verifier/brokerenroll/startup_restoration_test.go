package brokerenroll

import (
	"context"
	"encoding/json"
	"errors"
	"reflect"
	"runtime"
	"strings"
	"testing"

	"example.com/opennav-update-verifier/brokerpipe"
	"example.com/opennav-update-verifier/brokerregistration"
)

type startupRestorationFixture struct {
	d                              *decommissionFixture
	checks, moves, closes          int
	failMove                       int
	foreign, collision, closeFault bool
}

type preparedRestorationFixture struct {
	*startupRestorationFixture
	bound, bindFault bool
	binds            int
	drains           int
	drainFault       bool
}

func (s *preparedRestorationFixture) bindRetirement(context.Context, brokerpipe.DecommissionSnapshot) error {
	s.binds++
	s.d.events = append(s.d.events, "bind-retirement")
	if s.bindFault {
		return errors.New("same retained launcher is outside fixed retirement")
	}
	s.bound = true
	return nil
}
func (s *preparedRestorationFixture) restore(ctx context.Context, guard func(context.Context) error) error {
	if s.bound {
		return errors.New("restoration cannot follow retirement binding")
	}
	return s.startupRestorationFixture.restore(ctx, guard)
}
func (s *preparedRestorationFixture) drain(ctx context.Context, guard func(context.Context) error) error {
	if s.moves != 4 || s.drains != 0 {
		return errors.New("original restoration incomplete or drain repeated")
	}
	s.drains++
	s.d.events = append(s.d.events, "drain-startup")
	if err := guard(ctx); err != nil {
		return err
	}
	if s.drainFault {
		return errors.New("original descendant close uncertain")
	}
	return ctx.Err()
}

func prepareRestorationFixture(t *testing.T, owner *MaintenanceOwner, fixture *preparedRestorationFixture) error {
	t.Helper()
	approved := approvedRestorationFixture(t, owner)
	return prepareStartupRestorationWith(owner, context.Background(), approved, func(ctx context.Context, authority maintenanceAuthority, record startupBindingRecord, raw []byte) (startupRestorationBackend, error) {
		if record.ClientRequestSHA256 != authority.client.RequestSHA256 || digest(raw) != approved.SHA256() {
			t.Fatal("original startup preparation lost approved identity")
		}
		fixture.d.events = append(fixture.d.events, "prepare-startup")
		return fixture, nil
	})
}

func preparedRetirementSnapshot(t *testing.T, owner *MaintenanceOwner) brokerpipe.DecommissionSnapshot {
	t.Helper()
	raw, err := brokerregistration.EncodeProtectedRecord(owner.authority.client)
	if err != nil {
		t.Fatal(err)
	}
	return brokerpipe.DecommissionSnapshot{RequestID: strings.Repeat("1", 32), ClientSID: owner.authority.client.ClientSID,
		RegistrationSHA256: digest(raw), Generation: owner.authority.client.Installed.Generation, RetirementSHA256: strings.Repeat("a", 64),
		CompletionSHA256: strings.Repeat("b", 64), SelectedHeadSHA256: strings.Repeat("c", 64), CurrentProfileSHA256: strings.Repeat("d", 64),
		CurrentProfileIdentitySHA256: strings.Repeat("e", 64), FenceSHA256: strings.Repeat("f", 64), SourcePID: owner.pid, SourceCreation: 42}
}

func restorePreparedFixture(t *testing.T, owner *MaintenanceOwner, snapshot brokerpipe.DecommissionSnapshot, events *[]string) error {
	t.Helper()
	snapshot.RetirementSHA256 = ""
	return restoreStartupAndDrainWith(owner, context.Background(), &decommissionFenceFixture{events: events}, snapshot)
}

func TestPreparedStartupRestoresAndDrainsBeforeRetirementAndSCM(t *testing.T) {
	d := newDecommissionFixture(t)
	owner, err := openMaintenanceModeWith(context.Background(), d.req, d.op, true)
	if err != nil {
		t.Fatal(err)
	}
	defer owner.Close()
	s := &preparedRestorationFixture{startupRestorationFixture: &startupRestorationFixture{d: d}}
	if err := prepareRestorationFixture(t, owner, s); err != nil {
		t.Fatal(err)
	}
	if err := owner.Check(context.Background()); err != nil {
		t.Fatal(err)
	}
	if _, err := owner.Decommission(context.Background()); err == nil || d.disables != 0 || s.moves != 0 || s.binds != 0 {
		t.Fatal("unbound preparation performed namespace/SCM effects")
	}
	witness := &decommissionFenceFixture{events: &d.events}
	snapshot := preparedRetirementSnapshot(t, owner)
	if err := restorePreparedFixture(t, owner, snapshot, &d.events); err != nil {
		t.Fatal(err)
	}
	if err := bindDecommissionSnapshotWith(owner, context.Background(), witness, snapshot.ClientSID, snapshot.RegistrationSHA256, &snapshot); err != nil {
		t.Fatal(err)
	}
	if s.binds != 1 || s.moves != 4 || s.drains != 1 || d.disables != 0 || !owner.startupBound {
		t.Fatal("binding did not only finalize original startup custody")
	}
	if _, err := owner.Decommission(context.Background()); err != nil {
		t.Fatal(err)
	}
	prepared, bound, lastMove, disable := -1, -1, -1, -1
	for i, event := range d.events {
		switch event {
		case "prepare-startup":
			prepared = i
		case "bind-retirement":
			bound = i
		case "shortcut-move":
			lastMove = i
		case "disable":
			disable = i
		}
	}
	if prepared < 0 || lastMove <= prepared || bound <= lastMove || disable <= bound || s.binds != 1 || s.moves != 4 || s.drains != 1 || s.closes != 1 {
		t.Fatal("original prepared ownership or effect order lost", d.events)
	}
}

func TestPreparedStartupRefusalsConsumeOwnerWithoutSCMOrFallback(t *testing.T) {
	for _, fault := range []string{"no-snapshot", "service-only", "wrong-generation", "bind", "witness", "zero-native-witness", "no-preparation", "no-drain", "wrong-fence", "wrong-head", "wrong-process"} {
		t.Run(fault, func(t *testing.T) {
			d := newDecommissionFixture(t)
			owner, err := openMaintenanceModeWith(context.Background(), d.req, d.op, true)
			if err != nil {
				t.Fatal(err)
			}
			defer owner.Close()
			s := &preparedRestorationFixture{startupRestorationFixture: &startupRestorationFixture{d: d}, bindFault: fault == "bind"}
			if fault != "no-preparation" {
				if err := prepareRestorationFixture(t, owner, s); err != nil {
					t.Fatal(err)
				}
			}
			witness := &decommissionFenceFixture{events: &d.events}
			snapshot := preparedRetirementSnapshot(t, owner)
			if fault != "no-preparation" && fault != "no-drain" {
				if err := restorePreparedFixture(t, owner, snapshot, &d.events); err != nil {
					t.Fatal(err)
				}
			}
			movesBeforeBinding := s.moves
			receipt := &snapshot
			switch fault {
			case "no-snapshot":
				receipt = nil
			case "service-only":
				snapshot.RetirementSHA256 = ""
			case "wrong-generation":
				snapshot.Generation = strings.Repeat("f", 32)
			case "witness":
				witness.failAt = 1
			case "wrong-fence":
				snapshot.FenceSHA256 = strings.Repeat("a", 64)
			case "wrong-head":
				snapshot.SelectedHeadSHA256 = strings.Repeat("b", 64)
			case "wrong-process":
				snapshot.SourceCreation++
			}
			if fault == "zero-native-witness" {
				err = owner.BindDecommission(context.Background(), &brokerpipe.DecommissionWitness{})
			} else {
				err = bindDecommissionSnapshotWith(owner, context.Background(), witness, snapshot.ClientSID, snapshot.RegistrationSHA256, receipt)
			}
			if err == nil {
				t.Fatal("invalid prepared retirement bound")
			}
			if _, err := owner.Decommission(context.Background()); err == nil || d.disables != 0 || s.moves != movesBeforeBinding {
				t.Fatal("failed prepared binding bypassed by service-only decommission")
			}
			if err := bindRestorationFixture(t, owner, s.startupRestorationFixture); err == nil {
				t.Fatal("failed preparation fell back to a post-retirement path opener")
			}
		})
	}
}

func TestPreparedStartupMissingOrChangedOriginalNeverAcquiresFence(t *testing.T) {
	for _, fault := range []string{"nil-approval", "open", "incomplete-backend", "foreign"} {
		t.Run(fault, func(t *testing.T) {
			d := newDecommissionFixture(t)
			owner, err := openMaintenanceModeWith(context.Background(), d.req, d.op, true)
			if err != nil {
				t.Fatal(err)
			}
			defer owner.Close()
			s := &preparedRestorationFixture{startupRestorationFixture: &startupRestorationFixture{d: d, foreign: fault == "foreign"}}
			approved := approvedRestorationFixture(t, owner)
			if fault == "nil-approval" {
				approved = nil
			}
			err = prepareStartupRestorationWith(owner, context.Background(), approved, func(context.Context, maintenanceAuthority, startupBindingRecord, []byte) (startupRestorationBackend, error) {
				if fault == "open" {
					return s, errors.New("partial original open")
				}
				if fault == "incomplete-backend" {
					return s.startupRestorationFixture, nil
				}
				return s, nil
			})
			if err == nil || !owner.failed || owner.fence != nil || d.disables != 0 {
				t.Fatal("failed original preparation was not revoked")
			}
			if err := prepareRestorationFixture(t, owner, s); err == nil {
				t.Fatal("preparation retried")
			}
			if err := owner.Close(); err != nil {
				t.Fatal(err)
			}
			if fault != "nil-approval" && s.closes != 1 {
				t.Fatal("partial prepared owner not drained once")
			}
		})
	}
}

func TestPreparedStartupInterimRestoreDrainFaultsNeverReachSCMOrRetry(t *testing.T) {
	for _, fault := range []string{"nil-native", "zero-native", "wrong-registration", "wrong-process", "wrong-fence", "completed-interim", "witness", "partial-restoration", "descendant-close", "cancelled"} {
		t.Run(fault, func(t *testing.T) {
			d := newDecommissionFixture(t)
			owner, err := openMaintenanceModeWith(context.Background(), d.req, d.op, true)
			if err != nil {
				t.Fatal(err)
			}
			defer owner.Close()
			s := &preparedRestorationFixture{startupRestorationFixture: &startupRestorationFixture{d: d}, drainFault: fault == "descendant-close"}
			if fault == "partial-restoration" {
				s.failMove = 2
			}
			if err := prepareRestorationFixture(t, owner, s); err != nil {
				t.Fatal(err)
			}
			final := preparedRetirementSnapshot(t, owner)
			interim := final
			interim.RetirementSHA256 = ""
			fence := &decommissionFenceFixture{events: &d.events}
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			switch fault {
			case "wrong-registration":
				interim.RegistrationSHA256 = strings.Repeat("0", 64)
			case "wrong-process":
				interim.SourcePID++
			case "wrong-fence":
				interim.FenceSHA256 = ""
			case "completed-interim":
				interim.RetirementSHA256 = final.RetirementSHA256
			case "witness":
				fence.failAt = 1
			case "cancelled":
				cancel()
			}
			if fault == "nil-native" {
				err = owner.RestoreStartupAndDrain(ctx, nil)
			} else if fault == "zero-native" {
				err = owner.RestoreStartupAndDrain(ctx, &brokerpipe.UninstallFenceWitness{})
			} else {
				err = restoreStartupAndDrainWith(owner, ctx, fence, interim)
			}
			if err == nil || !owner.failed || owner.startupDrained || d.disables != 0 || d.stops != 0 || d.deletes != 0 {
				t.Fatal("failed original handoff reached service effects", err)
			}
			moves, drains := s.moves, s.drains
			if fault == "partial-restoration" && (moves != 2 || drains != 0) || fault == "descendant-close" && (moves != 4 || drains != 1) {
				t.Fatal("partial restoration/close failure evidence lost")
			}
			if err := restoreStartupAndDrainWith(owner, context.Background(), fence, interim); err == nil || s.moves != moves || s.drains != drains {
				t.Fatal("failed startup handoff retried")
			}
			if err := bindDecommissionSnapshotWith(owner, context.Background(), &decommissionFenceFixture{events: &d.events}, final.ClientSID, final.RegistrationSHA256, &final); err == nil {
				t.Fatal("final retirement adopted failed handoff")
			}
			if _, err := owner.Decommission(context.Background()); err == nil || d.disables != 0 || d.stops != 0 || d.deletes != 0 {
				t.Fatal("failed handoff fell back to service-only")
			}
		})
	}
}

func TestPreparedStartupDrainStillWithholdsSCMUntilFinalMatchingRetirement(t *testing.T) {
	d := newDecommissionFixture(t)
	owner, err := openMaintenanceModeWith(context.Background(), d.req, d.op, true)
	if err != nil {
		t.Fatal(err)
	}
	defer owner.Close()
	s := &preparedRestorationFixture{startupRestorationFixture: &startupRestorationFixture{d: d}}
	if err := prepareRestorationFixture(t, owner, s); err != nil {
		t.Fatal(err)
	}
	snapshot := preparedRetirementSnapshot(t, owner)
	if err := restorePreparedFixture(t, owner, snapshot, &d.events); err != nil {
		t.Fatal(err)
	}
	if _, err := owner.Decommission(context.Background()); err == nil || d.disables != 0 || d.stops != 0 || d.deletes != 0 || owner.fence != nil {
		t.Fatal("interim startup drain authorized SCM")
	}
	if err := bindDecommissionSnapshotWith(owner, context.Background(), &decommissionFenceFixture{events: &d.events}, snapshot.ClientSID, snapshot.RegistrationSHA256, &snapshot); err != nil {
		t.Fatal(err)
	}
	if _, err := owner.Decommission(context.Background()); err != nil || s.moves != 4 || s.drains != 1 || d.disables != 1 || d.stops != 1 || d.deletes != 1 {
		t.Fatal("same final retirement failed or repeated restoration", err)
	}
}

func (s *startupRestorationFixture) check(context.Context) error {
	s.checks++
	s.d.events = append(s.d.events, "shortcut-check")
	if s.foreign {
		return errors.New("same-byte foreign shortcut identity")
	}
	return nil
}
func (s *startupRestorationFixture) restore(ctx context.Context, guard func(context.Context) error) error {
	return startupRestoreSequence(ctx, true, func() error {
		if err := guard(ctx); err != nil {
			return err
		}
		if s.collision {
			return errors.New("retirement destination collision")
		}
		return s.check(ctx)
	}, func(step int) error {
		if err := guard(ctx); err != nil {
			return err
		}
		if err := s.check(ctx); err != nil {
			return err
		}
		s.moves++
		s.d.events = append(s.d.events, "shortcut-move")
		if s.d.disabled || s.d.stops != 0 || s.d.deletes != 0 {
			panic("shortcut mutation after broker shutdown began")
		}
		if s.failMove == s.moves {
			return errors.New("rename may have completed")
		}
		return guard(ctx)
	})
}
func (s *startupRestorationFixture) close() error {
	s.closes++
	s.d.events = append(s.d.events, "close-shortcuts")
	if s.closeFault {
		return errors.New("shortcut custody close uncertain")
	}
	return nil
}

func approvedRestorationFixture(t *testing.T, owner *MaintenanceOwner) *ApprovedStartupPublication {
	t.Helper()
	record, _ := startupBindingFixture(t)
	request := owner.authority.client
	record.ClientSID = request.ClientSID
	record.ClientRequestSHA256 = request.RequestSHA256
	record.Generation, record.Commit, record.PackageSHA256 = request.Installed.Generation, request.Installed.Commit, request.Installed.PackageSHA256
	raw, err := json.Marshal(record)
	if err != nil {
		t.Fatal(err)
	}
	approved, err := ParseApprovedStartupPublication(raw, digest(raw), request.OriginalBytes(), request.RequestSHA256)
	if err != nil {
		t.Fatal(err)
	}
	return approved
}

func bindRestorationFixture(t *testing.T, owner *MaintenanceOwner, fixture *startupRestorationFixture) error {
	t.Helper()
	approved := approvedRestorationFixture(t, owner)
	return bindStartupRestorationWith(owner, context.Background(), approved, func(ctx context.Context, authority maintenanceAuthority, record startupBindingRecord, raw []byte) (startupRestorationBackend, error) {
		if record.ClientRequestSHA256 != authority.client.RequestSHA256 || digest(raw) != approved.SHA256() {
			t.Fatal("original approved request/publication binding lost")
		}
		return fixture, nil
	})
}

func TestStartupRestorationRunsBeforeEverySCMEffect(t *testing.T) {
	d := newDecommissionFixture(t)
	owner, err := openMaintenanceModeWith(context.Background(), d.req, d.op, true)
	if err != nil {
		t.Fatal(err)
	}
	witness := &decommissionFenceFixture{events: &d.events}
	if err := bindDecommissionFenceWith(owner, context.Background(), witness, inertClientSID); err != nil {
		t.Fatal(err)
	}
	s := &startupRestorationFixture{d: d}
	if err := bindRestorationFixture(t, owner, s); err != nil {
		t.Fatal(err)
	}
	if s.moves != 0 || d.disables != 0 {
		t.Fatal("binding mutated shortcuts or SCM")
	}
	result, err := owner.Decommission(context.Background())
	if err != nil || result.Service != ServiceMarkedForRemoval || s.moves != 4 || s.closes != 1 {
		t.Fatal(result, err, d.events)
	}
	lastMove, disable := -1, -1
	for i, event := range d.events {
		if event == "shortcut-move" {
			lastMove = i
		}
		if event == "disable" {
			disable = i
		}
	}
	if lastMove < 0 || disable <= lastMove || witness.checks < 12 {
		t.Fatal("original witness not retained across moves before disable", d.events)
	}
	if _, err := owner.Decommission(context.Background()); err == nil || s.moves != 4 || d.deletes != 1 {
		t.Fatal("restoration/decommission replayed")
	}
}

func TestStartupRestorationFailuresBlockSCMAndReplay(t *testing.T) {
	for _, fault := range []string{"substitution-before-bind", "substitution-after-bind", "collision", "move-1", "move-3", "restore-original", "witness"} {
		t.Run(fault, func(t *testing.T) {
			d := newDecommissionFixture(t)
			owner, err := openMaintenanceModeWith(context.Background(), d.req, d.op, true)
			if err != nil {
				t.Fatal(err)
			}
			witness := &decommissionFenceFixture{events: &d.events}
			if err := bindDecommissionFenceWith(owner, context.Background(), witness, inertClientSID); err != nil {
				t.Fatal(err)
			}
			s := &startupRestorationFixture{d: d, foreign: fault == "substitution-before-bind"}
			bindErr := bindRestorationFixture(t, owner, s)
			if fault == "substitution-before-bind" {
				if bindErr == nil {
					t.Fatal("foreign original bound")
				}
			} else {
				if bindErr != nil {
					t.Fatal(bindErr)
				}
				switch fault {
				case "substitution-after-bind":
					s.foreign = true
				case "collision":
					s.collision = true
				case "move-1":
					s.failMove = 1
				case "move-3":
					s.failMove = 3
				case "restore-original":
					s.failMove = 4
				case "witness":
					witness.failAt = witness.checks + 3
				}
			}
			result, err := owner.Decommission(context.Background())
			if err == nil || result != (DecommissionResult{}) || d.disables != 0 || d.stops != 0 || d.deletes != 0 {
				t.Fatal("failed restoration allowed service mutation", result, err, d.events)
			}
			moves := s.moves
			if _, err := owner.Decommission(context.Background()); err == nil || s.moves != moves {
				t.Fatal("partial restoration retried")
			}
			_ = owner.Close()
			if s.closes != 1 || witness.closes != 1 {
				t.Fatal("failed original owners not drained once")
			}
		})
	}
}

func TestStartupRestorationRequiresExactOriginalApprovalAndOwner(t *testing.T) {
	for _, fault := range []string{"nil", "zero", "wrong-request", "no-fence", "copied-owner"} {
		t.Run(fault, func(t *testing.T) {
			d := newDecommissionFixture(t)
			owner, err := openMaintenanceModeWith(context.Background(), d.req, d.op, true)
			if err != nil {
				t.Fatal(err)
			}
			defer owner.Close()
			if fault != "no-fence" {
				if err := bindDecommissionFenceWith(owner, context.Background(), &decommissionFenceFixture{events: &d.events}, inertClientSID); err != nil {
					t.Fatal(err)
				}
			}
			approved := approvedRestorationFixture(t, owner)
			switch fault {
			case "nil":
				approved = nil
			case "zero":
				approved = &ApprovedStartupPublication{}
			case "wrong-request":
				record, request := startupBindingFixture(t)
				raw, _ := json.Marshal(record)
				approved, err = ParseApprovedStartupPublication(raw, digest(raw), request, digest(request))
				if err != nil {
					t.Fatal(err)
				}
			case "copied-owner":
				var copy MaintenanceOwner
				reflect.ValueOf(&copy).Elem().Set(reflect.ValueOf(owner).Elem())
				owner = &copy
			}
			opens := 0
			err = bindStartupRestorationWith(owner, context.Background(), approved, func(context.Context, maintenanceAuthority, startupBindingRecord, []byte) (startupRestorationBackend, error) {
				opens++
				return nil, nil
			})
			if err == nil || opens != 0 || d.disables != 0 {
				t.Fatal("unapproved/copy/unfenced input opened native mutation owners", err)
			}
			if fault != "no-fence" && fault != "copied-owner" {
				if _, err := owner.Decommission(context.Background()); err == nil || d.disables != 0 {
					t.Fatal("failed binding bypassed by service-only path")
				}
			}
		})
	}
}

func TestStartupRestorationSequenceWithoutOriginalAndPartialFailures(t *testing.T) {
	for _, original := range []bool{false, true} {
		for fail := -1; fail < 4; fail++ {
			moves := []int{}
			err := startupRestoreSequence(context.Background(), original, func() error {
				if fail == -1 {
					return errors.New("foreign target")
				}
				return nil
			}, func(step int) error {
				moves = append(moves, step)
				if step == fail {
					return errors.New("move failed")
				}
				return nil
			})
			count := 3
			if original {
				count++
			}
			if fail == -1 {
				if err == nil || len(moves) != 0 || errors.Is(err, ErrStartupRestorationUncertain) {
					t.Fatal("preflight mutation/uncertainty")
				}
			} else if fail < count {
				if !errors.Is(err, ErrStartupRestorationUncertain) || len(moves) != fail+1 {
					t.Fatal("partial move continued or lost uncertainty")
				}
			} else if err != nil || len(moves) != count {
				t.Fatal("no-original transaction failed")
			}
		}
	}
}

func TestStartupRestorationCloseFailureSuppressesSuccess(t *testing.T) {
	d := newDecommissionFixture(t)
	owner, err := openMaintenanceModeWith(context.Background(), d.req, d.op, true)
	if err != nil {
		t.Fatal(err)
	}
	if err := bindDecommissionFenceWith(owner, context.Background(), &decommissionFenceFixture{events: &d.events}, inertClientSID); err != nil {
		t.Fatal(err)
	}
	s := &startupRestorationFixture{d: d, closeFault: true}
	if err := bindRestorationFixture(t, owner, s); err != nil {
		t.Fatal(err)
	}
	result, err := owner.Decommission(context.Background())
	if result != (DecommissionResult{}) || !errors.Is(err, ErrCleanup) || !errors.Is(err, ErrMaintenanceUncertain) || s.closes != 1 {
		t.Fatal("uncertain close reported success", result, err)
	}
	if owner.Close() == nil || s.closes != 1 {
		t.Fatal("close failure lost or repeated")
	}
}

func TestStartupRestorationChangedAfterStopBlocksServiceDeletion(t *testing.T) {
	d := newDecommissionFixture(t)
	owner, err := openMaintenanceModeWith(context.Background(), d.req, d.op, true)
	if err != nil {
		t.Fatal(err)
	}
	defer owner.Close()
	if err := bindDecommissionFenceWith(owner, context.Background(), &decommissionFenceFixture{events: &d.events}, inertClientSID); err != nil {
		t.Fatal(err)
	}
	s := &startupRestorationFixture{d: d}
	if err := bindRestorationFixture(t, owner, s); err != nil {
		t.Fatal(err)
	}
	d.afterStop = func() { s.foreign = true }
	result, err := owner.Decommission(context.Background())
	if result != (DecommissionResult{}) || !errors.Is(err, ErrMaintenanceUncertain) || s.moves != 4 || d.disables != 1 || d.stops != 1 || d.deletes != 0 {
		t.Fatal("changed shortcut custody allowed service deletion", result, err, d.events)
	}
}

func TestDecommissionWitnessRequiresExactOriginalRequest(t *testing.T) {
	d := newDecommissionFixture(t)
	owner, err := openMaintenanceModeWith(context.Background(), d.req, d.op, true)
	if err != nil {
		t.Fatal(err)
	}
	defer owner.Close()
	witness := &decommissionFenceFixture{events: &d.events}
	if err := bindDecommissionFenceIdentityWith(owner, context.Background(), witness, inertClientSID, digest([]byte("foreign request"))); err == nil {
		t.Fatal("same-client foreign request bound")
	}
	if _, err := owner.Decommission(context.Background()); err == nil || d.disables != 0 {
		t.Fatal("foreign request witness changed SCM")
	}
}

func TestStartupRestorationCheckRevalidatesBoundObjects(t *testing.T) {
	d := newDecommissionFixture(t)
	owner, err := openMaintenanceModeWith(context.Background(), d.req, d.op, true)
	if err != nil {
		t.Fatal(err)
	}
	defer owner.Close()
	if err := bindDecommissionFenceWith(owner, context.Background(), &decommissionFenceFixture{events: &d.events}, inertClientSID); err != nil {
		t.Fatal(err)
	}
	s := &startupRestorationFixture{d: d}
	if err := bindRestorationFixture(t, owner, s); err != nil {
		t.Fatal(err)
	}
	s.foreign = true
	if err := owner.Check(context.Background()); err == nil {
		t.Fatal("owner Check ignored bound shortcut identity change")
	}
	if _, err := owner.Decommission(context.Background()); err == nil || s.moves != 0 || d.disables != 0 {
		t.Fatal("revoked shortcut Check allowed mutation")
	}
}

func TestStartupRestorationPublicNativeBoundary(t *testing.T) {
	if runtime.GOOS == "windows" {
		t.Skip("portable refusal only")
	}
	d := newDecommissionFixture(t)
	owner, err := openMaintenanceModeWith(context.Background(), d.req, d.op, true)
	if err != nil {
		t.Fatal(err)
	}
	defer owner.Close()
	if err := bindDecommissionFenceWith(owner, context.Background(), &decommissionFenceFixture{events: &d.events}, inertClientSID); err != nil {
		t.Fatal(err)
	}
	if err := owner.BindStartupRestoration(context.Background(), approvedRestorationFixture(t, owner)); err == nil {
		t.Fatal("portable mock gained native restoration authority")
	}
}
