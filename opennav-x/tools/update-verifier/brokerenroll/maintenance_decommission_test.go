package brokerenroll

import (
	"context"
	"errors"
	"reflect"
	"runtime"
	"testing"
)

type decommissionFixture struct {
	*maintenanceFixture
	process  decommissionProcessFixture
	disabled bool
	marked   bool
	disables int
	deletes  int
	fault    string
}

type decommissionProcessFixture struct{ *maintenanceProcessFixture }

func (decommissionProcessFixture) birth() uint64 { return 42 }

func newDecommissionFixture(t *testing.T) *decommissionFixture {
	t.Helper()
	f := newMaintenanceFixture(t)
	d := &decommissionFixture{maintenanceFixture: f, process: decommissionProcessFixture{&f.process}}
	f.op.openDecommission = func(ctx context.Context, authority maintenanceAuthority) (maintenanceBackend, error) {
		f.events = append(f.events, "open-decommission")
		if authority.brokerSHA != f.req.ApprovedBrokerSHA256 || authority.client.ClientSID != inertClientSID {
			t.Fatal("decommission lost approved original")
		}
		return d, nil
	}
	return d
}

func (d *decommissionFixture) bindProcess(pid uint32) (maintenanceProcess, error) {
	if _, err := d.maintenanceFixture.bindProcess(pid); err != nil {
		return nil, err
	}
	return d.process, nil
}

func (d *decommissionFixture) check(ctx context.Context) error {
	if err := d.maintenanceFixture.check(ctx); err != nil {
		return err
	}
	if d.marked {
		return errors.New("marked service cannot be rechecked as original")
	}
	return nil
}

func (d *decommissionFixture) disable(ctx context.Context, pid uint32) error {
	d.events = append(d.events, "disable")
	d.disables++
	if pid != 23 || d.state.state != 4 || d.process.closes != 0 || d.closes != 0 {
		panic("disable lacked original running owners")
	}
	d.disabled = true // Model an effect even when the API reports an error.
	if d.fault == "disable" {
		return errors.New("disable completion uncertain")
	}
	return nil
}

func (d *decommissionFixture) markRemoval(context.Context) error {
	d.events = append(d.events, "delete")
	d.deletes++
	if !d.disabled || d.state.state != serviceStopped || !d.process.exited || d.process.code != 0 || d.process.closes != 0 || d.closes != 0 {
		panic("delete preceded disable and original clean broker exit")
	}
	d.marked = true // Model DeleteService effect before an uncertain response.
	if d.fault == "delete" {
		return errors.New("DeleteService completion uncertain")
	}
	return nil
}

type decommissionFenceFixture struct {
	events *[]string
	checks int
	closes int
	failAt int
}

func (w *decommissionFenceFixture) Check(ctx context.Context, pid uint32, birth uint64) error {
	*w.events = append(*w.events, "fence")
	w.checks++
	if ctx == nil || ctx.Err() != nil || pid != 23 || birth != 42 || w.failAt == w.checks {
		return errors.New("original broker fence unavailable")
	}
	return nil
}

func (w *decommissionFenceFixture) Close() error {
	*w.events = append(*w.events, "close-fence")
	w.closes++
	return nil
}

func TestDecommissionRequiresOriginalFenceAndOrdersSingleEffects(t *testing.T) {
	d := newDecommissionFixture(t)
	owner, err := openMaintenanceModeWith(context.Background(), d.req, d.op, true)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := owner.Decommission(context.Background()); err == nil || d.disables != 0 {
		t.Fatal("unbound owner changed SCM")
	}
	if err := owner.Quiesce(context.Background()); err == nil || d.stops != 0 {
		t.Fatal("decommission owner allowed STOP before disable")
	}
	w := &decommissionFenceFixture{events: &d.events}
	if err := bindDecommissionFenceWith(owner, context.Background(), w, inertClientSID); err != nil {
		t.Fatal(err)
	}
	result, err := owner.Decommission(context.Background())
	if err != nil || result.Service != ServiceMarkedForRemoval || !d.disabled || !d.marked {
		t.Fatal("original SCM decommission did not finish as marked only", result, err)
	}
	if d.disables != 1 || d.stops != 1 || d.deletes != 1 || w.closes != 1 || d.process.closes != 1 || d.closes != 1 {
		t.Fatal("effects or owners were not single-use", d.events)
	}
	order := []string{"open-decommission", "bind", "fence", "disable", "fence", "stop", "delete", "close-fence", "close-process", "close-service"}
	position := 0
	for _, event := range d.events {
		if position < len(order) && event == order[position] {
			position++
		}
	}
	if position != len(order) {
		t.Fatal("decommission effects were reordered", d.events)
	}
	if second, err := owner.Decommission(context.Background()); err == nil || second != (DecommissionResult{}) || d.disables != 1 || d.deletes != 1 {
		t.Fatal("decommission replayed")
	}
	if err := owner.Close(); err != nil {
		t.Fatal(err)
	}
}

func TestDecommissionFenceAndEffectFaultsNeverAuthorizeSuccessOrRetry(t *testing.T) {
	for _, fault := range []string{"wrong-client", "fence-bind", "fence-before-disable", "disable", "fence-before-stop", "stop", "delete"} {
		t.Run(fault, func(t *testing.T) {
			d := newDecommissionFixture(t)
			owner, err := openMaintenanceModeWith(context.Background(), d.req, d.op, true)
			if err != nil {
				t.Fatal(err)
			}
			w := &decommissionFenceFixture{events: &d.events}
			sid := inertClientSID
			switch fault {
			case "wrong-client":
				sid = "S-1-5-21-1-1-1-999"
			case "fence-bind":
				w.failAt = 1
			case "fence-before-disable":
				w.failAt = 2
			case "fence-before-stop":
				w.failAt = 3
			case "disable", "delete":
				d.fault = fault
			case "stop":
				d.maintenanceFixture.fault = fault
			}
			bindErr := bindDecommissionFenceWith(owner, context.Background(), w, sid)
			if fault == "wrong-client" || fault == "fence-bind" {
				if bindErr == nil || d.disables != 0 || owner.Check(context.Background()) == nil {
					t.Fatal("invalid fence gained authority", bindErr)
				}
			} else {
				if bindErr != nil {
					t.Fatal(bindErr)
				}
				result, err := owner.Decommission(context.Background())
				if err == nil || result != (DecommissionResult{}) || owner.Check(context.Background()) == nil {
					t.Fatal("uncertain decommission reported success", result, err)
				}
				if (fault == "fence-before-disable" && errors.Is(err, ErrMaintenanceUncertain)) ||
					(fault != "fence-before-disable" && !errors.Is(err, ErrMaintenanceUncertain)) {
					t.Fatal("incorrect effect uncertainty", err)
				}
			}
			disables, stops, deletes := d.disables, d.stops, d.deletes
			if _, err := owner.Decommission(context.Background()); err == nil || d.disables != disables || d.stops != stops || d.deletes != deletes {
				t.Fatal("failed decommission retried")
			}
			_ = owner.Close()
			if w.closes != 1 {
				t.Fatal("failed fence handle not closed once")
			}
		})
	}
}

func TestDecommissionOwnerCannotBeCopiedOrUsedByOrdinaryMaintenance(t *testing.T) {
	d := newDecommissionFixture(t)
	owner, err := openMaintenanceModeWith(context.Background(), d.req, d.op, true)
	if err != nil {
		t.Fatal(err)
	}
	defer owner.Close()
	var copied MaintenanceOwner
	reflect.ValueOf(&copied).Elem().Set(reflect.ValueOf(owner).Elem())
	if _, err := copied.Decommission(context.Background()); err == nil || d.disables != 0 {
		t.Fatal("copied owner gained decommission authority")
	}
	f := newMaintenanceFixture(t)
	ordinary, err := openMaintenanceWith(context.Background(), f.req, f.op)
	if err != nil {
		t.Fatal(err)
	}
	defer ordinary.Close()
	if _, err := ordinary.Decommission(context.Background()); err == nil || f.stops != 0 {
		t.Fatal("STOP-only owner gained decommission authority")
	}
}

func TestDecommissionExplicitAPIRequiresWindows(t *testing.T) {
	if runtime.GOOS == "windows" {
		t.Skip("non-Windows boundary only")
	}
	d := newDecommissionFixture(t)
	if owner, err := OpenDecommissionMaintenance(context.Background(), d.req); err == nil || owner != nil {
		t.Fatal("portable fake opened native SCM decommission")
	}
}
