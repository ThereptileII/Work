package brokerenroll

import (
	"context"
	"errors"
	"strings"
	"testing"
)

type installedBackendFixture struct {
	d                        *decommissionFixture
	fault                    string
	checks, cleanups, closes int
	partial                  bool
}

func (i *installedBackendFixture) check(ctx context.Context) error {
	i.checks++
	i.d.events = append(i.d.events, "check-installed")
	if i.fault == "check" || i.fault == "post-cleanup" && i.partial {
		return errors.New("original installed custody changed")
	}
	return ctx.Err()
}
func (i *installedBackendFixture) cleanup(ctx context.Context, guard func(context.Context) error) error {
	i.cleanups++
	i.d.events = append(i.d.events, "cleanup-installed")
	if !i.d.disabled || !i.d.process.exited || i.d.state.state != serviceStopped || i.d.marked || i.d.process.closes != 0 || i.d.closes != 0 {
		panic("installed cleanup did not retain original stopped service/process")
	}
	if err := guard(ctx); err != nil {
		return err
	}
	if i.fault == "cleanup-before" {
		return errors.New("original registry conflict")
	}
	i.partial = true
	if i.fault == "cleanup-partial" {
		return errors.New("partial owned member removal")
	}
	return guard(ctx)
}
func (i *installedBackendFixture) ownsStartupLink(target, args, working string) bool {
	return target == "fixed-original-target" && args == "--xnav" && working == "fixed-original-app"
}
func (i *installedBackendFixture) close() error {
	i.closes++
	i.d.events = append(i.d.events, "close-installed")
	if i.fault == "close" {
		return errors.New("installed owner close uncertain")
	}
	return nil
}

type installedStartupFixture struct {
	*preparedRestorationFixture
	prepares, retires int
	fault             string
}

func (s *installedStartupFixture) prepareInstalled(ctx context.Context, owns func(string, string, string) bool) error {
	s.prepares++
	s.d.events = append(s.d.events, "prepare-installed-startup")
	if !owns("fixed-original-target", "--xnav", "fixed-original-app") || owns("foreign-target", "--xnav", "fixed-original-app") {
		return errors.New("lost original installed membership selector")
	}
	if s.fault == "prepare" {
		return errors.New("original normal link unreadable")
	}
	return ctx.Err()
}
func (s *installedStartupFixture) retireInstalled(ctx context.Context, guard func(context.Context) error) error {
	s.retires++
	s.d.events = append(s.d.events, "retire-original-normal")
	if s.d.disabled || s.d.process.exited || !s.bound {
		panic("normal link retirement lacked original live final witness")
	}
	if err := guard(ctx); err != nil {
		return err
	}
	if s.fault == "retire" {
		return errors.New("original normal link retirement uncertain")
	}
	return guard(ctx)
}

type installedFenceFixture struct {
	*decommissionFenceFixture
	d *decommissionFixture
}

func (f installedFenceFixture) Check(ctx context.Context, pid uint32, birth uint64) error {
	if f.d.process.exited {
		panic("live broker witness checked after original broker exit")
	}
	return f.decommissionFenceFixture.Check(ctx, pid, birth)
}

func installedUninstallFixture(t *testing.T) (*MaintenanceOwner, *decommissionFixture, *installedStartupFixture, *installedBackendFixture) {
	t.Helper()
	d := newDecommissionFixture(t)
	m, err := openMaintenanceModeWith(context.Background(), d.req, d.op, true)
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { _ = m.Close() })
	prepared := &preparedRestorationFixture{startupRestorationFixture: &startupRestorationFixture{d: d}}
	if err := prepareRestorationFixture(t, m, prepared); err != nil {
		t.Fatal(err)
	}
	s := &installedStartupFixture{preparedRestorationFixture: prepared}
	m.startup = s
	i := &installedBackendFixture{d: d}
	return m, d, s, i
}

func prepareInstalledFixture(t *testing.T, m *MaintenanceOwner, i *installedBackendFixture) error {
	t.Helper()
	return prepareInstalledUninstallWith(m, context.Background(), "independently-approved-input", func(ctx context.Context, a maintenanceAuthority, directory string) (installedUninstallBackend, error) {
		if a.client.RequestSHA256 != m.authority.client.RequestSHA256 || directory != "independently-approved-input" {
			t.Fatal("lost original input binding")
		}
		i.d.events = append(i.d.events, "prepare-installed")
		return i, nil
	})
}

func bindInstalledFixture(t *testing.T, m *MaintenanceOwner, d *decommissionFixture) error {
	t.Helper()
	snapshot := preparedRetirementSnapshot(t, m)
	if err := restorePreparedFixture(t, m, snapshot, &d.events); err != nil {
		return err
	}
	fence := installedFenceFixture{&decommissionFenceFixture{events: &d.events}, d}
	return bindDecommissionSnapshotWith(m, context.Background(), fence, snapshot.ClientSID, snapshot.RegistrationSHA256, &snapshot)
}

func TestInstalledUninstallDecommissionRetainsOriginalsAndCleansOnlyAfterExit(t *testing.T) {
	m, d, s, i := installedUninstallFixture(t)
	if err := prepareInstalledFixture(t, m, i); err != nil {
		t.Fatal(err)
	}
	if s.prepares != 1 || d.disables != 0 || d.stops != 0 || i.cleanups != 0 {
		t.Fatal("preparation mutated installed or SCM state")
	}
	if _, err := m.Decommission(context.Background()); err == nil || d.disables != 0 {
		t.Fatal("ordinary service-only fallback accepted installed preparation")
	}
	if _, err := m.DecommissionInstalled(context.Background()); err == nil || d.disables != 0 {
		t.Fatal("unbound final witness performed cleanup")
	}
	if err := bindInstalledFixture(t, m, d); err != nil {
		t.Fatal(err)
	}
	result, err := m.DecommissionInstalled(context.Background())
	if err != nil {
		t.Fatal(err)
	}
	if result != (InstalledUninstallResult{ServiceMarkedForRemoval, "verified-owned-members-removed", "unregistered", "preserved"}) {
		t.Fatal(result)
	}
	if s.retires != 1 || i.cleanups != 1 || i.closes != 1 || d.disables != 1 || d.stops != 1 || d.deletes != 1 {
		t.Fatal("effects/cleanup repeated")
	}
	index := func(name string) int {
		for i, value := range d.events {
			if value == name {
				return i
			}
		}
		return -1
	}
	if !(index("retire-original-normal") < index("disable") && index("stop") < index("cleanup-installed") && index("cleanup-installed") < index("delete") && index("delete") < index("close-installed") && index("close-installed") < index("close-fence")) {
		t.Fatal("native custody/effects reordered", d.events)
	}
	if _, err := m.DecommissionInstalled(context.Background()); err == nil || i.cleanups != 1 {
		t.Fatal("completed cleanup replayed")
	}
	if err := m.Close(); err != nil || i.closes != 1 {
		t.Fatal("repeat close changed custody", err)
	}
}

func TestInstalledUninstallDecommissionFaultsRemainStickyAndPreserveSCMOrdering(t *testing.T) {
	for _, fault := range []string{"prepare", "prepare-startup", "check", "retire", "cleanup-before", "cleanup-partial", "post-cleanup", "close", "disable", "stop", "delete", "cancel"} {
		t.Run(fault, func(t *testing.T) {
			m, d, s, i := installedUninstallFixture(t)
			if fault == "prepare-startup" {
				s.fault = "prepare"
			}
			if fault == "check" {
				i.fault = fault
			}
			if fault == "prepare" {
				err := prepareInstalledUninstallWith(m, context.Background(), "input", func(context.Context, maintenanceAuthority, string) (installedUninstallBackend, error) {
					return i, errors.New("native partial open")
				})
				if err == nil || !m.failed {
					t.Fatal("partial preparation accepted")
				}
			} else if err := prepareInstalledFixture(t, m, i); err != nil {
				if fault != "prepare-startup" && fault != "check" {
					t.Fatal(err)
				}
			}
			if !m.failed {
				if err := bindInstalledFixture(t, m, d); err != nil {
					t.Fatal(err)
				}
				if fault == "retire" {
					s.fault = "retire"
				}
				i.fault = fault
				if fault == "disable" || fault == "delete" {
					d.fault = fault
				}
				if fault == "stop" {
					d.maintenanceFixture.fault = "stop"
				}
			}
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			if fault == "cancel" {
				cancel()
			}
			result, err := m.DecommissionInstalled(ctx)
			if err == nil || result != (InstalledUninstallResult{}) {
				t.Fatal("uncertain cleanup emitted success", result, err)
			}
			cleanups, disables, stops, deletes := i.cleanups, d.disables, d.stops, d.deletes
			if _, err := m.DecommissionInstalled(context.Background()); err == nil || i.cleanups != cleanups || d.disables != disables || d.stops != stops || d.deletes != deletes {
				t.Fatal("uncertain action retried")
			}
			if strings.HasPrefix(fault, "cleanup") || fault == "post-cleanup" {
				if d.deletes != 0 || !d.process.exited || !d.disabled {
					t.Fatal("failed cleanup dispatched service removal or preceded exit")
				}
			}
			if fault == "prepare" || fault == "prepare-startup" || fault == "check" {
				if d.disables != 0 || i.cleanups != 0 {
					t.Fatal("refused preparation reached effects")
				}
			}
			_ = m.Close()
			if i.closes != 1 {
				t.Fatal("original installed owner not drained")
			}
		})
	}
}

func TestInstalledUninstallDecommissionRequiresFinalRetirementNotParsedData(t *testing.T) {
	m, d, _, i := installedUninstallFixture(t)
	if err := prepareInstalledFixture(t, m, i); err != nil {
		t.Fatal(err)
	}
	snapshot := preparedRetirementSnapshot(t, m)
	if err := restorePreparedFixture(t, m, snapshot, &d.events); err != nil {
		t.Fatal(err)
	}
	snapshot.RetirementSHA256 = ""
	fence := installedFenceFixture{&decommissionFenceFixture{events: &d.events}, d}
	if err := bindDecommissionSnapshotWith(m, context.Background(), fence, snapshot.ClientSID, snapshot.RegistrationSHA256, &snapshot); err == nil {
		t.Fatal("fence-only result authorized installed cleanup")
	}
	if _, err := m.DecommissionInstalled(context.Background()); err == nil || d.disables != 0 || i.cleanups != 0 {
		t.Fatal("missing final retirement fell through")
	}
	if (&MaintenanceOwner{}).PrepareInstalledUninstall(context.Background(), "path") == nil {
		t.Fatal("zero owner accepted installed input")
	}
	if _, err := (&MaintenanceOwner{}).DecommissionInstalled(context.Background()); err == nil {
		t.Fatal("zero owner authorized cleanup")
	}
}
