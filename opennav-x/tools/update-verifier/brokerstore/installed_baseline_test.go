package brokerstore

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"reflect"
	"sort"
	"strings"
	"testing"
	"time"

	"example.com/opennav-update-verifier/baselineevidence"
	"example.com/opennav-update-verifier/brokerregistration"
)

type baselineFixtureData struct {
	payload             map[string][]byte
	inventory, manifest []byte
	expected            brokerregistration.Installed
	identity            baselineevidence.Identity
}

func baselineData(t *testing.T) baselineFixtureData {
	t.Helper()
	f := baselineFixtureData{payload: map[string][]byte{"app/opencpn.exe": []byte("inert executable fixture"), "app/opennav-restart.exe": []byte("inert restart fixture"), "app/skager-start.exe": []byte("inert launcher fixture")}}
	marshal := func(value any) []byte {
		raw, err := json.Marshal(value)
		if err != nil {
			t.Fatal(err)
		}
		return raw
	}
	f.payload["docs/PRODUCT_BUILD.json"] = marshal(map[string]any{"version": "2.0.0-beta.2", "commit": strings.Repeat("1", 40), "test_fixtures": false, "build_purpose": "INSTALLED PRODUCT", "xnav_hardware_output_policy": "status-only", "xnav_manual_control_contract": 0, "executable_sha256": digest(string(f.payload["app/opencpn.exe"])), "restart_helper_sha256": digest(string(f.payload["app/opennav-restart.exe"])), "commissioning_restart_protocol": 1, "update_startup_health": 1, "startup_launcher_sha256": digest(string(f.payload["app/skager-start.exe"]))})
	observed := map[string]string{}
	var names []string
	for name, raw := range f.payload {
		observed[name] = digest(string(raw))
		names = append(names, name)
	}
	sort.Strings(names)
	var files []map[string]string
	for _, name := range names {
		files = append(files, map[string]string{"path": name, "sha256": observed[name]})
	}
	stock := brokerregistration.StockBaseline()
	f.manifest = marshal(map[string]any{"schema": 1, "version": "2.0.0-beta.2", "commit": strings.Repeat("1", 40), "payloadSha256": strings.Repeat("d", 64), "supportedOpenCpn": []map[string]string{{"version": stock.Version, "arch": stock.Arch, "executableSha256": stock.ExecutableSHA256, "upstreamCommit": stock.UpstreamCommit}}, "files": files, "qualification": "candidate"})
	inventory := map[string]string{}
	for name, hash := range observed {
		inventory[name] = hash
	}
	inventory["installer/package.json"] = digest(string(f.manifest))
	inventory["app/OPENNAV_PORTABLE_PREVIEW"] = digest("portable marker, excluded")
	inventory["skager-recovery.exe"] = digest("root recovery launcher, excluded")
	f.inventory = marshal(inventory)
	f.expected = brokerregistration.Installed{Version: "2.0.0-beta.2", Commit: strings.Repeat("1", 40), Generation: stageA, PackageSHA256: digest(string(f.manifest)), ProductBuildSHA256: observed["docs/PRODUCT_BUILD.json"], ExecutableSHA256: observed["app/opencpn.exe"], InventorySHA256: digest(string(f.inventory))}
	var err error
	f.identity, _, err = baselineevidence.Validate(context.Background(), f.inventory, f.manifest, f.payload["docs/PRODUCT_BUILD.json"], observed, f.expected)
	if err != nil {
		t.Fatal(err)
	}
	return f
}

func baselineEngineData(t *testing.T) baselineFixtureData {
	t.Helper()
	f := baselineData(t)
	var inventory map[string]string
	if err := json.Unmarshal(f.inventory, &inventory); err != nil {
		t.Fatal(err)
	}
	for _, name := range []string{"installer/engine/Lifecycle.ps1", "installer/engine/UpdateSupervisor.ps1", "installer/engine/UpdateTransaction.ps1"} {
		if !baselineevidence.IsEngineHelper(name) {
			t.Fatal("fixture is not an exact engine helper")
		}
		f.payload[name] = []byte("\xef\xbb\xbf# inert original " + name + "\r\n")
		inventory[name] = digest(string(f.payload[name]))
	}
	var err error
	f.inventory, err = json.Marshal(inventory)
	if err != nil {
		t.Fatal(err)
	}
	f.expected.InventorySHA256 = digest(string(f.inventory))
	observed := map[string]string{}
	for name, raw := range f.payload {
		observed[name] = digest(string(raw))
	}
	f.identity, _, err = baselineevidence.Validate(context.Background(), f.inventory, f.manifest, f.payload["docs/PRODUCT_BUILD.json"], observed, f.expected)
	if err != nil {
		t.Fatal(err)
	}
	return f
}

type memoryBaseline struct {
	*memoryCandidate
	events                           []string
	failAt                           int
	reserved, tree, completed, extra bool
	raw                              []byte
	publishErr, cleanupErr           error
	duringPublish                    func()
}

func (b *memoryBaseline) step(name string) error {
	b.events = append(b.events, name)
	if b.failAt == len(b.events) {
		return testFault
	}
	return nil
}
func (b *memoryBaseline) check() error {
	if err := b.step("check"); err != nil {
		return err
	}
	return b.memoryCandidate.check()
}
func (b *memoryBaseline) reserve(ctx context.Context) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	if err := b.step("reserve-sync"); err != nil {
		return err
	}
	if b.reserved {
		return errors.New("permanent reservation consumed")
	}
	b.reserved = true
	return nil
}
func (b *memoryBaseline) createTree() error {
	if !b.reserved {
		panic("tree before durable reservation")
	}
	if err := b.step("fresh-tree"); err != nil {
		return err
	}
	b.tree = true
	return nil
}
func (b *memoryBaseline) inventory(names []string, completed bool) error {
	if b.extra || len(names) != len(b.files) || completed != b.completed {
		return errors.New("exact baseline inventory differs")
	}
	for _, name := range names {
		if b.files[name] == nil {
			return testFault
		}
	}
	return nil
}
func baselineTestID(index int) diskIdentity {
	return diskIdentity{Volume: 1, FileID: fmt.Sprintf("%032x", index), Created: 1}
}
func (b *memoryBaseline) identities(m map[string]baselineMember) (map[string]diskIdentity, map[string]diskIdentity, error) {
	dirs := map[string]diskIdentity{"@user": baselineTestID(1), "@installed": baselineTestID(2), "": baselineTestID(3), "app": baselineTestID(4), "docs": baselineTestID(5), "installer": baselineTestID(6)}
	files := map[string]diskIdentity{}
	var names []string
	for name := range m {
		names = append(names, name)
		if baselineevidence.IsEngineHelper(name) {
			dirs["installer/engine"] = baselineTestID(7)
		}
	}
	sort.Strings(names)
	for i, name := range names {
		files[name] = baselineTestID(i + 10)
	}
	return dirs, files, nil
}
func (b *memoryBaseline) openMembers(m map[string]baselineMember, _ map[string]diskIdentity) (map[string]candidateFile, error) {
	result := map[string]candidateFile{}
	for name := range m {
		f := b.files[name]
		if f == nil {
			return result, testFault
		}
		result[name] = f
	}
	return result, nil
}
func (b *memoryBaseline) publish(ctx context.Context, raw []byte) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	if b.duringPublish != nil {
		b.duringPublish()
	}
	b.raw = bytes.Clone(raw)
	b.completed = true
	return b.publishErr
}
func (b *memoryBaseline) directory() (string, error) { return "fixed-completed-baseline", nil }
func (b *memoryBaseline) close() error {
	b.closes++
	b.events = append(b.events, "close")
	return b.cleanupErr
}

type baselineMemoryStore struct {
	*memoryStore
	b               *memoryBaseline
	prepares, opens int
}

func (s *baselineMemoryStore) prepareInstalledBaseline(_, _, _ string) (installedBaselineBackend, error) {
	s.prepares++
	return s.b, nil
}
func (s *baselineMemoryStore) openInstalledBaseline(_ context.Context, _, _, _ string) (installedBaselineBackend, []byte, error) {
	s.opens++
	return s.b, bytes.Clone(s.b.raw), nil
}
func baselineOwner(t *testing.T) (*Store, *InstalledBaseline, *memoryBaseline, baselineFixtureData) {
	t.Helper()
	b := &memoryBaseline{memoryCandidate: &memoryCandidate{files: map[string]*memoryFile{}}}
	s := &Store{backend: &baselineMemoryStore{memoryStore: &memoryStore{}, b: b}}
	r, err := s.BeginInstalledBaselineFresh(context.Background(), registrationTestSID, stageA, registrationTestApproval)
	if err != nil {
		t.Fatal(err)
	}
	return s, r, b, baselineData(t)
}
func writeBaseline(t *testing.T, r *InstalledBaseline, f baselineFixtureData) {
	t.Helper()
	var names []string
	for name := range f.payload {
		names = append(names, name)
	}
	sort.Strings(names)
	for _, name := range names {
		raw := f.payload[name]
		var err error
		if baselineevidence.IsEngineHelper(name) {
			err = r.WriteEvidence(context.Background(), name, raw, digest(string(raw)))
		} else {
			err = r.WriteFile(context.Background(), name, bytes.NewReader(raw), int64(len(raw)), digest(string(raw)))
		}
		if err != nil {
			t.Fatal(err)
		}
	}
	for name, raw := range map[string][]byte{"FILE_SHA256.json": f.inventory, "installer/package.json": f.manifest} {
		if err := r.WriteEvidence(context.Background(), name, raw, digest(string(raw))); err != nil {
			t.Fatal(err)
		}
	}
}

func TestInstalledBaselineEngineHelpersBindCompletionAndRetainedRead(t *testing.T) {
	for _, scenario := range []string{"complete", "missing", "tampered", "extra", "alias", "tamper-after-complete"} {
		t.Run(scenario, func(t *testing.T) {
			s, r, b, _ := baselineOwner(t)
			defer s.Close()
			f := baselineEngineData(t)
			if scenario == "missing" {
				delete(f.payload, "installer/engine/UpdateTransaction.ps1")
			}
			if scenario == "tampered" {
				f.payload["installer/engine/Lifecycle.ps1"] = []byte("changed original")
			}
			writeBaseline(t, r, f)
			if scenario == "extra" || scenario == "alias" {
				name := "installer/engine/extra.ps1"
				if scenario == "alias" {
					name = "installer/engine/lifecycle.ps1"
				}
				if err := r.WriteEvidence(context.Background(), name, []byte("extra"), digest("extra")); err == nil {
					t.Fatal("nonfixed helper accepted")
				}
			}
			err := r.Complete(context.Background(), f.identity)
			if scenario != "complete" && scenario != "tamper-after-complete" {
				if err == nil || b.completed {
					t.Fatal("incomplete or tampered engine helper set completed")
				}
				return
			}
			if err != nil {
				t.Fatal(err)
			}
			if err := r.Close(); err != nil {
				t.Fatal(err)
			}
			if scenario == "tamper-after-complete" {
				b.files["installer/engine/UpdateSupervisor.ps1"].data[0] ^= 1
			}
			opened, err := s.OpenInstalledBaseline(context.Background(), registrationTestSID, stageA, registrationTestApproval, f.expected)
			if scenario == "tamper-after-complete" {
				if err == nil {
					t.Fatal("retained reader accepted changed helper")
				}
				return
			}
			if err != nil || opened.Validate() != nil {
				t.Fatal("exact helper set could not be reopened", err)
			}
			for name, raw := range f.payload {
				if !bytes.Equal(b.files[name].data, raw) {
					t.Fatal("original helper bytes reconstructed")
				}
			}
		})
	}
}
func TestInstalledBaselineReservationPrecedesTreeAndEveryFaultRetainsPartialEvidence(t *testing.T) {
	for at := 0; at <= 5; at++ {
		t.Run(fmt.Sprint(at), func(t *testing.T) {
			cleanup := errors.New("close uncertainty")
			b := &memoryBaseline{memoryCandidate: &memoryCandidate{}, failAt: at, cleanupErr: cleanup}
			err := beginInstalledBaseline(context.Background(), b)
			if at == 0 {
				if err != nil || !b.tree || !b.reserved || b.closes != 0 {
					t.Fatalf("success %+v %v", b, err)
				}
				want := []string{"check", "reserve-sync", "check", "fresh-tree", "check"}
				if !reflect.DeepEqual(b.events, want) {
					t.Fatal(b.events)
				}
			} else {
				if !errors.Is(err, testFault) || !errors.Is(err, cleanup) || b.closes != 1 || len(b.events) != at+1 || b.reserved != (at > 2) || b.tree != (at > 4) {
					t.Fatalf("failure %d %+v %v", at, b, err)
				}
			}
		})
	}
}
func TestInstalledBaselineInvalidAndUnsupportedBeginsDoNotConsumeReservation(t *testing.T) {
	s, m := testStore()
	if _, err := s.BeginInstalledBaselineFresh(context.Background(), registrationTestSID, stageA, registrationTestApproval); !errors.Is(err, ErrNativeRequired) {
		t.Fatal(err)
	}
	if len(m.stages) != 0 {
		t.Fatal("ordinary staging seam used")
	}
	s, _, b, _ := baselineOwner(t)
	defer s.Close()
	b.events = nil
	for _, tc := range []struct{ sid, generation, approval string }{{"S-1-5-18", stageA, registrationTestApproval}, {registrationTestSID, strings.Repeat("A", 32), registrationTestApproval}, {registrationTestSID, stageA, strings.Repeat("A", 64)}} {
		if _, err := s.BeginInstalledBaselineFresh(context.Background(), tc.sid, tc.generation, tc.approval); err == nil {
			t.Fatal("invalid begin accepted")
		}
	}
	ctx, cancel := context.WithCancel(context.Background())
	cancel()
	if _, err := s.BeginInstalledBaselineFresh(ctx, registrationTestSID, stageA, registrationTestApproval); !errors.Is(err, context.Canceled) {
		t.Fatal(err)
	}
	if len(b.events) != 0 {
		t.Fatal("invalid begin reached native mutation")
	}
}
func TestInstalledBaselineCompleteReopenExactOriginalsAndIndependentPins(t *testing.T) {
	s, r, b, f := baselineOwner(t)
	writeBaseline(t, r, f)
	if err := r.Complete(context.Background(), f.identity); err != nil {
		t.Fatal(err)
	}
	if err := r.Validate(); err != nil {
		t.Fatal(err)
	}
	if _, ok := b.files["app/OPENNAV_PORTABLE_PREVIEW"]; ok {
		t.Fatal("portable marker staged")
	}
	if _, ok := b.files["skager-recovery.exe"]; ok {
		t.Fatal("root launcher staged")
	}
	if path, err := r.Directory(); err != nil || path != "fixed-completed-baseline" {
		t.Fatal(path, err)
	}
	if err := r.Close(); err != nil {
		t.Fatal(err)
	}
	opened, err := s.OpenInstalledBaseline(context.Background(), registrationTestSID, stageA, registrationTestApproval, f.expected)
	if err != nil {
		t.Fatal(err)
	}
	if err := opened.Validate(); err != nil {
		t.Fatal(err)
	}
	if err := opened.WriteFile(context.Background(), "app/new", strings.NewReader("x"), 1, digest("x")); err == nil {
		t.Fatal("read owner wrote")
	}
	if err := opened.Complete(context.Background(), f.identity); err == nil {
		t.Fatal("read owner completed twice")
	}
	if err := s.Close(); err != nil {
		t.Fatal(err)
	}
	if err := opened.Validate(); !errors.Is(err, ErrClosed) {
		t.Fatal(err)
	}
	for name, file := range b.files {
		if file.closes != 2 {
			t.Fatalf("%s close count %d", name, file.closes)
		}
	}
}
func TestInstalledBaselineCompletionRejectsMissingChangedAndExtraMembers(t *testing.T) {
	for _, kind := range []string{"missing-sidecar", "changed-member", "extra-namespace", "unmanifested-member", "changed-identity"} {
		t.Run(kind, func(t *testing.T) {
			s, r, b, f := baselineOwner(t)
			defer s.Close()
			writeBaseline(t, r, f)
			switch kind {
			case "missing-sidecar":
				delete(r.files, "installer/package.json")
			case "changed-member":
				b.files["app/opencpn.exe"].data[0] ^= 1
			case "extra-namespace":
				b.extra = true
			case "unmanifested-member":
				if err := r.WriteFile(context.Background(), "app/extra", strings.NewReader("extra"), 5, digest("extra")); err != nil {
					t.Fatal(err)
				}
			case "changed-identity":
				f.identity.DeclaredPayloadSHA256 = strings.Repeat("e", 64)
			}
			if err := r.Complete(context.Background(), f.identity); err == nil {
				t.Fatal("invalid completion accepted")
			}
			if b.completed || len(b.raw) != 0 {
				t.Fatal("invalid completion published")
			}
			if err := r.Validate(); !errors.Is(err, ErrPoisoned) {
				t.Fatal(err)
			}
		})
	}
}
func TestInstalledBaselineWriteFailuresPoisonAndRetainEvidence(t *testing.T) {
	for _, kind := range []string{"hash", "short", "long", "source", "sync", "alias", "sidecar", "portable"} {
		t.Run(kind, func(t *testing.T) {
			s, r, b, _ := baselineOwner(t)
			defer s.Close()
			source := io.Reader(strings.NewReader("payload"))
			name := "app/a"
			size := int64(7)
			hash := digest("payload")
			switch kind {
			case "hash":
				hash = digest("other")
			case "short":
				size = 8
			case "long":
				size = 6
			case "source":
				source = failingBaselineReader{}
			case "sync":
				b.configure = func(f *memoryFile) { f.syncFault = testFault }
			case "alias":
				if err := r.WriteFile(context.Background(), "app/A", strings.NewReader("payload"), 7, hash); err != nil {
					t.Fatal(err)
				}
			case "sidecar":
				name = "package.json"
			case "portable":
				name = "app/OPENNAV_PORTABLE_PREVIEW"
			}
			if err := r.WriteFile(context.Background(), name, source, size, hash); err == nil {
				t.Fatal("invalid write accepted")
			}
			if err := r.WriteFile(context.Background(), "app/next", strings.NewReader("x"), 1, digest("x")); !errors.Is(err, ErrPoisoned) {
				t.Fatal(err)
			}
			if !b.reserved || !b.tree {
				t.Fatal("failure erased namespace")
			}
		})
	}
}

type failingBaselineReader struct{}

func (failingBaselineReader) Read([]byte) (int, error) { return 0, testFault }
func TestInstalledBaselineCancellationAndUncertainCommitSemantics(t *testing.T) {
	for _, kind := range []string{"before", "during-success", "during-uncertain"} {
		t.Run(kind, func(t *testing.T) {
			s, r, b, f := baselineOwner(t)
			defer s.Close()
			writeBaseline(t, r, f)
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			if kind == "before" {
				cancel()
			} else {
				b.duringPublish = cancel
				if kind == "during-uncertain" {
					b.publishErr = errors.Join(ErrInstalledBaselineUncertain, testFault)
				}
			}
			err := r.Complete(ctx, f.identity)
			switch kind {
			case "before":
				if !errors.Is(err, context.Canceled) || b.completed {
					t.Fatal(err, b.completed)
				}
			case "during-success":
				if err != nil || !b.completed {
					t.Fatal(err)
				}
				if err := r.Validate(); err != nil {
					t.Fatal("successful durable publication poisoned:", err)
				}
			case "during-uncertain":
				if !errors.Is(err, ErrInstalledBaselineUncertain) || !errors.Is(err, testFault) || !b.completed {
					t.Fatal(err)
				}
				if err := r.Close(); !errors.Is(err, ErrInstalledBaselineUncertain) {
					t.Fatal("uncertainty lost on close:", err)
				}
				if err := r.Close(); !errors.Is(err, ErrInstalledBaselineUncertain) || b.closes != 1 {
					t.Fatal(err, b.closes)
				}
			}
		})
	}
}
func TestInstalledBaselineReaderRejectsWrongPinsMalformedRecordAndChangedBytes(t *testing.T) {
	for _, kind := range []string{"approval", "generation", "package", "inventory", "product", "executable", "version", "record", "bytes"} {
		t.Run(kind, func(t *testing.T) {
			s, r, b, f := baselineOwner(t)
			defer s.Close()
			writeBaseline(t, r, f)
			if err := r.Complete(context.Background(), f.identity); err != nil {
				t.Fatal(err)
			}
			if err := r.Close(); err != nil {
				t.Fatal(err)
			}
			approval := registrationTestApproval
			generation := stageA
			switch kind {
			case "approval":
				approval = strings.Repeat("b", 64)
			case "generation":
				generation = strings.Repeat("b", 32)
			case "package":
				f.expected.PackageSHA256 = strings.Repeat("b", 64)
			case "inventory":
				f.expected.InventorySHA256 = strings.Repeat("b", 64)
			case "product":
				f.expected.ProductBuildSHA256 = strings.Repeat("b", 64)
			case "executable":
				f.expected.ExecutableSHA256 = strings.Repeat("b", 64)
			case "version":
				f.expected.Version = "2.0.0-beta.3"
			case "record":
				b.raw = append(b.raw, ' ')
			case "bytes":
				b.files["app/opencpn.exe"].data[0] ^= 1
			}
			if opened, err := s.OpenInstalledBaseline(context.Background(), registrationTestSID, generation, approval, f.expected); err == nil {
				opened.Close()
				t.Fatal("mismatched retained baseline accepted")
			}
		})
	}
}
func TestInstalledBaselineMarkerExactCompletionAndApproval(t *testing.T) {
	raw := baselineMarker(registrationTestSID, stageA, registrationTestApproval, digest("record"), true)
	if got, err := completedBaselineDigest(raw, registrationTestSID, stageA, registrationTestApproval); err != nil || got != digest("record") {
		t.Fatal(got, err)
	}
	for _, bad := range [][]byte{baselineMarker(registrationTestSID, stageA, registrationTestApproval, "", false), append(bytes.Clone(raw), '\n'), bytes.Replace(raw, []byte(registrationTestApproval), []byte(strings.Repeat("b", 64)), 1)} {
		if _, err := completedBaselineDigest(bad, registrationTestSID, stageA, registrationTestApproval); err == nil {
			t.Fatal("partial/foreign reservation accepted")
		}
	}
}

func TestInstalledBaselineStoreCloseStopsWriteAndDrainsOnce(t *testing.T) {
	s, r, b, _ := baselineOwner(t)
	entered := make(chan struct{})
	release := make(chan struct{})
	writeDone := make(chan error, 1)
	closeDone := make(chan error, 1)
	go func() {
		writeDone <- r.WriteFile(context.Background(), "app/blocked", &blockedBaselineReader{entered: entered, release: release}, 1, digest("x"))
	}()
	<-entered
	go func() { closeDone <- s.Close() }()
	deadline := time.Now().Add(time.Second)
	for !s.closed.Load() {
		if time.Now().After(deadline) {
			close(release)
			t.Fatal("Store.Close did not stop owner")
		}
		time.Sleep(time.Millisecond)
	}
	select {
	case err := <-closeDone:
		close(release)
		t.Fatal("Close returned before writer drained:", err)
	default:
	}
	close(release)
	if err := <-writeDone; !errors.Is(err, ErrClosed) {
		t.Fatal("inflight write did not refuse closed store:", err)
	}
	if err := <-closeDone; !errors.Is(err, ErrClosed) {
		t.Fatal("owner failure lost by store cleanup:", err)
	}
	if err := s.Close(); !errors.Is(err, ErrClosed) || b.closes != 1 || b.files["app/blocked"].closes != 1 {
		t.Fatal("close retried or lost evidence:", err, b.closes)
	}
}

type blockedBaselineReader struct {
	entered, release chan struct{}
	read             bool
}

func (r *blockedBaselineReader) Read(p []byte) (int, error) {
	if r.read {
		return 0, io.EOF
	}
	r.read = true
	close(r.entered)
	<-r.release
	p[0] = 'x'
	return 1, nil
}
