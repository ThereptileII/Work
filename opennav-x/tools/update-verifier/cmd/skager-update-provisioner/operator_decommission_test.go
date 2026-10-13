package main

import (
	"bytes"
	"context"
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"errors"
	"reflect"
	"strings"
	"testing"

	"example.com/opennav-update-verifier/brokerenroll"
	"example.com/opennav-update-verifier/brokerpipe"
)

func decommissionArguments(raw []byte) []string {
	h := sha256.Sum256(raw)
	return []string{"--maintenance=decommission-service", "--broker-sha256=" + strings.Repeat("a", 64),
		"--client-registration-request=original-private-request", "--client-registration-request-sha256=" + hex.EncodeToString(h[:]),
		"--request-id=" + strings.Repeat("1", 32), "--expected-selected-head-sha256=" + strings.Repeat("2", 64)}
}

type refusedDecommissionOwner struct {
	checks, binds, effects, closes int
	checkErr, closeErr             error
}

func (f *refusedDecommissionOwner) Check(context.Context) error { f.checks++; return f.checkErr }
func (f *refusedDecommissionOwner) Close() error                { f.closes++; return f.closeErr }
func (f *refusedDecommissionOwner) BindDecommission(context.Context, *brokerpipe.DecommissionWitness) error {
	f.binds++
	return errors.New("private invalid original witness")
}
func (f *refusedDecommissionOwner) Decommission(context.Context) (brokerenroll.DecommissionResult, error) {
	f.effects++
	return brokerenroll.DecommissionResult{}, errors.New("unexpected decommission")
}
func (f *refusedDecommissionOwner) BindStartupRestoration(context.Context, *brokerenroll.ApprovedStartupPublication) error {
	return errors.New("unexpected startup restoration")
}

func TestOperatorDecommissionInvalidInputsHaveNoEffects(t *testing.T) {
	raw := []byte("original request")
	base := decommissionArguments(raw)
	for _, args := range [][]string{nil, base[:5], append(append([]string{}, base...), base[0]),
		append(append([]string{}, base...), "--root=C:/private"),
		append(append([]string{}, base...), "--client-sid=S-1-5-18"),
		append(append([]string{}, base...), "--maintenance=manual-rollback"),
		append([]string{"--maintenance=uninstall"}, base[1:]...),
		append(base[:4:4], "--request-id=ABC", base[5]),
		append(base[:5:5], "--expected-selected-head-sha256=bad"),
		append(append([]string{}, base...), "positional")} {
		var output bytes.Buffer
		calls := 0
		err := runDecommission(context.Background(), args, func(string) ([]byte, error) { calls++; return raw, nil },
			func(context.Context, brokerenroll.MaintenanceRequest) (decommissionOwner, error) {
				calls++
				return nil, nil
			},
			func(context.Context, brokerpipe.MaintenanceRequest) (*brokerpipe.DecommissionWitness, error) {
				calls++
				return nil, nil
			}, &output)
		if err == nil || calls != 0 || output.Len() != 0 || strings.Contains(err.Error(), "C:/private") {
			t.Fatalf("invalid action had effects: calls=%d output=%q err=%v", calls, output.String(), err)
		}
	}
}

func TestOperatorDecommissionRequiresOriginalServiceBeforeFenceRequest(t *testing.T) {
	raw := []byte("original request")
	for _, fault := range []string{"changed-request", "open", "check", "transport", "missing-witness", "zero-witness"} {
		t.Run(fault, func(t *testing.T) {
			owner, calls := &refusedDecommissionOwner{}, 0
			var output bytes.Buffer
			if fault == "check" {
				owner.checkErr = errors.New("private original service changed")
			}
			err := runDecommission(context.Background(), decommissionArguments(raw), func(string) ([]byte, error) {
				if fault == "changed-request" {
					return []byte("different"), nil
				}
				return raw, nil
			}, func(_ context.Context, request brokerenroll.MaintenanceRequest) (decommissionOwner, error) {
				if !bytes.Equal(request.ClientRegistrationRequest, raw) {
					t.Fatal("request changed")
				}
				if fault == "open" {
					return owner, errors.New("private open refused")
				}
				return owner, nil
			}, func(_ context.Context, q brokerpipe.MaintenanceRequest) (*brokerpipe.DecommissionWitness, error) {
				calls++
				if owner.checks != 1 || q.Schema != 1 || q.Operation != brokerpipe.PrepareDecommission || q.RequestID != strings.Repeat("1", 32) || q.ExpectedSelectedHeadSHA256 != strings.Repeat("2", 64) {
					t.Fatal("missing original service check or wrong closed request")
				}
				if fault == "transport" {
					return nil, errors.New("private transport refused")
				}
				if fault == "zero-witness" {
					return &brokerpipe.DecommissionWitness{}, nil
				}
				return nil, nil
			}, &output)
			wantCalls := 0
			if fault == "transport" || fault == "missing-witness" || fault == "zero-witness" {
				wantCalls = 1
			}
			if err == nil || calls != wantCalls || owner.effects != 0 || output.Len() != 0 || strings.Contains(err.Error(), "private") {
				t.Fatalf("calls=%d effects=%d output=%q err=%v", calls, owner.effects, output.String(), err)
			}
			if fault != "changed-request" && owner.closes != 1 {
				t.Fatalf("original owner not closed: %d", owner.closes)
			}
		})
	}
}

func TestDecommissionDispatchSelectsOnlyExplicitAction(t *testing.T) {
	for _, args := range [][]string{{"--maintenance=decommission-service"}, {"--maintenance", "decommission-service"}, {"--maintenance=uninstall-protected"}, {"--maintenance", "uninstall-protected"}} {
		if !decommissionRequested(args) {
			t.Fatal("explicit action ignored")
		}
	}
	for _, args := range [][]string{nil, {"--maintenance=uninstall"}, {"--maintenance=decommission-service-extra"}, {"--maintenance-other=decommission-service"}, {"--maintenance=uninstall-protected-extra"}} {
		if decommissionRequested(args) {
			t.Fatal("unrelated action selected")
		}
	}
}

func decommissionPublicationArguments(request, publication []byte) []string {
	h := sha256.Sum256(publication)
	return append(decommissionArguments(request), "--original-startup-publication=original-private-publication",
		"--original-startup-publication-sha256="+hex.EncodeToString(h[:]))
}

func TestOperatorDecommissionPublicationArgumentsRefuseBeforeAnyRead(t *testing.T) {
	base := decommissionArguments([]byte("request"))
	path := "--original-startup-publication=original-private-publication"
	approval := "--original-startup-publication-sha256=" + strings.Repeat("a", 64)
	for _, extra := range [][]string{{path}, {approval}, {path, "--original-startup-publication-sha256="},
		{"--original-startup-publication=", approval}, {path, approval, path}, {path, approval, approval},
		{path, "--original-startup-publication-sha256=" + strings.Repeat("A", 64)},
		{"--original-startup-publication", "--original-startup-publication-sha256", strings.Repeat("a", 64)},
		{"--startup-publication=alias", approval}, {path, approval, "positional"}} {
		calls := 0
		var output bytes.Buffer
		err := runDecommission(context.Background(), append(append([]string{}, base...), extra...),
			func(string) ([]byte, error) { calls++; return nil, nil },
			func(context.Context, brokerenroll.MaintenanceRequest) (decommissionOwner, error) {
				calls++
				return nil, nil
			},
			func(context.Context, brokerpipe.MaintenanceRequest) (*brokerpipe.DecommissionWitness, error) {
				calls++
				return nil, nil
			}, &output)
		if err == nil || calls != 0 || output.Len() != 0 {
			t.Fatalf("ambiguous restoration input read or acted: calls=%d err=%v", calls, err)
		}
	}
}

func TestOperatorDecommissionRealPublicationParserRefusesBeforeSCM(t *testing.T) {
	request := []byte("not an approved native request")
	for _, tc := range []struct {
		name string
		raw  []byte
		pin  string
	}{
		{"empty", nil, strings.Repeat("a", 64)},
		{"oversize", bytes.Repeat([]byte("x"), maxStartupPublicationBytes+1), strings.Repeat("a", 64)},
		{"unapproved", []byte(`{"schema":1}`), strings.Repeat("0", 64)},
		{"malformed", []byte(`{"schema":`), ""},
		{"unapproved-original-request", []byte(`{"schema":1}`), ""},
	} {
		t.Run(tc.name, func(t *testing.T) {
			args := decommissionPublicationArguments(request, tc.raw)
			if tc.pin != "" {
				args[len(args)-1] = "--original-startup-publication-sha256=" + tc.pin
			}
			opens, calls, reads := 0, 0, 0
			var output bytes.Buffer
			err := runDecommission(context.Background(), args, func(path string) ([]byte, error) {
				reads++
				if path == "original-private-publication" {
					return tc.raw, nil
				}
				return request, nil
			}, func(context.Context, brokerenroll.MaintenanceRequest) (decommissionOwner, error) {
				opens++
				return nil, nil
			},
				func(context.Context, brokerpipe.MaintenanceRequest) (*brokerpipe.DecommissionWitness, error) {
					calls++
					return nil, nil
				}, &output)
			if err == nil || reads != 2 || opens != 0 || calls != 0 || output.Len() != 0 || strings.Contains(err.Error(), "private") {
				t.Fatalf("unapproved publication had SCM effects: reads=%d opens=%d calls=%d err=%v", reads, opens, calls, err)
			}
		})
	}
}

type startupDecommissionFixture struct {
	t                        *testing.T
	events                   []string
	fault                    string
	approved                 *brokerenroll.ApprovedStartupPublication
	witness                  *brokerpipe.DecommissionWitness
	requested, fenced, bound bool
	uninstall                bool
	closed, witnessClosed    bool
	cancel                   context.CancelFunc
}

func (f *startupDecommissionFixture) event(name string) error {
	f.events = append(f.events, name)
	if f.fault == name {
		return errors.New("private fixture fault")
	}
	return nil
}
func (f *startupDecommissionFixture) Check(context.Context) error { return f.event("check") }
func (f *startupDecommissionFixture) BindDecommission(_ context.Context, witness *brokerpipe.DecommissionWitness) error {
	if witness != f.witness {
		f.t.Fatal("original live fence was substituted")
	}
	if err := f.event("bind-fence"); err != nil {
		return err
	}
	f.fenced = true
	return nil
}
func (f *startupDecommissionFixture) BindStartupRestoration(_ context.Context, approved *brokerenroll.ApprovedStartupPublication) error {
	if !f.requested || !f.fenced || approved != f.approved || approved == nil {
		f.t.Fatal("restoration did not bind the same approved publication and fenced owner")
	}
	if err := f.event("bind-startup"); err != nil {
		return err
	}
	f.bound = true
	if f.fault == "cancel-startup" {
		f.cancel()
	}
	return nil
}
func (f *startupDecommissionFixture) Decommission(context.Context) (brokerenroll.DecommissionResult, error) {
	if !f.fenced || f.requested && !f.bound {
		f.t.Fatal("mutation preceded required original fence/restoration binding")
	}
	if err := f.event("decommission"); err != nil {
		return brokerenroll.DecommissionResult{}, err
	}
	if f.fault == "wrong-result" {
		return brokerenroll.DecommissionResult{}, nil
	}
	return brokerenroll.DecommissionResult{Service: brokerenroll.ServiceMarkedForRemoval}, nil
}
func (f *startupDecommissionFixture) Close() error {
	if !f.closed {
		f.closed = true
		return f.event("close-owner")
	}
	if f.fault == "close-owner" {
		return errors.New("private cached owner close fault")
	}
	return nil
}

// Zero publication/witness objects are used ONLY behind private parser/owner/
// close seams. The public production entrypoint retains the actual parser,
// sealed native witness and native Close; these fakes grant no native authority.
func (f *startupDecommissionFixture) run(request, publication []byte, output *bytes.Buffer) error {
	args := decommissionArguments(request)
	if f.requested {
		args = decommissionPublicationArguments(request, publication)
	}
	if f.uninstall {
		args[0] = "--maintenance=" + uninstallProtected
	}
	ctx, cancel := context.WithCancel(context.Background())
	f.cancel = cancel
	defer cancel()
	f.approved, f.witness = &brokerenroll.ApprovedStartupPublication{}, &brokerpipe.DecommissionWitness{}
	return runDecommissionWith(ctx, args, func(path string) ([]byte, error) {
		if path == "original-private-publication" {
			if err := f.event("read-publication"); err != nil {
				return nil, err
			}
			return publication, nil
		}
		if err := f.event("read-request"); err != nil {
			return nil, err
		}
		return request, nil
	}, func(_ context.Context, r brokerenroll.MaintenanceRequest) (decommissionOwner, error) {
		if !bytes.Equal(r.ClientRegistrationRequest, request) || r.ApprovedClientRequestSHA256 != operatorRequestDigest(request) {
			f.t.Fatal("original independently pinned request changed before opening owner")
		}
		err := f.event("open")
		return f, err
	}, func(_ context.Context, q brokerpipe.MaintenanceRequest) (*brokerpipe.DecommissionWitness, error) {
		expectedOperation := brokerpipe.PrepareDecommission
		if f.uninstall {
			expectedOperation = brokerpipe.PrepareUninstall
		}
		if q.Operation != expectedOperation || q.ExpectedSelectedHeadSHA256 != strings.Repeat("2", 64) {
			f.t.Fatal("original terminal-fence operation or selected head changed")
		}
		err := f.event("fence")
		return f.witness, err
	}, output, decommissionOperations{
		parse: func(raw []byte, publicationPin string, original []byte, originalPin string) (*brokerenroll.ApprovedStartupPublication, error) {
			if !f.requested || !bytes.Equal(raw, publication) || publicationPin != operatorRequestDigest(publication) ||
				!bytes.Equal(original, request) || originalPin != operatorRequestDigest(request) {
				f.t.Fatal("parser lost separate original input bytes or independent pin")
			}
			if err := f.event("parse"); err != nil {
				return nil, err
			}
			if f.fault == "nil-publication" {
				return nil, nil
			}
			if f.fault == "cancel-parse" {
				cancel()
			}
			return f.approved, nil
		},
		closeWitness: func(witness *brokerpipe.DecommissionWitness) error {
			if witness != f.witness {
				f.t.Fatal("closing a substituted witness")
			}
			if !f.witnessClosed {
				f.witnessClosed = true
				return f.event("close-witness")
			}
			if f.fault == "close-witness" {
				return errors.New("private cached witness close fault")
			}
			return nil
		},
	})
}

func operatorRequestDigest(raw []byte) string {
	sum := sha256.Sum256(raw)
	return hex.EncodeToString(sum[:])
}

func TestOperatorDecommissionExplicitStartupBindsSameOwnerBeforeMutation(t *testing.T) {
	f := &startupDecommissionFixture{t: t, requested: true}
	var output bytes.Buffer
	err := f.run([]byte("request"), []byte("original publication"), &output)
	want := []string{"read-request", "read-publication", "parse", "open", "check", "fence", "bind-fence", "bind-startup", "decommission", "close-owner", "close-witness"}
	if err != nil || !reflect.DeepEqual(f.events, want) {
		t.Fatalf("incorrect ownership/effect order: events=%v err=%v", f.events, err)
	}
	var result map[string]any
	if json.Unmarshal(output.Bytes(), &result) != nil || result["startup"] != "original-shortcuts-restored" || result["service"] != brokerenroll.ServiceMarkedForRemoval || len(result) != 5 {
		t.Fatalf("bounded explicit restoration diagnostic differs: %s", output.String())
	}
}

func TestOperatorDecommissionWithoutStartupKeepsServiceOnlyResultAndNoPublicationRead(t *testing.T) {
	f := &startupDecommissionFixture{t: t}
	var output bytes.Buffer
	err := f.run([]byte("request"), nil, &output)
	want := []string{"read-request", "open", "check", "fence", "bind-fence", "decommission", "close-owner", "close-witness"}
	expected := `{"schema":1,"requestId":"` + strings.Repeat("1", 32) + `","state":"service-decommissioned","service":"marked-for-removal"}` + "\n"
	if err != nil || !reflect.DeepEqual(f.events, want) || output.String() != expected {
		t.Fatalf("service-only semantics changed: events=%v output=%s err=%v", f.events, output.String(), err)
	}
}

func TestOperatorDecommissionStartupFaultsNeverFallBackRetryOrReportSuccess(t *testing.T) {
	for _, fault := range []string{"read-request", "read-publication", "parse", "nil-publication", "cancel-parse", "open", "check", "fence", "bind-fence", "bind-startup", "cancel-startup", "decommission", "wrong-result", "close-owner", "close-witness"} {
		t.Run(fault, func(t *testing.T) {
			f := &startupDecommissionFixture{t: t, requested: true, fault: fault}
			var output bytes.Buffer
			err := f.run([]byte("request"), []byte("original publication"), &output)
			if err == nil || output.Len() != 0 || strings.Contains(err.Error(), "private") {
				t.Fatalf("fault leaked or reported success: events=%v output=%s err=%v", f.events, output.String(), err)
			}
			counts := map[string]int{}
			for _, event := range f.events {
				counts[event]++
			}
			for event, count := range counts {
				if count > 1 {
					t.Fatalf("operation retried after fault: %s %v", event, f.events)
				}
			}
			canMutate := fault == "decommission" || fault == "wrong-result" || fault == "close-owner" || fault == "close-witness"
			if (counts["decommission"] == 1) != canMutate || counts["decommission"] != 0 && !f.bound {
				t.Fatalf("mutation bypassed failed startup binding: %v", f.events)
			}
			canOpen := fault != "read-request" && fault != "read-publication" && fault != "parse" && fault != "nil-publication" && fault != "cancel-parse"
			if (counts["open"] == 1) != canOpen || canOpen && !f.closed {
				t.Fatalf("SCM opened before approval or original owner leaked: %v", f.events)
			}
		})
	}
}

func TestOperatorProtectedUninstallRequiresRetirementBeforeServiceEffects(t *testing.T) {
	f := &startupDecommissionFixture{t: t, requested: true, uninstall: true}
	var output bytes.Buffer
	err := f.run([]byte("request"), []byte("original publication"), &output)
	var result map[string]any
	if err != nil || json.Unmarshal(output.Bytes(), &result) != nil || result["state"] != "protected-retired-service-decommissioned" || result["service"] != brokerenroll.ServiceMarkedForRemoval || result["startup"] != "original-shortcuts-restored" {
		t.Fatalf("bad result: %s %v", output.String(), err)
	}
	want := []string{"read-request", "read-publication", "parse", "open", "check", "fence", "bind-fence", "bind-startup", "decommission", "close-owner", "close-witness"}
	if !reflect.DeepEqual(f.events, want) {
		t.Fatal(f.events)
	}
}
func TestOperatorProtectedUninstallRetirementFailureNeverStopsOrRetries(t *testing.T) {
	f := &startupDecommissionFixture{t: t, uninstall: true, requested: true, fault: "fence"}
	var output bytes.Buffer
	if err := f.run([]byte("request"), []byte("original publication"), &output); err == nil || output.Len() != 0 {
		t.Fatal("retirement failure escaped")
	}
	want := []string{"read-request", "read-publication", "parse", "open", "check", "fence", "close-witness", "close-owner"}
	if !reflect.DeepEqual(f.events, want) {
		t.Fatalf("service effects or retry after uncertain retirement: %v", f.events)
	}
}

func TestOperatorProtectedUninstallMissingStartupEvidenceRefusesBeforeReading(t *testing.T) {
	f := &startupDecommissionFixture{t: t, uninstall: true}
	var output bytes.Buffer
	if err := f.run([]byte("request"), nil, &output); err == nil || output.Len() != 0 || len(f.events) != 0 {
		t.Fatalf("startup ownership omitted: %v", f.events)
	}
}
