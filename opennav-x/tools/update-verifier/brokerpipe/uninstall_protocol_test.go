package brokerpipe

import (
	"context"
	"encoding/json"
	"errors"
	"reflect"
	"strings"
	"sync/atomic"
	"testing"
	"time"
)

func uninstallFixture() (MaintenanceRequest, MaintenanceResult, MaintenanceResult) {
	q, final := decommissionFixture()
	q.Operation = PrepareUninstall
	final.State, final.RetirementSHA256 = "uninstall-prepared", strings.Repeat("9", 64)
	interim := final
	interim.State, interim.RetirementSHA256 = "uninstall-fenced", ""
	return q, interim, final
}

func uninstallJSON(v any) []byte { raw, _ := json.Marshal(v); return raw }

type uninstallConnectionFixture struct {
	readFn  func(context.Context) ([]byte, error)
	writeFn func(context.Context, []byte) error
}

func (f uninstallConnectionFixture) read(ctx context.Context) ([]byte, error) {
	return f.readFn(ctx)
}
func (f uninstallConnectionFixture) write(ctx context.Context, raw []byte) error {
	return f.writeFn(ctx, raw)
}
func (uninstallConnectionFixture) idle() error { return nil }

func uninstallChannelConnection(input, output chan []byte) uninstallConnectionFixture {
	return uninstallConnectionFixture{
		readFn: func(ctx context.Context) ([]byte, error) {
			select {
			case raw := <-input:
				return raw, ctx.Err()
			case <-ctx.Done():
				return nil, ctx.Err()
			}
		},
		writeFn: func(ctx context.Context, raw []byte) error {
			select {
			case output <- raw:
				return ctx.Err()
			case <-ctx.Done():
				return ctx.Err()
			}
		},
	}
}

func TestProtectedUninstallTwoPhaseOrdersDrainBeforeRetirement(t *testing.T) {
	q, interim, final := uninstallFixture()
	ctx, cancel := context.WithTimeout(context.Background(), time.Second)
	defer cancel()
	toServer, toClient := make(chan []byte, 1), make(chan []byte, 1)
	server, client := uninstallChannelConnection(toServer, toClient), uninstallChannelConnection(toClient, toServer)
	var serverIncoming, clientIncoming, drained atomic.Bool
	check := func(input chan []byte, incoming *atomic.Bool) func(context.Context) error {
		return func(ctx context.Context) error {
			if ctx.Err() != nil || (!incoming.Load() && len(input) != 0) {
				return ErrRefused
			}
			return nil
		}
	}
	done := make(chan error, 1)
	go func() {
		raw, err := server.read(ctx)
		if err != nil {
			done <- err
			return
		}
		request, err := decodeMaintenanceRequest(raw)
		if err != nil || request != q {
			done <- ErrRefused
			return
		}
		handler := func(ctx context.Context, _ Peer, _ MaintenanceRequest, publish func(context.Context, MaintenanceResult) error) (MaintenanceResult, error) {
			if drained.Load() {
				return MaintenanceResult{}, errors.New("drained before authenticated fence")
			}
			if err := publish(ctx, interim); err != nil {
				return MaintenanceResult{}, err
			}
			if !drained.Load() {
				return MaintenanceResult{}, errors.New("retirement before drain")
			}
			return final, nil
		}
		result, err := invokeProtectedUninstallHandler(ctx, Peer{SID: interim.ClientSID}, request, handler, server, check(toServer, &serverIncoming), &serverIncoming)
		if err == nil {
			err = server.write(ctx, uninstallJSON(result))
		}
		done <- err
	}()
	var borrowed *UninstallFenceWitness
	result, err := callProtectedUninstallConversation(ctx, q, interim.ClientSID, 11, 12, client, check(toClient, &clientIncoming), &clientIncoming, func(ctx context.Context, w *UninstallFenceWitness) error {
		borrowed = w
		if w.Snapshot().RetirementSHA256 != "" || w.Snapshot().FenceSHA256 != interim.FenceSHA256 {
			return ErrRefused
		}
		if err := w.Check(ctx, 11, 12); err != nil {
			return err
		}
		drained.Store(true)
		return nil
	})
	if err != nil || result != final {
		t.Fatal("two-phase conversation failed", result, err)
	}
	if err := <-done; err != nil {
		t.Fatal(err)
	}
	if borrowed == nil || borrowed.Check(context.Background(), 11, 12) == nil || borrowed.Snapshot() != (DecommissionSnapshot{}) {
		t.Fatal("borrowed fence survived callback")
	}
	if serverIncoming.Load() || clientIncoming.Load() {
		t.Fatal("input remained armed")
	}
}

func TestProtectedUninstallInterimAndContinuationStrictBindings(t *testing.T) {
	q, interim, final := uninstallFixture()
	raw := uninstallJSON(interim)
	if _, err := decodeUninstallFenceResult(raw, q); err != nil || !validUninstallContinue(uninstallContinue(q, interim.FenceSHA256), q, interim.FenceSHA256) {
		t.Fatal("exact protocol refused", err)
	}
	for _, bad := range [][]byte{
		uninstallJSON(final),
		[]byte(strings.Replace(string(raw), `"schema":1`, `"schema":1,"schema":1`, 1)),
		[]byte(strings.TrimSuffix(string(raw), "}") + `,"retirementSha256":""}`),
		[]byte(strings.TrimSuffix(string(raw), "}") + `,"unknown":1}`),
		[]byte(strings.Replace(string(raw), q.ExpectedSelectedHeadSHA256, strings.Repeat("a", 64), 1)),
		[]byte(strings.Repeat(" ", maxFrame+1)),
	} {
		if _, err := decodeUninstallFenceResult(bad, q); err == nil {
			t.Fatal("invalid interim accepted", string(bad))
		}
	}
	continuation := string(uninstallContinue(q, interim.FenceSHA256))
	for _, bad := range []string{
		strings.Replace(continuation, q.RequestID, strings.Repeat("a", 32), 1),
		strings.Replace(continuation, interim.FenceSHA256, strings.Repeat("a", 64), 1),
		strings.Replace(continuation, `"schema":1`, `"schema":1,"schema":1`, 1),
		strings.Replace(continuation, `"kind":`, `"Kind":`, 1),
		strings.TrimSuffix(continuation, "}") + `,"unknown":1}`,
		continuation + continuation,
		string(maintenanceAck(q)),
		string(uninstallJSON(final)),
	} {
		if validUninstallContinue([]byte(bad), q, interim.FenceSHA256) {
			t.Fatal("invalid continuation accepted", bad)
		}
	}
}

func TestProtectedUninstallServerRefusesBypassAndDuplicatePublication(t *testing.T) {
	q, interim, final := uninstallFixture()
	for _, test := range []struct {
		name string
		run  func(context.Context, func(context.Context, MaintenanceResult) error) (MaintenanceResult, error)
	}{
		{"final-before-fence", func(context.Context, func(context.Context, MaintenanceResult) error) (MaintenanceResult, error) {
			return final, nil
		}},
		{"duplicate-publication-ignored", func(ctx context.Context, publish func(context.Context, MaintenanceResult) error) (MaintenanceResult, error) {
			if err := publish(ctx, interim); err != nil {
				return MaintenanceResult{}, err
			}
			_ = publish(ctx, interim)
			return final, nil
		}},
		{"invalid-interim-ignored", func(ctx context.Context, publish func(context.Context, MaintenanceResult) error) (MaintenanceResult, error) {
			_ = publish(ctx, final)
			return final, nil
		}},
		{"changed-final-fence", func(ctx context.Context, publish func(context.Context, MaintenanceResult) error) (MaintenanceResult, error) {
			if err := publish(ctx, interim); err != nil {
				return MaintenanceResult{}, err
			}
			bad := final
			bad.FenceSHA256 = strings.Repeat("a", 64)
			return bad, nil
		}},
	} {
		t.Run(test.name, func(t *testing.T) {
			var incoming atomic.Bool
			connection := uninstallConnectionFixture{readFn: func(context.Context) ([]byte, error) { return uninstallContinue(q, interim.FenceSHA256), nil }, writeFn: func(context.Context, []byte) error { return nil }}
			handler := func(ctx context.Context, _ Peer, _ MaintenanceRequest, publish func(context.Context, MaintenanceResult) error) (MaintenanceResult, error) {
				return test.run(ctx, publish)
			}
			result, err := invokeProtectedUninstallHandler(context.Background(), Peer{SID: interim.ClientSID}, q, handler, connection, func(ctx context.Context) error { return ctx.Err() }, &incoming)
			if err == nil || result != (MaintenanceResult{}) || incoming.Load() {
				t.Fatal("server accepted invalid sequence", result, err)
			}
		})
	}
}

func TestProtectedUninstallServerContinuationCancellationTimeoutAndInput(t *testing.T) {
	q, interim, final := uninstallFixture()
	for _, name := range []string{"wrong-request", "wrong-fence", "duplicate-input", "disconnect", "canceled", "deadline", "shortened-publish-context"} {
		t.Run(name, func(t *testing.T) {
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			var incoming atomic.Bool
			read := false
			connection := uninstallConnectionFixture{
				writeFn: func(context.Context, []byte) error { return nil },
				readFn: func(ctx context.Context) ([]byte, error) {
					read = true
					switch name {
					case "wrong-request":
						bad := q
						bad.RequestID = strings.Repeat("a", 32)
						return uninstallContinue(bad, interim.FenceSHA256), nil
					case "wrong-fence":
						return uninstallContinue(q, strings.Repeat("a", 64)), nil
					case "disconnect":
						return nil, errors.New("pipe disconnected")
					case "canceled":
						cancel()
						<-ctx.Done()
						return nil, ctx.Err()
					case "deadline", "shortened-publish-context":
						<-ctx.Done()
						return nil, ctx.Err()
					}
					return uninstallContinue(q, interim.FenceSHA256), nil
				},
			}
			if name == "deadline" {
				var stop context.CancelFunc
				ctx, stop = context.WithTimeout(ctx, 5*time.Millisecond)
				defer stop()
			}
			handler := func(ctx context.Context, _ Peer, _ MaintenanceRequest, publish func(context.Context, MaintenanceResult) error) (MaintenanceResult, error) {
				if name == "shortened-publish-context" {
					var stop context.CancelFunc
					ctx, stop = context.WithTimeout(ctx, 5*time.Millisecond)
					defer stop()
				}
				_ = publish(ctx, interim) // An ignored exchange error cannot authorize a final.
				return final, nil
			}
			check := func(ctx context.Context) error {
				if name == "duplicate-input" && read && !incoming.Load() {
					return ErrRefused
				}
				return ctx.Err()
			}
			if _, err := invokeProtectedUninstallHandler(ctx, Peer{SID: interim.ClientSID}, q, handler, connection, check, &incoming); err == nil || incoming.Load() {
				t.Fatal("failed continuation authorized final", err)
			}
		})
	}
}

func TestProtectedUninstallPublisherRevokedAtHandlerReturnAndPanic(t *testing.T) {
	q, interim, final := uninstallFixture()
	for _, panicHandler := range []bool{false, true} {
		t.Run(map[bool]string{false: "return-with-active-publisher", true: "panic"}[panicHandler], func(t *testing.T) {
			ctx, cancel := context.WithTimeout(context.Background(), time.Second)
			defer cancel()
			var incoming atomic.Bool
			written, done := make(chan struct{}), make(chan error, 1)
			connection := uninstallConnectionFixture{
				writeFn: func(context.Context, []byte) error { close(written); return nil },
				readFn:  func(ctx context.Context) ([]byte, error) { <-ctx.Done(); return nil, ctx.Err() },
			}
			var retained func(context.Context, MaintenanceResult) error
			handler := func(ctx context.Context, _ Peer, _ MaintenanceRequest, publish func(context.Context, MaintenanceResult) error) (MaintenanceResult, error) {
				retained = publish
				go func() { done <- publish(ctx, interim) }()
				<-written
				if panicHandler {
					panic("handler panic")
				}
				return final, nil
			}
			func() {
				defer func() {
					if p := recover(); p != nil && !panicHandler {
						t.Fatal(p)
					}
				}()
				if _, err := invokeProtectedUninstallHandler(ctx, Peer{SID: interim.ClientSID}, q, handler, connection, func(ctx context.Context) error { return ctx.Err() }, &incoming); err == nil {
					t.Fatal("active publisher outlived handler")
				}
			}()
			if err := <-done; err == nil || ctx.Err() != nil || incoming.Load() {
				t.Fatal("reader not canceled and joined at handler exit", err, ctx.Err())
			}
			if retained(context.Background(), interim) == nil {
				t.Fatal("retained publisher revived")
			}
		})
	}
}

func TestProtectedUninstallClientRefusesEarlyFinalChangedBindingAndCallbackFailure(t *testing.T) {
	q, interim, final := uninstallFixture()
	for _, name := range []string{"final-before-interim", "duplicate-interim", "changed-registration", "changed-profile", "changed-profile-identity", "changed-generation", "changed-completion", "changed-fence", "missing-retirement", "wrong-client", "callback-error", "callback-cancel", "pending-final-during-callback", "callback-panic"} {
		t.Run(name, func(t *testing.T) {
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			var incoming atomic.Bool
			reads, writes, callbacks := 0, 0, 0
			var borrowed *UninstallFenceWitness
			badFinal, badInterim := final, interim
			switch name {
			case "changed-registration":
				badFinal.RegistrationSHA256 = strings.Repeat("a", 64)
			case "changed-profile":
				badFinal.CurrentProfileSHA256 = strings.Repeat("a", 64)
			case "changed-profile-identity":
				badFinal.CurrentProfileIdentitySHA256 = strings.Repeat("a", 64)
			case "changed-generation":
				badFinal.Generation = strings.Repeat("a", 32)
			case "changed-completion":
				badFinal.CompletionSHA256 = strings.Repeat("a", 64)
			case "changed-fence":
				badFinal.FenceSHA256 = strings.Repeat("a", 64)
			case "missing-retirement":
				badFinal.RetirementSHA256 = ""
			case "wrong-client":
				badInterim.ClientSID = "S-1-5-21-1-2-3-1002"
			}
			connection := uninstallConnectionFixture{
				readFn: func(context.Context) ([]byte, error) {
					reads++
					if name == "final-before-interim" {
						return uninstallJSON(final), nil
					}
					if reads == 1 || name == "duplicate-interim" {
						return uninstallJSON(badInterim), nil
					}
					return uninstallJSON(badFinal), nil
				},
				writeFn: func(context.Context, []byte) error { writes++; return nil },
			}
			check := func(ctx context.Context) error {
				if name == "pending-final-during-callback" && callbacks != 0 && !incoming.Load() {
					return ErrRefused
				}
				return ctx.Err()
			}
			callback := func(ctx context.Context, w *UninstallFenceWitness) error {
				callbacks++
				borrowed = w
				if name == "callback-error" {
					return errors.New("drain failed")
				}
				if name == "callback-cancel" {
					cancel()
				}
				if name == "callback-panic" {
					panic("drain panic")
				}
				return nil
			}
			var err error
			func() {
				defer func() {
					if p := recover(); p != nil {
						if name != "callback-panic" {
							t.Fatal(p)
						}
						err = ErrRefused
					}
				}()
				_, err = callProtectedUninstallConversation(ctx, q, interim.ClientSID, 11, 12, connection, check, &incoming, callback)
			}()
			if err == nil || incoming.Load() {
				t.Fatal("invalid client sequence accepted", err)
			}
			if borrowed != nil && (borrowed.Check(context.Background(), 11, 12) == nil || borrowed.Snapshot() != (DecommissionSnapshot{})) {
				t.Fatal("borrowed witness escaped failed callback")
			}
			if strings.HasPrefix(name, "callback-") || name == "pending-final-during-callback" {
				if writes != 1 {
					t.Fatal("continued after failed restore/drain", writes)
				}
			}
			if (name == "final-before-interim" || name == "wrong-client") && callbacks != 0 {
				t.Fatal("unfenced callback invoked")
			}
		})
	}
}

func TestUninstallFenceWitnessBorrowedSealedAndSticky(t *testing.T) {
	q, interim, _ := uninstallFixture()
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	backend := &decommissionWitnessFixture{}
	w, err := newUninstallFenceWitness(ctx, q, interim, 11, 12, backend)
	if err != nil {
		t.Fatal(err)
	}
	copyW := reflect.New(reflect.TypeOf(w).Elem())
	copyW.Elem().Set(reflect.ValueOf(w).Elem())
	for _, bad := range []*UninstallFenceWitness{nil, {}, copyW.Interface().(*UninstallFenceWitness)} {
		if bad.Check(ctx, 11, 12) == nil || bad.Snapshot() != (DecommissionSnapshot{}) {
			t.Fatal("unsealed fence authority")
		}
	}
	if _, err := json.Marshal(w); err == nil || json.Unmarshal([]byte(`{}`), w) == nil {
		t.Fatal("serialized fence authority")
	}
	if w.Check(ctx, 11, 13) == nil {
		t.Fatal("wrong broker birth accepted")
	}
	backend.failure = errors.New("original pipe owner lost")
	if w.Check(ctx, 11, 12) == nil {
		t.Fatal("lost owner accepted")
	}
	backend.failure = nil
	if w.Check(ctx, 11, 12) == nil || backend.checks != 1 {
		t.Fatal("borrowed authority recovered after failure")
	}
	w.revoke()
	if backend.closes != 0 {
		t.Fatal("borrowed witness closed conversation owner")
	}
	if w.Snapshot() != (DecommissionSnapshot{}) {
		t.Fatal("revoked fence snapshot survived")
	}
}

func TestOldCallDecommissionRefusesProtectedUninstall(t *testing.T) {
	q, _, _ := uninstallFixture()
	if w, err := CallDecommission(context.Background(), q); w != nil || !errors.Is(err, ErrRefused) {
		t.Fatal("old single-phase client accepted protected uninstall", w, err)
	}
}
