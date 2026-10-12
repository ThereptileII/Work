package brokerpipe

import (
	"context"
	"encoding/json"
	"errors"
	"sync"
	"testing"
	"time"

	"example.com/opennav-update-verifier/brokerstore"
)

type fakePeerCapabilityBackend struct {
	mu      sync.Mutex
	failure error
	checks  int
}

func (f *fakePeerCapabilityBackend) check() error {
	f.mu.Lock()
	defer f.mu.Unlock()
	f.checks++
	return f.failure
}
func (f *fakePeerCapabilityBackend) fail(err error) {
	f.mu.Lock()
	defer f.mu.Unlock()
	f.failure = err
}
func privatePeerFixture(t *testing.T) (context.Context, *PeerCapability, *fakePeerCapabilityBackend, PeerIdentity) {
	t.Helper()
	identity := PeerIdentity{Peer: Peer{SID: "S-1-5-21-1-2-3-1001", SessionID: 1, ProcessID: 42}, ProcessCreation: 1234, LogonIDLow: 5678}
	backend := &fakePeerCapabilityBackend{}
	ctx, cap := withPeerCapability(context.Background(), identity, backend)
	t.Cleanup(func() { cap.Close() })
	return ctx, cap, backend, identity
}

type forgedPeerContext struct {
	context.Context
	value any
}

func (f forgedPeerContext) Value(any) any { return f.value }

func TestPeerCapabilityRefusesSerializedPeerAndForgedContext(t *testing.T) {
	ctx, cap, _, identity := privatePeerFixture(t)
	raw, err := json.Marshal(identity.Peer)
	if err != nil {
		t.Fatal(err)
	}
	var peer Peer
	if json.Unmarshal(raw, &peer) != nil {
		t.Fatal("peer fixture decode")
	}
	for _, bad := range []context.Context{nil, context.Background(), context.WithValue(context.Background(), "peer", cap), forgedPeerContext{context.Background(), &PeerCapability{}}, forgedPeerContext{context.Background(), peer}, forgedPeerContext{context.Background(), &PeerCapability{state: &peerCapabilityState{owner: context.Background(), identity: identity}}}} {
		if got, err := AuthorizePeer(bad, peer); err == nil || got != nil {
			t.Fatal("serialized/forged context supplied authority")
		}
	}
	if _, err := json.Marshal(cap); err == nil {
		t.Fatal("capability serialized")
	}
	if _, err := json.Marshal(identity); err == nil {
		t.Fatal("native evidence serialized")
	}
	if json.Unmarshal([]byte(`{}`), &PeerCapability{}) == nil {
		t.Fatal("JSON minted capability")
	}
	for _, wrong := range []Peer{{SID: peer.SID, SessionID: peer.SessionID, ProcessID: peer.ProcessID + 1}, {SID: peer.SID, SessionID: peer.SessionID + 1, ProcessID: peer.ProcessID}, {SID: peer.SID + "0", SessionID: peer.SessionID, ProcessID: peer.ProcessID}} {
		if got, err := AuthorizePeer(ctx, wrong); err == nil || got != nil {
			t.Fatal("forged metadata matched native owner")
		}
	}
	if got, err := AuthorizePeer(ctx, peer); err != nil || got != cap {
		t.Fatal("live context refused", err)
	}
	if (&PeerCapability{}).Validate(ctx) == nil {
		t.Fatal("zero capability accepted")
	}
}

func TestPeerCapabilityRevocationIsPermanentAndContextBound(t *testing.T) {
	for _, scenario := range []string{"disconnect", "dead-process", "reused-process", "different-logon", "closed-token", "closed-capability", "owner-cancel"} {
		t.Run(scenario, func(t *testing.T) {
			identity := PeerIdentity{Peer: Peer{SID: "S-1-5-21-1-2-3-1001", SessionID: 1, ProcessID: 42}, ProcessCreation: 1234, LogonIDLow: 5678}
			owner, cancel := context.WithCancel(context.Background())
			defer cancel()
			backend := &fakePeerCapabilityBackend{}
			ctx, cap := withPeerCapability(owner, identity, backend)
			defer cap.Close()
			if cap.Validate(ctx) != nil {
				t.Fatal("live fixture refused")
			}
			switch scenario {
			case "closed-capability":
				cap.Close()
			case "owner-cancel":
				cancel()
			default:
				backend.fail(ErrRefused)
			}
			if !errors.Is(cap.Validate(context.Background()), ErrPeerRevoked) {
				t.Fatal("native lifetime failure did not revoke")
			}
			backend.fail(nil)
			if _, err := AuthorizePeer(ctx, identity.Peer); !errors.Is(err, ErrPeerRevoked) {
				t.Fatal("matching public metadata restored revoked owner")
			}
			used := false
			if cap.WithPeer(context.Background(), func(PeerIdentity) error { used = true; return nil }) == nil || used {
				t.Fatal("revoked peer reached execution boundary")
			}
		})
	}
	ctx, cap, _, identity := privatePeerFixture(t)
	short, cancel := context.WithCancel(ctx)
	cancel()
	if _, err := AuthorizePeer(short, identity.Peer); !errors.Is(err, context.Canceled) {
		t.Fatal("derived cancellation ignored")
	}
	if cap.Validate(ctx) != nil {
		t.Fatal("a shortened derived context changed original lifetime")
	}
}

func TestPeerCapabilityHandlerScopeRevokesSuccessErrorAndPanic(t *testing.T) {
	for _, scenario := range []string{"success", "error", "panic"} {
		t.Run(scenario, func(t *testing.T) {
			ctx, cap, _, identity := privatePeerFixture(t)
			request := requestFixture()
			handler := func(ctx context.Context, peer Peer, _ Request) (Result, error) {
				if got, err := AuthorizePeer(ctx, peer); err != nil || got != cap {
					t.Fatal("native context not passed to handler", err)
				}
				switch scenario {
				case "error":
					return Result{}, ErrRefused
				case "panic":
					panic("owned fixture")
				}
				return resultFixture(request), nil
			}
			func() {
				defer func() {
					recovered := recover()
					if (recovered != nil) != (scenario == "panic") {
						t.Fatal("unexpected panic", recovered)
					}
				}()
				var replay requests
				_, err := invokeAuthenticatedHandler(ctx, cap, &replay, handler, identity.Peer, request)
				if (err != nil) != (scenario == "error") {
					t.Fatal("handler behavior changed", err)
				}
			}()
			if !errors.Is(cap.Validate(ctx), ErrPeerRevoked) {
				t.Fatal("handler escaped live authority")
			}
		})
	}
}

func TestPeerCapabilityWithPeerSerializesCloseAndRechecksAfterUse(t *testing.T) {
	ctx, cap, backend, identity := privatePeerFixture(t)
	entered := make(chan struct{})
	release := make(chan struct{})
	finished := make(chan error, 1)
	go func() {
		finished <- cap.WithPeer(ctx, func(got PeerIdentity) error {
			if got != identity {
				return ErrRefused
			}
			close(entered)
			<-release
			return nil
		})
	}()
	<-entered
	closed := make(chan struct{})
	go func() { cap.Close(); close(closed) }()
	select {
	case <-closed:
		t.Fatal("close escaped active use")
	case <-time.After(20 * time.Millisecond):
	}
	close(release)
	if err := <-finished; err != nil {
		t.Fatal(err)
	}
	<-closed
	if !errors.Is(cap.Validate(ctx), ErrPeerRevoked) {
		t.Fatal("close did not revoke")
	}
	ctx, cap, backend, _ = privatePeerFixture(t)
	effects := false
	err := cap.WithPeer(ctx, func(PeerIdentity) error { effects = true; backend.fail(ErrRefused); return nil })
	if !effects || !errors.Is(err, ErrPeerRevoked) {
		t.Fatal("post-use native failure lost", err)
	}
}

func TestPeerCapabilityRefusesUnownedExecutorImage(t *testing.T) {
	ctx, cap, _, _ := privatePeerFixture(t)
	used := false
	if err := cap.WithExecutor(ctx, func(PeerIdentity) error { used = true; return nil }); err == nil || used {
		t.Fatal("unbound peer reached executor boundary")
	}
	if cap.BindExecutorImage(ctx, nil) == nil {
		t.Fatal("nil image accepted")
	}
	if cap.BindExecutorImage(ctx, &brokerstore.GenerationExecutorImage{}) == nil {
		t.Fatal("zero image minted execution binding")
	}
	if !errors.Is(cap.Validate(ctx), ErrPeerRevoked) {
		t.Fatal("failed image binding was reusable")
	}
}

func TestPeerCapabilityCopyCannotEscapeRevocation(t *testing.T) {
	ctx, cap, _, identity := privatePeerFixture(t)
	copied := *cap
	forged := forgedPeerContext{context.Background(), &copied}
	if _, err := AuthorizePeer(forged, identity.Peer); err != nil {
		t.Fatal("live copied view lost same ownership", err)
	}
	if err := cap.Close(); err != nil {
		t.Fatal(err)
	}
	if !errors.Is(copied.Validate(ctx), ErrPeerRevoked) {
		t.Fatal("copy escaped shared revocation")
	}
	if _, err := AuthorizePeer(forged, identity.Peer); !errors.Is(err, ErrPeerRevoked) {
		t.Fatal("forged context restored copied authority")
	}
}
