package brokerhost

import (
	"context"
	"errors"
	"testing"
	"time"
)

func TestBrokerEndpointsRefuseBeforeStarting(t *testing.T) {
	ctx, cancel := context.WithCancel(context.Background())
	cancel()
	called := false
	endpoint := func(context.Context) error { called = true; return nil }
	for _, item := range []struct {
		ctx  context.Context
		a, b func(context.Context) error
	}{{nil, endpoint, endpoint}, {context.Background(), nil, endpoint},
		{context.Background(), endpoint, nil}, {ctx, endpoint, endpoint}} {
		if serveBrokerEndpoints(item.ctx, item.a, item.b) == nil || called {
			t.Fatal("invalid service started an endpoint")
		}
	}
}

func TestBrokerEndpointsFailureCancelsButWaitsForBothOwners(t *testing.T) {
	fault := errors.New("native endpoint fault")
	cancelled, release, done := make(chan struct{}), make(chan struct{}), make(chan error, 1)
	go func() {
		done <- serveBrokerEndpoints(context.Background(),
			func(context.Context) error { return fault },
			func(ctx context.Context) error {
				<-ctx.Done()
				close(cancelled)
				<-release
				return ctx.Err()
			})
	}()
	select {
	case <-cancelled:
	case <-time.After(5 * time.Second):
		t.Fatal("other protected endpoint was not cancelled")
	}
	select {
	case err := <-done:
		t.Fatal("returned while second endpoint retained owners", err)
	default:
	}
	close(release)
	if err := <-done; !errors.Is(err, fault) {
		t.Fatal("lost original endpoint failure", err)
	}
}

func TestBrokerEndpointsCancellationDoesNotHideCleanupFailure(t *testing.T) {
	for _, cleanup := range []bool{false, true} {
		t.Run(map[bool]string{false: "clean", true: "cleanup-failure"}[cleanup], func(t *testing.T) {
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			started := make(chan struct{}, 2)
			fault := errors.New("retained owner cleanup failed")
			endpoint := func(ctx context.Context) error { started <- struct{}{}; <-ctx.Done(); return ctx.Err() }
			other := func(ctx context.Context) error {
				started <- struct{}{}
				<-ctx.Done()
				if cleanup {
					return errors.Join(ctx.Err(), fault)
				}
				return ctx.Err()
			}
			done := make(chan error, 1)
			go func() { done <- serveBrokerEndpoints(ctx, endpoint, other) }()
			<-started
			<-started
			cancel()
			err := <-done
			if cleanup && !errors.Is(err, fault) {
				t.Fatal("cancel hid integrity failure", err)
			}
			if !cleanup && err != context.Canceled {
				t.Fatal("clean shutdown differed", err)
			}
		})
	}
}

func TestBrokerEndpointsUnexpectedReturnOrPanicDrainsPeer(t *testing.T) {
	for _, panicEndpoint := range []bool{false, true} {
		t.Run(map[bool]string{false: "unexpected-return", true: "panic"}[panicEndpoint], func(t *testing.T) {
			drained := make(chan struct{})
			err := serveBrokerEndpoints(context.Background(),
				func(context.Context) error {
					if panicEndpoint {
						panic("private error must not escape")
					}
					return nil
				},
				func(ctx context.Context) error { <-ctx.Done(); close(drained); return ctx.Err() })
			want := errBrokerEndpointStopped
			if panicEndpoint {
				want = errBrokerEndpointPanic
			}
			if !errors.Is(err, want) {
				t.Fatal("unexpected endpoint exit was accepted", err)
			}
			select {
			case <-drained:
			default:
				t.Fatal("peer not drained")
			}
		})
	}
}

func TestBrokerMaintenanceEndpointDrainsEveryOwnerAndPreservesFailure(t *testing.T) {
	fault := errors.New("maintenance cleanup failed")
	started := make(chan struct{}, 2)
	cancelled := make(chan struct{}, 2)
	release := make(chan struct{})
	peer := func(ctx context.Context) error {
		started <- struct{}{}
		<-ctx.Done()
		cancelled <- struct{}{}
		<-release
		return ctx.Err()
	}
	done := make(chan error, 1)
	go func() {
		done <- serveBrokerEndpoints(context.Background(), peer, peer, func(context.Context) error {
			<-started
			<-started
			return fault
		})
	}()
	for range 2 {
		select {
		case <-cancelled:
		case <-time.After(5 * time.Second):
			t.Fatal("maintenance failure did not cancel all peers")
		}
	}
	select {
	case err := <-done:
		t.Fatal("released service while a protected endpoint retained owners", err)
	default:
	}
	close(release)
	if err := <-done; !errors.Is(err, fault) {
		t.Fatal("lost maintenance failure", err)
	}
	called := false
	endpoint := func(context.Context) error { called = true; return nil }
	if serveBrokerEndpoints(context.Background(), endpoint, endpoint, nil) == nil || called {
		t.Fatal("nil maintenance endpoint started other endpoints")
	}
}
