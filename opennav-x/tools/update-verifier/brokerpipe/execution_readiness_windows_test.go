package brokerpipe

import (
	"context"
	"errors"
	"sync"
	"testing"
	"time"
	"unsafe"

	"golang.org/x/sys/windows"
)

type nativeReadinessResult struct {
	handle windows.Handle
	err    error
}
type nativeReadinessFixture struct {
	ctx            context.Context
	name, sid      string
	server, client windows.Handle
	connects       []<-chan error
}

func nativeReadinessBusyFixture(t *testing.T) *nativeReadinessFixture {
	t.Helper()
	ctx, cancel := context.WithTimeout(context.Background(), 4*time.Second)
	t.Cleanup(cancel)
	nonce, err := executionNonce()
	if err != nil {
		t.Fatal(err)
	}
	user, err := windows.GetCurrentProcessToken().GetTokenUser()
	if err != nil {
		t.Fatal(err)
	}
	f := &nativeReadinessFixture{ctx: ctx, name: `\\.\pipe\Skager.Inert.CurrentReadiness.` + nonce, sid: user.User.Sid.String()}
	f.server, err = newFixedPipe(f.name, f.sid)
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() {
		cancel()
		for _, done := range f.connects {
			<-done
		}
		if f.client != 0 {
			if err := windows.CloseHandle(f.client); err != nil {
				t.Error(err)
			}
		}
		if f.server != 0 {
			if err := windows.CloseHandle(f.server); err != nil {
				t.Error(err)
			}
		}
	})
	connected := nativeReadinessArm(ctx, f.server)
	f.connects = append(f.connects, connected)
	f.client, err = openFixedPipe(ctx, f.name)
	if err != nil {
		t.Fatal(err)
	}
	select {
	case err := <-connected:
		if err != nil {
			t.Fatal(err)
		}
	case <-ctx.Done():
		t.Fatal(ctx.Err())
	}
	return f
}
func nativeReadinessArm(ctx context.Context, server windows.Handle) <-chan error {
	done := make(chan error, 1)
	go func() {
		_, err := overlapped(ctx, server, func(ov *windows.Overlapped) error { return windows.ConnectNamedPipe(server, ov) })
		done <- err
		close(done)
	}()
	return done
}
func nativeReadinessAwait(t *testing.T, ctx context.Context, signal <-chan struct{}) {
	t.Helper()
	select {
	case <-signal:
	case <-ctx.Done():
		t.Fatal(ctx.Err())
	}
}
func nativeReadinessReceive(t *testing.T, ctx context.Context, result <-chan nativeReadinessResult) nativeReadinessResult {
	t.Helper()
	select {
	case r := <-result:
		return r
	case <-ctx.Done():
		t.Fatal(ctx.Err())
		return nativeReadinessResult{}
	}
}

// Actual unique disposable pipes and original instance closure/recreation only.
// No SCM authenticity/product launch is inferred. Production verifies the real
// server before any request bytes, independently of this readiness wait.
func TestNativeExecutionCurrentReadinessBusyGapThenFreshInstance(t *testing.T) {
	f := nativeReadinessBusyFixture(t)
	busy, gap := make(chan struct{}), make(chan struct{})
	var busyOnce, gapOnce sync.Once
	result := make(chan nativeReadinessResult, 1)
	go func() {
		h, err := openExecutionCurrentReady(f.ctx, f.name, func(err error) {
			if errors.Is(err, windows.ERROR_PIPE_BUSY) {
				busyOnce.Do(func() { close(busy) })
			}
			if errors.Is(err, windows.ERROR_FILE_NOT_FOUND) {
				gapOnce.Do(func() { close(gap) })
			}
		})
		result <- nativeReadinessResult{h, err}
	}()
	nativeReadinessAwait(t, f.ctx, busy)
	if err := windows.CloseHandle(f.client); err != nil {
		t.Fatal(err)
	}
	f.client = 0
	if err := windows.DisconnectNamedPipe(f.server); err != nil && !errors.Is(err, windows.ERROR_PIPE_NOT_CONNECTED) {
		t.Fatal(err)
	}
	if err := windows.CloseHandle(f.server); err != nil {
		t.Fatal(err)
	}
	f.server = 0
	// Synchronize on REAL missing-instance after BUSY, not a scheduling sleep.
	nativeReadinessAwait(t, f.ctx, gap)
	var err error
	f.server, err = newFixedPipe(f.name, f.sid)
	if err != nil {
		t.Fatal(err)
	}
	connected := nativeReadinessArm(f.ctx, f.server)
	f.connects = append(f.connects, connected)
	r := nativeReadinessReceive(t, f.ctx, result)
	if r.err != nil || r.handle == 0 {
		t.Fatal("fresh instance not connected", r.err)
	}
	f.client = r.handle
	select {
	case err = <-connected:
		if err != nil {
			t.Fatal(err)
		}
	case <-f.ctx.Done():
		t.Fatal(f.ctx.Err())
	}
	var pid uint32
	if err = windows.GetNamedPipeServerProcessId(f.client, &pid); err != nil || pid != windows.GetCurrentProcessId() {
		t.Fatal("wrong actual fixture server", pid, err)
	}
	var available uint32
	if ok, _, peekErr := peekPipe.Call(uintptr(f.server), 0, 0, 0, uintptr(unsafe.Pointer(&available)), 0); ok == 0 || available != 0 {
		t.Fatal("readiness published protocol bytes", available, peekErr)
	}
}
func TestNativeExecutionCurrentReadinessCancellationAndDeadline(t *testing.T) {
	for _, mode := range []string{"cancel", "deadline"} {
		t.Run(mode, func(t *testing.T) {
			f := nativeReadinessBusyFixture(t)
			ctx, cancel := context.WithTimeout(f.ctx, 200*time.Millisecond)
			defer cancel()
			busy := make(chan struct{})
			var once sync.Once
			result := make(chan nativeReadinessResult, 1)
			go func() {
				h, err := openExecutionCurrentReady(ctx, f.name, func(err error) {
					if errors.Is(err, windows.ERROR_PIPE_BUSY) {
						once.Do(func() { close(busy) })
					}
				})
				result <- nativeReadinessResult{h, err}
			}()
			nativeReadinessAwait(t, f.ctx, busy)
			want := context.DeadlineExceeded
			if mode == "cancel" {
				want = context.Canceled
				cancel()
			}
			r := nativeReadinessReceive(t, f.ctx, result)
			if r.handle != 0 || !errors.Is(r.err, want) {
				t.Fatal("readiness lost cancellation", r.handle, r.err)
			}
		})
	}
}
func TestNativeExecutionCurrentReadinessInitialMissingAndLegacyBusyRefuse(t *testing.T) {
	f := nativeReadinessBusyFixture(t)
	if h, err := openFixedPipe(f.ctx, f.name); h != 0 || !errors.Is(err, windows.ERROR_PIPE_BUSY) {
		t.Fatal("legacy open gained readiness wait", h, err)
	}
	observed := false
	h, err := openExecutionCurrentReady(f.ctx, f.name+".missing", func(error) { observed = true })
	if h != 0 || !errors.Is(err, windows.ERROR_FILE_NOT_FOUND) || observed {
		t.Fatal("initial absent endpoint entered queue", h, err, observed)
	}
	h, err = openExecutionCurrentReady(f.ctx, "bad\x00endpoint", nil)
	if h != 0 || err == nil {
		t.Fatal("malformed endpoint accepted")
	}
}
