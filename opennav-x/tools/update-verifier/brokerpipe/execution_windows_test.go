//go:build windows

package brokerpipe

import (
	"context"
	"errors"
	"io"
	"runtime"
	"sync/atomic"
	"testing"
	"time"

	"golang.org/x/sys/windows"
)

// Disposable unique pipes and this real interactive process/token only. SCM and
// protected-image establishment is bypassed only in private test owners. These
// fixtures never start a service, application or child and grant no launch claim.
func TestNativeExecutionInputArmingPreservesStagingRefusal(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	f := actualNativePeerFixture(t)
	var incoming atomic.Bool
	duplex := &executionNativePeer{native: f.backend, incoming: &incoming}
	if duplex.check() != nil || f.backend.check() != nil {
		t.Fatal("live empty pipe refused")
	}
	if _, err := (pipeIO{f.ctx, f.client}).Write([]byte{7}); err != nil {
		t.Fatal(err)
	}
	if duplex.check() == nil || f.backend.check() == nil {
		t.Fatal("unrequested input accepted")
	}
	incoming.Store(true)
	if err := duplex.check(); err != nil {
		t.Fatal("issued report bytes refused", err)
	}
	if f.backend.check() == nil {
		t.Fatal("staging accepted execution input")
	}
	var b [1]byte
	if _, err := io.ReadFull(pipeIO{f.ctx, f.pipe}, b[:]); err != nil || b[0] != 7 {
		t.Fatal("native input corrupted", err)
	}
	incoming.Store(false)
	if duplex.check() != nil {
		t.Fatal("drained input refused")
	}
	if err := windows.CloseHandle(f.client); err != nil {
		t.Fatal(err)
	}
	f.client = 0
	if duplex.check() == nil {
		t.Fatal("disconnected native execution remained valid")
	}
}
func TestNativeExecutionChallengedFramesRoundTrip(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	f := actualNativePeerFixture(t)
	ctx, cancel := context.WithCancel(f.ctx)
	defer cancel()
	a := &fakeExecutionAuthority{}
	s, err := newExecutionSession(ctx, cancel, executionTestRequest(), a, executionNativeConnection{f.pipe})
	if err != nil {
		t.Fatal(err)
	}
	defer s.Close()
	bindExecutionTestSession(t, s)
	done := make(chan error, 1)
	go func() {
		sequence := executionClientSequence{request: executionTestRequest()}
		for _, phase := range []ExecutionPhase{InspectPrevious, FinishExecution} {
			raw, err := readFrame(pipeIO{ctx, f.client})
			if err != nil {
				done <- err
				return
			}
			command, err := decodeExecutionCommand(raw, "command")
			if err != nil || sequence.accept(command) != nil || command.Phase != phase {
				done <- ErrRefused
				return
			}
			observation := executionTestObservation()
			if phase == FinishExecution {
				observation.Status = "reported-failed"
				observation.EvidenceSHA256 = ""
			}
			raw, err = encodeExecutionReport(command, observation)
			if err != nil {
				done <- err
				return
			}
			if err := writeFrame(pipeIO{ctx, f.client}, raw); err != nil {
				done <- err
				return
			}
		}
		done <- nil
	}()
	for _, phase := range []ExecutionPhase{InspectPrevious, FinishExecution} {
		if _, err := s.Exchange(ctx, phase); err != nil {
			cancel()
			<-done
			t.Fatal(err)
		}
	}
	if err := <-done; err != nil {
		t.Fatal(err)
	}
	if err := s.finishHandlerScope(); err != nil {
		t.Fatal(err)
	}
	if a.closes != 1 {
		t.Fatal("authority cleanup repeated")
	}
	terminal := s.state.terminal
	for _, kind := range []string{"closed", "closed-ack"} {
		raw, err := executionClosingFrame(terminal, kind)
		if err != nil || !validExecutionClosingFrame(raw, terminal, kind) {
			t.Fatal(kind, err)
		}
	}
}
func TestNativeExecutionWatcherJoinsAfterDisconnect(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	f := actualNativePeerFixture(t)
	ctx, cancel := context.WithCancel(f.ctx)
	defer cancel()
	var incoming atomic.Bool
	incoming.Store(true)
	native := &executionNativePeer{native: f.backend, incoming: &incoming}
	join := executionWatcher(ctx, cancel, native.check)
	defer join()
	done := make(chan error, 1)
	go func() { _, err := (executionNativeConnection{f.pipe}).read(ctx); done <- err }()
	if err := windows.CloseHandle(f.client); err != nil {
		t.Fatal(err)
	}
	f.client = 0
	select {
	case err := <-done:
		if err == nil {
			t.Fatal("disconnected read accepted")
		}
	case <-time.After(2 * time.Second):
		t.Fatal("pending native read not drained")
	}
	select {
	case <-ctx.Done():
	case <-time.After(2 * time.Second):
		t.Fatal("watcher did not cancel")
	}
	join()
	if !errors.Is(ctx.Err(), context.Canceled) {
		t.Fatal("disconnect cancellation lost")
	}
}
