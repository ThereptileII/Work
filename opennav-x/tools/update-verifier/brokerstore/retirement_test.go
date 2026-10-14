package brokerstore

import (
	"context"
	"errors"
	"runtime"
	"testing"
	"time"
)

type retirementTestStore struct {
	memoryStore
	prepareCount, commitCount, prepareClose, retirementClose int
	prepareErr, commitErr, checkErr                          error
	receiptCloseErr                                          error
	commitEntered                                            chan struct{}
	commitRelease                                            <-chan struct{}
	terminal                                                 bool
	started, completed                                       bool
	moves                                                    int
}

func (s *retirementTestStore) check() error {
	if s.terminal {
		return ErrDecommissioned
	}
	return s.memoryStore.check()
}
func (s *retirementTestStore) prepareRetirement(_ context.Context, _ DecommissionBinding) (retirementPreparationBackend, error) {
	s.prepareCount++
	if s.prepareErr != nil {
		return nil, s.prepareErr
	}
	return &retirementTestPreparation{s: s}, nil
}

type retirementTestPreparation struct {
	s      *retirementTestStore
	closed bool
}

func (p *retirementTestPreparation) check() error {
	if p.closed {
		return ErrClosed
	}
	return p.s.checkErr
}
func (p *retirementTestPreparation) commit(ctx context.Context) (retirementBackend, error) {
	p.s.commitCount++
	if p.s.commitEntered != nil {
		close(p.s.commitEntered)
		<-p.s.commitRelease
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if p.s.commitErr != nil {
		return &retirementTestReceipt{s: p.s}, p.s.commitErr
	}
	p.s.terminal = true
	return &retirementTestReceipt{s: p.s}, nil
}
func (p *retirementTestPreparation) commitWithStartupDrain(ctx context.Context, guard func(context.Context) error, drain func(context.Context, string) error) (retirementBackend, error) {
	p.s.commitCount++
	p.s.terminal, p.s.started = true, true
	r := &retirementTestReceipt{s: p.s}
	validate := func() error { return errors.Join(ctx.Err(), guard(ctx)) }
	if err := validate(); err != nil {
		return r, errors.Join(ErrRetirementUncertain, err)
	}
	if err := drain(ctx, digest("fence")); err != nil {
		return r, errors.Join(ErrRetirementUncertain, err)
	}
	if err := validate(); err != nil {
		return r, errors.Join(ErrRetirementUncertain, err)
	}
	p.s.moves, p.s.completed = 3, true
	return r, nil
}
func (p *retirementTestPreparation) close() error {
	if !p.closed {
		p.closed = true
		p.s.prepareClose++
	}
	return nil
}

type retirementTestReceipt struct {
	s      *retirementTestStore
	closed bool
}

func (r *retirementTestReceipt) check() error {
	if r.closed {
		return ErrClosed
	}
	return nil
}
func (r *retirementTestReceipt) fenceSHA256() (string, error)    { return digest("fence"), nil }
func (r *retirementTestReceipt) evidenceSHA256() (string, error) { return digest("retirement"), nil }
func (r *retirementTestReceipt) close() error {
	if !r.closed {
		r.closed = true
		r.s.retirementClose++
	}
	return r.s.receiptCloseErr
}

func TestRetirementOriginalHandoffAndTerminalReceipt(t *testing.T) {
	sb := &retirementTestStore{}
	s := &Store{backend: sb}
	b := decommissionTestBinding()
	ownerLive := true
	p, err := s.PrepareRetiringDecommission(context.Background(), b, func(context.Context) error {
		if !ownerLive {
			return testFault
		}
		return s.Validate()
	})
	if err != nil {
		t.Fatal(err)
	}
	if err = p.Validate(context.Background()); err != nil {
		t.Fatal(err)
	}
	ownerLive = false // host's selected owner closes cleanly before the second guard
	r, err := p.Commit(context.Background(), func(context.Context) error {
		if ownerLive {
			return testFault
		}
		return s.Validate()
	})
	if err != nil {
		t.Fatal(err)
	}
	if sb.prepareCount != 1 || sb.commitCount != 1 {
		t.Fatal("original handoff did not occur once")
	}
	if err = r.Validate(context.Background()); err != nil {
		t.Fatal(err)
	}
	if sha, err := r.FenceSHA256(context.Background()); err != nil || sha != digest("fence") {
		t.Fatal("terminal comparison evidence", err)
	}
	if sha, err := r.EvidenceSHA256(context.Background()); err != nil || sha != digest("retirement") {
		t.Fatal("retirement comparison evidence", err)
	}
	if err = s.Validate(); !errors.Is(err, ErrDecommissioned) {
		t.Fatal("ordinary store retained startup authority", err)
	}
	if _, err = p.Commit(context.Background(), func(context.Context) error { t.Fatal("reused guard"); return nil }); err == nil {
		t.Fatal("retirement retried")
	}
	if err = s.Close(); err != nil {
		t.Fatal(err)
	}
	if sb.prepareClose != 1 || sb.retirementClose != 1 {
		t.Fatal("original owners not drained exactly once")
	}
}

func TestRetirementRefusesGuardAndChangedIdentityBeforeCommit(t *testing.T) {
	for _, stage := range []string{"initial guard", "prepared identity", "post-selection guard", "cancelled"} {
		t.Run(stage, func(t *testing.T) {
			sb := &retirementTestStore{}
			s := &Store{backend: sb}
			ctx := context.Background()
			initial := func(context.Context) error { return nil }
			if stage == "initial guard" {
				initial = func(context.Context) error { return testFault }
			}
			p, err := s.PrepareRetiringDecommission(ctx, decommissionTestBinding(), initial)
			if stage == "initial guard" {
				if !errors.Is(err, testFault) || p != nil || sb.prepareCount != 0 {
					t.Fatal("invalid owner prepared", err)
				}
				return
			}
			if err != nil {
				t.Fatal(err)
			}
			if stage == "prepared identity" {
				sb.checkErr = testFault
				if !errors.Is(p.Validate(ctx), testFault) {
					t.Fatal("changed original accepted")
				}
			}
			if stage == "cancelled" {
				var cancel context.CancelFunc
				ctx, cancel = context.WithCancel(ctx)
				cancel()
			}
			post := func(context.Context) error { return nil }
			if stage == "post-selection guard" {
				post = func(context.Context) error { return testFault }
			}
			_, err = p.Commit(ctx, post)
			if err == nil || sb.commitCount != 0 || sb.terminal {
				t.Fatal("premarker refusal published terminal", err)
			}
			if err = s.Close(); err != nil {
				t.Fatal(err)
			}
		})
	}
}

func TestRetirementBackendFailureKeepsOriginalOwnerForDrain(t *testing.T) {
	sb := &retirementTestStore{commitErr: testFault}
	s := &Store{backend: sb}
	p, err := s.PrepareRetiringDecommission(context.Background(), decommissionTestBinding(), func(context.Context) error { return nil })
	if err != nil {
		t.Fatal(err)
	}
	if _, err = p.Commit(context.Background(), func(context.Context) error { return nil }); !errors.Is(err, testFault) {
		t.Fatal("retirement failure lost", err)
	}
	if err = s.Close(); err != nil {
		t.Fatal(err)
	}
	if sb.retirementClose != 1 || sb.prepareClose != 1 {
		t.Fatal("partial original ownership leaked")
	}
}

func TestRetirementCommitAndConcurrentStoreCloseDrainOriginalReceiptOnce(t *testing.T) {
	closeFault := errors.New("retirement receipt close failure")
	entered, release := make(chan struct{}), make(chan struct{})
	sb := &retirementTestStore{commitEntered: entered, commitRelease: release, receiptCloseErr: closeFault}
	s := &Store{backend: sb}
	p, err := s.PrepareRetiringDecommission(context.Background(), decommissionTestBinding(), func(context.Context) error { return nil })
	if err != nil {
		t.Fatal(err)
	}
	commitDone := make(chan error, 1)
	go func() {
		_, err := p.Commit(context.Background(), func(context.Context) error { return nil })
		commitDone <- err
	}()
	select {
	case <-entered:
	case <-time.After(5 * time.Second):
		t.Fatal("native commit did not enter")
	}
	closeDone := make(chan error, 1)
	go func() { closeDone <- s.Close() }()
	// Close must acquire closeMu before it can wait on the Store lock held by
	// Commit. This proves the shutdown overlaps the native operation.
	deadline := time.After(5 * time.Second)
	for {
		if !s.closeMu.TryLock() {
			break
		}
		s.closeMu.Unlock()
		select {
		case <-deadline:
			t.Fatal("store close did not start")
		default:
			runtime.Gosched()
		}
	}
	select {
	case err := <-closeDone:
		t.Fatal("close returned before native receipt registration", err)
	default:
	}
	close(release)
	select {
	case err := <-commitDone:
		if err != nil {
			t.Fatal(err)
		}
	case <-time.After(5 * time.Second):
		t.Fatal("commit did not complete")
	}
	select {
	case err := <-closeDone:
		if !errors.Is(err, closeFault) {
			t.Fatal("receipt close error lost", err)
		}
	case <-time.After(5 * time.Second):
		t.Fatal("close did not drain")
	}
	if sb.commitCount != 1 || sb.prepareClose != 1 || sb.retirementClose != 1 || sb.closes != 1 {
		t.Fatal("original retirement owners not drained exactly once")
	}
	if err := s.Close(); !errors.Is(err, closeFault) {
		t.Fatal("concurrent close error was not sticky", err)
	}
	if sb.retirementClose != 1 || sb.closes != 1 {
		t.Fatal("second close drained original owners again")
	}
}

func TestRetirementStartupDrainOriginalOwnerOrderAndRefusal(t *testing.T) {
	for _, stage := range []string{"success", "initial guard", "terminal guard before", "refused", "lost callback", "cancelled", "terminal guard after"} {
		t.Run(stage, func(t *testing.T) {
			sb := &retirementTestStore{}
			s := &Store{backend: sb}
			p, err := s.PrepareRetiringDecommission(context.Background(), decommissionTestBinding(), func(context.Context) error { return nil })
			if err != nil {
				t.Fatal(err)
			}
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			guards, callbacks := 0, 0
			guard := func(context.Context) error {
				guards++
				if guards > 1 && (!sb.terminal || !sb.started || sb.completed || sb.moves != 0) {
					t.Fatal("terminal guard outside durable pre-move boundary")
				}
				if stage == "initial guard" && guards == 1 || stage == "terminal guard before" && guards == 2 || stage == "terminal guard after" && guards == 3 {
					return testFault
				}
				return nil
			}
			r, err := p.CommitWithStartupDrain(ctx, guard, func(callbackCtx context.Context, fence string) error {
				callbacks++
				if callbackCtx != ctx || fence != digest("fence") || guards != 2 || !sb.terminal || !sb.started || sb.completed || sb.moves != 0 {
					t.Fatal("drain did not receive original cancellable terminal boundary")
				}
				if s.mu.TryLock() {
					s.mu.Unlock()
					t.Fatal("Store cleanup custody released across callback")
				}
				if p.mu.TryLock() {
					p.mu.Unlock()
					t.Fatal("preparation custody released across callback")
				}
				switch stage {
				case "refused":
					return testFault
				case "lost callback":
					return context.DeadlineExceeded
				case "cancelled":
					cancel()
				}
				return nil
			})
			if stage == "success" {
				if err != nil || r == nil || callbacks != 1 || guards != 3 || sb.moves != 3 || !sb.completed {
					t.Fatal("original startup drain failed", err)
				}
			} else {
				if err == nil || r != nil || sb.moves != 0 || sb.completed {
					t.Fatal("failed handoff moved or returned completed receipt", err)
				}
				if stage != "initial guard" && (!errors.Is(err, ErrRetirementUncertain) || !sb.terminal || !sb.started) {
					t.Fatal("terminal failure lost recovery evidence", err)
				}
			}
			if stage == "initial guard" || stage == "terminal guard before" {
				if callbacks != 0 {
					t.Fatal("drain ran despite guard refusal")
				}
			} else if callbacks != 1 {
				t.Fatal("drain did not run once")
			}
			if _, err := p.CommitWithStartupDrain(ctx, guard, func(context.Context, string) error { t.Fatal("callback retried"); return nil }); !errors.Is(err, ErrClosed) {
				t.Fatal("original attempt was reusable", err)
			}
			if err := s.Close(); err != nil {
				t.Fatal(err)
			}
			wantReceiptClose := 1
			if stage == "initial guard" {
				wantReceiptClose = 0
			}
			if sb.prepareClose != 1 || sb.retirementClose != wantReceiptClose {
				t.Fatal("failed original owner not retained for Store cleanup")
			}
		})
	}
}

func TestRetirementStartupDrainRequiresCallback(t *testing.T) {
	sb := &retirementTestStore{}
	s := &Store{backend: sb}
	defer s.Close()
	p, err := s.PrepareRetiringDecommission(context.Background(), decommissionTestBinding(), func(context.Context) error { return nil })
	if err != nil {
		t.Fatal(err)
	}
	if _, err = p.CommitWithStartupDrain(context.Background(), func(context.Context) error { t.Fatal("nil callback reached guard"); return nil }, nil); !errors.Is(err, ErrNativeRequired) || sb.commitCount != 0 || sb.terminal {
		t.Fatal("missing callback accepted", err)
	}
}

func TestRetirementStartupDrainConcurrentCloseRetainsFailedOriginalOwner(t *testing.T) {
	entered, release := make(chan struct{}), make(chan struct{})
	sb := &retirementTestStore{receiptCloseErr: testFault}
	s := &Store{backend: sb}
	p, err := s.PrepareRetiringDecommission(context.Background(), decommissionTestBinding(), func(context.Context) error { return nil })
	if err != nil {
		t.Fatal(err)
	}
	commitDone := make(chan error, 1)
	go func() {
		_, err := p.CommitWithStartupDrain(context.Background(), func(context.Context) error { return nil }, func(context.Context, string) error {
			close(entered)
			<-release
			return context.DeadlineExceeded
		})
		commitDone <- err
	}()
	select {
	case <-entered:
	case <-time.After(5 * time.Second):
		t.Fatal("startup drain did not enter")
	}
	closeDone := make(chan error, 1)
	go func() { closeDone <- s.Close() }()
	deadline := time.After(5 * time.Second)
	for {
		if !s.closeMu.TryLock() {
			break
		}
		s.closeMu.Unlock()
		select {
		case <-deadline:
			t.Fatal("Store close did not overlap drain")
		default:
			runtime.Gosched()
		}
	}
	select {
	case err := <-closeDone:
		t.Fatal("Store closed before callback returned", err)
	default:
	}
	close(release)
	select {
	case err := <-commitDone:
		if !errors.Is(err, context.DeadlineExceeded) {
			t.Fatal("lost callback failure changed", err)
		}
	case <-time.After(5 * time.Second):
		t.Fatal("drain did not return")
	}
	select {
	case err := <-closeDone:
		if !errors.Is(err, testFault) {
			t.Fatal("failed original cleanup error lost", err)
		}
	case <-time.After(5 * time.Second):
		t.Fatal("Store did not drain original owners")
	}
	if sb.moves != 0 || sb.completed || sb.prepareClose != 1 || sb.retirementClose != 1 || sb.closes != 1 {
		t.Fatal("concurrent close lost or repeated original owner cleanup")
	}
}
