package brokerstore

import (
	"context"
	"errors"
	"testing"
)

type retirementTestStore struct {
	memoryStore
	prepareCount, commitCount, prepareClose, retirementClose int
	prepareErr, commitErr, checkErr                          error
	terminal                                                 bool
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
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if p.s.commitErr != nil {
		return &retirementTestReceipt{s: p.s}, p.s.commitErr
	}
	p.s.terminal = true
	return &retirementTestReceipt{s: p.s}, nil
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
	return nil
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
