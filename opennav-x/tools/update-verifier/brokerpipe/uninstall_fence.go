package brokerpipe

import (
	"context"
	"errors"
	"sync"
)

// UninstallFenceWitness is a callback-scoped native original-broker proof. It
// authorizes only the caller's original shortcut restore and child-handle drain.
// It cannot be passed to BindDecommission, serialized, copied, or retained past
// the callback; only a later completed DecommissionWitness can authorize SCM.
type UninstallFenceWitness struct {
	self     *UninstallFenceWitness
	mu       sync.Mutex
	backend  decommissionWitnessBackend // borrowed from the whole conversation
	snapshot DecommissionSnapshot
	ctx      context.Context
	revoked  bool
}

func newUninstallFenceWitness(ctx context.Context, q MaintenanceRequest, r MaintenanceResult, pid uint32, creation uint64, backend decommissionWitnessBackend) (*UninstallFenceWitness, error) {
	if ctx == nil || ctx.Err() != nil || !validUninstallFenceResult(r, q) || pid == 0 || creation == 0 || backend == nil {
		return nil, ErrRefused
	}
	w := &UninstallFenceWitness{backend: backend, ctx: ctx, snapshot: DecommissionSnapshot{r.RequestID, r.ClientSID, r.RegistrationSHA256, r.Generation, r.CompletionSHA256, r.SelectedHeadSHA256, r.CurrentProfileSHA256, r.CurrentProfileIdentitySHA256, r.FenceSHA256, "", pid, creation}}
	w.self = w
	return w, nil
}

func (w *UninstallFenceWitness) Snapshot() DecommissionSnapshot {
	if w == nil || w.self != w {
		return DecommissionSnapshot{}
	}
	w.mu.Lock()
	defer w.mu.Unlock()
	if w.revoked {
		return DecommissionSnapshot{}
	}
	return w.snapshot
}

func (w *UninstallFenceWitness) Check(ctx context.Context, pid uint32, creation uint64) error {
	if w == nil || w.self != w || ctx == nil {
		return ErrRefused
	}
	w.mu.Lock()
	defer w.mu.Unlock()
	if w.revoked || w.backend == nil || w.ctx == nil || w.snapshot.SourcePID != pid || w.snapshot.SourceCreation != creation {
		return ErrRefused
	}
	if err := errors.Join(ctx.Err(), w.ctx.Err(), w.backend.check(ctx)); err != nil {
		w.revoked = true
		return errors.Join(ErrRefused, err)
	}
	return nil
}

func (w *UninstallFenceWitness) revoke() {
	if w != nil {
		w.mu.Lock()
		w.revoked = true
		w.mu.Unlock()
	}
}

func (*UninstallFenceWitness) MarshalJSON() ([]byte, error) { return nil, ErrRefused }
func (*UninstallFenceWitness) UnmarshalJSON([]byte) error   { return ErrRefused }
