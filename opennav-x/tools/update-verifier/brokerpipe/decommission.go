package brokerpipe

import (
	"context"
	"errors"
	"sync"

	"example.com/opennav-update-verifier/brokerregistration"
)

// DecommissionSnapshot is comparison data, never authority reconstructed from a
// response. Source identity comes from the original authenticated native server.
type DecommissionSnapshot struct {
	RequestID, ClientSID, RegistrationSHA256                        string
	Generation, CompletionSHA256, SelectedHeadSHA256                string
	CurrentProfileSHA256, CurrentProfileIdentitySHA256, FenceSHA256 string
	SourcePID                                                       uint32
	SourceCreation                                                  uint64
}
type decommissionWitnessBackend interface {
	check(context.Context) error
	close() error
}

// DecommissionWitness is a one-conversation original server owner. Only the
// authenticated native client constructs it; zero, copied and serialized values
// cannot authorize SCM effects. Close releases custody even after broker exit.
type DecommissionWitness struct {
	self             *DecommissionWitness
	mu               sync.Mutex
	backend          decommissionWitnessBackend
	snapshot         DecommissionSnapshot
	ctx              context.Context
	closed, poisoned bool
	closeErr         error
}

func emptyDecommissionResult(r MaintenanceResult) bool {
	return r.ClientSID == "" && r.RegistrationSHA256 == "" && r.CurrentProfileSHA256 == "" && r.CurrentProfileIdentitySHA256 == "" && r.FenceSHA256 == ""
}
func validDecommissionResult(r MaintenanceResult, q MaintenanceRequest) bool {
	return q.Operation == PrepareDecommission && r.State == "decommission-prepared" && r.Schema == 1 && r.RequestID == q.RequestID && r.SelectedHeadSHA256 == q.ExpectedSelectedHeadSHA256 && id32.MatchString(r.Generation) && hex64.MatchString(r.CompletionSHA256) && brokerregistration.IsCanonicalUserSID(r.ClientSID) && hex64.MatchString(r.RegistrationSHA256) && hex64.MatchString(r.CurrentProfileSHA256) && hex64.MatchString(r.CurrentProfileIdentitySHA256) && hex64.MatchString(r.FenceSHA256) && r.Version == "" && r.Commit == "" && r.QualificationSHA256 == "" && r.QualificationClosedSHA256 == "" && r.InventorySHA256 == "" && r.Page == nil && r.NextPage == nil && r.Ancestors == nil
}
func newDecommissionWitness(ctx context.Context, q MaintenanceRequest, r MaintenanceResult, pid uint32, creation uint64, b decommissionWitnessBackend) (*DecommissionWitness, error) {
	if ctx == nil || ctx.Err() != nil || !validMaintenanceRequest(q) || !validDecommissionResult(r, q) || pid == 0 || creation == 0 || b == nil {
		return nil, ErrRefused
	}
	w := &DecommissionWitness{ctx: ctx, backend: b, snapshot: DecommissionSnapshot{r.RequestID, r.ClientSID, r.RegistrationSHA256, r.Generation, r.CompletionSHA256, r.SelectedHeadSHA256, r.CurrentProfileSHA256, r.CurrentProfileIdentitySHA256, r.FenceSHA256, pid, creation}}
	w.self = w
	return w, nil
}
func (w *DecommissionWitness) Snapshot() DecommissionSnapshot {
	if w == nil || w.self != w {
		return DecommissionSnapshot{}
	}
	return w.snapshot
}
func (w *DecommissionWitness) Check(ctx context.Context, pid uint32, creation uint64) error {
	if w == nil || w.self != w || ctx == nil {
		return ErrRefused
	}
	w.mu.Lock()
	defer w.mu.Unlock()
	if w.closed || w.poisoned || w.backend == nil || w.ctx == nil || w.snapshot.SourcePID != pid || w.snapshot.SourceCreation != creation {
		return ErrRefused
	}
	if err := errors.Join(ctx.Err(), w.ctx.Err()); err != nil {
		w.poisoned = true
		return errors.Join(ErrRefused, err)
	}
	if err := w.backend.check(ctx); err != nil {
		w.poisoned = true
		return errors.Join(ErrRefused, err)
	}
	return nil
}
func (w *DecommissionWitness) Close() error {
	if w == nil {
		return nil
	}
	if w.self != w || w.backend == nil {
		return ErrRefused
	}
	w.mu.Lock()
	defer w.mu.Unlock()
	if !w.closed {
		w.closed = true
		w.closeErr = w.backend.close()
	}
	return w.closeErr
}
func (*DecommissionWitness) MarshalJSON() ([]byte, error) { return nil, ErrRefused }
func (*DecommissionWitness) UnmarshalJSON([]byte) error   { return ErrRefused }
