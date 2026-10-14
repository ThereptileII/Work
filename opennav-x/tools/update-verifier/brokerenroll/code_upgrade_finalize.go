package brokerenroll

import (
	"context"
	"crypto/rand"
	"encoding/hex"
	"errors"

	"example.com/opennav-update-verifier/brokerpipe"
)

// Resume owners expose only their selected authenticated phase. The disabled
// transaction keeps its existing recovery sequence; a running finalizer cannot
// reach replacement, service configuration, start, stop or containment cleanup.
type codeUpgradeResumeOwner interface {
	check(context.Context) error
	resume(context.Context) error
	close() error
}
type codeFinalizeBackend interface {
	check(context.Context) error
	status() (serviceState, error)
	bindProcess(uint32) (maintenanceProcess, error)
	close() error
}
type codeFinalizePublication interface {
	check(context.Context) error
	publish(context.Context, func(context.Context) error) error
	close() error
}

type codeUpgradeFinalizer struct {
	self                                             *codeUpgradeFinalizer
	backend                                          codeFinalizeBackend
	publication                                      codeFinalizePublication
	process                                          maintenanceProcess
	pid                                              uint32
	call                                             func(context.Context, brokerpipe.MaintenanceRequest) (brokerpipe.MaintenanceResult, error)
	attempted, publishing, completed, failed, closed bool
	closeErr                                         error
}

func openCodeFinalizerWith(ctx context.Context, backend codeFinalizeBackend, publication codeFinalizePublication, call func(context.Context, brokerpipe.MaintenanceRequest) (brokerpipe.MaintenanceResult, error)) (result *codeUpgradeFinalizer, resultErr error) {
	n := &codeUpgradeFinalizer{backend: backend, publication: publication, call: call}
	n.self = n
	defer func() {
		if resultErr != nil {
			resultErr = errors.Join(resultErr, n.close())
		}
	}()
	if ctx == nil || ctx.Err() != nil || backend == nil || publication == nil || call == nil {
		return nil, errors.New("live native running-pair finalization required")
	}
	if err := errors.Join(backend.check(ctx), publication.check(ctx)); err != nil {
		return nil, err
	}
	state, err := backend.status()
	if err != nil || !maintenanceRunning(state, state.processID) {
		return nil, errors.Join(errors.New("normally running successor required for finalization"), err)
	}
	n.pid = state.processID
	n.process, err = backend.bindProcess(n.pid)
	if err != nil || n.process == nil {
		return nil, errors.Join(errors.New("actual successor process binding required"), err)
	}
	if err := n.check(ctx); err != nil {
		return nil, err
	}
	return n, nil
}
func (n *codeUpgradeFinalizer) check(ctx context.Context) error {
	if n == nil || n.self != n || n.closed || n.failed || ctx == nil || ctx.Err() != nil || n.backend == nil || n.publication == nil || n.process == nil {
		return errors.New("live original running finalizer owner required")
	}
	if err := errors.Join(n.backend.check(ctx), n.publication.check(ctx)); err != nil {
		return err
	}
	state, err := n.backend.status()
	if err != nil || !maintenanceRunning(state, n.pid) {
		return errors.Join(errors.New("same running successor changed during finalization"), err)
	}
	return n.process.live()
}
func (n *codeUpgradeFinalizer) resume(ctx context.Context) (resultErr error) {
	if n == nil || n.self != n || n.attempted || n.failed || n.closed {
		return errors.New("running successor finalization already consumed or revoked")
	}
	n.attempted = true
	defer func() {
		if resultErr != nil {
			n.failed = true
			if n.publishing {
				resultErr = errors.Join(ErrCodeUpgradeUncertain, resultErr)
			}
		}
	}()
	if err := n.check(ctx); err != nil {
		return err
	}
	var nonce [16]byte
	if _, err := rand.Read(nonce[:]); err != nil {
		return err
	}
	page := 0
	q := brokerpipe.MaintenanceRequest{Schema: 1, Operation: brokerpipe.SelectedHistory, RequestID: hex.EncodeToString(nonce[:]), Page: &page}
	r, err := n.call(ctx, q)
	if err != nil || !brokerpipe.ValidMaintenanceClientResult(r, q) {
		return errors.Join(errors.New("running successor registered-client readiness refused"), err)
	}
	if err := n.check(ctx); err != nil {
		return err
	}
	n.publishing = true // An exclusive publication failure can leave partial evidence.
	if err := n.publication.publish(ctx, n.check); err != nil {
		return err
	}
	n.completed = true
	return n.check(ctx)
}
func (n *codeUpgradeFinalizer) close() error {
	if n == nil {
		return nil
	}
	if n.self != n {
		return errors.New("running finalizer owner cannot be copied")
	}
	if n.closed {
		return n.closeErr
	}
	n.closed = true
	// The process observer borrows its image pin. Release it before publication
	// custody closes the exact fixed pair/original-completion files.
	if n.process != nil {
		n.closeErr = errors.Join(n.closeErr, n.process.close())
		n.process = nil
	}
	if n.backend != nil {
		n.closeErr = errors.Join(n.closeErr, n.backend.close())
	}
	if n.publication != nil {
		n.closeErr = errors.Join(n.closeErr, n.publication.close())
	}
	if n.closeErr != nil {
		n.closeErr = errors.Join(ErrCleanup, n.closeErr)
		if n.publishing {
			n.closeErr = errors.Join(ErrCodeUpgradeUncertain, n.closeErr)
		}
	}
	return n.closeErr
}
