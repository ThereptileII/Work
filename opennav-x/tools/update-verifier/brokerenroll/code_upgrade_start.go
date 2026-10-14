package brokerenroll

import (
	"context"
	"encoding/json"
	"errors"
	"time"

	"example.com/opennav-update-verifier/brokerpipe"
)

const codeStartAttemptSuffix = ".code-successor.start-attempt.json"

type codeStartAttempt struct {
	Schema    int    `json:"schema"`
	Original  string `json:"originalCompletionSha256"`
	Policy    string `json:"policySha256"`
	Successor string `json:"successorSha256"`
	Pair      string `json:"pairSha256"`
}

func codeStartAttemptBytes(original, policy, successor, pair string) []byte {
	for _, v := range []string{original, policy, successor, pair} {
		if !validPin(v) {
			return nil
		}
	}
	raw, _ := json.Marshal(codeStartAttempt{1, original, policy, successor, pair})
	return raw
}

type codeStartBackend interface {
	codeFinalizeBackend
	fence(context.Context) error
	start() error
	finalization(context.Context) (codeFinalizeBackend, error)
}

// This owner consumes its durable attempt before the sole start call. It never
// retries, adopts an error-returning start, or releases/rebinds its process during
// the handoff to the query-only finalizer.
type codeUpgradeStarter struct {
	self              *codeUpgradeStarter
	backend           codeStartBackend
	publication       codeFinalizePublication
	call              func(context.Context, brokerpipe.MaintenanceRequest) (brokerpipe.MaintenanceResult, error)
	finalizer         *codeUpgradeFinalizer
	attempted, closed bool
}

func (n *codeUpgradeStarter) check(ctx context.Context) error {
	if n == nil || n.self != n || n.closed || ctx == nil || ctx.Err() != nil {
		return errors.New("live original stopped-start owner required")
	}
	if n.finalizer != nil {
		return n.finalizer.check(ctx)
	}
	return errors.Join(n.backend.check(ctx), n.publication.check(ctx))
}
func (n *codeUpgradeStarter) resume(ctx context.Context) error {
	if n == nil || n.self != n || n.attempted || n.closed {
		return errors.New("stopped-start attempt consumed")
	}
	n.attempted = true
	if err := n.check(ctx); err != nil {
		return err
	}
	state, err := n.backend.status()
	if err != nil || !serviceCompleted(state) {
		return errors.Join(errors.New("clean stopped automatic successor required"), err)
	}
	if err = n.backend.fence(ctx); err != nil {
		return err
	}
	if err = n.check(ctx); err != nil {
		return err
	}
	state, err = n.backend.status()
	if err != nil || !serviceCompleted(state) {
		return errors.Join(errors.New("stopped successor changed before start"), err)
	}
	if err = n.backend.start(); err != nil {
		return err
	}
	bounded, cancel := context.WithTimeout(ctx, 30*time.Second)
	defer cancel()
	for {
		if err = n.check(bounded); err != nil {
			return err
		}
		state, err = n.backend.status()
		if err != nil {
			return err
		}
		if maintenanceRunning(state, state.processID) {
			break
		}
		if state.state != serviceStartPending || state.win32Exit != 0 || state.serviceExit != 0 || state.controls != 0 {
			return errors.New("successor start did not reach normal running state")
		}
		select {
		case <-bounded.Done():
			return bounded.Err()
		case <-time.After(50 * time.Millisecond):
		}
	}
	// The finalizer binds exactly once and retains that process through readiness
	// and publication. No later-instance fallback is possible.
	process, err := n.backend.bindProcess(state.processID)
	if err != nil || process == nil {
		return errors.Join(errors.New("actual started process required"), err)
	}
	observer, err := n.backend.finalization(bounded)
	if err != nil {
		return errors.Join(err, process.close())
	}
	n.finalizer = &codeUpgradeFinalizer{backend: observer, publication: n.publication, call: n.call, process: process, pid: state.processID}
	n.finalizer.self = n.finalizer
	return n.finalizer.resume(bounded)
}
func (n *codeUpgradeStarter) close() error {
	if n == nil {
		return nil
	}
	if n.self != n {
		return errors.New("stopped-start owner cannot be copied")
	}
	if n.closed {
		return nil
	}
	n.closed = true
	if n.finalizer != nil {
		return n.finalizer.close()
	}
	return errors.Join(n.backend.close(), n.publication.close())
}
