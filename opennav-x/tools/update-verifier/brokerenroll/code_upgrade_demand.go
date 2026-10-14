package brokerenroll

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
)

const codeDemandAttemptSuffix = ".code-successor.demand-attempt.json"

type codeDemandAttempt struct {
	Schema    int    `json:"schema"`
	Purpose   string `json:"purpose"`
	Original  string `json:"originalCompletionSha256"`
	Policy    string `json:"policySha256"`
	Successor string `json:"successorSha256"`
	Pair      string `json:"pairSha256"`
}

func codeDemandAttemptBytes(original, policy, successor, pair string) []byte {
	for _, v := range []string{original, policy, successor, pair} {
		if !validPin(v) {
			return nil
		}
	}
	raw, _ := json.Marshal(codeDemandAttempt{1, "validated-code-pair-demand-to-auto", original, policy, successor, pair})
	return raw
}

func matchCodeDemandAttempt(raw, expected []byte) error {
	if len(expected) == 0 || len(raw) == 0 || !bytes.Equal(raw, expected) {
		return errors.New("demand transition attempt differs from exact validated pair")
	}
	return nil
}

// A separate capability may change only the authenticated start type. It cannot
// START or STOP, and its cleanup never attempts containment/configuration. After
// its handle is released the existing AUTO dispatcher reauthenticates everything
// and applies its independent durable normal-start fence.
type codeDemandBackend interface {
	check(context.Context) error
	status() (serviceState, error)
	fence(context.Context) error
	automatic(context.Context) error
	close() error
}

type codeUpgradeDemandTransition struct {
	self                                *codeUpgradeDemandTransition
	backend                             codeDemandBackend
	publication                         codeFinalizePublication
	openAutomatic                       func(context.Context) (codeUpgradeResumeOwner, error)
	next                                codeUpgradeResumeOwner
	attempted, effect, released, closed bool
	releaseErr                          error
	closeErr                            error
}

func (n *codeUpgradeDemandTransition) check(ctx context.Context) error {
	if n == nil || n.self != n || n.closed || ctx == nil || ctx.Err() != nil {
		return errors.New("live original demand transition owner required")
	}
	if n.next != nil {
		return n.next.check(ctx)
	}
	if n.released || n.backend == nil || n.publication == nil || n.openAutomatic == nil {
		return errors.New("retained demand transition capability required")
	}
	return errors.Join(n.backend.check(ctx), n.publication.check(ctx))
}

func (n *codeUpgradeDemandTransition) resume(ctx context.Context) (resultErr error) {
	if n == nil || n.self != n || n.closed || n.attempted {
		return errors.New("demand transition attempt consumed")
	}
	n.attempted = true
	defer func() {
		if resultErr != nil && n.effect {
			resultErr = errors.Join(ErrCodeUpgradeUncertain, resultErr)
		}
	}()
	if err := n.check(ctx); err != nil {
		return err
	}
	state, err := n.backend.status()
	if err != nil || !serviceCompleted(state) {
		return errors.Join(errors.New("validated demand successor must remain clean stopped"), err)
	}
	// Even a failed exclusive write can leave a consumed partial namespace.
	n.effect = true
	if err = n.backend.fence(ctx); err != nil {
		return err
	}
	if err = n.check(ctx); err != nil {
		return err
	}
	state, err = n.backend.status()
	if err != nil || !serviceCompleted(state) {
		return errors.Join(errors.New("demand successor changed before configuration"), err)
	}
	if err = n.backend.automatic(ctx); err != nil {
		return err
	}
	if err = n.check(ctx); err != nil {
		return err
	}
	state, err = n.backend.status()
	if err != nil || !serviceCompleted(state) {
		return errors.Join(errors.New("automatic successor changed before handoff"), err)
	}
	if err = n.release(); err != nil {
		return err
	}
	// A separate invocation may reconcile an uncertain config effect by observing
	// exact AUTO + this canonical attempt + the complete protected pair. It never
	// repeats ChangeServiceConfig. No completion journal is needed for that fact.
	n.next, err = n.openAutomatic(ctx)
	if err != nil || n.next == nil {
		return errors.Join(errors.New("fresh automatic resume owner required"), err)
	}
	if err = n.next.check(ctx); err != nil {
		return err
	}
	return n.next.resume(ctx)
}

func (n *codeUpgradeDemandTransition) release() error {
	if n.released {
		return n.releaseErr
	}
	n.released = true
	if n.backend != nil {
		n.releaseErr = n.backend.close()
	}
	if n.publication != nil {
		n.releaseErr = errors.Join(n.releaseErr, n.publication.close())
	}
	if n.releaseErr != nil {
		n.releaseErr = errors.Join(ErrCleanup, n.releaseErr)
	}
	return n.releaseErr
}

func (n *codeUpgradeDemandTransition) close() error {
	if n == nil {
		return nil
	}
	if n.self != n {
		return errors.New("demand transition owner cannot be copied")
	}
	if n.closed {
		return n.closeErr
	}
	n.closed = true
	n.closeErr = n.release()
	if n.next != nil {
		n.closeErr = errors.Join(n.closeErr, n.next.close())
	}
	return n.closeErr
}
