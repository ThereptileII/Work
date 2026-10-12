package brokerhost

import (
	"context"
	"errors"

	"example.com/opennav-update-verifier/brokerpipe"
)

var errExecutionCurrentRefused = errors.New("protected selected ordinary startup refused")
var errExecutionCurrentUncertain = errors.New("ordinary startup outcome uncertain; no retry or qualification fallback")

type executionCurrentHostChild interface {
	Validate(context.Context, brokerpipe.ExecutionObservation) error
	Close() error // Release retained handles only; never signal the ordinary app.
}

// Private fault seams. Production assembles this exclusively from native selected
// history, image-bound fresh session and retained actual child process/token.
type executionCurrentHostRun struct {
	generation          string
	conversation        executionHostConversation
	validate            func(context.Context) error
	beforeSpawn         func(context.Context) error
	corroborate         func(context.Context, brokerpipe.ExecutionObservation) (executionCurrentHostChild, error)
	attempted, consumed bool
}

func (r *executionCurrentHostRun) execute(ctx context.Context) (resultErr error) {
	if r == nil || ctx == nil || !requestPattern.MatchString(r.generation) || r.conversation == nil || r.validate == nil || r.beforeSpawn == nil || r.corroborate == nil || r.attempted {
		return errExecutionCurrentRefused
	}
	r.attempted = true
	defer func() {
		if resultErr != nil {
			if r.consumed {
				resultErr = errors.Join(errExecutionCurrentUncertain, resultErr)
			} else {
				resultErr = errors.Join(errExecutionCurrentRefused, resultErr)
			}
		}
	}()
	if err := r.validate(ctx); err != nil {
		return err
	}
	// Consume the local grant before entering the real creator command. The live
	// launcher independently consumes its current role before CREATE_SUSPENDED.
	// No qualification session can enter here: native binding requires phase zero.
	r.consumed = true
	if err := r.beforeSpawn(ctx); err != nil {
		return err
	}
	spawn := brokerpipe.ExecutionDirective{Phase: brokerpipe.SpawnCurrent, ApplicationGeneration: r.generation}
	report, err := r.conversation.ExchangeCommand(ctx, spawn)
	if err != nil {
		return err
	}
	child, err := r.corroborate(ctx, report)
	if child != nil {
		defer func() { resultErr = errors.Join(resultErr, child.Close()) }()
	}
	if err != nil || child == nil {
		return errors.Join(errExecutionCurrentRefused, err)
	}
	if err = r.validate(ctx); err != nil {
		return err
	}
	observe := brokerpipe.ExecutionDirective{Phase: brokerpipe.ObserveCurrent, ApplicationGeneration: r.generation}
	report, err = r.conversation.ExchangeCommand(ctx, observe)
	if err != nil {
		return err
	}
	if err = child.Validate(ctx, report); err != nil {
		return err
	}
	if err = r.validate(ctx); err != nil {
		return err
	}
	// There is no qualification environment, READY health claim, profile mutation,
	// selected fact or updater commit. Normal plugin/navigation/output semantics
	// belong to this new ordinary application; it remains alive after handle drain.
	_, err = r.conversation.ExchangeCommand(ctx, brokerpipe.ExecutionDirective{Phase: brokerpipe.FinishExecution})
	return err
}
