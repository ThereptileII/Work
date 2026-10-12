package brokerhost

import (
	"context"
	"encoding/json"
	"errors"
	"time"

	"example.com/opennav-update-verifier/brokerpipe"
	"example.com/opennav-update-verifier/brokerstore"
)

var errExecutionHostRefused = errors.New("protected execution refused")
var errExecutionHostUncertain = errors.New("protected execution requires recovery; no automatic retry")

// Admission may establish the independently approved baseline only. It cannot
// become an update when another transaction appears between inspection and the
// qualifier acquiring its operation lock. Existing selections require a fresh
// explicit update intent or an ordinary selected-start conversation.
func executionQualificationIntentAllowed(operation string, initial bool) bool {
	return operation == "execute-session" || (operation == "start-selected" && initial)
}

// These private interfaces are implemented by retained native owners. They are
// fault seams, not exported receipt constructors or production caller callbacks.
// A profile provider must retain the complete original backup, independently
// enforce coldness and verify actual restoration; a pipe report cannot do so.
type executionHostProfile interface {
	Validate(context.Context) error
	ReleaseForQualification(context.Context) error
	RestoreOriginal(context.Context) (string, error)
	Evidence(context.Context) (string, error)
	Close() error
}
type executionHostChild interface {
	Environment(context.Context) (map[string]string, error)
	BindSpawn(context.Context, brokerpipe.ExecutionObservation) error
	Observe(context.Context) (string, error)
	ValidateHealthy(context.Context) error
	WaitClosed(context.Context) (string, error)
	Close() error
}
type executionHostJournal interface {
	AppendFact(context.Context, brokerstore.ExecutionFactKind, []byte) error
	Close() error
}
type executionHostConversation interface {
	ExchangeCommand(context.Context, brokerpipe.ExecutionDirective) (brokerpipe.ExecutionObservation, error)
}

// A run has exactly one fresh journal and native-derived generation identities.
// Native assembly below supplies real store/image/peer/profile/child capabilities;
// no exported constructor can assemble a run from serialized declarations.
type executionHostRun struct {
	begin            executionSelectionBegin
	journal          executionHostJournal
	conversation     executionHostConversation
	profile          executionHostProfile
	capture          func(context.Context) (executionHostProfile, error)
	validate         func(context.Context) error
	validateCleanup  func(context.Context) error
	validateRecovery func(context.Context) error
	commission       func(context.Context, bool) (string, error)
	prepare          func(context.Context, bool) (executionHostChild, error)
	resolve          func(context.Context, string) error
	prepareRestore   func(context.Context, executionHostChild) error
	recovery         bool
	attempted        bool
	consumed         [2]bool
}

// Recovery is entered only after original candidate exit, native restoration
// authority and successful handle cleanup. It cannot authorize candidate selection.
func (r *executionHostRun) validateEffect(ctx context.Context) error {
	if r.recovery {
		return r.validateRecovery(ctx)
	}
	return r.validate(ctx)
}

// A consumed birth gets one bounded close attempt, including on early returns.
// Cancellation and retained custody errors remain effective: cleanup never invents
// a fresh peer, detaches the operation context, retries or trusts a client report.
type executionHostDrain struct {
	run                 *executionHostRun
	child               executionHostChild
	fallback, attempted bool
}

func (d *executionHostDrain) drain(ctx context.Context) (string, error) {
	d.attempted = true
	return d.run.drain(ctx, d.child, d.fallback)
}
func (d *executionHostDrain) cleanup(ctx context.Context) error {
	index := 0
	if d.fallback {
		index = 1
	}
	if d.child == nil || d.attempted || !d.run.consumed[index] {
		return nil
	}
	_, err := d.drain(ctx)
	return err
}

func (r *executionHostRun) append(ctx context.Context, kind brokerstore.ExecutionFactKind, step, generation, digest string) error {
	if !hashPattern.MatchString(digest) {
		return errExecutionHostRefused
	}
	if err := r.validateEffect(ctx); err != nil {
		return err
	}
	raw, err := json.Marshal(executionSelectionStep{1, "broker-selected-step-v1", r.begin.ExecutionID, step, generation, digest})
	if err != nil {
		return err
	}
	return r.journal.AppendFact(ctx, kind, raw)
}
func (r *executionHostRun) exchange(ctx context.Context, phase brokerpipe.ExecutionPhase, generation string, environment map[string]string) (brokerpipe.ExecutionObservation, error) {
	validator := r.validateEffect
	if phase == brokerpipe.CloseCandidate || phase == brokerpipe.CloseFallback {
		validator = r.validateCleanup
	}
	if err := validator(ctx); err != nil {
		return brokerpipe.ExecutionObservation{}, err
	}
	d := brokerpipe.ExecutionDirective{Phase: phase, ApplicationGeneration: generation}
	if environment != nil {
		if len(environment) != 4 || environment["SKAGER_UPDATE_GENERATION"] != generation || environment["SKAGER_UPDATE_QUALIFY_PROFILE"] != "1" {
			return brokerpipe.ExecutionObservation{}, errExecutionHostRefused
		}
		d.HealthPipe = environment["SKAGER_UPDATE_PIPE"]
		d.HealthChallenge = environment["SKAGER_UPDATE_CHALLENGE"]
	}
	// A spawn report is only input to native BindSpawn corroboration. It cannot
	// establish health, closure, restoration or selection.
	return r.conversation.ExchangeCommand(ctx, d)
}
func (r *executionHostRun) directive(ctx context.Context, phase brokerpipe.ExecutionPhase, generation string, environment map[string]string) error {
	_, err := r.exchange(ctx, phase, generation, environment)
	return err
}
func (r *executionHostRun) qualify(ctx context.Context, fallback bool) (child executionHostChild, digest string, resultErr error) {
	generation := r.begin.Generation
	commissionStep, spawnStep := "commissioned", "spawn-consumed"
	spawn := brokerpipe.SpawnCandidate
	if fallback {
		generation = r.begin.PreviousGeneration
		commissionStep = "fallback-commissioned"
		spawnStep = "fallback-spawn-consumed"
		spawn = brokerpipe.SpawnFallback
	}
	if err := r.profile.Validate(ctx); err != nil {
		return nil, "", err
	}
	commissioned, err := r.commission(ctx, fallback)
	if err != nil {
		return nil, "", err
	}
	if err = r.append(ctx, brokerstore.ExecutionCommissioning, commissionStep, generation, commissioned); err != nil {
		return nil, "", err
	}
	child, err = r.prepare(ctx, fallback)
	if err == nil && child == nil {
		err = errExecutionHostRefused
	}
	if err != nil {
		return child, "", err
	}
	environment, err := child.Environment(ctx)
	if err != nil {
		return child, "", err
	}
	// Durable one-use intent precedes the command and every native creator. An
	// ambiguous append or command is never retried, even if a client reports failure.
	if err = r.append(ctx, brokerstore.ExecutionSpawnIntent, spawnStep, generation, commissioned); err != nil {
		return child, "", err
	}
	index := 0
	if fallback {
		index = 1
	}
	r.consumed[index] = true
	report, err := r.exchange(ctx, spawn, generation, environment)
	if err != nil {
		return child, "", err
	}
	if err = child.BindSpawn(ctx, report); err != nil {
		return child, "", err
	}
	// Independent native pipe/CONFIG/READY/EOF observation is the health check.
	// A redundant launcher Observe command could poison the creator conversation
	// before the host can close its actual child after an observation failure.
	digest, err = child.Observe(ctx)
	if err != nil {
		return child, "", err
	}
	if err = child.ValidateHealthy(ctx); err != nil {
		return child, "", err
	}
	return child, digest, nil
}
func (r *executionHostRun) drain(ctx context.Context, child executionHostChild, fallback bool) (string, error) {
	if child == nil {
		return "", errExecutionHostUncertain
	}
	ctx, cancel := context.WithTimeout(ctx, 45*time.Second)
	defer cancel()
	phase, generation := brokerpipe.CloseCandidate, r.begin.Generation
	if fallback {
		phase = brokerpipe.CloseFallback
		generation = r.begin.PreviousGeneration
	}
	if err := r.directive(ctx, phase, generation, nil); err != nil {
		return "", err
	}
	return child.WaitClosed(ctx)
}

// execute authors reducer-compatible facts only after their native effects. On
// success qualification is drained and the protected selection is reopened: the
// application requires a fresh ordinary authenticated startup. Qualification's
// process-lifetime output/plugin restriction is never cleared in place.
func (r *executionHostRun) execute(ctx context.Context) (resultErr error) {
	if ctx == nil || r == nil || r.journal == nil || r.conversation == nil || (r.profile == nil && r.capture == nil) || r.validate == nil || r.validateCleanup == nil || r.validateRecovery == nil || r.prepareRestore == nil || r.commission == nil || r.prepare == nil || r.resolve == nil {
		return errExecutionHostRefused
	}
	if r.attempted {
		return errExecutionHostRefused
	}
	r.attempted = true
	defer func() {
		if r.profile != nil {
			resultErr = errors.Join(resultErr, r.profile.Close())
		}
		resultErr = errors.Join(resultErr, r.journal.Close())
		if resultErr != nil {
			resultErr = errors.Join(errExecutionHostUncertain, resultErr)
		}
	}()
	if err := r.validate(ctx); err != nil {
		return err
	}
	original, err := json.Marshal(r.begin)
	if err != nil {
		return err
	}
	if err = r.journal.AppendFact(ctx, brokerstore.ExecutionPreparation, original); err != nil {
		return err
	}
	if r.profile == nil {
		r.profile, err = r.capture(ctx)
		if err != nil || r.profile == nil {
			return errors.Join(errExecutionHostRefused, err)
		}
	}
	if err = r.profile.Validate(ctx); err != nil {
		return err
	}
	evidence, err := r.profile.Evidence(ctx)
	if err != nil || !hashPattern.MatchString(evidence) {
		return errors.Join(errExecutionHostRefused, err)
	}
	if err = r.profile.ReleaseForQualification(ctx); err != nil {
		return err
	}
	child, health, qualifyErr := r.qualify(ctx, false)
	if child != nil {
		defer func() { resultErr = errors.Join(resultErr, child.Close()) }()
	}
	candidateDrain := &executionHostDrain{run: r, child: child}
	defer func() { resultErr = errors.Join(resultErr, candidateDrain.cleanup(ctx)) }()
	if qualifyErr == nil {
		qualifyErr = r.append(ctx, brokerstore.ExecutionHealthObserved, "healthy", r.begin.Generation, health)
	}
	var closed string
	restorePrepared, candidateReleased := false, false
	if qualifyErr == nil {
		closed, err = candidateDrain.drain(ctx)
		if err != nil {
			return err
		}
		if r.begin.PreviousGeneration != "" {
			// Retain original exit authority before releasing observer handles. A
			// later policy failure can still recover without reopening any process.
			if err = r.prepareRestore(ctx, child); err != nil {
				return err
			}
			restorePrepared = true
		}
		if err = child.Close(); err != nil {
			return err
		}
		candidateReleased = true
		qualifyErr = r.append(ctx, brokerstore.ExecutionHealthObserved, "selected", r.begin.Generation, closed)
		if qualifyErr == nil {
			if err = r.resolve(ctx, "selected"); err != nil {
				return err
			}
		}
	}
	if qualifyErr != nil {
		// Cancelled operations, unknown children and initial baseline failures stay
		// pending. Only retained native exit/restoration authority enables fallback.
		if ctx.Err() != nil || r.begin.PreviousGeneration == "" || !r.consumed[0] {
			return qualifyErr
		}
		if !candidateDrain.attempted {
			closed, err = candidateDrain.drain(ctx)
			if err != nil {
				return errors.Join(qualifyErr, err)
			}
		}
		if !restorePrepared {
			if err = r.prepareRestore(ctx, child); err != nil {
				return errors.Join(qualifyErr, err)
			}
		}
		if !candidateReleased {
			if err = child.Close(); err != nil {
				return err
			}
		}
		// Only the independently observed original exit plus native restoration
		// authority permits offline recovery of the retained selected predecessor.
		r.recovery = true
		if err = r.append(ctx, brokerstore.ExecutionTeardown, "candidate-closed", r.begin.Generation, closed); err != nil {
			return err
		}
		restored, err := r.profile.RestoreOriginal(ctx)
		if err != nil {
			return errors.Join(qualifyErr, err)
		}
		if err = r.append(ctx, brokerstore.ExecutionRestoration, "profile-restored", r.begin.Generation, restored); err != nil {
			return err
		}
		fallback, health, err := r.qualify(ctx, true)
		if fallback != nil {
			defer func() { resultErr = errors.Join(resultErr, fallback.Close()) }()
		}
		fallbackDrain := &executionHostDrain{run: r, child: fallback, fallback: true}
		defer func() { resultErr = errors.Join(resultErr, fallbackDrain.cleanup(ctx)) }()
		if err != nil {
			return errors.Join(qualifyErr, err)
		}
		if err = r.append(ctx, brokerstore.ExecutionHealthObserved, "fallback-healthy", r.begin.PreviousGeneration, health); err != nil {
			return err
		}
		closed, err = fallbackDrain.drain(ctx)
		if err != nil {
			return err
		}
		if err = fallback.Close(); err != nil {
			return err
		}
		if err = r.append(ctx, brokerstore.ExecutionRestoration, "rolled-back", r.begin.PreviousGeneration, closed); err != nil {
			return err
		}
		if err = r.resolve(ctx, "rolled-back"); err != nil {
			return err
		}
	}
	// Transport finish is not transaction acceptance. A late disconnect/cleanup
	// failure reports uncertainty even if the completed history can later be read.
	_, err = r.conversation.ExchangeCommand(ctx, brokerpipe.ExecutionDirective{Phase: brokerpipe.FinishExecution})
	return err
}
