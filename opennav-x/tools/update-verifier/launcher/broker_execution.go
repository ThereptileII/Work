package launcher

import (
	"context"
	"crypto/rand"
	"encoding/hex"
	"errors"
	"strings"
	"sync"

	"example.com/opennav-update-verifier/brokerpipe"
	"example.com/opennav-update-verifier/brokerregistration"
)

var errBrokerExecutionRefused = errors.New("protected broker execution refused")
var errBrokerExecutionUncertain = errors.New("protected broker execution outcome uncertain; no retry or ordinary fallback")
var errBrokerExecutionNative = errors.New("protected broker execution requires native Windows")

// Only CallExecution invokes this private action class in production, after its
// actual SCM broker/pipe authentication. Reports remain untrusted corroboration,
// not commissioning, consumed-spawn, selected-chain or healthy-start authority.
type brokerExecutionBackend interface {
	spawn(context.Context, brokerpipe.ExecutionCommand) (brokerExecutionChild, error)
}
type brokerExecutionChild interface {
	observe(context.Context) (brokerExecutionChildInfo, error)
	closeApplication(context.Context) (brokerExecutionChildInfo, error)
	close() error // Releases handles only; never signals or terminates a resumed app.
}
type brokerExecutionChildInfo struct {
	pid       uint32
	created   uint64
	imageHash string
}

func (i brokerExecutionChildInfo) report() (brokerpipe.ExecutionObservation, error) {
	if i.pid == 0 || i.created == 0 || !hashPattern.MatchString(i.imageHash) {
		return brokerpipe.ExecutionObservation{}, errBrokerExecutionRefused
	}
	return brokerpipe.ExecutionObservation{Status: "reported-complete", EvidenceSHA256: i.imageHash, ProcessID: i.pid, ProcessCreation: i.created}, nil
}

type brokerExecutionRole struct {
	consumed, observed, closeAttempted bool
	command                            brokerpipe.ExecutionCommand
	child                              brokerExecutionChild
}
type brokerExecutionActions struct {
	mu                  sync.Mutex
	request             brokerpipe.ExecutionRequest
	backend             brokerExecutionBackend
	candidate, fallback brokerExecutionRole
	finished, closed    bool
	failure, closeErr   error
}

func brokerExecutionArgument(args []string) (string, bool, error) {
	matched := false
	for _, arg := range args {
		if strings.HasPrefix(arg, "--broker-execute") {
			matched = true
		}
	}
	if !matched {
		return "", false, nil
	}
	if len(args) != 1 || !strings.HasPrefix(args[0], "--broker-execute=") {
		return "", true, errBrokerExecutionRefused
	}
	id := strings.TrimPrefix(args[0], "--broker-execute=")
	if !generationPattern.MatchString(id) {
		return "", true, errBrokerExecutionRefused
	}
	return id, true, nil
}
func brokerExecutionApplication(programFiles, sid, generation string) (string, error) {
	if !brokerregistration.IsCanonicalUserSID(sid) || !generationPattern.MatchString(generation) {
		return "", errBrokerExecutionRefused
	}
	root, err := localPath(programFiles)
	if err != nil {
		return "", err
	}
	return localPath(strings.TrimRight(root, "/") + "/SKAGER/Applications/" + sid + "/generations/" + generation + "/app/opencpn.exe")
}
func brokerExecutionPayload(command brokerpipe.ExecutionCommand, stage string) error {
	if command.StageID != stage || !generationPattern.MatchString(stage) {
		return errBrokerExecutionRefused
	}
	switch command.Phase {
	case brokerpipe.SpawnCandidate, brokerpipe.ObserveCandidate, brokerpipe.SpawnFallback, brokerpipe.ObserveFallback:
		if !generationPattern.MatchString(command.ApplicationGeneration) || !strings.HasPrefix(command.HealthPipe, "Skager.Update.") || !generationPattern.MatchString(strings.TrimPrefix(command.HealthPipe, "Skager.Update.")) || !hashPattern.MatchString(command.HealthChallenge) {
			return errBrokerExecutionRefused
		}
	case brokerpipe.CloseCandidate, brokerpipe.CloseFallback:
		if command.HealthPipe != "" || command.HealthChallenge != "" || (command.ApplicationGeneration != "" && !generationPattern.MatchString(command.ApplicationGeneration)) {
			return errBrokerExecutionRefused
		}
	case brokerpipe.FinishExecution:
		if command.ApplicationGeneration != "" || command.HealthPipe != "" || command.HealthChallenge != "" {
			return errBrokerExecutionRefused
		}
	default:
		return errBrokerExecutionRefused // Actual inspect/commission/restore is not implemented here.
	}
	switch command.Phase {
	case brokerpipe.SpawnCandidate, brokerpipe.ObserveCandidate, brokerpipe.CloseCandidate:
		if command.ApplicationGeneration != "" && command.ApplicationGeneration != stage {
			return errBrokerExecutionRefused
		}
	case brokerpipe.SpawnFallback, brokerpipe.ObserveFallback, brokerpipe.CloseFallback:
		if command.ApplicationGeneration == stage {
			return errBrokerExecutionRefused
		}
	}
	return nil
}
func runBrokerExecution(ctx context.Context, stage string, backend brokerExecutionBackend, call func(context.Context, brokerpipe.ExecutionRequest, brokerpipe.ExecutionClientHandler) error) (resultErr error) {
	if ctx == nil || ctx.Err() != nil || !generationPattern.MatchString(stage) || backend == nil || call == nil {
		return errors.Join(errBrokerExecutionRefused, contextErrorBroker(ctx))
	}
	var nonce [16]byte
	if _, err := rand.Read(nonce[:]); err != nil {
		return err
	}
	actions := &brokerExecutionActions{request: brokerpipe.ExecutionRequest{Schema: 1, Operation: "execute-session", RequestID: hex.EncodeToString(nonce[:]), StageID: stage}, backend: backend}
	defer func() { resultErr = errors.Join(resultErr, actions.close()) }()
	// An authenticated transport finish is not transaction acceptance. Failure or
	// lost conversation never enters legacy startup or retries any consumed role.
	return call(ctx, actions.request, actions.handle)
}
func contextErrorBroker(ctx context.Context) error {
	if ctx == nil {
		return errBrokerExecutionRefused
	}
	return ctx.Err()
}
func (a *brokerExecutionActions) handle(ctx context.Context, c brokerpipe.ExecutionCommand) (result brokerpipe.ExecutionObservation, resultErr error) {
	a.mu.Lock()
	defer a.mu.Unlock()
	if ctx == nil || ctx.Err() != nil || a.closed || a.finished || a.failure != nil {
		return result, errors.Join(errBrokerExecutionRefused, a.failure, contextErrorBroker(ctx))
	}
	defer func() {
		if resultErr != nil {
			a.failure = errors.Join(a.failure, resultErr)
		}
	}()
	if c.Schema != 1 || c.Kind != "command" || c.RequestID != a.request.RequestID || brokerExecutionPayload(c, a.request.StageID) != nil {
		return result, errBrokerExecutionRefused
	}
	if c.Phase == brokerpipe.FinishExecution {
		a.finished = true
		if err := a.closeLocked(); err != nil {
			return result, err
		}
		return brokerpipe.ExecutionObservation{Status: "reported-complete"}, nil
	}
	role := &a.candidate
	switch c.Phase {
	case brokerpipe.SpawnFallback, brokerpipe.ObserveFallback, brokerpipe.CloseFallback:
		role = &a.fallback
	}
	switch c.Phase {
	case brokerpipe.SpawnCandidate, brokerpipe.SpawnFallback:
		if role.consumed || a.candidate.child != nil || a.fallback.child != nil {
			return result, errBrokerExecutionRefused
		}
		// The local attempt is consumed before acquiring an image or entering the
		// native creator. It is additional replay defense, not durable broker consent.
		role.consumed = true
		role.command = c
		child, err := a.backend.spawn(ctx, c)
		role.child = child
		if err != nil || child == nil {
			return result, errors.Join(errBrokerExecutionUncertain, err)
		}
		info, err := child.observe(ctx)
		if err != nil {
			return result, errors.Join(errBrokerExecutionUncertain, err)
		}
		return info.report()
	case brokerpipe.ObserveCandidate, brokerpipe.ObserveFallback:
		if role.child == nil || role.observed || role.closeAttempted || c.ApplicationGeneration != role.command.ApplicationGeneration || c.HealthPipe != role.command.HealthPipe || c.HealthChallenge != role.command.HealthChallenge {
			return result, errBrokerExecutionRefused
		}
		role.observed = true
		info, err := role.child.observe(ctx)
		if err != nil {
			return result, errors.Join(errBrokerExecutionUncertain, err)
		}
		return info.report()
	case brokerpipe.CloseCandidate, brokerpipe.CloseFallback:
		if role.child == nil || role.closeAttempted || (c.ApplicationGeneration != "" && c.ApplicationGeneration != role.command.ApplicationGeneration) {
			return result, errBrokerExecutionRefused
		}
		role.closeAttempted = true
		info, err := role.child.closeApplication(ctx)
		if err != nil {
			return result, errors.Join(errBrokerExecutionUncertain, err)
		}
		if err = role.child.close(); err != nil {
			return result, errors.Join(errBrokerExecutionUncertain, err)
		}
		role.child = nil
		return info.report()
	}
	return result, errBrokerExecutionRefused
}
func (a *brokerExecutionActions) closeLocked() error {
	if a.closed {
		return a.closeErr
	}
	a.closed = true
	for _, role := range []*brokerExecutionRole{&a.candidate, &a.fallback} {
		if role.child != nil {
			a.closeErr = errors.Join(a.closeErr, role.child.close())
			role.child = nil
		}
	}
	if a.closeErr != nil {
		a.closeErr = errors.Join(errBrokerExecutionUncertain, a.closeErr)
	}
	return a.closeErr
}
func (a *brokerExecutionActions) close() error {
	a.mu.Lock()
	defer a.mu.Unlock()
	return a.closeLocked()
}
