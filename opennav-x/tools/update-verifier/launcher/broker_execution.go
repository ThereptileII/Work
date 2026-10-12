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
	mu                                  sync.Mutex
	request                             brokerpipe.ExecutionRequest
	backend                             brokerExecutionBackend
	candidate, fallback, current, relay brokerExecutionRole
	finished, closed                    bool
	failure, closeErr                   error
}

func brokerStartupArgument(args []string) (string, bool, error) {
	return brokerIntentArgument(args, "--broker-start")
}
func brokerExecutionArgument(args []string) (string, bool, error) {
	return brokerIntentArgument(args, "--broker-execute")
}
func brokerCurrentArgument(args []string) (string, bool, error) {
	return brokerIntentArgument(args, "--broker-current")
}
func brokerIntentArgument(args []string, prefix string) (string, bool, error) {
	matched := false
	for _, arg := range args {
		if strings.HasPrefix(arg, prefix) {
			matched = true
		}
	}
	if !matched {
		return "", false, nil
	}
	if len(args) != 1 || !strings.HasPrefix(args[0], prefix+"=") {
		return "", true, errBrokerExecutionRefused
	}
	id := strings.TrimPrefix(args[0], prefix+"=")
	if prefix == "--broker-start" || prefix == "--broker-current" {
		if _, _, err := brokerStartupMode(id); err != nil {
			return "", true, err
		}
	} else if !generationPattern.MatchString(id) {
		return "", true, errBrokerExecutionRefused
	}
	return id, true, nil
}

// A sole canonical argument carries closed mode data, never extra process args.
func brokerStartupMode(value string) (string, brokerpipe.StartupMode, error) {
	generation, suffix, hasMode := strings.Cut(value, ":")
	mode := brokerpipe.StartupMode(suffix)
	if !generationPattern.MatchString(generation) || !mode.Valid() || (hasMode && mode == brokerpipe.StartupXNav) {
		return "", "", errBrokerExecutionRefused
	}
	return generation, mode, nil
}
func brokerModeIntent(generation string, mode brokerpipe.StartupMode) (string, error) {
	if !generationPattern.MatchString(generation) || !mode.Valid() {
		return "", errBrokerExecutionRefused
	}
	if mode != brokerpipe.StartupXNav {
		generation += ":" + string(mode)
	}
	return generation, nil
}
func brokerNormalPhase(p brokerpipe.ExecutionPhase) bool {
	return p == brokerpipe.RelaySelectedExecutor || p == brokerpipe.RelayStartupExecutor || p == brokerpipe.SpawnCurrent || p == brokerpipe.ObserveCurrent || p == brokerpipe.CloseCurrent
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
func brokerExecutionLauncher(programFiles, sid, generation string) (string, error) {
	application, err := brokerExecutionApplication(programFiles, sid, generation)
	if err != nil {
		return "", err
	}
	return strings.TrimSuffix(application, "opencpn.exe") + "skager-start.exe", nil
}
func brokerExecutionPayload(command brokerpipe.ExecutionCommand, stage string) error {
	if !command.StartupMode.Valid() || (!brokerNormalPhase(command.Phase) && command.StartupMode != brokerpipe.StartupXNav) || (command.Phase == brokerpipe.RelayStartupExecutor && command.StartupMode != brokerpipe.StartupXNav) || command.StageID != stage || !generationPattern.MatchString(stage) || command.OfferVersion != "" || command.OfferCommit != "" || command.OfferPolicySHA256 != "" || command.PromptBytes != 0 || command.PromptSHA256 != "" {
		return errBrokerExecutionRefused
	}
	switch command.Phase {
	case brokerpipe.RelaySelectedExecutor, brokerpipe.RelayStartupExecutor, brokerpipe.SpawnCurrent, brokerpipe.ObserveCurrent, brokerpipe.CloseCurrent:
		if !generationPattern.MatchString(command.ApplicationGeneration) || command.HealthPipe != "" || command.HealthChallenge != "" {
			return errBrokerExecutionRefused
		}
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
	return runBrokerExecutionIntent(ctx, stage, "execute-session", backend, call)
}
func runBrokerExecutionIntent(ctx context.Context, stage, operation string, backend brokerExecutionBackend, call func(context.Context, brokerpipe.ExecutionRequest, brokerpipe.ExecutionClientHandler) error) error {
	return runBrokerExecutionModeIntent(ctx, stage, operation, brokerpipe.StartupXNav, backend, call)
}
func runBrokerExecutionModeIntent(ctx context.Context, stage, operation string, mode brokerpipe.StartupMode, backend brokerExecutionBackend, call func(context.Context, brokerpipe.ExecutionRequest, brokerpipe.ExecutionClientHandler) error) (resultErr error) {
	if !mode.Valid() || (operation == "execute-session" && mode != brokerpipe.StartupXNav) || ctx == nil || ctx.Err() != nil || !generationPattern.MatchString(stage) || backend == nil || call == nil || (operation != "execute-session" && operation != "start-current" && operation != "start-selected") {
		return errors.Join(errBrokerExecutionRefused, contextErrorBroker(ctx))
	}
	var nonce [16]byte
	if _, err := rand.Read(nonce[:]); err != nil {
		return err
	}
	actions := &brokerExecutionActions{request: brokerpipe.ExecutionRequest{Schema: 1, Operation: operation, RequestID: hex.EncodeToString(nonce[:]), StageID: stage, StartupMode: mode}, backend: backend}
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
	if (brokerNormalPhase(c.Phase) && c.StartupMode != a.request.StartupMode) || c.Schema != 1 || c.Kind != "command" || c.RequestID != a.request.RequestID || brokerExecutionPayload(c, a.request.StageID) != nil {
		return result, errBrokerExecutionRefused
	}
	if a.relay.consumed && c.Phase != brokerpipe.FinishExecution {
		return result, errBrokerExecutionRefused
	}
	if c.Phase == brokerpipe.RelayStartupExecutor && (a.request.Operation != "start-selected" || a.candidate.consumed || a.fallback.consumed || a.current.consumed) {
		return result, errBrokerExecutionRefused
	}
	// Admission can qualify or terminally relay, but cannot create an ordinary
	// current application inside the same qualification/admission conversation.
	current := c.Phase == brokerpipe.SpawnCurrent || c.Phase == brokerpipe.ObserveCurrent || c.Phase == brokerpipe.CloseCurrent
	if c.Phase != brokerpipe.FinishExecution && c.Phase != brokerpipe.RelaySelectedExecutor && c.Phase != brokerpipe.RelayStartupExecutor && ((a.request.Operation == "start-current") != current) {
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
	case brokerpipe.RelaySelectedExecutor, brokerpipe.RelayStartupExecutor:
		role = &a.relay
	case brokerpipe.SpawnCurrent, brokerpipe.ObserveCurrent, brokerpipe.CloseCurrent:
		role = &a.current
	case brokerpipe.SpawnFallback, brokerpipe.ObserveFallback, brokerpipe.CloseFallback:
		role = &a.fallback
	}
	switch c.Phase {
	case brokerpipe.RelaySelectedExecutor, brokerpipe.RelayStartupExecutor, brokerpipe.SpawnCurrent, brokerpipe.SpawnCandidate, brokerpipe.SpawnFallback:
		// Normal and qualification are distinct conversations. Even a completed
		// close cannot convert a qualification child into an ordinary session, or
		// clear the application's already-latched qualification output fence.
		if role == &a.relay && a.current.consumed {
			return result, errBrokerExecutionRefused
		}
		if role != &a.relay && ((role == &a.current && (a.candidate.consumed || a.fallback.consumed)) || (role != &a.current && a.current.consumed)) {
			return result, errBrokerExecutionRefused
		}
		if role.consumed || a.current.child != nil || a.candidate.child != nil || a.fallback.child != nil {
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
	case brokerpipe.ObserveCurrent, brokerpipe.ObserveCandidate, brokerpipe.ObserveFallback:
		if role.child == nil || role.observed || role.closeAttempted || c.ApplicationGeneration != role.command.ApplicationGeneration || c.HealthPipe != role.command.HealthPipe || c.HealthChallenge != role.command.HealthChallenge || c.StartupMode != role.command.StartupMode {
			return result, errBrokerExecutionRefused
		}
		role.observed = true
		info, err := role.child.observe(ctx)
		if err != nil {
			return result, errors.Join(errBrokerExecutionUncertain, err)
		}
		return info.report()
	case brokerpipe.CloseCurrent, brokerpipe.CloseCandidate, brokerpipe.CloseFallback:
		if role.child == nil || role.closeAttempted || (c.ApplicationGeneration != "" && c.ApplicationGeneration != role.command.ApplicationGeneration) || c.StartupMode != role.command.StartupMode {
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
	for _, role := range []*brokerExecutionRole{&a.current, &a.candidate, &a.fallback, &a.relay} {
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
