package brokerhost

import (
	"context"
	"errors"
	"runtime"

	"example.com/opennav-update-verifier/brokerpipe"
	"example.com/opennav-update-verifier/brokerstore"
	"golang.org/x/sys/windows"
)

// Current startup is local selected-version use, including Later/offline. This
// REQUIRED native gate enforces armed/profile/no-other-writer policy without
// treating failure to acquire online update metadata as selection authority or a
// reason to run an unqualified baseline. No client choice/report constructs it.
type executionCurrentHostProviders struct {
	authorize func(context.Context, *nativeSelectedExecution) error
	// A one-use native prelaunch gate retains/rechecks actual cold/no-other-writer
	// custody through the creator boundary. It is not an absence JSON or PID list.
	prepare func(context.Context, *nativeSelectedExecution) error
}
type nativeExecutionCurrentHost struct {
	peer            *brokerpipe.PeerCapability
	selected        *nativeSelectedExecution
	image           *brokerstore.GenerationApplicationImage
	identity        brokerstore.GenerationApplicationIdentity
	providers       executionCurrentHostProviders
	bornAfter       uint64
	spawnAuthorized bool
}

// handleNativeCurrentExecution serves ONLY a new native authenticated session.
// The request's StageID is inert routing data, not selection. Actual selected
// journals determine both launcher image and application generation. A baseline
// without completed qualification, pending/forked history, old predecessor image
// after update, or an already used qualification session all refuse before spawn.
// Root wires explicit startup routing; never infer a current request by retrying
// a failed update handler or catching its selection/authorization errors.
func (e *engine) handleNativeCurrentExecution(ctx context.Context, peer brokerpipe.Peer, q brokerpipe.ExecutionRequest, session *brokerpipe.ExecutionSession, providers executionCurrentHostProviders) (resultErr error) {
	if e == nil || ctx == nil || ctx.Err() != nil || session == nil || providers.authorize == nil || providers.prepare == nil || e.observe == nil || q.Schema != 1 || q.Operation != "execute-session" || !requestPattern.MatchString(q.RequestID) || !requestPattern.MatchString(q.StageID) {
		return errExecutionCurrentRefused
	}
	adapter, ok := e.store.(storeAdapter)
	if !ok || adapter.Store == nil {
		return errExecutionCurrentRefused
	}
	capability, err := brokerpipe.AuthorizePeer(ctx, peer)
	if err != nil {
		return errExecutionCurrentRefused
	}
	if !e.busy.TryLock() {
		return errExecutionCurrentRefused
	}
	defer e.busy.Unlock()
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if err = adapter.Validate(); err != nil {
		return err
	}
	raw, err := adapter.ReadRegistration(peer.SID)
	if err != nil {
		return err
	}
	registration, err := parseRegistration(raw, peer.SID)
	if err != nil {
		return err
	}
	selected, err := captureSelectedExecutionNative(ctx, adapter.Store, registration)
	if err != nil {
		return errors.Join(errExecutionCurrentRefused, err)
	}
	defer func() { resultErr = errors.Join(resultErr, selected.Close()) }()
	executor, err := executionHostExecutor(ctx, selected.generation)
	if err != nil {
		return err
	}
	// BindExecutorImage refuses nonzero sequence or a previous binding. A new
	// transport alone cannot authenticate an obsolete launcher as current selected.
	if err = session.BindExecutorImage(ctx, executor); err != nil {
		return err
	}
	image, err := executionHostApplication(ctx, selected.generation)
	if err != nil {
		return err
	}
	identity, err := image.Identity(ctx)
	if err != nil {
		return err
	}
	if identity.ClientSID != peer.SID || identity.Generation != selected.selected.generation || identity.Commit != selected.selected.installed.Commit {
		return errExecutionCurrentRefused
	}
	observed, stock, err := e.observe(ctx, registration)
	if stock != nil {
		defer func() { resultErr = errors.Join(resultErr, stock.Close()) }()
	}
	if err != nil || stock == nil || observed != stockBaseline {
		return errors.Join(errExecutionCurrentRefused, err)
	}
	h := &nativeExecutionCurrentHost{peer: capability, selected: selected, image: image, identity: identity, providers: providers}
	validate := func(ctx context.Context) error {
		if err := h.validate(ctx); err != nil {
			return err
		}
		return stock.validate()
	}
	run := &executionCurrentHostRun{generation: selected.selected.generation, conversation: session, validate: validate, beforeSpawn: h.beforeSpawn, corroborate: h.corroborate}
	return run.execute(ctx)
}
func (h *nativeExecutionCurrentHost) validate(ctx context.Context) error {
	if ctx == nil || ctx.Err() != nil {
		return errors.Join(errExecutionCurrentRefused, contextError(ctx))
	}
	if err := h.selected.validate(); err != nil {
		return err
	}
	if err := h.peer.Validate(ctx); err != nil {
		return err
	}
	identity, err := h.image.Identity(ctx)
	if err != nil || identity != h.identity {
		return errors.Join(errExecutionCurrentRefused, err)
	}
	if err = h.providers.authorize(ctx, h.selected); err != nil {
		return err
	}
	return h.peer.Validate(ctx)
}
func (h *nativeExecutionCurrentHost) beforeSpawn(ctx context.Context) error {
	if h.spawnAuthorized {
		return errExecutionCurrentRefused
	}
	h.spawnAuthorized = true // burn before the final gate or native timestamp read
	if err := h.validate(ctx); err != nil {
		return err
	}
	if err := h.providers.prepare(ctx, h.selected); err != nil {
		return err
	}
	var now windows.Filetime
	windows.GetSystemTimeAsFileTime(&now)
	h.bornAfter = uint64(now.HighDateTime)<<32 | uint64(now.LowDateTime)
	if h.bornAfter == 0 {
		return errExecutionCurrentRefused
	}
	return ctx.Err()
}
func (h *nativeExecutionCurrentHost) corroborate(ctx context.Context, report brokerpipe.ExecutionObservation) (result executionCurrentHostChild, resultErr error) {
	if !h.spawnAuthorized || h.bornAfter == 0 || report.ProcessID == 0 || report.ProcessCreation == 0 {
		return nil, errExecutionCurrentRefused
	}
	// The reported PID is ONLY a lookup. OpenProcess owns the actual original
	// native process/token; creation, image, direct retained executor parent and
	// interactive SID/session/AuthLUID must independently corroborate it below.
	process, err := openExecutionProcess(report.ProcessID)
	if err != nil {
		return nil, err
	}
	child := &nativeExecutionCurrentChild{host: h, process: process}
	defer func() {
		if resultErr != nil {
			resultErr = errors.Join(resultErr, child.Close())
		}
	}()
	if err = child.Validate(ctx, report); err != nil {
		return nil, err
	}
	return child, nil
}

type nativeExecutionCurrentChild struct {
	host     *nativeExecutionCurrentHost
	process  *executionProcess
	closed   bool
	closeErr error
}

func (c *nativeExecutionCurrentChild) Validate(ctx context.Context, report brokerpipe.ExecutionObservation) error {
	if c == nil || c.closed || c.host == nil || c.process == nil || ctx == nil {
		return errExecutionCurrentRefused
	}
	h, p := c.host, c.process
	if report.ProcessID != p.pid || report.ProcessCreation != p.created || report.EvidenceSHA256 != h.identity.ExecutableSHA256 || p.created < h.bornAfter {
		return errExecutionCurrentRefused
	}
	return h.peer.WithExecutor(ctx, func(executor brokerpipe.PeerIdentity) error {
		if executor.SID != h.identity.ClientSID || p.parent != executor.ProcessID || p.pid == executor.ProcessID || p.created <= executor.ProcessCreation || p.identity.sid != executor.SID || p.identity.session != executor.SessionID || p.identity.logon.HighPart != executor.LogonIDHigh || p.identity.logon.LowPart != executor.LogonIDLow {
			return errExecutionCurrentRefused
		}
		if err := p.check(); err != nil {
			return err
		}
		if err := h.image.MatchesProcess(ctx, p.pid, p.created); err != nil {
			return err
		}
		return p.check()
	})
}
func (c *nativeExecutionCurrentChild) Close() error {
	if c == nil {
		return nil
	}
	if c.closed {
		return c.closeErr
	}
	c.closed = true
	c.closeErr = c.process.close()
	return c.closeErr
}
