package brokerhost

import (
	"context"
	"encoding/json"
	"errors"
	"runtime"
	"time"

	"example.com/opennav-update-verifier/brokerpipe"
	"example.com/opennav-update-verifier/brokerstore"
	"example.com/opennav-update-verifier/releasepolicy"
	"golang.org/x/sys/windows"
)

// The SCM wiring independently enforces explicit consent/current signed policy,
// all armed/commissioning denial guards, native nonmutating qualification and full real
// profile owners. Missing providers refuse before a new journal is reserved.
// These are private service integration seams, never pipe/client configuration.
type executionHostProviders struct {
	// Independently authorize the protected stage against current retained signed
	// policy and actual selected predecessor; completion provenance is insufficient.
	// Called read-only before reservation and at every effect boundary. During the
	// transaction selected.validate() cannot admit the new journal: this host owns
	// the exact inventory extension, while authorization rechecks retained policy,
	// consent and original selected bytes/version without inventing a new head.
	authorize  func(context.Context, *brokerstore.ReopenedGeneration, *nativeSelectedExecution) error
	capture    func(context.Context, *brokerstore.Store, *brokerstore.ExecutionJournal) (executionHostProfile, error)
	commission func(context.Context, *brokerstore.GenerationApplicationImage, *ExecutionColdProfile, executionHostProfile) (string, error)
}

type nativeExecutionHost struct {
	ctx                context.Context
	store              *brokerstore.Store
	peer               *brokerpipe.PeerCapability
	conversation       *brokerpipe.ExecutionSession
	registration       registration
	registrationDigest string
	selected           *nativeSelectedExecution
	target             *brokerstore.ReopenedGeneration
	journal            *brokerstore.ExecutionJournal
	inventory          string
	executionID        string
	startupMode        brokerpipe.StartupMode
	begin              executionSelectionBegin
	stock              stockOwner
	profile            executionHostProfile
	providers          executionHostProviders
	cold               []*ExecutionColdProfile
}

// handleNativeExecution is the actual service handler adapter. Root service
// wiring must use the same engine.busy as staging; ServeExecution retains and
// revokes the native peer/session at the handler boundary. No public request or
// registration version supplies selection, commissioning or spawn permission.
func (e *engine) handleNativeExecution(ctx context.Context, peer brokerpipe.Peer, request brokerpipe.ExecutionRequest, session *brokerpipe.ExecutionSession, providers executionHostProviders) (resultErr error) {
	if e == nil || ctx == nil || ctx.Err() != nil || session == nil || providers.capture == nil || providers.commission == nil || providers.authorize == nil || e.observe == nil || !request.StartupMode.Valid() || (request.Operation != "start-selected" && request.StartupMode != brokerpipe.StartupXNav) || request.Schema != 1 || !executionQualificationIntentAllowed(request.Operation, true) || !requestPattern.MatchString(request.RequestID) || !requestPattern.MatchString(request.StageID) {
		return errExecutionHostRefused
	}
	adapter, ok := e.store.(storeAdapter)
	if !ok || adapter.Store == nil {
		return errExecutionHostRefused
	}
	capability, err := brokerpipe.AuthorizePeer(ctx, peer)
	if err != nil {
		return errExecutionHostRefused
	}
	if !e.busy.TryLock() {
		return errExecutionHostRefused
	}
	defer e.busy.Unlock()
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	h := &nativeExecutionHost{ctx: ctx, store: adapter.Store, peer: capability, conversation: session, executionID: request.RequestID, startupMode: request.StartupMode, providers: providers}
	defer func() {
		// Cleanup failure does not undo a durable terminal. A later fresh read may
		// establish it; this handler never retries a consumed operation.
		var cleanup error
		for i := len(h.cold) - 1; i >= 0; i-- {
			cleanup = errors.Join(cleanup, h.cold[i].Close())
		}
		if h.profile != nil {
			cleanup = errors.Join(cleanup, h.profile.Close())
		}
		if h.journal != nil {
			cleanup = errors.Join(cleanup, h.journal.Close())
		}
		if h.target != nil {
			cleanup = errors.Join(cleanup, h.target.Close())
		}
		if h.selected != nil {
			cleanup = errors.Join(cleanup, h.selected.Close())
		}
		if h.stock != nil {
			cleanup = errors.Join(cleanup, h.stock.Close())
		}
		resultErr = errors.Join(resultErr, cleanup)
		if resultErr != nil {
			if h.journal != nil {
				resultErr = errors.Join(errExecutionHostUncertain, resultErr)
			} else {
				resultErr = errors.Join(errExecutionHostRefused, resultErr)
			}
		}
	}()
	if err = h.store.Validate(); err != nil {
		return err
	}
	raw, err := h.store.ReadRegistration(peer.SID)
	if err != nil {
		return err
	}
	h.registration, err = parseRegistration(raw, peer.SID)
	if err != nil {
		return err
	}
	h.registrationDigest = executionOriginalDigest(raw)
	entries, err := h.store.EnumerateExecutionJournals(ctx, peer.SID)
	if err != nil {
		return err
	}
	if len(entries) >= 128 {
		return errExecutionHostRefused
	}
	h.inventory, err = executionSelectionInventoryDigest(peer.SID, entries)
	if err != nil {
		return err
	}
	initial := len(entries) == 0
	if !executionQualificationIntentAllowed(request.Operation, initial) {
		return errExecutionHostRefused
	}
	if initial {
		// Registration produces provenance only. Only an explicit fresh baseline
		// qualification with an empty exact history may establish the first root.
		if request.StageID != h.registration.Installed.Generation {
			return errExecutionHostRefused
		}
	} else {
		h.selected, err = captureSelectedExecutionNative(ctx, h.store, h.registration)
		if err != nil {
			return err
		}
		if h.selected.inventory != h.inventory {
			return errExecutionHostRefused
		}
		if h.selected.selected.generation == request.StageID {
			return errExecutionHostRefused
		}
	}
	h.target, err = h.store.OpenReadOnlyGeneration(ctx, peer.SID, request.StageID)
	if err != nil {
		return err
	}
	var executor *brokerstore.GenerationExecutorImage
	var completion, version, commit string
	if initial {
		baseline, err := h.target.ApprovedBaseline()
		if err != nil {
			return err
		}
		c, err := baseline.Completion()
		if err != nil || c.ApprovedRequestSHA256 != h.registration.ApprovedRequestSHA256 {
			return errors.Join(errExecutionHostRefused, err)
		}
		completion, version, commit = c.CompletionSHA256, c.Identity.Version, c.Identity.Commit
		executor, err = baseline.ExecutorImage(ctx)
		if err != nil {
			return err
		}
	} else {
		candidate, err := h.target.Candidate()
		if err != nil {
			return err
		}
		c, err := candidate.Completion()
		if err != nil {
			return err
		}
		completion, version, commit = c.CompletionSHA256, c.Identity.Version, c.Identity.Commit
		nextVersion, nextErr := releasepolicy.ParseSemVer(version)
		previousVersion, previousErr := releasepolicy.ParseSemVer(h.selected.selected.installed.Version)
		if nextErr != nil || previousErr != nil || nextVersion.Compare(previousVersion) <= 0 {
			return errExecutionHostRefused
		}
		executor, err = executionHostExecutor(ctx, h.selected.generation)
		if err != nil {
			return err
		}
	}
	if err = providers.authorize(ctx, h.target, h.selected); err != nil {
		return err
	}
	if err = session.BindExecutorImage(ctx, executor); err != nil {
		return err
	}
	observed, stock, err := e.observe(ctx, h.registration)
	h.stock = stock
	if err != nil || stock == nil || observed != stockBaseline {
		return errors.Join(errExecutionHostRefused, err)
	}
	h.begin = executionSelectionBegin{Schema: 1, Purpose: "broker-selected-transaction-v1", ExecutionID: request.RequestID, ClientSID: peer.SID, Generation: request.StageID, Version: version, Commit: commit, CompletionSHA256: completion}
	if !initial {
		h.begin.PreviousGeneration = h.selected.selected.generation
		h.begin.PreviousHeadSHA256 = h.selected.selected.head
	}
	if err = h.validate(ctx); err != nil {
		return err
	}
	if initial {
		baseline, err := h.target.ApprovedBaseline()
		if err != nil {
			return err
		}
		h.journal, err = h.store.BeginBaselineQualificationJournal(ctx, baseline, request.RequestID)
		if err != nil {
			return err
		}
	} else {
		candidate, err := h.target.Candidate()
		if err != nil {
			return err
		}
		h.journal, err = h.store.BeginExecutionJournal(ctx, candidate, request.RequestID)
		if err != nil {
			return err
		}
	}
	if err = h.validate(ctx); err != nil {
		return err
	}
	run := &executionHostRun{begin: h.begin, journal: h.journal, conversation: session,
		validate: h.validate, validateCleanup: h.validateRecovery, validateRecovery: h.validateRecovery,
		commission: h.commission, prepare: h.prepare, resolve: h.resolve, prepareRestore: h.prepareRestore}
	run.capture = func(ctx context.Context) (executionHostProfile, error) {
		profile, err := providers.capture(ctx, h.store, h.journal)
		h.profile = profile
		return profile, err
	}

	return run.execute(ctx)
}

func executionHostExecutor(ctx context.Context, g *brokerstore.ReopenedGeneration) (*brokerstore.GenerationExecutorImage, error) {
	switch g.Purpose() {
	case brokerstore.GenerationPurposeApprovedBaseline:
		b, e := g.ApprovedBaseline()
		if e != nil {
			return nil, e
		}
		return b.ExecutorImage(ctx)
	case brokerstore.GenerationPurposeSignedCandidate:
		c, e := g.Candidate()
		if e != nil {
			return nil, e
		}
		return c.ExecutorImage(ctx)
	}
	return nil, errExecutionHostRefused
}
func executionHostApplication(ctx context.Context, g *brokerstore.ReopenedGeneration) (*brokerstore.GenerationApplicationImage, error) {
	switch g.Purpose() {
	case brokerstore.GenerationPurposeApprovedBaseline:
		b, e := g.ApprovedBaseline()
		if e != nil {
			return nil, e
		}
		return b.ApplicationImage(ctx)
	case brokerstore.GenerationPurposeSignedCandidate:
		c, e := g.Candidate()
		if e != nil {
			return nil, e
		}
		return c.ApplicationImage(ctx)
	}
	return nil, errExecutionHostRefused
}

func (h *nativeExecutionHost) validate(ctx context.Context) error {
	return h.validateState(ctx, true, true)
}

// Cleanup and recovery retain native transaction/predecessor custody. They do
// not need a still-reachable update server to stop the original child or restore
// the independently qualified previous version. New candidate launch/selection
// always uses validate, including current signed-offer eligibility.
func (h *nativeExecutionHost) validateRecovery(ctx context.Context) error {
	return h.validateState(ctx, true, false)
}

// Restore holds the profile owner's own mutex while validating retained native
// authority. Avoid recursively taking that mutex. Original peer, generation,
// archive and journal authority remain required; current network policy is
// deliberately separate from retained cleanup/recovery authority.
func (h *nativeExecutionHost) validateState(ctx context.Context, inspectProfile, currentPolicy bool) error {
	if ctx == nil || ctx.Err() != nil {
		return errors.Join(errExecutionHostRefused, contextError(ctx))
	}
	if err := h.peer.Validate(ctx); err != nil {
		return err
	}
	if err := h.store.Validate(); err != nil {
		return err
	}
	raw, err := h.store.ReadRegistration(h.registration.ClientSID)
	if err != nil || executionOriginalDigest(raw) != h.registrationDigest {
		return errors.Join(errExecutionHostRefused, err)
	}
	if h.selected != nil {
		// validate() intentionally cannot admit a new namespace. Retain its exact
		// originals, and use the narrowly extended inventory check below instead.
		for _, reader := range h.selected.readers {
			if err = reader.Validate(); err != nil {
				return err
			}
		}
		if err = h.selected.generation.Validate(); err != nil {
			return err
		}
	}
	if err = h.target.Validate(); err != nil {
		return err
	}
	if h.stock != nil {
		if err = h.stock.validate(); err != nil {
			return err
		}
	}
	var own *brokerstore.ExecutionJournalSnapshot
	if h.journal != nil {
		snapshot, err := h.journal.Snapshot()
		if err != nil {
			return err
		}
		if snapshot.Generation != h.begin.Generation || snapshot.CompletionSHA256 != h.begin.CompletionSHA256 {
			return errExecutionHostRefused
		}
		own = &snapshot
	}
	entries, enumerationErr := h.store.EnumerateExecutionJournals(ctx, h.registration.ClientSID)
	if err = ctx.Err(); err != nil {
		return err
	}
	if err = executionHostInventory(h.registration.ClientSID, h.executionID, h.inventory, entries, enumerationErr, own); err != nil {
		return err
	}
	if inspectProfile && h.profile != nil {
		if err = h.profile.Validate(ctx); err != nil {
			return err
		}
	}
	// Recheck the live original writer AFTER admitting its names. The exception
	// never authenticates bytes from an independently failed open.
	if own != nil {
		after, err := h.journal.Snapshot()
		if err != nil {
			return err
		}
		if after.ExecutionID != own.ExecutionID || after.ClientSID != own.ClientSID || after.Generation != own.Generation || after.CompletionSHA256 != own.CompletionSHA256 || after.JournalSHA256 != own.JournalSHA256 {
			return errExecutionHostRefused
		}
	}
	if err = h.peer.Validate(ctx); err != nil {
		return err
	}
	if currentPolicy {
		if err = h.providers.authorize(ctx, h.target, h.selected); err != nil {
			return err
		}
	}
	return ctx.Err()
}
func (h *nativeExecutionHost) commission(ctx context.Context, fallback bool) (string, error) {
	validate := h.validate
	if fallback {
		validate = h.validateRecovery
	}
	if err := validate(ctx); err != nil {
		return "", err
	}
	generation := h.target
	if fallback {
		if h.selected == nil {
			return "", errExecutionHostRefused
		}
		generation = h.selected.generation
	}
	image, err := executionHostApplication(ctx, generation)
	if err != nil {
		return "", err
	}
	cold, err := CaptureExecutionColdProfileNative(ctx)
	if err != nil {
		return "", err
	}
	h.cold = append(h.cold, cold)
	// Actual provider must be NONMUTATING: suppression/plugin deferral are fixed
	// producer capabilities. Stored connections, routes and COM8 stay untouched.
	digest, err := h.providers.commission(ctx, image, cold, h.profile)
	if err != nil {
		return "", err
	}
	if err = cold.ValidateBeforeLaunch(ctx); err != nil {
		return "", err
	}
	return digest, validate(ctx)
}
func (h *nativeExecutionHost) prepare(ctx context.Context, fallback bool) (executionHostChild, error) {
	if len(h.cold) == 0 {
		return nil, errExecutionHostRefused
	}
	generation := h.target
	if fallback {
		generation = h.selected.generation
	}
	image, err := executionHostApplication(ctx, generation)
	if err != nil {
		return nil, err
	}
	observer, err := PrepareExecutionChildNative(ctx, h.peer, image, h.cold[len(h.cold)-1])
	if err != nil {
		return nil, err
	}
	native, ok := observer.state.backend.(*nativeExecutionChild)
	if !ok {
		return nil, errors.Join(errExecutionHostRefused, observer.Close())
	}
	retained := &executionHostChildBackend{executionChildBackend: native, native: native}
	observer.state.backend = retained
	return &nativeExecutionHostChild{observer: observer, backend: retained}, nil
}
func (h *nativeExecutionHost) resolve(ctx context.Context, terminal string) (resultErr error) {
	snapshot, err := h.journal.Snapshot()
	if err != nil {
		return err
	}
	if snapshot.ExecutionID != h.executionID || snapshot.ClientSID != h.registration.ClientSID || snapshot.Generation != h.begin.Generation || snapshot.CompletionSHA256 != h.begin.CompletionSHA256 {
		return errExecutionHostRefused
	}
	if err = h.journal.Close(); err != nil {
		return err
	}
	selected, err := captureSelectedExecutionNative(ctx, h.store, h.registration)
	if err != nil {
		return err
	}
	defer func() { resultErr = errors.Join(resultErr, selected.Close()) }()
	want := h.begin.Generation
	if terminal == "rolled-back" {
		want = h.begin.PreviousGeneration
	}
	if selected.selected.generation != want || selected.selected.head != snapshot.JournalSHA256 {
		return errExecutionHostRefused
	}
	if terminal == "selected" && selected.selected.completion != h.begin.CompletionSHA256 {
		return errExecutionHostRefused
	}
	if terminal == "rolled-back" && (h.selected == nil || selected.selected.completion != h.selected.selected.completion || selected.selected.installed != h.selected.selected.installed) {
		return errExecutionHostRefused
	}
	if err = selected.validate(); err != nil {
		return err
	}
	// Close the complete profile archive BEFORE starting the fresh launcher.
	// Closing validates the original archive and may hash a large profile; that
	// work must not consume the new launcher's bounded pipe-handoff budget or
	// retain cold profile pins into ordinary startup. The reopened selected head
	// and original peer remain live and independently checked below. Close is
	// idempotent, so outer failure cleanup retains the same original owners.
	for i := len(h.cold) - 1; i >= 0; i-- {
		if err = h.cold[i].Close(); err != nil {
			return err
		}
	}
	if h.profile == nil {
		return errExecutionHostRefused
	}
	if err = h.profile.Close(); err != nil {
		return err
	}
	if err = errors.Join(selected.validate(), h.peer.Validate(ctx)); err != nil {
		return err
	}
	// The qualification child remains permanently restricted. Ordinary operation
	// always uses a new selected launcher and a new start-current conversation;
	// never clear qualification flags or let the old executor create the new
	// application's ordinary child in this already-used session.
	return relaySelectedExecutionNative(ctx, h.peer, selected, h.conversation, h.startupMode)
}

// Retain the independently corroborated ORIGINAL child immediately after the
// creator reports a spawn. A report supplies only a lookup; native parent/token,
// creation and held application-image custody establish identity. Later health
// pipe authentication must identify this same original process. Observer failure
// cannot erase its independent exit observation.
type executionHostChildBackend struct {
	executionChildBackend
	native  *nativeExecutionChild
	process windows.Handle
	pid     uint32
	created uint64
}

func (b *executionHostChildBackend) authenticate(ctx context.Context) (time.Duration, error) {
	age, err := b.executionChildBackend.authenticate(ctx)
	if err != nil {
		return 0, err
	}
	child := b.native.child
	if child == nil || b.process == 0 || b.pid == 0 || b.created == 0 || child.pid != b.pid || child.created != b.created {
		return 0, errExecutionHostRefused
	}
	if err = child.check(); err != nil {
		return 0, err
	}
	return age, nil
}

type nativeExecutionHostChild struct {
	observer *ExecutionChildObserver
	backend  *executionHostChildBackend
	healthy  *ObservedExecutionChild
	closed   bool
	closeErr error
}

func (c *nativeExecutionHostChild) Environment(ctx context.Context) (map[string]string, error) {
	return c.observer.Environment(ctx)
}
func (c *nativeExecutionHostChild) Observe(ctx context.Context) (string, error) {
	healthy, err := c.observer.Observe(ctx)
	if err != nil {
		return "", err
	}
	c.healthy = healthy
	if err = healthy.Validate(ctx); err != nil {
		return "", err
	}
	return c.evidence("healthy"), nil
}
func (c *nativeExecutionHostChild) ValidateHealthy(ctx context.Context) error {
	if c.healthy == nil {
		return errExecutionHostRefused
	}
	return c.healthy.Validate(ctx)
}
func (c *nativeExecutionHostChild) evidence(phase string) string {
	// Original native identity, source/configuration and fresh endpoint challenge
	// bind the local broker fact. This digest is informational outside this owner.
	s := c.observer.state
	configuration := s.profile.state.original.wire() // immutable original, no writes after native capture
	raw, _ := json.Marshal(struct {
		Phase                    string
		PID                      uint32
		Created                  uint64
		Identity                 brokerstore.GenerationApplicationIdentity
		Configuration, Challenge string
	}{phase, c.backend.pid, c.backend.created, s.identity, configuration, s.environment["SKAGER_UPDATE_CHALLENGE"]})
	return executionOriginalDigest(raw)
}
func (c *nativeExecutionHostChild) WaitClosed(ctx context.Context) (string, error) {
	if c.closed || c.backend.process == 0 || c.backend.pid == 0 || c.backend.created == 0 {
		return "", errExecutionHostUncertain
	}
	deadline, cancel := context.WithTimeout(ctx, 45*time.Second)
	defer cancel()
	for {
		if err := deadline.Err(); err != nil {
			return "", err
		}
		status, err := windows.WaitForSingleObject(c.backend.process, 100)
		if err != nil {
			return "", err
		}
		if status == windows.WAIT_OBJECT_0 {
			return c.evidence("closed"), nil
		}
		if status != uint32(windows.WAIT_TIMEOUT) {
			return "", errExecutionHostUncertain
		}
	}
}
func (c *nativeExecutionHostChild) Close() error {
	if c.closed {
		return c.closeErr
	}
	c.closed = true
	c.closeErr = c.observer.Close()
	if c.backend.process != 0 {
		c.closeErr = errors.Join(c.closeErr, windows.CloseHandle(c.backend.process))
		c.backend.process = 0
	}
	return c.closeErr
}
