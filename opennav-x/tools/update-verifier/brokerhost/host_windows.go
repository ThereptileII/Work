package brokerhost

import (
	"context"
	"errors"
	"path/filepath"
	"time"

	verifier "example.com/opennav-update-verifier"
	"example.com/opennav-update-verifier/brokerenroll"
	"example.com/opennav-update-verifier/brokerpipe"
	"example.com/opennav-update-verifier/brokerstore"
	"example.com/opennav-update-verifier/releasepolicy"
	"golang.org/x/sys/windows"
	"golang.org/x/sys/windows/svc"
)

type storeAdapter struct{ *brokerstore.Store }

func (s storeAdapter) openBaseline(ctx context.Context, r registration) (baselineOwner, error) {
	baseline, err := s.OpenInstalledBaseline(ctx, r.ClientSID, r.Installed.Generation,
		r.ApprovedRequestSHA256, r.Installed)
	if baseline == nil {
		return nil, err
	}
	return baseline, err
}

func (s storeAdapter) begin(id, sid string) (candidate, error) {
	c, err := s.Begin(id, sid)
	if err != nil {
		return nil, err
	}
	return c, nil
}

func (s storeAdapter) materialize(ctx context.Context, previous baselineOwner, stage candidate) (generationOwner, error) {
	baseline, ok := previous.(*brokerstore.InstalledBaseline)
	if !ok || baseline == nil {
		return nil, errors.New("native retained installed baseline required")
	}
	native, ok := stage.(*brokerstore.Candidate)
	if !ok || native == nil {
		return nil, errors.New("native reserved candidate required")
	}
	generation, err := s.MaterializeCandidateReadOnlyFresh(ctx, baseline, native)
	if generation == nil {
		return nil, err
	}
	// Preserve even an owner returned with an error so handle drains it.
	return generation, err
}

// Run is exclusively an SCM service entry point. It has no arbitrary-root,
// interactive/elevated console fallback, trust initialization or install command.
// Explicit SCM initialize-store / verify-store / register-client starts perform
// only fixed native initialization, completion validation or independently
// approved client registration, then stop without announcing readiness.
// Provisioning of the account, fixed files and protected registrations is a
// separate authenticated installer operation; missing setup fails closed.
func Run(ctx context.Context) error {
	if ctx == nil || ctx.Err() != nil {
		return errors.New("service context required")
	}
	isService, err := svc.IsWindowsService()
	if err != nil || !isService {
		return errors.New("Windows broker service required")
	}
	h := &serviceHost{ctx: ctx}
	if err := svc.Run(ServiceName, h); err != nil {
		return errors.New("broker service could not run")
	}
	return h.result
}

type serviceHost struct {
	ctx    context.Context
	result error
}

func (h *serviceHost) Execute(args []string, changes <-chan svc.ChangeRequest, status chan<- svc.Status) (bool, uint32) {
	status <- svc.Status{State: svc.StartPending, WaitHint: 10000}
	// The operator selects only SID + independently approved request hash. Read
	// original bounded bytes from the fixed native protected namespace, then
	// revalidate and register as the actual broker before Running or any pipe.
	registered, err := registerClientServiceStart(h.ctx, args,
		brokerenroll.ReadApprovedClientRequestNative, RegisterApprovedClientNative)
	if err != nil {
		h.result = err
		return false, 1
	}
	if registered {
		return false, 0
	}
	initialized, err := initializeServiceStart(h.ctx, args, brokerstore.InitializeFresh, brokerstore.ValidateInitialization)
	if err != nil {
		h.result = err
		return false, 1
	}
	if initialized {
		return false, 0
	}
	if err := brokerstore.ValidateInitialization(); err != nil {
		h.result = errors.New("completed broker initialization required")
		return false, 1
	}
	programData, err := windows.KnownFolderPath(windows.FOLDERID_ProgramData, 0)
	if err != nil {
		h.result = errors.New("machine data directory unavailable")
		return false, 1
	}
	store, err := brokerstore.Open(filepath.Join(programData, "SKAGER", "UpdateBroker"))
	if err != nil {
		h.result = errors.New("protected broker store unavailable")
		return false, 1
	}
	canClose := true
	defer func() {
		// On an undrained shutdown the own-process service must exit rather than
		// release protected owners concurrently with a still-active handler.
		// main exits nonzero after Run returns; the OS then releases its handles.
		if canClose {
			h.result = errors.Join(h.result, store.Close())
		}
	}()
	e := &engine{store: storeAdapter{store}, observe: observeStock,
		selected: func(ctx context.Context, r registration) (selectedExecutionOwner, error) {
			return captureSelectedExecutionNative(ctx, store, r)
		},
		prepare: func(ctx context.Context, c verifier.PrepareConfig, stock releasepolicy.OpenCpn, consent verifier.ConsentIdentity, stage candidate) (acquisition, error) {
			// Only the same newly reserved native candidate can own acquisition.
			// Neither a caller-selected directory nor a reconstructed receipt can
			// replace its protected lease and retained native artifact owners.
			native, ok := stage.(*brokerstore.Candidate)
			if !ok || native == nil {
				return nil, errors.New("native reserved candidate required")
			}
			p, err := verifier.PrepareBrokerUpdateSet(ctx, c, stock, consent, native)
			if err != nil {
				return nil, err
			}
			return p, nil
		}}
	ctx, cancel := context.WithCancel(h.ctx)
	defer cancel()
	current := svc.Status{State: svc.Running, Accepts: svc.AcceptStop | svc.AcceptShutdown}
	// Pipe authentication independently requires the SCM running identity.
	status <- current
	done := make(chan error, 1)
	canClose = false
	go func() {
		done <- serveBrokerEndpoints(ctx,
			func(ctx context.Context) error { return brokerpipe.Serve(ctx, e.handleNativeStaging) },
			func(ctx context.Context) error { return brokerpipe.ServeExecution(ctx, e.handleNativeExecutionRequest) })
	}()
	stop := func() (bool, uint32) {
		cancel()
		canClose = h.stop(done, status)
		if h.result != nil {
			return false, 1
		}
		return false, 0
	}
	for {
		select {
		case err := <-done:
			canClose = true
			h.result = err
			if err != nil {
				return false, 1
			}
			return false, 0
		case <-h.ctx.Done():
			return stop()
		case change, ok := <-changes:
			if !ok {
				return stop()
			}
			switch change.Cmd {
			case svc.Interrogate:
				status <- current
			case svc.Stop, svc.Shutdown:
				return stop()
			}
		}
	}
}

// Dispatch explicit protocol intent on the same protected Store/operation lock.
// A failed qualification is never reinterpreted as permission for current startup.
// Providers are private per-session owners and are drained before the session
// and its retained native peer can be released by the serving transport.
func (e *engine) handleNativeExecutionRequest(ctx context.Context, peer brokerpipe.Peer, request brokerpipe.ExecutionRequest, session *brokerpipe.ExecutionSession) (resultErr error) {
	if e == nil {
		return errExecutionHostRefused
	}
	adapter, ok := e.store.(storeAdapter)
	if !ok || adapter.Store == nil {
		return errExecutionHostRefused
	}
	switch request.Operation {
	case "execute-session":
		providers, closeProviders := nativeExecutionProviders(adapter.Store)
		defer func() { resultErr = errors.Join(resultErr, closeProviders()) }()
		return e.handleNativeExecution(ctx, peer, request, session, providers)
	case "start-current":
		providers, closeProviders := nativeExecutionCurrentProviders()
		defer func() { resultErr = errors.Join(resultErr, closeProviders()) }()
		return e.handleNativeCurrentExecution(ctx, peer, request, session, providers)
	default:
		return errExecutionHostRefused
	}
}

// Retain actual authenticated connection/process/token custody for the whole
// staging operation, not only its initial public Peer metadata. A late native
// identity/disconnect failure refuses the response; durable staged bytes remain
// reserved and cannot turn that uncertainty into launch or retry authority.
func (e *engine) handleNativeStaging(ctx context.Context, peer brokerpipe.Peer, request brokerpipe.Request) (brokerpipe.Result, error) {
	capability, err := brokerpipe.AuthorizePeer(ctx, peer)
	if err != nil {
		return brokerpipe.Result{}, errStageRefused
	}
	var result brokerpipe.Result
	err = capability.WithPeer(ctx, func(identity brokerpipe.PeerIdentity) error {
		if identity.Peer != peer {
			return errStageRefused
		}
		var operationErr error
		result, operationErr = e.handle(ctx, peer, request)
		return operationErr
	})
	if err != nil {
		return brokerpipe.Result{}, errStageRefused
	}
	return result, nil
}

func (h *serviceHost) stop(done <-chan error, status chan<- svc.Status) bool {
	status <- svc.Status{State: svc.StopPending, WaitHint: 45000}
	timer := time.NewTimer(45 * time.Second)
	defer timer.Stop()
	select {
	case err := <-done:
		// Joined cancellation + owner/cleanup failure is not a clean shutdown.
		if err != nil && err != context.Canceled {
			h.result = errors.New("broker shutdown refused")
			return true
		}
		return true
	case <-timer.C:
		h.result = errors.New("broker shutdown deadline exceeded")
		return false
	}
}
