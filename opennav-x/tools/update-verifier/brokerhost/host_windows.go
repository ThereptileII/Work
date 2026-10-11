package brokerhost

import (
	"context"
	"errors"
	"path/filepath"
	"time"

	verifier "example.com/opennav-update-verifier"
	"example.com/opennav-update-verifier/brokerpipe"
	"example.com/opennav-update-verifier/brokerstore"
	"example.com/opennav-update-verifier/releasepolicy"
	"golang.org/x/sys/windows"
	"golang.org/x/sys/windows/svc"
)

type storeAdapter struct{ *brokerstore.Store }

func (s storeAdapter) begin(id, sid string) (candidate, error) {
	c, err := s.Begin(id, sid)
	if err != nil {
		return nil, err
	}
	return c, nil
}

// Run is exclusively an SCM service entry point. It has no arbitrary-root,
// interactive/elevated console fallback, trust initialization or install command.
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
	if len(args) != 1 || args[0] != ServiceName {
		h.result = errors.New("unexpected broker service arguments")
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
		prepare: func(ctx context.Context, c verifier.PrepareConfig, stock releasepolicy.OpenCpn, consent verifier.ConsentIdentity) (acquisition, error) {
			p, err := verifier.PrepareUpdateSet(ctx, c, stock, consent)
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
	go func() { done <- brokerpipe.Serve(ctx, e.handle) }()
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

func (h *serviceHost) stop(done <-chan error, status chan<- svc.Status) bool {
	status <- svc.Status{State: svc.StopPending, WaitHint: 45000}
	timer := time.NewTimer(45 * time.Second)
	defer timer.Stop()
	select {
	case err := <-done:
		if err != nil && !errors.Is(err, context.Canceled) {
			h.result = errors.New("broker shutdown refused")
			return true
		}
		return true
	case <-timer.C:
		h.result = errors.New("broker shutdown deadline exceeded")
		return false
	}
}
