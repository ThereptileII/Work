package brokerenroll

import (
	"context"
	"errors"
	"time"
)

// The private backend is a lifecycle test seam, never caller-supplied authority.
// Closing it releases handles only; partial services are never deleted/adopted.
type serviceBackend interface {
	create(string) error
	check(string) error
	start(string) error
	status() (serviceState, error)
	close() error
}

type serviceState struct {
	state, processID, win32Exit, serviceExit, controls uint32
}

const (
	serviceStopped             = 1
	serviceStartPending        = 2
	serviceStopPending         = 3
	serviceInitializationLimit = 60 * time.Second
)

func initializeService(ctx context.Context, files enrollmentFiles) error {
	return initializeServiceWith(ctx, files, newServiceBackend, servicePoll)
}

func servicePoll(ctx context.Context) error {
	t := time.NewTimer(50 * time.Millisecond)
	defer t.Stop()
	select {
	case <-ctx.Done():
		return ctx.Err()
	case <-t.C:
		return nil
	}
}

func initializeServiceWith(ctx context.Context, files enrollmentFiles, open func() (serviceBackend, error), poll func(context.Context) error) (resultErr error) {
	return initializeServiceWithCompletion(ctx, files, open, poll, nil)
}

// complete is private and reached only after original stopped registration on
// this same freshly created retained SCM handle. It cannot adopt an existing service.
func initializeServiceWithCompletion(ctx context.Context, files enrollmentFiles, open func() (serviceBackend, error), poll func(context.Context) error, complete func(context.Context, enrollmentFiles, serviceBackend) error) (resultErr error) {
	if ctx == nil || ctx.Err() != nil || files == nil || open == nil || poll == nil {
		return errors.New("live native enrollment custody required")
	}
	if err := files.Check(); err != nil {
		return err
	}
	image := files.ImagePath()
	if image == "" {
		return errors.New("held broker image required")
	}
	var sid, approvedSHA string
	client, hasClient := files.(clientRequestFiles)
	if hasClient {
		sid, approvedSHA = client.clientRegistration()
		if (sid != "" || approvedSHA != "") && (!canonicalClientSID(sid) || !validPin(approvedSHA)) {
			return errors.New("held approved client request binding required")
		}
	}
	b, err := open()
	if err != nil {
		return err
	}
	if b == nil {
		return errors.New("native service backend required")
	}
	defer func() { resultErr = errors.Join(resultErr, b.close()) }()
	if err := ctx.Err(); err != nil {
		return err
	}
	if err := files.Check(); err != nil {
		return err
	}
	if err := b.create(image); err != nil {
		return err
	}
	check := func() error {
		if err := ctx.Err(); err != nil {
			return err
		}
		if err := files.Check(); err != nil {
			return err
		}
		if files.ImagePath() != image {
			return errors.New("held broker image spelling changed")
		}
		if hasClient {
			actualSID, actualSHA := client.clientRegistration()
			if actualSID != sid || actualSHA != approvedSHA {
				return errors.New("held client request binding changed")
			}
		}
		return b.check(image)
	}
	if err := check(); err != nil {
		return err
	}
	s, err := b.status()
	if err != nil || !serviceCompleted(s) {
		return errors.New("new service must remain stopped")
	}
	// Each invocation is bounded and stopped-only. The second independently
	// rereads the durable creation fence as the real non-impersonated broker.
	// Neither invocation opens the request pipe or establishes application health.
	for _, mode := range []string{"initialize-store", "verify-store"} {
		if err := check(); err != nil {
			return err
		}
		// SCM STOPPED is not a retained process-exit observation. If its previous
		// dispatcher is still draining, the second start may strictly refuse.
		// Never retry that uncertainty, kill a process, or adopt partial setup.
		if err := b.start(mode); err != nil {
			return err
		}
		waitCtx, cancel := context.WithTimeout(ctx, serviceInitializationLimit)
		err := waitServiceStopped(waitCtx, b, check, poll)
		cancel()
		if err != nil {
			return err
		}
	}
	if sid != "" {
		registration, ok := b.(clientRegistrationService)
		if !ok {
			return errors.New("retained stopped client registration backend required")
		}
		if err := check(); err != nil {
			return err
		}
		s, err := b.status()
		if err != nil || !serviceCompleted(s) {
			return errors.New("client registration requires successfully stopped service")
		}
		if err := check(); err != nil {
			return err
		}
		if err := registration.startClientRegistration(sid, approvedSHA); err != nil {
			return err
		}
		waitCtx, cancel := context.WithTimeout(ctx, serviceInitializationLimit)
		err = waitServiceStopped(waitCtx, b, check, poll)
		cancel()
		if err != nil {
			return err
		}
	}
	if err := check(); err != nil {
		return err
	}
	if complete != nil {
		if sid == "" {
			return errors.New("startup completion requires stopped original client registration")
		}
		return complete(ctx, files, b)
	}
	return nil
}

func serviceCompleted(s serviceState) bool {
	// SCM does not define ProcessId while STOPPED; it is not process-exit proof.
	return s.state == serviceStopped && s.win32Exit == 0 && s.serviceExit == 0 && s.controls == 0
}

func waitServiceStopped(ctx context.Context, b serviceBackend, check func() error, poll func(context.Context) error) error {
	for {
		if err := ctx.Err(); err != nil {
			return err
		}
		if err := check(); err != nil {
			return err
		}
		s, err := b.status()
		if err != nil {
			return err
		}
		if serviceCompleted(s) {
			return check()
		}
		if s.controls != 0 || s.win32Exit != 0 || s.serviceExit != 0 || (s.state != serviceStartPending && s.state != serviceStopPending) {
			return errors.New("broker initialization did not stop successfully without serving")
		}
		if err := poll(ctx); err != nil {
			return err
		}
	}
}
