package brokerenroll

import (
	"context"
	"errors"

	"example.com/opennav-update-verifier/brokerregistration"
)

// ErrStartupUncertain means shortcut publication or normal service activation
// may have taken effect. Original shortcut evidence is retained in the fixed
// current-user recovery folder. Never automatically retry, delete, adopt or
// reset trust in response; this is not rollback or application health evidence.
var ErrStartupUncertain = errors.New("protected broker startup publication or activation uncertain")

// RunAndActivateStartup is an explicit operator installation step. Unlike Run,
// it requires approved client registration for the ACTUAL current user, publishes
// that user's three fixed SKAGER shortcuts to the same protected baseline launcher,
// preserves an existing normal shortcut outside the SKAGER menu, and explicitly
// activates the original service only after stopped registration and file checks.
// Existing Legacy/Safe destinations refuse publication; unrelated links remain
// untouched. Three non-replacing moves are not atomic; partial recovery evidence
// never authorizes automatic retry or rollback. The installer must join
// its distinct protected-layout rollback contract; this API does not run it or
// grant selection, commissioning, healthy startup or ordinary application launch.
func RunAndActivateStartup(ctx context.Context, req Request) error {
	req.ClientRegistrationRequest = append([]byte(nil), req.ClientRegistrationRequest...)
	if err := startupCurrentClientNative(ctx, req.ClientRegistrationRequest, req.ApprovedClientRequestSHA256); err != nil {
		return err
	}
	op := nativeOperations()
	completed := false
	op.initializeStartup = func(ctx context.Context, files enrollmentFiles, client *brokerregistration.Request, baseline ApprovedClientBaseline) error {
		err := initializeStartupNative(ctx, files, client, baseline)
		completed = err == nil
		return err
	}
	err := run(ctx, req, op)
	if err != nil && completed {
		return errors.Join(ErrStartupUncertain, err)
	}
	return err
}

type startupPublication interface {
	Check(context.Context) error
	Close() error
}

type startupService interface{ activateNormal() error }

type startupPublisher func(context.Context, enrollmentFiles, *brokerregistration.Request, ApprovedClientBaseline) (startupPublication, error)

func completeStartup(ctx context.Context, files enrollmentFiles, b serviceBackend, client *brokerregistration.Request, baseline ApprovedClientBaseline, publish startupPublisher, poll func(context.Context) error) (result error) {
	if ctx == nil || ctx.Err() != nil || files == nil || b == nil || client == nil || baseline == nil || publish == nil || poll == nil {
		return errors.New("live original registered startup owners required")
	}
	activated := false
	defer func() {
		if result != nil && activated {
			result = errors.Join(ErrStartupUncertain, result)
		}
	}()
	check := func() error {
		if err := ctx.Err(); err != nil {
			return err
		}
		if err := files.Check(); err != nil {
			return err
		}
		publisher, ok := files.(clientRequestFiles)
		if !ok {
			return errors.New("original client publication owner required")
		}
		sid, sha := publisher.clientRegistration()
		if sid != client.ClientSID || sha != client.RequestSHA256 {
			return errors.New("original client publication binding differs")
		}
		if err := baseline.Validate(); err != nil {
			return err
		}
		return b.check(files.ImagePath())
	}
	if err := check(); err != nil {
		return err
	}
	state, err := b.status()
	if err != nil || !serviceCompleted(state) {
		return errors.Join(errors.New("startup requires completed stopped original registration"), err)
	}
	normal, ok := b.(startupService)
	if !ok {
		return errors.New("original native service activation capability required")
	}
	publication, err := publish(ctx, files, client, baseline)
	if publication != nil {
		defer func() {
			result = errors.Join(result, publication.Close())
			if result != nil {
				result = errors.Join(ErrStartupUncertain, result)
			}
		}()
	}
	if err != nil || publication == nil {
		return errors.Join(ErrStartupUncertain, errors.New("normal shortcut publication refused"), err)
	}
	if err := check(); err != nil {
		return errors.Join(ErrStartupUncertain, err)
	}
	if err := publication.Check(ctx); err != nil {
		return errors.Join(ErrStartupUncertain, err)
	}
	producer, ok := publication.(installedCompletionPublisher)
	if !ok {
		return errors.Join(ErrStartupUncertain, errors.New("original installed maintenance completion publisher required"))
	}
	completion, err := producer.prepareInstalledCompletion(ctx, files, client, baseline)
	if completion != nil {
		defer func() {
			result = errors.Join(result, completion.Close())
			if result != nil {
				result = errors.Join(ErrStartupUncertain, result)
			}
		}()
	}
	if err != nil || completion == nil {
		return errors.Join(ErrStartupUncertain, errors.New("original installed maintenance reservation refused"), err)
	}
	state, err = b.status()
	if err != nil || !serviceCompleted(state) {
		return errors.Join(ErrStartupUncertain, errors.New("original service changed before explicit activation"), err)
	}
	// A failed activation can already have changed durable service configuration.
	activated = true
	if err := normal.activateNormal(); err != nil {
		return err
	}
	waitCtx, cancel := context.WithTimeout(ctx, serviceInitializationLimit)
	defer cancel()
	for {
		if err := check(); err != nil {
			return err
		}
		if err := waitCtx.Err(); err != nil {
			return err
		}
		state, err = b.status()
		if err != nil {
			return err
		}
		if state.state == 4 && state.processID != 0 && state.win32Exit == 0 && state.serviceExit == 0 {
			// Rehash at the activation boundary, not on every 50ms SCM poll.
			// This is a bounded observation, not continuous loader exclusion.
			if err := publication.Check(waitCtx); err != nil {
				return err
			}
			if err := check(); err != nil {
				return err
			}
			pid := state.processID
			return completion.Complete(waitCtx, func(ctx context.Context) error {
				if err := ctx.Err(); err != nil {
					return err
				}
				if err := check(); err != nil {
					return err
				}
				current, err := b.status()
				if err != nil || current.state != 4 || current.processID != pid || current.win32Exit != 0 || current.serviceExit != 0 {
					return errors.Join(errors.New("original running service changed before maintenance completion"), err)
				}
				return nil
			})
		}
		if state.state != serviceStartPending || state.controls != 0 || state.win32Exit != 0 || state.serviceExit != 0 {
			return errors.New("explicitly activated broker did not enter Running")
		}
		if err := poll(waitCtx); err != nil {
			return err
		}
	}
}
