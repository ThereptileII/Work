package brokerhost

import (
	"context"
	"errors"
	"runtime"

	"example.com/opennav-update-verifier/brokerstore"
	"golang.org/x/sys/windows"
)

// The generation identity was corroborated at birth against the retained full
// generation. Its value records provenance only; it grants no reopening or cold
// profile authority. Lifetime observations use exclusively the original process
// and token handles, without retaining request/session/generation objects.
type nativeExecutionApplicationOwner struct {
	process  *executionProcess
	identity brokerstore.GenerationApplicationIdentity
	closed   bool
	closeErr error
}

func (o *nativeExecutionApplicationOwner) clientSID() string  { return o.identity.ClientSID }
func (o *nativeExecutionApplicationOwner) generation() string { return o.identity.Generation }

func (o *nativeExecutionApplicationOwner) observe(ctx context.Context) (executionApplicationState, error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if o == nil || o.closed || o.process == nil || o.process.handle == 0 || o.process.token == 0 || ctx == nil || ctx.Err() != nil {
		return executionApplicationUnknown, errors.Join(errExecutionApplicationOwner, contextError(ctx))
	}
	if err := executionNoImpersonation(); err != nil {
		return executionApplicationUnknown, err
	}
	p := o.process
	if p.identity.sid != o.identity.ClientSID {
		return executionApplicationUnknown, errExecutionApplicationOwner
	}
	status, err := windows.WaitForSingleObject(p.handle, 0)
	if err != nil {
		return executionApplicationUnknown, err
	}
	switch status {
	case uint32(windows.WAIT_TIMEOUT):
		if err := p.check(); err != nil {
			return executionApplicationUnknown, err
		}
		if err := ctx.Err(); err != nil {
			return executionApplicationUnknown, err
		}
		return executionApplicationLive, nil
	case windows.WAIT_OBJECT_0:
		var created, exited, kernel, user windows.Filetime
		if err := windows.GetProcessTimes(p.handle, &created, &exited, &kernel, &user); err != nil || uint64(created.HighDateTime)<<32|uint64(created.LowDateTime) != p.created || exited == (windows.Filetime{}) {
			return executionApplicationUnknown, errors.Join(errExecutionApplicationOwner, err)
		}
		// The signalled original process handle is the exit proof. A process can
		// legitimately return STILL_ACTIVE's numeric value as its actual exit code.
		var exitCode uint32
		if err := windows.GetExitCodeProcess(p.handle, &exitCode); err != nil {
			return executionApplicationUnknown, err
		}
		identity, err := executionToken(p.token)
		if err != nil || identity != p.identity {
			return executionApplicationUnknown, errors.Join(errExecutionApplicationOwner, err)
		}
		if err := ctx.Err(); err != nil {
			return executionApplicationUnknown, err
		}
		return executionApplicationExited, nil
	default:
		return executionApplicationUnknown, errExecutionApplicationOwner
	}
}

// Only release query custody. Neither this nor observation sends process,
// profile, navigation, or physical commands to the ordinary application.
func (o *nativeExecutionApplicationOwner) Close() error {
	if o == nil {
		return nil
	}
	if o.closed {
		return o.closeErr
	}
	o.closed = true
	o.closeErr = o.process.close()
	return o.closeErr
}
