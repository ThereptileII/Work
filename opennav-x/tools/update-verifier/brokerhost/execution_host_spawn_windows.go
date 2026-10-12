package brokerhost

import (
	"context"
	"errors"
	"runtime"

	"example.com/opennav-update-verifier/brokerpipe"
	"golang.org/x/sys/windows"
)

// BindSpawn uses the untrusted PID solely to acquire the actual native process.
// Keep its exit handle before waiting for the health pipe, so startup failure
// before READY can still be drained and the original profile restored. A process
// already gone before this binding remains uncertain; its serialized PID cannot
// prove original child custody and is never used as restoration authority.
func (c *nativeExecutionHostChild) BindSpawn(ctx context.Context, report brokerpipe.ExecutionObservation) (resultErr error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if c == nil || c.closed || c.backend == nil || c.backend.native == nil || c.backend.process != 0 ||
		ctx == nil || ctx.Err() != nil || report.Status != "reported-complete" || report.ProcessID == 0 || report.ProcessCreation == 0 {
		return errExecutionHostUncertain
	}
	b := c.backend.native
	if b.closed || b.parent == nil || b.image == nil || b.child != nil || b.prepared == 0 {
		return errExecutionHostUncertain
	}
	if err := b.check(ctx); err != nil {
		return err
	}
	child, err := openExecutionProcess(report.ProcessID)
	if err != nil {
		return err
	}
	defer func() { resultErr = errors.Join(resultErr, child.close()) }()
	if child.created != report.ProcessCreation || child.created < b.prepared || child.created <= b.parent.created ||
		child.parent != b.parent.pid || child.pid == b.parent.pid || child.identity != b.parent.identity {
		return errExecutionHostUncertain
	}
	if err = b.image.MatchesProcess(ctx, child.pid, child.created); err != nil {
		return err
	}
	if err = errors.Join(b.check(ctx), child.check()); err != nil {
		return err
	}
	// No mutation rights and no PID reopening after this point. This handle is
	// retained independently of observer authentication/pipe cleanup.
	if err = windows.DuplicateHandle(windows.CurrentProcess(), child.handle, windows.CurrentProcess(), &c.backend.process, windows.SYNCHRONIZE, false, 0); err != nil {
		return err
	}
	c.backend.pid, c.backend.created = child.pid, child.created
	return ctx.Err()
}
