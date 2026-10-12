//go:build windows

package brokerpipe

import (
	"context"
	"errors"
	"runtime"
	"time"
	"unsafe"

	"golang.org/x/sys/windows"
)

// Every operation owns its OVERLAPPED/event until completion, including after
// cancellation. There are no detached I/O goroutines or stack-backed buffers
// left with the kernel. Context polling does not alter the handler's budget.
func overlapped(ctx context.Context, h windows.Handle, start func(*windows.Overlapped) error) (uint32, error) {
	if e := ctx.Err(); e != nil {
		return 0, e
	}
	event, e := windows.CreateEvent(nil, 1, 0, nil)
	if e != nil {
		return 0, e
	}
	defer windows.CloseHandle(event)
	ov := windows.Overlapped{HEvent: event}
	e = start(&ov)
	if e == windows.ERROR_PIPE_CONNECTED {
		return 0, nil
	}
	if e != nil && e != windows.ERROR_IO_PENDING {
		return 0, e
	}
	var n uint32
	if e == nil {
		e = windows.GetOverlappedResult(h, &ov, &n, true)
		return n, e
	}
	for {
		if e = ctx.Err(); e != nil {
			_ = windows.CancelIoEx(h, &ov)
			// Cancellation is asynchronous. Drain its completion before releasing any
			// buffer, event or OVERLAPPED; ERROR_NOT_FOUND may mean it just completed.
			_ = windows.GetOverlappedResult(h, &ov, &n, true)
			return 0, e
		}
		state, waitErr := windows.WaitForSingleObject(event, 100)
		if waitErr != nil {
			_ = windows.CancelIoEx(h, &ov)
			_ = windows.GetOverlappedResult(h, &ov, &n, true)
			return 0, waitErr
		}
		if state == windows.WAIT_OBJECT_0 {
			e = windows.GetOverlappedResult(h, &ov, &n, false)
			return n, e
		}
		if state != uint32(windows.WAIT_TIMEOUT) {
			_ = windows.CancelIoEx(h, &ov)
			_ = windows.GetOverlappedResult(h, &ov, &n, true)
			return 0, ErrRefused
		}
	}
}

type pipeIO struct {
	ctx    context.Context
	handle windows.Handle
}

func (p pipeIO) Read(b []byte) (int, error) {
	if len(b) == 0 {
		return 0, nil
	}
	n, e := overlapped(p.ctx, p.handle, func(o *windows.Overlapped) error { return windows.ReadFile(p.handle, b, nil, o) })
	runtime.KeepAlive(b)
	return int(n), e
}
func (p pipeIO) Write(b []byte) (int, error) {
	if len(b) == 0 {
		return 0, nil
	}
	n, e := overlapped(p.ctx, p.handle, func(o *windows.Overlapped) error { return windows.WriteFile(p.handle, b, nil, o) })
	runtime.KeepAlive(b)
	return int(n), e
}

const clientPipeRights = windows.FILE_READ_DATA | windows.FILE_WRITE_DATA | windows.FILE_READ_ATTRIBUTES | windows.FILE_WRITE_ATTRIBUTES | windows.READ_CONTROL | windows.SYNCHRONIZE

func newPipe(sid string) (windows.Handle, error) { return newFixedPipe(PipeName, sid) }
func newExecutionPipe(sid string) (windows.Handle, error) {
	return newFixedPipe(ExecutionPipeName, sid)
}

// Only fixed in-package protocol endpoints call this helper.
func newFixedPipe(pipeName, sid string) (windows.Handle, error) {
	// IU may connect, but cannot create another server instance: no GA/GW nor
	// FILE_APPEND_DATA (= FILE_CREATE_PIPE_INSTANCE) is granted to clients.
	sd, e := windows.SecurityDescriptorFromString("O:" + sid + "D:P(A;;GA;;;" + sid + ")(A;;GA;;;SY)(A;;0x00120183;;;IU)")
	if e != nil {
		return 0, e
	}
	sa := windows.SecurityAttributes{Length: uint32(unsafe.Sizeof(windows.SecurityAttributes{})), SecurityDescriptor: sd}
	name, e := windows.UTF16PtrFromString(pipeName)
	if e != nil {
		return 0, e
	}
	h, e := windows.CreateNamedPipe(name, windows.PIPE_ACCESS_DUPLEX|windows.FILE_FLAG_FIRST_PIPE_INSTANCE|windows.FILE_FLAG_OVERLAPPED, windows.PIPE_REJECT_REMOTE_CLIENTS, 1, maxFrame+4, maxFrame+4, 0, &sa)
	runtime.KeepAlive(sd)
	return h, e
}
func openPipe(ctx context.Context) (windows.Handle, error) { return openFixedPipe(ctx, PipeName) }
func openExecutionPipe(ctx context.Context) (windows.Handle, error) {
	return openFixedPipe(ctx, ExecutionPipeName)
}
func openFixedPipe(ctx context.Context, pipeName string) (windows.Handle, error) {
	if ctx == nil || ctx.Err() != nil {
		return 0, ErrRefused
	}
	name, e := windows.UTF16PtrFromString(pipeName)
	if e != nil {
		return 0, e
	}
	// Identification permits token inspection, not use of the user's authority.
	// Busy/missing service refuses immediately: no unbounded client queue/retry.
	h, e := windows.CreateFile(name, clientPipeRights, 0, nil, windows.OPEN_EXISTING, windows.FILE_FLAG_OVERLAPPED|windows.SECURITY_SQOS_PRESENT|windows.SECURITY_IDENTIFICATION, 0)
	if e != nil {
		return 0, errors.Join(ErrUnavailable, e)
	}
	return h, nil
}

const executionCurrentReadinessBudget = 5 * time.Second

var executionWaitNamedPipe = windows.NewLazySystemDLL("kernel32.dll").NewProc("WaitNamedPipeW")

// Only start-current may await phase-zero handoff from an occupied original
// conversation. Staging and qualification retain immediate busy/missing refusal.
// No bytes have been written; once connected CallExecution verifies the actual
// SCM/process/token server before publication and never reconnects/replays.
func openExecutionCurrentPipe(ctx context.Context) (windows.Handle, error) {
	return openExecutionCurrentReady(ctx, ExecutionPipeName, nil)
}

// Endpoint selection remains private: production supplies only ExecutionPipeName.
// observed is a private inert-fixture notification AFTER real native errors; it
// supplies no readiness/trust decision and production never installs it.
func openExecutionCurrentReady(ctx context.Context, pipeName string, observed func(error)) (windows.Handle, error) {
	if ctx == nil || ctx.Err() != nil {
		return 0, ErrRefused
	}
	budget, cancel := context.WithTimeout(ctx, executionCurrentReadinessBudget)
	defer cancel()
	name, err := windows.UTF16PtrFromString(pipeName)
	if err != nil {
		return 0, err
	}
	h, err := openFixedPipe(budget, pipeName)
	if !errors.Is(err, windows.ERROR_PIPE_BUSY) {
		return h, err
	}
	if observed != nil {
		observed(windows.ERROR_PIPE_BUSY)
	}
	deadline, _ := budget.Deadline()
	for {
		if err := budget.Err(); err != nil {
			return 0, errors.Join(ErrUnavailable, err)
		}
		remaining := time.Until(deadline)
		milliseconds := remaining / time.Millisecond
		if milliseconds < 1 {
			return 0, errors.Join(ErrUnavailable, context.DeadlineExceeded)
		}
		if milliseconds > 100 {
			milliseconds = 100
		}
		available, _, waitErr := executionWaitNamedPipe.Call(uintptr(unsafe.Pointer(name)), uintptr(milliseconds))
		runtime.KeepAlive(name)
		if err := budget.Err(); err != nil {
			return 0, errors.Join(ErrUnavailable, err)
		}
		if available != 0 {
			h, err = openFixedPipe(budget, pipeName)
			if err == nil {
				if e := budget.Err(); e != nil {
					return 0, errors.Join(ErrUnavailable, e, windows.CloseHandle(h))
				}
				return h, nil
			}
			if errors.Is(err, windows.ERROR_PIPE_BUSY) {
				continue
			} // availability was consumed by another opener
		} else {
			err = waitErr
		}
		switch {
		case errors.Is(err, windows.ERROR_SEM_TIMEOUT):
			// Kernel readiness waits are sliced so cancellation is checked <=100ms.
			continue
		case errors.Is(err, windows.ERROR_FILE_NOT_FOUND):
			// Allowed only AFTER actual BUSY: the old FIRST instance may have closed
			// before the fresh one exists. This <=25ms cancellable backoff prevents a
			// spin; elapsed time is NOT readiness evidence. Only successful CreateFile
			// and subsequent native server verification can establish the connection.
			if observed != nil {
				observed(windows.ERROR_FILE_NOT_FOUND)
			}
			timer := time.NewTimer(25 * time.Millisecond)
			select {
			case <-budget.Done():
				timer.Stop()
				return 0, errors.Join(ErrUnavailable, budget.Err())
			case <-timer.C:
			}
		default:
			return 0, errors.Join(ErrUnavailable, err)
		}
	}
}
