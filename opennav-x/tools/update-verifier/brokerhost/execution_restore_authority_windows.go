package brokerhost

import (
	"context"
	"encoding/json"
	"errors"

	"example.com/opennav-update-verifier/brokerstore"
	"golang.org/x/sys/windows"
)

// This owner is minted solely from the native observer's retained duplicate of
// a natively corroborated ORIGINAL child. No PID/report/digest can construct it. A
// second SYNCHRONIZE handle survives observer Close through actual restoration.
type executionHostClosedChild struct {
	host                    *nativeExecutionHost
	process                 windows.Handle
	pid                     uint32
	created                 uint64
	binding                 profileRestoreBinding
	intentAttempted, closed bool
	closeErr                error
}

func (h *nativeExecutionHost) prepareRestore(ctx context.Context, child executionHostChild) (resultErr error) {
	provider, ok := h.profile.(*nativeExecutionProfileProvider)
	native, childOK := child.(*nativeExecutionHostChild)
	if !ok || provider == nil || provider.ExecutionProfileCapture == nil || provider.restoration != nil ||
		!childOK || native == nil || native.closed || native.backend == nil || native.backend.process == 0 ||
		native.backend.pid == 0 || native.backend.created == 0 || h.begin.PreviousGeneration == "" {
		return errExecutionHostUncertain
	}
	if err := h.validateRecovery(ctx); err != nil {
		return err
	}
	status, err := windows.WaitForSingleObject(native.backend.process, 0)
	if err != nil || status != windows.WAIT_OBJECT_0 {
		return errors.Join(errExecutionHostUncertain, err)
	}
	snapshot, err := provider.ExecutionProfileCapture.Snapshot(ctx)
	if err != nil || snapshot.Archive.ClientSID != h.begin.ClientSID || snapshot.Archive.ExecutionID != h.begin.ExecutionID ||
		snapshot.Archive.Generation != h.begin.Generation || snapshot.Archive.CompletionSHA256 != h.begin.CompletionSHA256 {
		return errors.Join(errExecutionHostUncertain, err)
	}
	authority := &executionHostClosedChild{host: h, pid: native.backend.pid, created: native.backend.created,
		binding: profileRestoreBinding{h.begin.ClientSID, h.begin.ExecutionID, h.begin.Generation, h.begin.CompletionSHA256, snapshot.ManifestSHA256}}
	if err = windows.DuplicateHandle(windows.CurrentProcess(), native.backend.process, windows.CurrentProcess(), &authority.process, windows.SYNCHRONIZE, false, 0); err != nil {
		return err
	}
	if err = authority.validateProfileRestore(ctx, authority.binding); err != nil {
		return errors.Join(err, authority.Close())
	}
	provider.restoration = authority
	return nil
}

func (a *executionHostClosedChild) validateProfileRestore(ctx context.Context, binding profileRestoreBinding) error {
	if a == nil || a.closed || a.host == nil || a.process == 0 || a.pid == 0 || a.created == 0 ||
		ctx == nil || ctx.Err() != nil || binding != a.binding {
		return errExecutionHostUncertain
	}
	if err := a.host.validateState(ctx, false, false); err != nil {
		return err
	}
	status, err := windows.WaitForSingleObject(a.process, 0)
	if err != nil || status != windows.WAIT_OBJECT_0 {
		return errors.Join(errExecutionHostUncertain, err)
	}
	return ctx.Err()
}

func (a *executionHostClosedChild) recordProfileRestoreIntent(ctx context.Context, intent profileRestoreIntent) error {
	if a == nil || a.intentAttempted || intent.Schema != 1 || intent.Purpose != "automatic-original-profile-restore-v1" ||
		!hashPattern.MatchString(intent.CurrentManifestSHA256) || !hashPattern.MatchString(intent.PlanSHA256) {
		return errExecutionHostUncertain
	}
	if err := a.validateProfileRestore(ctx, intent.Binding); err != nil {
		return err
	}
	a.intentAttempted = true // burn before canonical encoding or durable append
	raw, err := json.Marshal(intent)
	if err != nil {
		return err
	}
	step, err := json.Marshal(executionSelectionStep{1, "broker-selected-step-v1", a.binding.ExecutionID,
		"profile-restore-intent", a.binding.Generation, executionOriginalDigest(raw)})
	if err != nil {
		return err
	}
	// The protected original journal records intent before the first profile
	// effect. The digest binds the entire closed plan/observed-current identity;
	// neither that fact nor its copied JSON grants native overwrite authority.
	return a.host.journal.AppendFact(ctx, brokerstore.ExecutionRestoration, step)
}

func (a *executionHostClosedChild) Close() error {
	if a == nil || a.closed {
		if a == nil {
			return nil
		}
		return a.closeErr
	}
	a.closed = true
	if a.process != 0 {
		a.closeErr = windows.CloseHandle(a.process)
		a.process = 0
	}
	return a.closeErr
}
