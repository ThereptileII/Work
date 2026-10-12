package brokerhost

import (
	"context"
	"errors"

	"example.com/opennav-update-verifier/brokerpipe"
	"golang.org/x/sys/windows"
)

// Only a newly reopened durable selection can choose this relay. The old
// executor's actual session is already image-bound; its report supplies a PID
// lookup only. The new launcher is independently corroborated against the held
// selected executor image, and must authenticate a fresh start-current session.
func relaySelectedExecutionNative(ctx context.Context, peer *brokerpipe.PeerCapability, selected *nativeSelectedExecution, conversation executionHostConversation) (resultErr error) {
	return relayExecutionNative(ctx, peer, selected, conversation, brokerpipe.RelaySelectedExecutor)
}

// Ordinary shortcut startup checks for an update once. Qualification and recovery
// retain the terminal relay above so completing an update never opens another offer.
func relayStartupExecutionNative(ctx context.Context, peer *brokerpipe.PeerCapability, selected *nativeSelectedExecution, conversation executionHostConversation) error {
	return relayExecutionNative(ctx, peer, selected, conversation, brokerpipe.RelayStartupExecutor)
}

func relayExecutionNative(ctx context.Context, peer *brokerpipe.PeerCapability, selected *nativeSelectedExecution, conversation executionHostConversation, phase brokerpipe.ExecutionPhase) (resultErr error) {
	if ctx == nil || peer == nil || selected == nil || conversation == nil ||
		(phase != brokerpipe.RelaySelectedExecutor && phase != brokerpipe.RelayStartupExecutor) {
		return errExecutionCurrentRefused
	}
	if err := errors.Join(selected.validate(), peer.Validate(ctx)); err != nil {
		return err
	}
	image, err := executionHostExecutor(ctx, selected.generation)
	if err != nil {
		return err
	}
	var born windows.Filetime
	windows.GetSystemTimeAsFileTime(&born)
	birthFloor := uint64(born.HighDateTime)<<32 | uint64(born.LowDateTime)
	if birthFloor == 0 {
		return errExecutionCurrentRefused
	}
	report, err := conversation.ExchangeCommand(ctx, brokerpipe.ExecutionDirective{
		Phase: phase, ApplicationGeneration: selected.selected.generation})
	if err != nil {
		return err
	}
	if report.Status != "reported-complete" || report.ProcessID == 0 || report.ProcessCreation == 0 {
		return errExecutionCurrentUncertain
	}
	process, err := openExecutionProcess(report.ProcessID)
	if err != nil {
		return err
	}
	defer func() { resultErr = errors.Join(resultErr, process.close()) }()
	if process.created != report.ProcessCreation || process.created < birthFloor {
		return errExecutionCurrentUncertain
	}
	if err = peer.WithExecutor(ctx, func(parent brokerpipe.PeerIdentity) error {
		if parent.SID != selected.registration.ClientSID || process.parent != parent.ProcessID || process.pid == parent.ProcessID ||
			process.created <= parent.ProcessCreation || process.identity.sid != parent.SID || process.identity.session != parent.SessionID ||
			process.identity.logon.HighPart != parent.LogonIDHigh || process.identity.logon.LowPart != parent.LogonIDLow {
			return errExecutionCurrentUncertain
		}
		if err := process.check(); err != nil {
			return err
		}
		// Original held full-file/ancestry identity, not the reported image hash.
		if err := image.MatchesProcess(ctx, process.pid, process.created); err != nil {
			return err
		}
		return process.check()
	}); err != nil {
		return err
	}
	return errors.Join(selected.validate(), peer.Validate(ctx), ctx.Err())
}
