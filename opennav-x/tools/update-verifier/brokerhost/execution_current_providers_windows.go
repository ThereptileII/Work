package brokerhost

import (
	"context"
	"errors"
)

// Ordinary selected startup is independent of online metadata availability.
// Protected selected history and the real retained launch image remain required;
// registration-only provenance can never become a Later/offline fallback.
func nativeExecutionCurrentProviders() (executionCurrentHostProviders, func() error) {
	var cold *ExecutionColdProfile
	prepared := false
	return executionCurrentHostProviders{
		authorize: func(ctx context.Context, selected *nativeSelectedExecution) error {
			if ctx == nil || selected == nil {
				return errExecutionCurrentRefused
			}
			if err := selected.validate(); err != nil {
				return err
			}
			if cold != nil {
				// After our own ordinary child starts, normal wx writes are allowed.
				// Retain ancestry, not an assertion that the original INI is frozen.
				return cold.Validate(ctx)
			}
			return ctx.Err()
		},
		prepare: func(ctx context.Context, selected *nativeSelectedExecution) error {
			if prepared || ctx == nil || selected == nil {
				return errExecutionCurrentRefused
			}
			prepared = true
			if err := errors.Join(selected.validate(), profileNoNamedProcesses(ctx)); err != nil {
				return err
			}
			var err error
			cold, err = CaptureExecutionColdProfileNative(ctx)
			if err != nil {
				return err
			}
			// The actual launcher independently retains its physical image ancestry
			// and checks the armed marker immediately before create and resume.
			// This gate neither clears that marker nor enables hardware controls.
			return errors.Join(cold.ValidateBeforeLaunch(ctx), profileNoNamedProcesses(ctx), selected.validate(), ctx.Err())
		},
	}, func() error {
		if cold != nil {
			return cold.Close()
		}
		return nil
	}
}
