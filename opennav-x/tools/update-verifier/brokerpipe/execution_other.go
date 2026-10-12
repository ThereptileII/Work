//go:build !windows

package brokerpipe

import "context"

// No portable mock replaces native SCM/session/image authentication.
func ServeExecution(context.Context, ExecutionHandler) error { return ErrUnavailable }
func CallExecution(context.Context, ExecutionRequest, ExecutionClientHandler) error {
	return ErrUnavailable
}
