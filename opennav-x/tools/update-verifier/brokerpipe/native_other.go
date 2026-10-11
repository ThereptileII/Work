//go:build !windows

package brokerpipe

import "context"

// No portable mock is substituted for native peer authentication.
func Serve(context.Context, Handler) error          { return ErrUnavailable }
func Call(context.Context, Request) (Result, error) { return Result{}, ErrUnavailable }
