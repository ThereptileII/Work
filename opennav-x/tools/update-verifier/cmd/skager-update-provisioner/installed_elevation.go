package main

import (
	"context"
	"errors"
	"strconv"
	"strings"
	"time"
)

const installedElevationAction = "uninstall-current"
const installedElevationChildAction = "uninstall-current-elevated"
const installedElevationLimit = 105 * time.Second

// This private command is comparison-only. No command argument supplies an
// executable, user SID, installation target, approval pin or mutation authority.
type installedElevationRequest struct {
	child   bool
	pid     uint32
	created uint64
}
type installedElevationIdentity struct {
	pid      uint32
	created  uint64
	sid      string
	session  uint32
	elevated bool
}
type installedElevationOwner interface {
	check(context.Context) error
	close() error
}
type installedElevationChild interface {
	installedElevationOwner
	wait(context.Context) (uint32, error)
}
type installedElevationSelf struct {
	identity installedElevationIdentity
	owner    installedElevationOwner
}
type installedElevationOperations struct {
	openSelf   func(context.Context) (installedElevationSelf, error)
	openParent func(context.Context, installedElevationRequest, installedElevationIdentity) (installedElevationOwner, error)
	launch     func(context.Context, installedElevationIdentity, []string) (installedElevationChild, error)
}

func parseInstalledElevation(args []string) (installedElevationRequest, bool, error) {
	var q installedElevationRequest
	if len(args) == 0 {
		return q, false, nil
	}
	action := strings.TrimPrefix(args[0], "--maintenance=")
	if args[0] == "--maintenance" && len(args) > 1 {
		action = args[1]
	}
	if action != installedElevationAction && action != installedElevationChildAction {
		return q, false, nil
	}
	refused := errors.New("fixed installed maintenance elevation arguments required")
	if action == installedElevationAction {
		if !(len(args) == 1 && args[0] == "--maintenance="+action || len(args) == 2 && args[0] == "--maintenance" && args[1] == action) {
			return q, true, refused
		}
		return q, true, nil
	}
	q.child = true
	if len(args) != 3 || args[0] != "--maintenance="+installedElevationChildAction {
		return q, true, refused
	}
	pidText, createdText := strings.TrimPrefix(args[1], "--elevation-parent-pid="), strings.TrimPrefix(args[2], "--elevation-parent-created=")
	pid, e1 := strconv.ParseUint(pidText, 10, 32)
	created, e2 := strconv.ParseUint(createdText, 10, 64)
	if e1 != nil || e2 != nil || pid == 0 || created == 0 || args[1] != "--elevation-parent-pid="+strconv.FormatUint(pid, 10) || args[2] != "--elevation-parent-created="+strconv.FormatUint(created, 10) {
		return q, true, refused
	}
	q.pid, q.created = uint32(pid), created
	return q, true, nil
}

// installedElevationDispatch is called before any customer maintenance opener.
// The child callback must synchronously observe its context at every operation;
// its authority still comes only from the actual elevated token and completion.
func installedElevationDispatch(ctx context.Context, args []string, run func(context.Context) error) (bool, error) {
	return installedElevationDispatchWith(ctx, args, run, installedElevationNativeOperations())
}

func installedElevationDispatchWith(ctx context.Context, args []string, run func(context.Context) error, op installedElevationOperations) (handled bool, resultErr error) {
	q, handled, err := parseInstalledElevation(args)
	if !handled || err != nil {
		return handled, err
	}
	if ctx == nil || ctx.Err() != nil || run == nil || op.openSelf == nil || op.openParent == nil || op.launch == nil {
		return true, errors.New("live fixed installed elevation required")
	}
	ctx, cancel := context.WithTimeout(ctx, installedElevationLimit)
	defer cancel()
	self, err := op.openSelf(ctx)
	if self.owner != nil {
		defer func() { resultErr = errors.Join(resultErr, self.owner.close()) }()
	}
	if err != nil || self.owner == nil || self.identity.pid == 0 || self.identity.created == 0 || self.identity.sid == "" || self.identity.session == 0 {
		return true, errors.Join(errors.New("original fixed maintenance process required"), err)
	}
	if err := self.owner.check(ctx); err != nil {
		return true, err
	}
	if !q.child {
		if self.identity.elevated {
			return true, errors.New("customer elevation requires the original unelevated account; use explicit operator maintenance")
		}
		childArgs := []string{"--maintenance=" + installedElevationChildAction, "--elevation-parent-pid=" + strconv.FormatUint(uint64(self.identity.pid), 10), "--elevation-parent-created=" + strconv.FormatUint(self.identity.created, 10)}
		child, err := op.launch(ctx, self.identity, childArgs)
		if child != nil {
			defer func() { resultErr = errors.Join(resultErr, child.close()) }()
		}
		if err != nil || child == nil {
			return true, errors.Join(errors.New("same-account maintenance elevation refused"), err)
		}
		if err := errors.Join(self.owner.check(ctx), child.check(ctx)); err != nil {
			return true, err
		}
		exit, err := child.wait(ctx)
		if err != nil || exit != 0 {
			return true, errors.Join(errors.New("original elevated maintenance did not complete"), err)
		}
		return true, errors.Join(self.owner.check(ctx), child.check(ctx), ctx.Err())
	}
	if !self.identity.elevated || q.pid == self.identity.pid {
		return true, errors.New("original separate elevated maintenance child required")
	}
	parent, err := op.openParent(ctx, q, self.identity)
	if parent != nil {
		defer func() { resultErr = errors.Join(resultErr, parent.close()) }()
	}
	if err != nil || parent == nil {
		return true, errors.Join(errors.New("original same-account maintenance parent required"), err)
	}
	if err := errors.Join(parent.check(ctx), self.owner.check(ctx)); err != nil {
		return true, err
	}
	joined, stop := context.WithCancelCause(ctx)
	done := make(chan struct{})
	go func() {
		defer close(done)
		ticker := time.NewTicker(10 * time.Millisecond)
		defer ticker.Stop()
		for {
			select {
			case <-joined.Done():
				return
			case <-ticker.C:
				if err := parent.check(joined); err != nil {
					stop(err)
					return
				}
			}
		}
	}()
	err = run(joined)
	parentFailure := context.Cause(joined)
	// Join the watcher before closing either original native process/token owner.
	stop(context.Canceled)
	<-done
	return true, errors.Join(err, parentFailure, ctx.Err(), parent.check(ctx), self.owner.check(ctx))
}
