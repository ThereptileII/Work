package main

import (
	"context"
	"encoding/json"
	"errors"
	"io"
	"strconv"
	"strings"
	"time"
)

const codeUpgradeCurrent = "code-upgrade-current"
const codeUpgradeSuccessor = "code-upgrade-successor"
const installedCodeUpgradeLimit = 30 * time.Minute

// These are comparisons, never executable paths, trust data or approval inputs.
// The successor must independently reopen the protected installed authority.
type installedCodeComparisons struct {
	policy, provisioner, trust, completion string
}
type installedCodeSuccessorRequest struct {
	parent installedElevationRequest
	pins   installedCodeComparisons
}

func parseInstalledCodeSuccessor(args []string) (q installedCodeSuccessorRequest, handled bool, err error) {
	for i, arg := range args {
		if arg == "--maintenance="+codeUpgradeSuccessor || arg == "--maintenance" && i+1 < len(args) && args[i+1] == codeUpgradeSuccessor {
			handled = true
		}
	}
	if !handled {
		return q, false, nil
	}
	refused := errors.New("exact private installed successor comparisons required")
	if len(args) != 7 || args[0] != "--maintenance="+codeUpgradeSuccessor {
		return q, true, refused
	}
	pid, e1 := strconv.ParseUint(strings.TrimPrefix(args[1], "--elevation-parent-pid="), 10, 32)
	created, e2 := strconv.ParseUint(strings.TrimPrefix(args[2], "--elevation-parent-created="), 10, 64)
	if e1 != nil || e2 != nil || pid == 0 || created == 0 || args[1] != "--elevation-parent-pid="+strconv.FormatUint(pid, 10) || args[2] != "--elevation-parent-created="+strconv.FormatUint(created, 10) {
		return q, true, refused
	}
	names := []string{"policy", "provisioner", "trust-config", "original-completion"}
	values := make([]string, len(names))
	for i, name := range names {
		prefix := "--" + name + "-sha256="
		values[i] = strings.TrimPrefix(args[3+i], prefix)
		if !strings.HasPrefix(args[3+i], prefix) || !pin(values[i]) {
			return q, true, refused
		}
	}
	q.parent = installedElevationRequest{action: codeUpgradeCurrent, child: true, pid: uint32(pid), created: created}
	q.pins = installedCodeComparisons{values[0], values[1], values[2], values[3]}
	return q, true, nil
}

func installedCodeSuccessorArgs(parent installedElevationIdentity, pins installedCodeComparisons) []string {
	return []string{"--maintenance=" + codeUpgradeSuccessor,
		"--elevation-parent-pid=" + strconv.FormatUint(uint64(parent.pid), 10),
		"--elevation-parent-created=" + strconv.FormatUint(parent.created, 10),
		"--policy-sha256=" + pins.policy, "--provisioner-sha256=" + pins.provisioner,
		"--trust-config-sha256=" + pins.trust, "--original-completion-sha256=" + pins.completion}
}

type installedCodeSuccessorOperations struct {
	openSelf   func(context.Context) (installedElevationSelf, error)
	openParent func(context.Context, installedElevationRequest, installedElevationIdentity) (installedElevationOwner, error)
	run        func(context.Context, installedCodeComparisons) error
}

func installedCodeSuccessorDispatch(ctx context.Context, args []string) (bool, error) {
	return installedCodeSuccessorDispatchWith(ctx, args, installedCodeSuccessorNativeOperations())
}

func installedCodeSuccessorDispatchWith(ctx context.Context, args []string, op installedCodeSuccessorOperations) (handled bool, resultErr error) {
	q, handled, err := parseInstalledCodeSuccessor(args)
	if !handled || err != nil {
		return handled, err
	}
	if ctx == nil || ctx.Err() != nil || op.openSelf == nil || op.openParent == nil || op.run == nil {
		return true, errors.New("live native successor handoff required")
	}
	ctx, cancel := context.WithTimeout(ctx, installedCodeUpgradeLimit)
	defer cancel()
	self, err := op.openSelf(ctx)
	if self.owner != nil {
		defer func() { resultErr = errors.Join(resultErr, self.owner.close()) }()
	}
	if err != nil || self.owner == nil || !self.identity.elevated || self.identity.pid == 0 || self.identity.created == 0 || self.identity.sid == "" || self.identity.session == 0 || q.parent.pid == self.identity.pid {
		return true, errors.Join(errors.New("original separate elevated successor required"), err)
	}
	parent, err := op.openParent(ctx, q.parent, self.identity)
	if parent != nil {
		defer func() { resultErr = errors.Join(resultErr, parent.close()) }()
	}
	if err != nil || parent == nil {
		return true, errors.Join(errors.New("original elevated fixed helper parent required"), err)
	}
	if err := errors.Join(self.owner.check(ctx), parent.check(ctx)); err != nil {
		return true, err
	}
	return true, runWithInstalledParent(ctx, parent, self.owner, func(ctx context.Context) error { return op.run(ctx, q.pins) })
}

type installedCodeSnapshot struct {
	path, directory string
	pins            installedCodeComparisons
}
type installedCodeLaunchOwner interface {
	installedElevationOwner
	snapshot(context.Context) (installedCodeSnapshot, error)
}
type installedCodeAdmissionOwner interface {
	installedCodeLaunchOwner
	consume(context.Context) (installedCodeLaunchOwner, error)
}
type installedCodeCustomerOperations struct {
	acquire func(context.Context) (installedCodeAdmissionOwner, error)
	launch  func(context.Context, installedCodeLaunchOwner, installedCodeSnapshot) (installedElevationChild, error)
	// Called only after a launch returned an original process whose exit is not
	// proven. It transfers custody, and must preserve the admitted image/cache
	// until that exact process exits; it never kills or retries the child.
	retainUntilExit      func(installedElevationChild, installedCodeLaunchOwner) error
	current, unavailable error
}

func runCustomerCodeUpgradeWith(ctx context.Context, output io.Writer, op installedCodeCustomerOperations) (resultErr error) {
	if ctx == nil || ctx.Err() != nil || output == nil || op.acquire == nil || op.launch == nil || op.retainUntilExit == nil {
		return errors.New("live native installed code admission required")
	}
	state := ""
	owner, err := op.acquire(ctx)
	if owner == nil {
		if op.current != nil && errors.Is(err, op.current) {
			state = "broker-code-current"
		}
		if op.unavailable != nil && errors.Is(err, op.unavailable) {
			state = "broker-code-update-unavailable"
		}
		if state == "" || ctx.Err() != nil {
			return errors.Join(errors.New("installed code admission refused"), err, ctx.Err())
		}
		return json.NewEncoder(output).Encode(struct {
			State string `json:"state"`
		}{state})
	}
	defer func() { resultErr = errors.Join(resultErr, owner.close()) }()
	if err != nil {
		return err
	}
	if err := owner.check(ctx); err != nil {
		return err
	}
	before, err := owner.snapshot(ctx)
	if err != nil {
		return err
	}
	launchOwner, err := owner.consume(ctx)
	var child installedElevationChild
	exited := false
	defer func() {
		if child != nil && !exited {
			resultErr = errors.Join(resultErr, op.retainUntilExit(child, launchOwner))
			return
		}
		if child != nil {
			resultErr = errors.Join(resultErr, child.close())
		}
		if launchOwner != nil {
			resultErr = errors.Join(resultErr, launchOwner.close())
		}
	}()
	if err != nil || launchOwner == nil {
		return errors.Join(errors.New("installed code launch admission refused"), err)
	}
	if err := launchOwner.check(ctx); err != nil {
		return err
	}
	after, err := launchOwner.snapshot(ctx)
	if err != nil || before != after || !pin(after.pins.policy) || !pin(after.pins.provisioner) || !pin(after.pins.trust) || !pin(after.pins.completion) {
		return errors.Join(errors.New("installed code launch comparisons changed"), err)
	}
	child, err = op.launch(ctx, launchOwner, after)
	if err != nil || child == nil {
		return errors.Join(errors.New("original admitted successor launch refused"), err)
	}
	if err := errors.Join(launchOwner.check(ctx), child.check(ctx)); err != nil {
		return err
	}
	exit, err := child.wait(ctx)
	if err == nil {
		exited = true
	}
	if err != nil || exit != 0 {
		return errors.Join(errors.New("original admitted successor did not complete"), err)
	}
	if err := errors.Join(launchOwner.check(ctx), child.check(ctx), ctx.Err()); err != nil {
		return err
	}
	// Emit into the caller's private buffer only. Outer native owners and all
	// deferred cleanup must pass before that buffer reaches customer stdout.
	return json.NewEncoder(output).Encode(struct {
		State string `json:"state"`
	}{"broker-code-upgraded"})
}
