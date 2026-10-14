package main

import (
	"context"
	"errors"
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
