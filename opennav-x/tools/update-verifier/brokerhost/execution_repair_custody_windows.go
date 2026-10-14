//go:build windows

package brokerhost

import (
	"context"
	"path/filepath"
	"strings"

	"golang.org/x/sys/windows"
)

type nativeRepairFrozenProfile struct{ source *nativeProfileCapture }

func (p *nativeRepairFrozenProfile) validateFrozen(ctx context.Context) error {
	if p == nil || p.source == nil || p.source.closed || !p.source.frozen || !p.source.checkProcesses ||
		p.source.watch == nil || p.source.watch.closed || !p.source.watch.pending ||
		p.source.watch.handle == 0 || p.source.watch.event == 0 {
		return errExecutionRepairCustody
	}
	// The actual full-tree validator checks original handles, physical IDs,
	// contents/metadata, watch events/overflow and named navigation processes.
	return p.source.validate(ctx, true)
}

// The handler already owns its original elevated peer, selected source/head,
// stock and armed-marker guards and holds engine.busy. It captures the fixed
// current profile ONCE before this call, keeping its nativeManualRollbackProfile
// owner alive through every journal fact and committed readback. This factory
// takes no serialized coldness, process ID, path, digest or keeper authority.
func captureRepairApplicationCustodyLocked(ctx context.Context, e *engine, sid, generation string, frozen profileCaptureSource) (*executionRepairApplicationCustody, error) {
	profile, ok := frozen.(*nativeProfileCapture)
	if !ok || profile == nil || ctx == nil || ctx.Err() != nil {
		return nil, errExecutionRepairCustody
	}
	root, err := windows.KnownFolderPath(windows.FOLDERID_ProgramData, 0)
	if err != nil || !strings.EqualFold(profile.fixed, filepath.Join(root, "opencpn")) {
		return nil, errExecutionRepairCustody
	}
	return captureRepairApplicationCustodyWith(ctx, e, sid, generation, &nativeRepairFrozenProfile{source: profile})
}
