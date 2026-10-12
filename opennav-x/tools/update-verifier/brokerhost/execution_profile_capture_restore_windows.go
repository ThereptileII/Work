package brokerhost

import (
	"context"
	"errors"
	"io"
	"runtime"

	"golang.org/x/sys/windows"
)

func (b *nativeProfileCapture) inspectWritable(ctx context.Context) (profileRestoreSource, error) {
	if b.frozen {
		return nil, ErrExecutionProfileCaptureRefused
	}
	if err := b.validate(ctx, false); err != nil {
		return nil, err
	}
	source, err := captureProfileTreePathMode(ctx, b.fixed, b.checkProcesses, true)
	if err != nil {
		return nil, err
	}
	current := source.(*nativeProfileCapture)
	if len(current.ancestors) != len(b.ancestors) {
		return nil, errors.Join(ErrExecutionProfileCaptureRefused, current.close())
	}
	for i, p := range current.ancestors {
		if p.id != b.ancestors[i].id {
			return nil, errors.Join(ErrExecutionProfileCaptureRefused, current.close())
		}
	}
	return current, nil
}
func (b *nativeProfileCapture) readRestoreFile(ctx context.Context, name string, limit int64) ([]byte, error) {
	if name != "opencpn.ini" && name != "opencpn.log" {
		return nil, ErrExecutionProfileCaptureRefused
	}
	if ctx == nil || b.closed || !b.frozen {
		return nil, ErrExecutionProfileCaptureRefused
	}
	if err := errors.Join(ctx.Err(), b.watch.check()); err != nil {
		return nil, err
	}
	for _, m := range b.members {
		if m.entry.Path != name {
			continue
		}
		if m.entry.Directory || m.entry.Bytes > limit {
			return nil, ErrExecutionProfileRestoreReviewRequired
		}
		raw, err := io.ReadAll(io.NewSectionReader(m.file, 0, m.entry.Bytes+1))
		if err != nil {
			return nil, err
		}
		if int64(len(raw)) != m.entry.Bytes || captureDigest(raw) != m.entry.SHA256 {
			return nil, ErrExecutionProfileCaptureRefused
		}
		if err = errors.Join(ctx.Err(), b.watch.check()); err != nil {
			return nil, err
		}
		return raw, nil
	}
	return nil, ErrExecutionProfileRestoreReviewRequired
}
func profileRestoreTime(t uint64) windows.Filetime {
	return windows.Filetime{LowDateTime: uint32(t), HighDateTime: uint32(t >> 32)}
}
func (b *nativeProfileCapture) applyRestore(ctx context.Context, plan profileRestorePlan, check func() error) error {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if err := executionNoImpersonation(); err != nil {
		return err
	}
	if err := errors.Join(check(), b.validate(ctx, true)); err != nil {
		return err
	}
	// Own writes necessarily notify the watch. Join its pending operation only
	// after closed policy + durable intent, while every CURRENT file still denies
	// all other writers/deleters. Pinned directories deny deletion. Any new child
	// namespace cannot be erased here and the complete final inventory rejects it.
	if err := b.watch.close(true); err != nil {
		return err
	}
	b.watch = nil
	members := map[string]*nativeProfileCaptureMember{}
	for _, m := range b.members {
		members[m.entry.Path] = m
	}
	for _, op := range plan.Files {
		if err := check(); err != nil {
			return err
		}
		m := members[op.Original.Path]
		if m == nil || m.file == nil || m.entry != op.Current || int64(len(op.Bytes)) != op.Original.Bytes || captureDigest(op.Bytes) != op.Original.SHA256 {
			return ErrExecutionProfileCaptureRefused
		}
		// Writes use only original archive-derived bytes and the retained current
		// full-ID RW handle. Never rename/reopen/delete/recreate or replace any ACL
		// (including a SACL we did not read). A wx-replaced INI has its own pinned ID.
		for offset := 0; offset < len(op.Bytes); {
			if err := check(); err != nil {
				return err
			}
			end := offset + (64 << 10)
			if end > len(op.Bytes) {
				end = len(op.Bytes)
			}
			n, err := m.file.WriteAt(op.Bytes[offset:end], int64(offset))
			if err != nil {
				return err
			}
			if n != end-offset {
				return io.ErrShortWrite
			}
			offset = end
		}
		if err := check(); err != nil {
			return err
		}
		if err := m.file.Truncate(op.Original.Bytes); err != nil {
			return err
		}
		if err := m.file.Sync(); err != nil {
			return err
		}
		hash, err := profileFileDigest(ctx, m.file, op.Original.Bytes)
		if err != nil || hash != op.Original.SHA256 {
			return errors.Join(ErrExecutionProfileRestoreUncertain, err)
		}
		if err = check(); err != nil {
			return err
		}
		created, access, written := profileRestoreTime(op.Original.Created), profileRestoreTime(op.Original.LastAccess), profileRestoreTime(op.Original.LastWrite)
		if err = windows.SetFileTime(windows.Handle(m.file.Fd()), &created, &access, &written); err != nil {
			return err
		}
		if err = m.file.Sync(); err != nil {
			return err
		}
	}
	for i := len(plan.Directories) - 1; i >= 0; i-- {
		if err := check(); err != nil {
			return err
		}
		e := plan.Directories[i]
		m := members[e.Path]
		if m == nil || !m.entry.Directory {
			return ErrExecutionProfileCaptureRefused
		}
		created, access, written := profileRestoreTime(e.Created), profileRestoreTime(e.LastAccess), profileRestoreTime(e.LastWrite)
		if err := windows.SetFileTime(windows.Handle(m.file.Fd()), &created, &access, &written); err != nil {
			return err
		}
	}
	// Re-arm before reading any final metadata/content. No untrusted post-check
	// mutation can be silently accepted while this native owner is retained.
	var err error
	b.watch, err = startProfileWatch(b.physical)
	if err != nil {
		return err
	}
	for _, m := range b.members {
		e, err := profileMemberMetadata(m.file, m.entry.Path, m.entry.Directory)
		if err != nil {
			return err
		}
		if !e.Directory {
			e.SHA256, err = profileFileDigest(ctx, m.file, e.Bytes)
			if err != nil {
				return err
			}
		}
		m.entry = e
	}
	if err = b.validate(ctx, true); err != nil {
		return err
	}
	if entries, _ := b.originals(); profileTreeDigest(entries) != plan.OriginalTreeSHA256 {
		return ErrExecutionProfileRestoreUncertain
	}
	return check()
}
