//go:build windows

package brokerenroll

import (
	"context"
	"errors"
	"io"
	"runtime"
)

func bindOriginalBootstrapCacheNative(files enrollmentFiles, inputs io.Closer) error {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	f, ok := files.(*nativeEnrollmentFiles)
	source, native := inputs.(*nativeEnrollmentFiles)
	if !ok || !native || f == nil || source == nil || f == source || source.bootstrapCache == nil || f.bootstrapCacheOwner != nil {
		return errors.New("original retained bootstrap cache owner required")
	}
	if err := source.Check(); err != nil {
		return err
	}
	f.bootstrapCache, f.bootstrapCacheOwner = source.bootstrapCache, source
	_, err := originalBootstrapCacheBindingNative(f)
	return err
}

func originalBootstrapCacheBindingNative(f *nativeEnrollmentFiles) (*installedBootstrapCacheBinding, error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if f == nil || f.bootstrapCache == nil || f.bootstrapCacheOwner == nil || f.bootstrapCacheOwner == f || f.bootstrapCacheOwner.bootstrapCache != f.bootstrapCache {
		return nil, errors.New("original bootstrap cache binding missing")
	}
	if err := f.bootstrapCacheOwner.Check(); err != nil {
		return nil, err
	}
	id, err := startupDirectoryIdentity(f.bootstrapCache)
	if err != nil {
		return nil, err
	}
	b := &installedBootstrapCacheBinding{Path: f.bootstrapCache.dos, Directory: id}
	if !validInstalledBootstrapCache(b) {
		return nil, errors.New("canonical original bootstrap cache required")
	}
	return b, nil
}

// Reopen only the cache authenticated by a genuine schema-2 completion owner.
// Schema-1 installations retain ordinary maintenance/operator compatibility but
// cannot discover a cache from its current ACL or create a fresh replacement.
func openInstalledBootstrapCacheNative(ctx context.Context, c *nativeInstalledMaintenanceAuthority) (result *nativeEnrollmentFiles, resultErr error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if c == nil || ctx == nil || ctx.Err() != nil {
		return nil, errors.New("live original installed completion required")
	}
	if err := c.check(ctx); err != nil {
		return nil, err
	}
	if c.record.Schema != 2 || !validInstalledBootstrapCache(c.record.BootstrapCache) {
		return nil, errors.New("original cache binding unavailable; independently proven legacy cache migration required")
	}
	b := c.record.BootstrapCache
	f := &nativeEnrollmentFiles{}
	defer func() {
		if resultErr != nil {
			resultErr = errors.Join(resultErr, f.Close())
		}
	}()
	pin, err := f.holdAncestry(b.Path, true)
	if err != nil {
		return nil, err
	}
	pin.allowAdds = false
	if err := matchInstalledBootstrapCachePinNative(pin, b); err != nil {
		return nil, err
	}
	f.bootstrapCache = pin
	return f, errors.Join(c.check(ctx), f.Check(), ctx.Err())
}

// Component identity check; the caller separately owns genuine completion and
// protected ancestry. Matching a supplied record alone grants no authority.
func matchInstalledBootstrapCachePinNative(pin *enrollmentPin, b *installedBootstrapCacheBinding) error {
	if pin == nil || !validInstalledBootstrapCache(b) || pin.dos != b.Path {
		return errors.New("exact original bootstrap cache path required")
	}
	id, err := startupDirectoryIdentity(pin)
	if err != nil || id != b.Directory {
		return errors.Join(errors.New("original bootstrap cache identity changed"), err)
	}
	return nil
}
