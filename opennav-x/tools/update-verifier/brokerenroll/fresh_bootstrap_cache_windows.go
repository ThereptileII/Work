//go:build windows

package brokerenroll

import (
	"context"
	"errors"
	"io"
	"path/filepath"
	"runtime"
	"strings"

	"golang.org/x/sys/windows"
)

type nativeFreshBootstrapCache struct {
	files                         *nativeEnrollmentFiles
	parent, cache, metadata, root *enrollmentPin
	name                          string
	attempted                     bool
	rootSHA                       string
}

func (n *nativeFreshBootstrapCache) check(ctx context.Context) error {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if n == nil || n.files == nil || n.parent == nil || ctx == nil || ctx.Err() != nil {
		return errors.New("retained fresh-cache parent required")
	}
	if err := n.files.Check(); err != nil {
		return err
	}
	if !n.attempted {
		return enrollmentAbsent(n.parent.physical + `\` + n.name)
	}
	if n.cache == nil || n.metadata == nil || n.root == nil || !validPin(n.rootSHA) {
		return errors.New("fresh cache publication incomplete")
	}
	return startupHashPin(ctx, n.root, n.rootSHA)
}
func (n *nativeFreshBootstrapCache) close() error {
	if n == nil || n.files == nil {
		return nil
	}
	return n.files.Close()
}
func (n *nativeFreshBootstrapCache) seed(ctx context.Context, root []byte) error {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if n == nil || n.attempted || len(root) == 0 || len(root) > 512<<10 {
		return errors.New("single bounded original root publication required")
	}
	if err := n.check(ctx); err != nil {
		return err
	}
	directorySD, err := windows.SecurityDescriptorFromString("O:BAD:P(A;OICI;FA;;;SY)(A;OICI;FA;;;BA)")
	if err != nil {
		return err
	}
	fileSD, err := windows.SecurityDescriptorFromString("O:BAD:P(A;;FA;;;SY)(A;;FA;;;BA)")
	if err != nil {
		return err
	}
	n.attempted = true // A creation failure may still have affected the namespace.
	n.cache, err = n.files.newDirectory(n.parent, n.name, directorySD, "")
	if err != nil {
		return err
	}
	if err = ctx.Err(); err != nil {
		return err
	}
	n.metadata, err = n.files.newDirectory(n.cache, "metadata", directorySD, "")
	if err != nil {
		return err
	}
	if err = ctx.Err(); err != nil {
		return err
	}
	// Reuse the existing exclusive protected writer's flush, native identity,
	// readonly reopening and exact byte-hash readback; no replacement write exists.
	n.root, err = writeCodeUpgradeFileNative(ctx, n.files, n.metadata, "root.json", fileSD, root)
	if err != nil {
		return err
	}
	n.rootSHA = digest(root)
	return n.check(ctx)
}
func openFreshBootstrapCacheNative(ctx context.Context, path string) (result freshBootstrapCacheOwner, resultErr error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if ctx == nil || ctx.Err() != nil || filepath.Clean(path) != path || len(path) < 4 || filepath.Dir(path) == path {
		return nil, errors.New("canonical fresh cache path required")
	}
	name := filepath.Base(path)
	if name == "" || name == "." || name == ".." || strings.ContainsAny(name, `:/\`+"\x00") || strings.HasSuffix(name, ".") || strings.HasSuffix(name, " ") {
		return nil, errors.New("unambiguous fresh cache component required")
	}
	n := &nativeFreshBootstrapCache{files: &nativeEnrollmentFiles{}, name: name}
	defer func() {
		if resultErr != nil {
			resultErr = errors.Join(resultErr, n.close())
		}
	}()
	var err error
	n.parent, err = n.files.holdAncestry(filepath.Dir(path), true)
	if err != nil {
		return nil, err
	}
	n.parent.allowAdds = false
	if err = n.check(ctx); err != nil {
		return nil, err
	}
	return n, nil
}
func openFreshCacheProvisionerNative(ctx context.Context, sha string) (result maintenanceImageSource, resultErr error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if ctx == nil || ctx.Err() != nil || !validPin(sha) {
		return nil, errors.New("independently approved running provisioner required")
	}
	if err := enrollmentOperator(); err != nil {
		return nil, err
	}
	user, err := windows.GetCurrentProcessToken().GetTokenUser()
	if err != nil || user == nil || user.User.Sid == nil {
		return nil, errors.New("actual original preparation account required")
	}
	path, err := maintenanceCurrentImagePath(0)
	if err != nil {
		return nil, err
	}
	n := &nativeMaintenanceImage{ctx: ctx, files: &nativeEnrollmentFiles{}, sha: sha}
	defer func() {
		if resultErr != nil {
			resultErr = errors.Join(resultErr, n.Close())
		}
	}()
	n.pin, err = maintenanceHoldOriginalImage(n.files, path, user.User.Sid.String())
	if err != nil {
		return nil, err
	}
	if err = errors.Join(n.Check(ctx), startupHashPin(ctx, n.pin, sha)); err != nil {
		return nil, err
	}
	if err = validateMaintenanceLoader(clientBaselinePinReader{n.pin}, n.pin.length); err != nil {
		return nil, err
	}
	n.reader = io.NewSectionReader(clientBaselinePinReader{n.pin}, 0, n.pin.length)
	return n, n.Check(ctx)
}
func freshBootstrapCacheNativeOperations() freshBootstrapCacheOperations {
	return freshBootstrapCacheOperations{source: openFreshCacheProvisionerNative, open: openFreshBootstrapCacheNative}
}
