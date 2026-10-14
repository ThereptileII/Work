//go:build windows

package brokerenroll

import (
	"context"
	"golang.org/x/sys/windows"
	"os"
	"path/filepath"
	"testing"
	"unsafe"
)

func TestNativeInstalledCodeRefreshExclusiveAndReusableWithoutReset(t *testing.T) {
	owner, _, cache, _ := startupNativeFixture(t)
	owner.files.bootstrapCache = cache
	ctx := context.Background()
	first, err := openInstalledCodeRefreshNative(ctx, owner.files)
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { first.close() })
	if second, e := openInstalledCodeRefreshNative(ctx, owner.files); e == nil || second != nil {
		if second != nil {
			second.close()
		}
		t.Fatal("concurrent customer refresh admitted")
	}
	originalID := first.pin.id
	if err := first.close(); err != nil {
		t.Fatal(err)
	}
	next, err := openInstalledCodeRefreshNative(ctx, owner.files)
	if err != nil {
		t.Fatal(err)
	}
	defer next.close()
	if next.pin.id != originalID {
		t.Fatal("refresh coordination file recreated")
	}
	if err := next.check(ctx); err != nil {
		t.Fatal(err)
	}
}
func TestNativeInstalledCodeRefreshRefusesForeignLockBytesOrDescriptor(t *testing.T) {
	for _, kind := range []string{"nonzero", "foreign-descriptor", "directory", "reparse"} {
		t.Run(kind, func(t *testing.T) {
			owner, _, cache, sd := startupNativeFixture(t)
			owner.files.bootstrapCache = cache
			ctx := context.Background()
			if kind == "reparse" {
				target := filepath.Join(cache.dos, "inert-refresh-target")
				if err := os.WriteFile(target, nil, 0600); err != nil {
					t.Fatal(err)
				}
				p, _ := windows.UTF16PtrFromString(filepath.Join(cache.dos, installedCodeRefreshLockName))
				q, _ := windows.UTF16PtrFromString(target)
				if err := windows.CreateSymbolicLink(p, q, 0); err != nil {
					t.Fatal("elevated inert reparse fixture unavailable", err)
				}
			} else if kind == "directory" {
				if _, err := owner.files.newDirectory(cache, installedCodeRefreshLockName, sd, owner.clientSID); err != nil {
					t.Fatal(err)
				}
			} else if kind == "foreign-descriptor" {
				ptr, err := windows.UTF16PtrFromString(filepath.Join(cache.dos, installedCodeRefreshLockName))
				if err != nil {
					t.Fatal(err)
				}
				sa := windows.SecurityAttributes{Length: uint32(unsafe.Sizeof(windows.SecurityAttributes{})), SecurityDescriptor: sd}
				h, err := windows.CreateFile(ptr, windows.GENERIC_READ|windows.GENERIC_WRITE, 0, &sa, windows.CREATE_NEW, windows.FILE_FLAG_OPEN_REPARSE_POINT, 0)
				if err != nil {
					t.Fatal(err)
				}
				var info windows.ByHandleFileInformation
				if err = windows.GetFileInformationByHandle(h, &info); err != nil || info.FileSizeHigh != 0 || info.FileSizeLow != 0 {
					windows.CloseHandle(h)
					t.Fatal("foreign-descriptor fixture is not an existing zero-byte file", err)
				}
				if err = windows.CloseHandle(h); err != nil {
					t.Fatal(err)
				}
			} else {
				// Release the creator pin before testing actual lock format/security refusal.
				body := []byte("foreign lock bytes")
				pin, err := startupWriteFile(ctx, owner.files, cache, installedCodeRefreshLockName, body, sd, false)
				if err != nil {
					t.Fatal(err)
				}
				if err = codeReleasePin(owner.files, pin); err != nil {
					t.Fatal(err)
				}
			}
			if n, err := openInstalledCodeRefreshNative(ctx, owner.files); err == nil || n != nil {
				if n != nil {
					n.close()
				}
				t.Fatal("foreign lock adopted", kind)
			}
		})
	}
}
