//go:build windows

package brokerenroll

import (
	"errors"
	"fmt"
	"runtime"
	"syscall"
	"unsafe"

	"golang.org/x/sys/windows"
)

// Windows hosts CLSID_ShellLink in windows.storage.dll. Resolve its class
// factory only in the Windows system directory, without consulting HKCU/HKCR
// COM registration. Persist only into an in-memory native stream; publication
// exclusively creates and durably writes exact bytes. See Microsoft's documented
// ShellLink InProcServer32 identity:
// https://learn.microsoft.com/en-us/troubleshoot/windows-client/shell-experience/application-shortcuts-show-blank-icons
var startupShellFactory = windows.NewLazySystemDLL("windows.storage.dll").NewProc("DllGetClassObject")
var startupMemoryStream = windows.NewLazySystemDLL("ole32.dll").NewProc("CreateStreamOnHGlobal")
var startupUninitialize = windows.NewLazySystemDLL("ole32.dll").NewProc("CoUninitialize")

type startupCOM struct{ table *[21]uintptr }

func startupCOMCall(object *startupCOM, method int, args ...uintptr) error {
	if object == nil || object.table == nil {
		return errors.New("native shortcut interface unavailable")
	}
	values := append([]uintptr{uintptr(unsafe.Pointer(object))}, args...)
	hr, _, _ := syscall.SyscallN(object.table[method], values...)
	runtime.KeepAlive(object)
	if int32(hr) < 0 {
		return fmt.Errorf("native shortcut method %d failed HRESULT %#x", method, uint32(hr))
	}
	return nil
}
func startupCOMRelease(object *startupCOM) {
	if object != nil {
		_, _, _ = syscall.SyscallN(object.table[2], uintptr(unsafe.Pointer(object)))
	}
}
func startupStreamRewind(stream *startupCOM) error {
	if unsafe.Sizeof(uintptr(0)) == 4 {
		return startupCOMCall(stream, 5, 0, 0, 0, 0)
	}
	return startupCOMCall(stream, 5, 0, 0, 0)
}
func startupLinkBytes(target, arguments, working string) ([]byte, error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	// S_FALSE still requires CoUninitialize. Changed apartment mode refuses.
	if err := windows.CoInitializeEx(0, 2); err != nil && !errors.Is(err, syscall.Errno(1)) {
		return nil, err
	}
	defer startupUninitialize.Call()
	clsid, _ := windows.GUIDFromString("{00021401-0000-0000-C000-000000000046}")
	classIID, _ := windows.GUIDFromString("{00000001-0000-0000-C000-000000000046}")
	linkIID, _ := windows.GUIDFromString("{000214F9-0000-0000-C000-000000000046}")
	persistIID, _ := windows.GUIDFromString("{00000109-0000-0000-C000-000000000046}")
	var factory, link, persist, stream *startupCOM
	if err := startupShellFactory.Find(); err != nil {
		return nil, fmt.Errorf("system ShellLink class factory unavailable: %w", err)
	}
	hr, _, _ := startupShellFactory.Call(uintptr(unsafe.Pointer(&clsid)), uintptr(unsafe.Pointer(&classIID)), uintptr(unsafe.Pointer(&factory)))
	if int32(hr) < 0 || factory == nil {
		return nil, fmt.Errorf("system ShellLink class factory unavailable HRESULT %#x", uint32(hr))
	}
	defer startupCOMRelease(factory)
	if err := startupCOMCall(factory, 3, 0, uintptr(unsafe.Pointer(&linkIID)), uintptr(unsafe.Pointer(&link))); err != nil {
		return nil, err
	}
	defer startupCOMRelease(link)
	for _, field := range []struct {
		method int
		value  string
	}{{20, target}, {11, arguments}, {9, working}, {7, "SKAGER"}} {
		p, err := windows.UTF16PtrFromString(field.value)
		if err != nil {
			return nil, err
		}
		if err := startupCOMCall(link, field.method, uintptr(unsafe.Pointer(p))); err != nil {
			return nil, err
		}
		runtime.KeepAlive(p)
	}
	if err := startupCOMCall(link, 0, uintptr(unsafe.Pointer(&persistIID)), uintptr(unsafe.Pointer(&persist))); err != nil {
		return nil, err
	}
	defer startupCOMRelease(persist)
	hr, _, _ = startupMemoryStream.Call(0, 1, uintptr(unsafe.Pointer(&stream)))
	if int32(hr) < 0 || stream == nil {
		return nil, errors.New("native shortcut memory stream unavailable")
	}
	defer startupCOMRelease(stream)
	if err := startupCOMCall(persist, 6, uintptr(unsafe.Pointer(stream)), 1); err != nil {
		return nil, err
	}
	if err := startupStreamRewind(stream); err != nil {
		return nil, err
	}
	raw := make([]byte, maxStartupShortcutBytes+1)
	var used uint32
	if err := startupCOMCall(stream, 3, uintptr(unsafe.Pointer(&raw[0])), uintptr(len(raw)), uintptr(unsafe.Pointer(&used))); err != nil {
		return nil, err
	}
	if used < 76 || used > maxStartupShortcutBytes {
		return nil, errors.New("native shortcut bytes outside bound")
	}
	return raw[:used], nil
}
