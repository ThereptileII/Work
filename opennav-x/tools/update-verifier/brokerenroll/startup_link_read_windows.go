//go:build windows

package brokerenroll

import (
	"bytes"
	"encoding/binary"
	"errors"
	"runtime"
	"syscall"
	"unicode/utf16"
	"unsafe"

	"golang.org/x/sys/windows"
)

var errStartupLinkFields = errors.New("bounded original native shortcut fields required")

// startupLinkFields decodes only the caller's bounded original bytes through
// the same system-DLL ShellLink factory as publication. No COM registration,
// IPersistFile path load, Resolve, target launch or caller-path open is used.
// Fields are comparison data; neither successful decoding nor a target string
// establishes ownership of the shortcut or permission to remove its target.
func startupLinkFields(raw []byte) (target, arguments, working string, err error) {
	defer func() {
		if err != nil {
			target, arguments, working = "", "", ""
			err = errStartupLinkFields
		}
	}()
	// HeaderSize is 0x4c, the CLSID is fixed, and ExtraData ends in its
	// mandatory four-byte TerminalBlock (<4). Reject incomplete serialization.
	class := []byte{0x01, 0x14, 0x02, 0, 0, 0, 0, 0, 0xc0, 0, 0, 0, 0, 0, 0, 0x46}
	if len(raw) < 80 || len(raw) > maxStartupShortcutBytes || binary.LittleEndian.Uint32(raw[:4]) != 76 ||
		!bytes.Equal(raw[4:20], class) || binary.LittleEndian.Uint32(raw[len(raw)-4:]) >= 4 {
		return "", "", "", errStartupLinkFields
	}
	raw = bytes.Clone(raw)
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if err = windows.CoInitializeEx(0, 2); err != nil && !errors.Is(err, syscall.Errno(1)) {
		return "", "", "", err
	}
	defer startupUninitialize.Call()
	clsid, _ := windows.GUIDFromString("{00021401-0000-0000-C000-000000000046}")
	classIID, _ := windows.GUIDFromString("{00000001-0000-0000-C000-000000000046}")
	linkIID, _ := windows.GUIDFromString("{000214F9-0000-0000-C000-000000000046}")
	persistIID, _ := windows.GUIDFromString("{00000109-0000-0000-C000-000000000046}")
	var factory, link, persist, stream *startupCOM
	if err = startupShellFactory.Find(); err != nil {
		return "", "", "", err
	}
	hr, _, _ := startupShellFactory.Call(uintptr(unsafe.Pointer(&clsid)), uintptr(unsafe.Pointer(&classIID)), uintptr(unsafe.Pointer(&factory)))
	if int32(hr) < 0 || factory == nil {
		return "", "", "", errStartupLinkFields
	}
	defer startupCOMRelease(factory)
	if err = startupCOMCall(factory, 3, 0, uintptr(unsafe.Pointer(&linkIID)), uintptr(unsafe.Pointer(&link))); err != nil {
		return "", "", "", err
	}
	defer startupCOMRelease(link)
	if err = startupCOMCall(link, 0, uintptr(unsafe.Pointer(&persistIID)), uintptr(unsafe.Pointer(&persist))); err != nil {
		return "", "", "", err
	}
	defer startupCOMRelease(persist)
	hr, _, _ = startupMemoryStream.Call(0, 1, uintptr(unsafe.Pointer(&stream)))
	if int32(hr) < 0 || stream == nil {
		return "", "", "", errStartupLinkFields
	}
	defer startupCOMRelease(stream)
	var used uint32
	err = startupCOMCall(stream, 4, uintptr(unsafe.Pointer(&raw[0])), uintptr(len(raw)), uintptr(unsafe.Pointer(&used)))
	runtime.KeepAlive(raw)
	if err != nil || used != uint32(len(raw)) {
		return "", "", "", errStartupLinkFields
	}
	if err = startupStreamRewind(stream); err != nil {
		return "", "", "", err
	}
	if err = startupCOMCall(persist, 5, uintptr(unsafe.Pointer(stream))); err != nil {
		return "", "", "", err
	}
	// A loaded prefix cannot authenticate ignored trailing data. Load must leave
	// the stream at the exact end of the original serialized object.
	var tail [1]byte
	used = 0
	if err = startupCOMCall(stream, 3, uintptr(unsafe.Pointer(&tail[0])), 1, uintptr(unsafe.Pointer(&used))); err != nil || used != 0 {
		return "", "", "", errStartupLinkFields
	}
	// GetPath documents a MAX_PATH result even for a larger buffer. Reject a
	// full result instead of accepting a possibly truncated ownership prefix.
	if target, err = startupReadLinkField(link, 3, windows.MAX_PATH); err != nil || target == "" {
		return "", "", "", errStartupLinkFields
	}
	// Arguments and working directory getters can silently truncate. One spare
	// unit makes strings beyond this 32K-unit policy unambiguously unacceptable.
	if arguments, err = startupReadLinkField(link, 10, 32769); err != nil {
		return "", "", "", err
	}
	if working, err = startupReadLinkField(link, 8, 32769); err != nil {
		return "", "", "", err
	}
	return target, arguments, working, nil
}

func startupReadLinkField(link *startupCOM, method, limit int) (string, error) {
	buffer := make([]uint16, limit)
	for i := range buffer {
		buffer[i] = 0xffff // An untouched buffer is not a successful empty field.
	}
	args := []uintptr{uintptr(unsafe.Pointer(&buffer[0])), uintptr(len(buffer))}
	if method == 3 {
		args = append(args, 0, 4) // No FIND_DATA, SLGP_RAWPATH; never Resolve.
	}
	if err := startupCOMCall(link, method, args...); err != nil {
		return "", err
	}
	runtime.KeepAlive(buffer)
	return startupLinkUTF16(buffer)
}

func startupLinkUTF16(buffer []uint16) (string, error) {
	end := -1
	for i, value := range buffer {
		if value == 0 {
			end = i
			break
		}
	}
	if end < 0 || end == len(buffer)-1 {
		return "", errStartupLinkFields // Missing terminator or possible truncation.
	}
	text := buffer[:end]
	for i := 0; i < len(text); i++ {
		switch {
		case text[i] >= 0xd800 && text[i] <= 0xdbff:
			if i+1 >= len(text) || text[i+1] < 0xdc00 || text[i+1] > 0xdfff {
				return "", errStartupLinkFields
			}
			i++
		case text[i] >= 0xdc00 && text[i] <= 0xdfff:
			return "", errStartupLinkFields
		}
	}
	return string(utf16.Decode(text)), nil
}
