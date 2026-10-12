package brokerhost

import (
	"bytes"
	"context"
	"crypto/sha256"
	"encoding/hex"
	"errors"
	"io"
	"os"
	"path/filepath"
	"runtime"
	"strings"
	"unsafe"

	"golang.org/x/sys/windows"
)

type executionProfilePin struct {
	handle   windows.Handle
	id       executionProfileID
	physical string
}
type nativeExecutionProfile struct {
	path, physical string
	pins           []executionProfilePin
	closed         bool
	closeErr       error
}

func executionProfileIdentity(h windows.Handle, directory bool) (executionProfileID, error) {
	var id executionProfileID
	var info windows.ByHandleFileInformation
	if err := windows.GetFileInformationByHandle(h, &info); err != nil {
		return id, err
	}
	if info.FileAttributes&windows.FILE_ATTRIBUTE_REPARSE_POINT != 0 || (info.FileAttributes&windows.FILE_ATTRIBUTE_DIRECTORY != 0) != directory || (!directory && info.NumberOfLinks != 1) {
		return id, ErrExecutionChildRefused
	}
	var full struct {
		volume uint64
		file   [16]byte
	}
	if err := windows.GetFileInformationByHandleEx(h, 18, (*byte)(unsafe.Pointer(&full)), uint32(unsafe.Sizeof(full))); err != nil {
		return id, err
	}
	if full.volume == 0 || full.file == ([16]byte{}) {
		return id, ErrExecutionChildRefused
	}
	id = executionProfileID{volume: full.volume, file: full.file, created: uint64(info.CreationTime.HighDateTime)<<32 | uint64(info.CreationTime.LowDateTime)}
	if id.created == 0 {
		return executionProfileID{}, ErrExecutionChildRefused
	}
	return id, nil
}
func captureExecutionColdProfileNative(ctx context.Context) (*ExecutionColdProfile, error) {
	root, err := windows.KnownFolderPath(windows.FOLDERID_ProgramData, 0)
	if err != nil {
		return nil, err
	}
	return captureExecutionProfilePath(ctx, filepath.Join(root, "opencpn", "opencpn.ini"))
}

// This path seam is private and used only for inert native fixtures. The exported
// factory always uses KnownFolder ProgramData/opencpn/opencpn.ini.
func captureExecutionProfilePath(ctx context.Context, path string) (result *ExecutionColdProfile, resultErr error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if err := executionNoImpersonation(); err != nil {
		return nil, err
	}
	b := &nativeExecutionProfile{path: path}
	defer func() {
		if resultErr != nil {
			resultErr = errors.Join(resultErr, b.close())
		}
	}()
	if ctx == nil || ctx.Err() != nil {
		return nil, errors.Join(ErrExecutionChildRefused, contextError(ctx))
	}
	// Resolve and pin physical ancestors root-first before reading original bytes.
	h, err := openStock(filepath.Dir(path), true)
	if err != nil {
		return nil, err
	}
	physical, physicalErr := stockPhysical(h)
	closeErr := windows.CloseHandle(h)
	if physicalErr != nil || closeErr != nil {
		return nil, errors.Join(physicalErr, closeErr)
	}
	parts := strings.Split(strings.TrimRight(physical, `\`), `\`)
	if len(parts) < 4 || !physicalVolume.MatchString(strings.Join(parts[:3], `\`)) {
		return nil, ErrExecutionChildRefused
	}
	for count := 3; count <= len(parts); count++ {
		if err = ctx.Err(); err != nil {
			return nil, err
		}
		name := strings.Join(parts[:count], `\`)
		openName := `\\?\GLOBALROOT` + name
		if count == 3 {
			openName += `\`
		}
		handle, err := openStock(openName, true)
		if err != nil {
			return nil, err
		}
		b.pins = append(b.pins, executionProfilePin{handle: handle, physical: name})
		pin := &b.pins[len(b.pins)-1]
		pin.id, err = executionProfileIdentity(handle, true)
		if err != nil {
			return nil, err
		}
		actual, err := stockPhysical(handle)
		if err != nil || !strings.EqualFold(strings.TrimRight(actual, `\`), name) {
			return nil, errors.Join(ErrExecutionChildRefused, err)
		}
	}
	b.physical = strings.TrimRight(physical, `\`) + `\` + filepath.Base(path)
	original, err := b.readOriginal(ctx)
	if err != nil {
		return nil, err
	}
	original.directory = b.pins[len(b.pins)-1].id
	if err = b.validate(ctx); err != nil {
		return nil, err
	}
	return &ExecutionColdProfile{state: &executionColdProfileState{original: original, backend: b}}, nil
}
func (b *nativeExecutionProfile) readOriginal(ctx context.Context) (result executionProfileOriginal, resultErr error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if err := executionNoImpersonation(); err != nil {
		return result, err
	}
	if ctx == nil || ctx.Err() != nil {
		return result, errors.Join(ErrExecutionChildRefused, contextError(ctx))
	}
	h, err := openStock(b.path, false)
	if err != nil {
		return result, err
	}
	file := os.NewFile(uintptr(h), "native-cold-profile")
	if file == nil {
		return result, errors.Join(ErrExecutionChildRefused, windows.CloseHandle(h))
	}
	defer func() { resultErr = errors.Join(resultErr, file.Close()) }()
	result.file, err = executionProfileIdentity(h, false)
	if err != nil {
		return result, err
	}
	physical, err := stockPhysical(h)
	if err != nil || !strings.EqualFold(physical, b.physical) {
		return result, errors.Join(ErrExecutionChildRefused, err)
	}
	var info windows.ByHandleFileInformation
	if err = windows.GetFileInformationByHandle(h, &info); err != nil {
		return result, err
	}
	size := uint64(info.FileSizeHigh)<<32 | uint64(info.FileSizeLow)
	if size == 0 || size > executionProfileLimit {
		return result, ErrExecutionChildRefused
	}
	result.raw = make([]byte, int(size))
	offset := 0
	for offset < len(result.raw) {
		if err = ctx.Err(); err != nil {
			return result, err
		}
		end := offset + (64 << 10)
		if end > len(result.raw) {
			end = len(result.raw)
		}
		n, readErr := file.Read(result.raw[offset:end])
		offset += n
		if readErr != nil {
			return result, readErr
		}
		if n == 0 {
			return result, io.ErrNoProgress
		}
	}
	var extra [1]byte
	if n, readErr := file.Read(extra[:]); n != 0 || readErr != io.EOF {
		return result, errors.Join(ErrExecutionChildRefused, readErr)
	}
	after, err := executionProfileIdentity(h, false)
	var afterInfo windows.ByHandleFileInformation
	if err != nil || after != result.file || windows.GetFileInformationByHandle(h, &afterInfo) != nil || afterInfo.FileSizeHigh != info.FileSizeHigh || afterInfo.FileSizeLow != info.FileSizeLow || afterInfo.LastWriteTime != info.LastWriteTime {
		return result, errors.Join(ErrExecutionChildRefused, err)
	}
	sum := sha256.Sum256(result.raw)
	result.digest = hex.EncodeToString(sum[:])
	return result, ctx.Err()
}
func (b *nativeExecutionProfile) validate(ctx context.Context) error {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if err := executionNoImpersonation(); err != nil {
		return err
	}
	if b == nil || b.closed || len(b.pins) == 0 || ctx == nil {
		return ErrExecutionChildRefused
	}
	for _, pin := range b.pins {
		if err := ctx.Err(); err != nil {
			return err
		}
		id, err := executionProfileIdentity(pin.handle, true)
		if err != nil || id != pin.id {
			return errors.Join(ErrExecutionChildRefused, err)
		}
		name, err := stockPhysical(pin.handle)
		if err != nil || !strings.EqualFold(strings.TrimRight(name, `\`), pin.physical) {
			return errors.Join(ErrExecutionChildRefused, err)
		}
	}
	// Recheck current DOS resolution of the fixed profile directory. Its original
	// physical ancestry, not a mutable alias or current ACL, is the retained pin.
	h, err := openStock(filepath.Dir(b.path), true)
	if err != nil {
		return err
	}
	id, idErr := executionProfileIdentity(h, true)
	name, nameErr := stockPhysical(h)
	closeErr := windows.CloseHandle(h)
	expected := b.pins[len(b.pins)-1]
	if idErr != nil || nameErr != nil || closeErr != nil || id != expected.id || !strings.EqualFold(strings.TrimRight(name, `\`), expected.physical) {
		return errors.Join(ErrExecutionChildRefused, idErr, nameErr, closeErr)
	}
	return ctx.Err()
}
func (b *nativeExecutionProfile) beforeLaunch(ctx context.Context, expected executionProfileOriginal) error {
	original, err := b.readOriginal(ctx)
	if err != nil {
		return err
	}
	if original.file != expected.file || original.digest != expected.digest || !bytes.Equal(original.raw, expected.raw) {
		return ErrExecutionChildRefused
	}
	return b.validate(ctx)
}
func (b *nativeExecutionProfile) close() error {
	if b == nil || b.closed {
		if b == nil {
			return nil
		}
		return b.closeErr
	}
	b.closed = true
	for n := len(b.pins) - 1; n >= 0; n-- {
		b.closeErr = errors.Join(b.closeErr, windows.CloseHandle(b.pins[n].handle))
	}
	b.pins = nil
	return b.closeErr
}
