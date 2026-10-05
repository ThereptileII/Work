package launcher

import (
	"bytes"
	"context"
	"crypto/sha256"
	"encoding/hex"
	"errors"
	"io"
	"os"
	"os/exec"
	"path"
	"strings"
	"syscall"
	"time"
	"unsafe"

	"golang.org/x/sys/windows"
)

type nativePlatform struct{}

// ReportFailure supplies fixed, nontechnical guidance when a GUI-subsystem
// launcher has no visible stderr. No server text, paths or credentials appear.
func ReportFailure() {
	message, _ := windows.UTF16PtrFromString("SKAGER could not complete startup.\n\nAn update may need recovery. Open SKAGER Maintenance diagnostics before trying again.\n\nYou can try the Legacy or Safe Mode shortcut while checking the problem.")
	title, _ := windows.UTF16PtrFromString("SKAGER startup")
	procedure := windows.NewLazySystemDLL("user32.dll").NewProc("MessageBoxW")
	_, _, _ = procedure.Call(0, uintptr(unsafe.Pointer(message)), uintptr(unsafe.Pointer(title)), 0x10)
}

func newPlatform() (platform, error)               { return nativePlatform{}, nil }
func (nativePlatform) executable() (string, error) { return os.Executable() }

func plainNativePath(name string) error {
	canonical, err := localPath(name)
	if err != nil {
		return err
	}
	for current := canonical; ; current = path.Dir(current) {
		if len(current) == 2 {
			current += "/"
		}
		pointer, err := windows.UTF16PtrFromString(current)
		if err != nil {
			return err
		}
		attributes, err := windows.GetFileAttributes(pointer)
		if err != nil {
			return err
		}
		if attributes&windows.FILE_ATTRIBUTE_REPARSE_POINT != 0 {
			return errors.New("reparse path prohibited")
		}
		if len(current) == 3 {
			break
		}
	}
	return nil
}
func openNativeRead(name string) (*os.File, error) {
	if err := plainNativePath(name); err != nil {
		return nil, err
	}
	pointer, err := windows.UTF16PtrFromString(name)
	if err != nil {
		return nil, err
	}
	handle, err := windows.CreateFile(pointer, windows.GENERIC_READ, windows.FILE_SHARE_READ, nil, windows.OPEN_EXISTING, windows.FILE_FLAG_OPEN_REPARSE_POINT, 0)
	if err != nil {
		return nil, err
	}
	var info windows.ByHandleFileInformation
	if err := windows.GetFileInformationByHandle(handle, &info); err != nil {
		windows.CloseHandle(handle)
		return nil, err
	}
	if info.FileAttributes&(windows.FILE_ATTRIBUTE_REPARSE_POINT|windows.FILE_ATTRIBUTE_DIRECTORY) != 0 {
		windows.CloseHandle(handle)
		return nil, errors.New("ordinary owned file required")
	}
	return os.NewFile(uintptr(handle), name), nil
}
func (nativePlatform) read(name string, limit int64) ([]byte, error) {
	f, err := openNativeRead(name)
	if err != nil {
		return nil, err
	}
	defer f.Close()
	info, err := f.Stat()
	if err != nil || info.Size() <= 0 || info.Size() > limit {
		return nil, errors.New("record length outside bound")
	}
	data, err := io.ReadAll(io.LimitReader(f, limit+1))
	if err != nil || int64(len(data)) > limit {
		return nil, errors.New("record read failed")
	}
	return data, nil
}
func hashFile(f *os.File) (string, error) {
	info, err := f.Stat()
	if err != nil || !info.Mode().IsRegular() || info.Size() <= 0 || info.Size() > 512<<20 {
		return "", errors.New("executable/helper size outside bound")
	}
	if _, err := f.Seek(0, io.SeekStart); err != nil {
		return "", err
	}
	h := sha256.New()
	n, err := io.CopyBuffer(h, io.LimitReader(f, (512<<20)+1), make([]byte, 32<<10))
	if err != nil || n != info.Size() {
		return "", errors.New("executable/helper hash failed")
	}
	return hex.EncodeToString(h.Sum(nil)), nil
}
func (nativePlatform) hold(name, expected string) (io.Closer, error) {
	f, err := openNativeRead(name)
	if err != nil {
		return nil, err
	}
	actual, err := hashFile(f)
	if err != nil || !hashPattern.MatchString(expected) || actual != expected {
		f.Close()
		return nil, errors.New("owned file hash differs")
	}
	return f, nil
}
func (nativePlatform) measure(name string) (string, error) {
	f, err := openNativeRead(name)
	if err != nil {
		return "", err
	}
	defer f.Close()
	info, err := f.Stat()
	if err != nil {
		return "", err
	}
	if err := validatePE32(f, info.Size()); err != nil {
		return "", err
	}
	return hashFile(f)
}
func (nativePlatform) powershell() (string, error) {
	system, err := windows.GetSystemDirectory()
	if err != nil {
		return "", err
	}
	engine, err := localPath(system + "/WindowsPowerShell/v1.0/powershell.exe")
	if err != nil {
		return "", err
	}
	if err := plainNativePath(engine); err != nil {
		return "", err
	}
	return engine, nil
}

func (nativePlatform) lockStartup(name string) (io.Closer, error) {
	canonical, err := localPath(name)
	if err != nil {
		return nil, err
	}
	if err := plainNativePath(path.Dir(canonical)); err != nil {
		return nil, err
	}
	pointer, err := windows.UTF16PtrFromString(canonical)
	if err != nil {
		return nil, err
	}
	handle, err := windows.CreateFile(pointer, windows.GENERIC_READ|windows.GENERIC_WRITE, 0, nil, windows.OPEN_ALWAYS, windows.FILE_FLAG_OPEN_REPARSE_POINT, 0)
	if err != nil {
		return nil, errors.New("installation transaction is busy")
	}
	var info windows.ByHandleFileInformation
	if err := windows.GetFileInformationByHandle(handle, &info); err != nil {
		windows.CloseHandle(handle)
		return nil, err
	}
	if info.FileAttributes&(windows.FILE_ATTRIBUTE_REPARSE_POINT|windows.FILE_ATTRIBUTE_DIRECTORY) != 0 {
		windows.CloseHandle(handle)
		return nil, errors.New("invalid installation lock")
	}
	return os.NewFile(uintptr(handle), canonical), nil
}
func nativeCommand(spec processSpec) *exec.Cmd {
	command := exec.Command(spec.executable, spec.arguments...)
	command.Dir = spec.directory
	command.Stdin = bytes.NewReader(spec.input)
	command.SysProcAttr = &syscall.SysProcAttr{HideWindow: true}
	// A directly started app cannot inherit a health challenge from a caller.
	for _, value := range os.Environ() {
		key, _, _ := strings.Cut(value, "=")
		if !strings.HasPrefix(strings.ToUpper(key), "SKAGER_UPDATE_") {
			command.Env = append(command.Env, value)
		}
	}
	return command
}
func (nativePlatform) wait(ctx context.Context, spec processSpec) (processResult, error) {
	if err := ctx.Err(); err != nil {
		return processResult{}, err
	}
	if _, err := localPath(spec.executable); err != nil {
		return processResult{}, err
	}
	if spec.limit <= 0 || spec.limit > 15*time.Minute || len(spec.input) > 256 {
		return processResult{}, errors.New("invalid child process bound")
	}
	command := nativeCommand(spec)
	if err := command.Start(); err != nil {
		return processResult{}, errors.New("verified child process did not start")
	}
	done := make(chan struct{})
	var waitErr error
	go func() { waitErr = command.Wait(); close(done) }()
	timer := time.NewTimer(spec.limit)
	defer timer.Stop()
	select {
	case <-done:
		if waitErr != nil {
			var exit *exec.ExitError
			if !errors.As(waitErr, &exit) {
				return processResult{}, errors.New("child process wait failed")
			}
		}
		return processResult{code: command.ProcessState.ExitCode(), done: done}, nil
	case <-timer.C:
	case <-ctx.Done():
	}
	// Only the inert prompt can be safely terminated. Installers/supervisors
	// retain their transaction semantics and are guarded by the caller instead.
	if strings.EqualFold(path.Base(strings.ReplaceAll(spec.executable, "\\", "/")), "skager-update-prompt.exe") {
		_ = command.Process.Kill()
		<-done
	}
	return processResult{timedOut: true, done: done}, errors.New("child process wait budget expired")
}
func (nativePlatform) start(spec processSpec) error {
	command := nativeCommand(spec)
	if err := command.Start(); err != nil {
		return errors.New("verified application did not start")
	}
	return command.Process.Release()
}

func privateNativeSecurity() (*windows.SECURITY_DESCRIPTOR, error) {
	user, err := windows.GetCurrentProcessToken().GetTokenUser()
	if err != nil {
		return nil, err
	}
	return windows.SecurityDescriptorFromString("O:" + user.User.Sid.String() + "D:P(A;OICI;FA;;;" + user.User.Sid.String() + ")(A;OICI;FA;;;SY)")
}
func (nativePlatform) createMarker(name string, data []byte) error {
	if err := plainNativePath(path.Dir(name)); err != nil {
		return err
	}
	sd, err := privateNativeSecurity()
	if err != nil {
		return err
	}
	pointer, err := windows.UTF16PtrFromString(name)
	if err != nil {
		return err
	}
	attributes := windows.SecurityAttributes{Length: uint32(unsafe.Sizeof(windows.SecurityAttributes{})), SecurityDescriptor: sd}
	handle, err := windows.CreateFile(pointer, windows.GENERIC_READ|windows.GENERIC_WRITE, 0, &attributes, windows.CREATE_NEW, windows.FILE_FLAG_OPEN_REPARSE_POINT, 0)
	if err != nil {
		return err
	}
	f := os.NewFile(uintptr(handle), name)
	defer f.Close()
	if _, err := f.Write(data); err != nil {
		return err
	}
	return f.Sync()
}
func (nativePlatform) privateDirectory(name string, create bool) error {
	canonical, err := localPath(name)
	if err != nil {
		return err
	}
	name = canonical
	if err := plainNativePath(path.Dir(name)); err != nil {
		return err
	}
	sd, err := privateNativeSecurity()
	if err != nil {
		return err
	}
	pointer, err := windows.UTF16PtrFromString(name)
	if err != nil {
		return err
	}
	if create {
		attributes := windows.SecurityAttributes{Length: uint32(unsafe.Sizeof(windows.SecurityAttributes{})), SecurityDescriptor: sd}
		if err := windows.CreateDirectory(pointer, &attributes); err != nil && !errors.Is(err, windows.ERROR_ALREADY_EXISTS) {
			return err
		}
	}
	if err := plainNativePath(name); err != nil {
		return err
	}
	actual, err := windows.GetNamedSecurityInfo(name, windows.SE_FILE_OBJECT, windows.DACL_SECURITY_INFORMATION|windows.OWNER_SECURITY_INFORMATION)
	if err != nil {
		return err
	}
	if actual.String() != sd.String() {
		return errors.New("provisioned directory permissions or owner differ")
	}
	return nil
}

// progress starts only the adjacent, already held prompt, with no identity,
// URL, artifact path or other data in its dedicated empty-stdin protocol.
func (nativePlatform) progress(executable, directory string) (downloadProgress, error) {
	if _, err := localPath(executable); err != nil {
		return nil, err
	}
	if !strings.EqualFold(path.Base(strings.ReplaceAll(executable, "\\", "/")), "skager-update-prompt.exe") {
		return nil, errors.New("unexpected progress helper")
	}
	command := nativeCommand(processSpec{executable: executable, directory: directory, arguments: []string{"--download-progress"}})
	return startProgressCommand(command)
}

type nativeProgress struct {
	command *exec.Cmd
	input   *os.File
	exited  chan struct{}
	waitErr error
}

func startProgressCommand(command *exec.Cmd) (*nativeProgress, error) {
	reader, writer, err := os.Pipe()
	if err != nil {
		return nil, errors.New("progress pipe unavailable")
	}
	command.Stdin = reader
	if err := command.Start(); err != nil {
		_ = reader.Close()
		_ = writer.Close()
		return nil, errors.New("progress helper did not start")
	}
	_ = reader.Close()
	progress := &nativeProgress{command: command, input: writer, exited: make(chan struct{})}
	go func() {
		progress.waitErr = command.Wait()
		close(progress.exited)
	}()
	return progress, nil
}
func (p *nativeProgress) done() <-chan struct{} { return p.exited }
func (p *nativeProgress) finish(ctx context.Context) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	select {
	case <-p.exited:
		return errors.New("progress helper exited before completion")
	default:
	}
	// No bytes are ever written. Only EOF grants the prompt permission to
	// report successful completion; Cancel/error exits are always nonzero.
	if err := p.input.Close(); err != nil {
		return errors.New("progress completion signal failed")
	}
	select {
	case <-p.exited:
		if p.waitErr != nil || p.command.ProcessState.ExitCode() != 0 || ctx.Err() != nil {
			return errors.New("progress completion was cancelled or failed")
		}
		return nil
	case <-ctx.Done():
		return errors.New("progress completion wait expired")
	}
}
func (p *nativeProgress) close() {
	_ = p.input.Close()
	select {
	case <-p.exited:
		return
	default:
	}
	// This handle is exclusively the inert prompt. No transaction process is
	// managed here, and cleanup cannot cancel a running installer.
	_ = p.command.Process.Kill()
	timer := time.NewTimer(5 * time.Second)
	defer timer.Stop()
	select {
	case <-p.exited:
	case <-timer.C:
	}
}
