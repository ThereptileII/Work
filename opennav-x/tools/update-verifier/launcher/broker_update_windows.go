package launcher

import (
	"context"
	"errors"
	"os"
	"runtime"
	"strings"
	"time"
	"unsafe"

	"example.com/opennav-update-verifier/brokerpipe"
	"golang.org/x/sys/windows"
)

func runBrokerOfferNative(ctx context.Context, generation string) (resultErr error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	backend, err := newNativeBrokerExecutionBackend(ctx)
	if err != nil {
		return err
	}
	return runBrokerUpdate(ctx, generation, brokerUpdateServices{access: readCloudToken, conversation: brokerpipe.CallExecution, prompt: backend.promptUpdate, close: backend.close, stage: brokerpipe.Call, execute: runBrokerExecutionNative, current: runBrokerCurrentNative})
}

// The authenticated original selected-member pin comes only from OfferUpdate.
// Hold a deny-write/delete file plus every physical ancestor across suspended
// creation, corroboration, stdin publication, exit and cleanup. No local inventory
// or current ACL is interpreted as creation provenance. Only this inert UI child
// may be terminated on timeout; no installer/application is launched here.
func (b *nativeBrokerExecutionBackend) promptUpdate(ctx context.Context, c brokerpipe.ExecutionCommand) (now bool, resultErr error) {
	if err := b.check(ctx); err != nil {
		return false, err
	}
	if !validBrokerOffer(c, c.StageID) {
		return false, errBrokerExecutionRefused
	}
	executable, err := brokerExecutionApplication(b.programFiles, b.user.sid, c.ApplicationGeneration)
	if err != nil {
		return false, err
	}
	executable = strings.TrimSuffix(executable, "opencpn.exe") + "skager-update-prompt.exe"
	acquired, err := acquireNativePinnedBrokerImage(executable, c.PromptSHA256)
	if err != nil {
		return false, err
	}
	image, ok := acquired.(*nativeOrdinaryImage)
	if !ok {
		return false, errors.Join(errBrokerExecutionRefused, acquired.Close())
	}
	defer func() { resultErr = errors.Join(resultErr, image.Close()) }()
	if int64(image.info.FileSizeHigh)<<32|int64(image.info.FileSizeLow) != c.PromptBytes {
		return false, errBrokerExecutionRefused
	}
	if err = validatePE32(image.file, c.PromptBytes); err != nil {
		return false, err
	}
	input, err := promptInput(consentFromBrokerOffer(c))
	if err != nil {
		return false, err
	}
	var read, write windows.Handle
	if err = windows.CreatePipe(&read, &write, nil, 0); err != nil {
		return false, err
	}
	defer func() {
		if read != 0 {
			resultErr = errors.Join(resultErr, windows.CloseHandle(read))
		}
		if write != 0 {
			resultErr = errors.Join(resultErr, windows.CloseHandle(write))
		}
	}()
	if err = windows.SetHandleInformation(read, windows.HANDLE_FLAG_INHERIT, windows.HANDLE_FLAG_INHERIT); err != nil {
		return false, err
	}
	nulName, _ := windows.UTF16PtrFromString("NUL")
	attributes := windows.SecurityAttributes{Length: uint32(unsafe.Sizeof(windows.SecurityAttributes{})), InheritHandle: 1}
	nul, err := windows.CreateFile(nulName, windows.GENERIC_WRITE, windows.FILE_SHARE_READ|windows.FILE_SHARE_WRITE, &attributes, windows.OPEN_EXISTING, 0, 0)
	if err != nil {
		return false, err
	}
	defer func() { resultErr = errors.Join(resultErr, windows.CloseHandle(nul)) }()
	list, err := windows.NewProcThreadAttributeList(1)
	if err != nil {
		return false, err
	}
	defer list.Delete()
	handles := []windows.Handle{read, nul}
	if err = list.Update(windows.PROC_THREAD_ATTRIBUTE_HANDLE_LIST, unsafe.Pointer(&handles[0]), uintptr(len(handles))*unsafe.Sizeof(handles[0])); err != nil {
		return false, err
	}
	startup := windows.StartupInfoEx{StartupInfo: windows.StartupInfo{Cb: uint32(unsafe.Sizeof(windows.StartupInfoEx{})), Flags: windows.STARTF_USESTDHANDLES | windows.STARTF_USESHOWWINDOW, ShowWindow: windows.SW_SHOWNORMAL, StdInput: read, StdOutput: nul, StdErr: nul}, ProcThreadAttributeList: list.List()}
	application, err := windows.UTF16PtrFromString(image.application)
	if err != nil {
		return false, err
	}
	command, err := windows.UTF16FromString(windows.ComposeCommandLine([]string{physicalWin32(image.name)}))
	if err != nil {
		return false, err
	}
	directory, err := windows.UTF16PtrFromString(physicalWin32(image.name[:strings.LastIndex(image.name, `\`)]))
	if err != nil {
		return false, err
	}
	environment, err := ordinaryEnvironment(os.Environ())
	if err != nil {
		return false, err
	}
	if err = errors.Join(b.check(ctx), image.recheck()); err != nil {
		return false, err
	}
	var process windows.ProcessInformation
	if err = windows.CreateProcess(application, &command[0], nil, nil, true, windows.CREATE_SUSPENDED|windows.CREATE_UNICODE_ENVIRONMENT|windows.EXTENDED_STARTUPINFO_PRESENT, &environment[0], directory, &startup.StartupInfo, &process); err != nil {
		return false, err
	}
	child := &nativeOrdinaryChild{process: process.Process, thread: process.Thread, pid: process.ProcessId, tid: process.ThreadId, image: image.name}
	exited := false
	defer func() {
		if !exited {
			// The returned creator process handle is exclusive to this inert popup. It
			// remains the authority even if image/token corroboration or Resume fails.
			resultErr = errors.Join(resultErr, windows.TerminateProcess(process.Process, 1))
			state, err := windows.WaitForSingleObject(process.Process, 5000)
			if err != nil || state != windows.WAIT_OBJECT_0 {
				resultErr = errors.Join(resultErr, errBrokerExecutionUncertain, err)
			}
		}
		resultErr = errors.Join(resultErr, child.Close())
	}()
	if err = child.corroborate(); err != nil {
		return false, err
	}
	var token windows.Token
	if err = windows.OpenProcessToken(child.process, windows.TOKEN_QUERY, &token); err != nil {
		return false, err
	}
	user, userErr := brokerExecutionToken(token)
	err = errors.Join(userErr, token.Close())
	if err != nil || user != b.user {
		return false, errors.Join(errBrokerExecutionRefused, err)
	}
	if err = errors.Join(b.check(ctx), image.recheck()); err != nil {
		return false, err
	}
	count, err := child.resume()
	if err != nil || count != 1 {
		return false, errors.Join(errBrokerExecutionUncertain, err)
	}
	// Only the stdin reader and NUL were inherited. Close our reader, publish the
	// bounded existing prompt protocol, then EOF; no secret/pipe handle inheritance.
	if err = windows.CloseHandle(read); err != nil {
		return false, err
	}
	read = 0
	writer := os.NewFile(uintptr(write), "protected-update-prompt-stdin")
	write = 0
	n, writeErr := writer.Write(input)
	err = errors.Join(writeErr, writer.Close())
	if err != nil || n != len(input) {
		return false, errors.Join(errBrokerExecutionUncertain, err)
	}
	deadline := time.Now().Add(4*time.Minute + 30*time.Second)
	for {
		state, waitErr := windows.WaitForSingleObject(child.process, 100)
		if waitErr != nil {
			return false, waitErr
		}
		if state == windows.WAIT_OBJECT_0 {
			exited = true
			break
		}
		if state != uint32(windows.WAIT_TIMEOUT) {
			return false, errBrokerExecutionUncertain
		}
		if ctx.Err() != nil || time.Now().After(deadline) {
			return false, errors.Join(errBrokerExecutionUncertain, ctx.Err())
		}
		if err = b.check(ctx); err != nil {
			return false, err
		}
	}
	var code uint32
	if err = windows.GetExitCodeProcess(child.process, &code); err != nil {
		return false, err
	}
	if err = errors.Join(b.check(ctx), image.recheck()); err != nil {
		return false, err
	}
	return brokerUpdatePromptResult(code)
}
