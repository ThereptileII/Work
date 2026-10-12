package brokerstore

import (
	"context"
	"errors"
	"strings"

	"golang.org/x/sys/windows"
)

func executorCreationMatches(process windows.Handle, expected uint64) error {
	status, err := windows.WaitForSingleObject(process, 0)
	if err != nil || status != uint32(windows.WAIT_TIMEOUT) {
		return errors.New("interactive executor is not live")
	}
	var created, exited, kernel, user windows.Filetime
	if err := windows.GetProcessTimes(process, &created, &exited, &kernel, &user); err != nil {
		return err
	}
	actual := uint64(created.HighDateTime)<<32 | uint64(created.LowDateTime)
	if actual != expected {
		return errors.New("interactive executor creation identity differs")
	}
	return nil
}

func (g *nativeInstalledGeneration) matchExecutorProcess(ctx context.Context, processID uint32, created uint64) (resultErr error) {
	if err := ctx.Err(); err != nil {
		return err
	}
	if err := g.check(); err != nil {
		return err
	}
	member := g.members[generationExecutorMember]
	if !g.completed || !g.readOnlyAttempted || member == nil || member.identity.valid() == false {
		return errors.New("retained original readonly executor required")
	}
	process, err := windows.OpenProcess(windows.PROCESS_QUERY_LIMITED_INFORMATION|windows.SYNCHRONIZE, false, processID)
	if err != nil {
		return err
	}
	defer func() { resultErr = errors.Join(resultErr, windows.CloseHandle(process)) }()
	if err := executorCreationMatches(process, created); err != nil {
		return err
	}
	var image [32768]uint16
	length := uint32(len(image))
	if err := windows.QueryFullProcessImageName(process, windows.PROCESS_NAME_NATIVE, &image[0], &length); err != nil || length == 0 || length >= uint32(len(image)) {
		return errors.Join(errors.New("bounded native executor image required"), err)
	}
	actual := windows.UTF16ToString(image[:length])
	if _, err := physicalParts(actual); err != nil || !strings.EqualFold(actual, member.pin.name) {
		return errors.New("executor does not run the fixed retained launcher")
	}
	pointer, err := windows.UTF16PtrFromString(physical(actual))
	if err != nil {
		return err
	}
	file, err := windows.CreateFile(pointer, windows.GENERIC_READ|windows.READ_CONTROL, windows.FILE_SHARE_READ, nil, windows.OPEN_EXISTING, windows.FILE_FLAG_OPEN_REPARSE_POINT, 0)
	if err != nil {
		return err
	}
	defer func() { resultErr = errors.Join(resultErr, windows.CloseHandle(file)) }()
	var info windows.ByHandleFileInformation
	if err := windows.GetFileInformationByHandle(file, &info); err != nil || info.FileAttributes&(windows.FILE_ATTRIBUTE_REPARSE_POINT|windows.FILE_ATTRIBUTE_DIRECTORY) != 0 || info.NumberOfLinks != 1 {
		return errors.Join(errors.New("plain single-link original executor required"), err)
	}
	identity, err := fileIdentity(file)
	if err != nil || !identity.valid() || identity != member.identity {
		return errors.Join(errors.New("executor original full file identity differs"), err)
	}
	if err := executorCreationMatches(process, created); err != nil {
		return err
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	return g.check()
}
