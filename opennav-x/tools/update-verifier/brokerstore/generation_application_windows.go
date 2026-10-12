package brokerstore

import (
	"context"
	"errors"
	"strings"

	"golang.org/x/sys/windows"
)

func (g *nativeInstalledGeneration) matchApplicationProcess(ctx context.Context, processID uint32, created uint64) (resultErr error) {
	if err := ctx.Err(); err != nil {
		return err
	}
	if err := g.check(); err != nil {
		return err
	}
	member := g.members[generationApplicationMember]
	if !g.completed || !g.readOnlyAttempted || member == nil || member.identity.valid() == false {
		return errors.New("retained original readonly application required")
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
		return errors.Join(errors.New("bounded native application image required"), err)
	}
	actual := windows.UTF16ToString(image[:length])
	if _, err := physicalParts(actual); err != nil || !strings.EqualFold(actual, member.pin.name) {
		return errors.New("process does not run the fixed retained application")
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
		return errors.Join(errors.New("plain single-link original application required"), err)
	}
	identity, err := fileIdentity(file)
	if err != nil || !identity.valid() || identity != member.identity {
		return errors.Join(errors.New("application original full file identity differs"), err)
	}
	if err := executorCreationMatches(process, created); err != nil {
		return err
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	return g.check()
}
