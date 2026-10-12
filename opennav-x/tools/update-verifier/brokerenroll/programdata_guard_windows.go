//go:build windows

package brokerenroll

import (
	"errors"
	"path/filepath"
	"strings"
	"sync"

	"golang.org/x/sys/windows"
)

// The common menu supplies an existing, independently resolved immediate child
// of actual ProgramData. Retaining its original plain directory chain without
// delete sharing keeps ProgramData nonempty, which prevents SET_REPARSE_POINT.
// Its contents and ACLs grant no enrollment, application or selection authority.
// MS-FSA 2.1.5.10.37 requires rejection for nonempty DirectoryList even when
// FILE_WRITE_DATA or FILE_WRITE_ATTRIBUTES would authorize the FSCTL:
// https://learn.microsoft.com/en-us/openspecs/windows_protocols/ms-fsa/4aeefef8-92c3-4abc-af7a-a610caf8a165
type enrollmentProgramDataGuard struct {
	mu                      sync.Mutex
	programData, commonMenu string
	parentPhysical          string
	parentID                enrollmentFileID
	parentInfo              windows.ByHandleFileInformation
	children                []enrollmentGuardDirectory
	closed                  bool
	closeErr                error
}

type enrollmentGuardDirectory struct {
	handle        windows.Handle
	physical, dos string
	id            enrollmentFileID
	info          windows.ByHandleFileInformation
}

func enrollmentProgramDataComponents(data, menu string) ([]string, error) {
	for _, path := range []string{data, menu} {
		if len(path) < 4 || filepath.Clean(path) != path || path[1:3] != `:\` ||
			!((path[0] >= 'A' && path[0] <= 'Z') || (path[0] >= 'a' && path[0] <= 'z')) ||
			strings.ContainsAny(path[3:], ":/\x00") {
			return nil, errors.New("canonical local ProgramData menu custody required")
		}
		for _, part := range strings.Split(path[3:], `\`) {
			if part == "" || part == "." || part == ".." || strings.HasSuffix(part, ".") || strings.HasSuffix(part, " ") {
				return nil, errors.New("ambiguous ProgramData menu custody")
			}
		}
	}
	if len(menu) <= len(data)+1 || !strings.EqualFold(menu[:len(data)], data) || menu[len(data)] != '\\' {
		return nil, errors.New("common menu must remain strictly inside actual ProgramData")
	}
	return strings.Split(menu[len(data)+1:], `\`), nil
}

func enrollmentOpenProgramDataGuard(parent *enrollmentPin) (result *enrollmentProgramDataGuard, resultErr error) {
	data, err := windows.KnownFolderPath(windows.FOLDERID_ProgramData, windows.KF_FLAG_DEFAULT)
	if err != nil || !strings.EqualFold(parent.dos, data) {
		// No exception for another ancestor, or without independently obtained
		// actual ProgramData. Its original strict policy remains decisive.
		return nil, nil
	}
	menu, err := windows.KnownFolderPath(windows.FOLDERID_CommonStartMenu, windows.KF_FLAG_DEFAULT)
	if err != nil {
		return nil, err
	}
	parts, err := enrollmentProgramDataComponents(data, menu)
	if err != nil {
		return nil, err
	}
	g := &enrollmentProgramDataGuard{programData: data, commonMenu: menu, parentPhysical: parent.physical,
		parentID: parent.id, parentInfo: parent.info}
	defer func() {
		if result == nil {
			resultErr = errors.Join(resultErr, g.Close())
		}
	}()
	physical, dos := parent.physical, data
	for _, part := range parts {
		physical += `\` + part
		dos += `\` + part
		child, err := enrollmentOpenGuardDirectory(enrollmentPhysical(physical), physical, dos)
		if err != nil {
			return nil, err
		}
		g.children = append(g.children, child)
	}
	if err := g.check(parent); err != nil {
		return nil, err
	}
	return g, nil
}

func enrollmentOpenGuardDirectory(path, expected, dos string) (result enrollmentGuardDirectory, resultErr error) {
	pointer, err := windows.UTF16PtrFromString(path)
	if err != nil {
		return result, err
	}
	handle, err := windows.CreateFile(pointer, windows.FILE_LIST_DIRECTORY|windows.FILE_READ_ATTRIBUTES,
		windows.FILE_SHARE_READ|windows.FILE_SHARE_WRITE, nil, windows.OPEN_EXISTING,
		windows.FILE_FLAG_OPEN_REPARSE_POINT|windows.FILE_FLAG_BACKUP_SEMANTICS, 0)
	if err != nil {
		return result, err
	}
	ready := false
	defer func() {
		if !ready {
			resultErr = errors.Join(resultErr, enrollmentCleanup(windows.CloseHandle(handle)))
		}
	}()
	info, id, err := enrollmentInfo(handle)
	if err != nil {
		return result, err
	}
	name, err := enrollmentPhysicalPath(handle)
	if err != nil || name != expected || info.FileAttributes&(windows.FILE_ATTRIBUTE_DIRECTORY|windows.FILE_ATTRIBUTE_REPARSE_POINT) != windows.FILE_ATTRIBUTE_DIRECTORY {
		return result, errors.New("original plain ProgramData menu directory required")
	}
	ready = true
	return enrollmentGuardDirectory{handle: handle, physical: name, dos: dos, id: id, info: info}, nil
}

func (g *enrollmentProgramDataGuard) check(parent *enrollmentPin) error {
	g.mu.Lock()
	defer g.mu.Unlock()
	if g.closed || len(g.children) == 0 || parent.id != g.parentID || parent.physical != g.parentPhysical {
		return errors.New("original ProgramData nonempty custody unavailable")
	}
	data, err := windows.KnownFolderPath(windows.FOLDERID_ProgramData, windows.KF_FLAG_DEFAULT)
	if err != nil || data != g.programData {
		return errors.New("actual ProgramData custody resolution changed")
	}
	menu, err := windows.KnownFolderPath(windows.FOLDERID_CommonStartMenu, windows.KF_FLAG_DEFAULT)
	if err != nil || menu != g.commonMenu {
		return errors.New("actual common menu custody resolution changed")
	}
	parts, err := enrollmentProgramDataComponents(data, menu)
	if err != nil || len(parts) != len(g.children) {
		return errors.New("original ProgramData menu chain changed")
	}
	if err := enrollmentCheckGuardDirectory(parent.handle, g.parentPhysical, g.parentID, g.parentInfo); err != nil {
		return err
	}
	var serial, component, flags uint32
	fs := make([]uint16, 32)
	if err := windows.GetVolumeInformationByHandle(parent.handle, nil, 0, &serial, &component, &flags, &fs[0], uint32(len(fs))); err != nil ||
		windows.UTF16ToString(fs) != "NTFS" || serial != g.parentInfo.VolumeSerialNumber {
		return errors.New("original ProgramData NTFS volume changed")
	}
	if err := enrollmentProbeGuardDirectory(data, g.parentPhysical, g.parentID); err != nil {
		return err
	}
	physical := g.parentPhysical
	previousID := g.parentID
	for i, child := range g.children {
		physical += `\` + parts[i]
		if child.physical != physical || child.id == previousID || child.id.Volume != g.parentID.Volume || child.info.VolumeSerialNumber != serial {
			return errors.New("original ProgramData immediate-child binding changed")
		}
		if err := enrollmentCheckGuardDirectory(child.handle, physical, child.id, child.info); err != nil {
			return err
		}
		if err := enrollmentProbeGuardDirectory(child.dos, physical, child.id); err != nil {
			return err
		}
		previousID = child.id
	}
	return nil
}

func enrollmentCheckGuardDirectory(handle windows.Handle, physical string, id enrollmentFileID, original windows.ByHandleFileInformation) error {
	info, current, err := enrollmentInfo(handle)
	if err != nil {
		return err
	}
	name, err := enrollmentPhysicalPath(handle)
	if err != nil || name != physical || current != id || info.CreationTime != original.CreationTime ||
		info.VolumeSerialNumber != original.VolumeSerialNumber ||
		info.FileAttributes&(windows.FILE_ATTRIBUTE_DIRECTORY|windows.FILE_ATTRIBUTE_REPARSE_POINT) != windows.FILE_ATTRIBUTE_DIRECTORY {
		return errors.New("original ProgramData directory custody changed")
	}
	return nil
}

func enrollmentProbeGuardDirectory(dos, physical string, id enrollmentFileID) error {
	probe, err := enrollmentOpenGuardDirectory(dos, physical, dos)
	if err != nil {
		return err
	}
	closeErr := enrollmentCleanup(windows.CloseHandle(probe.handle))
	if probe.id != id {
		return errors.Join(errors.New("ProgramData menu DOS resolution changed"), closeErr)
	}
	return closeErr
}

func (g *enrollmentProgramDataGuard) Close() error {
	g.mu.Lock()
	defer g.mu.Unlock()
	if g.closed {
		return g.closeErr
	}
	g.closed = true
	for i := len(g.children) - 1; i >= 0; i-- {
		g.closeErr = errors.Join(g.closeErr, enrollmentCleanup(windows.CloseHandle(g.children[i].handle)))
	}
	return g.closeErr
}
