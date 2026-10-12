package brokerstore

import (
	"errors"
	"path/filepath"
	"strings"

	"golang.org/x/sys/windows"
)

// programDataGuard is a retained nonempty-directory witness, not authority over
// its contents. The independent machine Common Start Menu KnownFolder must be a
// plain descendant of this exact ProgramData parent. Keeping its first child
// open without delete sharing prevents an add-permitted parent becoming empty
// while the broker relies on the narrow ProgramData metadata-ACE exception.
type programDataGuard struct {
	parent   *nativePin
	parentID diskIdentity
	children []programDataGuardChild
}

type programDataGuardChild struct {
	pin *nativePin
	id  diskIdentity
}

func fixedProgramDataGuard(parent *nativePin, programDataDOS string) (*programDataGuard, []*nativePin, error) {
	menuDOS, err := windows.KnownFolderPath(windows.FOLDERID_CommonStartMenu, windows.KF_FLAG_DEFAULT)
	if err != nil {
		return nil, nil, err
	}
	return retainProgramDataGuard(parent, programDataDOS, menuDOS)
}

// retainProgramDataGuard takes no caller-selected child in production. The
// explicit path arguments are confined to the private inert Windows fixture.
func retainProgramDataGuard(parent *nativePin, programDataDOS, menuDOS string) (guard *programDataGuard, owned []*nativePin, resultErr error) {
	if parent == nil || parent.security != nil || filepath.Clean(programDataDOS) != programDataDOS || filepath.Clean(menuDOS) != menuDOS || len(programDataDOS) < 4 || programDataDOS[1] != ':' || programDataDOS[2] != '\\' {
		return nil, nil, errors.New("original fixed ProgramData parent required")
	}
	prefix := programDataDOS + `\`
	if len(menuDOS) <= len(prefix) || !strings.EqualFold(menuDOS[:len(prefix)], prefix) {
		return nil, nil, errors.New("independent machine KnownFolder is not below ProgramData")
	}
	components := strings.Split(menuDOS[len(prefix):], `\`)
	if len(components) < 1 || len(components) > 8 {
		return nil, nil, errors.New("bounded nonempty machine KnownFolder descendant required")
	}
	for _, component := range components {
		if !safeComponent(component) {
			return nil, nil, errors.New("ambiguous machine KnownFolder descendant")
		}
	}
	parentID, err := fileIdentity(parent.handle)
	if err != nil {
		return nil, nil, err
	}
	guard = &programDataGuard{parent: parent, parentID: parentID}
	var acquired []*nativePin
	ready := false
	defer func() {
		if !ready {
			for i := len(acquired) - 1; i >= 0; i-- {
				resultErr = errors.Join(resultErr, windows.CloseHandle(acquired[i].handle))
			}
			guard, owned = nil, nil
		}
	}()
	walk := parent.name
	for i, component := range components {
		walk += `\` + component
		child, err := openPin(physical(walk), walk, nil)
		if err != nil {
			return nil, nil, err
		}
		acquired = append(acquired, child)
		if child.info.VolumeSerialNumber != parent.info.VolumeSerialNumber {
			return nil, nil, errors.New("machine KnownFolder descendant crossed volumes")
		}
		id, err := fileIdentity(child.handle)
		if err != nil || id.Volume != parentID.Volume {
			return nil, nil, errors.Join(errors.New("machine KnownFolder descendant full identity unavailable"), err)
		}
		guard.children = append(guard.children, programDataGuardChild{child, id})
		if i == len(components)-1 {
			child.dos = menuDOS
		}
	}
	if err := guard.check(parent.handle); err != nil {
		return nil, nil, err
	}
	ready = true
	owned = acquired
	return guard, owned, nil
}

func (g *programDataGuard) check(parent windows.Handle) error {
	if g == nil || g.parent == nil || g.parent.handle != parent || len(g.children) == 0 {
		return errors.New("original ProgramData nonempty witness unavailable")
	}
	if err := g.parent.checkIdentityOnly(); err != nil {
		return err
	}
	id, err := fileIdentity(parent)
	if err != nil || id != g.parentID {
		return errors.Join(errors.New("original ProgramData full identity changed"), err)
	}
	previous := g.parent.name
	for _, child := range g.children {
		if child.pin == nil || !strings.HasPrefix(child.pin.name, previous+`\`) || strings.Contains(child.pin.name[len(previous)+1:], `\`) || child.pin.info.VolumeSerialNumber != g.parent.info.VolumeSerialNumber {
			return errors.New("machine KnownFolder is not an immediate physical child")
		}
		if err := child.pin.check(); err != nil {
			return err
		}
		actual, err := fileIdentity(child.pin.handle)
		if err != nil || actual != child.id || actual.Volume != g.parentID.Volume {
			return errors.Join(errors.New("retained machine KnownFolder child identity changed"), err)
		}
		previous = child.pin.name
	}
	return nil
}
