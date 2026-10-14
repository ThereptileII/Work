//go:build windows

package brokerenroll

import (
	"context"
	"errors"
	"io"
	"path/filepath"
	"runtime"
	"strings"

	"example.com/opennav-update-verifier/releasepolicy"
	"golang.org/x/sys/windows"
)

type nativeMaintenanceImage struct {
	ctx    context.Context
	files  *nativeEnrollmentFiles
	pin    *enrollmentPin
	reader *io.SectionReader
	sha    string
}

// The process already runs under independently authenticated operator authority.
// The signed policy authenticates these exact original bytes; neither their
// current directory nor a current-process self-hash creates that authority.
func openMaintenanceImageNative(ctx context.Context, artifact releasepolicy.Artifact) (result maintenanceImageSource, resultErr error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if ctx == nil || ctx.Err() != nil || validateMaintenanceArtifact(artifact) != nil {
		return nil, errors.New("live original authenticated maintenance image required")
	}
	if err := enrollmentOperator(); err != nil {
		return nil, err
	}
	user, err := windows.GetCurrentProcessToken().GetTokenUser()
	if err != nil || user == nil || user.User.Sid == nil {
		return nil, errors.Join(errors.New("actual original provisioner user required"), err)
	}
	path, err := maintenanceCurrentImagePath(0)
	if err != nil {
		return nil, err
	}
	n := &nativeMaintenanceImage{ctx: ctx, files: &nativeEnrollmentFiles{}, sha: artifact.SHA256}
	defer func() {
		if resultErr != nil {
			resultErr = errors.Join(resultErr, n.Close())
		}
	}()
	parent, err := startupHoldAncestry(n.files, filepath.Dir(path), user.User.Sid.String())
	if err != nil {
		return nil, err
	}
	n.pin, err = startupOpenFile(n.files, parent, filepath.Base(path), false, maxBrokerBytes)
	if err != nil {
		return nil, err
	}
	if n.pin.length != artifact.Bytes {
		return nil, errors.New("running provisioner length differs from signed policy")
	}
	if err := n.Check(ctx); err != nil {
		return nil, err
	}
	if err := startupHashPin(ctx, n.pin, artifact.SHA256); err != nil {
		return nil, err
	}
	if err := validateMaintenanceLoader(clientBaselinePinReader{n.pin}, n.pin.length); err != nil {
		return nil, err
	}
	n.reader = io.NewSectionReader(clientBaselinePinReader{n.pin}, 0, n.pin.length)
	if err := n.Check(ctx); err != nil {
		return nil, err
	}
	return n, nil
}

func maintenanceCurrentImagePath(flags uint32) (string, error) {
	buffer := make([]uint16, 32768)
	length := uint32(len(buffer))
	if err := windows.QueryFullProcessImageName(windows.CurrentProcess(), flags, &buffer[0], &length); err != nil || length == 0 || length >= uint32(len(buffer)) {
		return "", errors.Join(errors.New("actual running provisioner image unavailable"), err)
	}
	return windows.UTF16ToString(buffer[:length]), nil
}

func (n *nativeMaintenanceImage) Check(ctx context.Context) error {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if n == nil || n.ctx == nil || ctx == nil || n.files == nil || n.pin == nil {
		return errors.New("retained original maintenance source required")
	}
	if err := errors.Join(ctx.Err(), n.ctx.Err(), n.files.Check()); err != nil {
		return err
	}
	path, err := maintenanceCurrentImagePath(windows.PROCESS_NAME_NATIVE)
	if err != nil {
		return err
	}
	pointer, err := windows.UTF16PtrFromString(enrollmentPhysical(path))
	if err != nil {
		return err
	}
	h, err := windows.CreateFile(pointer, windows.FILE_READ_ATTRIBUTES|windows.READ_CONTROL, windows.FILE_SHARE_READ, nil, windows.OPEN_EXISTING, windows.FILE_FLAG_OPEN_REPARSE_POINT, 0)
	if err != nil {
		return err
	}
	info, id, identityErr := enrollmentInfo(h)
	physical, nameErr := enrollmentPhysicalPath(h)
	closeErr := enrollmentCleanup(windows.CloseHandle(h))
	if identityErr != nil || nameErr != nil || closeErr != nil || id != n.pin.id || info.CreationTime != n.pin.info.CreationTime || !strings.EqualFold(physical, n.pin.physical) {
		return errors.Join(errors.New("actual running image differs from original source"), identityErr, nameErr, closeErr)
	}
	return errors.Join(ctx.Err(), n.ctx.Err())
}

func (n *nativeMaintenanceImage) Read(raw []byte) (int, error) {
	if n == nil || n.reader == nil {
		return 0, errors.New("original maintenance image stream required")
	}
	if err := errors.Join(n.ctx.Err(), n.pin.check()); err != nil {
		return 0, err
	}
	return n.reader.Read(raw)
}

func (n *nativeMaintenanceImage) Close() error {
	if n == nil || n.files == nil {
		return nil
	}
	return n.files.Close()
}

func publishMaintenanceImageNative(ctx context.Context, files enrollmentFiles, source maintenanceImageSource, artifact releasepolicy.Artifact) error {
	f, ok := files.(*nativeEnrollmentFiles)
	native, original := source.(*nativeMaintenanceImage)
	if !ok || !original || native == nil || native.sha != artifact.SHA256 || native.pin == nil || native.pin.length != artifact.Bytes {
		return errors.New("original native enrollment and running source required")
	}
	return f.publishMaintenanceImage(ctx, native, artifact)
}

func (f *nativeEnrollmentFiles) publishMaintenanceImage(ctx context.Context, source maintenanceImageSource, artifact releasepolicy.Artifact) (resultErr error) {
	f.mu.Lock()
	defer f.mu.Unlock()
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if ctx == nil || ctx.Err() != nil || source == nil || f.maintenanceAttempted || f.maintenanceCodeParent == nil || validateMaintenanceArtifact(artifact) != nil {
		return errors.New("fresh fixed original maintenance image publication required")
	}
	if err := errors.Join(f.check(), source.Check(ctx)); err != nil {
		return err
	}
	parent := f.maintenanceCodeParent
	if filepath.Join(parent.dos, enrollmentImageName) != f.imagePath {
		return errors.New("original protected code parent differs")
	}
	broker, err := enrollmentBrokerSID()
	if err != nil {
		return err
	}
	sd, err := enrollmentCodeDescriptor(broker, false)
	if err != nil {
		return err
	}
	f.maintenanceAttempted = true // Never adopt/retry a partial destination.
	f.maintenanceSource = source  // run retains this original owner through f.Close.
	writer, pin, err := f.newFile(parent, maintenanceImageName, sd)
	if err != nil {
		return err
	}
	writerOpen := true
	defer func() {
		if writerOpen {
			resultErr = errors.Join(resultErr, enrollmentCleanup(writer.Close()))
		}
	}()
	if err := copyMaintenanceImage(ctx, writer, source, artifact); err != nil {
		return err
	}
	pin.length = artifact.Bytes
	if err := writer.Sync(); err != nil {
		return err
	}
	if err := errors.Join(source.Check(ctx), startupHashPin(ctx, pin, artifact.SHA256)); err != nil {
		return err
	}
	closeErr := enrollmentCleanup(writer.Close())
	writerOpen = false
	if closeErr != nil {
		return closeErr
	}
	// The original protected parent and at-creation ACL span this close/reopen.
	// The pin retains its original full file identity; no replacement is adopted.
	if err := reopenClientRequest(pin); err != nil {
		return err
	}
	pin.dos = filepath.Join(parent.dos, maintenanceImageName)
	f.pins = append(f.pins, pin)
	if err := errors.Join(pin.check(), startupHashPin(ctx, pin, artifact.SHA256), source.Check(ctx), f.check(), ctx.Err()); err != nil {
		return err
	}
	f.maintenanceHelper, f.maintenanceHelperSHA = pin, artifact.SHA256
	return nil
}
