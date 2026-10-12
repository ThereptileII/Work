//go:build windows

package brokerenroll

import (
	"context"
	"crypto/sha1" // Windows service SID derivation, never artifact authentication.
	"encoding/binary"
	"errors"
	"fmt"
	"io"
	"os"
	"path/filepath"
	"runtime"
	"strings"
	"sync"
	"unicode/utf16"
	"unsafe"

	"example.com/opennav-update-verifier/releasepolicy"
	"golang.org/x/sys/windows"
)

const enrollmentFenceName = "UpdateBroker.enrollment"
const filesStarted = "SKAGER-UpdateBroker-files-v1:0\n"
const filesCompleted = "SKAGER-UpdateBroker-files-v1:1\n"
const enrollmentTrustedInstaller = "S-1-5-80-956008885-3418522649-1831038044-1853292631-2271478464"

var rtlCreateServiceSID = windows.NewLazySystemDLL("ntdll.dll").NewProc("RtlCreateServiceSid")

// enrollmentBrokerSID derives the virtual account SID without assuming that an
// SCM service already exists. Independently corroborate the native result with
// Windows' deterministic uppercase UTF-16 service-name SID derivation.
func enrollmentBrokerSID() (*windows.SID, error) {
	if serviceAccount != `NT SERVICE\`+serviceName {
		return nil, errors.New("fixed enrollment service identity differs")
	}
	name, err := windows.NewNTUnicodeString(serviceName)
	if err != nil {
		return nil, err
	}
	buffer := make([]byte, 68)
	length := uint32(len(buffer))
	status, _, _ := rtlCreateServiceSID.Call(uintptr(unsafe.Pointer(name)), uintptr(unsafe.Pointer(&buffer[0])), uintptr(unsafe.Pointer(&length)))
	runtime.KeepAlive(name)
	if status != 0 || length != 32 {
		return nil, errors.New("native service SID derivation refused")
	}
	sid := (*windows.SID)(unsafe.Pointer(&buffer[0]))
	if !sid.IsValid() {
		return nil, errors.New("invalid native service SID")
	}
	units := utf16.Encode([]rune(strings.ToUpper(serviceName)))
	encoded := make([]byte, 2*len(units))
	for i, unit := range units {
		binary.LittleEndian.PutUint16(encoded[2*i:], unit)
	}
	digest := sha1.Sum(encoded)
	wanted := "S-1-5-80"
	for i := 0; i < 5; i++ {
		wanted += fmt.Sprintf("-%d", binary.LittleEndian.Uint32(digest[4*i:]))
	}
	if sid.String() != wanted {
		return nil, errors.New("native service SID differs from fixed service name")
	}
	return sid.Copy()
}

func enrollmentOperator() error {
	var thread windows.Token
	err := windows.OpenThreadToken(windows.CurrentThread(), windows.TOKEN_QUERY, true, &thread)
	if err == nil {
		return errors.Join(errors.New("impersonated enrollment refused"), enrollmentCleanup(thread.Close()))
	}
	if !errors.Is(err, windows.ERROR_NO_TOKEN) {
		return errors.New("non-impersonated enrollment token required")
	}
	token := windows.GetCurrentProcessToken()
	if !token.IsElevated() {
		return errors.New("independently authenticated elevated enrollment required")
	}
	admin, err := windows.CreateWellKnownSid(windows.WinBuiltinAdministratorsSid)
	if err != nil {
		return err
	}
	member, err := windows.Token(0).IsMember(admin)
	if err != nil || !member {
		return errors.New("elevated administrator enrollment required")
	}
	return nil
}

type enrollmentFileID struct {
	Volume uint64
	ID     [16]byte
}

type enrollmentPin struct {
	handle        windows.Handle
	physical, dos string
	info          windows.ByHandleFileInformation
	id            enrollmentFileID
	descriptor    *windows.SECURITY_DESCRIPTOR
	allowAdds     bool
	broker        string
	file          bool
	length        int64
}

type nativeEnrollmentFiles struct {
	mu                   sync.Mutex
	pins                 []*enrollmentPin
	imagePath            string
	fence                *os.File
	fencePin             *enrollmentPin
	closed               bool
	closeErr             error
	data                 *enrollmentPin
	clientAttempted      bool
	clientSID, clientSHA string
}

// createFreshFiles requires an independently authenticated provisioner before
// its first privileged instruction. Token checks are not self-authentication.
// It creates only the fixed fresh machine namespace. Directories and the durable
// fence are never deleted on error: any interrupted creation requires separate
// operator recovery, never automatic adoption, ACL repair or trust-floor reset.
// Admin/SYSTEM are trusted for machine code, not adversaries excluded by this API.
func createFreshFiles(ctx context.Context, source io.Reader, artifact releasepolicy.Artifact) (result enrollmentFiles, resultErr error) {
	if err := validateEnrollmentImage(ctx, source, artifact); err != nil {
		return nil, err
	}
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if err := enrollmentOperator(); err != nil {
		return nil, err
	}
	broker, err := enrollmentBrokerSID()
	if err != nil {
		return nil, err
	}
	files := &nativeEnrollmentFiles{}
	ready := false
	defer func() {
		if !ready {
			resultErr = errors.Join(resultErr, files.Close())
		}
	}()
	programFiles, err := windows.KnownFolderPath(windows.FOLDERID_ProgramFilesX86, windows.KF_FLAG_DEFAULT)
	if err != nil {
		return nil, err
	}
	programData, err := windows.KnownFolderPath(windows.FOLDERID_ProgramData, windows.KF_FLAG_DEFAULT)
	if err != nil {
		return nil, err
	}
	codeParent, err := files.holdAncestry(programFiles, false)
	if err != nil {
		return nil, err
	}
	dataParent, err := files.holdAncestry(programData, true)
	if err != nil {
		return nil, err
	}
	codeSD, err := windows.SecurityDescriptorFromString("O:BAD:P(A;OICI;FA;;;SY)(A;OICI;FA;;;BA)(A;OICI;FRFX;;;AU)(A;OICI;FRFX;;;" + broker.String() + ")")
	if err != nil {
		return nil, err
	}
	dataSD, err := windows.SecurityDescriptorFromString("O:BAD:P(A;OICI;FA;;;SY)(A;OICI;FA;;;BA)(A;OICI;FA;;;" + broker.String() + ")")
	if err != nil {
		return nil, err
	}
	fileSD, err := windows.SecurityDescriptorFromString("O:BAD:P(A;;FA;;;SY)(A;;FA;;;BA)(A;;FRFX;;;AU)(A;;FRFX;;;" + broker.String() + ")")
	if err != nil {
		return nil, err
	}
	fenceSD, err := windows.SecurityDescriptorFromString("O:BAD:P(A;;FA;;;SY)(A;;FA;;;BA)")
	if err != nil {
		return nil, err
	}
	// Refuse either existing fixed child before creating either namespace. These
	// checks grant no authority; exclusive native creation remains decisive.
	for _, parent := range []*enrollmentPin{codeParent, dataParent} {
		if err := enrollmentAbsent(parent.physical + `\SKAGER`); err != nil {
			return nil, err
		}
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	data, err := files.newDirectory(dataParent, "SKAGER", dataSD, broker.String())
	if err != nil {
		return nil, err
	}
	files.data = data
	if err := files.createFence(data, fenceSD); err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	code, err := files.newDirectory(codeParent, "SKAGER", codeSD, "")
	if err != nil {
		return nil, err
	}
	code, err = files.newDirectory(code, "UpdateBroker", codeSD, "")
	if err != nil {
		return nil, err
	}
	if err := files.createImage(ctx, code, fileSD, source, artifact); err != nil {
		return nil, err
	}
	if err := files.check(); err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	// This completion describes only files, never service enrollment/readiness.
	if _, err := files.fence.Seek(0, io.SeekStart); err != nil {
		return nil, err
	}
	if n, err := files.fence.WriteString(filesCompleted); err != nil {
		return nil, err
	} else if n != len(filesCompleted) {
		return nil, io.ErrShortWrite
	}
	if err := files.fence.Sync(); err != nil {
		return nil, err
	}
	if err := files.check(); err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	ready = true
	return files, nil
}

func (f *nativeEnrollmentFiles) ImagePath() string { return f.imagePath }

func (f *nativeEnrollmentFiles) Check() error {
	f.mu.Lock()
	defer f.mu.Unlock()
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	return f.check()
}

func (f *nativeEnrollmentFiles) check() error {
	if f.closed {
		return errors.New("enrollment file custody closed")
	}
	if err := enrollmentOperator(); err != nil {
		return err
	}
	for _, pin := range f.pins {
		if err := pin.check(); err != nil {
			return err
		}
	}
	return nil
}

func (f *nativeEnrollmentFiles) Close() error {
	f.mu.Lock()
	defer f.mu.Unlock()
	if f.closed {
		return f.closeErr
	}
	f.closed = true
	for i := len(f.pins) - 1; i >= 0; i-- {
		pin := f.pins[i]
		if pin == f.fencePin && f.fence != nil {
			f.closeErr = errors.Join(f.closeErr, f.fence.Close())
			f.fence = nil
		} else {
			f.closeErr = errors.Join(f.closeErr, windows.CloseHandle(pin.handle))
		}
	}
	f.pins = nil
	if f.closeErr != nil {
		f.closeErr = errors.Join(ErrCleanup, f.closeErr)
	}
	return f.closeErr
}

func enrollmentCleanup(err error) error {
	if err == nil {
		return nil
	}
	return errors.Join(ErrCleanup, err)
}

func enrollmentPhysicalPath(handle windows.Handle) (string, error) {
	buffer := make([]uint16, 32768)
	n, err := windows.GetFinalPathNameByHandle(handle, &buffer[0], uint32(len(buffer)), 2)
	if err != nil || n == 0 || n >= uint32(len(buffer)) {
		return "", errors.New("physical enrollment identity unavailable")
	}
	name := strings.TrimRight(windows.UTF16ToString(buffer[:n]), `\`)
	if _, err := enrollmentPhysicalParts(name); err != nil {
		return "", err
	}
	return name, nil
}

func enrollmentPhysicalParts(name string) ([]string, error) {
	parts := strings.Split(name, `\`)
	if len(parts) < 3 || parts[0] != "" || parts[1] != "Device" || !strings.HasPrefix(parts[2], "HarddiskVolume") {
		return nil, errors.New("physical local enrollment volume required")
	}
	digits := strings.TrimPrefix(parts[2], "HarddiskVolume")
	if len(digits) < 1 || len(digits) > 10 {
		return nil, errors.New("invalid enrollment volume")
	}
	for _, c := range digits {
		if c < '0' || c > '9' {
			return nil, errors.New("invalid enrollment volume")
		}
	}
	for _, p := range parts[3:] {
		if p == "" || p == "." || p == ".." || strings.HasSuffix(p, ".") || strings.HasSuffix(p, " ") || strings.ContainsAny(p, ":/\x00") {
			return nil, errors.New("ambiguous physical enrollment path")
		}
	}
	return parts, nil
}

func enrollmentPhysical(name string) string {
	parts, _ := enrollmentPhysicalParts(name)
	if len(parts) == 3 {
		return `\\?\GLOBALROOT` + name + `\`
	}
	return `\\?\GLOBALROOT` + name
}

func enrollmentInfo(handle windows.Handle) (windows.ByHandleFileInformation, enrollmentFileID, error) {
	var info windows.ByHandleFileInformation
	var id enrollmentFileID
	if err := windows.GetFileInformationByHandle(handle, &info); err != nil {
		return info, id, err
	}
	if err := windows.GetFileInformationByHandleEx(handle, windows.FileIdInfo, (*byte)(unsafe.Pointer(&id)), uint32(unsafe.Sizeof(id))); err != nil {
		return info, id, err
	}
	return info, id, nil
}

func (p *enrollmentPin) check() error {
	info, id, err := enrollmentInfo(p.handle)
	if err != nil {
		return err
	}
	name, err := enrollmentPhysicalPath(p.handle)
	if err != nil || name != p.physical || id != p.id || info.CreationTime != p.info.CreationTime || info.VolumeSerialNumber != p.info.VolumeSerialNumber || info.FileAttributes&windows.FILE_ATTRIBUTE_REPARSE_POINT != 0 {
		return errors.New("held enrollment native identity changed")
	}
	if p.file {
		if info.FileAttributes&windows.FILE_ATTRIBUTE_DIRECTORY != 0 || info.NumberOfLinks != 1 || int64(uint64(info.FileSizeHigh)<<32|uint64(info.FileSizeLow)) != p.length {
			return errors.New("held enrollment file type, links or length changed")
		}
	} else if info.FileAttributes&windows.FILE_ATTRIBUTE_DIRECTORY == 0 {
		return errors.New("held enrollment directory required")
	}
	if err := enrollmentSecurity(p.handle, p.descriptor, p.allowAdds, p.broker); err != nil {
		return err
	}
	if p.dos != "" {
		pointer, err := windows.UTF16PtrFromString(p.dos)
		if err != nil {
			return err
		}
		access := uint32(windows.FILE_LIST_DIRECTORY | windows.FILE_READ_ATTRIBUTES | windows.READ_CONTROL)
		flags := uint32(windows.FILE_FLAG_OPEN_REPARSE_POINT | windows.FILE_FLAG_BACKUP_SEMANTICS)
		if p.file {
			access = windows.GENERIC_READ | windows.READ_CONTROL
			flags = windows.FILE_FLAG_OPEN_REPARSE_POINT
		}
		h, err := windows.CreateFile(pointer, access, windows.FILE_SHARE_READ|windows.FILE_SHARE_WRITE, nil, windows.OPEN_EXISTING, flags, 0)
		if err != nil {
			return err
		}
		_, actualID, identityErr := enrollmentInfo(h)
		actualName, nameErr := enrollmentPhysicalPath(h)
		closeErr := enrollmentCleanup(windows.CloseHandle(h))
		if identityErr != nil || nameErr != nil || actualID != p.id || actualName != p.physical {
			return errors.Join(errors.New("fixed enrollment DOS path resolution changed"), identityErr, nameErr, closeErr)
		}
		if closeErr != nil {
			return closeErr
		}
	}
	return nil
}

func enrollmentSecurity(handle windows.Handle, expected *windows.SECURITY_DESCRIPTOR, allowAdds bool, broker string) error {
	actual, err := windows.GetSecurityInfo(handle, windows.SE_FILE_OBJECT, windows.OWNER_SECURITY_INFORMATION|windows.DACL_SECURITY_INFORMATION)
	if err != nil {
		return err
	}
	return enrollmentSecurityDescriptor(actual, expected, allowAdds, broker)
}

func enrollmentSecurityDescriptor(actual, expected *windows.SECURITY_DESCRIPTOR, allowAdds bool, broker string) error {
	if actual == nil {
		return errors.New("enrollment security descriptor required")
	}
	owner, _, err := actual.Owner()
	if err != nil || owner == nil {
		return errors.New("enrollment owner unavailable")
	}
	trusted := func(s string) bool {
		return s == "S-1-5-18" || s == "S-1-5-32-544" || s == enrollmentTrustedInstaller || broker != "" && s == broker
	}
	if !trusted(owner.String()) {
		return errors.New("untrusted enrollment ancestor owner")
	}
	control, _, err := actual.Control()
	if err != nil || control&windows.SE_DACL_PRESENT == 0 {
		return errors.New("enrollment DACL unavailable")
	}
	acl, _, err := actual.DACL()
	if err != nil || acl == nil {
		return errors.New("non-null enrollment DACL required")
	}
	if expected != nil {
		wantOwner, _, err := expected.Owner()
		if err != nil || !owner.Equals(wantOwner) {
			return errors.New("fresh enrollment owner changed")
		}
		wantControl, _, err := expected.Control()
		const inheritance = windows.SE_DACL_DEFAULTED | windows.SE_DACL_AUTO_INHERIT_REQ | windows.SE_DACL_AUTO_INHERITED | windows.SE_DACL_PROTECTED
		if err != nil || control&inheritance != wantControl&inheritance {
			return errors.New("fresh enrollment inheritance changed")
		}
		want, _, err := expected.DACL()
		if err != nil || want == nil || want.AceCount != acl.AceCount {
			return errors.New("fresh enrollment DACL changed")
		}
		for i := uint32(0); i < uint32(acl.AceCount); i++ {
			var ace, allowed *windows.ACCESS_ALLOWED_ACE
			if err := windows.GetAce(acl, i, &ace); err != nil {
				return err
			}
			if err := windows.GetAce(want, i, &allowed); err != nil {
				return err
			}
			sid := (*windows.SID)(unsafe.Pointer(&ace.SidStart))
			wantSID := (*windows.SID)(unsafe.Pointer(&allowed.SidStart))
			if ace.Header.AceType != windows.ACCESS_ALLOWED_ACE_TYPE || !sid.IsValid() || !sid.Equals(wantSID) || ace.Header.AceFlags != allowed.Header.AceFlags || ace.Mask != allowed.Mask {
				return errors.New("fresh enrollment ACE changed")
			}
		}
		runtime.KeepAlive(expected)
	} else {
		mask := uint32(windows.DELETE|windows.WRITE_DAC|windows.WRITE_OWNER|windows.GENERIC_ALL|windows.GENERIC_WRITE|windows.FILE_WRITE_ATTRIBUTES|windows.FILE_WRITE_EA) | 0x40
		if !allowAdds {
			mask |= windows.FILE_WRITE_DATA | windows.FILE_APPEND_DATA
		}
		for i := uint32(0); i < uint32(acl.AceCount); i++ {
			var ace *windows.ACCESS_ALLOWED_ACE
			if err := windows.GetAce(acl, i, &ace); err != nil {
				return err
			}
			if ace.Header.AceFlags&windows.INHERIT_ONLY_ACE != 0 || ace.Header.AceType == windows.ACCESS_DENIED_ACE_TYPE {
				continue
			}
			if ace.Header.AceType != windows.ACCESS_ALLOWED_ACE_TYPE {
				return errors.New("unsupported enrollment ancestor ACE")
			}
			sid := (*windows.SID)(unsafe.Pointer(&ace.SidStart))
			if !sid.IsValid() || !trusted(sid.String()) && uint32(ace.Mask)&mask != 0 {
				return errors.New("untrusted enrollment ancestor mutation")
			}
		}
	}
	runtime.KeepAlive(actual)
	return nil
}

func (f *nativeEnrollmentFiles) pinDirectory(path, physical, dos string, sd *windows.SECURITY_DESCRIPTOR, allowAdds bool, broker string) (*enrollmentPin, error) {
	pointer, err := windows.UTF16PtrFromString(path)
	if err != nil {
		return nil, err
	}
	h, err := windows.CreateFile(pointer, windows.FILE_LIST_DIRECTORY|windows.FILE_READ_ATTRIBUTES|windows.READ_CONTROL, windows.FILE_SHARE_READ|windows.FILE_SHARE_WRITE, nil, windows.OPEN_EXISTING, windows.FILE_FLAG_OPEN_REPARSE_POINT|windows.FILE_FLAG_BACKUP_SEMANTICS, 0)
	if err != nil {
		return nil, err
	}
	pin := &enrollmentPin{handle: h, physical: physical, dos: dos, descriptor: sd, allowAdds: allowAdds, broker: broker}
	f.pins = append(f.pins, pin)
	pin.info, pin.id, err = enrollmentInfo(h)
	if err != nil {
		return nil, err
	}
	if pin.physical == "" {
		pin.physical, err = enrollmentPhysicalPath(h)
		if err != nil {
			return nil, err
		}
	}
	if err := pin.check(); err != nil {
		return nil, err
	}
	return pin, nil
}

func (f *nativeEnrollmentFiles) holdAncestry(dos string, data bool) (*enrollmentPin, error) {
	if len(dos) < 4 || filepath.Clean(dos) != dos || dos[1:3] != `:\` || !((dos[0] >= 'A' && dos[0] <= 'Z') || (dos[0] >= 'a' && dos[0] <= 'z')) || strings.ContainsAny(dos[3:], ":/\x00") {
		return nil, errors.New("fixed local KnownFolder spelling required")
	}
	drive, err := f.pinDirectory(dos[:3], "", dos[:3], nil, true, "")
	if err != nil {
		return nil, err
	}
	parts, err := enrollmentPhysicalParts(drive.physical)
	if err != nil || len(parts) != 3 {
		return nil, errors.New("KnownFolder drive alias refused")
	}
	var serial, component, flags uint32
	fs := make([]uint16, 32)
	if err := windows.GetVolumeInformationByHandle(drive.handle, nil, 0, &serial, &component, &flags, &fs[0], uint32(len(fs))); err != nil || windows.UTF16ToString(fs) != "NTFS" || serial != drive.info.VolumeSerialNumber {
		return nil, errors.New("native NTFS enrollment volume required")
	}
	name := drive.physical
	currentDOS := dos[:2]
	parent := drive
	for _, part := range strings.Split(dos[3:], `\`) {
		if part == "" || part == "." || part == ".." || strings.HasSuffix(part, ".") || strings.HasSuffix(part, " ") {
			return nil, errors.New("ambiguous KnownFolder ancestry")
		}
		name += `\` + part
		currentDOS += `\` + part
		parent, err = f.pinDirectory(enrollmentPhysical(name), name, currentDOS, nil, data, "")
		if err != nil {
			return nil, err
		}
		if parent.info.VolumeSerialNumber != serial {
			return nil, errors.New("KnownFolder ancestry crossed volumes")
		}
	}
	return parent, nil
}

func enrollmentAbsent(name string) error {
	pointer, err := windows.UTF16PtrFromString(enrollmentPhysical(name))
	if err != nil {
		return err
	}
	_, err = windows.GetFileAttributes(pointer)
	if errors.Is(err, windows.ERROR_FILE_NOT_FOUND) {
		return nil
	}
	if err != nil {
		return err
	}
	return errors.New("existing fixed enrollment namespace refuses fresh creation")
}

func (f *nativeEnrollmentFiles) newDirectory(parent *enrollmentPin, component string, sd *windows.SECURITY_DESCRIPTOR, broker string) (*enrollmentPin, error) {
	if err := f.check(); err != nil {
		return nil, err
	}
	name := parent.physical + `\` + component
	pointer, err := windows.UTF16PtrFromString(enrollmentPhysical(name))
	if err != nil {
		return nil, err
	}
	attributes := windows.SecurityAttributes{Length: uint32(unsafe.Sizeof(windows.SecurityAttributes{})), SecurityDescriptor: sd}
	if err := windows.CreateDirectory(pointer, &attributes); err != nil {
		return nil, err
	}
	pin, err := f.pinDirectory(enrollmentPhysical(name), name, filepath.Join(parent.dos, component), sd, false, broker)
	if err != nil {
		return nil, err
	}
	if pin.info.VolumeSerialNumber != parent.info.VolumeSerialNumber {
		return nil, errors.New("fresh enrollment child crossed volume")
	}
	return pin, f.check()
}

func (f *nativeEnrollmentFiles) createFence(parent *enrollmentPin, sd *windows.SECURITY_DESCRIPTOR) error {
	file, pin, err := f.newFile(parent, enrollmentFenceName, sd)
	if err != nil {
		return err
	}
	f.fence, f.fencePin = file, pin
	f.pins = append(f.pins, pin)
	if n, err := file.WriteString(filesStarted); err != nil {
		return err
	} else if n != len(filesStarted) {
		return io.ErrShortWrite
	}
	pin.length = int64(len(filesStarted))
	if err := file.Sync(); err != nil {
		return err
	}
	return f.check()
}

// holdOperatorDirectories preserves the current protected native namespace
// through verification and provisioning. It cannot establish the history of a
// preexisting TUF cache or authenticate the first provisioner invocation.
func holdOperatorDirectories(cacheDir, artifactDir string) (io.Closer, error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if err := enrollmentOperator(); err != nil {
		return nil, err
	}
	f := &nativeEnrollmentFiles{}
	for _, directory := range []string{cacheDir, artifactDir} {
		pin, err := f.holdAncestry(directory, true)
		if err != nil {
			return nil, errors.Join(err, f.Close())
		}
		pin.allowAdds = false
		if err := pin.check(); err != nil {
			return nil, errors.Join(err, f.Close())
		}
		sd, err := windows.GetSecurityInfo(pin.handle, windows.SE_FILE_OBJECT, windows.OWNER_SECURITY_INFORMATION)
		if err != nil {
			return nil, errors.Join(err, f.Close())
		}
		owner, _, err := sd.Owner()
		if err != nil || owner == nil || (owner.String() != "S-1-5-18" && owner.String() != "S-1-5-32-544") {
			return nil, errors.Join(errors.New("machine-protected operator directory owner required"), err, f.Close())
		}
	}
	if err := f.Check(); err != nil {
		return nil, errors.Join(err, f.Close())
	}
	return f, nil
}

func (f *nativeEnrollmentFiles) newFile(parent *enrollmentPin, component string, sd *windows.SECURITY_DESCRIPTOR) (*os.File, *enrollmentPin, error) {
	if err := f.check(); err != nil {
		return nil, nil, err
	}
	name := parent.physical + `\` + component
	pointer, err := windows.UTF16PtrFromString(enrollmentPhysical(name))
	if err != nil {
		return nil, nil, err
	}
	attributes := windows.SecurityAttributes{Length: uint32(unsafe.Sizeof(windows.SecurityAttributes{})), SecurityDescriptor: sd}
	h, err := windows.CreateFile(pointer, windows.GENERIC_READ|windows.GENERIC_WRITE|windows.READ_CONTROL, windows.FILE_SHARE_READ, &attributes, windows.CREATE_NEW, windows.FILE_FLAG_OPEN_REPARSE_POINT, 0)
	if err != nil {
		return nil, nil, err
	}
	file := os.NewFile(uintptr(h), enrollmentPhysical(name))
	pin := &enrollmentPin{handle: h, physical: name, descriptor: sd, file: true}
	pin.info, pin.id, err = enrollmentInfo(h)
	if err == nil {
		err = pin.check()
	}
	if err != nil {
		return nil, nil, errors.Join(err, enrollmentCleanup(file.Close()))
	}
	return file, pin, nil
}

func (f *nativeEnrollmentFiles) createImage(ctx context.Context, parent *enrollmentPin, sd *windows.SECURITY_DESCRIPTOR, source io.Reader, artifact releasepolicy.Artifact) (resultErr error) {
	file, pin, err := f.newFile(parent, enrollmentImageName, sd)
	if err != nil {
		return err
	}
	writerOpen := true
	defer func() {
		if writerOpen {
			resultErr = errors.Join(resultErr, enrollmentCleanup(file.Close()))
		}
	}()
	if err := copyEnrollmentImage(ctx, file, source, artifact); err != nil {
		return err
	}
	pin.length = artifact.Bytes
	if err := file.Sync(); err != nil {
		return err
	}
	if err := pin.check(); err != nil {
		return err
	}
	// The new parent was protected from its first exposure. Closing the writer
	// and reopening read-only cannot expose these bytes to an untrusted writer.
	closeErr := enrollmentCleanup(file.Close())
	writerOpen = false
	if closeErr != nil {
		return closeErr
	}
	pointer, err := windows.UTF16PtrFromString(enrollmentPhysical(pin.physical))
	if err != nil {
		return err
	}
	h, err := windows.CreateFile(pointer, windows.GENERIC_READ|windows.READ_CONTROL, windows.FILE_SHARE_READ, nil, windows.OPEN_EXISTING, windows.FILE_FLAG_OPEN_REPARSE_POINT, 0)
	if err != nil {
		return err
	}
	pin.handle = h
	pin.dos = filepath.Join(parent.dos, enrollmentImageName)
	f.pins = append(f.pins, pin)
	if err := pin.check(); err != nil {
		return err
	}
	f.imagePath = pin.dos
	return f.check()
}
