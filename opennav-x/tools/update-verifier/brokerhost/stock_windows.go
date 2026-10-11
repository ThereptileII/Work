package brokerhost

import (
	"context"
	"crypto/sha256"
	"encoding/hex"
	"errors"
	"io"
	"os"
	"regexp"
	"strings"

	"example.com/opennav-update-verifier/releasepolicy"
	"golang.org/x/sys/windows"
)

var physicalVolume = regexp.MustCompile(`^\\Device\\HarddiskVolume[0-9]{1,10}$`)

type stockPin struct {
	handle windows.Handle
	info   windows.ByHandleFileInformation
	path   string
}
type nativeStock struct {
	file     *os.File
	info     windows.ByHandleFileInformation
	path     string
	physical string
	parents  []stockPin
}

func observeStock(ctx context.Context, r registration) (_ releasepolicy.OpenCpn, _ stockOwner, resultErr error) {
	if ctx == nil || ctx.Err() != nil || !localStockPath(r.Stock.Path) || r.observedDeclaration() != stockBaseline {
		return releasepolicy.OpenCpn{}, nil, errors.New("supported stock observation required")
	}
	owner := &nativeStock{path: r.Stock.Path}
	defer func() {
		if resultErr != nil {
			_ = owner.Close()
		}
	}()
	h, err := openStock(r.Stock.Path, false)
	if err != nil {
		return releasepolicy.OpenCpn{}, nil, err
	}
	owner.file = os.NewFile(uintptr(h), "retained-stock-opencpn")
	if owner.file == nil {
		_ = windows.CloseHandle(h)
		return releasepolicy.OpenCpn{}, nil, errors.New("stock file owner unavailable")
	}
	if windows.GetFileInformationByHandle(h, &owner.info) != nil || !stockFileInfo(owner.info) {
		return releasepolicy.OpenCpn{}, nil, errors.New("stock image is not a plain single-link bounded file")
	}
	owner.physical, err = stockPhysical(h)
	if err != nil {
		return releasepolicy.OpenCpn{}, nil, err
	}
	parts := strings.Split(owner.physical, `\`)
	if len(parts) < 4 || !physicalVolume.MatchString(strings.Join(parts[:3], `\`)) {
		return releasepolicy.OpenCpn{}, nil, errors.New("local physical stock volume required")
	}
	for n := 3; n < len(parts); n++ {
		name := strings.Join(parts[:n], `\`)
		openName := `\\?\GLOBALROOT` + name
		if n == 3 {
			openName += `\`
		}
		p, err := openStock(openName, true)
		if err != nil {
			return releasepolicy.OpenCpn{}, nil, err
		}
		pin := stockPin{handle: p, path: name}
		owner.parents = append(owner.parents, pin)
		last := &owner.parents[len(owner.parents)-1]
		if windows.GetFileInformationByHandle(p, &last.info) != nil || !stockDirectoryInfo(last.info) {
			return releasepolicy.OpenCpn{}, nil, errors.New("stock ancestor is not plain")
		}
		physical, err := stockPhysical(p)
		if err != nil || !strings.EqualFold(strings.TrimRight(physical, `\`), name) {
			return releasepolicy.OpenCpn{}, nil, errors.New("stock physical ancestor differs")
		}
	}
	// The approved complete-file hash identifies version/architecture. There is
	// no execution of the stock binary or trust in a displayed version string.
	digest := sha256.New()
	buffer := make([]byte, 64<<10)
	var count int64
	for {
		if err := ctx.Err(); err != nil {
			return releasepolicy.OpenCpn{}, nil, err
		}
		n, err := owner.file.Read(buffer)
		count += int64(n)
		if count > 512<<20 {
			return releasepolicy.OpenCpn{}, nil, errors.New("stock image exceeds bound")
		}
		_, _ = digest.Write(buffer[:n])
		if err == io.EOF {
			break
		}
		if err != nil {
			return releasepolicy.OpenCpn{}, nil, errors.New("stock image read refused")
		}
	}
	if count != (int64(owner.info.FileSizeHigh)<<32|int64(owner.info.FileSizeLow)) || hex.EncodeToString(digest.Sum(nil)) != stockBaseline.ExecutableSHA256 {
		return releasepolicy.OpenCpn{}, nil, errors.New("stock executable hash differs")
	}
	if err := owner.validate(); err != nil {
		return releasepolicy.OpenCpn{}, nil, err
	}
	return stockBaseline, owner, nil
}

func openStock(name string, directory bool) (windows.Handle, error) {
	p, err := windows.UTF16PtrFromString(name)
	if err != nil {
		return 0, err
	}
	access, share := uint32(windows.GENERIC_READ), uint32(windows.FILE_SHARE_READ)
	flags := uint32(windows.FILE_FLAG_OPEN_REPARSE_POINT)
	if directory {
		// Metadata-only opens do not participate in delete sharing checks.
		// Directory list access makes this no-delete-share handle a rename pin.
		access = windows.FILE_LIST_DIRECTORY | windows.FILE_READ_ATTRIBUTES
		share |= windows.FILE_SHARE_WRITE
		flags |= windows.FILE_FLAG_BACKUP_SEMANTICS
	}
	return windows.CreateFile(p, access, share, nil, windows.OPEN_EXISTING, flags, 0)
}
func stockPhysical(h windows.Handle) (string, error) {
	buffer := make([]uint16, 32768)
	n, err := windows.GetFinalPathNameByHandle(h, &buffer[0], uint32(len(buffer)), 2)
	if err != nil || n == 0 || n >= uint32(len(buffer)) {
		return "", errors.New("physical stock path unavailable")
	}
	return windows.UTF16ToString(buffer[:n]), nil
}
func stockFileInfo(i windows.ByHandleFileInformation) bool {
	return i.FileAttributes&(windows.FILE_ATTRIBUTE_DIRECTORY|windows.FILE_ATTRIBUTE_REPARSE_POINT) == 0 && i.NumberOfLinks == 1 &&
		(uint64(i.FileSizeHigh)<<32|uint64(i.FileSizeLow)) > 0 && (uint64(i.FileSizeHigh)<<32|uint64(i.FileSizeLow)) <= 512<<20
}
func stockDirectoryInfo(i windows.ByHandleFileInformation) bool {
	return i.FileAttributes&windows.FILE_ATTRIBUTE_DIRECTORY != 0 && i.FileAttributes&windows.FILE_ATTRIBUTE_REPARSE_POINT == 0
}
func sameStock(a, b windows.ByHandleFileInformation) bool {
	return a.VolumeSerialNumber == b.VolumeSerialNumber && a.FileIndexHigh == b.FileIndexHigh && a.FileIndexLow == b.FileIndexLow &&
		a.CreationTime == b.CreationTime && a.FileAttributes == b.FileAttributes && a.NumberOfLinks == b.NumberOfLinks
}
func (s *nativeStock) validate() error {
	if s == nil || s.file == nil {
		return errors.New("retained stock owner required")
	}
	var info windows.ByHandleFileInformation
	if windows.GetFileInformationByHandle(windows.Handle(s.file.Fd()), &info) != nil || !sameStock(s.info, info) ||
		info.FileSizeHigh != s.info.FileSizeHigh || info.FileSizeLow != s.info.FileSizeLow || info.LastWriteTime != s.info.LastWriteTime {
		return errors.New("retained stock image changed")
	}
	for _, p := range s.parents {
		if windows.GetFileInformationByHandle(p.handle, &info) != nil || !sameStock(p.info, info) || !stockDirectoryInfo(info) {
			return errors.New("retained stock ancestor changed")
		}
		physical, err := stockPhysical(p.handle)
		if err != nil || !strings.EqualFold(strings.TrimRight(physical, `\`), p.path) {
			return errors.New("retained stock ancestor moved")
		}
	}
	h, err := openStock(s.path, false)
	if err != nil {
		return errors.New("stock DOS resolution unavailable")
	}
	defer windows.CloseHandle(h)
	physical, err := stockPhysical(h)
	if err != nil || !strings.EqualFold(physical, s.physical) || windows.GetFileInformationByHandle(h, &info) != nil || !sameStock(s.info, info) {
		return errors.New("stock DOS resolution differs from retained image")
	}
	return nil
}
func (s *nativeStock) Close() error {
	if s == nil {
		return nil
	}
	var errs []error
	if s.file != nil {
		errs = append(errs, s.file.Close())
		s.file = nil
	}
	for _, p := range s.parents {
		errs = append(errs, windows.CloseHandle(p.handle))
	}
	s.parents = nil
	return errors.Join(errs...)
}
