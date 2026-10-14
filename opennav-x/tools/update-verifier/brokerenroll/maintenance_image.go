package brokerenroll

import (
	"context"
	"debug/pe"
	"encoding/binary"
	"errors"
	"io"
	"strings"

	"example.com/opennav-update-verifier/releasepolicy"
)

const maintenanceImageName = "skager-update-provisioner.exe"

// Only the native opener derives this stream from the actual running image.
// Independent first-execution approval is still mandatory; checking/copying
// running bytes cannot retroactively authenticate their initial loader chain.
type maintenanceImageSource interface {
	io.Reader
	Check(context.Context) error
	Close() error
}

func validateMaintenanceArtifact(a releasepolicy.Artifact) error {
	if !validPin(a.SHA256) || a.Bytes < 1 || a.Bytes > maxBrokerBytes || !strings.HasSuffix(a.Path, "/"+maintenanceImageName) {
		return errors.New("bounded authenticated original provisioner required")
	}
	return nil
}

func copyMaintenanceImage(ctx context.Context, destination io.Writer, source io.Reader, a releasepolicy.Artifact) error {
	if err := validateMaintenanceArtifact(a); err != nil {
		return err
	}
	// Reuse the exact-length/SHA/EOF copier. Its filename check selects its
	// internal broker stream contract only; the real helper artifact above has
	// already been selected by the independently verified bootstrap policy.
	copyContract := a
	copyContract.Path = "maintenance-copy/" + enrollmentImageName
	return copyEnrollmentImage(ctx, destination, source, copyContract)
}

// Go's native provisioner is CGO-free: only kernel32 is statically imported;
// runtime/Windows helpers load other system DLLs through SYSTEM32. Refuse an
// approved artifact with a different loader contract. debug/pe's
// ImportedLibraries is unimplemented, so inspect bounded import descriptors.
func validateMaintenanceLoader(source io.ReaderAt, length int64) error {
	if source == nil || length < 1 || length > maxBrokerBytes {
		return errors.New("bounded provisioner PE required")
	}
	image, err := pe.NewFile(io.NewSectionReader(source, 0, length))
	if err != nil {
		return errors.New("native provisioner PE required")
	}
	defer image.Close()
	optional, ok := image.OptionalHeader.(*pe.OptionalHeader32)
	if !ok || image.Machine != pe.IMAGE_FILE_MACHINE_I386 || image.Characteristics&pe.IMAGE_FILE_EXECUTABLE_IMAGE == 0 || image.Characteristics&pe.IMAGE_FILE_DLL != 0 || optional.Subsystem != 2 && optional.Subsystem != 3 || optional.NumberOfRvaAndSizes < 15 {
		return errors.New("native Win32 provisioner executable required")
	}
	for _, index := range []int{13, 14} { // delay imports and CLR execution
		if optional.DataDirectory[index].VirtualAddress != 0 || optional.DataDirectory[index].Size != 0 {
			return errors.New("provisioner alternate loader refused")
		}
	}
	imports := optional.DataDirectory[pe.IMAGE_DIRECTORY_ENTRY_IMPORT]
	if imports.VirtualAddress == 0 || imports.Size < 40 || imports.Size > 32768 {
		return errors.New("bounded native provisioner imports required")
	}
	readRVA := func(rva uint32, count int) ([]byte, error) {
		var offset int64 = -1
		for _, section := range image.Sections {
			if rva < section.VirtualAddress {
				continue
			}
			delta := uint64(rva) - uint64(section.VirtualAddress)
			if delta >= uint64(section.Size) {
				continue
			}
			if offset != -1 || uint64(count) > uint64(section.Size)-delta {
				return nil, errors.New("ambiguous provisioner import range")
			}
			offset = int64(uint64(section.Offset) + delta)
		}
		if offset < 0 || int64(count) > length-offset {
			return nil, errors.New("provisioner import range outside original bytes")
		}
		raw := make([]byte, count)
		if _, err := source.ReadAt(raw, offset); err != nil {
			return nil, err
		}
		return raw, nil
	}
	count := 0
	for offset := uint32(0); offset+20 <= imports.Size; offset += 20 {
		if uint64(imports.VirtualAddress)+uint64(offset) > uint64(^uint32(0)) {
			return errors.New("provisioner import RVA overflow")
		}
		raw, err := readRVA(imports.VirtualAddress+offset, 20)
		if err != nil {
			return err
		}
		if string(raw) == string(make([]byte, 20)) {
			if count != 1 {
				return errors.New("single system provisioner import required")
			}
			return nil
		}
		nameRVA := binary.LittleEndian.Uint32(raw[12:16])
		if nameRVA == 0 || binary.LittleEndian.Uint32(raw[16:20]) == 0 || count != 0 {
			return errors.New("closed provisioner import table required")
		}
		name := make([]byte, 0, 32)
		for i := uint32(0); i < 260; i++ {
			if uint64(nameRVA)+uint64(i) > uint64(^uint32(0)) {
				return errors.New("provisioner import name overflow")
			}
			b, err := readRVA(nameRVA+i, 1)
			if err != nil {
				return err
			}
			if b[0] == 0 {
				break
			}
			name = append(name, b[0])
		}
		if !strings.EqualFold(string(name), "kernel32.dll") {
			return errors.New("non-system provisioner import refused")
		}
		count++
	}
	return errors.New("unterminated provisioner import table")
}
