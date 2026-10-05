package launcher

import (
	"encoding/binary"
	"errors"
	"io"
)

// Inspect only bounded DOS/COFF/optional header bytes; no imports are loaded.
func validatePE32(reader io.ReaderAt, size int64) error {
	var dos [64]byte
	if size < 128 {
		return errors.New("stock executable is too short")
	}
	if _, err := reader.ReadAt(dos[:], 0); err != nil || dos[0] != 'M' || dos[1] != 'Z' {
		return errors.New("stock DOS header invalid")
	}
	offset := int64(binary.LittleEndian.Uint32(dos[60:64]))
	if offset < 64 || offset > size-26 {
		return errors.New("stock PE header outside file")
	}
	var pe [26]byte
	if _, err := reader.ReadAt(pe[:], offset); err != nil || string(pe[:4]) != "PE\x00\x00" || binary.LittleEndian.Uint16(pe[4:6]) != 0x14c || binary.LittleEndian.Uint16(pe[20:22]) < 2 || binary.LittleEndian.Uint16(pe[24:26]) != 0x10b {
		return errors.New("stock executable must be PE32 x86")
	}
	return nil
}
