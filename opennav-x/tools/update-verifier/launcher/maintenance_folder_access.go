package launcher

import (
	"bytes"
	"encoding/binary"
)

const maintenanceFolderTokenMask uint32 = 0x0c // TOKEN_QUERY | TOKEN_IMPERSONATE, no duplication or adjustment

// The ordinary query policy remains strict. Maintenance may change only the
// already present, exact broker TOKEN_QUERY ACE after that strict validation.
// All other bytes, including trustees, ACE ordering/flags and padding survive.
func maintenanceFolderACL(raw, broker []byte, trusted [][]byte) ([]byte, error) {
	checked, err := appendBrokerQueryACE(raw, broker, brokerQueryTokenMask, trusted)
	if err != nil || !bytes.Equal(checked, raw) {
		return nil, errBrokerQueryAccess
	}
	out := bytes.Clone(raw)
	pos := 8
	for index := 0; index < int(binary.LittleEndian.Uint16(out[4:6])); index++ {
		size := int(binary.LittleEndian.Uint16(out[pos+2 : pos+4]))
		if bytes.Equal(out[pos+8:pos+size], broker) {
			binary.LittleEndian.PutUint32(out[pos+4:pos+8], maintenanceFolderTokenMask)
			return out, nil
		}
		pos += size
	}
	return nil, errBrokerQueryAccess
}
