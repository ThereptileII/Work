package launcher

import (
	"bytes"
	"encoding/binary"
	"errors"
)

const brokerQueryProcessMask uint32 = 0x00101400
const brokerQueryTokenMask uint32 = 0x00000008

var errBrokerQueryAccess = errors.New("broker query access refused or uncertain")

// appendBrokerQueryACE preserves every original ACE byte and ACL padding. Only
// simple allow ACEs with unambiguous SIDs are supported. Existing explicit deny,
// callback/object/inheritance semantics and duplicate or excessive broker ACEs
// refuse; none are silently removed, merged or rewritten. trusted identifies
// existing local user/logon/SYSTEM/admin trustees, never a new grant to them.
func appendBrokerQueryACE(raw, broker []byte, mask uint32, trusted [][]byte) ([]byte, error) {
	if !brokerQuerySID(broker) || (mask != brokerQueryProcessMask && mask != brokerQueryTokenMask) || len(raw) < 8 || len(raw) > 65535 || raw[0] != 2 || raw[1] != 0 || binary.LittleEndian.Uint16(raw[6:8]) != 0 || int(binary.LittleEndian.Uint16(raw[2:4])) != len(raw) {
		return nil, errBrokerQueryAccess
	}
	count := int(binary.LittleEndian.Uint16(raw[4:6]))
	if count == 0 || count > 256 {
		return nil, errBrokerQueryAccess
	}
	pos, seen := 8, false
	for i := 0; i < count; i++ {
		if pos+8 > len(raw) {
			return nil, errBrokerQueryAccess
		}
		size := int(binary.LittleEndian.Uint16(raw[pos+2 : pos+4]))
		if size < 16 || size%4 != 0 || pos+size > len(raw) || raw[pos] != 0 || raw[pos+1]&^byte(0x10) != 0 {
			return nil, errBrokerQueryAccess
		}
		sid := raw[pos+8 : pos+size]
		if !brokerQuerySID(sid) {
			return nil, errBrokerQueryAccess
		}
		if bytes.Equal(sid, broker) {
			if seen || raw[pos+1] != 0 || binary.LittleEndian.Uint32(raw[pos+4:pos+8]) != mask {
				return nil, errBrokerQueryAccess
			}
			seen = true
		} else {
			found := false
			for _, allowed := range trusted {
				if bytes.Equal(sid, allowed) {
					found = true
					break
				}
			}
			if !found {
				return nil, errBrokerQueryAccess
			}
		}
		pos += size
	}
	if seen {
		return bytes.Clone(raw), nil
	}
	size := 8 + len(broker)
	if len(raw)+size > 65535 || count == 256 {
		return nil, errBrokerQueryAccess
	}
	out := make([]byte, len(raw)+size)
	copy(out, raw[:pos])
	copy(out[pos+size:], raw[pos:])
	binary.LittleEndian.PutUint16(out[2:4], uint16(len(out)))
	binary.LittleEndian.PutUint16(out[4:6], uint16(count+1))
	binary.LittleEndian.PutUint16(out[pos+2:pos+4], uint16(size))
	binary.LittleEndian.PutUint32(out[pos+4:pos+8], mask)
	copy(out[pos+8:], broker)
	return out, nil
}
func brokerQuerySID(raw []byte) bool {
	return len(raw) >= 8 && raw[0] == 1 && raw[1] <= 15 && len(raw) == 8+4*int(raw[1])
}
